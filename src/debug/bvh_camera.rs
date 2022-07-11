use crate::{PhysicsSystems, Ray, Tlas};
use bevy::{
    math::vec3,
    prelude::*,
    render::render_resource::{Extent3d, TextureDimension, TextureFormat},
    tasks::*,
};

pub struct PhysicsBvhCameraPlugin;

// Really should only be used to debug and profile bvh
impl Plugin for PhysicsBvhCameraPlugin {
    fn build(&self, app: &mut App) {
        app.add_system_set_to_stage(
            CoreStage::PostUpdate,
            SystemSet::new()
                .after(PhysicsSystems::Resolved)
                .with_system(init_camera_image)
                .with_system(update_camera.after(init_camera_image))
                .with_system(display_camera)
                .with_system(render_camera.after(update_camera))
                .with_system(display_camera.after(render_camera)),
        );
    }
}

pub fn update_camera(mut camera_query: Query<(&mut BvhCamera, &GlobalTransform)>) {
    for (mut camera, trans) in camera_query.iter_mut() {
        camera.update(trans);
    }
}

// TODO: Make this projection based
#[derive(Component)]
pub struct BvhCamera {
    pub width: u32,
    pub height: u32,
    pub origin: Vec3,
    viewport_height: f32,
    viewport_width: f32,
    lower_left_corner: Vec3,
    focus_dist: f32,
    horizontal: Vec3,
    vertical: Vec3,
    u: Vec3,
    v: Vec3,
    w: Vec3,
    pub samples: u32,
    pub image: Option<Handle<Image>>,
}

impl BvhCamera {
    pub fn new(width: u32, height: u32) -> Self {
        // TODO: after messing the params I am defualting more
        let vfov: f32 = 45.0; // vertical field of view
        let focus_dist: f32 = 1.0; // TODO: not using this yet
        let samples: u32 = 1;

        let aspect_ratio = width as f32 / height as f32;
        let theta = vfov * std::f32::consts::PI / 180.0;
        let half_height = (theta / 2.0).tan();
        let viewport_height = 2.0 * half_height;
        let viewport_width = aspect_ratio * viewport_height;

        Self {
            width,
            height,
            viewport_height,
            viewport_width,
            focus_dist,
            samples,
            // Rest will be updated every frame for now
            origin: Vec3::ZERO,
            lower_left_corner: Vec3::ZERO,
            horizontal: Vec3::ZERO,
            vertical: Vec3::ZERO,
            u: Vec3::ZERO,
            v: Vec3::ZERO,
            w: Vec3::ONE,
            image: None,
        }
    }

    pub fn update(&mut self, trans: &GlobalTransform) {
        self.origin = trans.translation;

        self.w = -trans.forward();
        self.u = trans.right();
        self.v = trans.up();

        self.horizontal = self.focus_dist * self.viewport_width * self.u;
        self.vertical = self.focus_dist * self.viewport_height * self.v;

        self.lower_left_corner =
            self.origin - self.horizontal / 2.0 - self.vertical / 2.0 - self.focus_dist * self.w;
    }

    pub fn get_ray(&self, u: f32, v: f32) -> Ray {
        let direction = (self.lower_left_corner + u * self.horizontal + v * self.vertical
            - self.origin)
            .normalize();
        Ray {
            origin: self.origin,
            direction,
            direction_inv: direction.recip(),
            distance: f32::MAX,
            hit: None,
        }
    }
}

pub fn display_camera(mut commands: Commands, camera_query: Query<&BvhCamera> ) {
    if let Ok(camera) = camera_query.get_single() {
        if let Some(image) = &camera.image {
            commands
                .spawn_bundle(ImageBundle {
                    style: Style {
                        align_self: AlignSelf::FlexEnd,
                        position_type: PositionType::Absolute,
                        position: Rect {
                            bottom: Val::Px(50.0),
                            right: Val::Px(10.0),
                            ..Default::default()
                        },
                        ..default()
                    },
                    image: image.clone().into(),
                    ..default()
                })
                .insert(Name::new("BVH Image"));
        }
    }
}

pub fn init_camera_image(
    mut query: Query<&mut BvhCamera, Added<BvhCamera>>,
    mut images: ResMut<Assets<Image>>,
) {
    for mut camera in query.iter_mut() {
        let image = images.add(Image::new(
            Extent3d {
                width: camera.width as u32,
                height: camera.height as u32,
                depth_or_array_layers: 1,
            },
            TextureDimension::D2,
            vec![0; (camera.width * camera.height) as usize * 4],
            TextureFormat::Rgba8UnormSrgb,
        ));
        camera.image = Some(image);
    }
}

pub trait ParallelSliceEnumerateMut<T: Send>: AsMut<[T]> {
    fn par_chunk_map_enumerate_mut<F, R>(
        &mut self,
        task_pool: &TaskPool,
        chunk_size: usize,
        f: F,
    ) -> Vec<R>
    where
        F: Fn(usize, &mut [T]) -> R + Send + Sync,
        R: Send + 'static,
    {
        let slice = self.as_mut();
        let f = &f;
        task_pool.scope(|scope| {
            for (i, chunk) in slice.chunks_mut(chunk_size).enumerate() {
                scope.spawn(async move { f(i, chunk) });
            }
        })
    }
}

impl<S, T: Send> ParallelSliceEnumerateMut<T> for S where S: AsMut<[T]> {}

pub fn render_camera(
    camera_query: Query<&BvhCamera>,
    mut images: ResMut<Assets<Image>>,
    tlas: Res<Tlas>,
    task_pool: Res<ComputeTaskPool>,
) {
    if let Ok(camera) = camera_query.get_single() {
        if let Some(image) = &camera.image {
            let image = images.get_mut(image).unwrap();

            // TODO: Make this acutally tilings, currenty this just takes a slice pixels in a row
            const PIXEL_TILE_COUNT: usize = 64;
            const PIXEL_TILE: usize = 4 * PIXEL_TILE_COUNT;

            image
                .data
                .par_chunk_map_enumerate_mut(&task_pool, PIXEL_TILE, |i, pixels| {
                    for pixel_offset in 0..(pixels.len() / 4) {
                        let index = i * PIXEL_TILE_COUNT + pixel_offset;
                        let offset = pixel_offset * 4;

                        let x = index as u32 % camera.width;
                        let y = index as u32 / camera.width;
                        let u = x as f32 / camera.width as f32;
                        let v = y as f32 / camera.height as f32;
                        // TODO: Revisit multiple samples later
                        // if samples > 0 {
                        //     u += rng.gen::<f32>() / camera.width as f32;
                        //     v += rng.gen::<f32>() / camera.height as f32;
                        // }

                        // TODO: flip v since image is upside down, figure out why
                        let mut ray = camera.get_ray(u, 1.0 - v);
                        let color = if let Some(hit) = ray.intersect_tlas(&tlas) {
                            vec3(hit.u, hit.v, 1.0 - (hit.u + hit.v)) * 255.0
                        } else {
                            Vec3::ZERO
                        };

                        pixels[offset] = color.x as u8;
                        pixels[offset + 1] = color.y as u8;
                        pixels[offset + 2] = color.z as u8;
                        pixels[offset + 3] = 255;
                    }
                });
        }
    }
}
