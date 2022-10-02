use crate::{
    prelude::Collider, utils::ParallelSliceEnumerateMut, PhysicsFixedUpdate, PhysicsSystem, Ray,
    Tlas, TlasQuery,
};
use bevy::{
    math::vec3,
    prelude::*,
    render::render_resource::{Extent3d, TextureDimension, TextureFormat},
    tasks::*,
    utils::HashMap,
};
use iyes_loopless::prelude::*;

use super::PhysicsDebugState;

pub struct DebugBvhCameraPlugin;

// Really should only be used to debug and profile bvh
impl Plugin for DebugBvhCameraPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<CameraDebugColors>()
            .add_system_to_stage(
                PhysicsFixedUpdate,
                update_camera_debug_colors
                    .run_in_state(PhysicsDebugState::Running)
                    .before(PhysicsSystem::Resolve),
            )
            .add_system_to_stage(
                PhysicsFixedUpdate,
                render_image
                    .run_in_state(PhysicsDebugState::Running)
                    .after(PhysicsSystem::Resolve),
            )
            .add_exit_system(PhysicsDebugState::Running, remove_ui);
    }
}

const COLORS: [Color; 5] = [
    Color::rgb(0.0, 0.0, 0.0),
    Color::rgb(1.0, 0.0, 0.0),
    Color::rgb(0.0, 1.0, 0.0),
    Color::rgb(0.0, 0.0, 1.0),
    Color::rgb(1.0, 1.0, 1.0),
];

#[derive(Default, Deref, DerefMut)]
pub struct CameraDebugColors(pub HashMap<Entity, Color>);

fn update_camera_debug_colors(
    mut entity_colors: ResMut<CameraDebugColors>,
    collider_query: Query<Entity, With<Handle<Collider>>>,
) {
    for entity in collider_query.iter() {
        if !entity_colors.contains_key(&entity) {
            let color = COLORS[entity_colors.len() % COLORS.len()];
            entity_colors.insert(entity, color);
        }
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
    pub ui_id: Option<Entity>,
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
            ui_id: None,
        }
    }

    pub fn update(&mut self, trans: &Transform) {
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

pub fn render_image(
    mut commands: Commands,
    mut camera_query: Query<(&mut BvhCamera, &Transform)>,
    tlas_query: Query<TlasQuery>,
    node_query: Query<&Node>,
    mut images: ResMut<Assets<Image>>,
    tlas: Res<Tlas>,
    colliders: Res<Assets<Collider>>,
    entity_colors: Res<CameraDebugColors>,
) {
    // you really only want one bvh camera,
    let (mut camera, trans) = camera_query.single_mut();

    // setup image if needed
    if camera.image.is_none() {
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

    // check that the node still exists
    if let Some(id) = camera.ui_id {
        if node_query.get(id).is_err() {
            camera.ui_id = None;
        }
    }

    // setup ui node if needed
    if camera.ui_id.is_none() {
        let image_handle = camera.image.as_ref().unwrap().clone();
        let id = commands
            .spawn_bundle(ImageBundle {
                style: Style {
                    align_self: AlignSelf::FlexEnd,
                    position_type: PositionType::Absolute,
                    position: UiRect {
                        bottom: Val::Px(50.0),
                        right: Val::Px(10.0),
                        ..Default::default()
                    },
                    ..default()
                },
                image: image_handle.into(),
                ..default()
            })
            .insert(Name::new("BVH Image"))
            .id();

        camera.ui_id = Some(id);
    }

    camera.update(trans);

    if let Some(image) = &camera.image {
        let image = images.get_mut(image).unwrap();

        // TODO: Make this acutally tilings, currenty this just takes a slice of pixels in a row
        const PIXEL_TILE_COUNT: usize = 64;
        const PIXEL_TILE: usize = 4 * PIXEL_TILE_COUNT;

        image
            .data
            .par_chunk_map_enumerate_mut(ComputeTaskPool::get(), PIXEL_TILE, |i, pixels| {
                for pixel_offset in 0..(pixels.len() / 4) {
                    let index = i * PIXEL_TILE_COUNT + pixel_offset;
                    let offset = pixel_offset * 4;

                    let x = index as u32 % camera.width;
                    let y = index as u32 / camera.width;
                    let u = x as f32 / camera.width as f32;
                    let v = y as f32 / camera.height as f32;
                    let mut ray = camera.get_ray(u, 1.0 - v);
                    let mut hit_color = Color::BLACK;
                    if let Some(hit) = ray.intersect_tlas(&tlas, &tlas_query, &colliders) {
                        if let Some(color) = entity_colors.get(&hit.entity) {
                            hit_color = *color;
                        }
                    }
                    let color = vec3(hit_color.r(), hit_color.g(), hit_color.b()) * 255.0;
                    pixels[offset] = color.x as u8;
                    pixels[offset + 1] = color.y as u8;
                    pixels[offset + 2] = color.z as u8;
                    pixels[offset + 3] = 255;
                }
            });
    }
}

fn remove_ui(mut commands: Commands, mut camera_query: Query<&mut BvhCamera>) {
    for mut camera in camera_query.iter_mut() {
        if let Some(id) = camera.ui_id {
            camera.ui_id = None;
            camera.image = None;
            commands.entity(id).despawn_recursive();
        }
    }
}
