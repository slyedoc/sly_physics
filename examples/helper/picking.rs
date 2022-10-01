use bevy::{pbr::NotShadowCaster, prelude::*};
use bevy_inspector_egui::{bevy_egui::EguiContext, prelude::*};
use sly_physics::prelude::*;

use super::Keep;

pub struct PickingPlugin;

impl Plugin for PickingPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugin(InspectorPlugin::<Inspector>::new())
            .add_startup_system(setup_cursor)
            .add_system_to_stage(
                PhysicsFixedUpdate,
                cursor_system.after(PhysicsSystem::Resolve),
            );
    }
}

#[derive(Inspectable, Default)]
struct Inspector {
    #[inspectable(deletable = true)]
    active: Option<Entity>,
}

#[derive(Component)]
pub struct Cursor;

fn setup_cursor(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands
        .spawn_bundle(PbrBundle {
            mesh: meshes.add(Mesh::from(shape::UVSphere {
                radius: 0.1,
                sectors: 30,
                stacks: 30,
            })),
            material: materials.add(StandardMaterial {
                base_color: Color::rgba(1.0, 0.0, 0.0, 0.2),
                unlit: true,
                alpha_mode: AlphaMode::Blend,
                ..default()
            }),
            //visibility: Visibility { is_visible: true },
            ..default()
        })
        .insert(Cursor)
        .insert(NotShadowCaster)
        .insert(Keep)
        .insert(Name::new("Cursor"));
}

fn cursor_system(
    windows: Res<Windows>,
    camera_query: Query<(&Camera, &Transform), Without<Cursor>>,
    mut cusror_query: Query<(&mut Transform, &mut Visibility), With<Cursor>>,
    tlas: Res<Tlas>,
    colliders: Res<Assets<Collider>>,
    mouse_input: Res<Input<MouseButton>>,
    mut egui_context: ResMut<EguiContext>,
    mut inspector: ResMut<Inspector>,
) {
    if let Ok((camera, camera_transform)) = camera_query.get_single() {
        if let Some(window) = windows.get_primary() {
            if egui_context.ctx_mut().wants_pointer_input() {
                return;
            }
            if let Some(mouse_pos) = window.cursor_position() {
                let (mut cursor_trans, mut cursor_vis) = cusror_query.single_mut();
                // create a ray
                let mut ray = Ray::from_camera(camera, camera_transform, mouse_pos);

                // test ray agaist tlas and see if we hit
                let hit_maybe = ray.intersect_tlas(&tlas, &colliders);

                if let Some(hit) = hit_maybe {
                    // we could do something with the entity here
                    cursor_trans.translation = ray.origin + ray.direction * hit.distance;
                    cursor_vis.is_visible = true;
                } else {
                    cursor_vis.is_visible = false;
                }

                if mouse_input.pressed(MouseButton::Left) {
                    if let Some(hit) = hit_maybe {
                        inspector.active = Some(hit.entity);
                    } else {
                        inspector.active = None;
                    }
                }
            }
        }
    }
}
