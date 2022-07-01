use bevy::{math::vec3, prelude::*, window::PresentMode};
use bevy_inspector_egui::{WorldInspectorPlugin, InspectorPlugin, Inspectable};
use helper::{AppState, HelperPlugin};
use sly_camera_controller::*;
use sly_physics::{Collider,  PhysicsPlugin, RigidbodyBundle, RigidBodyMode};
mod helper;

fn main() {
    App::new()
        .insert_resource(WindowDescriptor {
            present_mode: PresentMode::Fifo,
            ..default()
        })
        .add_plugins(DefaultPlugins)
        .add_plugin(HelperPlugin)
        .add_plugin(WorldInspectorPlugin::default())
        .add_plugin(CameraControllerPlugin)
        .add_plugin(PhysicsPlugin)
        .add_plugin(InspectorPlugin::<Stack>::new())
        .add_startup_system(helper::setup_camera)
        .add_system_set(SystemSet::on_enter(AppState::Playing).with_system(setup))
        .run();
}

#[derive(Inspectable)]
pub struct Stack {
    count: u32,
}

impl Default for Stack {
    fn default() -> Self {
        Self { count: 1 }
    }
}

pub fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    asset_server: Res<AssetServer>,
    stact: Res<Stack>,
) {
    // ground
    commands
        .spawn_bundle(PbrBundle {
            transform: Transform::from_xyz(0.0, -100.0, 0.0),
            mesh: meshes.add(Mesh::from(shape::UVSphere {
                radius: 100.0,
                sectors: 64,
                stacks: 64,
            })),
            material: materials.add(StandardMaterial {
                base_color: Color::DARK_GREEN,
                ..default()
            }),
            ..default()
        })
        .insert_bundle(RigidbodyBundle {
            collider: Collider::Sphere { radius: 100.0 },
            mode: RigidBodyMode::Static,
            ..default()
        })
        .insert(Name::new("Ground"));

    let radius = 0.5;
    let size = stact.count;
    for i in 0..size {
        for j in 0..size {
            for k in 0..size {
                let pos = vec3(i as f32, j as f32 + radius, k as f32) * 1.1;
                commands
                    .spawn_bundle(PbrBundle {
                        mesh: meshes.add(Mesh::from(shape::UVSphere {
                            radius,
                            ..default()
                        })),
                        material: materials.add(StandardMaterial {
                            base_color_texture: Some(asset_server.load("checker_red.png")),
                            ..default()
                        }),
                        transform: Transform::from_translation(pos),
                        ..default()
                    })
                    .insert_bundle(RigidbodyBundle {
                        collider: Collider::Sphere { radius },
                        mode: RigidBodyMode::Dynamic,
                        ..default()
                    })
                    .insert(Name::new(format!("Sphere ({}, {}, {})", i, j, k)));
            }
        }
    }
}
