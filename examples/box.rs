
use std::f32::consts::*;

use bevy::{math::vec3, prelude::*, window::PresentMode};
use bevy_inspector_egui::{Inspectable, InspectorPlugin, WorldInspectorPlugin};
use helper::{AppState, HelperPlugin};
use sly_camera_controller::*;
use sly_physics::{
    Collider, PhysicsConfig, PhysicsPlugin, RigidBodyMode, RigidbodyBundle, PHYSISCS_TIMESTEP,
};
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
        .add_system_set(SystemSet::on_enter(AppState::Playing).with_system(setup_room))
        .add_system(apply_scale)
        .run();
}

fn apply_scale(mut config: ResMut<PhysicsConfig>, stack: Res<Stack>) {
    config.time = stack.time_scale * PHYSISCS_TIMESTEP as f32;
}

#[derive(Inspectable)]
pub struct Stack {
    count: u32,
    //#[inspector()]
    time_scale: f32,
}

impl Default for Stack {
    fn default() -> Self {
        Self {
            count: 5,
            time_scale: 1.0,
        }
    }
}

pub fn setup_room(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    asset_server: Res<AssetServer>,
    stact: Res<Stack>,
) {

    let floor_size = 100.0;
    let wall_height = 10.0;
    let floor_half = floor_size * 0.5;
    let wall_height_half = wall_height * 0.5;
    // floor
    commands
        .spawn_bundle(PbrBundle {
            transform: Transform::from_xyz(0.0, -0.5, 0.0),
            mesh: meshes.add(Mesh::from(shape::Box::new(floor_size, 1.0, floor_size))),
            material: materials.add(StandardMaterial {
                base_color: Color::DARK_GREEN,
                ..default()
            }),
            ..default()
        })
        .insert_bundle(RigidbodyBundle {
            collider: Collider::Cuboid {
                size: vec3(floor_size, 1.0, floor_size),
            },
            mode: RigidBodyMode::Static,
            ..default()
        })
        .insert(Name::new("Floor"));


    // walls
    for wall in 0..2 {
        let mut transform = Transform::from_xyz(0.0, wall_height_half, floor_half);       
        transform.rotate_around(
            Vec3::ZERO,
            Quat::from_axis_angle(Vec3::Y, wall as f32 * FRAC_PI_2),
        );

        commands
            .spawn_bundle(PbrBundle {
                transform,
                mesh: meshes.add(Mesh::from(shape::Box::new(floor_size, wall_height, 1.0))),
                material: materials.add(StandardMaterial {
                    base_color: Color::ALICE_BLUE,
                    ..default()
                }),
                ..default()
            })
            .insert_bundle(RigidbodyBundle {
                collider: Collider::Cuboid {
                    size: vec3(floor_size, wall_height, 1.0),
                },
                mode: RigidBodyMode::Static,
                ..default()
            })
            .insert(Name::new("Wall"));
    }

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
