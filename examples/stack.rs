use std::f32::consts::*;

use bevy::{math::vec3, prelude::*, window::PresentMode};
use bevy_inspector_egui::{Inspectable, InspectorPlugin, WorldInspectorPlugin};
use helper::{AppState, HelperPlugin};
use iyes_loopless::prelude::*;
use sly_camera_controller::*;
use sly_physics::prelude::*;
mod helper;

fn main() {
    App::new()
        .insert_resource(WindowDescriptor {
            present_mode: PresentMode::Fifo,
            ..default()
        })
        .add_plugins(DefaultPlugins)
        .add_plugin(WorldInspectorPlugin::default())
       
        // our phsycis plugin
        .add_plugin(PhysicsPlugin)
        .add_plugin(PhysicsDebugPlugin)
        .add_plugin(PhysicsBvhCameraPlugin)

        // local setup stuff
        .add_plugin(HelperPlugin)
        .add_plugin(CameraControllerPlugin)
        .add_plugin(InspectorPlugin::<Stack>::new())
        .add_startup_system(helper::setup_camera)
        .add_enter_system(AppState::Playing, setup_room)
        .add_system(apply_scale)
        .run();
}

fn apply_scale(mut config: ResMut<PhysicsConfig>, stack: Res<Stack>) {
    config.time = stack.time_scale * PHYSISCS_TIMESTEP as f32;
}

#[derive(Inspectable)]
pub struct Stack {
    count: (u32, u32, u32),
    spacing: f32,
    //#[inspector()]
    time_scale: f32,
    mode: StackMode,
    ball_velocity: f32,
}

#[derive(Inspectable)]
pub enum StackMode {
    Sphere,
    Cube,
}

impl Default for Stack {
    fn default() -> Self {
        Self {
            count: (1u32,5u32,1u32),
            spacing: 1.0,
            time_scale: 1.0,
            mode: StackMode::Cube,
            ball_velocity: 60.0,
            
        }
    }
}

pub fn setup_room(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    asset_server: Res<AssetServer>,
    stack: Res<Stack>,
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
        .insert_bundle(RigidBodyBundle {
            collider: Collider::Cuboid {
                size: vec3(floor_size, 1.0, floor_size),
            },
            mode: RigidBodyMode::Static,
            ..default()
        })
        .insert(Name::new("Floor"));

    // walls
    for wall in 0..4 {
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
            .insert_bundle(RigidBodyBundle {
                collider: Collider::Cuboid {
                    size: vec3(floor_size, wall_height, 1.0),
                },
                mode: RigidBodyMode::Static,
                ..default()
            })
            .insert(Name::new("Wall"));
    }

    // Stack
    let radius = 0.5;
    let size = stack.count;
    for i in 0..size.0 {
        for j in 0..size.1 {
            for k in 0..size.2 {
                let pos = vec3(
                    i as f32,
                    j as f32 + radius,
                    k as f32 + radius - (radius * 2.0 * size.2 as f32 / 2.0),
                ) * stack.spacing;
                commands
                    .spawn_bundle(PbrBundle {                        
                        mesh: match stack.mode {
                            StackMode::Sphere => meshes.add(Mesh::from(shape::UVSphere {
                                radius,
                                ..default()
                            })),
                            StackMode::Cube => {
                                meshes.add(Mesh::from(shape::Box::new(1.0, 1.0, 1.0)))
                            }
                        },
                        material: materials.add(StandardMaterial {
                            base_color_texture: Some(asset_server.load("checker_red.png")),
                            ..default()
                        }),
                        transform: Transform::from_translation(pos),
                        ..default()
                    })
                    .insert_bundle(RigidBodyBundle {
                        collider: match stack.mode {
                            StackMode::Sphere => Collider::Sphere { radius: radius },
                            StackMode::Cube => Collider::Cuboid { size: Vec3::ONE },
                        },
                        ..default()
                    })
                    .insert(Name::new(format!("Sphere ({}, {}, {})", i, j, k)));
            }
        }
    }

    // Wreaking Ball

    let wreak_ball_radius = 2.0;

    commands
        .spawn_bundle(PbrBundle {
            transform: Transform::from_xyz(30.0, wreak_ball_radius + 1.0, 0.0),
            mesh: meshes.add(Mesh::from(shape::UVSphere {
                radius: wreak_ball_radius,
                sectors: 64,
                stacks: 64,
            })),
            material: materials.add(StandardMaterial {
                base_color_texture: Some(asset_server.load("checker_red.png")),
                ..default()
            }),
            ..default()
        })
        .insert_bundle(RigidBodyBundle {
            linear_velocity: LinearVelocity(vec3(-stack.ball_velocity, 0.0, 0.0)),
            collider: Collider::Sphere {
                radius: wreak_ball_radius,
            },
            mass: Mass(20.0),
            ..default()
        })
        .insert(Name::new("Wreaking Ball"));
}
