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
        .add_enter_system(AppState::Playing, helper::setup_room)
        .add_enter_system(AppState::Playing, setup)
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
    Box,
}

impl Default for Stack {
    fn default() -> Self {
        Self {
            count: (6u32, 6u32, 6u32),
            spacing: 1.0,
            time_scale: 1.0,
            mode: StackMode::Sphere,
            ball_velocity: 60.0,
        }
    }
}

pub fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    asset_server: Res<AssetServer>,
    stack: Res<Stack>,
    mut collider_resources: ResMut<ColliderResources>,
) {
    // Stack
    let radius = 0.5;
    let size = stack.count;

    let collider = match stack.mode {
        StackMode::Sphere => collider_resources.add_sphere(radius),
        StackMode::Box => collider_resources.add_box(Vec3::splat(1.0)),
    };

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
                            StackMode::Box => {
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
                        collider: collider.clone(),
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
            collider: collider_resources.add_sphere(wreak_ball_radius),
            mass: Mass(20.0),
            ..default()
        })
        .insert(Name::new("Wreaking Ball"));
}
