use bevy::{math::vec3, prelude::*, window::PresentMode};
use bevy_inspector_egui::prelude::*;
use helper::{AppState, HelperPlugin};
use iyes_loopless::prelude::*;
use sly_camera_controller::*;
use sly_physics::prelude::*;
mod helper;

fn main() {
    App::new()
        .insert_resource(WindowDescriptor {
            present_mode: PresentMode::AutoNoVsync,
            ..default()
        })
        .add_plugins(DefaultPlugins)
        .add_plugin(WorldInspectorPlugin::default())
        // our phsycis plugin
        .add_plugin(PhysicsPlugin)
        .add_plugin(GravityPlugin)
        .add_plugin(PhysicsDebugPlugin)
        //.add_plugin(PhysicsBvhCameraPlugin)
        // testing aabb debug plugin
        //.add_plugin(DebugAabbPlugin)
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
    Cube,
}

impl Default for Stack {
    fn default() -> Self {
        Self {
            count: (10, 10, 10),
            spacing: 1.1,
            time_scale: 1.0,
            mode: StackMode::Cube,
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
    mut colliders: ResMut<Assets<Collider>>,
) {
    // Stack
    let radius = 0.5;
    let size = stack.count;

    let mat = materials.add(StandardMaterial {
        base_color_texture: Some(asset_server.load("checker_red.png")),
        ..default()
    });

    let (mesh, collider) = match stack.mode {
        StackMode::Sphere => (
            meshes.add(Mesh::from(shape::UVSphere {
                radius,
                ..default()
            })),
            colliders.add(Collider::from(Sphere::new(radius))),
        ),
        StackMode::Cube => (
            meshes.add(Mesh::from(shape::Box::new(1.0, 1.0, 1.0))),
            colliders.add(Collider::from(Box::new(Vec3::splat(1.0)))),
        ),
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
                        mesh: mesh.clone(),
                        material: mat.clone(),
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
            velocity: Velocity {
                linear: vec3(-stack.ball_velocity, 0.0, 0.0),
                ..default()
            },
            collider: colliders.add(Collider::from(Sphere::new(wreak_ball_radius))),
            mass: Mass(20.0),
            ..default()
        })
        .insert(Name::new("Wreaking Ball"));

    // Diamond

    let diamond = make_diamond_convex_shape();
    let diamond_convex_collider = Convex::new(&diamond);
    let diamond_mesh = Mesh::from(&diamond_convex_collider);


    commands
        .spawn_bundle(PbrBundle {
            transform: Transform::from_xyz(30.0, wreak_ball_radius + 1.0, 10.0),
            mesh: meshes.add(Mesh::from(diamond_mesh)),
            material: materials.add(StandardMaterial {
                base_color: Color::rgb(0.0, 0.0, 1.0),
                ..default()
            }),
            ..default()
        })
        .insert_bundle(RigidBodyBundle {
            collider: colliders.add(Collider::from(diamond_convex_collider)),
            mass: Mass(2.0),
            ..default()
        })
        .insert(Name::new("Diamond"));
}

fn make_diamond_convex_shape() -> Vec<Vec3> {
    let mut diamond = [Vec3::ZERO; 7 * 8];
    let quat_half = Quat::from_rotation_y(2.0 * std::f32::consts::PI * 0.125 * 0.5);
    let mut pts = [Vec3::ZERO; 7];
    pts[0] = Vec3::new(0.1, -1.0, 0.0);
    pts[1] = Vec3::new(1.0, 0.0, 0.0);
    pts[2] = Vec3::new(1.0, 0.1, 0.0);
    pts[3] = Vec3::new(0.4, 0.4, 0.0);
    pts[4] = Vec3::new(0.8, 0.3, 0.0);
    pts[4] = quat_half * pts[4];
    pts[5] = quat_half * pts[1];
    pts[6] = quat_half * pts[2];

    let quat = Quat::from_rotation_y(2.0 * std::f32::consts::PI * 0.125);
    let mut idx = 0;
    for pt in &pts {
        diamond[idx] = *pt;
        idx += 1;
    }

    let mut quat_acc = Quat::IDENTITY;
    for _ in 1..8 {
        quat_acc *= quat;
        for pt in &pts {
            diamond[idx] = quat_acc * *pt;
            idx += 1;
        }
    }

    diamond.to_vec()
}
