use bevy::{prelude::*, window::PresentMode};
use bevy_inspector_egui::prelude::*;
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
        .add_plugin(HelperPlugin)
        .add_plugin(WorldInspectorPlugin::default())
        .add_plugin(CameraControllerPlugin)
        // our phsycis plugin
        .add_plugin(PhysicsPlugin)
        
        .add_plugin(PhysicsDebugPlugin)
        .add_plugin(PhysicsBvhCameraPlugin)

        .add_startup_system(helper::setup_camera)
        .add_enter_system(AppState::Playing, helper::setup_room)
        .add_enter_system(AppState::Playing, setup)
        .run();
}

pub fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    asset_server: Res<AssetServer>,
    mut collider_resources: ResMut<ColliderResources>,
) {
    let radius = 1.0;
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
            transform: Transform::from_translation(Vec3::Y * 4.0),
            ..default()
        })
        .insert_bundle(RigidBodyBundle {
            collider: collider_resources.add_sphere(radius),
            mode: RigidBodyMode::Dynamic,
            elasticity: Elasticity(1.0),
            ..default()
        })
        .insert(Name::new("Sphere A"));
}
