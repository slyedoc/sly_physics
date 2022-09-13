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
            present_mode: PresentMode::Fifo,
            ..default()
        })
        .add_plugins(DefaultPlugins)
        .add_plugin(WorldInspectorPlugin::default())
        
        // out plugins
        .add_plugin(PhysicsPlugin)
        .add_plugin(GravityPlugin)
        .add_plugin(PhysicsDebugPlugin)
        .add_plugin(PhysicsBvhCameraPlugin)
        
        .add_plugin(HelperPlugin)
        .add_plugin(CameraControllerPlugin)
        .add_startup_system(helper::setup_camera)
        .add_enter_system(AppState::Playing, helper::setup_room)
        .add_enter_system(AppState::Playing, setup)
        .run();
}

pub fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut collider_resources: ResMut<ColliderResources>,
    asset_server: Res<AssetServer>,
) {
    let box_a = collider_resources.add_box(Vec3::ONE);
    let box_mesh = meshes.add(Mesh::from(shape::Cube { size: 1.0 }));
    let box_mat = materials.add(StandardMaterial {
        base_color_texture: Some(asset_server.load("checker_red.png")),
        ..default()
    });

    commands
        .spawn_bundle(PbrBundle {
            mesh: box_mesh.clone(),
            material: box_mat.clone(),
            transform: Transform {
                //translation: vec3(-8.00000667, 9.4973526, 2.5000124),
                translation: vec3(8.00000667, 9.4973526, 2.5000124),
                rotation: Quat::from_xyzw(-1.19166614E-7, -0.00000143082571, 0.00000248814399, 1.0),
                ..default()
            },
            ..default()
        })
        .insert_bundle(RigidBodyBundle {
            collider: box_a.clone(),
            mass: Mass(1.0),
            ..default()
        });

    commands
        .spawn_bundle(PbrBundle {
            transform: Transform {
                translation: vec3(8.00001144, 8.49734306, 3.50001287),
                rotation: Quat::from_xyzw(9.16530154E-7, 5.70654834E-7, 0.00000295896189, 1.0),
                ..default()
            },
            mesh: box_mesh.clone(),
            material: box_mat.clone(),
            ..default()
        })
        .insert_bundle(RigidBodyBundle {
            collider: box_a,
            mass: Mass(1.0),
            ..default()
        });
}
