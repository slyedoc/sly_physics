use bevy::{prelude::*, window::PresentMode, math::vec3};
use bevy_inspector_egui::prelude::*;
use helper::{AppState, HelperPlugin};
use iyes_loopless::prelude::*;
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

        .add_plugin(HelperPlugin)
        .add_startup_system(helper::setup_camera)        
        //.add_enter_system(AppState::Playing, helper::setup_room)
        .add_enter_system(AppState::Playing, setup_test)
        .run();
}

fn setup_test(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut colliders: ResMut<Assets<Collider>>,
) {

    commands.spawn_bundle(PbrBundle {
        mesh: meshes.add(Mesh::from(shape::Box::new( 10.0, 1., 10.))),
        material: materials.add(Color::GREEN.into()),
        transform: Transform::from_xyz(0.0, -0.5, 0.0),
        ..default()
    })
    .insert_bundle(RigidBodyBundle {
        collider: colliders.add(Collider::from(Box::new(vec3(10.0, 1., 10.)))),
        mode: RigidBody::Static,
        ..default()
    })
    .insert(Name::new("Ground"));


    commands.spawn_bundle(PbrBundle {
        mesh: meshes.add(Mesh::from(shape::Cube { size: 1.0 })),
        material: materials.add(Color::DARK_GRAY.into()),
        transform: Transform::from_xyz(0.0, 1.0, 0.0),
        ..default()
    })
    .insert_bundle(RigidBodyBundle {
        collider: colliders.add(Collider::from(Box::new(Vec3::ONE))),

        ..default()
    })
    .insert(Name::new("Box A"));

    commands.spawn_bundle(PbrBundle {
        transform: Transform::from_xyz(0.0, 2.0, 0.0),
        mesh: meshes.add(Mesh::from(shape::Cube { size: 1.0 })),
        material: materials.add(Color::rgb(0.8, 0.7, 0.6).into()),

        ..default()
    })
    .insert_bundle(RigidBodyBundle {
        collider: colliders.add(Collider::from(Box::new(Vec3::ONE))),
        ..default()
    })
    .insert(Name::new("Box B"));
}