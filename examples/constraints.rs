use bevy::{math::vec3, prelude::*, window::PresentMode};
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
        .add_plugin(DebugBvhCameraPlugin)
        .add_plugin(HelperPlugin)
        .add_startup_system(helper::setup_camera)
        .init_resource::<ItemAssets>()
        .add_enter_system(AppState::Playing, helper::setup_room)
        .add_enter_system(AppState::Playing, setup_motor_constraint)
        .add_enter_system(AppState::Playing, setup_orientation_constraint)
        .add_enter_system(AppState::Playing, setup_distance_constraint)
        .add_enter_system(AppState::Playing, setup_hinge_quat_constraint)
        .add_enter_system(AppState::Playing, setup_hinge_quat_limited_constraint)
        .run();
}

struct ItemAssets {
    material: Handle<StandardMaterial>,

    box_mesh: Handle<Mesh>,
    box_collider: Handle<Collider>,

    bar_mesh: Handle<Mesh>,
    bar_collider: Handle<Collider>,
}

impl FromWorld for ItemAssets {
    fn from_world(world: &mut World) -> Self {
        let mut meshes = world.get_resource_mut::<Assets<Mesh>>().unwrap();
        let box_mesh = meshes.add(Mesh::from(shape::Cube { size: 1.0 }));
        let bar_mesh = meshes.add(Mesh::from(shape::Box::new(5.0, 1.0, 1.0)));

        let asset_server = world.get_resource::<AssetServer>().unwrap();
        let texture = asset_server.load("checker_red.png");
        let mut materials = world
            .get_resource_mut::<Assets<StandardMaterial>>()
            .unwrap();
        let material = materials.add(StandardMaterial {
            base_color_texture: Some(texture),
            ..default()
        });

        let mut colliders = world.get_resource_mut::<Assets<Collider>>().unwrap();
        let box_collider = colliders.add(Collider::from(Box::new(Vec3::ONE)));
        let bar_collider = colliders.add(Collider::from(Box::new(vec3(5.0, 1.0, 1.0))));

        Self {
            box_mesh,
            box_collider,
            material,
            bar_mesh,
            bar_collider,
        }
    }
}

fn setup_motor_constraint(mut commands: Commands, item_assets: Res<ItemAssets>) {
    let pos = vec3(4., 0.5, 0.);

    let base = commands
        .spawn_bundle(PbrBundle {
            mesh: item_assets.box_mesh.clone(),
            material: item_assets.material.clone(),
            transform: Transform::from_translation(pos),
            ..default()
        })
        .insert_bundle(RigidBodyBundle {
            collider: item_assets.box_collider.clone(),
            mass: Mass(1.0),

            ..default()
        })
        .insert(Name::new("Motor Anchor"))
        .id();

    // motor
    commands
        .spawn_bundle(PbrBundle {
            mesh: item_assets.bar_mesh.clone(),
            material: item_assets.material.clone(),
            transform: Transform::from_translation(pos + vec3(0., 2., 0.)),
            ..default()
        })
        .insert_bundle(RigidBodyBundle {
            collider: item_assets.bar_collider.clone(),
            mass: Mass(0.1),
            ..default()
        })
        // Add our motor
        .insert(MotorConstraint {
            parent: Some(base),
            anchor_b: vec3(0., 2., 0.),
            motor_speed: 1.0,
            motor_axis: Vec3::Y,
            ..default()
        })
        .insert(Name::new("Motor"));
}

fn setup_orientation_constraint(mut commands: Commands, box_config: Res<ItemAssets>) {
    let pos = vec3(0., 0.5, 0.);

    let base = commands
        .spawn_bundle(PbrBundle {
            mesh: box_config.box_mesh.clone(),
            material: box_config.material.clone(),
            transform: Transform::from_translation(pos),
            ..default()
        })
        .insert_bundle(RigidBodyBundle {
            collider: box_config.box_collider.clone(),
            mass: Mass(1.0),
            ..default()
        })
        .insert(Name::new("Orientation base"))
        .id();

    commands
        .spawn_bundle(PbrBundle {
            mesh: box_config.box_mesh.clone(),
            material: box_config.material.clone(),
            transform: Transform::from_translation(pos + vec3(0., 2., 0.0)),
            ..default()
        })
        .insert_bundle(RigidBodyBundle {
            collider: box_config.box_collider.clone(),
            mass: Mass(0.1),
            ..default()
        })
        .insert(OrientationConstraint {
            parent: Some(base),
            anchor_b: vec3(0., 2.0, 0.),
            ..default()
        })
        .insert(Name::new("Orientation"));
}

fn setup_distance_constraint(mut commands: Commands, box_config: Res<ItemAssets>) {
    let pos = vec3(-4., 0.5, 0.);

    let parent = commands
        .spawn_bundle(PbrBundle {
            mesh: box_config.box_mesh.clone(),
            material: box_config.material.clone(),
            transform: Transform::from_translation(pos),
            ..default()
        })
        .insert_bundle(RigidBodyBundle {
            collider: box_config.box_collider.clone(),
            mass: Mass(1.0),
            ..default()
        })
        .insert(Name::new("Distance base"))
        .id();

    commands
        .spawn_bundle(PbrBundle {
            mesh: box_config.box_mesh.clone(),
            material: box_config.material.clone(),
            transform: Transform::from_translation(pos + vec3(0., 2., 0.)),
            ..default()
        })
        .insert_bundle(RigidBodyBundle {
            collider: box_config.box_collider.clone(),
            mass: Mass(1.0),
            ..default()
        })
        .insert(DistanceConstraint {
            parent_offset: vec3(0., 2., 0.),
            parent: Some(parent),
            ..default()
        })
        .insert(Name::new("Distance"));
}

fn setup_hinge_quat_constraint(mut commands: Commands, box_config: Res<ItemAssets>) {
    let pos = vec3(-8., 2.5, 0.);

    let parent = commands
        .spawn_bundle(PbrBundle {
            mesh: box_config.box_mesh.clone(),
            material: box_config.material.clone(),
            transform: Transform::from_translation(pos + vec3(0.0, 2.0, 0.0)),
            ..default()
        })
        .insert_bundle(RigidBodyBundle {
            collider: box_config.box_collider.clone(),
            mass: Mass(1.0),
            ..default()
        })
        .insert(Name::new("Hinge Quat Parent"))
        .id();

    commands
        .spawn_bundle(PbrBundle {
            mesh: box_config.box_mesh.clone(),
            material: box_config.material.clone(),
            transform: Transform::from_translation(pos),
            ..default()
        })
        .insert_bundle(RigidBodyBundle {
            collider: box_config.box_collider.clone(),
            mass: Mass(1.0),
            ..default()
        })
        .insert(HingeQuatConstraint {
            offset: vec3(0., 1.0, 0.),
            parent: Some(parent),
            ..default()
        })
        .insert(Name::new("Hinge Quat"));
}

fn setup_hinge_quat_limited_constraint(mut commands: Commands, box_config: Res<ItemAssets>) {
    let pos = vec3(-12., 2.5, 0.);

    let parent = commands
        .spawn_bundle(PbrBundle {
            mesh: box_config.box_mesh.clone(),
            material: box_config.material.clone(),
            transform: Transform::from_translation(pos + vec3(0.0, 2.0, 0.0)),
            ..default()
        })
        .insert_bundle(RigidBodyBundle {
            collider: box_config.box_collider.clone(),
            mass: Mass(1.0),
            ..default()
        })
        .insert(Name::new("Hinge Quat Limited Parent"))
        .id();

    commands
        .spawn_bundle(PbrBundle {
            mesh: box_config.box_mesh.clone(),
            material: box_config.material.clone(),
            transform: Transform::from_translation(pos),
            ..default()
        })
        .insert_bundle(RigidBodyBundle {
            collider: box_config.box_collider.clone(),
            mass: Mass(1.0),
            ..default()
        })
        .insert(HingeQuatLimitedConstraint {
            offset: vec3(0., 1.0, 0.),
            parent: Some(parent),
            ..default()
        })
        .insert(Name::new("Hinge Quat Limited"));
}
