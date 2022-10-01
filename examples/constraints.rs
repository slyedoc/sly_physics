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
        .add_plugin(HelperPlugin)
        .add_startup_system(helper::setup_camera)
        .init_resource::<ItemAssets>()
        .add_enter_system(AppState::Playing, helper::setup_room)
        .add_enter_system(AppState::Playing, setup_motor_constraint)
        .add_enter_system(AppState::Playing, setup_orientation_constraint)
        .add_enter_system(AppState::Playing, setup_distance_constraint)
        .add_enter_system(AppState::Playing, setup_hinge_constraint)
        .add_enter_system(AppState::Playing, setup_hinge_limited_constraint)
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

    let anchor = commands
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
            b: Some(anchor),
            anchor_b: vec3(0., 2., 0.),
            motor_speed: 1.0,
            motor_axis: Vec3::Y,
            ..default()
        })
        .insert(Name::new("Motor"));
}

fn setup_orientation_constraint(mut commands: Commands, box_config: Res<ItemAssets>) {
    let pos = vec3(0., 5., 0.);

    let anchor = commands
        .spawn_bundle(PbrBundle {
            mesh: box_config.box_mesh.clone(),
            material: box_config.material.clone(),
            transform: Transform::from_translation(pos),
            ..default()
        })
        .insert_bundle(RigidBodyBundle {
            collider: box_config.box_collider.clone(),
            mode: RigidBody::Static,
            ..default()
        })
        .insert(Name::new("Orientation Anchor"))
        .id();

    commands
        .spawn_bundle(PbrBundle {
            mesh: box_config.box_mesh.clone(),
            material: box_config.material.clone(),
            transform: Transform::from_translation(pos + vec3(1., -2., 0.0)),
            ..default()
        })
        .insert_bundle(RigidBodyBundle {
            collider: box_config.box_collider.clone(),
            mass: Mass(0.1),
            ..default()
        })
        .insert(OrientationConstraint {
            anchor_a: vec3(0., 2.0, 0.),
            b: Some(anchor),
            
            ..default()
        })
        .insert(Name::new("Orientation"));
}

fn setup_distance_constraint(mut commands: Commands, box_config: Res<ItemAssets>) {
    let pos = vec3(-4., 10., -5.);

    let anchor = commands
        .spawn_bundle(PbrBundle {
            mesh: box_config.box_mesh.clone(),
            material: box_config.material.clone(),
            transform: Transform::from_translation(pos),
            ..default()
        })
        .insert_bundle(RigidBodyBundle {
            collider: box_config.box_collider.clone(),
            mode: RigidBody::Static,
            ..default()
        })
        .insert(Name::new("Distance Anchor"))
        .id();

        // create links
        let mut b = anchor;
        let offset = 1.2;
        for i in 0..5 {
            let link = commands
                .spawn_bundle(PbrBundle {
                    mesh: box_config.box_mesh.clone(),
                    material: box_config.material.clone(),
                    transform: Transform::from_translation(pos + vec3(i as f32, (i + 1) as f32 * -offset, 0.0)),
                    ..default()
                })
                .insert_bundle(RigidBodyBundle {
                    collider: box_config.box_collider.clone(),
                    mass: Mass(0.1),
                    ..default()
                })
                .insert(DistanceConstraint {
                    b: Some(b),
                    anchor_a: vec3(0., offset, 0.),
                    ..default()
                })
                .insert(Name::new(format!("Distance {i}")))
                .id();
            b = link;
        }

}

fn setup_hinge_constraint(mut commands: Commands, box_config: Res<ItemAssets>) {
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
            mode: RigidBody::Static,
            ..default()
        })
        .insert(Name::new("Hinge Anchor"))
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
        .insert(HingeConstraint {
            anchor_a: vec3(-0.5, 0.5, 0.0),
            anchor_b: vec3(-0.5, -0.5, 0.),
            b: Some(parent),

            ..default()
        })
        .insert(Name::new("Hinge Quat"));
}

fn setup_hinge_limited_constraint(mut commands: Commands, box_config: Res<ItemAssets>) {
    let pos = vec3(-12., 2.5, 0.);

    let anchor = commands
        .spawn_bundle(PbrBundle {
            mesh: box_config.box_mesh.clone(),
            material: box_config.material.clone(),
            transform: Transform::from_translation(pos + vec3(0.0, 2.0, 0.0)),
            ..default()
        })
        .insert_bundle(RigidBodyBundle {
            collider: box_config.box_collider.clone(),
            mode: RigidBody::Static,
            ..default()
        })
        .insert(Name::new("Hinge Limited Anchor"))
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
        .insert(HingeLimitedConstraint {
            anchor_a: vec3(-0.5, 0.5, 0.),
            b: Some(anchor),
            anchor_b: vec3(-0.5, -0.5, 0.),
            axis: Vec3::Z,
            ..default()
        })
        .insert(Name::new("Hinge Limited"));
}
