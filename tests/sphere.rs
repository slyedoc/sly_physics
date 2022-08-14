use bevy::{
    math::{vec3},
    prelude::*,
};
use sly_physics::prelude::*;

#[test]
fn did_sphere_fall() {
    // Setup app
    let mut app = App::new();
    app.add_plugin(PhysicsPlugin);

    app.add_startup_system(spawn_ground);

    let mut collider_resources = app.world.get_resource_mut::<ColliderResources>().unwrap();
    let sphere_collider = collider_resources.add_sphere(0.5);
    let id = app
        .world
        .spawn()
        .insert_bundle(SpatialBundle {
            transform: Transform::from_translation(vec3(0.0, 0.0, 0.0)),
            ..default()
        })
        .insert_bundle(RigidBodyBundle {
            collider: sphere_collider,
            mode: RigidBodyMode::Dynamic,
            elasticity: Elasticity(1.0),
            ..default()
        })
        .id();

    // Run systems
    app.update();

    // Check resulting changes
    assert!(app.world.get::<Transform>(id).is_some());

    info!("Translation {:?}", app.world.get::<Transform>(id).unwrap().translation);
    assert_eq!(app.world.get::<Transform>(id).unwrap().translation, vec3(0.0, 0.0, 0.0));
}

fn spawn_ground(
    mut commands: Commands,
    mut collider_resources: ResMut<ColliderResources>,
) {

    commands
        .spawn_bundle(SpatialBundle {
            transform: Transform::from_translation(Vec3::ZERO),
            ..default()
        })
        .insert_bundle(RigidBodyBundle {
            collider: collider_resources.add_box(vec3(10.0, 1.0, 10.0)),
            mode: RigidBodyMode::Static,
            elasticity: Elasticity(1.0),
            ..default()
        });
}
