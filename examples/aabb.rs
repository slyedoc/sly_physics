//! A shader that renders a mesh multiple times in one draw call.
mod helper;
use bevy::{prelude::*, math::vec3};
use bevy_inspector_egui::WorldInspectorPlugin;
use sly_camera_controller::CameraControllerPlugin;
use sly_physics::prelude::*;

fn main() {
    let mut app = App::new();
    app.add_plugins(DefaultPlugins)
        .add_plugin(WorldInspectorPlugin::default())
        // out plugins
        .add_plugin(PhysicsPlugin)
        .add_plugin(GravityPlugin)
        .add_plugin(PhysicsDebugPlugin)        
        .add_plugin(PhysicsBvhCameraPlugin)
        // Out testing plugin
        // example stuff
        .add_plugin(helper::HelperPlugin)
        .add_plugin(CameraControllerPlugin)
        .add_startup_system(helper::setup_camera)
        .add_startup_system(helper::setup_room)
        .add_startup_system(setup_box);

    app.run();
}


fn setup_box(
    mut commands: Commands,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut collider_resources: ResMut<ColliderResources>,
    mut meshes: ResMut<Assets<Mesh>>,
) {

     // floor
     commands
     .spawn_bundle(PbrBundle {
         transform: Transform::from_xyz(0.0, 1.0, 0.0),
         mesh: meshes.add(Mesh::from(shape::Box::new(1.0, 1.0, 1.0))),
         material: materials.add(StandardMaterial {
             base_color: Color::rgba(1.0, 0.0, 0.0, 0.1),
             alpha_mode: AlphaMode::Blend,
             unlit: true,
             ..default()
         }),
         ..default()
     })
     .insert_bundle(RigidBodyBundle {
         collider: collider_resources.add_box(vec3(1.0, 1.0, 1.0)),
         mode: RigidBody::Static,
         ..default()
     })
     .insert(Name::new("Box"));
}