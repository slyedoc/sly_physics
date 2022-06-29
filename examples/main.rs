use bevy::{prelude::*, window::PresentMode};
use bevy_inspector_egui::WorldInspectorPlugin;
use sly_camera_controller::*;
use sly_physics::PhysicsPlugin;
mod helper;

#[derive(Clone, Eq, PartialEq, Debug, Hash)]
enum AppState {
    Reset,
    Playing,
}

fn main() {
    App::new()
        .add_state(AppState::Playing)
        .insert_resource(WindowDescriptor {
            present_mode: PresentMode::Fifo,
            ..default()
        })
        .add_plugins(DefaultPlugins)
        .add_plugin(WorldInspectorPlugin::default())
        .add_plugin(CameraControllerPlugin)
        .add_plugin(PhysicsPlugin)
        .add_startup_system(setup)
  
        .add_system_set(
            SystemSet::on_enter(AppState::Playing)                
                .with_system(helper::setup_sandbox),
        )
        .add_system_set(SystemSet::on_update(AppState::Playing).with_system(reset_listen))
        .add_system_set(SystemSet::on_enter(AppState::Reset).with_system(reset))
        .run();
}

fn setup(
    mut commands: Commands,
    mut _meshes: ResMut<Assets<Mesh>>,
    mut _materials: ResMut<Assets<StandardMaterial>>,
) {
    // light
    commands.spawn_bundle(DirectionalLightBundle {
        transform: Transform::from_xyz(50.0, 50.0, 50.0).looking_at(Vec3::ZERO, Vec3::Y),
        ..default()
    })
    .insert(Keep);

    // camera
    commands
        .spawn_bundle(PerspectiveCameraBundle {
            transform: Transform::from_xyz(0.0, 2.0, -2.0).looking_at(Vec3::ZERO, Vec3::Y),
            ..default()
        })
        // Add our controller
        .insert(CameraController::default())
        .insert(Keep);
}


// Theses system is used to reset the game when the user presses the 'r' key.
#[derive(Component)]
struct Keep;

fn reset(mut commands: Commands, query: Query<Entity, Without<Keep>>, mut state: ResMut<State<AppState>>) {
    for e in query.iter() {
        commands.entity(e).despawn();
    }
    state.set(AppState::Playing).unwrap();
}

fn reset_listen(mut input: ResMut<Input<KeyCode>>, mut state: ResMut<State<AppState>>) {
    if input.just_pressed(KeyCode::R) {
        state.set(AppState::Reset).unwrap();
        input.clear();
    }
}
