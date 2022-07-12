use bevy::prelude::*;
use iyes_loopless::prelude::*;
use sly_camera_controller::CameraController;
use sly_physics::prelude::*;

use overlay::OverlayPlugin;
mod overlay;

pub struct HelperPlugin;

impl Plugin for HelperPlugin {
    fn build(&self, app: &mut App) {
        app.add_loopless_state(AppState::Playing)
            .add_plugin(OverlayPlugin)
            .add_system(reset_listen.run_in_state(AppState::Playing))
            .add_enter_system(AppState::Reset, reset)
            .add_system(toggle_physics)
            .add_system(toggle_physics_debug);
    }
}

#[derive(Clone, Eq, PartialEq, Debug, Hash)]
pub enum AppState {
    Reset,
    Playing,
}

pub fn setup_camera(mut commands: Commands) {
    // light
    commands
        .spawn_bundle(DirectionalLightBundle {
            transform: Transform::from_xyz(50.0, 50.0, 50.0).looking_at(Vec3::ZERO, Vec3::Y),
            ..default()
        })
        .insert(Keep);

    // cameras
    commands
        .spawn_bundle(UiCameraBundle::default())
        .insert(Keep);

    commands
        .spawn_bundle(PerspectiveCameraBundle {
            transform: Transform::from_xyz(0.0, 2.0, -30.0).looking_at(Vec3::ZERO, Vec3::Y),
            ..default()
        })
        .insert(BvhCamera::new(256, 256))
        // Add our controller
        .insert(CameraController::default())
        .insert(Keep);
}


fn toggle_physics(
    mut commands: Commands,
    input: Res<Input<KeyCode>>,
    state: Res<CurrentState<PhysicsState>>,
) {
    if input.just_pressed(KeyCode::Space) {
        let target = match state.0 {            
            PhysicsState::Running => PhysicsState::Paused,
            PhysicsState::Paused => PhysicsState::Running,
        };
        commands.insert_resource(NextState(target));
    }
}


fn toggle_physics_debug(
    mut commands: Commands,
    input: Res<Input<KeyCode>>,    
    state: Res<CurrentState<PhysicsDebugState>>,
) {
    if input.just_pressed(KeyCode::Key1) {
        let target = match state.0 {
            PhysicsDebugState::Running => PhysicsDebugState::Paused,
            PhysicsDebugState::Paused => PhysicsDebugState::Running,
        };
        commands.insert_resource(NextState(target));
    }
}

// Theses system is used to reset the game when the user presses the 'r' key.
#[derive(Component)]
pub struct Keep;

pub fn reset(
    mut commands: Commands,
    query: Query<Entity, Without<Keep>>,
) {
    for e in query.iter() {
        commands.entity(e).despawn();
    }
    commands.insert_resource(NextState(AppState::Playing));
}

pub fn reset_listen(
    mut commands: Commands,
    input: Res<Input<KeyCode>>,
) {
    if input.just_pressed(KeyCode::R) {
        commands.insert_resource(NextState(AppState::Reset));
    }
}

