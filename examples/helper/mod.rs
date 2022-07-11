use bevy::prelude::*;
use sly_camera_controller::CameraController;
use sly_physics::prelude::*;

use overlay::OverlayPlugin;
mod overlay;

pub struct HelperPlugin;

impl Plugin for HelperPlugin {
    fn build(&self, app: &mut App) {
        app.add_state(AppState::Playing)
            .add_plugin(OverlayPlugin)
            .add_system_set(SystemSet::on_update(AppState::Playing).with_system(reset_listen))         
            .add_system_set(SystemSet::on_enter(AppState::Reset).with_system(reset))
            .add_system(toggle_state)
            .add_system(toggle_debug);
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
            transform: Transform::from_xyz(0.0, 2.0, -2.0).looking_at(Vec3::ZERO, Vec3::Y),
            ..default()
        })
        // Add our controller
        .insert(CameraController::default())
        .insert(Keep);
}


fn toggle_state(mut input: ResMut<Input<KeyCode>>, mut state: ResMut<State<PhysicsState>>) {
    if input.just_pressed(KeyCode::Space) {
        match state.current() {            
            PhysicsState::Running => state.push(PhysicsState::Paused).unwrap(),
            PhysicsState::Paused => state.pop().unwrap(),
        }
        input.clear();
    }
}


fn toggle_debug(mut input: ResMut<Input<KeyCode>>, mut state: ResMut<State<DebugState>>) {
    if input.just_pressed(KeyCode::Key1) {
        match state.current() {
            DebugState::Running => state.push(DebugState::Paused).unwrap(),
            DebugState::Paused => state.pop().unwrap(),
        }
        input.clear();
    }
}

// Theses system is used to reset the game when the user presses the 'r' key.
#[derive(Component)]
pub struct Keep;

pub fn reset(
    mut commands: Commands,
    query: Query<Entity, Without<Keep>>,
    mut state: ResMut<State<AppState>>,
) {
    for e in query.iter() {
        commands.entity(e).despawn();
    }
    state.set(AppState::Playing).unwrap();
}

pub fn reset_listen(mut input: ResMut<Input<KeyCode>>, mut state: ResMut<State<AppState>>) {
    if input.just_pressed(KeyCode::R) {
        state.set(AppState::Reset).unwrap();
        input.clear();
    }
}


// pub fn setup_sandbox(
//     mut commands: Commands,
//     mut meshes: ResMut<Assets<Mesh>>,
//     mut materials: ResMut<Assets<StandardMaterial>>,
//     asset_server: Res<AssetServer>,
// ) {
//     // ground
//     commands
//         .spawn_bundle(PbrBundle {
//             transform: Transform::from_xyz(0.0, -100.0, 0.0),
//             mesh: meshes.add(Mesh::from(shape::UVSphere {
//                 radius: 100.0,
//                 ..default()
//             })),
//             material: materials.add(StandardMaterial {
//                 base_color: Color::DARK_GREEN,
//                 ..default()
//             }),
//             ..default()
//         })
//         .insert(Collider::sphere(100.0))
//         .insert(Static)
//         .insert(Name::new("Ground"));

//     let radius = 0.5;
//     commands
//         .spawn_bundle(PbrBundle {
//             mesh: meshes.add(Mesh::from(shape::UVSphere {
//                 radius,
//                 ..default()
//             })),
//             material: materials.add(StandardMaterial {
//                 base_color_texture: Some(asset_server.load("checker_red.png")),
//                 ..default()
//             }),
//             transform: Transform::from_xyz(0.0, radius, 0.0),
//             ..default()
//         })
//         .insert(Collider::sphere(radius))
//         //.insert(Static)
//         //.insert(ActiveEvents::COLLISION_EVENTS)
//         //.insert(LockedAxes::ROTATION_LOCKED)
//         //.insert(ColliderMassProperties::Density(2.0))
//         .insert(Name::new("Sphere"));
// }
