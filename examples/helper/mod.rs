mod assets;
mod interaction_menu_overlay;
mod picking;
mod text_overlay;

use std::f32::consts::FRAC_PI_2;

use bevy::{math::vec3, prelude::*};
use bevy_inspector_egui::{InspectorPlugin, Inspectable, widgets::ResourceInspector};
use iyes_loopless::prelude::*;
use sly_camera_controller::{CameraController, CameraControllerPlugin};
use sly_physics::prelude::*;

use text_overlay::TextOverlayPlugin;
//use interaction_menu_overlay::InteractionMenuOverlayPlugin;
use assets::*;
use picking::PickingPlugin;

pub struct HelperPlugin;

impl Plugin for HelperPlugin {
    fn build(&self, app: &mut App) {
        app.add_loopless_state(AppState::Playing)
            .add_plugin(InspectorPlugin::<DebugConfig>::new())
            .init_resource::<FontAssets>()
            .init_resource::<ButtonColors>()
            .add_plugin(CameraControllerPlugin)
            .add_plugin(TextOverlayPlugin)
            //.add_plugin(InteractionMenuOverlayPlugin)
            .add_plugin(PickingPlugin)
            .add_system_to_stage(
                CoreStage::Update,
                reset_listen.run_in_state(AppState::Playing),
            )
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

#[derive(Default, Inspectable)]
pub struct DebugConfig {
    config: ResourceInspector<PhysicsConfig>,
    //tlas: ResourceInspector<Tlas>,
}

pub fn setup_camera(mut commands: Commands) {
    // light
    commands
        .spawn_bundle(DirectionalLightBundle {
            transform: Transform::from_xyz(50.0, 50.0, 50.0).looking_at(Vec3::ZERO, Vec3::Y),
            directional_light: DirectionalLight {
                shadows_enabled: true,
                ..default()
            },
            ..default()
        })
        .insert(Keep);

    commands
        .spawn_bundle(Camera3dBundle {
            transform: Transform::from_xyz(0.0, 2.0, 30.0).looking_at(Vec3::ZERO, Vec3::Y),
            ..default()
        })
        .insert(BvhCamera::new(256, 256))
        // Add our controller
        .insert(CameraController::default())
        .insert(Keep);
}

pub fn setup_room(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut colliders: ResMut<Assets<Collider>>,
) {
    let floor_size = 100.0;
    let wall_height = 10.0;
    let floor_half = floor_size * 0.5;
    let wall_height_half = wall_height * 0.5;

    // floor
    commands
        .spawn_bundle(PbrBundle {
            transform: Transform::from_xyz(0.0, -0.5, 0.0),
            mesh: meshes.add(Mesh::from(shape::Box::new(floor_size, 1.0, floor_size))),
            material: materials.add(StandardMaterial {
                base_color: Color::DARK_GREEN,
                ..default()
            }),
            ..default()
        })
        .insert_bundle(RigidBodyBundle {
            collider: colliders.add(Collider::from(Box::new(vec3(floor_size, 1.0, floor_size)))),
            mode: RigidBody::Static,
            ..default()
        })
        .insert(Name::new("Floor"));

    // walls
    for wall in 0..4 {
        let mut transform = Transform::from_xyz(0.0, wall_height_half, floor_half);
        transform.rotate_around(
            Vec3::ZERO,
            Quat::from_axis_angle(Vec3::Y, wall as f32 * FRAC_PI_2),
        );

        commands
            .spawn_bundle(PbrBundle {
                transform,
                mesh: meshes.add(Mesh::from(shape::Box::new(floor_size, wall_height, 1.0))),
                material: materials.add(StandardMaterial {
                    base_color: Color::ALICE_BLUE,
                    ..default()
                }),
                ..default()
            })
            .insert_bundle(RigidBodyBundle {
                collider: colliders.add(Collider::from(Box::new(vec3(
                    floor_size,
                    wall_height,
                    1.0,
                )))),
                mode: RigidBody::Static,
                ..default()
            })
            .insert(Name::new("Wall"));
    }
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

pub fn reset(mut commands: Commands, query: Query<Entity, (Without<Keep>, Without<Parent>)>) {
    for e in query.iter() {
        commands.entity(e).despawn();
    }
    commands.insert_resource(NextState(AppState::Playing));
}

pub fn reset_listen(mut commands: Commands, input: Res<Input<KeyCode>>) {
    if input.just_pressed(KeyCode::R) {
        commands.insert_resource(NextState(AppState::Reset));
    }
}
