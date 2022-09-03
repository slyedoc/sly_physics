use bevy::{prelude::*, window::PresentMode};
use bevy_inspector_egui::WorldInspectorPlugin;
use helper::{HelperPlugin, AppState};
use iyes_loopless::prelude::*;
use sly_camera_controller::CameraControllerPlugin;
use sly_physics::prelude::*;

// TODO: make this work

#[derive(Component)]
pub struct ConstraintMoverSimple {
    time: f32,
}

impl Default for ConstraintMoverSimple {
    fn default() -> Self {
        Self { time:  0.0 }
    }
}

fn pre_solve(
    mut query: Query<(&mut ConstraintMoverSimple, &mut Velocity)>,        
    config: Res<PhysicsConfig>,
) { 
    for (mut constrain_move, mut vel) in query.iter_mut() {
        constrain_move.time += config.time;
        vel.linear.z = f32::cos(constrain_move.time * 0.25) * 4.0;
    }
}

mod helper;

fn main() {
    App::new()
        .insert_resource(WindowDescriptor {
            present_mode: PresentMode::Fifo,
            ..default()
        })
        .add_plugins(DefaultPlugins)

        .add_plugin(WorldInspectorPlugin::default())
        // our phsycis plugin
        .add_plugin(PhysicsPlugin)
        .add_plugin(GravityPlugin)
        .add_plugin(PhysicsDebugPlugin)
        .add_plugin(PhysicsBvhCameraPlugin)
        
        .add_plugin(HelperPlugin)
        .add_plugin(CameraControllerPlugin)
        .add_startup_system(helper::setup_camera)
        .add_enter_system(AppState::Playing, helper::setup_room)
        //.add_enter_system(AppState::Playing, setup)
        .run();
}