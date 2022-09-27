use bevy::prelude::*;

use crate::types::{Velocity, Drag};

#[derive(Debug, Clone, PartialEq, Eq, Hash, Copy)]
pub enum DragState {
    Running,
    Paused,
}

#[derive(Component, Reflect, Debug)]
#[reflect(Component)]
pub struct Drag {
    pub linear_velocity: f32,
}

impl Default for Drag {
    fn default() -> Drag {
        Drag {
            linear_velocity: 0.01,
        }
    }
}

pub struct DragPlugin;

impl Plugin for DragPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<Gravity>()
            .add_loopless_state(Drag::Running)
            .add_system_set_to_stage(
                PhysicsFixedUpdate,
                ConditionSet::new()
                    .run_in_state(PhysicsState::Running)
                    .run_in_state(GravityState::Running)
                    .label(PhysicsSystems::Dynamics)
                    .after(PhysicsSystems::Update)
                    .before(PhysicsSystems::Broad)
                    .with_system(drag)
                    .into(),
            )
            .register_type::<Drag>();
    }
}


pub fn drag(
    mut _query: Query<(        
        &mut Velocity,
        &Drag,
    )>
) {
    // TODO
    //  for (mut vel, drag) in _query.iter_mut() {
    //      if !vel.linear.eq(&Vec3::ZERO) {

    //          let drag_force = drag.linear_velocity * vel.linear;
    //          vel.linear -= drag_force;

    //        // TODO: this is a hack to stop things floating around on ground
    //          if vel.linear.length_squared() < 0.001 {
    //              vel.linear = Vec3::ZERO;
    //          }
    //      }
    //  }
}