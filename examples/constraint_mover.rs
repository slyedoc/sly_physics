use bevy::prelude::*;

use crate::{types::{
    Velocity,
}, PhysicsConfig};


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

