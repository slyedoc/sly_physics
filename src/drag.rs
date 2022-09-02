use bevy::prelude::*;

use crate::types::{Velocity, Drag};

#[allow(dead_code)]
pub fn drag_system(
    mut _query: Query<(        
        &mut Velocity,
        &Drag,
    )>
) {
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