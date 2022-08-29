use bevy::prelude::*;

use crate::types::{LinearVelocity, AngularVelocity, Drag};

#[allow(dead_code)]
pub fn drag_system(
    mut query: Query<(        
        &mut LinearVelocity,
        &mut AngularVelocity,
        &Drag,
    )>
) {
    // for (mut lin_vel, mut _ang_vel, drag) in query.iter_mut() {
    //     if !lin_vel.0.eq(&Vec3::ZERO) {

    //         let drag_force = drag.linear_velocity * lin_vel.0;
    //         lin_vel.0 -= drag_force;

    //         // TODO: this is a hack to stop things floating around on ground
    //         if lin_vel.0.length_squared() < 0.001 {
    //             lin_vel.0 = Vec3::ZERO;
    //         }
    //     }
    // }
}