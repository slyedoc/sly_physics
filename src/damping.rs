use bevy::prelude::*;

use crate::types::{LinearVelocity, AngularVelocity, Damping};

#[allow(dead_code)]
pub fn damping_system(
    mut query: Query<(        
        &mut LinearVelocity,
        &mut AngularVelocity,
        &Damping,
    )>
) {
    for (mut lin_vel, mut ang_vel, damping) in query.iter_mut() {
        if lin_vel.length() < damping.linear {
            lin_vel.0 = Vec3::ZERO;
        }
        if ang_vel.length() < damping.angular {
            ang_vel.0 = Vec3::ZERO;
        }
    }
}