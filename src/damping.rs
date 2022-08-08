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
    for (mut _lin_vel, mut _ang_vel, _damping) in query.iter_mut() {
        // if lin_vel.length() < damping.linear {
        //     lin_vel.0 = Vec3::ZERO;
        // }
        // if ang_vel.length() < damping.angular {
        //     ang_vel.0 = Vec3::ZERO;
        // }
    }
}