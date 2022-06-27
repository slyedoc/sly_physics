mod dynamics;

pub use dynamics::*;

use bevy::prelude::*;

use super::{
    AngularVelocity, CenterOfMassWorld, InertiaTensor, LinearVelocity, PhysicsTime, RigidBody,
    Static, Mass, InvMass, Gravity,
};

pub fn dynamics_system(mut query: Query<(&mut LinearVelocity, &Mass, &InvMass), Without<Static>>, gravity: Res<Gravity>, pt: Res<PhysicsTime>) {
    for (mut linear_velocity, mass, inv_mass) in query.iter_mut(){

        // Apply Gravity, it needs to be an impluse        
        let gravey_impluse = gravity.0 * mass.0 * pt.time;

        // since rb is not static, inv mass shouldnt be 0 or less
        assert!(inv_mass.0 > 0.0);

        linear_velocity.0 += gravey_impluse * inv_mass.0;
    }
}

pub fn update_system(
    mut query: Query<
        (
            &mut Transform,
            &mut LinearVelocity,
            &mut AngularVelocity,
            &CenterOfMassWorld,
            &InertiaTensor,
        ),
        Without<Static>,
    >,
    pt: Res<PhysicsTime>,
) {
    for (mut t, mut lin_vel, mut ang_vel, com_w, inertia_tensor) in query.iter_mut() {
        RigidBody::update(
            &mut t,
            &mut lin_vel,
            &mut ang_vel,
            com_w,
            inertia_tensor,
            pt.time,
        );
    }
}
