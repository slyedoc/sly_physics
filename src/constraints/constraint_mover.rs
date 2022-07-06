use bevy::prelude::{Entity, Query, Transform};

use crate::{
    AngularVelocity, CenterOfMass, Elasticity, Friction, InertiaTensor, InverseInertiaTensor,
    InverseMass, LinearVelocity,
};

pub struct ConstraintMoverSimple {
    time: f32,
}

impl ConstraintMoverSimple {
    pub fn new() -> Self {
        ConstraintMoverSimple { time: 0.0 }
    }
}

impl ConstraintMoverSimple {
    fn pre_solve(
        &mut self,
        rb_query: &mut Query<&mut LinearVelocity>,
        a: Entity,
        b: Entity,
        dt_sec: f32,
    ) {
        self.time += dt_sec;

        let mut lin_vel = rb_query.get_mut(a).expect("Mover Entity not found");

        lin_vel.0.z = f32::cos(self.time * 0.25) * 4.0;
    }
}
