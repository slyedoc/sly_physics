use bevy::prelude::*;
use bevy_inspector_egui::Inspectable;

use crate::{MAX_ANGULAR_SPEED_SQ, MAX_ANGULAR_SPEED};

#[derive(Component, Inspectable, Debug)]
pub struct RBHelper;

impl RBHelper {
    pub fn world_to_local(
        trans: &Transform,
        com: &CenterOfMass,
        world_point: Vec3,

    ) -> Vec3 {
        let com_world = trans.translation + trans.rotation * com.0;
        let inv_orientation = trans.rotation.conjugate();
        inv_orientation * (world_point - com_world)
    }

    pub fn local_to_world(
        trans: &Transform,
        com: &CenterOfMass,
        local_point: Vec3,
    ) -> Vec3 {
        let com_world = trans.translation + trans.rotation * com.0;
        com_world + trans.rotation * local_point
    }

    pub fn centre_of_mass_world(t: &Transform, com: Vec3) -> Vec3 {
        t.translation + t.rotation * com
    }

    pub fn apply_impulse(
        linear_velocity: &mut LinearVelocity,
        angular_velocity: &mut AngularVelocity,
        inv_mass: &InverseMass,
        com_world: &CenterOfMassWorld,
        inv_inertia_tensor_world: &InverseInertiaTensorWorld,
        impulse_point: Vec3,
        impulse: Vec3,
    ) {
        if inv_mass.0 == 0.0 {
            return;
        }
        // impulse_point is in world space location of the applied impulse
        // impulse is in world space direction and magnitude of the impulse
        RBHelper::apply_impulse_linear(linear_velocity, inv_mass, impulse);

        let r = impulse_point - com_world.0;
        let dl = r.cross(impulse); // this is in world space
        RBHelper::apply_impulse_angular(angular_velocity, inv_mass, inv_inertia_tensor_world, dl);
    }

    pub fn apply_impulse_linear(
        linear_velocity: &mut LinearVelocity,
        inv_mass: &InverseMass,
        impulse: Vec3,
    ) {
        if inv_mass.0 == 0.0 {
            return;
        }
        // p = mv
        // dp = m dv = J
        // => dv = J / m
        linear_velocity.0 += impulse * inv_mass.0;
    }
    pub fn apply_impulse_angular(
        angular_velocity: &mut AngularVelocity,
        inv_mass: &InverseMass,
        inv_inertia_tensor_world: &InverseInertiaTensorWorld,
        impulse: Vec3,
    ) {
        if inv_mass.0 == 0.0 {
            return;
        }

        // L = I w = r x p
        // dL = I dw = r x J
        // => dw = I^-1 * (r x J)
        angular_velocity.0 += inv_inertia_tensor_world.0 * impulse;

        // clamp angular_velocity 
        if angular_velocity.0.length_squared() > MAX_ANGULAR_SPEED_SQ {
            angular_velocity.0 = angular_velocity.0.normalize() * MAX_ANGULAR_SPEED;
        }
    }

    pub fn update(
        transform: &mut Transform,
        angular_velocity: &mut AngularVelocity,
        linear_velocity: &LinearVelocity,
        com: &CenterOfMass,
        inertia_tensor: &InertiaTensor,
        dt: f32,
    ) {
        transform.translation += linear_velocity.0 * dt;
        // we have an angular velocity around the centre of mass, this needs to be converted to
        // relative body translation. This way we can properly update the rotation of the model
        let com_world = transform.translation + transform.rotation * com.0;
        let com_to_position = transform.translation - com_world;
        // total torque is equal to external applied torques + internal torque (precession)
        // T = T_external + omega x I * omega
        // T_external = 0 because it was applied in the collision response function
        // T = Ia = w x I * w
        // a = I^-1 (w x I * w)
        let orientation = Mat3::from_quat(transform.rotation);
        let inertia_tensor = orientation * inertia_tensor.0 * orientation.transpose();
        let alpha = inertia_tensor.inverse()
            * (angular_velocity
                .0
                .cross(inertia_tensor * angular_velocity.0));
        angular_velocity.0 += alpha * dt;
        
        // update orientation
        let d_angle = angular_velocity.0 * dt;
        let angle = d_angle.length();
        let inv_angle = angle.recip();
        let dq = if inv_angle.is_finite() {
            Quat::from_axis_angle(d_angle * inv_angle, angle)
        } else {
            Quat::IDENTITY
        };
        transform.rotation = (dq * transform.rotation).normalize();
        // now get the new body position
        transform.translation = com_world + dq * com_to_position;
    }
}

#[derive(Component, Inspectable, Debug, PartialEq, Eq)]
pub enum RigidBodyMode {
    Static,
    Dynamic,
}

impl Default for RigidBodyMode {
    fn default() -> Self {
        RigidBodyMode::Dynamic
    }
}

#[derive(Component, Inspectable, Debug)]
pub struct Static;

#[derive(Component, Deref, DerefMut, Inspectable, Debug, Default)]
pub struct LinearVelocity(pub Vec3);

#[derive(Component, Deref, DerefMut, Inspectable, Debug, Default)]
pub struct AngularVelocity(pub Vec3);

#[derive(Component, Deref, DerefMut, Inspectable, Debug)]
pub struct Elasticity(pub f32); // assumed [0,1]

impl Default for Elasticity {
    fn default() -> Elasticity {
        Elasticity(1.0)
    }
}

#[derive(Component, Deref, DerefMut, Inspectable, Debug)]
pub struct Friction(pub f32); // assumed [0,1]

impl Default for Friction {
    fn default() -> Friction {
        Friction(0.5)
    }
}

#[derive(Component, Deref, DerefMut, Inspectable, Debug)]
pub struct Mass(pub f32);

impl Default for Mass {
    fn default() -> Mass {
        Mass(1.0)
    }
}

#[derive(Component, Deref, DerefMut, Inspectable, Debug, Default)]
pub struct InverseMass(pub f32);

#[derive(Component, Inspectable, Debug, Default)]
pub struct CenterOfMass(pub Vec3);

#[derive(Component, Deref, DerefMut, Inspectable, Debug, Default)]
pub struct CenterOfMassWorld(pub Vec3);

#[derive(Component, Deref, DerefMut, Inspectable, Debug, Default)]
pub struct InertiaTensor(pub Mat3);

#[derive(Component, Deref, DerefMut, Inspectable, Debug, Default)]
pub struct InertiaTensorWorld(pub Mat3);

#[derive(Component, Inspectable, Debug, Default)]
pub struct InverseInertiaTensor(pub Mat3);

#[derive(Component, Inspectable, Debug, Default)]
pub struct InverseInertiaTensorWorld(pub Mat3);

#[derive(Debug, Component, Inspectable)]
pub struct Aabb {
    pub minimums: Vec3,
    pub maximums: Vec3,
}

impl Default for Aabb {
    fn default() -> Self {
        Self {
            minimums: Vec3::splat(std::f32::MAX),
            maximums: Vec3::splat(std::f32::MIN),
        }
    }
}

#[derive(Debug, Component, Inspectable)]
pub struct AabbWorld {
    pub minimums: Vec3,
    pub maximums: Vec3,
}

impl Default for AabbWorld {
    fn default() -> Self {
        Self {
            minimums: Vec3::splat(std::f32::MAX),
            maximums: Vec3::splat(std::f32::MIN),
        }
    }
}
