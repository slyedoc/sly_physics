mod aabb;
use crate::{MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED_SQ};
pub use aabb::*;

use bevy::prelude::*;
#[derive(Debug)]
pub struct BroadContact {
    pub a: Entity,
    pub b: Entity,
}

#[derive(Copy, Clone, Debug)]
pub struct Contact {
    pub a: Entity,
    pub b: Entity,
    pub world_point_a: Vec3,
    pub world_point_b: Vec3,
    pub local_point_a: Vec3,
    pub local_point_b: Vec3,
    pub normal: Vec3,
    pub separation_dist: f32,
    pub time_of_impact: f32,
}

pub struct ManifoldContact(pub Contact);

pub struct RBHelper;

impl RBHelper {
    #[inline]
    pub fn world_to_local(trans: &Transform, com: &CenterOfMass, world_point: Vec3) -> Vec3 {
        let com_world = trans.translation + trans.rotation * com.0;
        let inv_orientation = trans.rotation.conjugate();
        inv_orientation * (world_point - com_world)
    }

    #[inline]
    pub fn local_to_world(trans: &Transform, com: &CenterOfMass, local_point: Vec3) -> Vec3 {
        trans.mul_vec3(com.0 + local_point)
    }

    #[inline]
    pub fn inv_inertia_tensor_world(
        trans: &Transform,
        inv_inertia_tensor: &InverseInertiaTensor,
    ) -> Mat3 {
        let orientation = Mat3::from_quat(trans.rotation);
        orientation * inv_inertia_tensor.0 * orientation.transpose()
    }
    #[inline]
    pub fn centre_of_mass_world(t: &Transform, com: &CenterOfMass) -> Vec3 {
        t.translation + t.rotation * com.0
    }

    #[inline]
    #[allow(clippy::too_many_arguments)]
    pub fn apply_impulse(
        trans: &Transform,
        velocity: &mut Velocity,
        inv_mass: &InverseMass,
        com: &CenterOfMass,
        inv_inertia_tensor: &InverseInertiaTensor,
        impulse_point: Vec3,
        impulse: Vec3,
    ) {
        if inv_mass.0 == 0.0 {
            return;
        }
        // impulse_point is in world space location of the applied impulse
        // impulse is in world space direction and magnitude of the impulse
        RBHelper::apply_impulse_linear(velocity, inv_mass, impulse);

        let r = impulse_point - RBHelper::centre_of_mass_world(trans, com);
        let dl = r.cross(impulse); // this is in world space
        RBHelper::apply_impulse_angular(trans, velocity, inv_mass, inv_inertia_tensor, dl);
    }

    #[inline]
    pub fn apply_impulse_linear(velocity: &mut Velocity, inv_mass: &InverseMass, impulse: Vec3) {
        if inv_mass.0 == 0.0 {
            return;
        }
        // p = mv
        // dp = m dv = J
        // => dv = J / m
        velocity.linear += impulse * inv_mass.0;
    }
    pub fn apply_impulse_angular(
        trans: &Transform,
        velocity: &mut Velocity,
        inv_mass: &InverseMass,
        inv_inertia_tensor: &InverseInertiaTensor,
        impulse: Vec3,
    ) {
        if inv_mass.0 == 0.0 {
            return;
        }

        // L = I w = r x p
        // dL = I dw = r x J
        // => dw = I^-1 * (r x J)
        velocity.angular += RBHelper::inv_inertia_tensor_world(trans, inv_inertia_tensor) * impulse;

        // clamp angular_velocity
        if velocity.angular.length_squared() > MAX_ANGULAR_SPEED_SQ {
            velocity.angular = velocity.angular.normalize() * MAX_ANGULAR_SPEED;
        }
    }

    #[inline]
    pub fn update(
        transform: &mut Transform,
        velocity: &mut Velocity,
        com: &CenterOfMass,
        inertia_tensor: &InertiaTensor,
        dt: f32,
    ) {
        transform.translation += velocity.linear * dt;
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
        let alpha =
            inertia_tensor.inverse() * (velocity.angular.cross(inertia_tensor * velocity.angular));
        velocity.angular += alpha * dt;

        // update orientation
        let d_angle = velocity.angular * dt;
        let angle = d_angle.length();
        let rcp_angle = angle.recip();
        let dq = if rcp_angle.is_finite() {
            Quat::from_axis_angle(d_angle * rcp_angle, angle)
        } else {
            Quat::IDENTITY
        };
        transform.rotation = (dq * transform.rotation).normalize();
        // now get the new body position
        transform.translation = com_world + dq * com_to_position;
    }
}

#[derive(Component, Reflect, Debug, PartialEq, Eq, Clone, Copy)]
#[reflect(Component)]
pub enum RigidBody {
    Static,
    Dynamic,
}

impl Default for RigidBody {
    fn default() -> Self {
        RigidBody::Dynamic
    }
}

#[derive(Component, Debug)]
pub struct Static;

#[derive(Component, Reflect, Debug, Default, Clone, Copy)]
#[reflect(Component)]
pub struct Velocity {
    pub linear: Vec3,
    pub angular: Vec3,
}

/// assumed 0.0..=1.0
#[derive(Component, Deref, DerefMut, Reflect, Debug)]
#[reflect(Component)]
pub struct Elasticity(pub f32);

impl Default for Elasticity {
    fn default() -> Elasticity {
        Elasticity(0.9)
    }
}

/// assumed 0.0..=1.0
#[derive(Component, Deref, DerefMut, Reflect, Debug)]
#[reflect(Component)]
pub struct Friction(pub f32);

impl Default for Friction {
    fn default() -> Friction {
        Friction(0.5)
    }
}

#[derive(Component, Deref, DerefMut, Reflect, Debug)]
#[reflect(Component)]
pub struct Mass(pub f32);

impl Default for Mass {
    fn default() -> Mass {
        Mass(1.0)
    }
}

#[derive(Component, Deref, DerefMut, Reflect, Debug, Default)]
#[reflect(Component)]
pub struct InverseMass(pub f32);

#[derive(Component, Deref, DerefMut, Reflect, Debug, Default)]
#[reflect(Component)]
pub struct CenterOfMass(pub Vec3);

#[derive(Component, Deref, DerefMut, Reflect, Debug, Default)]
#[reflect(Component)]
pub struct InertiaTensor(pub Mat3);

#[derive(Component, Reflect, Debug, Default)]
#[reflect(Component)]
pub struct InverseInertiaTensor(pub Mat3);
