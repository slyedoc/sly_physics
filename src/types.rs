use bevy::prelude::*;
use bevy_inspector_egui::Inspectable;

#[derive(Component, Inspectable, Debug)]
pub struct RigidBody;

impl RigidBody {
    pub fn world_to_local(
        world_point: Vec3,
        trans: &Transform,
        center_of_mass_world: &CenterOfMassWorld,
    ) -> Vec3 {
        let tmp = world_point - center_of_mass_world.0;
        let inv_orientation = trans.rotation.conjugate();
        inv_orientation * tmp
    }

    pub fn local_to_world(
        local_point: Vec3,
        trans: &Transform,
        center_of_mass_world: &CenterOfMassWorld,
    ) -> Vec3 {
        center_of_mass_world.0 + trans.rotation * local_point
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
        RigidBody::apply_impulse_linear(linear_velocity, inv_mass, impulse);

        let r = impulse_point - com_world.0;
        let dl = r.cross(impulse); // this is in world space
        RigidBody::apply_impulse_angular(
            angular_velocity,
            inv_mass,
            inv_inertia_tensor_world,
            dl,
        );
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

        // clamp angular_velocity - 30 rad/s is fast enough for us
        const MAX_ANGULAR_SPEED: f32 = 30.0;
        const MAX_ANGULAR_SPEED_SQ: f32 = MAX_ANGULAR_SPEED * MAX_ANGULAR_SPEED;
        if angular_velocity.0.length_squared() > MAX_ANGULAR_SPEED_SQ {
            angular_velocity.0 = angular_velocity.0.normalize() * MAX_ANGULAR_SPEED;
        }
    }
}

#[derive(Component, Inspectable, Debug)]
pub struct Static;

#[derive(Component, Deref, DerefMut, Inspectable, Debug, Default)]
pub struct LinearVelocity(pub Vec3);

#[derive(Component, Deref, DerefMut, Inspectable, Debug, Default)]
pub struct AngularVelocity(pub Vec3);

#[derive(Component, Deref, DerefMut, Inspectable, Debug, Default)]
pub struct Elasticity(pub f32); // assumed [0,1]

#[derive(Component, Deref, DerefMut, Inspectable, Debug, Default)]
pub struct Friction(pub f32); // assumed [0,1]

#[derive(Component, Deref, Inspectable, Debug)]
pub struct Mass(pub f32);

#[derive(Component, Deref, DerefMut, Inspectable, Debug, Default)]
pub struct InverseMass(pub f32);

#[derive(Component, Inspectable, Debug, Default)]
pub struct CenterOfMass(pub Vec3);

#[derive(Component, Deref, DerefMut, Inspectable, Debug, Default)]
pub struct CenterOfMassWorld(pub Vec3);

#[derive(Component, Deref, DerefMut, Inspectable, Debug, Default)]
pub struct InertiaTensor(pub Mat3);

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
