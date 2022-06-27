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

    pub fn update(
        transform: &mut Transform,
        linear_velocity: &mut LinearVelocity,
        angular_velocity: &mut AngularVelocity,
        center_of_mass_world: &CenterOfMassWorld,
        inertia_tensor: &InertiaTensor,        
        dt: f32,
    ) {
        // apply linear velocity
        transform.translation += linear_velocity.0 * dt;

        // we have an angular velocity around the centre of mass, this needs to be converted to
        // relative body translation. This way we can properly update the rotation of the model

        let com_to_position = transform.translation - center_of_mass_world.0;

        // total torque is equal to external applied torques + internal torque (precession)
        // T = T_external + omega x I * omega
        // T_external = 0 because it was applied in the collision response function
        // T = Ia = w x I * w
        // a = I^-1 (w x I * w)
        let orientation = Mat3::from_quat(transform.rotation);
        let inertia_tensor = orientation * inertia_tensor.0 * orientation.transpose();
        let alpha = inertia_tensor.inverse()
            * (angular_velocity.0
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
        transform.translation = center_of_mass_world.0 + dq * com_to_position;

        // // update center of mass and inverse inertia tensor
        // center_of_mass_world =
        //     transform.translation + transform.rotation * self.center_of_mass;
        // let orientation = Mat3::from_quat(transform.rotation);
        // self.inverse_inertia_tensor_world =
        //     orientation * self.inverse_inertia_tensor_local * orientation.transpose();
    }
}

#[derive(Component, Inspectable, Debug)]
pub struct Static;

#[derive(Component, Inspectable, Debug, Default)]
pub struct LinearVelocity(pub Vec3);

#[derive(Component, Inspectable, Debug, Default)]
pub struct AngularVelocity(pub Vec3);

#[derive(Component, Inspectable, Debug, Default)]
pub struct Elasticity(pub f32); // assumed [0,1]

#[derive(Component, Inspectable, Debug, Default)]
pub struct Friction(pub f32); // assumed [0,1]

#[derive(Component, Inspectable, Debug, Default)]
pub struct Mass(pub f32);

#[derive(Component, Inspectable, Debug, Default)]
pub struct InvMass(pub f32);

#[derive(Component, Inspectable, Debug, Default)]
pub struct CenterOfMass(pub Vec3);

#[derive(Component, Inspectable, Debug, Default)]
pub struct CenterOfMassWorld(pub Vec3);

#[derive(Component, Inspectable, Debug, Default)]
pub struct InertiaTensor(pub Mat3);