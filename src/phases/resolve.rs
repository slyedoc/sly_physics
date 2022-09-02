use bevy::prelude::*;

use crate::*;

#[allow(clippy::type_complexity)]
pub fn resolve_phase(
    mut contacts_events: EventReader<Contact>,
    mut query: Query<(
        &mut Transform,
        &mut Velocity,
        &InverseMass,
        &Elasticity,
        &Friction,
        &CenterOfMass,
        &InertiaTensor,
        &InverseInertiaTensor,
    )>,
    config: Res<PhysicsConfig>,
) {
    let mut contacts = contacts_events.iter().collect::<Vec<_>>();

    contacts.sort_by(|a, b| a.time_of_impact.partial_cmp(&b.time_of_impact).unwrap());

    // Apply Ballistics
    let mut accumulated_time = 0.0;
    for contact in contacts.iter() {
        //info!("{:?}", contact);
        let contact_time = contact.time_of_impact - accumulated_time;

        step_rigibbodies(&mut query, contact_time);
        resolve_contact(&mut query, contact);

        accumulated_time += contact_time;
    }

    // update positions for the rest of this frame's time
    let time_remaining = config.time - accumulated_time;
    if time_remaining > 0.0 {
        step_rigibbodies(&mut query, time_remaining);
    }
}

#[allow(clippy::type_complexity)]
fn step_rigibbodies(
    query: &mut Query<(
        &mut Transform,
        &mut Velocity,
        &InverseMass,
        &Elasticity,
        &Friction,
        &CenterOfMass,
        &InertiaTensor,
        &InverseInertiaTensor,
    )>,
    time: f32,
) {
    for (
        mut transform,
        mut vel,
        _inv_mass,
        _elas,
        _friction,
        com,
        inertia_tensor,
        _inv_inertia_tensor,
    ) in query.iter_mut()
    {
        RBHelper::update(
            &mut transform,
            &mut vel,
            com,
            inertia_tensor,
            time,
        );
    }
}

#[allow(clippy::type_complexity)]
fn resolve_contact(
    query: &mut Query<(
        &mut Transform,
        &mut Velocity,
        &InverseMass,
        &Elasticity,
        &Friction,
        &CenterOfMass,
        &InertiaTensor,
        &InverseInertiaTensor,
    )>,
    contact: &Contact,
) {
    let [(
        mut trans_a,
        mut vel_a,
        inv_mass_a,
        elas_a,
        friction_a,
        com_a,
        _inertia_tensor_a,
        inv_inertia_a,
    ), (
        mut trans_b,
        mut vel_b,
        inv_mass_b,
        elas_b,
        friction_b,
        com_b,
        _inertia_tensor_b,
        inv_inertia_b,
    )] = query.many_mut([contact.a, contact.b]);

    let elasticity = elas_a.0 * elas_b.0;

    let orientation_a = Mat3::from_quat(trans_a.rotation);
    let orientation_b = Mat3::from_quat(trans_b.rotation);

    let inv_inertia_world_a = orientation_a * inv_inertia_a.0 * orientation_a.transpose();
    let inv_inertia_world_b = orientation_b * inv_inertia_b.0 * orientation_b.transpose();

    let total_inv_mass = inv_mass_a.0 + inv_mass_b.0;

    let com_world_a = trans_a.translation + trans_a.rotation * com_a.0;
    let com_world_b = trans_b.translation + trans_b.rotation * com_b.0;
    let ra = contact.world_point_a - com_world_a;
    let rb = contact.world_point_b - com_world_b;

    let angular_j_a = (inv_inertia_world_a * ra.cross(contact.normal)).cross(ra);
    let angular_j_b = (inv_inertia_world_b * rb.cross(contact.normal)).cross(rb);
    let angular_factor = (angular_j_a + angular_j_b).dot(contact.normal);

    // Get the world space velocity of the motion and rotation
    let world_vel_a = vel_a.linear + vel_a.angular.cross(ra);
    let world_vel_b = vel_b.linear + vel_b.angular.cross(rb);

    // Calculate the collion impulse
    let vab = world_vel_a - world_vel_b;
    let impluse_j =
        -(1.0 + elasticity) * vab.dot(contact.normal) / (total_inv_mass + angular_factor);
    let impluse_vec_j = contact.normal * impluse_j;
    RBHelper::apply_impulse(
        &trans_a,
        &mut vel_a,
        inv_mass_a,
        com_a,
        inv_inertia_a,
        contact.world_point_a,
        impluse_vec_j,
    );
    RBHelper::apply_impulse(
        &trans_b,
        &mut vel_b,
        inv_mass_b,
        com_a,
        inv_inertia_b,
        contact.world_point_b,
        -impluse_vec_j,
    );

    // Calculate the friction impulse
    let friction = friction_a.0 * friction_b.0;

    // Find the normal direction of the velocity with respoect to the normal of the collison
    let velocity_normal = contact.normal * contact.normal.dot(vab);

    // find the tangent direction of the velocity with respect to the normal of the collision
    let velocity_tangent = vab - velocity_normal;

    // Get the tangent velocities relative to the other body
    let relative_velocity_tangent = velocity_tangent.normalize_or_zero();

    let inertia_a = (inv_inertia_world_a * ra.cross(relative_velocity_tangent)).cross(ra);
    let inertia_b = (inv_inertia_world_b * rb.cross(relative_velocity_tangent)).cross(rb);
    let inv_inertia = (inertia_a + inertia_b).dot(relative_velocity_tangent);

    // calculat the tangential impluse for friction
    let reduced_mass = 1.0 / (total_inv_mass + inv_inertia);
    let impluse_friction = velocity_tangent * (reduced_mass * friction);

    // TODO: Book didnt have this if check, but I was getitng velocity_tangent of zero leading to
    // a Vec3 Nan when normalized if perfectly lined up on ground
    if !impluse_friction.is_nan() {
        // apply kinetic friction
        RBHelper::apply_impulse(
            &trans_a,
            &mut vel_a,
            inv_mass_a,
            com_a,
            inv_inertia_a,
            contact.world_point_a,
            -impluse_friction,
        );
        RBHelper::apply_impulse(
            &trans_b,
            &mut vel_b,
            inv_mass_b,
            com_b,
            inv_inertia_b,
            contact.world_point_b,
            impluse_friction,
        );
    }
    // Lets also move our colliding object to just outside of each other
    if contact.time_of_impact == 0.0 {
        let a_move_weight = inv_mass_a.0 / total_inv_mass;
        let b_move_weight = inv_mass_b.0 / total_inv_mass;

        let distance = contact.world_point_b - contact.world_point_a;

        trans_a.translation += distance * a_move_weight;
        trans_b.translation -= distance * b_move_weight;
    }
}
