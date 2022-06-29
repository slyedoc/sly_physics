use bevy::prelude::*;

use crate::{narrow::Contact, Elasticity, InverseInertiaTensorWorld, CenterOfMassWorld, LinearVelocity, AngularVelocity, InverseMass, Friction, RigidBody};

pub fn resolve_system(
    mut contacts: EventReader<Contact>,
    query: Query<(&mut GlobalTransform, &mut LinearVelocity, &mut AngularVelocity, &InverseMass, &Elasticity, &Friction, &CenterOfMassWorld, &InverseInertiaTensorWorld )>,
) {
    for contact in contacts.iter() {
        unsafe {

            let (mut trans_a, mut linear_vel_a, mut ang_vel_a, inv_mass_a, elas_a, friction_a, com_w_a, inv_inertia_world_a) = query.get_unchecked(contact.a).unwrap();
            let (mut trans_b, mut linear_vel_b, mut ang_vel_b, inv_mass_b, elas_b, friction_b, com_w_b, inv_inertia_world_b) = query.get_unchecked(contact.b).unwrap();

            let elasticity = elas_a.0 * elas_b.0;
            let total_inv_mass = inv_mass_a.0 + inv_mass_b.0;
        
            let ra = contact.world_point_a - com_w_a.0;
            let rb = contact.world_point_b - com_w_b.0;
        
            let angular_j_a = (inv_inertia_world_a.0 * ra.cross(contact.normal)).cross(ra);
            let angular_j_b = (inv_inertia_world_b.0 * rb.cross(contact.normal)).cross(rb);
            let angular_factor = (angular_j_a + angular_j_b).dot(contact.normal);
        
            // Get the world space velocity of the motion and rotation
            let vel_a = linear_vel_a.0 + ang_vel_a.0.cross(ra);
            let vel_b = linear_vel_b.0 + ang_vel_b.0.cross(rb);
        
            // Calculate the collion impulse
            let vab = vel_a - vel_b;
            let impluse_j = -(1.0 + elasticity) * vab.dot(contact.normal) / (total_inv_mass + angular_factor);
            let impluse_vec_j = contact.normal * impluse_j;

            RigidBody::apply_impulse(&mut linear_vel_a, &mut  ang_vel_a, inv_mass_a, com_w_a, inv_inertia_world_a, contact.world_point_a, impluse_vec_j);
            RigidBody::apply_impulse(&mut linear_vel_b, &mut  ang_vel_b, inv_mass_b, com_w_b, inv_inertia_world_b, contact.world_point_b, -impluse_vec_j);            

            // Calculate the friction impulse
            let friction = friction_a.0 * friction_b.0;

            // Find the normal direction of the velocity with respoect to the normal of the collison
            let velocity_normal = contact.normal * contact.normal.dot(vab);
            let velocity_tangent = vab - velocity_normal;
        
            // Get the tangent velocities relative to the other body
            let relative_velocity_tangent = velocity_tangent.normalize();
        
            let inertia_a = (inv_inertia_world_a.0 * ra.cross(relative_velocity_tangent)).cross(ra);
            let inertia_b = (inv_inertia_world_b.0 * rb.cross(relative_velocity_tangent)).cross(rb);
            let inv_inertia = (inertia_a + inertia_b).dot(relative_velocity_tangent);
        
            // calculat the tangential impluse for friction
            let reduced_mass = 1.0 / (total_inv_mass + inv_inertia);
            let impluse_friction = velocity_tangent * (reduced_mass * friction);
        
            // TODO: Book didnt have this if check, but I was getitng velocity_tangent of zero leading to
            // a Vec3 Nan when normalized if perfectly lined up on ground
            if !impluse_friction.is_nan() {
                 // apply kinetic friction
    
                 RigidBody::apply_impulse(&mut linear_vel_a, &mut  ang_vel_a, inv_mass_a, com_w_a, inv_inertia_world_a, contact.world_point_a, -impluse_friction);
                 RigidBody::apply_impulse(&mut linear_vel_b, &mut  ang_vel_b, inv_mass_b, com_w_b, inv_inertia_world_b, contact.world_point_b, impluse_friction);            
     
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
    }
}

