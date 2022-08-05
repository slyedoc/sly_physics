use crate::{
    math::{lcp_gauss_seidel, MatMN, MatN, VecN},
    types::*,
    PhysicsConfig,
};
use bevy::prelude::*;
use bevy_inspector_egui::Inspectable;

use super::Constraint;

#[derive(Component, Inspectable)]
pub struct ConstraintDistance {
    pub body_a: Entity,
    pub body_b: Entity,
    pub anchor_a: Vec3,
    pub anchor_b: Vec3,

    #[inspectable(ignore)]
    jacobian: MatMN<1, 12>,
    #[inspectable(ignore)]
    cached_lambda: VecN<1>,
    baumgarte: f32,
}

pub fn pre_solve(
    mut rb_query: Query<(
        &mut Transform,
        &mut LinearVelocity,
        &mut AngularVelocity,
        &InverseMass,
        &CenterOfMass,
        &InverseInertiaTensor,
    )>,
    mut constrain_distances: Query<&mut ConstraintDistance>,
    config: Res<PhysicsConfig>,
) {
    for mut constraint in constrain_distances.iter_mut() {
        let [(trans_a, mut lin_vel_a, mut ang_vel_a, inv_mass_a, com_a, inv_inertia_t_a), (trans_b, mut lin_vel_b, mut ang_vel_b, inv_mass_b, com_b, inv_inertia_t_b)] =
            rb_query.many_mut([constraint.body_a, constraint.body_b]);

        // get the world space position of the hinge from body_a's orientation
        let world_anchor_a = RBHelper::local_to_world(&trans_a, com_a, constraint.anchor_a);

        // get the world space position of the hinge from body_b's orientation
        let world_anchor_b = RBHelper::local_to_world(&trans_b, com_b, constraint.anchor_b);

        let r = world_anchor_b - world_anchor_a;
        let ra = world_anchor_a - RBHelper::centre_of_mass_world(&trans_a, com_a);
        let rb = world_anchor_b - RBHelper::centre_of_mass_world(&trans_b, com_b);
        let a = world_anchor_a;
        let b = world_anchor_b;

        {
            let j1 = (a - b) * 2.0;
            constraint.jacobian.rows[0][0] = j1.x;
            constraint.jacobian.rows[0][1] = j1.y;
            constraint.jacobian.rows[0][2] = j1.z;
        }

        {
            let j2 = ra.cross((a - b) * 2.0);
            constraint.jacobian.rows[0][3] = j2.x;
            constraint.jacobian.rows[0][4] = j2.y;
            constraint.jacobian.rows[0][5] = j2.z;
        }

        {
            let j3 = (b - a) * 2.0;
            constraint.jacobian.rows[0][6] = j3.x;
            constraint.jacobian.rows[0][7] = j3.y;
            constraint.jacobian.rows[0][8] = j3.z;
        }

        {
            let j4 = rb.cross((b - a) * 2.0);
            constraint.jacobian.rows[0][9] = j4.x;
            constraint.jacobian.rows[0][10] = j4.y;
            constraint.jacobian.rows[0][11] = j4.z;
        }

        // apply warm starting from the last frame
        let impulses = constraint.jacobian.transpose() * constraint.cached_lambda;
        Constraint::apply_impulses(
            &trans_a,
            &mut lin_vel_a,
            &mut ang_vel_a,
            inv_mass_a,
            inv_inertia_t_a,
            &trans_b,
            &mut lin_vel_b,
            &mut ang_vel_b,
            inv_mass_b,
            inv_inertia_t_b,
            impulses,
        );

        // calculate the baumgarte stabilization
        let mut c = r.dot(r);
        c = f32::max(0.0, c - 0.01);
        let beta = 0.05;
        constraint.baumgarte = (beta / config.time) * c;
    }
}

pub fn solve(
    mut rb_query: Query<(
        &mut Transform,
        &mut LinearVelocity,
        &mut AngularVelocity,
        &InverseMass,
        &InverseInertiaTensor,
    )>,
    mut constrain_distances: Query<&mut ConstraintDistance>,
) {
    for mut constraint in constrain_distances.iter_mut() {
        let jacobian_transpose = constraint.jacobian.transpose();

        let [(trans_a, mut lin_vel_a, mut ang_vel_a, inv_mass_a, inv_inertia_t_a), (trans_b, mut lin_vel_b, mut ang_vel_b, inv_mass_b, inv_inertia_t_b)] =
            rb_query.many_mut([constraint.body_a, constraint.body_b]);

        // build the system of equations
        let q_dt = Constraint::get_velocities(&lin_vel_a, &ang_vel_a, &lin_vel_b, &ang_vel_b);

        let inv_mass_matrix = Constraint::get_inverse_mass_matrix(
            &trans_a,
            inv_mass_a,
            inv_inertia_t_a,
            &trans_b,
            inv_mass_b,
            inv_inertia_t_b,
        );
        let j_w_jt = constraint.jacobian * inv_mass_matrix * jacobian_transpose;
        let mut rhs = constraint.jacobian * q_dt * -1.0;
        rhs[0] -= constraint.baumgarte;

        // solve for the Lagrange multipliers
        let lambda_n = lcp_gauss_seidel(&MatN::from(j_w_jt), &rhs);

        // apply the impulses
        let impulses = jacobian_transpose * lambda_n;
        Constraint::apply_impulses(
            &trans_a,
            &mut lin_vel_a,
            &mut ang_vel_a,
            inv_mass_a,
            inv_inertia_t_a,
            &trans_b,
            &mut lin_vel_b,
            &mut ang_vel_b,
            inv_mass_b,
            inv_inertia_t_b,
            impulses,
        );

        // accumulate the impulses for warm starting
        constraint.cached_lambda += lambda_n;
    }
}

pub fn post_solve(mut constrain_distances: Query<&mut ConstraintDistance>) {
    for mut constraint in constrain_distances.iter_mut() {
        // limit the warm starting to reasonable limits
        if !constraint.cached_lambda[0].is_finite() {
            constraint.cached_lambda[0] = 0.0
        }

        const LIMIT: f32 = 1e5;
        if constraint.cached_lambda[0] > LIMIT {
            constraint.cached_lambda[0] = LIMIT;
        }
        if constraint.cached_lambda[0] < -LIMIT {
            constraint.cached_lambda[0] = -LIMIT;
        }
    }
}
