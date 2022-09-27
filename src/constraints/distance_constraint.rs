use crate::{
    math::{lcp_gauss_seidel, MatMN, MatN, VecN},
    types::*,
    PhysicsConfig,
};
use bevy::prelude::*;

use super::{Constraint, Constrainable};

#[derive(Reflect, Component)]
#[reflect(Component)]
pub struct DistanceConstraint {
    pub parent: Option<Entity>,
    pub parent_offset: Vec3,
    pub offset: Vec3,
    #[reflect(ignore)]
    pub jacobian: MatMN<1, 12>,
    #[reflect(ignore)]
    pub cached_lambda: VecN<1>,
    pub baumgarte: f32,
}

impl Default for DistanceConstraint {
    fn default() -> Self {
        Self {
            parent_offset: Vec3::ZERO,
            parent: None,
            offset: Vec3::ZERO,
            jacobian: MatMN::zero(),
            cached_lambda: VecN::zero(),
            baumgarte: Default::default(),
        }
    }
}

impl Constrainable for DistanceConstraint {
    fn get_parent(&self) -> Option<Entity> {
        self.parent
    }

    fn get_anchor(&self) -> Vec3 {
        self.offset
    }

    fn get_parent_anchor(&self) -> Vec3 {
        self.parent_offset
    }
}

pub fn pre_solve(
    mut rb_query: Query<(
        &mut Transform,
        &mut Velocity,
        &InverseMass,
        &CenterOfMass,
        &InverseInertiaTensor,
    )>,
    mut constraints_query: Query<(Entity, &mut DistanceConstraint)>,
    config: Res<PhysicsConfig>,
) {
    for (e, mut constraint) in constraints_query
        .iter_mut()
        .filter(|(_e, c)| c.parent.is_some())
    {
        if let Ok(
            [(trans_a, mut vel_a, inv_mass_a, com_a, inv_inertia_t_a), (trans_b, mut vel_b, inv_mass_b, com_b, inv_inertia_t_b)],
        ) = rb_query.get_many_mut([e, constraint.parent.unwrap()])
        {
            // get the world space position of the hinge from body_a's orientation
            let world_anchor_a = RBHelper::local_to_world(&trans_a, com_a, constraint.offset);

            // get the world space position of the hinge from body_b's orientation
            let world_anchor_b =
                RBHelper::local_to_world(&trans_b, com_b, constraint.parent_offset);

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
                &mut vel_a,
                inv_mass_a,
                inv_inertia_t_a,
                &trans_b,
                &mut vel_b,
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
}

pub fn solve(
    mut rb_query: Query<(
        &mut Transform,
        &mut Velocity,
        &InverseMass,
        &InverseInertiaTensor,
    )>,
    mut constraints_query: Query<(Entity, &mut DistanceConstraint)>,
) {
    for (e, mut constraint) in constraints_query
        .iter_mut()
        .filter(|(_e, c)| c.parent.is_some())
    {
        if let Ok(
            [(trans_a, mut vel_a, inv_mass_a, inv_inertia_t_a), (trans_b, mut vel_b, inv_mass_b, inv_inertia_t_b)],
        ) = rb_query.get_many_mut([e, constraint.parent.unwrap()])
        {
            let jacobian_transpose = constraint.jacobian.transpose();
            // build the system of equations
            let q_dt = Constraint::get_velocities(&vel_a, &vel_b);

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
                &mut vel_a,
                inv_mass_a,
                inv_inertia_t_a,
                &trans_b,
                &mut vel_b,
                inv_mass_b,
                inv_inertia_t_b,
                impulses,
            );

            // accumulate the impulses for warm starting
            constraint.cached_lambda += lambda_n;
        }
    }
}

pub fn post_solve(mut constraints_query: Query<&mut DistanceConstraint>) {
    for mut constraint in constraints_query.iter_mut().filter(|c| c.parent.is_some()) {
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