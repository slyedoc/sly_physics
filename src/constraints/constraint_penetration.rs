use crate::{
    math::*, AngularVelocity, CenterOfMass, Elasticity, Friction, InertiaTensor,
    InverseInertiaTensor, InverseMass, LinearVelocity, RBHelper,
};
use bevy::prelude::*;

use super::Constraint;

#[derive(Copy, Clone, Debug, Default)]
pub struct ConstraintPenetration {
    pub anchor_a: Vec3, // the anchor location in body_a's space
    pub anchor_b: Vec3, // the anchor location in body_b's space

    jacobian: MatMN<3, 12>,
    cached_lambda: VecN<3>,
    normal: Vec3, // in body A's local space
    baumgarte: f32,
    friction: f32,
}

impl ConstraintPenetration {
    pub fn new(local_point_a: Vec3, local_point_b: Vec3, normal: Vec3) -> Self {
        Self {
            anchor_a: local_point_a,
            anchor_b: local_point_b,
            jacobian: MatMN::zero(),
            cached_lambda: VecN::zero(),
            normal,
            baumgarte: 0.0,
            friction: 0.0,
        }
    }

    pub fn normal(&self) -> Vec3 {
        self.normal
    }

    pub fn clear_cached_lambda(&mut self) {
        self.cached_lambda = VecN::zero();
    }
}

impl ConstraintPenetration {
    pub fn pre_solve(
        &mut self,
        rb_query: &mut Query<(
            &mut Transform,
            &mut LinearVelocity,
            &mut AngularVelocity,
            &InverseMass,
            &Elasticity,
            &Friction,
            &CenterOfMass,
            &InertiaTensor,
            &InverseInertiaTensor,
        )>,
        a: Entity,
        b: Entity,
        dt_sec: f32,
    ) {
        if let Ok(
            [(
                trans_a,
                mut lin_vel_a,
                mut ang_vel_a,
                inv_mass_a,
                _elas_a,
                frict_a,
                com_a,
                _inertia_t_a,
                inv_inert_t_a,
            ), (
                trans_b,
                mut lin_vel_b,
                mut ang_vel_b,
                inv_mass_b,
                _elas_b,
                frict_b,
                com_b,
                _inertia_t_b,
                inv_inert_t_b,
            )],
        ) = rb_query.get_many_mut([a, b])
        {
            // get the world space position of the hinge from body_a's orientation
            let world_anchor_a = RBHelper::local_to_world(&trans_a, com_a, self.anchor_a);
            // get the world space position of the hinge from body_b's orientation
            let world_anchor_b = RBHelper::local_to_world(&trans_b, com_b, self.anchor_b);

            let com_world_a = trans_a.translation + trans_a.rotation * com_a.0;
            let com_world_b = trans_b.translation + trans_b.rotation * com_b.0;
            let ra = world_anchor_a - com_world_a;
            let rb = world_anchor_b - com_world_b;
            self.friction = frict_a.0 * frict_b.0;

            // should be equivalent to Vec3::GetOrtho() from the book
            let (mut u, mut v) = self.normal.any_orthonormal_pair();

            // convert tangent space from model space to world space
            let normal = trans_a.rotation * self.normal;
            u = trans_a.rotation * u;
            v = trans_a.rotation * v;

            // penetration constraint
            self.jacobian = MatMN::zero();

            // first row is the primary distance constraint that holds the anchor points together
            {
                let j1 = -normal;
                self.jacobian.rows[0][0] = j1.x;
                self.jacobian.rows[0][1] = j1.y;
                self.jacobian.rows[0][2] = j1.z;
            }

            {
                let j2 = ra.cross(-normal);
                self.jacobian.rows[0][3] = j2.x;
                self.jacobian.rows[0][4] = j2.y;
                self.jacobian.rows[0][5] = j2.z;
            }

            {
                let j3 = normal;
                self.jacobian.rows[0][6] = j3.x;
                self.jacobian.rows[0][7] = j3.y;
                self.jacobian.rows[0][8] = j3.z;
            }

            {
                let j4 = rb.cross(normal);
                self.jacobian.rows[0][9] = j4.x;
                self.jacobian.rows[0][10] = j4.y;
                self.jacobian.rows[0][11] = j4.z;
            }

            // friction jacobians
            if self.friction > 0.0 {
                {
                    let j1 = -u;
                    self.jacobian.rows[1][0] = j1.x;
                    self.jacobian.rows[1][1] = j1.y;
                    self.jacobian.rows[1][2] = j1.z;
                }
                {
                    let j2 = ra.cross(-u);
                    self.jacobian.rows[1][3] = j2.x;
                    self.jacobian.rows[1][4] = j2.y;
                    self.jacobian.rows[1][5] = j2.z;
                }
                {
                    let j3 = u;
                    self.jacobian.rows[1][6] = j3.x;
                    self.jacobian.rows[1][7] = j3.y;
                    self.jacobian.rows[1][8] = j3.z;
                }
                {
                    let j4 = rb.cross(u);
                    self.jacobian.rows[1][9] = j4.x;
                    self.jacobian.rows[1][10] = j4.y;
                    self.jacobian.rows[1][11] = j4.z;
                }

                {
                    let j1 = -v;
                    self.jacobian.rows[2][0] = j1.x;
                    self.jacobian.rows[2][1] = j1.y;
                    self.jacobian.rows[2][2] = j1.z;
                }
                {
                    let j2 = ra.cross(-v);
                    self.jacobian.rows[2][3] = j2.x;
                    self.jacobian.rows[2][4] = j2.y;
                    self.jacobian.rows[2][5] = j2.z;
                }
                {
                    let j3 = v;
                    self.jacobian.rows[2][6] = j3.x;
                    self.jacobian.rows[2][7] = j3.y;
                    self.jacobian.rows[2][8] = j3.z;
                }
                {
                    let j4 = rb.cross(v);
                    self.jacobian.rows[2][9] = j4.x;
                    self.jacobian.rows[2][10] = j4.y;
                    self.jacobian.rows[2][11] = j4.z;
                }
            }

            // apply warm starting from last frame
            let impulses = self.jacobian.transpose() * self.cached_lambda;
            Constraint::apply_impulses(
                &trans_a,
                &mut lin_vel_a,
                &mut ang_vel_a,
                inv_mass_a,
                inv_inert_t_a,
                &trans_b,
                &mut lin_vel_b,
                &mut ang_vel_b,
                inv_mass_b,
                inv_inert_t_b,
                impulses,
            );
            // calculate the baumgarte stabilization
            let mut c = (world_anchor_b - world_anchor_a).dot(normal);
            c = f32::min(0.0, c + 0.02); // add slop
            let beta = 0.25;
            self.baumgarte = beta * c / dt_sec;
        } else {
            warn!("ConstraintPenetration: invalid query");
        }
    }

    pub fn solve(
        &mut self,
        rb_query: &mut Query<(
            &mut Transform,
            &mut LinearVelocity,
            &mut AngularVelocity,
            &InverseMass,
            &Elasticity,
            &Friction,
            &CenterOfMass,
            &InertiaTensor,
            &InverseInertiaTensor,
        )>,
        a: Entity,
        b: Entity,
    ) {
        let jacobian_transpose = self.jacobian.transpose();

        let [(
            trans_a,
            mut lin_vel_a,
            mut ang_vel_a,
            inv_mass_a,
            _elas_a,
            _frict_a,
            _com_a,
            _inertia_t_a,
            inv_inertia_tensor_a,
        ), (
            trans_b,
            mut lin_vel_b,
            mut ang_vel_b,
            inv_mass_b,
            _elas_b,
            _frict_b,
            _com_b,
            _inertia_t_b,
            inv_inertia_tensor_b,
        )] = rb_query.many_mut([a, b]);

        // build the system of equations
        let q_dt = Constraint::get_velocities(&lin_vel_a, &ang_vel_a, &lin_vel_b, &ang_vel_b);
        let inv_mass_matrix = Constraint::get_inverse_mass_matrix(
            &trans_a,
            inv_mass_a,
            inv_inertia_tensor_a,
            &trans_b,
            inv_mass_b,
            inv_inertia_tensor_b,
        );
        let j_w_jt = self.jacobian * inv_mass_matrix * jacobian_transpose;
        let mut rhs = self.jacobian * q_dt * -1.0;
        rhs[0] -= self.baumgarte;

        // solve for the Lagrange multipliers
        let mut lambda_n = lcp_gauss_seidel(&MatN::from(j_w_jt), &rhs);

        // accumulate the impulses and clamp within the constraint limits
        let old_lambda = self.cached_lambda;
        self.cached_lambda += lambda_n;
        let lambda_limit = 0.0;
        if self.cached_lambda[0] < lambda_limit {
            self.cached_lambda[0] = lambda_limit;
        }

        if self.friction > 0.0 {
            let umg = self.friction * 10.0 * 1.0 / (inv_mass_a.0 + inv_mass_b.0);
            let normal_force = (lambda_n[0] * self.friction).abs();
            let max_force = if umg > normal_force {
                umg
            } else {
                normal_force
            };

            if self.cached_lambda[1] > max_force {
                self.cached_lambda[1] = max_force;
            }
            if self.cached_lambda[1] < -max_force {
                self.cached_lambda[1] = -max_force;
            }

            if self.cached_lambda[2] > max_force {
                self.cached_lambda[2] = max_force;
            }
            if self.cached_lambda[2] < -max_force {
                self.cached_lambda[2] = -max_force;
            }
        }
        lambda_n = self.cached_lambda - old_lambda;

        // apply the impulses
        let impulses = jacobian_transpose * lambda_n;
        Constraint::apply_impulses(
            &trans_a,
            &mut lin_vel_a,
            &mut ang_vel_a,
            inv_mass_a,
            inv_inertia_tensor_a,
            &trans_b,
            &mut lin_vel_b,
            &mut ang_vel_b,
            inv_mass_b,
            inv_inertia_tensor_b,
            impulses,
        );
    }
}
