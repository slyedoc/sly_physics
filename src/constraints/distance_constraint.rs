use crate::{
    math::{lcp_gauss_seidel, MatMN, MatN, VecN},
    types::*,
    RBQueryItem,
};
use bevy::prelude::*;

use super::{Constrainable, Constraint};

#[derive(Default, Reflect, Component)]
#[reflect(Component)]
pub struct DistanceConstraint {
    pub b: Option<Entity>,
    pub anchor_b: Vec3,
    pub anchor_a: Vec3,
    #[reflect(ignore)]
    pub jacobian: MatMN<1, 12>,
    #[reflect(ignore)]
    pub cached_lambda: VecN<1>,
    pub baumgarte: f32,
}



impl Constrainable for DistanceConstraint {
    fn get_b(&self) -> Option<Entity> {
        self.b
    }

    fn get_anchor_a(&self) -> Vec3 {
        self.anchor_a
    }

    fn get_anchor_b(&self) -> Vec3 {
        self.anchor_b
    }

    fn pre_solve(&mut self, a: &mut RBQueryItem, b: &mut RBQueryItem, dt: f32) {
        // get the world space position of the hinge from body_a's orientation
        let world_anchor_a =
            RBHelper::local_to_world(a.transform.as_ref(), a.center_of_mass, self.anchor_a);

        // get the world space position of the hinge from body_b's orientation
        let world_anchor_b =
            RBHelper::local_to_world(b.transform.as_ref(), b.center_of_mass, self.anchor_b);

        let r = world_anchor_b - world_anchor_a;
        let ra =
            world_anchor_a - RBHelper::centre_of_mass_world(a.transform.as_ref(), a.center_of_mass);
        let rb =
            world_anchor_b - RBHelper::centre_of_mass_world(b.transform.as_ref(), b.center_of_mass);

        {
            let j1 = (world_anchor_a - world_anchor_b) * 2.0;
            self.jacobian.rows[0][0] = j1.x;
            self.jacobian.rows[0][1] = j1.y;
            self.jacobian.rows[0][2] = j1.z;
        }

        {
            let j2 = ra.cross((world_anchor_a - world_anchor_b) * 2.0);
            self.jacobian.rows[0][3] = j2.x;
            self.jacobian.rows[0][4] = j2.y;
            self.jacobian.rows[0][5] = j2.z;
        }

        {
            let j3 = (world_anchor_b - world_anchor_a) * 2.0;
            self.jacobian.rows[0][6] = j3.x;
            self.jacobian.rows[0][7] = j3.y;
            self.jacobian.rows[0][8] = j3.z;
        }

        {
            let j4 = rb.cross((world_anchor_b - world_anchor_a) * 2.0);
            self.jacobian.rows[0][9] = j4.x;
            self.jacobian.rows[0][10] = j4.y;
            self.jacobian.rows[0][11] = j4.z;
        }

        // apply warm starting from the last frame
        let impulses = self.jacobian.transpose() * self.cached_lambda;
        Constraint::apply_impulses(a, b, impulses);

        // calculate the baumgarte stabilization
        let mut c = r.dot(r);
        c = f32::max(0.0, c - 0.01);
        let beta = 0.05;
        self.baumgarte = (beta / dt) * c;
    }

    fn solve(&mut self, a: &mut RBQueryItem, b: &mut RBQueryItem) {
        let jacobian_transpose = self.jacobian.transpose();
        // build the system of equations
        let q_dt = Constraint::get_velocities(&a.velocity, &b.velocity);

        let inv_mass_matrix = Constraint::get_inverse_mass_matrix(a, b);
        let j_w_jt = self.jacobian * inv_mass_matrix * jacobian_transpose;
        let mut rhs = self.jacobian * q_dt * -1.0;
        rhs[0] -= self.baumgarte;

        // solve for the Lagrange multipliers
        let lambda_n = lcp_gauss_seidel(&MatN::from(j_w_jt), &rhs);

        // apply the impulses
        let impulses = jacobian_transpose * lambda_n;
        Constraint::apply_impulses(a, b, impulses);

        // accumulate the impulses for warm starting
        self.cached_lambda += lambda_n;
    }

    fn post_solve(&mut self) {
        // limit the warm starting to reasonable limits
        if !self.cached_lambda[0].is_finite() {
            self.cached_lambda[0] = 0.0
        }

        const LIMIT: f32 = 1e5;
        if self.cached_lambda[0] > LIMIT {
            self.cached_lambda[0] = LIMIT;
        }
        if self.cached_lambda[0] < -LIMIT {
            self.cached_lambda[0] = -LIMIT;
        }
    }
}
