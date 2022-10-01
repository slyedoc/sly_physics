use crate::{
    RBQueryItem,
    math::*,
    types::*,
};
use bevy::prelude::*;

use super::{Constrainable, Constraint};

#[derive(Reflect, Default, Component)]
#[reflect(Component)]
pub struct OrientationConstraint {
    pub anchor_a: Vec3,
    #[reflect(ignore)]
    pub b: Option<Entity>,
    pub anchor_b: Vec3,
    pub q0: Quat,
    #[reflect(ignore)]
    pub jacobian: MatMN<4, 12>,
    pub baumgarte: f32,
}

impl Constrainable for OrientationConstraint {
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
        let world_anchor_a =
            RBHelper::local_to_world(&a.transform, a.center_of_mass, self.anchor_a);
        let world_anchor_b =
            RBHelper::local_to_world(&b.transform, b.center_of_mass, self.anchor_b);

        let r = world_anchor_b - world_anchor_a;
        let ra = world_anchor_a - RBHelper::centre_of_mass_world(&a.transform, a.center_of_mass);
        let rb = world_anchor_b - RBHelper::centre_of_mass_world(&b.transform, b.center_of_mass);

        // get the orientation information of the bodies
        let q1 = a.transform.rotation;
        let q2 = b.transform.rotation;
        let q0_inv = self.q0.inverse();
        let q1_inv = q1.inverse();

        let u = Vec3::X;
        let v = Vec3::Y;
        let w = Vec3::Z;

        let p = Mat4::from_cols(Vec4::ZERO, Vec4::Y, Vec4::Z, Vec4::W);
        let p_t = p.transpose(); // pointless but self documenting

        let mat_a = p * quat_left(q1_inv) * quat_right(q2 * q0_inv) * p_t * -0.5;
        let mat_b = p * quat_left(q1_inv) * quat_right(q2 * q0_inv) * p_t * 0.5;

        // the distance constraint
        self.jacobian = MatMN::zero();

        {
            // first row is the primary distance constraint that holds anchor points together
            let j1 = (world_anchor_a - world_anchor_b) * 2.0;
            self.jacobian.rows[0][0] = j1.x;
            self.jacobian.rows[0][1] = j1.y;
            self.jacobian.rows[0][2] = j1.z;

            let j2 = ra.cross((world_anchor_a - world_anchor_b) * 2.0);
            self.jacobian.rows[0][3] = j2.x;
            self.jacobian.rows[0][4] = j2.y;
            self.jacobian.rows[0][5] = j2.z;

            let j3 = (world_anchor_b - world_anchor_a) * 2.0;
            self.jacobian.rows[0][6] = j3.x;
            self.jacobian.rows[0][7] = j3.y;
            self.jacobian.rows[0][8] = j3.z;

            let j4 = rb.cross((world_anchor_b - world_anchor_a) * 2.0);
            self.jacobian.rows[0][9] = j4.x;
            self.jacobian.rows[0][10] = j4.y;
            self.jacobian.rows[0][11] = j4.z;
        }

        const IDX: usize = 1;

        // the quaternion jacobians
        {
            let j1 = Vec3::ZERO;
            self.jacobian.rows[1][0] = j1.x;
            self.jacobian.rows[1][1] = j1.y;
            self.jacobian.rows[1][2] = j1.z;

            let tmp = mat_a * Vec4::from((0.0, u));
            let j2 = Vec3::new(tmp[IDX], tmp[IDX + 1], tmp[IDX + 2]);
            self.jacobian.rows[1][3] = j2.x;
            self.jacobian.rows[1][4] = j2.y;
            self.jacobian.rows[1][5] = j2.z;

            let j3 = Vec3::ZERO;
            self.jacobian.rows[1][6] = j3.x;
            self.jacobian.rows[1][7] = j3.y;
            self.jacobian.rows[1][8] = j3.z;

            let tmp = mat_b * Vec4::from((0.0, u));
            let j4 = Vec3::new(tmp[IDX], tmp[IDX + 1], tmp[IDX + 2]);
            self.jacobian.rows[1][9] = j4.x;
            self.jacobian.rows[1][10] = j4.y;
            self.jacobian.rows[1][11] = j4.z;
        }
        {
            let j1 = Vec3::ZERO;
            self.jacobian.rows[2][0] = j1.x;
            self.jacobian.rows[2][1] = j1.y;
            self.jacobian.rows[2][2] = j1.z;

            let tmp = mat_a * Vec4::from((0.0, v));
            let j2 = Vec3::new(tmp[IDX], tmp[IDX + 1], tmp[IDX + 2]);
            self.jacobian.rows[2][3] = j2.x;
            self.jacobian.rows[2][4] = j2.y;
            self.jacobian.rows[2][5] = j2.z;

            let j3 = Vec3::ZERO;
            self.jacobian.rows[2][6] = j3.x;
            self.jacobian.rows[2][7] = j3.y;
            self.jacobian.rows[2][8] = j3.z;

            let tmp = mat_b * Vec4::from((0.0, v));
            let j4 = Vec3::new(tmp[IDX], tmp[IDX + 1], tmp[IDX + 2]);
            self.jacobian.rows[2][9] = j4.x;
            self.jacobian.rows[2][10] = j4.y;
            self.jacobian.rows[2][11] = j4.z;
        }
        {
            let j1 = Vec3::ZERO;
            self.jacobian.rows[3][0] = j1.x;
            self.jacobian.rows[3][1] = j1.y;
            self.jacobian.rows[3][2] = j1.z;

            let tmp = mat_a * Vec4::from((0.0, w));
            let j2 = Vec3::new(tmp[IDX], tmp[IDX + 1], tmp[IDX + 2]);
            self.jacobian.rows[3][3] = j2.x;
            self.jacobian.rows[3][4] = j2.y;
            self.jacobian.rows[3][5] = j2.z;

            let j3 = Vec3::ZERO;
            self.jacobian.rows[3][6] = j3.x;
            self.jacobian.rows[3][7] = j3.y;
            self.jacobian.rows[3][8] = j3.z;

            let tmp = mat_b * Vec4::from((0.0, w));
            let j4 = Vec3::new(tmp[IDX], tmp[IDX + 1], tmp[IDX + 2]);
            self.jacobian.rows[3][9] = j4.x;
            self.jacobian.rows[3][10] = j4.y;
            self.jacobian.rows[3][11] = j4.z;
        }

        // calculate the baumgarte stabilization
        let c = r.dot(r);
        const BETA: f32 = 0.5;
        self.baumgarte = (BETA / dt) * c;
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
    }

    fn post_solve(&mut self) {}
}
