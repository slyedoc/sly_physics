use crate::{
    constraints::{quat_left, quat_right, RBQueryItem},
    math::{lcp_gauss_seidel, MatMN, MatN, VecN},
    types::*,
};
use bevy::prelude::*;

use super::{Constrainable, Constraint};

#[derive(Component, Reflect)]
#[reflect(Component)]
pub struct MotorConstraint {
    pub anchor_a: Vec3,
    pub parent: Option<Entity>,
    pub anchor_b: Vec3,

    pub motor_axis: Vec3,
    pub motor_speed: f32,
    pub q0: Option<Quat>,
    #[reflect(ignore)]
    pub jacobian: MatMN<4, 12>,
    pub baumgarte: Vec3,
}

impl Default for MotorConstraint {
    fn default() -> Self {
        Self {
            anchor_b: Vec3::ZERO,
            parent: None,
            anchor_a: Vec3::ZERO,
            q0: None,
            motor_axis: Vec3::Y,
            motor_speed: 0.0,

            jacobian: MatMN::zero(),
            baumgarte: Default::default(),
        }
    }
}

impl Constrainable for MotorConstraint {
    fn get_parent(&self) -> Option<Entity> {
        self.parent
    }

    fn get_anchor_a(&self) -> Vec3 {
        self.anchor_a
    }

    fn get_anchor_b(&self) -> Vec3 {
        self.anchor_b
    }

    fn pre_solve(&mut self, a: &mut RBQueryItem, b: &mut RBQueryItem, dt: f32) {
        // set initial rotation
        if self.q0.is_none() {
            self.q0 = Some(a.transform.rotation.inverse() * b.transform.rotation);
        }

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
        let q0_inv = self.q0.unwrap().inverse();
        let q1_inv = q1.inverse();

        // the axis is defined in the local space of body_a
        let motor_axis = a.transform.rotation * self.motor_axis;
        let (motor_u, motor_v) = motor_axis.any_orthonormal_pair();

        let u = motor_u;
        let v = motor_v;
        let w = motor_axis;

        let p = Mat4::from_cols(Vec4::ZERO, Vec4::Y, Vec4::Z, Vec4::W);
        let p_t = p.transpose(); // pointless but self documenting

        let mat_a = p * quat_left(q1_inv) * quat_right(q2 * q0_inv) * p_t * -0.5;
        let mat_b = p * quat_left(q1_inv) * quat_right(q2 * q0_inv) * p_t * 0.5;

        self.jacobian = MatMN::zero();

        // first row is primary distance constraint that holds the anchor points together
        {
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

        // the quaternion jacobians
        const IDX: usize = 1;

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
        let beta = 0.05;
        let c = r.dot(r);

        let qr = a.transform.rotation.inverse() * b.transform.rotation;
        let qr_a = qr * q0_inv; // relative orientation in body_a's space

        // get the world space axis for the relative rotation
        let axis_a = a.transform.rotation * qr_a.xyz();

        self.baumgarte[0] = (beta / dt) * c;
        self.baumgarte[1] = motor_u.dot(axis_a) * (beta / dt);
        self.baumgarte[2] = motor_v.dot(axis_a) * (beta / dt);
    }

    fn solve(&mut self, a: &mut RBQueryItem, b: &mut RBQueryItem) {
        let motor_axis = a.transform.rotation * self.motor_axis;

        let mut w_dt = VecN::zero();
        w_dt[3] = motor_axis[0] * -self.motor_speed;
        w_dt[4] = motor_axis[1] * -self.motor_speed;
        w_dt[5] = motor_axis[2] * -self.motor_speed;
        w_dt[9] = motor_axis[0] * self.motor_speed;
        w_dt[10] = motor_axis[1] * self.motor_speed;
        w_dt[11] = motor_axis[2] * self.motor_speed;

        let jacobian_transpose = self.jacobian.transpose();

        // build the system of equations
        let q_dt = Constraint::get_velocities(&a.velocity, &b.velocity) - w_dt; // by subtracting by the desired velocity, the solver is tricked into applying the impulse to give us that velocity
        let inv_mass_matrix = Constraint::get_inverse_mass_matrix(a, b);
        let j_w_jt = self.jacobian * inv_mass_matrix * jacobian_transpose;
        let mut rhs = self.jacobian * q_dt * -1.0;
        rhs[0] -= self.baumgarte[0];
        rhs[1] -= self.baumgarte[1];
        rhs[2] -= self.baumgarte[2];

        // solve for the lagrange multipliers
        let lambda_n = lcp_gauss_seidel(&MatN::from(j_w_jt), &rhs);

        // apply the impulses
        let impulses = jacobian_transpose * lambda_n;
        Constraint::apply_impulses(a, b, impulses);
    }

    fn post_solve(&mut self) {}
}
