use crate::{
    constraints::{quat_left, quat_right},
    math::{lcp_gauss_seidel, MatMN, MatN, VecN},
    types::*,
    PhysicsConfig,
};
use bevy::prelude::*;

use super::{Constraint, Constrainable};

#[derive(Component, Reflect)]
#[reflect(Component)]
pub struct MotorConstraint {
    pub offset: Vec3,
    pub parent: Option<Entity>,
    pub parent_offset: Vec3,

    pub motor_axis: Vec3,
    pub motor_speed: f32,
    pub q0: Option<Quat>,
    #[reflect(ignore)]
    pub jacobian: MatMN<4, 12>,
    pub baumgarte: Vec3,
}

impl Constrainable for MotorConstraint {
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

impl Default for MotorConstraint {
    fn default() -> Self {
        Self {
            parent_offset: Vec3::ZERO,
            parent: None,
            offset: Vec3::ZERO,
            q0: None,
            motor_axis: Vec3::Y,
            motor_speed: 0.0,

            jacobian: MatMN::zero(),
            baumgarte: Default::default(),
        }
    }
}

pub fn pre_solve(
    mut constraint_query: Query<(Entity, &mut MotorConstraint)>,
    rb_query: Query<(&Transform, &CenterOfMass)>,
    config: Res<PhysicsConfig>,
) {
    for (e, mut constraint) in constraint_query
        .iter_mut()
        .filter(|(_e, c)| c.parent.is_some())
    {
        if let Ok([(trans_a, com_a), (trans_b, com_b)]) =
            rb_query.get_many([e, constraint.parent.unwrap()])
        {
            // set initial rotation
            if constraint.q0.is_none() {
                constraint.q0 = Some(trans_a.rotation.inverse() * trans_b.rotation);
            }

            let world_anchor_a = RBHelper::local_to_world(trans_a, com_a, constraint.offset);
            let world_anchor_b = RBHelper::local_to_world(trans_b, com_b, constraint.parent_offset);

            let r = world_anchor_b - world_anchor_a;
            let ra = world_anchor_a - RBHelper::centre_of_mass_world(trans_a, com_a);
            let rb = world_anchor_b - RBHelper::centre_of_mass_world(trans_b, com_b);
            let a = world_anchor_a;
            let b = world_anchor_b;

            // get the orientation information of the bodies
            let q1 = trans_a.rotation;
            let q2 = trans_b.rotation;
            let q0_inv = constraint.q0.unwrap().inverse();
            let q1_inv = q1.inverse();

            // the axis is defined in the local space of body_a
            let motor_axis = trans_a.rotation * constraint.motor_axis;
            let (motor_u, motor_v) = motor_axis.any_orthonormal_pair();

            let u = motor_u;
            let v = motor_v;
            let w = motor_axis;

            let p = Mat4::from_cols(Vec4::ZERO, Vec4::Y, Vec4::Z, Vec4::W);
            let p_t = p.transpose(); // pointless but self documenting

            let mat_a = p * quat_left(q1_inv) * quat_right(q2 * q0_inv) * p_t * -0.5;
            let mat_b = p * quat_left(q1_inv) * quat_right(q2 * q0_inv) * p_t * 0.5;

            constraint.jacobian = MatMN::zero();

            // first row is primary distance constraint that holds the anchor points together
            {
                let j1 = (a - b) * 2.0;
                constraint.jacobian.rows[0][0] = j1.x;
                constraint.jacobian.rows[0][1] = j1.y;
                constraint.jacobian.rows[0][2] = j1.z;

                let j2 = ra.cross((a - b) * 2.0);
                constraint.jacobian.rows[0][3] = j2.x;
                constraint.jacobian.rows[0][4] = j2.y;
                constraint.jacobian.rows[0][5] = j2.z;

                let j3 = (b - a) * 2.0;
                constraint.jacobian.rows[0][6] = j3.x;
                constraint.jacobian.rows[0][7] = j3.y;
                constraint.jacobian.rows[0][8] = j3.z;

                let j4 = rb.cross((b - a) * 2.0);
                constraint.jacobian.rows[0][9] = j4.x;
                constraint.jacobian.rows[0][10] = j4.y;
                constraint.jacobian.rows[0][11] = j4.z;
            }

            // the quaternion jacobians
            const IDX: usize = 1;

            {
                let j1 = Vec3::ZERO;
                constraint.jacobian.rows[1][0] = j1.x;
                constraint.jacobian.rows[1][1] = j1.y;
                constraint.jacobian.rows[1][2] = j1.z;

                let tmp = mat_a * Vec4::from((0.0, u));
                let j2 = Vec3::new(tmp[IDX + 0], tmp[IDX + 1], tmp[IDX + 2]);
                constraint.jacobian.rows[1][3] = j2.x;
                constraint.jacobian.rows[1][4] = j2.y;
                constraint.jacobian.rows[1][5] = j2.z;

                let j3 = Vec3::ZERO;
                constraint.jacobian.rows[1][6] = j3.x;
                constraint.jacobian.rows[1][7] = j3.y;
                constraint.jacobian.rows[1][8] = j3.z;

                let tmp = mat_b * Vec4::from((0.0, u));
                let j4 = Vec3::new(tmp[IDX + 0], tmp[IDX + 1], tmp[IDX + 2]);
                constraint.jacobian.rows[1][9] = j4.x;
                constraint.jacobian.rows[1][10] = j4.y;
                constraint.jacobian.rows[1][11] = j4.z;
            }
            {
                let j1 = Vec3::ZERO;
                constraint.jacobian.rows[2][0] = j1.x;
                constraint.jacobian.rows[2][1] = j1.y;
                constraint.jacobian.rows[2][2] = j1.z;

                let tmp = mat_a * Vec4::from((0.0, v));
                let j2 = Vec3::new(tmp[IDX + 0], tmp[IDX + 1], tmp[IDX + 2]);
                constraint.jacobian.rows[2][3] = j2.x;
                constraint.jacobian.rows[2][4] = j2.y;
                constraint.jacobian.rows[2][5] = j2.z;

                let j3 = Vec3::ZERO;
                constraint.jacobian.rows[2][6] = j3.x;
                constraint.jacobian.rows[2][7] = j3.y;
                constraint.jacobian.rows[2][8] = j3.z;

                let tmp = mat_b * Vec4::from((0.0, v));
                let j4 = Vec3::new(tmp[IDX + 0], tmp[IDX + 1], tmp[IDX + 2]);
                constraint.jacobian.rows[2][9] = j4.x;
                constraint.jacobian.rows[2][10] = j4.y;
                constraint.jacobian.rows[2][11] = j4.z;
            }
            {
                let j1 = Vec3::ZERO;
                constraint.jacobian.rows[3][0] = j1.x;
                constraint.jacobian.rows[3][1] = j1.y;
                constraint.jacobian.rows[3][2] = j1.z;

                let tmp = mat_a * Vec4::from((0.0, w));
                let j2 = Vec3::new(tmp[IDX + 0], tmp[IDX + 1], tmp[IDX + 2]);
                constraint.jacobian.rows[3][3] = j2.x;
                constraint.jacobian.rows[3][4] = j2.y;
                constraint.jacobian.rows[3][5] = j2.z;

                let j3 = Vec3::ZERO;
                constraint.jacobian.rows[3][6] = j3.x;
                constraint.jacobian.rows[3][7] = j3.y;
                constraint.jacobian.rows[3][8] = j3.z;

                let tmp = mat_b * Vec4::from((0.0, w));
                let j4 = Vec3::new(tmp[IDX + 0], tmp[IDX + 1], tmp[IDX + 2]);
                constraint.jacobian.rows[3][9] = j4.x;
                constraint.jacobian.rows[3][10] = j4.y;
                constraint.jacobian.rows[3][11] = j4.z;
            }

            // calculate the baumgarte stabilization
            let beta = 0.05;
            let c = r.dot(r);

            let qr = trans_a.rotation.inverse() * trans_b.rotation;
            let qr_a = qr * q0_inv; // relative orientation in body_a's space

            // get the world space axis for the relative rotation
            let axis_a = trans_a.rotation * qr_a.xyz();

            constraint.baumgarte[0] = (beta / config.time) * c;
            constraint.baumgarte[1] = motor_u.dot(axis_a) * (beta / config.time);
            constraint.baumgarte[2] = motor_v.dot(axis_a) * (beta / config.time);
        }
    }
}

pub fn solve(
    constraint_query: Query<(Entity, &MotorConstraint)>,
    mut rb_query: Query<(
        &mut Transform,
        &mut Velocity,
        &InverseMass,
        &InverseInertiaTensor,
    )>,
) {
    for (e, constraint) in constraint_query
        .iter()
        .filter(|(_e, c)| c.parent.is_some())
    {
        if let Ok(
            [(trans_a, mut vel_a, inv_mass_a, inv_inertia_tensor_a), (trans_b, mut vel_b, inv_mass_b, inv_inertia_tensor_b)],
        ) = rb_query.get_many_mut([e, constraint.parent.unwrap()])
        {
            let motor_axis = trans_a.rotation * constraint.motor_axis;

            let mut w_dt = VecN::zero();
            w_dt[3] = motor_axis[0] * -constraint.motor_speed;
            w_dt[4] = motor_axis[1] * -constraint.motor_speed;
            w_dt[5] = motor_axis[2] * -constraint.motor_speed;
            w_dt[9] = motor_axis[0] * constraint.motor_speed;
            w_dt[10] = motor_axis[1] * constraint.motor_speed;
            w_dt[11] = motor_axis[2] * constraint.motor_speed;

            let jacobian_transpose = constraint.jacobian.transpose();

            // build the system of equations
            let q_dt = Constraint::get_velocities(&vel_a, &vel_b) - w_dt; // by subtracting by the desired velocity, the solver is tricked into applying the impulse to give us that velocity
            let inv_mass_matrix = Constraint::get_inverse_mass_matrix(
                &trans_a,
                inv_mass_a,
                inv_inertia_tensor_a,
                &trans_b,
                inv_mass_b,
                inv_inertia_tensor_b,
            );
            let j_w_jt = constraint.jacobian * inv_mass_matrix * jacobian_transpose;
            let mut rhs = constraint.jacobian * q_dt * -1.0;
            rhs[0] -= constraint.baumgarte[0];
            rhs[1] -= constraint.baumgarte[1];
            rhs[2] -= constraint.baumgarte[2];

            // solve for the lagrange multipliers
            let lambda_n = lcp_gauss_seidel(&MatN::from(j_w_jt), &rhs);

            // apply the impulses
            let impulses = jacobian_transpose * lambda_n;
            Constraint::apply_impulses(
                &trans_a,
                &mut vel_a,
                inv_mass_a,
                inv_inertia_tensor_a,
                &trans_b,
                &mut vel_b,
                inv_mass_b,
                inv_inertia_tensor_b,
                impulses,
            );

        }
    }
}
