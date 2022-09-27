use crate::{
    math::{lcp_gauss_seidel, MatMN, MatN, VecN},
    prelude::{quat_left, quat_right},
    types::*,
    PhysicsConfig,
};
use bevy::prelude::*;

use super::{Constraint, Constrainable};

#[derive(Reflect, Component)]
#[reflect(Component)]
pub struct HingeQuatLimitedConstraint {
    #[reflect(ignore)]
    pub parent: Option<Entity>,
    pub parent_offset: Vec3,
    pub offset: Vec3,
    // the axis is defined in the local space
    pub axis: Vec3,
    pub relative_angle: f32, // in degrees
    pub is_angle_violated: bool,

    // the initial relative quaternion q1^-1 * q2
    pub q0: Option<Quat>,
    #[reflect(ignore)]
    pub jacobian: MatMN<3, 12>,
    #[reflect(ignore)]
    pub cached_lambda: VecN<3>,
    pub baumgarte: f32,
}

impl Default for HingeQuatLimitedConstraint {
    fn default() -> Self {
        Self {
            parent_offset: Vec3::ZERO,
            parent: None,
            axis: Vec3::ZERO,
            offset: Vec3::ZERO,
            q0: None,
            jacobian: MatMN::zero(),
            cached_lambda: VecN::zero(),
            baumgarte: Default::default(),
            relative_angle: 0.0,
            is_angle_violated: false,
        }
    }
}

impl Constrainable for HingeQuatLimitedConstraint {
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
    mut constraints_query: Query<(Entity, &mut HingeQuatLimitedConstraint)>,
    mut rb_query: Query<(
        &mut Transform,
        &mut Velocity,
        &InverseMass,
        &InverseInertiaTensor,
        &CenterOfMass,
    )>,
    config: Res<PhysicsConfig>,
) {
    for (e, mut constraint) in constraints_query
        .iter_mut()
        .filter(|(_e, c)| c.parent.is_some())
    {
        if let Ok(
            [(trans_a, mut vel_a, inv_mass_a, inv_inertia_tensor_a, com_a), (trans_b, mut vel_b, inv_mass_b, inv_inertia_tensor_b, com_b)],
        ) = rb_query.get_many_mut([e, constraint.parent.unwrap()])
        {
            if constraint.q0.is_none() {
                constraint.q0 = Some(trans_a.rotation.inverse() * trans_b.rotation);
            }
            // get the world space position of the hinge from body_a's orientation
            let world_anchor_a =
                RBHelper::local_to_world(trans_a.as_ref(), com_a, constraint.offset);
            let world_anchor_b =
                RBHelper::local_to_world(trans_b.as_ref(), com_b, constraint.parent_offset);

            let r = world_anchor_b - world_anchor_a;
            let ra = world_anchor_a - RBHelper::centre_of_mass_world(trans_a.as_ref(), com_a);
            let rb = world_anchor_b - RBHelper::centre_of_mass_world(trans_b.as_ref(), com_b);
            let a = world_anchor_a;
            let b = world_anchor_b;

            // get the orientation information of the bodies
            let q1 = trans_a.rotation;
            let q2 = trans_b.rotation;
            let q0_inv = constraint.q0.unwrap().inverse();
            let q1_inv = q1.inverse();

            // the axis is defined in the local space of body_a
            let hinge_axis = constraint.axis;
            let (u, v) = hinge_axis.any_orthonormal_pair();

            let p = Mat4::from_cols(Vec4::ZERO, Vec4::Y, Vec4::Z, Vec4::W);
            let p_t = p.transpose(); // pointless but self documenting

            let mat_a = p * quat_left(q1_inv) * quat_right(q2 * q0_inv) * p_t * -0.5;
            let mat_b = p * quat_left(q1_inv) * quat_right(q2 * q0_inv) * p_t * 0.5;

            let qr = q1_inv * q2;
            let qrr = qr * q0_inv;
            constraint.relative_angle = 2.0 * qrr.xyz().dot(hinge_axis).asin().to_degrees();

            // check if there's an angle violation
            constraint.is_angle_violated = constraint.relative_angle.abs() > 45.0;

            constraint.jacobian = MatMN::zero();

            // first row is the primary distance constraint that holds the anchor points together
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

            const IDX: usize = 1;

            // the quaternion jacobians
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
            if constraint.is_angle_violated {
                let j1 = Vec3::ZERO;
                constraint.jacobian.rows[3][0] = j1.x;
                constraint.jacobian.rows[3][1] = j1.y;
                constraint.jacobian.rows[3][2] = j1.z;

                let tmp = mat_a * Vec4::from((0.0, hinge_axis));
                // TODO: might be a good addition
                // let tmp = mat_a * Vec4::from((0.0, hinge_axis));
                let j2 = Vec3::new(tmp[IDX + 0], tmp[IDX + 1], tmp[IDX + 2]);
                constraint.jacobian.rows[3][3] = j2.x;
                constraint.jacobian.rows[3][4] = j2.y;
                constraint.jacobian.rows[3][5] = j2.z;

                let j3 = Vec3::ZERO;
                constraint.jacobian.rows[3][6] = j3.x;
                constraint.jacobian.rows[3][7] = j3.y;
                constraint.jacobian.rows[3][8] = j3.z;

                let tmp = mat_b * Vec4::from((0.0, hinge_axis));
                let j4 = Vec3::new(tmp[IDX + 0], tmp[IDX + 1], tmp[IDX + 2]);
                constraint.jacobian.rows[3][9] = j4.x;
                constraint.jacobian.rows[3][10] = j4.y;
                constraint.jacobian.rows[3][11] = j4.z;
            }

            // apply warm starting from last frame
            let impulses = constraint.jacobian.transpose() * constraint.cached_lambda;
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

            // calculate the baumgarte stabilization
            let c = r.dot(r);
            let c = f32::max(0.0, c - 0.01);
            const BETA: f32 = 0.05;
            constraint.baumgarte = (BETA / config.time) * c;
        }
    }
}

pub fn solve(
    mut constraints_query: Query<(Entity, &mut HingeQuatLimitedConstraint)>,
    mut rb_query: Query<(
        &mut Transform,
        &mut Velocity,
        &InverseMass,
        &InverseInertiaTensor,
    )>,
) {
    for (e, mut constraint) in constraints_query
        .iter_mut()
        .filter(|(_e, c)| c.parent.is_some())
    {
        if let Ok(
            [(trans_a, mut vel_a, inv_mass_a, inv_inertia_tensor_a), (trans_b, mut vel_b, inv_mass_b, inv_inertia_tensor_b)],
        ) = rb_query.get_many_mut([e, constraint.parent.unwrap()])
        {
            let jacobian_transpose = constraint.jacobian.transpose();
            // build the system of equations
            let q_dt = Constraint::get_velocities(&vel_a, &vel_b);
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
            rhs[0] -= constraint.baumgarte;

            // solve for the Lagrange multipliers
            let mut lambda_n = lcp_gauss_seidel(&MatN::from(j_w_jt), &rhs);

            // clamp the torque from the angle constraint
            // we need to make sure it's a restorative torque
            if constraint.is_angle_violated {
                if constraint.relative_angle > 0.0 {
                    lambda_n[3] = f32::min(0.0, lambda_n[3]);
                } else if constraint.relative_angle < 0.0 {
                    lambda_n[3] = f32::max(0.0, lambda_n[3]);
                }
            }

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
            // accumulate the impulses for warm starting
            constraint.cached_lambda += lambda_n;
        }
    }
}

pub fn post_solve(mut constraints_query: Query<&mut HingeQuatLimitedConstraint>) {
    for mut constraint in constraints_query.iter_mut().filter(|c| c.parent.is_some()) {
        // limit the warm starting to reasonable limits
        for cached_lambda in constraint.cached_lambda.iter_mut() {
            if !cached_lambda.is_finite() {
                *cached_lambda = 0.0
            }

            const LIMIT: f32 = 20.0;
            if *cached_lambda > LIMIT {
                *cached_lambda = LIMIT;
            }
            if *cached_lambda < -LIMIT {
                *cached_lambda = -LIMIT;
            }
        }
    }
}

// pub struct ConstraintHingeQuatLimited {
//     config: ConstraintConfig,
//     // the initial relative quaternion q1^-1 * q2
//     q0: Quat,
//     jacobian: MatMN<4, 12>,
//     cached_lambda: VecN<4>,
//     baumgarte: f32,
//     relative_angle: f32, // in degrees
//     is_angle_violated: bool,
// }

// impl ConstraintHingeQuatLimited {
//     pub fn new(config: ConstraintConfig, q0: Quat) -> Self {
//         Self {
//             config,
//             q0,
//             jacobian: MatMN::zero(),
//             cached_lambda: VecN::zero(),
//             baumgarte: 0.0,
//             relative_angle: 0.0,
//             is_angle_violated: false,
//         }
//     }
// }

// impl Constraint for ConstraintHingeQuatLimited {
//     fn pre_solve(&mut self, bodies: &mut BodyArena, dt_sec: f32) {
//         let body_a = bodies.get_body(constraint.config.handle_a);
//         let body_b = bodies.get_body(constraint.config.handle_b);

//         // get the world space position of the hinge from body_a's orientation
//         let world_anchor_a = body_a.local_to_world(constraint.config.anchor_a);

//         // get the world space position of the hinge from body_b's orientation
//         let world_anchor_b = body_b.local_to_world(constraint.config.anchor_b);

//         let r = world_anchor_b - world_anchor_a;
//         let ra = world_anchor_a - body_a.centre_of_mass_world();
//         let rb = world_anchor_b - body_b.centre_of_mass_world();
//         let a = world_anchor_a;
//         let b = world_anchor_b;

//         // get the orientation information of the bodies
//         let q1 = body_a.orientation;
//         let q2 = body_b.orientation;
//         let q0_inv = constraint.q0.inverse();
//         let q1_inv = q1.inverse();

//         // the axis is defined in the local space of body_a
//         let hinge_axis = constraint.config.axis_a;
//         let (u, v) = hinge_axis.any_orthonormal_pair();

//         let p = Mat4::from_cols(Vec4::ZERO, Vec4::Y, Vec4::Z, Vec4::W);
//         let p_t = p.transpose(); // pointless but self documenting

//         let mat_a = p * quat_left(q1_inv) * quat_right(q2 * q0_inv) * p_t * -0.5;
//         let mat_b = p * quat_left(q1_inv) * quat_right(q2 * q0_inv) * p_t * 0.5;

//         let qr = q1_inv * q2;
//         let qrr = qr * q0_inv;
//         constraint.relative_angle = 2.0 * qrr.xyz().dot(hinge_axis).asin().to_degrees();

//         // check if there's an angle violation
//         constraint.is_angle_violated = constraint.relative_angle.abs() > 45.0;

//         constraint.jacobian = MatMN::zero();

//         // first row is the primary distance constraint that holds the anchor points together
//         {
//             let j1 = (a - b) * 2.0;
//             constraint.jacobian.rows[0][0] = j1.x;
//             constraint.jacobian.rows[0][1] = j1.y;
//             constraint.jacobian.rows[0][2] = j1.z;

//             let j2 = ra.cross((a - b) * 2.0);
//             constraint.jacobian.rows[0][3] = j2.x;
//             constraint.jacobian.rows[0][4] = j2.y;
//             constraint.jacobian.rows[0][5] = j2.z;

//             let j3 = (b - a) * 2.0;
//             constraint.jacobian.rows[0][6] = j3.x;
//             constraint.jacobian.rows[0][7] = j3.y;
//             constraint.jacobian.rows[0][8] = j3.z;

//             let j4 = rb.cross((b - a) * 2.0);
//             constraint.jacobian.rows[0][9] = j4.x;
//             constraint.jacobian.rows[0][10] = j4.y;
//             constraint.jacobian.rows[0][11] = j4.z;
//         }

//         const IDX: usize = 1;

//         // the quaternion jacobians
//         {
//             let j1 = Vec3::ZERO;
//             constraint.jacobian.rows[1][0] = j1.x;
//             constraint.jacobian.rows[1][1] = j1.y;
//             constraint.jacobian.rows[1][2] = j1.z;

//             let tmp = mat_a * Vec4::from((0.0, u));
//             let j2 = Vec3::new(tmp[IDX + 0], tmp[IDX + 1], tmp[IDX + 2]);
//             constraint.jacobian.rows[1][3] = j2.x;
//             constraint.jacobian.rows[1][4] = j2.y;
//             constraint.jacobian.rows[1][5] = j2.z;

//             let j3 = Vec3::ZERO;
//             constraint.jacobian.rows[1][6] = j3.x;
//             constraint.jacobian.rows[1][7] = j3.y;
//             constraint.jacobian.rows[1][8] = j3.z;

//             let tmp = mat_b * Vec4::from((0.0, u));
//             let j4 = Vec3::new(tmp[IDX + 0], tmp[IDX + 1], tmp[IDX + 2]);
//             constraint.jacobian.rows[1][9] = j4.x;
//             constraint.jacobian.rows[1][10] = j4.y;
//             constraint.jacobian.rows[1][11] = j4.z;
//         }
//         {
//             let j1 = Vec3::ZERO;
//             constraint.jacobian.rows[2][0] = j1.x;
//             constraint.jacobian.rows[2][1] = j1.y;
//             constraint.jacobian.rows[2][2] = j1.z;

//             let tmp = mat_a * Vec4::from((0.0, v));
//             let j2 = Vec3::new(tmp[IDX + 0], tmp[IDX + 1], tmp[IDX + 2]);
//             constraint.jacobian.rows[2][3] = j2.x;
//             constraint.jacobian.rows[2][4] = j2.y;
//             constraint.jacobian.rows[2][5] = j2.z;

//             let j3 = Vec3::ZERO;
//             constraint.jacobian.rows[2][6] = j3.x;
//             constraint.jacobian.rows[2][7] = j3.y;
//             constraint.jacobian.rows[2][8] = j3.z;

//             let tmp = mat_b * Vec4::from((0.0, v));
//             let j4 = Vec3::new(tmp[IDX + 0], tmp[IDX + 1], tmp[IDX + 2]);
//             constraint.jacobian.rows[2][9] = j4.x;
//             constraint.jacobian.rows[2][10] = j4.y;
//             constraint.jacobian.rows[2][11] = j4.z;
//         }
//         if constraint.is_angle_violated {
//             let j1 = Vec3::ZERO;
//             constraint.jacobian.rows[3][0] = j1.x;
//             constraint.jacobian.rows[3][1] = j1.y;
//             constraint.jacobian.rows[3][2] = j1.z;

//             let tmp = mat_a * Vec4::from((0.0, hinge_axis));
//             // TODO: might be a good addition
//             // let tmp = mat_a * Vec4::from((0.0, hinge_axis));
//             let j2 = Vec3::new(tmp[IDX + 0], tmp[IDX + 1], tmp[IDX + 2]);
//             constraint.jacobian.rows[3][3] = j2.x;
//             constraint.jacobian.rows[3][4] = j2.y;
//             constraint.jacobian.rows[3][5] = j2.z;

//             let j3 = Vec3::ZERO;
//             constraint.jacobian.rows[3][6] = j3.x;
//             constraint.jacobian.rows[3][7] = j3.y;
//             constraint.jacobian.rows[3][8] = j3.z;

//             let tmp = mat_b * Vec4::from((0.0, hinge_axis));
//             let j4 = Vec3::new(tmp[IDX + 0], tmp[IDX + 1], tmp[IDX + 2]);
//             constraint.jacobian.rows[3][9] = j4.x;
//             constraint.jacobian.rows[3][10] = j4.y;
//             constraint.jacobian.rows[3][11] = j4.z;
//         }

//         // apply warm starting from last frame
//         let impulses = constraint.jacobian.transpose() * constraint.cached_lambda;
//         constraint.config.apply_impulses(bodies, impulses);

//         // calculate the baumgarte stabilization
//         let c = r.dot(r);
//         let c = f32::max(0.0, c - 0.01);
//         const BETA: f32 = 0.05;
//         constraint.baumgarte = (BETA / dt_sec) * c;
//     }

//     fn solve(&mut self, bodies: &mut BodyArena) {
//         let jacobian_transpose = constraint.jacobian.transpose();

//         // build the system of equations
//         let q_dt = constraint.config.get_velocities(bodies);
//         let inv_mass_matrix = constraint.config.get_inverse_mass_matrix(bodies);
//         let j_w_jt = constraint.jacobian * inv_mass_matrix * jacobian_transpose;
//         let mut rhs = constraint.jacobian * q_dt * -1.0;
//         rhs[0] -= constraint.baumgarte;

//         // solve for the Lagrange multipliers
//         let mut lambda_n = lcp_gauss_seidel(&MatN::from(j_w_jt), &rhs);

//         // clamp the torque from the angle constraint
//         // we need to make sure it's a restorative torque
//         if constraint.is_angle_violated {
//             if constraint.relative_angle > 0.0 {
//                 lambda_n[3] = f32::min(0.0, lambda_n[3]);
//             } else if constraint.relative_angle < 0.0 {
//                 lambda_n[3] = f32::max(0.0, lambda_n[3]);
//             }
//         }

//         // apply the impulses
//         let impulses = jacobian_transpose * lambda_n;
//         constraint.config.apply_impulses(bodies, impulses);

//         // accumulate the impulses for warm starting
//         constraint.cached_lambda += lambda_n;
//     }

//     fn post_solve(&mut self) {
//         // limit the warm starting to reasonable limits
//         for cached_lambda in constraint.cached_lambda.iter_mut() {
//             if !cached_lambda.is_finite() {
//                 *cached_lambda = 0.0
//             }

//             const LIMIT: f32 = 20.0;
//             if *cached_lambda > LIMIT {
//                 *cached_lambda = LIMIT;
//             }
//             if *cached_lambda < -LIMIT {
//                 *cached_lambda = -LIMIT;
//             }
//         }
//     }
// }
