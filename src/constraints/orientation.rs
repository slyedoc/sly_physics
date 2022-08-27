// use super::{quat_left, quat_right, Constraint, ConstraintConfig};
// use crate::{
//     body::BodyArena,
//     math::{lcp_gauss_seidel, MatMN, MatN},
// };
use crate::{
    math::{MatMN},
    //types::*,
    //PhysicsConfig,
};
use bevy::prelude::*;

#[derive(Component)]
pub struct ConstraintOrientation {
    q0: Quat,
    jacobian: MatMN<4, 12>,
    baumgarte: f32,
}

impl Default for ConstraintOrientation {
    fn default() -> Self {
        Self {
            q0: Default::default(),
            jacobian: MatMN::zero(),
            baumgarte: 0.0,
        }
    }
}

// fn pre_solve(
//     mut query: Query<(
//         &mut Transform,
//         &mut LinearVelocity,
//         &mut AngularVelocity,
//         &InverseMass,
//         &CenterOfMass,
//         &InverseInertiaTensor,
//         &ConstraintOrientation,
//     )>,
//     config: Res<PhysicsConfig>,
// ) {
    // for (trans, mut lin_vel, mut ang_vel, inv_mass, com, inv_inertia_t, constraint_orientation) in
    //     query.iter_mut()
    // {
        // let q0_inv = constraint_orientation.q0.inverse();
        // let q1_inv = trans.rotation.inverse();

        // let u = Vec3::X;
        // let v = Vec3::Y;
        // let w = Vec3::Z;

        // let p = Mat4::from_cols(Vec4::ZERO, Vec4::Y, Vec4::Z, Vec4::W);
        // let p_t = p.transpose(); // pointless but self documenting

        // let mat_a = p * quat_left(q1_inv) * quat_right(q2 * q0_inv) * p_t * -0.5;
        // let mat_b = p * quat_left(q1_inv) * quat_right(q2 * q0_inv) * p_t * 0.5;

        //         // the distance constraint
        //         self.jacobian = MatMN::zero();

        //         {
        //             // first row is the primary distance constraint that holds anchor points together
        //             let j1 = (a - b) * 2.0;
        //             self.jacobian.rows[0][0] = j1.x;
        //             self.jacobian.rows[0][1] = j1.y;
        //             self.jacobian.rows[0][2] = j1.z;

        //             let j2 = ra.cross((a - b) * 2.0);
        //             self.jacobian.rows[0][3] = j2.x;
        //             self.jacobian.rows[0][4] = j2.y;
        //             self.jacobian.rows[0][5] = j2.z;

        //             let j3 = (b - a) * 2.0;
        //             self.jacobian.rows[0][6] = j3.x;
        //             self.jacobian.rows[0][7] = j3.y;
        //             self.jacobian.rows[0][8] = j3.z;

        //             let j4 = rb.cross((b - a) * 2.0);
        //             self.jacobian.rows[0][9] = j4.x;
        //             self.jacobian.rows[0][10] = j4.y;
        //             self.jacobian.rows[0][11] = j4.z;
        //         }

        //         const IDX: usize = 1;

        //         // the quaternion jacobians
        //         {
        //             let j1 = Vec3::ZERO;
        //             self.jacobian.rows[1][0] = j1.x;
        //             self.jacobian.rows[1][1] = j1.y;
        //             self.jacobian.rows[1][2] = j1.z;

        //             let tmp = mat_a * Vec4::from((0.0, u));
        //             let j2 = Vec3::new(tmp[IDX + 0], tmp[IDX + 1], tmp[IDX + 2]);
        //             self.jacobian.rows[1][3] = j2.x;
        //             self.jacobian.rows[1][4] = j2.y;
        //             self.jacobian.rows[1][5] = j2.z;

        //             let j3 = Vec3::ZERO;
        //             self.jacobian.rows[1][6] = j3.x;
        //             self.jacobian.rows[1][7] = j3.y;
        //             self.jacobian.rows[1][8] = j3.z;

        //             let tmp = mat_b * Vec4::from((0.0, u));
        //             let j4 = Vec3::new(tmp[IDX + 0], tmp[IDX + 1], tmp[IDX + 2]);
        //             self.jacobian.rows[1][9] = j4.x;
        //             self.jacobian.rows[1][10] = j4.y;
        //             self.jacobian.rows[1][11] = j4.z;
        //         }
        //         {
        //             let j1 = Vec3::ZERO;
        //             self.jacobian.rows[2][0] = j1.x;
        //             self.jacobian.rows[2][1] = j1.y;
        //             self.jacobian.rows[2][2] = j1.z;

        //             let tmp = mat_a * Vec4::from((0.0, v));
        //             let j2 = Vec3::new(tmp[IDX + 0], tmp[IDX + 1], tmp[IDX + 2]);
        //             self.jacobian.rows[2][3] = j2.x;
        //             self.jacobian.rows[2][4] = j2.y;
        //             self.jacobian.rows[2][5] = j2.z;

        //             let j3 = Vec3::ZERO;
        //             self.jacobian.rows[2][6] = j3.x;
        //             self.jacobian.rows[2][7] = j3.y;
        //             self.jacobian.rows[2][8] = j3.z;

        //             let tmp = mat_b * Vec4::from((0.0, v));
        //             let j4 = Vec3::new(tmp[IDX + 0], tmp[IDX + 1], tmp[IDX + 2]);
        //             self.jacobian.rows[2][9] = j4.x;
        //             self.jacobian.rows[2][10] = j4.y;
        //             self.jacobian.rows[2][11] = j4.z;
        //         }
        //         {
        //             let j1 = Vec3::ZERO;
        //             self.jacobian.rows[3][0] = j1.x;
        //             self.jacobian.rows[3][1] = j1.y;
        //             self.jacobian.rows[3][2] = j1.z;

        //             let tmp = mat_a * Vec4::from((0.0, w));
        //             let j2 = Vec3::new(tmp[IDX + 0], tmp[IDX + 1], tmp[IDX + 2]);
        //             self.jacobian.rows[3][3] = j2.x;
        //             self.jacobian.rows[3][4] = j2.y;
        //             self.jacobian.rows[3][5] = j2.z;

        //             let j3 = Vec3::ZERO;
        //             self.jacobian.rows[3][6] = j3.x;
        //             self.jacobian.rows[3][7] = j3.y;
        //             self.jacobian.rows[3][8] = j3.z;

        //             let tmp = mat_b * Vec4::from((0.0, w));
        //             let j4 = Vec3::new(tmp[IDX + 0], tmp[IDX + 1], tmp[IDX + 2]);
        //             self.jacobian.rows[3][9] = j4.x;
        //             self.jacobian.rows[3][10] = j4.y;
        //             self.jacobian.rows[3][11] = j4.z;
        //         }

        //         // calculate the baumgarte stabilization
        //         let c = r.dot(r);
        //         const BETA: f32 = 0.5;
        //         self.baumgarte = (BETA / dt_sec) * c;
//     }
// }

//     fn solve(&mut self, bodies: &mut BodyArena) {
//         let jacobian_transpose = self.jacobian.transpose();

//         // build the system of equations
//         let q_dt = self.config.get_velocities(bodies);
//         let inv_mass_matrix = self.config.get_inverse_mass_matrix(bodies);
//         let j_w_jt = self.jacobian * inv_mass_matrix * jacobian_transpose;
//         let mut rhs = self.jacobian * q_dt * -1.0;
//         rhs[0] -= self.baumgarte;

//         // solve for the Lagrange multipliers
//         let lambda_n = lcp_gauss_seidel(&MatN::from(j_w_jt), &rhs);

//         // apply the impulses
//         let impulses = jacobian_transpose * lambda_n;
//         self.config.apply_impulses(bodies, impulses);
//     }
// }
