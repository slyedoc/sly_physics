#![allow(dead_code)]
mod constraint_constant_velocity;
mod constraint_hinge_quat;
mod constraint_motor;
mod constraint_mover;
mod constraint_orientation;
pub mod penetration_manifold;
pub mod distance;

use crate::{
    math::{MatMN, VecN},
    AngularVelocity, InverseInertiaTensor,
    InverseMass, LinearVelocity, RBHelper
};
use bevy::prelude::*;

// use constraint_constant_velocity::ConstraintConstantVelocityLimited;
// use constraint_distance::ConstraintDistance;
// use constraint_hinge_quat::ConstraintHingeQuatLimited;
// use constraint_motor::ConstraintMotor;
// use constraint_mover::ConstraintMoverSimple;
// use constraint_orientation::ConstraintOrientation;
pub use penetration_manifold::*;



pub struct Constraint;

impl Constraint {
    pub fn get_inverse_mass_matrix(
        // A
        trans_a: &Transform,
        inv_mass_a: &InverseMass,
        inv_inertia_tensor_a: &InverseInertiaTensor,
        // B
        trans_b: &Transform,
        inv_mass_b: &InverseMass,
        inv_inertia_tensor_b: &InverseInertiaTensor,
    ) -> MatMN<12, 12> {
        let mut inv_mass_matrix = MatMN::zero();

        {
            inv_mass_matrix.rows[0][0] = inv_mass_a.0;
            inv_mass_matrix.rows[1][1] = inv_mass_a.0;
            inv_mass_matrix.rows[2][2] = inv_mass_a.0;

            let inv_intertia_a = RBHelper::inv_intertia_tensor_world(trans_a, inv_inertia_tensor_a);
            for i in 0..3 {
                inv_mass_matrix.rows[3 + i][3] = inv_intertia_a.col(i)[0];
                inv_mass_matrix.rows[3 + i][3 + 1] = inv_intertia_a.col(i)[1];
                inv_mass_matrix.rows[3 + i][3 + 2] = inv_intertia_a.col(i)[2];
            }
        }

        {
            inv_mass_matrix.rows[6][6] = inv_mass_b.0;
            inv_mass_matrix.rows[7][7] = inv_mass_b.0;
            inv_mass_matrix.rows[8][8] = inv_mass_b.0;

            let inv_intertia_b = RBHelper::inv_intertia_tensor_world(trans_b, inv_inertia_tensor_b);

            for i in 0..3 {
                inv_mass_matrix.rows[9 + i][9] = inv_intertia_b.col(i)[0];
                inv_mass_matrix.rows[9 + i][9 + 1] = inv_intertia_b.col(i)[1];
                inv_mass_matrix.rows[9 + i][9 + 2] = inv_intertia_b.col(i)[2];
            }
        }

        inv_mass_matrix
    }

    pub fn get_velocities(
        lin_vel_a: &LinearVelocity,
        ang_vel_a: &AngularVelocity,
        lin_vel_b: &LinearVelocity,
        ang_vel_b: &AngularVelocity,
    ) -> VecN<12> {
        let mut q_dt = VecN::zero();

        q_dt[0] = lin_vel_a.x;
        q_dt[1] = lin_vel_a.y;
        q_dt[2] = lin_vel_a.z;

        q_dt[3] = ang_vel_a.x;
        q_dt[4] = ang_vel_a.y;
        q_dt[5] = ang_vel_a.z;

        q_dt[6] = lin_vel_b.x;
        q_dt[7] = lin_vel_b.y;
        q_dt[8] = lin_vel_b.z;

        q_dt[9] = ang_vel_b.x;
        q_dt[10] = ang_vel_b.y;
        q_dt[11] = ang_vel_b.z;

        q_dt
    }

    #[allow(clippy::too_many_arguments)]
    pub fn apply_impulses(
        // A
        trans_a: &Transform,
        lin_vel_a: &mut LinearVelocity,
        ang_vel_a: &mut AngularVelocity,
        inv_mass_a: &InverseMass,
        inv_inertia_tensor_a: &InverseInertiaTensor,
        // B
        trans_b: &Transform,
        lin_vel_b: &mut LinearVelocity,
        ang_vel_b: &mut AngularVelocity,
        inv_mass_b: &InverseMass,
        inv_inertia_tensor_b: &InverseInertiaTensor,
        impulses: VecN<12>,
    ) {
        {
            let force_internal_a = Vec3::from_slice(&impulses[0..]);
            let torque_internal_a = Vec3::from_slice(&impulses[3..]);

            RBHelper::apply_impulse_linear(lin_vel_a, inv_mass_a, force_internal_a);
            RBHelper::apply_impulse_angular(
                trans_a,
                ang_vel_a,
                inv_mass_a,
                inv_inertia_tensor_a,
                torque_internal_a,
            )
        }

        {
            let force_internal_b = Vec3::from_slice(&impulses[6..]);
            let torque_internal_b = Vec3::from_slice(&impulses[9..]);
            RBHelper::apply_impulse_linear(lin_vel_b, inv_mass_b, force_internal_b);
            RBHelper::apply_impulse_angular(
                trans_b,
                ang_vel_b,
                inv_mass_b,
                inv_inertia_tensor_b,
                torque_internal_b,
            )
        }
    }
}

// impl ConstraintArena {
//     pub fn clear(&mut self) {
//         self.penetration_constraints.clear();
//     }

//     // pub fn add_orientation_constraint(
//     //     &mut self,
//     //     bodies: &BodyArena,
//     //     handle_a: BodyHandle,
//     //     handle_b: BodyHandle,
//     // ) {
//     //     let body_a = bodies.get_body(handle_a);
//     //     let body_b = bodies.get_body(handle_b);

//     //     let world_space_anchor = body_a.position;

//     //     self.constraints.push(Box::new(ConstraintOrientation::new(
//     //         ConstraintConfig {
//     //             handle_a,
//     //             handle_b,
//     //             anchor_a: body_a.world_to_local(world_space_anchor),
//     //             anchor_b: body_b.world_to_local(world_space_anchor),
//     //             ..ConstraintConfig::default()
//     //         },
//     //         body_a.orientation.inverse() * body_b.orientation,
//     //     )))
//     // }

//     // pub fn add_distance_constraint(
//     //     &mut self,
//     //     bodies: &BodyArena,
//     //     handle_a: BodyHandle,
//     //     handle_b: BodyHandle,
//     // ) {
//     //     let body_a = bodies.get_body(handle_a);
//     //     let body_b = bodies.get_body(handle_b);
//     //     let joint_world_space_anchor = body_a.position;

//     //     let anchor_a = body_a.world_to_local(joint_world_space_anchor);
//     //     let anchor_b = body_b.world_to_local(joint_world_space_anchor);

//     //     self.constraints
//     //         .push(Box::new(ConstraintDistance::new(ConstraintConfig {
//     //             handle_a,
//     //             handle_b,
//     //             anchor_a,
//     //             axis_a: Vec3::ZERO,
//     //             anchor_b,
//     //             axis_b: Vec3::ZERO,
//     //         })));
//     // }

//     // pub fn add_hinge_constraint(
//     //     &mut self,
//     //     bodies: &BodyArena,
//     //     handle_a: BodyHandle,
//     //     handle_b: BodyHandle,
//     //     world_space_anchor: Vec3,
//     //     axis: Vec3,
//     // ) {
//     //     let body_a = bodies.get_body(handle_a);
//     //     let body_b = bodies.get_body(handle_b);

//     //     let relative_orientation = body_a.orientation.inverse() * body_b.orientation;

//     //     self.constraints
//     //         .push(Box::new(ConstraintHingeQuatLimited::new(
//     //             ConstraintConfig {
//     //                 handle_a,
//     //                 handle_b,
//     //                 anchor_a: body_a.world_to_local(world_space_anchor),
//     //                 anchor_b: body_b.world_to_local(world_space_anchor),
//     //                 axis_a: axis,
//     //                 axis_b: Vec3::ZERO,
//     //             },
//     //             relative_orientation,
//     //         )))
//     // }

//     // pub fn add_constant_velocity_constraint(
//     //     &mut self,
//     //     bodies: &BodyArena,
//     //     handle_a: BodyHandle,
//     //     handle_b: BodyHandle,
//     //     world_space_anchor: Vec3,
//     //     axis: Vec3,
//     // ) {
//     //     let body_a = bodies.get_body(handle_a);
//     //     let body_b = bodies.get_body(handle_b);

//     //     let relative_orientation = body_a.orientation.inverse() * body_b.orientation;

//     //     self.constraints
//     //         .push(Box::new(ConstraintConstantVelocityLimited::new(
//     //             ConstraintConfig {
//     //                 handle_a,
//     //                 handle_b,
//     //                 anchor_a: body_a.world_to_local(world_space_anchor),
//     //                 anchor_b: body_b.world_to_local(world_space_anchor),
//     //                 axis_a: axis,
//     //                 axis_b: Vec3::ZERO,
//     //             },
//     //             relative_orientation,
//     //         )))
//     // }

//     // pub fn add_constraint_motor(
//     //     &mut self,
//     //     bodies: &BodyArena,
//     //     handle_a: BodyHandle,
//     //     handle_b: BodyHandle,
//     //     world_space_anchor: Vec3,
//     //     motor_axis: Vec3,
//     //     motor_speed: f32,
//     // ) {
//     //     let body_a = bodies.get_body(handle_a);
//     //     let body_b = bodies.get_body(handle_b);

//     //     // set the initial relative orientation (in body_a's space)
//     //     let q0 = body_a.orientation.inverse() * body_b.orientation;

//     //     self.constraints.push(Box::new(ConstraintMotor::new(
//     //         ConstraintConfig {
//     //             handle_a,
//     //             handle_b,
//     //             anchor_a: body_a.world_to_local(world_space_anchor),
//     //             anchor_b: body_b.world_to_local(world_space_anchor),
//     //             ..ConstraintConfig::default()
//     //         },
//     //         q0,
//     //         motor_axis,
//     //         motor_speed,
//     //     )))
//     // }

//     // pub fn add_constraint_mover(&mut self, _bodies: &BodyArena, handle_a: BodyHandle) {
//     //     self.constraints
//     //         .push(Box::new(ConstraintMoverSimple::new(ConstraintConfig {
//     //             handle_a,
//     //             ..ConstraintConfig::default()
//     //         })))
//     // }

//     // pub fn pre_solve(&mut self, bodies: &mut BodyArena, dt_sec: f32) {
//     //     for constraint in &mut self.constraints {
//     //         constraint.pre_solve(bodies, dt_sec);
//     //     }
//     // }

//     // pub fn solve(&mut self, bodies: &mut BodyArena) {
//     //     for constraint in &mut self.constraints {
//     //         constraint.solve(bodies);
//     //     }
//     // }

//     // pub fn post_solve(&mut self) {
//     //     for constraint in &mut self.constraints {
//     //         constraint.post_solve();
//     //     }
//     // }
// }

pub fn quat_left(q: Quat) -> Mat4 {
    Mat4::from_cols(
        Vec4::new(q.w, -q.x, -q.y, -q.z),
        Vec4::new(q.x, q.w, -q.z, q.y),
        Vec4::new(q.y, q.z, q.w, -q.x),
        Vec4::new(q.z, -q.y, q.x, q.w),
    )
}

pub fn quat_right(q: Quat) -> Mat4 {
    Mat4::from_cols(
        Vec4::new(q.w, -q.x, -q.y, -q.z),
        Vec4::new(q.x, q.w, q.z, -q.y),
        Vec4::new(q.y, -q.z, q.w, q.x),
        Vec4::new(q.z, q.y, -q.x, q.w),
    )
}
