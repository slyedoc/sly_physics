#![allow(dead_code)]
mod constraint_constant_velocity;
mod contact_constraint;
mod distance_constraint;
mod hinge_quat_constraint;
mod hinge_quat_limited_constraint;
mod motor_constraint;
mod orientation_constraint;

use crate::{
    math::{MatMN, VecN},
    prelude::Velocity,
    InverseInertiaTensor, InverseMass, PhysicsFixedUpdate, PhysicsState,
    PhysicsSystem, RBHelper, MAX_SOLVE_ITERS,
};
use bevy::prelude::*;

use iyes_loopless::prelude::*;
// use constraint_constant_velocity::ConstraintConstantVelocityLimited;
// use constraint_distance::ConstraintDistance;
// use constraint_hinge_quat::ConstraintHingeQuatLimited;

// use constraint_mover::ConstraintMoverSimple;
pub use contact_constraint::*;
pub use distance_constraint::*;
pub use hinge_quat_constraint::*;
pub use motor_constraint::*;
pub use orientation_constraint::*;
pub use hinge_quat_limited_constraint::*;

pub struct PhysicsConstraintsPlugin;

#[derive(Debug, Clone, PartialEq, Eq, Hash, SystemLabel)]
pub enum ConstraintSystem {
    ConstraintPreSolve,
    ConstraintSolve0,
    ConstraintSolve1,
    ConstraintSolve2,
    ConstraintSolve3,
    ConstraintSolve4,
    ConstraintPostSolve,
}

impl Plugin for PhysicsConstraintsPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<ContactArena>().add_system_set_to_stage(
            PhysicsFixedUpdate,
            ConditionSet::new()
                .run_in_state(PhysicsState::Running)
                .label(ConstraintSystem::ConstraintPreSolve)
                .after(PhysicsSystem::Narrow)
                .with_system(contact_constraint::pre_solve)
                .with_system(orientation_constraint::pre_solve)
                .with_system(distance_constraint::pre_solve)
                .with_system(motor_constraint::pre_solve)
                .with_system(hinge_quat_constraint::pre_solve)
                .with_system(hinge_quat_limited_constraint::pre_solve)
                .into(),
        );

        // TODO: find better way to reuse systems
        for i in 0..MAX_SOLVE_ITERS {
            let labels = constraint_labels(i);
            app.add_system_set_to_stage(
                PhysicsFixedUpdate,
                ConditionSet::new()
                    .run_in_state(PhysicsState::Running)
                    .label(labels.0)
                    .after(labels.1)
                    .with_system(contact_constraint::solve)
                    .with_system(distance_constraint::solve)
                    .with_system(orientation_constraint::solve)
                    .with_system(motor_constraint::solve)
                    .with_system(hinge_quat_constraint::solve)
                    .with_system(hinge_quat_limited_constraint::solve)
                    .into(),
            );
        }
        app.add_system_set_to_stage(
            PhysicsFixedUpdate,
            ConditionSet::new()
                .run_in_state(PhysicsState::Running)
                .label(ConstraintSystem::ConstraintPostSolve)
                .after(constraint_post_solve_label())
                .before(PhysicsSystem::Resolve)
                .with_system(contact_constraint::post_solve)
                .with_system(distance_constraint::post_solve)
                .with_system(hinge_quat_constraint::post_solve)
                .with_system(hinge_quat_limited_constraint::post_solve)
                .into(),
        )
        .register_type::<MotorConstraint>()
        .register_type::<OrientationConstraint>()
        .register_type::<HingeQuatConstraint>()
        .register_type::<HingeQuatLimitedConstraint>();
    }
}

fn constraint_labels(i: u32) -> (ConstraintSystem, ConstraintSystem) {
    (
        match i {
            0 => ConstraintSystem::ConstraintSolve0,
            1 => ConstraintSystem::ConstraintSolve1,
            2 => ConstraintSystem::ConstraintSolve2,
            3 => ConstraintSystem::ConstraintSolve3,
            4 => ConstraintSystem::ConstraintSolve4,
            _ => unreachable!(),
        },
        match i {
            0 => ConstraintSystem::ConstraintPreSolve,
            1 => ConstraintSystem::ConstraintSolve0,
            2 => ConstraintSystem::ConstraintSolve1,
            3 => ConstraintSystem::ConstraintSolve2,
            4 => ConstraintSystem::ConstraintSolve3,
            _ => unreachable!(),
        },
    )
}

fn constraint_post_solve_label() -> ConstraintSystem {
    match MAX_SOLVE_ITERS {
        0 => unreachable!(),
        1 => ConstraintSystem::ConstraintSolve0,
        2 => ConstraintSystem::ConstraintSolve1,
        3 => ConstraintSystem::ConstraintSolve2,
        4 => ConstraintSystem::ConstraintSolve3,
        5 => ConstraintSystem::ConstraintSolve4,
        _ => unreachable!(),
    }
}

// TODO: make this a trait
pub trait Constrainable {

    fn get_parent(&self) -> Option<Entity>;
    fn get_anchor(&self) -> Vec3;
    fn get_parent_anchor(&self) -> Vec3;
}

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

            let inv_inertia_a = RBHelper::inv_inertia_tensor_world(trans_a, inv_inertia_tensor_a);
            for i in 0..3 {
                inv_mass_matrix.rows[3 + i][3] = inv_inertia_a.col(i)[0];
                inv_mass_matrix.rows[3 + i][3 + 1] = inv_inertia_a.col(i)[1];
                inv_mass_matrix.rows[3 + i][3 + 2] = inv_inertia_a.col(i)[2];
            }
        }

        {
            inv_mass_matrix.rows[6][6] = inv_mass_b.0;
            inv_mass_matrix.rows[7][7] = inv_mass_b.0;
            inv_mass_matrix.rows[8][8] = inv_mass_b.0;

            let inv_inertia_b = RBHelper::inv_inertia_tensor_world(trans_b, inv_inertia_tensor_b);

            for i in 0..3 {
                inv_mass_matrix.rows[9 + i][9] = inv_inertia_b.col(i)[0];
                inv_mass_matrix.rows[9 + i][9 + 1] = inv_inertia_b.col(i)[1];
                inv_mass_matrix.rows[9 + i][9 + 2] = inv_inertia_b.col(i)[2];
            }
        }

        inv_mass_matrix
    }

    pub fn get_velocities(vel_a: &Velocity, vel_b: &Velocity) -> VecN<12> {
        let mut q_dt = VecN::zero();

        q_dt[0] = vel_a.linear.x;
        q_dt[1] = vel_a.linear.y;
        q_dt[2] = vel_a.linear.z;

        q_dt[3] = vel_a.angular.x;
        q_dt[4] = vel_a.angular.y;
        q_dt[5] = vel_a.angular.z;

        q_dt[6] = vel_b.linear.x;
        q_dt[7] = vel_b.linear.y;
        q_dt[8] = vel_b.linear.z;

        q_dt[9] = vel_b.angular.x;
        q_dt[10] = vel_b.angular.y;
        q_dt[11] = vel_b.angular.z;

        q_dt
    }

    #[allow(clippy::too_many_arguments)]
    pub fn apply_impulses(
        // A
        trans_a: &Transform,
        vel_a: &mut Velocity,
        inv_mass_a: &InverseMass,
        inv_inertia_tensor_a: &InverseInertiaTensor,
        // B
        trans_b: &Transform,
        vel_b: &mut Velocity,
        inv_mass_b: &InverseMass,
        inv_inertia_tensor_b: &InverseInertiaTensor,
        impulses: VecN<12>,
    ) {
        {
            let force_internal_a = Vec3::from_slice(&impulses[0..]);
            let torque_internal_a = Vec3::from_slice(&impulses[3..]);

            RBHelper::apply_impulse_linear(vel_a, inv_mass_a, force_internal_a);
            RBHelper::apply_impulse_angular(
                trans_a,
                vel_a,
                inv_mass_a,
                inv_inertia_tensor_a,
                torque_internal_a,
            )
        }

        {
            let force_internal_b = Vec3::from_slice(&impulses[6..]);
            let torque_internal_b = Vec3::from_slice(&impulses[9..]);
            RBHelper::apply_impulse_linear(vel_b, inv_mass_b, force_internal_b);
            RBHelper::apply_impulse_angular(
                trans_b,
                vel_b,
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
