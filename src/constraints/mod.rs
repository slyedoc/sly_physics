#![allow(dead_code)]
mod constant_velocity_constrant;
mod contact_manifold;
mod distance_constraint;
mod hinge_constraint;
mod hinge_limited_constraint;
mod motor_constraint;
mod orientation_constraint;

pub use contact_manifold::*;
pub use distance_constraint::*;
pub use hinge_constraint::*;
pub use hinge_limited_constraint::*;
pub use motor_constraint::*;
pub use orientation_constraint::*;

use crate::{
    math::*, types::*, PhysicsConfig, PhysicsFixedUpdate, PhysicsState, PhysicsSystem, RBHelper,
    RBQuery, RBQueryItem,
};
use bevy::prelude::*;

use iyes_loopless::prelude::*;

pub struct PhysicsConstraintsPlugin;

#[derive(Debug, Clone, PartialEq, Eq, Hash, SystemLabel)]
pub enum ConstraintSystem {
    Manifold,
    PreSolve,
    Solve,
    PostSolve,
}

impl Plugin for PhysicsConstraintsPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<ContactManifolds>()
            .add_system_set_to_stage(
                PhysicsFixedUpdate,
                ConditionSet::new()
                    .run_in_state(PhysicsState::Running)
                    .label(ConstraintSystem::Manifold)
                    .after(PhysicsSystem::Narrow)
                    .with_system(add_manifold_contacts)
                    .into(),
            )
            .add_system_set_to_stage(
                PhysicsFixedUpdate,
                ConditionSet::new()
                    .run_in_state(PhysicsState::Running)
                    .label(ConstraintSystem::PreSolve)
                    .after(ConstraintSystem::Manifold)
                    .with_system(pre_solve)
                    .with_system(contact_manifold::pre_solve)
                    .into(),
            )
            .add_system_set_to_stage(
                PhysicsFixedUpdate,
                ConditionSet::new()
                    .run_in_state(PhysicsState::Running)
                    .label(ConstraintSystem::Solve)
                    .after(ConstraintSystem::PreSolve)
                    .with_system(solve)
                    .with_system(contact_manifold::solve)
                    .into(),
            )
            .add_system_set_to_stage(
                PhysicsFixedUpdate,
                ConditionSet::new()
                    .run_in_state(PhysicsState::Running)
                    .label(ConstraintSystem::PostSolve)
                    .after(ConstraintSystem::Solve)
                    .before(PhysicsSystem::Resolve)
                    .with_system(post_solve)
                    .with_system(contact_manifold::post_solve)
                    .into(),
            )
            .register_type::<MotorConstraint>()
            .register_type::<OrientationConstraint>()
            .register_type::<HingeConstraint>()
            .register_type::<HingeLimitedConstraint>();
    }
}

fn add_manifold_contacts(
    query: Query<(&Transform, &CenterOfMass)>,
    mut manifold_arena: ResMut<ContactManifolds>,
    mut manifold_contacts: EventReader<ManifoldContact>,
) {
    for contact in manifold_contacts.iter() {
        let contact = contact.0;
        if let Ok([(trans_a, com_a), (trans_b, com_b)]) = query.get_many([contact.a, contact.b]) {
            let pair = EntityPair::new(contact.a, contact.b);
            if !manifold_arena.manifolds.contains_key(&pair) {
                manifold_arena
                    .manifolds
                    .insert(pair.clone(), Manifold::default());
            }
            let manifold = manifold_arena.manifolds.get_mut(&pair).unwrap();
            manifold.add_contact(trans_a, com_a, trans_b, com_b, contact);
        } else {
            warn!("manifold entities not found!");
        }
    }
}

pub trait Constrainable {
    fn get_b(&self) -> Option<Entity>;
    fn get_anchor_a(&self) -> Vec3;
    fn get_anchor_b(&self) -> Vec3;

    fn pre_solve(&mut self, a: &mut RBQueryItem, b: &mut RBQueryItem, dt: f32);
    fn solve(&mut self, a: &mut RBQueryItem, b: &mut RBQueryItem);
    fn post_solve(&mut self);
}

fn pre_solve(
    mut distance_query: Query<(Entity, &mut DistanceConstraint)>,
    mut hinge_query: Query<(Entity, &mut HingeConstraint)>,
    mut hinge_limited_query: Query<(Entity, &mut HingeLimitedConstraint)>,
    mut motor_query: Query<(Entity, &mut MotorConstraint)>,
    mut orientation_query: Query<(Entity, &mut OrientationConstraint)>,
    mut rb_query: Query<RBQuery>,
    config: Res<PhysicsConfig>,
) {
    for (entity, mut c) in distance_query.iter_mut() {
        if let Ok([mut a, mut b]) = rb_query.get_many_mut([entity, c.b.unwrap()]) {
            c.pre_solve(&mut a, &mut b, config.time);
        }
    }
    for (entity, mut c) in hinge_query.iter_mut() {
        if let Ok([mut a, mut b]) = rb_query.get_many_mut([entity, c.b.unwrap()]) {
            c.pre_solve(&mut a, &mut b, config.time);
        }
    }
    for (entity, mut c) in hinge_limited_query.iter_mut() {
        if let Ok([mut a, mut b]) = rb_query.get_many_mut([entity, c.b.unwrap()]) {
            c.pre_solve(&mut a, &mut b, config.time);
        }
    }
    for (entity, mut c) in motor_query.iter_mut() {
        if let Ok([mut a, mut b]) = rb_query.get_many_mut([entity, c.b.unwrap()]) {
            c.pre_solve(&mut a, &mut b, config.time);
        }
    }
    for (entity, mut c) in orientation_query.iter_mut() {
        if let Ok([mut a, mut b]) = rb_query.get_many_mut([entity, c.b.unwrap()]) {
            c.pre_solve(&mut a, &mut b, config.time);
        }
    }
}

fn solve(
    mut distance_query: Query<(Entity, &mut DistanceConstraint)>,
    mut hinge_query: Query<(Entity, &mut HingeConstraint)>,
    mut hinge_limited_query: Query<(Entity, &mut HingeLimitedConstraint)>,
    mut motor_query: Query<(Entity, &mut MotorConstraint)>,
    mut orientation_query: Query<(Entity, &mut OrientationConstraint)>,
    mut rb_query: Query<RBQuery>,
    config: Res<PhysicsConfig>,
) {
    for _ in 0..config.solver_iterations {
        for (entity, mut c) in distance_query
            .iter_mut()
            .filter(|(_, c)| c.b.is_some())
        {
            if let Ok([mut a, mut b]) = rb_query.get_many_mut([entity, c.b.unwrap()]) {
                c.solve(&mut a, &mut b);
            }
        }
        for (entity, mut c) in hinge_query.iter_mut().filter(|(_, c)| c.b.is_some()) {
            if let Ok([mut a, mut b]) = rb_query.get_many_mut([entity, c.b.unwrap()]) {
                c.solve(&mut a, &mut b);
            }
        }
        for (entity, mut c) in hinge_limited_query
            .iter_mut()
            .filter(|(_, c)| c.b.is_some())
        {
            if let Ok([mut a, mut b]) = rb_query.get_many_mut([entity, c.b.unwrap()]) {
                c.solve(&mut a, &mut b);
            }
        }
        for (entity, mut c) in motor_query.iter_mut().filter(|(_, c)| c.b.is_some()) {
            if let Ok([mut a, mut b]) = rb_query.get_many_mut([entity, c.b.unwrap()]) {
                c.solve(&mut a, &mut b);
            }
        }
        for (entity, mut c) in orientation_query
            .iter_mut()
            .filter(|(_, c)| c.b.is_some())
        {
            if let Ok([mut a, mut b]) = rb_query.get_many_mut([entity, c.b.unwrap()]) {
                c.solve(&mut a, &mut b);
            }
        }
    }
}

fn post_solve(
    mut distance_query: Query<&mut DistanceConstraint>,
    mut hinge_query: Query<&mut HingeConstraint>,
    mut hinge_limited_query: Query<&mut HingeLimitedConstraint>,
    mut motor_query: Query<&mut MotorConstraint>,
    mut orientation_query: Query<&mut OrientationConstraint>,
) {
    for mut c in distance_query.iter_mut() {
        c.post_solve();
    }
    for mut c in hinge_query.iter_mut() {
        c.post_solve();
    }
    for mut c in hinge_limited_query.iter_mut() {
        c.post_solve();
    }
    for mut c in motor_query.iter_mut() {
        c.post_solve();
    }
    for mut c in orientation_query.iter_mut() {
        c.post_solve();
    }
}

pub struct Constraint;

impl Constraint {
    pub fn get_inverse_mass_matrix(a: &RBQueryItem, b: &RBQueryItem) -> MatMN<12, 12> {
        let mut inv_mass_matrix = MatMN::zero();

        {
            inv_mass_matrix.rows[0][0] = a.inv_mass.0;
            inv_mass_matrix.rows[1][1] = a.inv_mass.0;
            inv_mass_matrix.rows[2][2] = a.inv_mass.0;

            let inv_inertia_a =
                RBHelper::inv_inertia_tensor_world(&a.transform, a.inv_inertia_tensor);
            for i in 0..3 {
                inv_mass_matrix.rows[3 + i][3] = inv_inertia_a.col(i)[0];
                inv_mass_matrix.rows[3 + i][3 + 1] = inv_inertia_a.col(i)[1];
                inv_mass_matrix.rows[3 + i][3 + 2] = inv_inertia_a.col(i)[2];
            }
        }

        {
            inv_mass_matrix.rows[6][6] = b.inv_mass.0;
            inv_mass_matrix.rows[7][7] = b.inv_mass.0;
            inv_mass_matrix.rows[8][8] = b.inv_mass.0;

            let inv_inertia_b =
                RBHelper::inv_inertia_tensor_world(&b.transform, b.inv_inertia_tensor);

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

    pub fn apply_impulses(a: &mut RBQueryItem, b: &mut RBQueryItem, impulses: VecN<12>) {
        {
            let force_internal_a = Vec3::from_slice(&impulses[0..]);
            let torque_internal_a = Vec3::from_slice(&impulses[3..]);

            RBHelper::apply_impulse_linear(&mut a.velocity, a.inv_mass, force_internal_a);
            RBHelper::apply_impulse_angular(
                &a.transform,
                &mut a.velocity,
                a.inv_mass,
                a.inv_inertia_tensor,
                torque_internal_a,
            )
        }

        {
            let force_internal_b = Vec3::from_slice(&impulses[6..]);
            let torque_internal_b = Vec3::from_slice(&impulses[9..]);
            RBHelper::apply_impulse_linear(&mut b.velocity, b.inv_mass, force_internal_b);
            RBHelper::apply_impulse_angular(
                &b.transform,
                &mut b.velocity,
                b.inv_mass,
                b.inv_inertia_tensor,
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
