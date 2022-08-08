use bevy::{prelude::*, utils::HashMap};
use bevy_inspector_egui::Inspectable;

use crate::{
    math::{lcp_gauss_seidel, MatMN, MatN, VecN},
    types::*,
    CenterOfMass, Contact, PhysicsConfig, RBHelper, MAX_MANIFOLD_CONTACTS,
};

use super::Constraint;

#[derive(Debug,  Default)]
pub struct PenetrationArena {
    pub manifolds: HashMap<EntityPair, Manifold>,
}

#[derive(Debug, Inspectable, Clone)]
pub struct EntityPair {
    pub a: Entity,
    pub b: Entity,
}

impl EntityPair {
    pub fn new(a: Entity, b: Entity) -> Self {
        Self { a, b }
    }
}

impl PartialEq for EntityPair {
    fn eq(&self, other: &Self) -> bool {
        (self.a == other.a && self.b == other.b)
        || (self.a == other.a && self.b == other.a)
    }
}

impl Eq for EntityPair {}

impl std::hash::Hash for EntityPair {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        let product = self.a.id() * self.b.id();
        product.hash(state);
    }
}

impl PenetrationArena {
    pub fn add_contact(
        &mut self,
        contact: Contact,
        trans_a: &Transform,
        com_a: &CenterOfMass,
        trans_b: &Transform,
        com_b: &CenterOfMass,
    ) {
        let pair = EntityPair::new(contact.a, contact.b);

        if let Some(m) = self.manifolds.get_mut( &pair) {
            m.add_contact(trans_a, com_a, trans_b, com_b, contact);
        } else {
            let mut m = Manifold::default();
            m.add_contact(trans_a, com_a, trans_b, com_b, contact);
            self.manifolds.insert(pair, m);
        }
    }

    pub fn clear(&mut self) {
        self.manifolds.clear();
    }
}

#[derive(Debug)]
pub struct Manifold {
    pub contacts: Vec<Contact>,
    pub constraints: Vec<ConstraintPenetration>,
}

impl Default for Manifold {
    fn default() -> Self {
        Self {
            contacts: Vec::<Contact>::with_capacity(MAX_MANIFOLD_CONTACTS),
            constraints: Vec::<ConstraintPenetration>::with_capacity(MAX_MANIFOLD_CONTACTS),
        }
    }
}

impl Manifold {
    pub fn add_contact(
        &mut self,
        trans_a: &Transform,
        com_a: &CenterOfMass,
        trans_b: &Transform,
        com_b: &CenterOfMass,
        contact: Contact,
    ) {
        // if this contact is close to another contact then keep the old contact
        for manifold_contact in &self.contacts {
            // TODO: can we just the world_points on the contact?
            let old_a = RBHelper::local_to_world(trans_a, com_a, manifold_contact.local_point_b);
            let old_b = RBHelper::local_to_world(trans_b, com_b, manifold_contact.local_point_b);

            let new_a = RBHelper::local_to_world(trans_a, com_a, contact.local_point_b);
            let new_b = RBHelper::local_to_world(trans_b, com_b, contact.local_point_b);

            let aa = new_a - old_a;
            let bb = new_b - old_b;

            const DISTANCE_THRESHOLD: f32 = 0.02;
            const DISTANCE_THRESHOLD_SQ: f32 = DISTANCE_THRESHOLD * DISTANCE_THRESHOLD;
            if aa.length_squared() < DISTANCE_THRESHOLD_SQ {
                return;
            }
            if bb.length_squared() < DISTANCE_THRESHOLD_SQ {
                return;
            }
        }

        // if we're all full on contacts then keep the contacts that are furthest away from each
        // other
        let index = if self.contacts.len() == MAX_MANIFOLD_CONTACTS {
            let mut avg = Vec3::ZERO;
            for manifold_contact in &self.contacts {
                avg += manifold_contact.local_point_a;
            }
            avg += contact.local_point_a;
            avg *= 1.0 / MAX_MANIFOLD_CONTACTS as f32 + 1.0;

            let mut min_dist = (avg - contact.local_point_a).length_squared();
            let mut new_idx = None;
            for (i, c) in self.contacts.iter().enumerate() {
                let dist2 = (avg - c.local_point_a).length_squared();
                if dist2 < min_dist {
                    min_dist = dist2;
                    new_idx = Some(i);
                }
            }
            if new_idx.is_none() {
                return;
            } else {
                let i = new_idx.unwrap();
                self.contacts[i] = contact;
                i
            }
        } else {
            self.contacts.push(contact);
            self.contacts.len() - 1
        };

        let normal = (trans_a.rotation.inverse() * -contact.normal).normalize();
        let constraint =
            ConstraintPenetration::new(contact.local_point_a, contact.local_point_b, normal);
        if index < self.constraints.len() {
            self.constraints[index] = constraint;
        } else {
            self.constraints.push(constraint);
        }
    }
}

#[derive(Inspectable, Copy, Clone, Debug, Default)]
pub struct ConstraintPenetration {
    pub anchor_a: Vec3, // the anchor location in body_a's space
    pub anchor_b: Vec3, // the anchor location in body_b's space

    #[inspectable(ignore)]
    pub jacobian: MatMN<3, 12>,
    #[inspectable(ignore)]
    pub cached_lambda: VecN<3>,
    pub normal: Vec3, // in body A's local space
    pub baumgarte: f32,
    pub friction: f32,
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

pub fn pre_solve(
    mut rb_query: Query<(
        &mut Transform,
        &mut LinearVelocity,
        &mut AngularVelocity,
        &InverseMass,
        &Friction,
        &CenterOfMass,
        &InverseInertiaTensor,
    )>,
    mut manifold_arena: ResMut<PenetrationArena>,
    config: Res<PhysicsConfig>,
) {
    for (pair, manifold) in &mut manifold_arena.manifolds {
        if let Ok(
            [(trans_a, mut lin_vel_a, mut ang_vel_a, inv_mass_a, frict_a, com_a, inv_inert_t_a), (trans_b, mut lin_vel_b, mut ang_vel_b, inv_mass_b, frict_b, com_b, inv_inert_t_b)],
        ) = rb_query.get_many_mut([pair.a, pair.b])
        {
            for constraint in manifold.constraints.iter_mut() {
                // get the world space position of the hinge from body_a's orientation
                let world_anchor_a = RBHelper::local_to_world(&trans_a, com_a, constraint.anchor_a);
                // get the world space position of the hinge from body_b's orientation
                let world_anchor_b = RBHelper::local_to_world(&trans_b, com_b, constraint.anchor_b);

                let com_world_a = trans_a.translation + trans_a.rotation * com_a.0;
                let com_world_b = trans_b.translation + trans_b.rotation * com_b.0;
                let ra = world_anchor_a - com_world_a;
                let rb = world_anchor_b - com_world_b;
                constraint.friction = frict_a.0 * frict_b.0;

                // should be equivalent to Vec3::GetOrtho() from the book
                let (mut u, mut v) = constraint.normal.any_orthonormal_pair();

                // convert tangent space from model space to world space
                let normal = trans_a.rotation * constraint.normal;
                u = trans_a.rotation * u;
                v = trans_a.rotation * v;

                // penetration constraint
                constraint.jacobian = MatMN::zero();

                // first row is the primary distance constraint that holds the anchor points together
                {
                    let j1 = -normal;
                    constraint.jacobian.rows[0][0] = j1.x;
                    constraint.jacobian.rows[0][1] = j1.y;
                    constraint.jacobian.rows[0][2] = j1.z;
                }

                {
                    let j2 = ra.cross(-normal);
                    constraint.jacobian.rows[0][3] = j2.x;
                    constraint.jacobian.rows[0][4] = j2.y;
                    constraint.jacobian.rows[0][5] = j2.z;
                }

                {
                    let j3 = normal;
                    constraint.jacobian.rows[0][6] = j3.x;
                    constraint.jacobian.rows[0][7] = j3.y;
                    constraint.jacobian.rows[0][8] = j3.z;
                }

                {
                    let j4 = rb.cross(normal);
                    constraint.jacobian.rows[0][9] = j4.x;
                    constraint.jacobian.rows[0][10] = j4.y;
                    constraint.jacobian.rows[0][11] = j4.z;
                }

                // friction jacobians
                if constraint.friction > 0.0 {
                    {
                        let j1 = -u;
                        constraint.jacobian.rows[1][0] = j1.x;
                        constraint.jacobian.rows[1][1] = j1.y;
                        constraint.jacobian.rows[1][2] = j1.z;
                    }
                    {
                        let j2 = ra.cross(-u);
                        constraint.jacobian.rows[1][3] = j2.x;
                        constraint.jacobian.rows[1][4] = j2.y;
                        constraint.jacobian.rows[1][5] = j2.z;
                    }
                    {
                        let j3 = u;
                        constraint.jacobian.rows[1][6] = j3.x;
                        constraint.jacobian.rows[1][7] = j3.y;
                        constraint.jacobian.rows[1][8] = j3.z;
                    }
                    {
                        let j4 = rb.cross(u);
                        constraint.jacobian.rows[1][9] = j4.x;
                        constraint.jacobian.rows[1][10] = j4.y;
                        constraint.jacobian.rows[1][11] = j4.z;
                    }

                    {
                        let j1 = -v;
                        constraint.jacobian.rows[2][0] = j1.x;
                        constraint.jacobian.rows[2][1] = j1.y;
                        constraint.jacobian.rows[2][2] = j1.z;
                    }
                    {
                        let j2 = ra.cross(-v);
                        constraint.jacobian.rows[2][3] = j2.x;
                        constraint.jacobian.rows[2][4] = j2.y;
                        constraint.jacobian.rows[2][5] = j2.z;
                    }
                    {
                        let j3 = v;
                        constraint.jacobian.rows[2][6] = j3.x;
                        constraint.jacobian.rows[2][7] = j3.y;
                        constraint.jacobian.rows[2][8] = j3.z;
                    }
                    {
                        let j4 = rb.cross(v);
                        constraint.jacobian.rows[2][9] = j4.x;
                        constraint.jacobian.rows[2][10] = j4.y;
                        constraint.jacobian.rows[2][11] = j4.z;
                    }
                }

                // apply warm starting from last frame
                let impulses = constraint.jacobian.transpose() * constraint.cached_lambda;
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
                constraint.baumgarte = beta * c / config.time;
            }
        } else {
            // enties dont exist, so remove contacts so it get cleared
            manifold.contacts.clear();
        }
    }

    manifold_arena
        .manifolds
        .retain(|_pair, manifold| manifold.contacts.len() > 0)
}

pub fn solve(
    mut manifold_arena: ResMut<PenetrationArena>,
    mut rb_query: Query<(
        &mut Transform,
        &mut LinearVelocity,
        &mut AngularVelocity,
        &InverseMass,
        &InverseInertiaTensor,
    )>,
) {
    for (pair, manifold) in &mut manifold_arena.manifolds {
        let [(
            trans_a,
            mut lin_vel_a,
            mut ang_vel_a,
            inv_mass_a,
           
            inv_inertia_tensor_a,
        ), (
            trans_b,
            mut lin_vel_b,
            mut ang_vel_b,
            inv_mass_b,
            inv_inertia_tensor_b,
        )] = rb_query.many_mut([pair.a, pair.b]);

        for constraint in manifold.constraints.iter_mut() {
            let jacobian_transpose = constraint.jacobian.transpose();

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
            let j_w_jt = constraint.jacobian * inv_mass_matrix * jacobian_transpose;
            let mut rhs = constraint.jacobian * q_dt * -1.0;
            rhs[0] -= constraint.baumgarte;

            // solve for the Lagrange multipliers
            let mut lambda_n = lcp_gauss_seidel(&MatN::from(j_w_jt), &rhs);

            // accumulate the impulses and clamp within the constraint limits
            let old_lambda = constraint.cached_lambda;
            constraint.cached_lambda += lambda_n;
            let lambda_limit = 0.0;
            if constraint.cached_lambda[0] < lambda_limit {
                constraint.cached_lambda[0] = lambda_limit;
            }

            if constraint.friction > 0.0 {
                let umg = constraint.friction * 10.0 * 1.0 / (inv_mass_a.0 + inv_mass_b.0);
                let normal_force = (lambda_n[0] * constraint.friction).abs();
                let max_force = if umg > normal_force {
                    umg
                } else {
                    normal_force
                };

                if constraint.cached_lambda[1] > max_force {
                    constraint.cached_lambda[1] = max_force;
                }
                if constraint.cached_lambda[1] < -max_force {
                    constraint.cached_lambda[1] = -max_force;
                }

                if constraint.cached_lambda[2] > max_force {
                    constraint.cached_lambda[2] = max_force;
                }
                if constraint.cached_lambda[2] < -max_force {
                    constraint.cached_lambda[2] = -max_force;
                }
            }
            lambda_n = constraint.cached_lambda - old_lambda;

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
}

// This cleans up any constraints that are no longer in range
pub fn post_solve(
    mut manifold_arena: ResMut<PenetrationArena>,
    rb_query: Query<(&Transform, &CenterOfMass)>,
) {
    for (pair, manifold) in &mut manifold_arena.manifolds {
        // remove any contacts that have drifted too far
        let mut i = 0;
        if let Ok([(trans_a, com_a), (trans_b, com_b)]) = rb_query.get_many([pair.a, pair.b]) {
            while i < manifold.contacts.len() {
                let contact = manifold.contacts[i];

                // get the tangential distance of the point on a and the point on b
                let a = RBHelper::local_to_world(trans_a, com_a, contact.local_point_a);
                let b = RBHelper::local_to_world(trans_b, com_b, contact.local_point_b);

                let mut normal = manifold.constraints[i].normal();
                normal = trans_a.rotation * normal;

                // calculate the tangential separation and penetration depth
                let ab = b - a;
                let penetration_depth = normal.dot(ab);
                let ab_normal = normal * penetration_depth;
                let ab_tangent = ab - ab_normal;

                // if the tangential displacement is less than a specific threshold, it's okay to keep
                // it.
                const DISTANCE_THRESHOLD: f32 = 0.02;
                if ab_tangent.length_squared() < DISTANCE_THRESHOLD * DISTANCE_THRESHOLD
                    && penetration_depth <= 0.0
                {
                    i += 1;
                    continue;
                }

                // this contact has moved beyond its threshold and should be removed
                manifold.constraints.remove(i);
                manifold.contacts.remove(i);
            }
        }
    }

    // if there are no contacts left, remove the manifold
    manifold_arena
        .manifolds
        .retain(|_pair, manifold| manifold.contacts.len() > 0)
}
