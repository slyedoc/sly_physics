
use bevy::{prelude::*, utils::HashMap};

use crate::{Contact, CenterOfMass, MAX_MANIFOLD_CONTACTS, RBHelper};

use super::{ConstraintPenetration};

#[derive(Debug, Default)]
pub struct ManifoldArena {
    pub manifolds: HashMap<(Entity, Entity), Manifold>,
}

impl ManifoldArena {
    pub fn add_contact(
        &mut self,
        contact: Contact,
        trans_a: &Transform,
        com_a: &CenterOfMass,
        trans_b: &Transform,
        com_b: &CenterOfMass,
    ) {
        if let Some(m) = self.manifolds.get_mut(&(contact.a, contact.b)) {
            m.add_contact(trans_a, com_a, trans_b, com_b, contact);
        } else {
            let mut m = Manifold::default();
            m.add_contact(trans_a, com_a, trans_b, com_b, contact);
            self.manifolds.insert((contact.a, contact.b), m);
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
        // TODO: Need to make sure the contact's body_a and body_b are of the correct order
        // 

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
        let index = if self.contacts.len() == MAX_MANIFOLD_CONTACTS  {
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
        let constraint = ConstraintPenetration::new(contact.local_point_a, contact.local_point_b, normal);
        if index < self.constraints.len() {
            self.constraints[index] = constraint;
        } else {
            self.constraints.push(constraint);
        }        
    }


}
