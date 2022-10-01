mod ray;
mod tlas;
mod tri;

use bevy_inspector_egui::Inspectable;
pub use ray::*;
pub use tlas::*;
pub use tri::*;

use crate::{prelude::Collider, types::Aabb};
use bevy::prelude::*;

#[derive(Default, Inspectable, Debug)]
pub struct BvhNode {
    pub aabb: Aabb,
    pub left_first: u32,
    pub tri_count: u32,
}

impl BvhNode {
    pub fn is_leaf(&self) -> bool {
        self.tri_count > 0
    }

    pub fn calculate_cost(&self) -> f32 {
        let e = self.aabb.maxs - self.aabb.mins; // extent of the node
        let surface_area = e.x * e.y + e.y * e.z + e.z * e.x;
        self.tri_count as f32 * surface_area
    }
}

#[derive(Debug, Inspectable, Default)]
pub struct BvhInstance {
    pub entity: Option<Entity>,
    pub collider: Handle<Collider>,
    pub inv_trans: Mat4,
    pub bounds: Aabb,
}

#[derive(Default, Debug, Clone)]
pub struct Bin {
    pub bounds: Aabb,
    pub tri_count: u32,
}
