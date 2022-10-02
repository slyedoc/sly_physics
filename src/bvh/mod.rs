mod ray;
mod tri;

use bevy_inspector_egui::Inspectable;
pub use ray::*;
pub use tri::*;

use crate::{types::Aabb, TlasQuery};
use bevy::prelude::*;


use enum_dispatch::enum_dispatch;

#[enum_dispatch(TlasNode)]
pub trait TlasNodeTrait {    
    fn get_aabb<'a> (& 'a self, query: & 'a Query<TlasQuery>) -> &Aabb;
}

#[derive(Debug, Inspectable)]
#[enum_dispatch]
pub enum TlasNode {
    Leaf(Leaf),
    Trunk(Trunk),
}

#[derive(Default, Inspectable, Debug)]
pub struct Leaf {
    pub entity: Option<Entity>,
}

impl TlasNodeTrait for Leaf {
    fn get_aabb<'a>(& 'a self, query: & 'a Query<TlasQuery>) -> &Aabb {
        query.get(self.entity.unwrap()).unwrap().aabb        
    }
}

impl Default for TlasNode {
    fn default() -> Self {
        Self::Leaf( Leaf { entity: None } )
    }
}

#[derive(Default, Debug, Inspectable, Copy, Clone)]
pub struct Trunk {
    pub aabb: Aabb,
    pub left: u16,
    pub right: u16,
}

impl TlasNodeTrait for Trunk {
    fn get_aabb(&self, _query: &Query<TlasQuery>) -> &Aabb {
        &self.aabb
    }
}

#[test]
fn test_size() {
    let s1 =  std::mem::size_of::<TlasNode>();
    //let s2 =  std::mem::size_of::<TlasNode2>();

    info!("TLAS Node size: {}", s1);
    //info!("TLAS Node2 size: {}", s2);
    assert_eq!(s1, 32 );
}

#[derive(Debug, Default, Inspectable)]
pub struct Tlas {
    pub nodes: Vec<TlasNode>,
}

impl Tlas {
    pub fn find_best_match(&self, list: &[u32], n: i32, a: i32, query: &Query<TlasQuery>) -> i32 {
        let mut smallest = f32::MAX;
        let mut best_b = -1i32;
        for b in 0..n {
            if b != a {
                let aabb_a = self.nodes[list[a as usize] as usize].get_aabb(query);
                let aabb_b = self.nodes[list[b as usize] as usize].get_aabb(query);
                let best_max = aabb_a
                    .maxs
                    .max(aabb_b.maxs);
                let best_min = aabb_a.mins.min(aabb_b.mins);
                let e = best_max - best_min;
                let surface_area = e.x * e.y + e.y * e.z + e.z * e.x;
                if surface_area < smallest {
                    smallest = surface_area;
                    best_b = b;
                }
            }
        }
        best_b
    }
}


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

#[derive(Default, Debug, Clone)]
pub struct Bin {
    pub bounds: Aabb,
    pub tri_count: u32,
}
