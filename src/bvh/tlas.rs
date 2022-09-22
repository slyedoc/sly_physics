use bevy::{prelude::*};

use crate::{types::Aabb, Bvh, BvhInstance};

#[derive(Default, Debug, Copy, Clone)]
pub struct TlasNode {
    pub aabb: Aabb,
    pub left_right: u32, // 2x16 bits
    pub blas: u32,
}

impl TlasNode {
    #[inline]
    pub fn left(&self) -> usize {
        (self.left_right & 0xffff) as usize
    }

    #[inline]
    pub fn right(&self) -> usize {
        (self.left_right >> 16) as usize
    }
    #[inline]
    pub fn is_leaf(&self) -> bool {
        self.left_right == 0
    }
}

#[derive(Debug)]
pub struct Tlas {
    pub nodes: Vec<TlasNode>,
    pub blas: Vec<BvhInstance>,
    pub bounding_volumes: Vec<Bvh>,
}

impl Default for Tlas {
    fn default() -> Self {
        Tlas {
            nodes: Vec::with_capacity(0),
            blas: Default::default(),
            bounding_volumes: Default::default(),
        }
    }
}

impl Tlas {
    pub fn add_bvh(&mut self, bvh: Bvh) -> usize {
        self.bounding_volumes.push(bvh);
        self.bounding_volumes.len() - 1
    }

    pub fn add_instance(&mut self, instance: BvhInstance) {
        self.blas.push(instance);
    }

    pub fn build(&mut self) {
        self.nodes.clear();
        self.nodes.reserve(self.blas.len() + 1);
        // reserve root node
        self.nodes.push(TlasNode::default());

        let mut node_index = vec![0u32; self.blas.len() + 1];
        let mut node_indices = self.blas.len() as i32;

        // assign a TLASleaf node to each BLAS
        // and index
        for (i, b) in self.blas.iter().enumerate() {
            node_index[i] = i as u32 + 1;
            self.nodes.push(TlasNode {
                aabb: b.bounds,
                left_right: 0, // is leaf
                blas: i as u32,
            });
        }

        // use agglomerative clustering to build the TLAS
        let mut a = 0i32;
        let mut b = self.find_best_match(&node_index, node_indices, a);
        while node_indices > 1 {
            let c = self.find_best_match(&node_index, node_indices, b);
            if a == c {
                let node_index_a = node_index[a as usize];
                let node_index_b = node_index[b as usize];
                let node_a = &self.nodes[node_index_a as usize];
                let node_b = &self.nodes[node_index_b as usize];
                self.nodes.push(TlasNode {
                    aabb: Aabb {
                        mins: node_a.aabb.mins.min(node_b.aabb.mins),
                        maxs: node_a.aabb.maxs.max(node_b.aabb.maxs),
                    },
                    left_right: node_index_a + (node_index_b << 16),
                    blas: 0,
                });
                node_index[a as usize] = self.nodes.len() as u32 - 1;
                node_index[b as usize] = node_index[node_indices as usize - 1];
                node_indices -= 1;
                b = self.find_best_match(&node_index, node_indices, a);
            } else {
                a = b;
                b = c;
            }
        }
        self.nodes[0] = self.nodes[node_index[a as usize] as usize];
    }

    pub fn find_best_match(&self, list: &[u32], n: i32, a: i32) -> i32 {
        let mut smallest = f32::MAX;
        let mut best_b = -1i32;
        for b in 0..n {
            if b != a {
                let best_max = self.nodes[list[a as usize] as usize]
                    .aabb
                    .maxs
                    .max(self.nodes[list[b as usize] as usize].aabb.maxs);
                let best_min = self.nodes[list[a as usize] as usize]
                    .aabb
                    .mins
                    .min(self.nodes[list[b as usize] as usize].aabb.mins);
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

    pub fn update_bvh_instances(&mut self, query: &Query<(&Transform, &Aabb)>) {
        let mut remove_list = Vec::new();
        for instance in &mut self.blas {
            if let Ok( (trans, aabb) ) = query.get(instance.entity) {
                let trans_matrix = trans.compute_matrix();
                instance.inv_trans = trans_matrix.inverse();
                instance.bounds = *aabb;

            } else {
                remove_list.push(instance.entity);
            }
        }

        //remove any not found
        self.blas.retain(|b| !remove_list.contains(&b.entity))
    }
}
