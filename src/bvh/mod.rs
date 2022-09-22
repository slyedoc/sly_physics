mod tlas;
mod tri;
mod ray;

pub use tlas::*;
pub use tri::*;
pub use ray::*;

use crate::{types::Aabb, BVH_BIN_COUNT};
use bevy::{ prelude::*, reflect::TypeUuid};

#[derive(Default, Debug)]
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

#[derive(Debug)]
pub struct BvhInstance {
    pub entity: Entity,
    pub bvh_index: usize,
    pub inv_trans: Mat4,
    pub bounds: Aabb,
}

impl BvhInstance {
    pub fn new(entity: Entity, bvh_index: usize) -> Self {
        Self {
            entity,
            bvh_index,
            inv_trans: Mat4::default(),
            bounds: Aabb::default(),
        }
    }
}

// TODO: Set this up like I did colliders
#[derive(Default, Component, Debug, TypeUuid)]
#[uuid = "81299f9d-41e0-4ff0-86b7-6bef6c3f67c1"]
pub struct Bvh {
    pub nodes: Vec<BvhNode>,
    pub tris: Vec<BvhTri>,
    pub triangle_indexes: Vec<usize>,
}

impl Bvh {
    // TODO: for now bvh get a copy of there down tris, this allows tlas to self contained
    // allowing for tlas intersection to used with any other resources, and no need for events
    // This last part is the largest reason for keeping it like this, not needing to wait on a
    // raycast event makes using it more friendly
    pub fn new(triangles: Vec<BvhTri>) -> Bvh {
        assert!(!triangles.is_empty());
        let count = triangles.len() as u32;
        let mut bvh = Bvh {
            tris: triangles,
            nodes: {
                // Add root node
                let mut nodes = Vec::with_capacity(64);
                nodes.push(BvhNode {
                    left_first: 0,
                    tri_count: count,
                    aabb: Aabb::default(),
                });
                // add empty node to offset reset of the vec by 1, so left 
                nodes.push(BvhNode {
                    left_first: 0,
                    tri_count: 0,
                    aabb: Aabb::default(),
                });
                nodes
            },
            triangle_indexes: (0..count as usize).collect::<Vec<_>>(),
        };

        bvh.update_node_bounds(0);
        bvh.subdivide_node(0);
        bvh
    }

    // pub fn refit(&mut self, triangles: &[Tri]) {
    //     for i in (0..(self.open_node - 1)).rev() {
    //         if i != 1 {
    //             let node = &mut self.nodes[i];
    //             if node.is_leaf() {
    //                 // leaf node: adjust bounds to contained triangles
    //                 self.update_node_bounds(i, triangles);
    //                 continue;
    //             }
    //             // interior node: adjust bounds to child node bounds

    //             let leftChild = &self.nodes[node.left_first as usize];
    //             let rightChild = &self.nodes[(node.left_first + 1) as usize];

    //             node.aabb_min = leftChild.aabb_min.min(rightChild.aabb_min);
    //             node.aabb_max = leftChild.aabb_max.max(rightChild.aabb_max);
    //         }
    //     }
    // }

    fn update_node_bounds(&mut self, node_idx: usize) {
        let node = &mut self.nodes[node_idx];
        node.aabb.mins = Vec3::splat(f32::MAX);
        node.aabb.maxs = Vec3::splat(-f32::MAX);
        for i in 0..node.tri_count {
            let leaf_tri_index = self.triangle_indexes[(node.left_first + i) as usize];
            let leaf_tri = self.tris[leaf_tri_index];
            node.aabb.mins = node.aabb.mins.min(leaf_tri.vertex0);
            node.aabb.mins = node.aabb.mins.min(leaf_tri.vertex1);
            node.aabb.mins = node.aabb.mins.min(leaf_tri.vertex2);
            node.aabb.maxs = node.aabb.maxs.max(leaf_tri.vertex0);
            node.aabb.maxs = node.aabb.maxs.max(leaf_tri.vertex1);
            node.aabb.maxs = node.aabb.maxs.max(leaf_tri.vertex2);
        }
    }

    fn subdivide_node(&mut self, node_idx: usize) {
        let node = &self.nodes[node_idx];

        // determine split axis using SAH
        let (axis, split_pos, split_cost) = self.find_best_split_plane(node);
        let no_split_cost = node.calculate_cost();
        if split_cost >= no_split_cost {
            return;
        }

        // in-place partition
        let mut i = node.left_first;
        let mut j = i + node.tri_count - 1;
        while i <= j {
            if self.tris[self.triangle_indexes[i as usize]].centroid[axis] < split_pos {
                i += 1;
            } else {
                self.triangle_indexes.swap(i as usize, j as usize);
                j -= 1;
            }
        }

        // abort split if one of the sides is empty
        let left_count = i - node.left_first;
        if left_count == 0 || left_count == node.tri_count {
            return;
        }

        // create child nodes
        self.nodes.push(BvhNode::default());
        let left_child_idx = self.nodes.len() as u32 - 1;
        self.nodes.push(BvhNode::default());
        let right_child_idx = self.nodes.len() as u32 - 1;

        self.nodes[left_child_idx as usize].left_first = self.nodes[node_idx].left_first;
        self.nodes[left_child_idx as usize].tri_count = left_count;
        self.nodes[right_child_idx as usize].left_first = i;
        self.nodes[right_child_idx as usize].tri_count =
            self.nodes[node_idx].tri_count - left_count;

        self.nodes[node_idx].left_first = left_child_idx;
        self.nodes[node_idx].tri_count = 0;

        self.update_node_bounds(left_child_idx as usize);
        self.update_node_bounds(right_child_idx as usize);
        // recurse
        self.subdivide_node(left_child_idx as usize);
        self.subdivide_node(right_child_idx as usize);
    }

    fn find_best_split_plane(&self, node: &BvhNode) -> (usize, f32, f32) {
        // determine split axis using SAH
        let mut best_axis = 0;
        let mut split_pos = 0.0f32;
        let mut best_cost = f32::MAX;

        for a in 0..3 {
            let mut bounds_min = f32::MAX;
            let mut bounds_max = -f32::MAX;
            for i in 0..node.tri_count {
                let triangle = &self.tris[self.triangle_indexes[(node.left_first + i) as usize]];
                bounds_min = bounds_min.min(triangle.centroid[a]);
                bounds_max = bounds_max.max(triangle.centroid[a]);
            }
            if bounds_min == bounds_max {
                continue;
            }
            // populate bins
            let mut bin = vec![Bin::default(); BVH_BIN_COUNT];
            let mut scale = BVH_BIN_COUNT as f32 / (bounds_max - bounds_min);
            for i in 0..node.tri_count {
                let triangle = &self.tris[self.triangle_indexes[(node.left_first + i) as usize]];
                let bin_idx =
                    (BVH_BIN_COUNT - 1).min(((triangle.centroid[a] - bounds_min) * scale) as usize);
                bin[bin_idx].tri_count += 1;
                bin[bin_idx].bounds.grow(triangle.vertex0);
                bin[bin_idx].bounds.grow(triangle.vertex1);
                bin[bin_idx].bounds.grow(triangle.vertex2);
            }

            // gather data for the BINS - 1 planes between the bins
            let mut left_area = [0.0f32; BVH_BIN_COUNT - 1];
            let mut right_area = [0.0f32; BVH_BIN_COUNT - 1];
            let mut left_count = [0u32; BVH_BIN_COUNT - 1];
            let mut right_count = [0u32; BVH_BIN_COUNT - 1];
            let mut left_box = Aabb::default();
            let mut right_box = Aabb::default();
            let mut left_sum = 0u32;
            let mut right_sum = 0u32;
            for i in 0..(BVH_BIN_COUNT - 1) {
                left_sum += bin[i].tri_count;
                left_count[i] = left_sum;
                left_box.grow_aabb(&bin[i].bounds);
                left_area[i] = left_box.area();
                right_sum += bin[BVH_BIN_COUNT - 1 - i].tri_count;
                right_count[BVH_BIN_COUNT - 2 - i] = right_sum;
                right_box.grow_aabb(&bin[BVH_BIN_COUNT - 1 - i].bounds);
                right_area[BVH_BIN_COUNT - 2 - i] = right_box.area();
            }

            // calculate SAH cost for the 7 planes
            scale = (bounds_max - bounds_min) / BVH_BIN_COUNT as f32;
            for i in 0..BVH_BIN_COUNT - 1 {
                let plane_cost =
                    left_count[i] as f32 * left_area[i] + right_count[i] as f32 * right_area[i];
                if plane_cost < best_cost {
                    best_axis = a;
                    split_pos = bounds_min + scale * (i + 1) as f32;
                    best_cost = plane_cost;
                }
            }
        }
        (best_axis, split_pos, best_cost)
    }
}

#[derive(Default, Debug, Clone)]
struct Bin {
    bounds: Aabb,
    tri_count: u32,
}
