use crate::{types::Aabb, BvhInstance};

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
}

impl Default for Tlas {
    fn default() -> Self {
        Tlas {
            nodes: Vec::with_capacity(0),
            blas: Default::default(),
        }
    }
}

impl Tlas {

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
}
