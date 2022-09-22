use bevy::{asset::HandleId, prelude::*, math::vec3};

use crate::{prelude::Collider, types::Aabb, Bvh, BvhInstance};

use super::BvhTri;

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
    pub colliders: Vec<Bvh>,
}

impl Default for Tlas {
    fn default() -> Self {
        Tlas {
            nodes: Vec::with_capacity(0),
            blas: Default::default(),
            colliders: Default::default(),
        }
    }
}

impl Tlas {
    pub fn add(&mut self, colider_id: HandleId, collider: &Collider, entity: Entity) {
        match collider {
            Collider::Sphere(sphere) => {
                let bvh_mesh = Mesh::from(shape::UVSphere {
                    radius: sphere.radius,
                    sectors: 6,
                    stacks: 6,
                });

                let bvh_tri = parse_bvh_mesh(&bvh_mesh);
                //info!("e: {:?} bvh_tri: {:?}", e, bvh_tri);
                let bvh_index = self.add_bvh(Bvh::new(bvh_tri));
                self.add_instance(BvhInstance::new(entity, bvh_index));
            }
            Collider::Box(b) => {
                let bvh_mesh = Mesh::from(shape::Box::new(b.size.x, b.size.y, b.size.z));
                let bvh_tri = parse_bvh_mesh(&bvh_mesh);
                let bvh_index = self.add_bvh(Bvh::new(bvh_tri));
                self.add_instance(BvhInstance::new(entity, bvh_index));
            }
            Collider::Convex(convex) => {
                let bvh_mesh = Mesh::from(convex);
                let bvh_tri = parse_bvh_mesh(&bvh_mesh);
                let bvh_index = self.add_bvh(Bvh::new(bvh_tri));
                self.add_instance(BvhInstance::new(entity, bvh_index));
            }
        }
    }
    pub fn add_bvh(&mut self, bvh: Bvh) -> usize {
        self.colliders.push(bvh);
        self.colliders.len() - 1
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
            if let Ok((trans, aabb)) = query.get(instance.entity) {
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


// TODO: We don't really want to copy the all tris, find better way
pub fn parse_bvh_mesh(mesh: &Mesh) -> Vec<BvhTri> {
    match mesh.primitive_topology() {
        bevy::render::mesh::PrimitiveTopology::TriangleList => {
            let indexes = match mesh.indices().expect("No Indices") {
                bevy::render::mesh::Indices::U32(vec) => vec,
                _ => todo!(),
            };

            let verts = match mesh
                .attribute(Mesh::ATTRIBUTE_POSITION)
                .expect("No Position Attribute")
            {
                bevy::render::mesh::VertexAttributeValues::Float32x3(vec) => {
                    vec.iter().map(|vec| vec3(vec[0], vec[1], vec[2]))
                }
                _ => todo!(),
            }
            .collect::<Vec<_>>();

            let mut triangles = Vec::with_capacity(indexes.len() / 3);
            for tri_indexes in indexes.chunks(3) {
                let v0 = verts[tri_indexes[0] as usize];
                let v1 = verts[tri_indexes[1] as usize];
                let v2 = verts[tri_indexes[2] as usize];
                triangles.push(BvhTri::new(
                    vec3(v0[0], v0[1], v0[2]),
                    vec3(v1[0], v1[1], v1[2]),
                    vec3(v2[0], v2[1], v2[2]),
                ));
            }
            triangles
        }
        _ => todo!(),
    }
}