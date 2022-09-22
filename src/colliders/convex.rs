use std::mem::swap;

use bevy::{
    math::vec3,
    prelude::*,
    render::mesh::{Indices, PrimitiveTopology},
};

use crate::{
    bvh::{Bin, BvhNode, BvhTri},
    prelude::Ray,
    types::*,
    BOUNDS_EPS, BVH_BIN_COUNT,
};

use super::{fastest_linear_speed, find_support_point, Collidable};

#[derive(Debug)]
pub struct Convex {
    verts: Vec<Vec3>,
    pub mesh: Mesh,
    bounds: Aabb,
    center_of_mass: Vec3,
    inertia_tensor: Mat3,

    pub nodes: Vec<BvhNode>,
    pub bvh_tris: Vec<BvhTri>,
    pub triangle_indexes: Vec<usize>,
}

impl Convex {
    pub fn new(verts: &[Vec3]) -> Self {
        let (hull_points, hull_tris) = build_convex_hull(verts);
        let bounds = Aabb::from_points(&hull_points);
        let centre_of_mass = calculate_center_of_mass(&hull_points, &hull_tris);
        let inertia_tensor = calculate_inertia_tensor(&hull_points, &hull_tris, centre_of_mass);

        // bvh stuff
        let mesh = mesh_from_hull_points(&hull_points, &hull_tris);
        let bvh_tris = parse_bvh_mesh(&mesh);

        let count = bvh_tris.len() as u32;

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

        let triangle_indexes = (0..count as usize).collect::<Vec<_>>();

        let mut convex = Convex {
            verts: hull_points,
            bounds,
            center_of_mass: centre_of_mass,
            inertia_tensor,
            mesh,
            nodes,
            bvh_tris,
            triangle_indexes,
        };

        convex.update_node_bounds(0);
        convex.subdivide_node(0);

        convex
    }

    fn update_node_bounds(&mut self, node_idx: usize) {
        let node = &mut self.nodes[node_idx];
        node.aabb.mins = Vec3::splat(f32::MAX);
        node.aabb.maxs = Vec3::splat(-f32::MAX);
        for i in 0..node.tri_count {
            let leaf_tri_index = self.triangle_indexes[(node.left_first + i) as usize];
            let leaf_tri = self.bvh_tris[leaf_tri_index];
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
            if self.bvh_tris[self.triangle_indexes[i as usize]].centroid[axis] < split_pos {
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
                let triangle =
                    &self.bvh_tris[self.triangle_indexes[(node.left_first + i) as usize]];
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
                let triangle = &self.bvh_tris[self.triangle_indexes[(node.left_first + i) as usize]];
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

fn mesh_from_hull_points(points: &[Vec3], tri_index: &[TriIndexed]) -> Mesh {
    let mut verts: Vec<[f32; 3]> = Vec::with_capacity(points.len() * 3);
    let mut normals: Vec<[f32; 3]> = Vec::with_capacity(points.len() * 3);
    let mut indices: Vec<u32> = Vec::with_capacity(points.len() * 3);

    for (i, tri) in tri_index.iter().enumerate() {
        let a = points[tri.a as usize];
        verts.push(a.into());
        let b = points[tri.b as usize];
        verts.push(b.into());
        let c = points[tri.c as usize];
        verts.push(c.into());

        let ab = b - a;
        let ac = c - a;
        let n = ab.cross(ac).normalize().into();
        normals.push(n);
        normals.push(n);
        normals.push(n);

        let offset = i as u32 * 3;
        indices.push(offset);
        indices.push(offset + 1);
        indices.push(offset + 2);
    }

    let mut mesh = Mesh::new(PrimitiveTopology::TriangleList);
    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, verts);
    mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, normals);
    mesh.set_indices(Some(Indices::U32(indices)));

    // fake some UVs for the default shader
    // let uvs: Vec<[f32; 2]> = std::iter::repeat([0.0; 2])
    //     .take(collider.verts.len() * 3)
    //     .collect();
    // mesh.insert_attribute(Mesh::ATTRIBUTE_UV_0, uvs);
    mesh
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

impl Collidable for Convex {
    fn get_center_of_mass(&self) -> Vec3 {
        self.center_of_mass
    }

    fn get_inertia_tensor(&self) -> Mat3 {
        self.inertia_tensor
    }

    fn get_aabb(&self) -> Aabb {
        self.bounds
    }

    fn get_world_aabb(&self, trans: &Transform, velocity: &Velocity, time: f32) -> Aabb {
        let mut aabb = Aabb::default();
        for pt in &self.verts {
            let pt = (trans.rotation * *pt) + trans.translation;
            aabb.expand_by_point(pt);
        }
        // expand by the linear velocity
        let p1 = aabb.mins + velocity.linear * time;
        aabb.expand_by_point(p1);
        let p2 = aabb.maxs + velocity.linear * time;
        aabb.expand_by_point(p2);

        let p3 = aabb.mins - Vec3::splat(BOUNDS_EPS);
        aabb.expand_by_point(p3);
        let p4 = aabb.maxs + Vec3::splat(BOUNDS_EPS);
        aabb.expand_by_point(p4);

        aabb
    }

    fn fastest_linear_speed(&self, angular_velocity: Vec3, dir: Vec3) -> f32 {
        fastest_linear_speed(&self.verts, angular_velocity, self.center_of_mass, dir)
    }

    fn get_support(&self, trans: &Transform, dir: Vec3, bias: f32) -> Vec3 {
        find_support_point(&self.verts, dir, trans.translation, trans.rotation, bias)
    }

    fn intersect(&self, ray: &mut Ray) -> Option<f32> {
        //pub fn intersect_collider(&mut self, collider: &Bvh, entity: Entity) {
        let mut node = &self.nodes[0];
        let mut stack = Vec::with_capacity(64);
        let mut distance = None;
        loop {
            if node.is_leaf() {
                for i in 0..node.tri_count {
                    let tri_index = self.triangle_indexes[(node.left_first + i) as usize];
                    if let Some(t) = ray.intersect_triangle(&self.bvh_tris[tri_index]) {
                        if distance.is_none() || t < distance.unwrap() {
                            distance = Some(t);
                        }
                    }                        
                }
                if stack.is_empty() {
                    return distance;
                }
                node = stack.pop().unwrap();
                continue;
            }
            let mut child1 = &self.nodes[node.left_first as usize];
            let mut child2 = &self.nodes[(node.left_first + 1) as usize];
            let mut dist1 = ray.intersect_aabb(&child1.aabb);
            let mut dist2 = ray.intersect_aabb(&child2.aabb);
            if dist1 > dist2 {
                swap(&mut dist1, &mut dist2);
                swap(&mut child1, &mut child2);
            }
            if dist1 == f32::MAX {
                if stack.is_empty() {
                    return distance;
                }
                node = stack.pop().unwrap();
            } else {
                node = child1;
                if dist2 != f32::MAX {
                    stack.push(child2);
                }
            }
        }
    }
}

pub fn calculate_center_of_mass(pts: &[Vec3], tris: &[TriIndexed]) -> Vec3 {
    const NUM_SAMPLES: usize = 100;

    let bounds = Aabb::from_points(pts);

    let dv = bounds.width() / NUM_SAMPLES as f32;

    let mut cm = Vec3::ZERO;
    let mut sample_count = 0;

    for i in 0..NUM_SAMPLES {
        let x = bounds.mins.x + dv.x * i as f32;
        for j in 0..NUM_SAMPLES {
            let y = bounds.mins.y + dv.y * j as f32;
            for k in 0..NUM_SAMPLES {
                let z = bounds.mins.z + dv.z * k as f32;
                let pt = Vec3::new(x, y, z);
                if is_external(pts, tris, pt) {
                    continue;
                }

                cm += pt;
                sample_count += 1;
            }
        }
    }

    cm / sample_count as f32
}

fn is_external(pts: &[Vec3], tris: &[TriIndexed], pt: Vec3) -> bool {
    for tri in tris {
        let a = pts[tri.a as usize];
        let b = pts[tri.b as usize];
        let c = pts[tri.c as usize];

        // if the point is in front of any triangle then it's external
        let dist = distance_from_triangle(a, b, c, pt);
        if dist > 0.0 {
            return true;
        }
    }
    false
}

#[derive(Clone, Copy, Debug)]
pub struct TriIndexed {
    pub a: u32,
    pub b: u32,
    pub c: u32,
}

#[derive(Copy, Clone, Debug)]
struct Edge {
    pub a: u32,
    pub b: u32,
}

impl PartialEq for Edge {
    fn eq(&self, other: &Self) -> bool {
        (self.a == other.a && self.b == other.b) || (self.a == other.b && self.b == other.a)
    }
}

impl Eq for Edge {}

fn build_convex_hull(verts: &[Vec3]) -> (Vec<Vec3>, Vec<TriIndexed>) {
    let mut hull_points = Vec::new();
    let mut hull_tris = Vec::new();

    assert!(verts.len() > 3);

    build_tetrahedron(verts, &mut hull_points, &mut hull_tris);

    expand_convex_hull(&mut hull_points, &mut hull_tris, verts);

    (hull_points, hull_tris)
}

fn build_tetrahedron(verts: &[Vec3], hull_points: &mut Vec<Vec3>, hull_tris: &mut Vec<TriIndexed>) {
    hull_points.clear();
    hull_tris.clear();

    let mut point0 = verts[find_point_furthest_in_dir(verts, Vec3::X)];
    let mut point1 = verts[find_point_furthest_in_dir(verts, -point0)];
    let point2 = find_point_furthest_from_line(verts, point0, point1);
    let point3 = find_point_furthest_from_triangle(verts, point0, point1, point2);

    // this is important for making sure the ordering is CCW for all faces
    if distance_from_triangle(point0, point1, point2, point3) > 0.0 {
        std::mem::swap(&mut point0, &mut point1);
    }

    // build the tetrahedron
    hull_points.extend_from_slice(&[point0, point1, point2, point3]);

    hull_tris.extend_from_slice(&[
        TriIndexed { a: 0, b: 1, c: 2 },
        TriIndexed { a: 0, b: 2, c: 3 },
        TriIndexed { a: 2, b: 1, c: 3 },
        TriIndexed { a: 1, b: 0, c: 3 },
    ]);
}

pub fn calculate_inertia_tensor(pts: &[Vec3], tris: &[TriIndexed], cm: Vec3) -> Mat3 {
    const NUM_SAMPLES: usize = 100;

    let bounds = Aabb::from_points(pts);

    let mut tensor = Mat3::ZERO;

    let dv = bounds.width() / NUM_SAMPLES as f32;

    let mut sample_count = 0;

    for i in 0..NUM_SAMPLES {
        let x = bounds.mins.x + dv.x * i as f32;
        for j in 0..NUM_SAMPLES {
            let y = bounds.mins.y + dv.y * j as f32;
            for k in 0..NUM_SAMPLES {
                let z = bounds.mins.z + dv.z * k as f32;
                let mut pt = Vec3::new(x, y, z);
                if is_external(pts, tris, pt) {
                    continue;
                }

                // Get the point relative to the center of mass
                pt -= cm;

                tensor.col_mut(0)[0] += pt.y * pt.y + pt.z * pt.z;
                tensor.col_mut(1)[1] += pt.z * pt.z + pt.x * pt.x;
                tensor.col_mut(2)[2] += pt.x * pt.x + pt.y * pt.y;

                tensor.col_mut(0)[1] += -pt.x * pt.y;
                tensor.col_mut(0)[2] += -pt.x * pt.z;
                tensor.col_mut(1)[2] += -pt.y * pt.z;

                tensor.col_mut(1)[0] += -pt.x * pt.y;
                tensor.col_mut(2)[0] += -pt.x * pt.z;
                tensor.col_mut(2)[1] += -pt.y * pt.z;

                sample_count += 1;
            }
        }
    }

    tensor * (sample_count as f32).recip()
}

fn find_point_furthest_in_dir(pts: &[Vec3], dir: Vec3) -> usize {
    let mut max_idx = 0;
    let mut max_dist = dir.dot(pts[0]);
    for (i, &pt) in pts.iter().enumerate().skip(1) {
        let dist = dir.dot(pt);
        if dist > max_dist {
            max_dist = dist;
            max_idx = i;
        }
    }
    max_idx
}

fn distance_from_line(a: Vec3, b: Vec3, pt: Vec3) -> f32 {
    let ab = (b - a).normalize();
    let ray = pt - a;
    let projection = ab * ray.dot(ab); // project the ray onto ab
    let perpendicular = ray - projection;
    perpendicular.length()
}

fn find_point_furthest_from_line(pts: &[Vec3], a: Vec3, b: Vec3) -> Vec3 {
    // TODO: don't need the index, could track the point
    // TODO: ab is recalculated every time
    let mut max_idx = 0;
    let mut max_dist = distance_from_line(a, b, pts[0]);
    for (i, &pt) in pts.iter().enumerate().skip(1) {
        let dist = distance_from_line(a, b, pt);
        if dist > max_dist {
            max_dist = dist;
            max_idx = i;
        }
    }
    pts[max_idx]
}

fn distance_from_triangle(a: Vec3, b: Vec3, c: Vec3, pt: Vec3) -> f32 {
    let ab = b - a;
    let ac = c - a;
    let normal = ab.cross(ac).normalize();

    let ray = pt - a;
    ray.dot(normal)
}

fn find_point_furthest_from_triangle(pts: &[Vec3], a: Vec3, b: Vec3, c: Vec3) -> Vec3 {
    // TODO: don't need the index, could track the point
    let mut max_idx = 0;
    let mut max_dist = distance_from_triangle(a, b, c, pts[0]);
    for (i, &pt) in pts.iter().enumerate().skip(1) {
        // TODO: triangle normal is recalculated every iteration
        let dist = distance_from_triangle(a, b, c, pt);
        if dist * dist > max_dist * max_dist {
            max_dist = dist;
            max_idx = i;
        }
    }
    pts[max_idx]
}

fn expand_convex_hull(
    hull_points: &mut Vec<Vec3>,
    hull_tris: &mut Vec<TriIndexed>,
    verts: &[Vec3],
) {
    let mut external_verts = Vec::from(verts);
    remove_internal_points(hull_points, hull_tris, &mut external_verts);

    while !external_verts.is_empty() {
        let pt_idx = find_point_furthest_in_dir(&external_verts, external_verts[0]);

        let pt = external_verts[pt_idx];

        // remove this element
        // TODO: could use swap_remove? Is ordering important?
        external_verts.remove(pt_idx);

        add_point(hull_points, hull_tris, pt);

        remove_internal_points(hull_points, hull_tris, &mut external_verts);
    }

    remove_unreferenced_verts(hull_points, hull_tris);
}

fn remove_internal_points(
    hull_points: &[Vec3],
    hull_tris: &[TriIndexed],
    check_pts: &mut Vec<Vec3>,
) {
    // for i in 0..check_pts.len() {
    let mut i = 0;
    while i < check_pts.len() {
        let pt = check_pts[i];

        let mut is_external = false;
        // for t in 0..hull_tris.len() {
        //     let tri = hull_tris[t];
        for tri in hull_tris {
            let a = hull_points[tri.a as usize];
            let b = hull_points[tri.b as usize];
            let c = hull_points[tri.c as usize];

            // if the point is in front of any triangle then it's external
            let dist = distance_from_triangle(a, b, c, pt);
            if dist > 0.0 {
                is_external = true;
                break;
            }
        }

        // if it's not external, then it's inside the polyhedron and should be removed
        if !is_external {
            check_pts.remove(i);
            // i -= 1;
        } else {
            i += 1;
        }
    }

    // also remove any points that are just a little too close to the hull points
    // for i in 0..check_pts.len() {
    let mut i = 0;
    while i < check_pts.len() {
        let pt = check_pts[i];

        let mut is_too_close = false;
        // for j in 0..hull_points.len() {
        //     let hull_pt = hull_points[j];
        for hull_pt in hull_points {
            let ray = *hull_pt - pt;
            if ray.length_squared() < 0.01 * 0.01 {
                // 1cm is too close
                is_too_close = true;
                break;
            }
        }

        if is_too_close {
            check_pts.remove(i);
            // i -= 1;
        } else {
            i += 1;
        }
    }
}

// This will compare the incoming edge with all the edges in the facing tris and then return true
// if it's unique.
fn is_edge_unique(tris: &[TriIndexed], facing_tris: &[u32], ignore_tri: u32, edge: &Edge) -> bool {
    for &tri_idx in facing_tris {
        if ignore_tri == tri_idx {
            continue;
        }

        let tri = tris[tri_idx as usize];

        let edges = [
            Edge { a: tri.a, b: tri.b },
            Edge { a: tri.b, b: tri.c },
            Edge { a: tri.c, b: tri.a },
        ];

        for e in &edges {
            if *edge == *e {
                return false;
            }
        }
    }

    true
}

fn add_point(hull_points: &mut Vec<Vec3>, hull_tris: &mut Vec<TriIndexed>, pt: Vec3) {
    // This point is outside
    // Now ew need to remove old triangles and build new ones

    // Find all the triangles that face this point
    let mut facing_tris = Vec::new();
    for i in (0..hull_tris.len()).rev() {
        let tri = hull_tris[i];
        let a = hull_points[tri.a as usize];
        let b = hull_points[tri.b as usize];
        let c = hull_points[tri.c as usize];

        let dist = distance_from_triangle(a, b, c, pt);
        if dist > 0.0 {
            facing_tris.push(i as u32);
        }
    }

    // Now find all edges that are unique to the tris, these will be the edges that form the new
    // trianges
    let mut unique_edges = Vec::new();
    for tri_idx in &facing_tris {
        let tri = hull_tris[*tri_idx as usize];

        let edges = [
            Edge { a: tri.a, b: tri.b },
            Edge { a: tri.b, b: tri.c },
            Edge { a: tri.c, b: tri.a },
        ];

        for edge in &edges {
            if is_edge_unique(hull_tris, &facing_tris, *tri_idx, edge) {
                unique_edges.push(*edge);
            }
        }
    }

    // now remove the old facing tris
    for tri_idx in &facing_tris {
        hull_tris.remove(*tri_idx as usize);
    }

    // now add the new point
    hull_points.push(pt);
    let new_pt_idx = hull_points.len() as u32 - 1;

    // now add triangles for each unique edge
    for edge in &unique_edges {
        let tri = TriIndexed {
            a: edge.a,
            b: edge.b,
            c: new_pt_idx,
        };
        hull_tris.push(tri);
    }
}

fn remove_unreferenced_verts(hull_points: &mut Vec<Vec3>, hull_tris: &mut [TriIndexed]) {
    // for i in 0..hull_points.len() as u32 {
    let mut i = 0;
    while i < hull_points.len() as u32 {
        let mut is_used = false;
        for tri in hull_tris.iter() {
            if tri.a == i || tri.b == i || tri.c == i {
                is_used = true;
                break;
            }
        }

        if is_used {
            i += 1;
            continue;
        }

        for tri in hull_tris.iter_mut() {
            if tri.a > i {
                tri.a -= 1;
            }
            if tri.b > i {
                tri.b -= 1;
            }
            if tri.c > i {
                tri.c -= 1;
            }
        }

        hull_points.remove(i as usize);
        // i -= 1;
    }
}
