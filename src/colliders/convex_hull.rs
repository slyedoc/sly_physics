use bevy::{prelude::*, render::mesh::{PrimitiveTopology, Indices}};

use crate::types::Aabb;


pub fn create_mesh_from_verts(verts: &[Vec3]) -> Mesh {

    let (mut hull_pts, mut hull_tris) = build_convex_hull(&verts);
    
    // calculate smoothed normals
    // TODO: Could use map?
    let mut normals: Vec<[f32; 3]> = Vec::with_capacity(hull_pts.len());
    for i in 0..(hull_pts.len() as u32) {
        // TODO: Could use sum?
        let mut n = Vec3::ZERO;
        for tri in hull_tris {
            if i != tri.a && i != tri.b && i != tri.c {
                continue;
            }

            let a = hull_pts[tri.a as usize];
            let b = hull_pts[tri.b as usize];
            let c = hull_pts[tri.c as usize];

            let ab = b - a;
            let ac = c - a;
            n += ab.cross(ac);
        }

        normals.push(n.normalize().into());
    }

    // TODO: Could add `From<&Vec3>` to help here or `to_array`
    let positions: Vec<[f32; 3]> = hull_pts.iter().map(|pt| (*pt).into()).collect();

    // TODO: Could use flat_map?
    let mut indices = Vec::with_capacity(hull_tris.len() * 3);
    for tri in hull_tris {
        indices.push(tri.a);
        indices.push(tri.b);
        indices.push(tri.c);
    }

    let mut mesh = Mesh::new(PrimitiveTopology::TriangleList);
    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions);
    mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, normals);
    mesh.set_indices(Some(Indices::U32(indices)));

    // fake some UVs for the default shader
    let uvs: Vec<[f32; 2]> = std::iter::repeat([0.0; 2]).take(hull_pts.len()).collect();
    mesh.insert_attribute(Mesh::ATTRIBUTE_UV_0, uvs);

    mesh
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
pub struct  TriIndexed {
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

pub fn build_convex_hull(verts: &[Vec3]) -> (Vec<Vec3>, Vec<TriIndexed>) {
    
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

fn expand_convex_hull(hull_points: &mut Vec<Vec3>, hull_tris: &mut Vec<TriIndexed>, verts: &[Vec3]) {
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

fn remove_internal_points(hull_points: &[Vec3], hull_tris: &[TriIndexed], check_pts: &mut Vec<Vec3>) {
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

fn remove_unreferenced_verts(hull_points: &mut Vec<Vec3>, hull_tris: &mut Vec<TriIndexed>) {
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