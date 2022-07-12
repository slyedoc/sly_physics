
mod convex_hull;
use bevy::{prelude::*, math::vec3};
use bevy_inspector_egui::Inspectable;
use crate::{Aabb};
pub use convex_hull::*;

// Avoiding Box<dyn Trait> as to try and keep everything on the stack
#[derive(Component, Inspectable, Debug)]
pub enum Collider {
    Sphere { radius: f32 },
    Cuboid { size: Vec3 },
    ConvexHull,
}

impl Default for Collider {
    fn default() -> Self {
        Self::Sphere {
            radius: 0.5,
        }
    }
}

impl Collider {
    pub fn sphere(radius: f32) -> Self {
        Collider::Sphere { radius }
    }

    pub fn cuboid(x: f32, y: f32, z: f32) -> Self {
        Collider::Cuboid { size: vec3(x, y, z) }
    }

    pub fn get_center_of_mass(&self) -> Vec3 {
        match self {
            Collider::Sphere { radius: _ } => vec3(0.0, 0.0, 0.0),
            Collider::Cuboid { size: _,} => vec3(0.0, 0.0, 0.0),
            Collider::ConvexHull => vec3(0.0, 0.0, 0.0),
        }
    }
    
    pub fn get_inertia_tensor(&self) -> Mat3 {
        match self {
            Collider::Sphere { radius } => {
                let i = 2.0 * radius * radius / 5.0;
                Mat3::from_diagonal(Vec3::splat(i) )
            },
            Collider::Cuboid { size: _ } => {
                // inertia tensor for box centered around zero
                let bounds = self.get_aabb();
                let d = bounds.maxs - bounds.mins;
                let dd = d * d;
                let diagonal = Vec3::new(dd.y + dd.z, dd.x + dd.z, dd.x + dd.y) / 12.0;
                let tensor = Mat3::from_diagonal(diagonal);

                // now we need to use the parallel axis theorem to get the ineria tensor for a box that is
                // not centered around the origin

                let cm = (bounds.maxs + bounds.mins) * 0.5;

                // the displacement from the center of mass to the origin
                let r = -cm;
                let r2 = r.length_squared();

                let pat_tensor = Mat3::from_cols(
                    Vec3::new(r2 - r.x * r.x, r.x * r.y, r.x * r.z),
                    Vec3::new(r.y * r.x, r2 - r.y * r.y, r.y * r.z),
                    Vec3::new(r.z * r.x, r.z * r.y, r2 - r.z * r.z),
                );

                // now we need to add the centre of mass tensor and the parallel axis theorem tensor
                // together
                tensor + pat_tensor
            },
            Collider::ConvexHull => {
                todo!()
            },
        }

    }

    pub fn get_aabb(&self) -> Aabb {
        match self {
            Collider::Sphere { radius } => {
                Aabb {
                    mins: Vec3::new(-radius, -radius, -radius),
                    maxs: Vec3::new(*radius, *radius, *radius),
                }
            },
            Collider::Cuboid { size } => {
                let half_size = *size * 0.5;
                Aabb {
                    mins: -half_size,
                    maxs: half_size,
                }
            },
            Collider::ConvexHull => {
                todo!()
            },
        }
    }


    pub fn get_gjk_verts(&self) -> Vec<Vec3> {
        match self {
            Collider::Sphere { radius: _ } => vec![Vec3::ZERO],
            Collider::Cuboid { size } => {
                let half_size = *size * 0.5;
                vec![
                    Vec3::new(-half_size.x, -half_size.y, -half_size.z),
                    Vec3::new(-half_size.x, -half_size.y, half_size.z),
                    Vec3::new(-half_size.x, half_size.y, -half_size.z),
                    Vec3::new(-half_size.x, half_size.y, half_size.z),
                    Vec3::new(half_size.x, -half_size.y, -half_size.z),
                    Vec3::new(half_size.x, -half_size.y, half_size.z),
                    Vec3::new(half_size.x, half_size.y, -half_size.z),
                    Vec3::new(half_size.x, half_size.y, half_size.z),
                ]
            },
            Collider::ConvexHull => unimplemented!(),
        }
    }

    /// Given a set of points, fit an axis oriented bounding box to the vertices by finding the
    /// extents of the mesh.
    pub fn get_aabb_mesh(vertices: &[Vec3]) -> Aabb {        
        let mut maximums = Vec3::new(f32::MIN, f32::MIN, f32::MIN);
        let mut minimums = Vec3::new(f32::MAX, f32::MAX, f32::MAX);
        for vertex in vertices.iter() {
            maximums = vertex.max(maximums);
            minimums = vertex.min(minimums);
        }
        Aabb { mins: minimums, maxs: maximums }
    }

    pub fn fastest_linear_speed(&self, angular_velocity: Vec3, dir: Vec3) -> f32 {
        match self {
            Collider::Sphere { radius: _radius } => {
                0.0
            },
            Collider::Cuboid { size } => {
                let mut max_speed = 0.0;
                let half_size = *size * 0.5;
                for i in 0..8 {
                    let pt = vec3(
                        if i & 1 != 0 { half_size.x } else { -half_size.x },
                        if i & 2 != 0 { half_size.y } else { -half_size.y },
                        if i & 4 != 0 { half_size.z } else { -half_size.z },
                    );
                
                    let r = pt - Vec3::ZERO;
                    let linear_velocity = angular_velocity.cross(r);
                    let speed = dir.dot(linear_velocity);
                    if speed > max_speed {
                        max_speed = speed;
                    }
                }
                max_speed        
            },
            Collider::ConvexHull => {
                todo!()
            },
        }
    }

    pub fn get_support(&self, trans: &Transform, verts: &Vec<Vec3>, dir: Vec3, bias: f32) -> Vec3 {
        match self {
            Collider::Sphere { radius } => {
                trans.translation + dir * (radius + bias)
            },
            Collider::Cuboid { size: _size } => {                
                let mut max_pt = (trans.rotation * verts[0]) + trans.translation;
                let mut max_dist = dir.dot(max_pt);
                for &pt in &verts[1..] {
                    let pt = (trans.rotation * pt) + trans.translation;
                    let dist = dir.dot(pt);
                    if dist > max_dist {
                        max_dist = dist;
                        max_pt = pt;
                    }
                }
            
                let norm = dir.normalize() * bias;
                max_pt + norm
            },
            Collider::ConvexHull => {
                todo!()
            },
        }
        
    }
}


