mod convex;
mod cube;
mod sphere;

pub use convex::*;
pub use cube::*;
pub use sphere::*;

use crate::aabb::Aabb;
use crate::types::{Velocity};
use bevy::prelude::*;
use bevy_inspector_egui::Inspectable;


#[derive(Component, Clone, Inspectable, Debug)]
pub enum Collider {
    Sphere(usize),
    Box(usize),
    Convex(usize),
}

impl Default for Collider {
    fn default() -> Self {
        Collider::Sphere(0)
    }
}

impl Collider {
    pub fn index(&self) -> usize {
        match self {
            Collider::Sphere(index) => *index,
            Collider::Box(index) => *index,
            Collider::Convex(index) => *index,
        }
    }
}

pub struct ColliderResources {
    spheres: Vec<SphereCollider>,
    boxes: Vec<BoxCollider>,
    convexs: Vec<ConvexCollider>,
}

impl Default for ColliderResources {
    fn default() -> Self {
        ColliderResources {
            spheres: vec![SphereCollider::new(0.5)], // adding a instance for a default value
            boxes: Vec::new(),
            convexs: Vec::new(),
        }
    }
}

impl ColliderResources {
    pub fn add_sphere(&mut self, radius: f32) -> Collider {
        self.spheres.push(SphereCollider::new(radius));
        Collider::Sphere(self.spheres.len() - 1)
    }

    pub fn get_sphere(&self, index: usize) -> &SphereCollider {
        &self.spheres[index]
    }

    pub fn add_box(&mut self, size: Vec3) -> Collider {
        self.boxes.push(BoxCollider::new(size));
        Collider::Box(self.boxes.len() - 1)
    }

    pub fn get_cube(&self, index: usize) -> &BoxCollider {
        &self.boxes[index]
    }

    pub fn add_convex(&mut self, verts: &[Vec3]) -> Collider {
        self.convexs.push(ConvexCollider::new(verts));
        Collider::Convex(self.convexs.len() - 1)
    }

    pub fn get_convex(&self, index: usize) -> &ConvexCollider {
        &self.convexs[index]
    }
}

pub trait ColliderTrait {
    fn get_center_of_mass(&self) -> Vec3;
    fn get_inertia_tensor(&self) -> Mat3;
    fn get_aabb(&self) -> Aabb;
    fn get_world_aabb(&self,  trans: &Transform, velocity: &Velocity, time: f32) -> Aabb;
    fn get_support(&self, trans: &Transform, dir: Vec3, bias: f32) -> Vec3;
    fn fastest_linear_speed(&self, angular_velocity: Vec3, dir: Vec3) -> f32;
}


// used by cube and convex
fn find_support_point(verts: &[Vec3], dir: Vec3, pos: Vec3, orient: Quat, bias: f32) -> Vec3 {
    // find the point in the furthest in direction
    let mut max_pt = (orient * verts[0]) + pos;
    let mut max_dist = dir.dot(max_pt);
    for &pt in &verts[1..] {
        let pt = (orient * pt) + pos;
        let dist = dir.dot(pt);
        if dist > max_dist {
            max_dist = dist;
            max_pt = pt;
        }
    }

    let norm = dir.normalize() * bias;

    max_pt + norm
}

// used by cube and convex
fn fastest_linear_speed(verts: &[Vec3], angular_velocity: Vec3, center_of_mass: Vec3, dir: Vec3) -> f32 {
    let mut max_speed = 0.0;
    for pt in verts {
        let r = *pt - center_of_mass;
        let linear_velocity = angular_velocity.cross(r);
        let speed = dir.dot(linear_velocity);
        if speed > max_speed {
            max_speed = speed;
        }
    }
    max_speed
}

