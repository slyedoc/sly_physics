mod convex;
mod cube;
mod sphere;

pub use convex::*;
pub use cube::*;
pub use sphere::*;

use crate::Aabb;
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

pub struct ColliderResources {
    spheres: Vec<SphereCollider>,
    boxes: Vec<BoxCollider>,
    convexs: Vec<ConvexCollider>,
}

impl Default for ColliderResources {
    fn default() -> Self {
        ColliderResources {
            spheres: vec![SphereCollider { radius: 0.5 }], // adding a instance for a default value
            boxes: Vec::new(),
            convexs: Vec::new(),
        }
    }
}

impl ColliderResources {
    pub fn add_sphere(&mut self, radius: f32) -> Collider {
        self.spheres.push(SphereCollider { radius });
        Collider::Sphere(self.spheres.len() - 1)
    }

    pub fn get_sphere(&self, index: usize) -> &SphereCollider {
        &self.spheres[index]
    }

    pub fn add_box(&mut self, size: Vec3) -> Collider {
        self.boxes.push(BoxCollider::new(size));
        Collider::Box(self.boxes.len() - 1)
    }

    pub fn get_box(&self, index: usize) -> &BoxCollider {
        &self.boxes[index]
    }

    pub fn add_convex(&mut self, verts: &[Vec3]) -> Collider {
        self.convexs.push(ConvexCollider::new(verts));
        Collider::Box(self.boxes.len() - 1)
    }

    pub fn get_convex(&self, index: usize) -> &ConvexCollider {
        &self.convexs[index]
    }
}

pub trait ColliderTrait {
    fn get_center_of_mass(&self) -> Vec3;
    fn get_inertia_tensor(&self) -> Mat3;
    fn get_aabb(&self) -> Aabb;
    fn get_support(&self, trans: &Transform, dir: Vec3, bias: f32) -> Vec3;
    fn fastest_linear_speed(&self, angular_velocity: Vec3, dir: Vec3) -> f32;
}
