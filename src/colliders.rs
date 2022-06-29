
use bevy::{prelude::*, math::vec3};
use bevy_inspector_egui::Inspectable;
use crate::Aabb;

#[derive(Component, Inspectable, Debug)]
pub enum Collider {
    Sphere { radius: f32 },
    Cuboid { size: Vec3 },
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
            Collider::Cuboid { size: _ } => vec3(0.0, 0.0, 0.0),
        }
    }

    pub fn get_inertia_tensor(&self) -> Mat3 {
        match self {
            Collider::Sphere { radius } => {
                let i = 2.0 * radius * radius / 5.0;
                Mat3::from_diagonal(Vec3::splat(i) )
            },
            Collider::Cuboid { size: _ } => todo!(),
        }

    }

    pub fn get_aabb(&self) -> Aabb {
        match self {
            Collider::Sphere { radius } => {
                Aabb {
                    minimums: Vec3::new(-radius, -radius, -radius),
                    maximums: Vec3::new(*radius, *radius, *radius),
                }
            },
            Collider::Cuboid { size: _ } => todo!(),
        }

    }
}
