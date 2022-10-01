use crate::{prelude::Ray, types::*, BOUNDS_EPS};
use bevy::{math::vec3, prelude::*};
use bevy_inspector_egui::Inspectable;

use super::Collidable;

#[derive(Debug, Inspectable)]
pub struct Sphere {
    pub radius: f32,
    aabb: Aabb,
    center_of_mass: Vec3,
    inertia_tensor: Mat3,
}

impl Default for Sphere {
    fn default() -> Self {
        Self::new(0.5)
    }
}

impl Sphere {
    pub fn new(radius: f32) -> Self {
        let aabb = Aabb {
            mins: vec3(-radius, -radius, -radius),
            maxs: vec3(radius, radius, radius),
        };

        let i = 2.0 * radius * radius / 5.0;
        let inertia_tensor = Mat3::from_diagonal(Vec3::splat(i));

        Sphere {
            radius,
            aabb,
            center_of_mass: vec3(0.0, 0.0, 0.0),
            inertia_tensor,
        }
    }
}

impl Collidable for Sphere {
    fn get_center_of_mass(&self) -> Vec3 {
        self.center_of_mass
    }

    fn get_inertia_tensor(&self) -> Mat3 {
        self.inertia_tensor
    }

    fn get_aabb(&self) -> Aabb {
        self.aabb
    }

    fn get_world_aabb(&self, trans: &GlobalTransform, velocity: &Velocity, time: f32) -> Aabb {
        let translation = trans.translation();
        let mut aabb = Aabb {
            mins: translation - self.radius,
            maxs: translation + self.radius,
        };

        // expand by the linear velocity
        let p1 = aabb.mins + velocity.linear * time;
        aabb.expand_by_point(p1);
        let p2 = aabb.maxs + velocity.linear * time;
        aabb.expand_by_point(p2);

        // ex

        let p3 = aabb.mins - Vec3::splat(BOUNDS_EPS);
        aabb.expand_by_point(p3);
        let p4 = aabb.maxs + Vec3::splat(BOUNDS_EPS);
        aabb.expand_by_point(p4);

        aabb
    }

    fn get_support(&self, trans: &Transform, dir: Vec3, bias: f32) -> Vec3 {
        trans.translation + dir * (self.radius + bias)
    }

    fn fastest_linear_speed(&self, _angular_velocity: Vec3, _dir: Vec3) -> f32 {
        0.0
    }

    // Returns distance at which ray would hit the sphere, or None if it doesn't hit
    fn intersect(&self, ray: &mut Ray) -> Option<f32> {
        let sphere_to_ray = ray.origin;
        let a = ray.direction.dot(ray.direction);
        let b = 2.0 * ray.direction.dot(ray.origin);
        let c = sphere_to_ray.dot(sphere_to_ray) - self.radius * self.radius;

        let discriminant = b * b - 4.0 * a * c;
        if discriminant < 0.0 {
            return None;
        } else {
            let t1 = (-b - discriminant.sqrt()) / (2.0 * a);
            let t2 = (-b + discriminant.sqrt()) / (2.0 * a);

            if t1 >= 0.0 && t2 >= 0.0 {
                return Some(t1.min(t2));
            } else if t1 >= 0.0 {
                return Some(t1);
            } else if t2 >= 0.0 {
                return Some(t2);
            }
        }
        None
    }
}
