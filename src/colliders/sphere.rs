
use crate::{types::*, BOUNDS_EPS};
use bevy::{math::vec3, prelude::*};

use super::ColliderTrait;

#[derive(Debug)]
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

impl ColliderTrait for Sphere {
    fn get_center_of_mass(&self) -> Vec3 {
        self.center_of_mass
    }

    fn get_inertia_tensor(&self) -> Mat3 {
        self.inertia_tensor
    }

    fn get_aabb(&self) -> Aabb {
        self.aabb.clone()
    }

    fn get_world_aabb(
        &self,
        trans: &Transform,
        velocity: &Velocity,
        time: f32,
    ) -> Aabb {
        let mut aabb = Aabb {
            mins: trans.translation - self.radius,
            maxs: trans.translation + self.radius,
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
}
