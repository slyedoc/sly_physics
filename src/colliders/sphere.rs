use bevy::{prelude::*, math::vec3};
use crate::types::Aabb;

use super::ColliderTrait;


pub struct SphereCollider { 
    pub radius: f32 
}

impl ColliderTrait for SphereCollider {
    fn get_center_of_mass(&self) -> Vec3 {
        vec3(0.0, 0.0, 0.0)
    }

    fn get_inertia_tensor(&self) -> Mat3 {
        let i = 2.0 * self.radius * self.radius / 5.0;
        Mat3::from_diagonal(Vec3::splat(i) )
    }

    fn get_aabb(&self) -> Aabb {
        Aabb {
            mins: Vec3::new(-self.radius, -self.radius, -self.radius),
            maxs: Vec3::new(self.radius, self.radius, self.radius),
        }
    }

    fn get_support(&self, trans: &Transform, dir: Vec3, bias: f32) -> Vec3 {
        trans.translation + dir * (self.radius + bias)
    }

    fn fastest_linear_speed(&self, _angular_velocity: Vec3, _dir: Vec3) -> f32 {
        0.0
    }
}
