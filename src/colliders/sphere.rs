use bevy::{prelude::*, math::vec3};
use crate::types::Aabb;

use super::ColliderTrait;


pub struct SphereCollider { 
    pub radius: f32,
    aabb: Aabb,
    center_of_mass: Vec3,
    inertia_tensor: Mat3,
}

impl SphereCollider {
    pub fn new(radius: f32) -> Self {
        let aabb = Aabb {
            mins: vec3(-radius, -radius, -radius),
            maxs: vec3(radius, radius, radius),
        };

        let i = 2.0 * radius * radius / 5.0;
        let inertia_tensor = Mat3::from_diagonal(Vec3::splat(i));

        SphereCollider {
            radius,
            aabb,
            center_of_mass: vec3(0.0, 0.0, 0.0),
            inertia_tensor,
        }
    }
}


impl ColliderTrait for SphereCollider {
    fn get_center_of_mass(&self) -> Vec3 {
        self.center_of_mass
    }

    fn get_inertia_tensor(&self) -> Mat3 {
        self.inertia_tensor
    }

    fn get_aabb(&self) -> Aabb {
        self.aabb
    }

    fn get_world_aabb(&self,  trans: &Transform) -> Aabb {
            Aabb {
                mins: trans.translation - self.radius,
                maxs: trans.translation + self.radius,
            }
    }

    fn get_support(&self, trans: &Transform, dir: Vec3, bias: f32) -> Vec3 {
        trans.translation + dir * (self.radius + bias)
    }

    fn fastest_linear_speed(&self, _angular_velocity: Vec3, _dir: Vec3) -> f32 {
        0.0
    }
}
