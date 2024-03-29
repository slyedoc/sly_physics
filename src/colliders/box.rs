use bevy::{math::vec3, prelude::*};
use bevy_inspector_egui::Inspectable;

use crate::{prelude::Ray, types::*, BOUNDS_EPS};

use super::{fastest_linear_speed, find_support_point, Collidable};

#[derive(Debug, Inspectable)]
pub struct Box {
    pub size: Vec3,
    pub verts: Vec<Vec3>,
    pub center_of_mass: Vec3,
    pub aabb: Aabb,
    pub inertia_tensor: Mat3,
}
impl Default for Box {
    fn default() -> Self {
        Self::new(vec3(1.0, 1.0, 1.0))
    }
}

impl Box {
    pub fn new(size: Vec3) -> Self {
        let half_size = size * 0.5;
        let aabb = Aabb {
            mins: -half_size,
            maxs: half_size,
        };

        // inertia tensor for box centered around zero
        let d = aabb.maxs - aabb.mins;
        let dd = d * d;
        let diagonal = Vec3::new(dd.y + dd.z, dd.x + dd.z, dd.x + dd.y) / 12.0;
        let tensor = Mat3::from_diagonal(diagonal);

        // now we need to use the parallel axis theorem to get the ineria tensor for a box that is
        // not centered around the origin

        let cm = (aabb.maxs + aabb.mins) * 0.5;

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
        let inertia_tensor = tensor + pat_tensor;

        Box {
            size,
            verts: vec![
                Vec3::new(-half_size.x, -half_size.y, -half_size.z),
                Vec3::new(-half_size.x, -half_size.y, half_size.z),
                Vec3::new(-half_size.x, half_size.y, -half_size.z),
                Vec3::new(-half_size.x, half_size.y, half_size.z),
                Vec3::new(half_size.x, -half_size.y, -half_size.z),
                Vec3::new(half_size.x, -half_size.y, half_size.z),
                Vec3::new(half_size.x, half_size.y, -half_size.z),
                Vec3::new(half_size.x, half_size.y, half_size.z),
            ],
            center_of_mass: vec3(0.0, 0.0, 0.0),
            aabb,
            inertia_tensor,
        }
    }
}

impl Collidable for Box {
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
        let mut aabb = Aabb::default();
        for pt in &self.verts {
            aabb.expand_by_point(trans.mul_vec3(*pt));
        }
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
        find_support_point(&self.verts, dir, trans.translation, trans.rotation, bias)
    }

    fn fastest_linear_speed(&self, angular_velocity: Vec3, dir: Vec3) -> f32 {
        fastest_linear_speed(&self.verts, angular_velocity, self.center_of_mass, dir)
    }
    // Returns distance at which ray would hit the sphere, or None if it doesn't hit
    fn intersect(&self, ray: &mut Ray) -> Option<f32> {
        let mut tmin = (self.aabb.mins.x - ray.origin.x) / ray.direction.x;
        let mut tmax = (self.aabb.maxs.x - ray.origin.x) / ray.direction.x;

        if tmin > tmax {
            std::mem::swap(&mut tmin, &mut tmax);
        }

        let mut tymin = (self.aabb.mins.y - ray.origin.y) / ray.direction.y;
        let mut tymax = (self.aabb.maxs.y - ray.origin.y) / ray.direction.y;

        if tymin > tymax {
            std::mem::swap(&mut tymin, &mut tymax);
        }

        if (tmin > tymax) || (tymin > tmax) {
            return None;
        }

        if tymin > tmin {
            tmin = tymin;
        }

        if tymax < tmax {
            tmax = tymax;
        }

        let mut tzmin = (self.aabb.mins.z - ray.origin.z) / ray.direction.z;
        let mut tzmax = (self.aabb.maxs.z - ray.origin.z) / ray.direction.z;

        if tzmin > tzmax {
            std::mem::swap(&mut tzmin, &mut tzmax);
        }

        if (tmin > tzmax) || (tzmin > tmax) {
            return None;
        }

        if tzmin > tmin {
            tmin = tzmin;
        }

        if tzmax < tmax {
            tmax = tzmax;
        }

        if tmax < 0.0 {
            return None;
        }

        if tmin < 0.0 {
            return Some(tmax);
        }

        Some(tmin)
    }
}
