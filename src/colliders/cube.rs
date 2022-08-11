use bevy::{prelude::*, math::vec3};

use crate::types::Aabb;

use super::ColliderTrait;

#[derive(Debug)]
pub struct BoxCollider { 
    pub size: Vec3,
    pub verts: Vec<Vec3>
}

impl BoxCollider {
    pub fn new(size: Vec3) -> Self {
        let half_size = size * 0.5;
        BoxCollider {
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
            ]
        }
    }
}

impl ColliderTrait for BoxCollider {

    fn get_center_of_mass(&self) -> Vec3 {
        vec3(0.0, 0.0, 0.0)
    }

    fn get_inertia_tensor(&self) -> Mat3 {
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
    }

    fn get_aabb(&self) -> Aabb {
        let half_size = self.size * 0.5;
        Aabb {
            mins: -half_size,
            maxs: half_size,
        }
    }

    fn get_support(&self, trans: &Transform,  dir: Vec3, bias: f32) -> Vec3 {
        let mut max_pt = (trans.rotation * self.verts[0]) + trans.translation;
        let mut max_dist = dir.dot(max_pt);
        for &pt in &self.verts[1..] {
            let pt = (trans.rotation * pt) + trans.translation;
            let dist = dir.dot(pt);
            if dist > max_dist {
                max_dist = dist;
                max_pt = pt;
            }
        }
    
        let norm = dir.normalize() * bias;
        max_pt + norm
    }

    fn fastest_linear_speed(&self, angular_velocity: Vec3, dir: Vec3) -> f32 {
        let mut max_speed = 0.0;
        let half_size = self.size * 0.5;
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
    }

}


