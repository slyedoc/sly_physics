use std::mem::swap;

use crate::{
    bvh::{BvhInstance, BvhTri, Tlas, TlasNode},
    prelude::{Collidable, Collider},
    types::Aabb,
};
use bevy::prelude::*;

#[derive(Debug, Clone, Copy)]
pub struct Hit {
    pub distance: f32, // intersection distance along ray, often seen as t
    pub u: f32,        // barycentric coordinates of the intersection
    pub v: f32,
    // We are using more bits here than in tutorial
    pub tri_index: usize,
    pub entity: Entity,
}

impl Default for Hit {
    fn default() -> Self {
        Self {
            distance: f32::MAX,
            u: Default::default(),
            v: Default::default(),
            tri_index: Default::default(),
            // TODO: Yes this isn't ideal, should be an option, will come back to this
            entity: Entity::from_raw(0),
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct Ray {
    pub origin: Vec3,
    pub direction: Vec3, // Should be normalized
    pub direction_inv: Vec3,
    pub distance: f32,
    pub hit: Option<Hit>,
}

impl Default for Ray {
    fn default() -> Self {
        Ray {
            origin: Vec3::ZERO,
            direction: Vec3::Z,
            distance: f32::MAX,
            direction_inv: Vec3::ZERO,
            hit: None,
        }
    }
}

impl Ray {
    pub fn new(origin: Vec3, direction: Vec3) -> Self {
        let direction_inv = direction.recip();
        Self {
            origin,
            direction,
            direction_inv,
            ..Default::default()
        }
    }

    // TODO: This is from bevy_mod_raycast, need to do more reading up on ndc
    pub fn from_camera(camera: &Camera, camera_transform: &Transform, cursor: Vec2) -> Self {
        let screen_size = camera.logical_viewport_size().unwrap();
        let camera_position = camera_transform.compute_matrix();
        let projection_matrix = camera.projection_matrix();

        // Normalized device coordinate cursor position from (-1, -1, -1) to (1, 1, 1)
        let cursor_ndc = (cursor / screen_size) * 2.0 - Vec2::from([1.0, 1.0]);
        let cursor_pos_ndc_near: Vec3 = cursor_ndc.extend(-1.0);
        let cursor_pos_ndc_far: Vec3 = cursor_ndc.extend(1.0);

        // Use near and far ndc points to generate a ray in world space
        // This method is more robust than using the location of the camera as the start of
        // the ray, because ortho cameras have a focal point at infinity!
        let ndc_to_world: Mat4 = camera_position * projection_matrix.inverse();
        let cursor_pos_near: Vec3 = ndc_to_world.project_point3(cursor_pos_ndc_near);
        let cursor_pos_far: Vec3 = ndc_to_world.project_point3(cursor_pos_ndc_far);
        let ray_direction = cursor_pos_far - cursor_pos_near;

        Ray {
            origin: cursor_pos_near,
            direction: ray_direction,
            direction_inv: ray_direction.recip(),
            distance: f32::MAX,
            hit: None,
        }
    }

    // Moller Trumbore
    // https://en.wikipedia.org/wiki/M%C3%B6ller%E2%80%93Trumbore_intersection_algorithm
    #[inline(always)]
    pub fn intersect_triangle(&mut self, tri: &BvhTri) -> Option<f32> {
        #[cfg(feature = "trace")]
        let _span = info_span!("intersect_triangle").entered();
        let edge1 = tri.vertex1 - tri.vertex0;
        let edge2 = tri.vertex2 - tri.vertex0;
        let h = self.direction.cross(edge2);
        let a = edge1.dot(h);
        if a.abs() < 0.00001 {
            return None;
        }

        // ray parallel to triangle
        let f = 1.0 / a;
        let s = self.origin - tri.vertex0;
        let u = f * s.dot(h);
        if !(0.0..=1.0).contains(&u) {
            return None;
        }
        let q = s.cross(edge1);
        let v = f * self.direction.dot(q);
        if v < 0.0 || u + v > 1.0 {
            return None;
        }
        let t = f * edge2.dot(q);

        if t < 0.0001 {
            return None;
        }
        Some(t)
    }

    #[inline(always)]
    pub fn intersect_aabb(&self, aabb: &Aabb) -> f32 {
        #[cfg(feature = "trace")]
        let _span = info_span!("intersect_aabb").entered();
        let tx1 = (aabb.mins.x - self.origin.x) * self.direction_inv.x;
        let tx2 = (aabb.maxs.x - self.origin.x) * self.direction_inv.x;
        let mut t_min = tx1.min(tx2);
        let mut t_max = tx1.max(tx2);
        let ty1 = (aabb.mins.y - self.origin.y) * self.direction_inv.y;
        let ty2 = (aabb.maxs.y - self.origin.y) * self.direction_inv.y;
        t_min = t_min.max(ty1.min(ty2));
        t_max = t_max.min(ty1.max(ty2));
        let tz1 = (aabb.mins.z - self.origin.z) * self.direction_inv.z;
        let tz2 = (aabb.maxs.z - self.origin.z) * self.direction_inv.z;
        t_min = t_min.max(tz1.min(tz2));
        t_max = t_max.min(tz1.max(tz2));

        // Most intersect test would return here with a t_max and min test
        // but we are also sorting
        let t_hit = if let Some(hit) = self.hit {
            hit.distance
        } else {
            f32::MAX
        };

        if t_max >= t_min && t_min < t_hit && t_max > 0.0 {
            t_min
        } else {
            f32::MAX
        }
    }

    pub fn intersect_collider_instance(
        &mut self,
        colliders: &Assets<Collider>,
        bvh_instance: &BvhInstance,
    ) {
        #[cfg(feature = "trace")]
        let _span = info_span!("intersect_bvh_instance").entered();
        let collider = colliders.get(&bvh_instance.collider).unwrap();
        // backup ray and transform original
        let mut backup_ray = *self;

        self.origin = bvh_instance.inv_trans.transform_point3(self.origin);
        self.direction = bvh_instance.inv_trans.transform_vector3(self.direction);
        self.direction_inv = self.direction.recip();
        if let Some(t) = collider.intersect(self) {
            if let Some(hit) = self.hit {
                if t < hit.distance {
                    self.hit = Some(Hit {
                        distance: t,
                        u: 0.5,
                        v: 0.5,
                        tri_index: 0,
                        entity: bvh_instance.entity,
                    });
                }
            } else {
                self.hit = Some(Hit {
                    distance: t,
                    u: 0.5,
                    v: 0.5,
                    tri_index: 0,
                    entity: bvh_instance.entity,
                });
            }
        };

        // restore ray origin and direction
        backup_ray.hit = self.hit;
        *self = backup_ray;
    }

    pub fn intersect_tlas(&mut self, tlas: &Tlas, colliders: &Assets<Collider>) -> Option<Hit> {
        if tlas.nodes.is_empty() || tlas.blas.is_empty() {
            return None;
        }
        let mut stack = Vec::<&TlasNode>::with_capacity(64);
        let mut node = &tlas.nodes[0];
        loop {
            if node.is_leaf() {
                self.intersect_collider_instance(colliders, &tlas.blas[node.blas as usize]);
                if stack.is_empty() {
                    break;
                } else {
                    node = stack.pop().unwrap();
                }
                continue;
            }
            let mut child1 = &tlas.nodes[(node.left_right & 0xffff) as usize];
            let mut child2 = &tlas.nodes[(node.left_right >> 16) as usize];
            let mut dist1 = self.intersect_aabb(&child1.aabb);
            let mut dist2 = self.intersect_aabb(&child2.aabb);
            if dist1 > dist2 {
                swap(&mut dist1, &mut dist2);
                swap(&mut child1, &mut child2);
            }
            if dist1 == f32::MAX {
                if stack.is_empty() {
                    break;
                } else {
                    node = stack.pop().unwrap();
                }
            } else {
                node = child1;
                if dist2 != f32::MAX {
                    stack.push(child2);
                }
            }
        }
        self.hit
    }
}
