use bevy::math::Vec3;

use crate::{LinearVelocity, CenterOfMassWorld};

#[allow(dead_code)]
pub fn ray_sphere_intersect(
    ray_start: Vec3,
    ray_direction: Vec3,
    sphere_center: Vec3,
    sphere_radius: f32,
) -> Option<(f32, f32)> {
    let m = sphere_center - ray_start;
    let a = ray_direction.dot(ray_direction);
    let b = m.dot(ray_direction);
    let c = m.dot(m) - sphere_radius * sphere_radius;

    let delta = b * b - a * c;

    if delta < 0.0 {
        None
    } else {
        let inv_a = 1.0 / a;
        let delta_root = delta.sqrt();
        let t1 = inv_a * (b - delta_root);
        let t2 = inv_a * (b + delta_root);
        Some((t1, t2))
    }
}

#[allow(dead_code)]
pub fn sphere_sphere_static(
    radius_a: f32,
    radius_b: f32,
    pos_a: Vec3,
    pos_b: Vec3,
) -> Option<(Vec3, Vec3)> {
    let ab = pos_b - pos_a;
    let radius_ab = radius_a + radius_b;
    let length_squared = ab.length_squared();
    if length_squared < radius_ab * radius_ab {
        let norm = ab.normalize_or_zero();
        
        let pt_on_a = pos_a + norm * radius_a;
        let pt_on_b = pos_b - norm * radius_b;

        Some((pt_on_a, pt_on_b))
    } else {
        None
    }
}

#[allow(dead_code)]
pub fn sphere_sphere_dynamic(
    radius_a: f32,
    radius_b: f32,
    linear_velocity_a: &LinearVelocity,
    linear_velocity_b: &LinearVelocity,
    com_world_a: &CenterOfMassWorld,
    com_world_b: &CenterOfMassWorld,
    dt: f32,
) -> Option<(Vec3, Vec3, f32)> {
    let relative_velocity = linear_velocity_a.0 - linear_velocity_b.0;

    let start_pt_a = com_world_a.0;
    let end_pt_a = com_world_a.0 + relative_velocity * dt;
    let ray_dir = end_pt_a - start_pt_a;

    let mut t0 = 0.0;
    let mut t1 = 0.0;

    const EPSILON: f32 = 0.001;
    const EPSILON_SQ: f32 = EPSILON * EPSILON;
    if ray_dir.length_squared() < EPSILON_SQ {
        // ray is too short, just check if intersecting
        let ab = com_world_b.0 - com_world_a.0;
        let radius = radius_a + radius_b + EPSILON;
        if ab.length_squared() > radius * radius {
            return None;
        }
    } else if let Some(toi) = ray_sphere_intersect(com_world_a.0, ray_dir, com_world_b.0, radius_a + radius_b) {
        t0 = toi.0;
        t1 = toi.1;
    } else {
        return None;
    }

    // Change from [0,1] range to [0,dt] range
    t0 *= dt;
    t1 *= dt;

    // If the collision is only in the past, then there's not future collision this frame
    if t1 < 0.0 {
        return None;
    }

    // Get the earliest positive time of impact
    let toi = if t0 < 0.0 { 0.0 } else { t0 };

    // If the earliest collision is too far in the future, then there's no collision this frame
    if toi > dt {
        return None;
    }

    // get the points on the respective points of collision
    let new_pos_a = com_world_b.0 + linear_velocity_a.0 * toi;
    let new_pos_b = com_world_b.0 + linear_velocity_b.0 * toi;
    let ab = (new_pos_b - new_pos_a).normalize();

    let pt_on_a = new_pos_a + ab * radius_a;
    let pt_on_b = new_pos_b - ab * radius_b;

    Some((pt_on_a, pt_on_b, toi))
}
