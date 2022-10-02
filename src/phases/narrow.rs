use bevy::{
    prelude::*,
    tasks::{ComputeTaskPool, ParallelSlice},
};

use crate::{colliders::*, intersect::*, types::*, PhysicsConfig};

pub fn narrow_phase(
    query: Query<(
        Entity,
        &Transform,
        &Handle<Collider>,
        &Velocity,
        &CenterOfMass,
        &InertiaTensor,
    )>,
    mut broad_contacts: EventReader<BroadContact>,
    mut contacts: EventWriter<Contact>,
    mut manifold_contacts: EventWriter<ManifoldContact>,
    config: Res<PhysicsConfig>,
    colliders: Res<Assets<Collider>>,
) {
    let broad_contacts = broad_contacts.iter()
     .collect::<Vec<_>>();
    //info!("Narrow phase contacts: {}", broad_contacts.len());

    let contact_results = broad_contacts
     .par_splat_map(ComputeTaskPool::get(), None, |chunk| {
        let mut chunk_contacts = Vec::with_capacity(chunk.len());
        for broad_contact in chunk.iter() {

            let [(a, trans_a, col_a, vel_a, com_a, i_tensor_a), (b, trans_b, col_b, vel_b, com_b, i_tensor_b)] =
                query.get_many([broad_contact.a, broad_contact.b]).unwrap();

            let collider_a = colliders.get(col_a).unwrap();
            let collider_b = colliders.get(col_b).unwrap();

            let contact = match (collider_a, collider_b) {
                (Collider::Sphere(sphere_a), Collider::Sphere(sphere_b)) => {
                    if let Some((world_point_a, world_point_b, time_of_impact)) = sphere_sphere_dynamic(
                        sphere_a.radius,
                        sphere_b.radius,
                        trans_a.translation,
                        trans_b.translation,
                        vel_a.linear,
                        vel_b.linear,
                        config.time,
                    ) {
                        let trans_a = &mut trans_a.clone();
                        let vel_a = &mut vel_a.clone();

                        let trans_b = &mut trans_b.clone();
                        let vel_b = &mut vel_b.clone();

                        // step bodies forward to get local space collision points
                        RBHelper::update(trans_a, vel_a, com_a, i_tensor_a, time_of_impact);

                        RBHelper::update(trans_b, vel_b, com_b, i_tensor_b, time_of_impact);

                        // convert world space contacts to local space
                        let local_point_a = RBHelper::world_to_local(trans_a, com_a, world_point_a);
                        let local_point_b = RBHelper::world_to_local(trans_b, com_b, world_point_b);

                        let normal = (trans_a.translation - trans_b.translation).normalize();

                        // calculate the separation distance
                        let ab = trans_a.translation - trans_b.translation;
                        let separation_dist = ab.length() - (sphere_a.radius + sphere_b.radius);

                        Some(Contact {
                            a,
                            b,
                            world_point_a,
                            world_point_b,
                            local_point_a,
                            local_point_b,
                            normal,
                            separation_dist,
                            time_of_impact,
                        })
                    } else {
                        None
                    }
                }
                (collider_a, collider_b) => conservative_advancement(
                    &a,
                    collider_a,
                    trans_a,
                    vel_a,
                    com_a,
                    i_tensor_a,
                    &b,
                    collider_b,
                    trans_b,
                    vel_b,
                    com_b,
                    i_tensor_b,
                    config.time,
                ),
            };
            if let Some(mut c) = contact {    
                c.correct();            
                chunk_contacts.push(c);
            }
        }
        chunk_contacts
    });

    for c in contact_results.into_iter().flatten() {
        if c.time_of_impact == 0.0 {
            manifold_contacts.send(ManifoldContact(c));
        } else {
            contacts.send(c);
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn conservative_advancement(
    // Shape A
    a: &Entity,
    shape_a: &Collider,
    trans_a: &Transform,
    vel_a: &Velocity,
    com_a: &CenterOfMass,
    i_tensor_a: &InertiaTensor,
    // Shape B
    b: &Entity,
    shape_b: &Collider,
    trans_b: &Transform,
    vel_b: &Velocity,
    com_b: &CenterOfMass,
    i_tensor_b: &InertiaTensor,
    time: f32,
) -> Option<Contact> {
    let trans_a = &mut trans_a.clone();
    let vel_a = &mut vel_a.clone();

    let trans_b = &mut trans_b.clone();
    let vel_b = &mut vel_b.clone();

    let mut result = None;
    let mut toi = 0.0;
    let mut num_iters = 0;
    let mut dt = time;
    // advance the positions of the bodies until they touch or there's not time left
    while dt > 0.0 {
        // check for intersection
        const BIAS: f32 = 0.001;

        if let Some((mut world_point_a, mut world_point_b)) =
            gjk_does_intersect(shape_a, trans_a, shape_b, trans_b, BIAS)
        {
            let normal = (world_point_b - world_point_a).normalize_or_zero();
            world_point_a -= normal * BIAS;
            world_point_b += normal * BIAS;

            result = Some(Contact {
                a: *a,
                b: *b,
                world_point_a,
                world_point_b,
                local_point_a: RBHelper::world_to_local(trans_a, com_a, world_point_a),
                local_point_b: RBHelper::world_to_local(trans_b, com_b, world_point_b),
                normal,
                separation_dist: -(world_point_a - world_point_b).length(),
                time_of_impact: toi,
            });
            break;
        }

        // TODO: limit the number of iterations
        num_iters += 1;
        if num_iters > 10 {
            break;
        }

        // advance based on closest point
        let (world_point_a, world_point_b) = gjk_closest_points(shape_a, trans_a, shape_b, trans_b);
        let separation_dist = (world_point_a - world_point_b).length();

        // get the vector from the closest point on A to the closest point on B
        let ab = (world_point_b - world_point_a).normalize_or_zero();

        // project the relative velocity onto the ray of shortest distance
        let relative_velocity = vel_a.linear - vel_b.linear;
        let mut ortho_speed = relative_velocity.dot(ab);

        // add to the ortho_speed the maximum angular speeds of the relative shapes
        let angular_speed_a = shape_a.fastest_linear_speed(vel_a.angular, ab);
        let angular_speed_b = shape_b.fastest_linear_speed(vel_b.angular, -ab);

        ortho_speed += angular_speed_a + angular_speed_b;

        if ortho_speed <= 0.0 {
            break;
        }

        let time_to_go = separation_dist / ortho_speed;
        if time_to_go > dt {
            break;
        }

        dt -= time_to_go;
        toi += time_to_go;

        // advance
        RBHelper::update(trans_a, vel_a, com_a, i_tensor_a, time_to_go);
        RBHelper::update(trans_b, vel_b, com_b, i_tensor_b, time_to_go);
    }

    result
}
