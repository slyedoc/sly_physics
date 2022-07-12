use bevy::prelude::*;

use crate::{
    PhysicsConfig,
    constraints::ManifoldArena,
    intersect::{*},
    types::{*},
    colliders::{*},
};

#[cfg(not(feature = "static"))]
pub fn narrow_system(
    mut query: Query<(
        &mut Transform,
        &Collider,
        &LinearVelocity,
        &mut AngularVelocity,
        &CenterOfMass,
        &InertiaTensor,
        &GJKVerts,
    )>,
    mut broad_contacts: EventReader<BroadContact>,
    mut contacts: EventWriter<Contact>,
    mut manifold_arean: ResMut<ManifoldArena>,
    config: Res<PhysicsConfig>,
) {
    for pair in broad_contacts.iter() {
        
        let bodies = query.get_many_mut([pair.a, pair.b]);
        if bodies.is_err() {
            warn!("narrow_system: bodies not found");  
            continue;
        }
        
        let [(mut trans_a, type_a, lin_vel_a, mut ang_vel_a, com_a, i_tensor_a, gjk_verts_a), (mut trans_b, type_b, lin_vel_b, mut ang_vel_b, com_b, i_tensor_b, gjk_verts_b)] = bodies.unwrap();
        match (type_a, type_b) {
            (Collider::Sphere { radius: radius_a }, Collider::Sphere { radius: radius_b }) => {
                if let Some((world_point_a, world_point_b, time_of_impact)) = sphere_sphere_dynamic(
                    *radius_a,
                    *radius_b,
                    trans_a.translation,
                    trans_b.translation,
                    lin_vel_a.0,
                    lin_vel_b.0,
                    config.time,
                ) {
                    // step bodies forward to get local space collision points
                    RBHelper::update(
                        &mut trans_a,
                        &mut ang_vel_a,
                        lin_vel_a,
                        com_a,
                        i_tensor_a,
                        time_of_impact,
                    );

                    RBHelper::update(
                        &mut trans_b,
                        &mut ang_vel_b,
                        lin_vel_b,
                        com_b,
                        i_tensor_b,
                        time_of_impact,
                    );

                    // convert world space contacts to local space
                    let local_point_a = RBHelper::world_to_local(&trans_a, com_a, world_point_a);
                    let local_point_b = RBHelper::world_to_local(&trans_b, com_b, world_point_b);

                    let normal = (trans_a.translation - trans_b.translation).normalize();

                    // unwind time step
                    RBHelper::update(
                        &mut trans_a,
                        &mut ang_vel_a,
                        lin_vel_a,
                        com_a,
                        i_tensor_a,
                        -time_of_impact,
                    );
                    RBHelper::update(
                        &mut trans_b,
                        &mut ang_vel_b,
                        lin_vel_b,
                        com_b,
                        i_tensor_b,
                        -time_of_impact,
                    );

                    // calculate the separation distance
                    let ab = trans_a.translation - trans_b.translation;
                    let separation_dist = ab.length() - (radius_a + radius_b);

                    contacts.send(Contact {
                        a: pair.a,
                        b: pair.b,
                        world_point_a,
                        world_point_b,
                        local_point_a,
                        local_point_b,
                        normal,
                        separation_dist,
                        time_of_impact,
                    });
                }
            }
            // Conservative advancement
            (collider_a, collider_b) => {
                let mut toi = 0.0;
                let mut num_iters = 0;
                let mut dt = config.time;

                // advance the positions of the bodies until they touch or there's not time left
                while dt > 0.0 {
                    // check for intersection
                    const BIAS: f32 = 0.001;
   
                    if let Some((mut world_point_a, mut world_point_b)) = gjk_does_intersect(
                        &collider_a,
                        &trans_a,
                        &gjk_verts_a.0,
                        &collider_b,
                        &trans_b,
                        &gjk_verts_b.0,
                        BIAS,
                    ) {
                        let normal = (world_point_b - world_point_a).normalize_or_zero();
                        world_point_a -= normal * BIAS;
                        world_point_b += normal * BIAS;

                        // roll back advancement
                        RBHelper::update(
                            &mut trans_a,
                            &mut ang_vel_a,
                            lin_vel_a,
                            com_a,
                            i_tensor_a,
                            -toi,
                        );

                        RBHelper::update(
                            &mut trans_b,
                            &mut ang_vel_b,
                            lin_vel_b,
                            com_b,
                            i_tensor_b,
                            -toi,
                        );

                        let contact = Contact {
                            a: pair.a,
                            b: pair.b,
                            world_point_a,
                            world_point_b,
                            local_point_a: RBHelper::world_to_local(&trans_a, com_a, world_point_a),
                            local_point_b: RBHelper::world_to_local(&trans_b, com_b, world_point_b),
                            normal,
                            separation_dist: -(world_point_a - world_point_b).length(),
                            time_of_impact: toi,
                        };

                        if contact.time_of_impact == 0.0 {
                            manifold_arean.add_contact(contact, &trans_a, com_a, &trans_b, com_b);
                        } else {
                            // ballistic contact
                            contacts.send(contact);
                        }
                        break;
                    }

                    num_iters += 1;
                    // if num_iters > 2 {
                    //     // TODO: remove this, just wanted to see how oftent this is hit
                    //     info!("num_iters: {}", num_iters);
                    // }
                    if num_iters > 10 {
                        break;
                    }

                    // advance based on closest point
                    let (world_point_a, world_point_b) = gjk_closest_points(
                        &collider_a,
                        &trans_a,
                        &gjk_verts_a.0,
                        &collider_b,
                        &trans_b,
                        &gjk_verts_b.0,
                    );
                    let separation_dist = (world_point_a - world_point_b).length();

                    // get the vector from the closest point on A to the closest point on B
                    let ab = (world_point_b - world_point_a).normalize_or_zero();

                    // project the relative velocity onto the ray of shortest distance
                    let relative_velocity = lin_vel_a.0 - lin_vel_b.0;
                    let mut ortho_speed = relative_velocity.dot(ab);

                    // add to the ortho_speed the maximum angular speeds of the relative shaps
                    let angular_speed_a = collider_a.fastest_linear_speed(ang_vel_a.0, ab);
                    let angular_speed_b = collider_b.fastest_linear_speed(ang_vel_b.0, -ab);

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

                    // advanceme
                    RBHelper::update(
                        &mut trans_a,
                        &mut ang_vel_a,
                        lin_vel_a,
                        com_a,
                        i_tensor_a,
                        time_to_go,
                    );

                    RBHelper::update(
                        &mut trans_b,
                        &mut ang_vel_b,
                        lin_vel_b,
                        com_b,
                        i_tensor_b,
                        time_to_go,
                    );
                }

                // unwind the clock
                RBHelper::update(
                    &mut trans_a,
                    &mut ang_vel_a,
                    lin_vel_a,
                    com_a,
                    i_tensor_a,
                    -toi,
                );

                RBHelper::update(
                    &mut trans_b,
                    &mut ang_vel_b,
                    lin_vel_b,
                    com_b,
                    i_tensor_b,
                    -toi,
                );
            }
        }
    }
}

#[cfg(feature = "static")]
pub fn narrow_system(
    query: Query<(&Transform, &Collider)>,
    mut broad_contacts: EventReader<BroadContact>,
    mut contacts: EventWriter<Contact>,
) {
    for pair in broad_contacts.iter() {
        let [(trans_a, type_a), (trans_b, type_b)] = query.many([pair.a, pair.b]);

        match (type_a, type_b) {
            (Collider::Sphere { radius: radius_a }, Collider::Sphere { radius: radius_b }) => {
                let ab = trans_b.translation - trans_a.translation;
                let radius_ab = radius_a + radius_b;
                let radius_ab_sq = radius_ab * radius_ab;
                let ab_len_sq = ab.length_squared();
                if ab_len_sq <= radius_ab_sq {
                    let normal = ab.normalize();
                    let ab = trans_a.translation - trans_b.translation;
                    let separation_dist = ab.length() - (radius_a + radius_b);

                    // convert world space contacts to local space
                    contacts.send(Contact {
                        a: pair.a,
                        b: pair.b,
                        world_point_a: trans_a.translation + (normal * *radius_a),
                        world_point_b: trans_b.translation - (normal * *radius_b),
                        normal,
                        local_point_a: Vec3::ZERO,
                        local_point_b: Vec3::ZERO,
                        separation_dist,
                        time_of_impact: 0.0,
                    });
                }
            }
            (_, _) => todo!(),
        }
    }
}
