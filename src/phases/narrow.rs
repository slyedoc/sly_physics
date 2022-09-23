use bevy::{prelude::*, tasks::*};

use crate::{colliders::*, constraints::ContactArena, intersect::*, types::*, PhysicsConfig};

#[cfg(feature = "continuous")]
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
    mut manifold_arena: ResMut<ContactArena>,
    config: Res<PhysicsConfig>,
    colliders: Res<Assets<Collider>>,

) {
    //let start = Instant::now();
    // TODO: after 0.8 ComputeTaskPool::get() fails, using this for now
    // let broad_contacts = broad_contacts.iter().map(|pair| {
    //     query.get_many([pair.a, pair.b]).unwrap()            
    // }).collect::<Vec<_>>();
    //info!("Narrow Phase: {} contacts", broad_contacts.len());

  
    let contact_results = broad_contacts.iter()
    .collect::<Vec<_>>()
    .par_splat_map(ComputeTaskPool::get(), None, |chunk| {
        #[cfg(feature = "trace")]
        let _span = info_span!("narrow_phase").entered();
        let mut contact_results = Vec::new();
        //let mut contact_results = Vec::new();
        //for broad_contact in broad_contacts.iter()
        for broad_contact in chunk.iter()
        {
           let [
                (a, trans_a, col_a, vel_a, com_a, i_tensor_a),
                (b, trans_b, col_b, vel_b, com_b, i_tensor_b),
            ] = query.get_many([broad_contact.a, broad_contact.b]).unwrap();

            let collider_a = colliders.get(col_a).unwrap();
            let collider_b = colliders.get(col_b).unwrap();

            if let Some(contact) = match (collider_a, collider_b) {
                (Collider::Sphere(sphere_a), Collider::Sphere(sphere_b)) => {
                    if let Some((world_point_a, world_point_b, time_of_impact)) =
                        sphere_sphere_dynamic(
                            sphere_a.radius,
                            sphere_b.radius,
                            trans_a.translation,
                            trans_b.translation,
                            vel_a.linear,
                            vel_b.linear,
                            config.time,
                        )
                    {
                        let trans_a = &mut trans_a.clone();
                        let vel_a = &mut vel_a.clone();

                        let trans_b = &mut trans_b.clone();
                        let vel_b = &mut vel_b.clone();

                        // step bodies forward to get local space collision points
                        RBHelper::update(trans_a, vel_a, com_a, i_tensor_a, time_of_impact);

                        RBHelper::update(trans_b, vel_b, com_b, i_tensor_b, time_of_impact);

                        // convert world space contacts to local space
                        let local_point_a =
                            RBHelper::world_to_local(trans_a, com_a, world_point_a);
                        let local_point_b =
                            RBHelper::world_to_local(trans_b, com_b, world_point_b);

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
            } {
                contact_results.push(contact);        
            }
        }
         contact_results
     });

    //for contact in contact_results.iter() {
    for contact in contact_results.into_iter().flatten() {
        //info!("Contact: {:?}", contact);
         if contact.time_of_impact == 0.0 {
             let [(_a, trans_a, _col_a, _vel_a, com_a, _i_tensor_a), (_b, trans_b, _col_b, _vel_b, com_b, _i_tensor_b)] = query.get_many([contact.a, contact.b]).unwrap();
             manifold_arena.add_contact(contact, trans_a, com_a, trans_b, com_b);
         } else {
             // ballistic contact
             contacts.send(contact);
         }
    }

    // let end = Instant::now();
    // info!("Narrow par: {:?}", end - start);
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

        num_iters += 1;
        if num_iters > 10 {
            break;
        }

        // advance based on closest point
        let (world_point_a, world_point_b) =
            gjk_closest_points(shape_a, trans_a, shape_b, trans_b);
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

#[cfg(feature = "discrete")]
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
                    let mut c = Contact {
                        a: pair.a,
                        b: pair.b,
                        world_point_a: trans_a.translation + (normal * *radius_a),
                        world_point_b: trans_b.translation - (normal * *radius_b),
                        normal,
                        local_point_a: Vec3::ZERO,
                        local_point_b: Vec3::ZERO,
                        separation_dist,
                        time_of_impact: 0.0,
                    };
                    c.correct(&trans_a, &trans_b);
                    contacts.send(c);
                }
            }
            (_, _) => todo!(),
        }
    }
}
