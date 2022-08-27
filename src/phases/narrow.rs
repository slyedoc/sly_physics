use bevy::prelude::*;

use crate::{colliders::*, constraints::PenetrationArena, intersect::*, types::*, PhysicsConfig};

#[cfg(feature = "continuous")]
pub fn narrow_system(
    mut query: Query<(
        &mut Transform,
        &Collider,
        &LinearVelocity,
        &mut AngularVelocity,
        &CenterOfMass,
        &InertiaTensor,
    )>,
    mut broad_contacts: EventReader<BroadContact>,
    mut contacts: EventWriter<Contact>,
    mut manifold_arean: ResMut<PenetrationArena>,
    config: Res<PhysicsConfig>,
    collider_resources: Res<ColliderResources>,
) {

    for pair in broad_contacts.iter() {
        let bodies = query.get_many_mut([pair.a, pair.b]);
        let [(mut trans_a, type_a, lin_vel_a, mut ang_vel_a, com_a, i_tensor_a), (mut trans_b, type_b, lin_vel_b, mut ang_vel_b, com_b, i_tensor_b)] =
            bodies.unwrap();

        // TODO: ideally we can use different collision test between different shapes, for now really only sphere sphere has its own
        // and due to avoiding dynamic dispatching we have split everything out, in hind sight we really should have just started with dynamic dispatching
        // and removed it later if testing show it was faster, at this point I may not be and it does create alot of boiler plate and code dupilication

        if let Some(contact) = match (type_a, type_b) {
            (Collider::Sphere(sphere_a_handle), Collider::Sphere(sphere_b_handle)) => {
                let sphere_a = collider_resources.get_sphere(*sphere_a_handle);
                let sphere_b = collider_resources.get_sphere(*sphere_b_handle);

                if let Some((world_point_a, world_point_b, time_of_impact)) = sphere_sphere_dynamic(
                    sphere_a.radius,
                    sphere_b.radius,
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
                    let separation_dist = ab.length() - (sphere_a.radius + sphere_b.radius);

                    Some(Contact {
                        a: pair.a,
                        b: pair.b,
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
            (Collider::Box(box_a_index), Collider::Box(box_b_index)) => {
                let cube_a = collider_resources.get_cube(*box_a_index);
                let cube_b = collider_resources.get_cube(*box_b_index);
                conservative_advancement::<BoxCollider, BoxCollider>(
                    cube_a,
                    &mut trans_a,
                    &mut ang_vel_a,
                    lin_vel_a,
                    com_a,
                    i_tensor_a,
                    cube_b,
                    &mut trans_b,
                    &mut ang_vel_b,
                    lin_vel_b,
                    com_b,
                    i_tensor_b,
                    pair,
                    config.time,
                )
            }
            (Collider::Sphere(sphere_index), Collider::Box(box_index)) => {
                let sphere_a = collider_resources.get_sphere(*sphere_index);
                let cube = collider_resources.get_cube(*box_index);
                conservative_advancement::<SphereCollider, BoxCollider>(
                    sphere_a,
                    &mut trans_a,
                    &mut ang_vel_a,
                    lin_vel_a,
                    com_a,
                    i_tensor_a,
                    cube,
                    &mut trans_b,
                    &mut ang_vel_b,
                    lin_vel_b,
                    com_b,
                    i_tensor_b,
                    pair,
                    config.time,
                )
            }
            (Collider::Box(cube_index), Collider::Sphere(sphere_index)) => {
                let sphere = collider_resources.get_sphere(*sphere_index);
                let cube = collider_resources.get_cube(*cube_index);
                conservative_advancement::<BoxCollider, SphereCollider>(
                    cube,
                    &mut trans_a,
                    &mut ang_vel_a,
                    lin_vel_a,
                    com_a,
                    i_tensor_a,
                    sphere,
                    &mut trans_b,
                    &mut ang_vel_b,
                    lin_vel_b,
                    com_b,
                    i_tensor_b,
                    pair,
                    config.time,
                )
            }
            (Collider::Sphere(sphere_index), Collider::Convex(convex_index)) => {
                let sphere = collider_resources.get_sphere(*sphere_index);
                let convex = collider_resources.get_convex(*convex_index);
                conservative_advancement::<SphereCollider, ConvexCollider>(
                    sphere,
                    &mut trans_a,
                    &mut ang_vel_a,
                    lin_vel_a,
                    com_a,
                    i_tensor_a,
                    convex,
                    &mut trans_b,
                    &mut ang_vel_b,
                    lin_vel_b,
                    com_b,
                    i_tensor_b,
                    pair,
                    config.time,
                )
            }
            (Collider::Box(cube_index), Collider::Convex(convex_index)) => {
                let cube = collider_resources.get_cube(*cube_index);
                let convex = collider_resources.get_convex(*convex_index);
                conservative_advancement::<BoxCollider, ConvexCollider>(
                    cube,
                    &mut trans_a,
                    &mut ang_vel_a,
                    lin_vel_a,
                    com_a,
                    i_tensor_a,
                    convex,
                    &mut trans_b,
                    &mut ang_vel_b,
                    lin_vel_b,
                    com_b,
                    i_tensor_b,
                    pair,
                    config.time,
                )
            }
            (Collider::Convex(convex_index), Collider::Sphere(sphere_index)) => {
                let convex = collider_resources.get_convex(*convex_index);
                let sphere = collider_resources.get_sphere(*sphere_index);
                conservative_advancement::<ConvexCollider, SphereCollider>(
                    convex,
                    &mut trans_a,
                    &mut ang_vel_a,
                    lin_vel_a,
                    com_a,
                    i_tensor_a,
                    sphere,
                    &mut trans_b,
                    &mut ang_vel_b,
                    lin_vel_b,
                    com_b,
                    i_tensor_b,
                    pair,
                    config.time,
                )
            }
            (Collider::Convex(convex_index), Collider::Box(cube_index)) => {
                let convex = collider_resources.get_convex(*convex_index);
                let cube = collider_resources.get_cube(*cube_index);
                conservative_advancement::<ConvexCollider, BoxCollider>(
                    convex,
                    &mut trans_a,
                    &mut ang_vel_a,
                    lin_vel_a,
                    com_a,
                    i_tensor_a,
                    cube,
                    &mut trans_b,
                    &mut ang_vel_b,
                    lin_vel_b,
                    com_b,
                    i_tensor_b,
                    pair,
                    config.time,
                )
            }
            (Collider::Convex(convex_index_a), Collider::Convex(convex_index_b)) => {
                let convex_a = collider_resources.get_convex(*convex_index_a);
                let convex_b = collider_resources.get_convex(*convex_index_b);
                conservative_advancement::<ConvexCollider, ConvexCollider>(
                    convex_a,
                    &mut trans_a,
                    &mut ang_vel_a,
                    lin_vel_a,
                    com_a,
                    i_tensor_a,
                    convex_b,
                    &mut trans_b,
                    &mut ang_vel_b,
                    lin_vel_b,
                    com_b,
                    i_tensor_b,
                    pair,
                    config.time,
                )
            }
        } {
            if contact.time_of_impact == 0.0 {
                manifold_arean.add_contact(contact, &trans_a, com_a, &trans_b, com_b);
            } else { 
                // ballistic contact
                contacts.send(contact);
            }
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn conservative_advancement<T: ColliderTrait, K: ColliderTrait>(
    // Shape A
    shape_a: &T,
    trans_a: &mut Transform,
    ang_vel_a: &mut AngularVelocity,
    lin_vel_a: &LinearVelocity,
    com_a: &CenterOfMass,
    i_tensor_a: &InertiaTensor,
    // Shape B
    shape_b: &K,
    trans_b: &mut Transform,
    ang_vel_b: &mut AngularVelocity,
    lin_vel_b: &LinearVelocity,
    com_b: &CenterOfMass,
    i_tensor_b: &InertiaTensor,
    pair: &BroadContact,
    time: f32,
) -> Option<Contact> {


    let mut result = None;
    let mut toi = 0.0;
    let mut num_iters = 0;
    let mut dt = time;
    // advance the positions of the bodies until they touch or there's not time left
    while dt > 0.0 {
        // check for intersection
        const BIAS: f32 = 0.001;

        if let Some((mut world_point_a, mut world_point_b)) =
            gjk_does_intersect::<T, K>(shape_a, trans_a, shape_b, trans_b, BIAS)
        {
            let normal = (world_point_b - world_point_a).normalize_or_zero();
            world_point_a -= normal * BIAS;
            world_point_b += normal * BIAS;

            result = Some(Contact {
                a: pair.a,
                b: pair.b,
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
            gjk_closest_points::<T, K>(shape_a, trans_a, shape_b, trans_b);
        let separation_dist = (world_point_a - world_point_b).length();

        // get the vector from the closest point on A to the closest point on B
        let ab = (world_point_b - world_point_a).normalize_or_zero();

        // project the relative velocity onto the ray of shortest distance
        let relative_velocity = lin_vel_a.0 - lin_vel_b.0;
        let mut ortho_speed = relative_velocity.dot(ab);

        // add to the ortho_speed the maximum angular speeds of the relative shapes
        let angular_speed_a = shape_a.fastest_linear_speed(ang_vel_a.0, ab);
        let angular_speed_b = shape_b.fastest_linear_speed(ang_vel_b.0, -ab);

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
        RBHelper::update(trans_a, ang_vel_a, lin_vel_a, com_a, i_tensor_a, time_to_go);
        RBHelper::update(trans_b, ang_vel_b, lin_vel_b, com_b, i_tensor_b, time_to_go);
    }
    // unwind the clock
    RBHelper::update(trans_a, ang_vel_a, lin_vel_a, com_a, i_tensor_a, -toi);
    RBHelper::update(trans_b, ang_vel_b, lin_vel_b, com_b, i_tensor_b, -toi);
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
