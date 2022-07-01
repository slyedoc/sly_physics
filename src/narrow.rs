use bevy::prelude::*;

use crate::{
    broad::BroadContact,  AngularVelocity, CenterOfMass, Collider,
    InertiaTensor, LinearVelocity, PhysicsConfig,
    
};

#[cfg(not(feature = "static"))]
use crate::{intersect::sphere_sphere_dynamic, RBHelper };

#[derive(Debug)]
pub struct Contact {
    pub a: Entity,
    pub b: Entity,
    pub world_point_a: Vec3,
    pub world_point_b: Vec3,
    pub local_point_a: Vec3,
    pub local_point_b: Vec3,
    pub normal: Vec3,
    pub separation_dist: f32,
    pub time_of_impact: f32,
}

pub fn narrow_system(
    query: Query<(
        &mut Transform,
        &Collider,
        &LinearVelocity,
        &mut AngularVelocity,
        &CenterOfMass,
        &InertiaTensor,
    )>,
    mut broad_contacts: EventReader<BroadContact>,
    mut contacts: EventWriter<Contact>,
    config: Res<PhysicsConfig>,
) {
    for pair in broad_contacts.iter() {
        unsafe {
            let (mut trans_a, type_a, lin_vel_a, mut ang_vel_a, com_a, i_tensor_a) =
                query.get_unchecked(pair.a).unwrap();
            let (mut trans_b, type_b, lin_vel_b, mut ang_vel_b, com_b, i_tensor_b) =
                query.get_unchecked(pair.b).unwrap();

            match (type_a, type_b) {
                (Collider::Sphere { radius: radius_a }, Collider::Sphere { radius: radius_b }) => {
                    #[cfg(not(feature = "static"))]
                    {
                        if let Some((world_point_a, world_point_b, time_of_impact)) =
                            sphere_sphere_dynamic(
                                *radius_a,
                                *radius_b,
                                trans_a.translation,
                                trans_b.translation,
                                lin_vel_a.0,
                                lin_vel_b.0,
                                config.time,
                            )
                        {
                            info!(
                                "point a {}, point b {}, toi {} ",
                                world_point_a, world_point_b, time_of_impact
                            );
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
                            let local_point_a =
                                RBHelper::world_to_local(&trans_a, com_a, world_point_a);
                            let local_point_b =
                                RBHelper::world_to_local(&trans_b, com_b, world_point_b);

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

                            let c = Contact {
                                a: pair.a,
                                b: pair.b,
                                world_point_a,
                                world_point_b,
                                local_point_a,
                                local_point_b,
                                normal,
                                separation_dist,
                                time_of_impact,
                            };

                            info!("{:?}", c);
                            contacts.send(c);
                        }
                    }
                    #[cfg(feature = "static")]
                    {
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
                }
                (_, _) => todo!(),
            }
        }
    }
}
