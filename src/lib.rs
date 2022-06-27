
mod colliders;
mod phases;
mod rigid_body;

use bevy::prelude::*;
use bevy_inspector_egui::{Inspectable, RegisterInspectable};
pub use colliders::*;

use phases::*;

pub use self::rigid_body::*;


 
#[derive(Inspectable, Debug)]
pub struct Gravity(pub Vec3);
impl Default for Gravity {
    fn default() -> Self {
        Gravity(Vec3::new(0.0, -9.8, 0.0))
    }
}
#[derive(Component, Inspectable, Debug, Default)]
pub struct InverseInertiaTensor(pub Mat3);

#[derive(Default)]
pub struct PhysicsTime {
    pub time: f32,
}

pub struct PhysicsPlugin;



impl Plugin for PhysicsPlugin {
    fn build(&self, app: &mut App) {

        app
            .init_resource::<Gravity>()
            .init_resource::<PhysicsTime>()
            //.register_inspectable::<RigidBody>()
            //.register_inspectable::<Static>()
            //.register_inspectable::<LinearVelocity>()
             //.register_inspectable::<AngularVelocity>()
            //  .register_inspectable::<Elasticity>()
            //  .register_inspectable::<Friction>()
            //.register_inspectable::<Mass>()
            //.register_inspectable::<InvMass>()
            //  .register_inspectable::<CenterOfMass>()
            //  .register_inspectable::<Collider>()            
            
            .add_system_to_stage(CoreStage::PreUpdate, update_time_system)
            .add_system(spawn_components_system)
            .add_system(update_com_world_system.after(spawn_components_system))
            .add_system(dynamics_system.after(update_com_world_system))
            .add_system(update_system.after(dynamics_system))
            ;
    }
}


pub fn spawn_components_system(
    mut commands: Commands,
    mut query: Query<
        (
            Entity,
            &Collider,
            &Transform,
            Option<&RigidBody>,
            Option<&Mass>,
            Option<&InvMass>,
            Option<&LinearVelocity>,
            Option<&AngularVelocity>,
            Option<&Elasticity>,
            Option<&Friction>,
            Option<&CenterOfMass>,
            Option<&CenterOfMassWorld>,
        ),
        (Added<Collider>),
    >,
) {
    for (
        e,
        collider,
        transform,
        //optional after this
        rigid_body,
        mass,
        inv_mass,
        linear_vel,
        angular_vel,
        elasticity,
        friction,
        center_of_mass,
        center_of_mass_world,
    ) in query.iter()
    {
        // add rigid body if not already added
        if rigid_body.is_none() {
            commands.entity(e).insert(RigidBody);
        }

        // add inv_mass
        if inv_mass.is_none() {
            if let Some(mass) = mass {
                commands.entity(e).insert(InvMass(1.0 / mass.0));
            } else {
                commands.entity(e).insert(InvMass(1.0));
            }
        }

        // add linear velocity
        if linear_vel.is_none() {
            commands.entity(e).insert(LinearVelocity::default());
        }

        // add angular velocity
        if angular_vel.is_none() {
            commands.entity(e).insert(AngularVelocity::default());
        }

        // add elasticity
        if elasticity.is_none() {
            commands.entity(e).insert(Elasticity::default());
        }

        // add friction
        if friction.is_none() {
            commands.entity(e).insert(Friction::default());
        }

        // add center of mass
        if center_of_mass.is_none() {
            commands
                .entity(e)
                .insert(CenterOfMass(collider.get_center_of_mass()));
        }

        if center_of_mass_world.is_none() {
            commands
                .entity(e)
                .insert(CenterOfMassWorld::default());
        }
    }
}

pub fn update_com_world_system(
    mut commands: Commands,
    mut query: Query<(&mut CenterOfMassWorld, &Transform, &CenterOfMass)>
) {
    for (mut center_of_mass_world, trans, center_of_mass) in query.iter_mut() {
        center_of_mass_world.0 = trans.translation + trans.rotation * center_of_mass.0;
    }
}

fn update_time_system(time: Res<Time>, mut pt: ResMut<PhysicsTime>) {
    pt.time = time.delta_seconds();
}