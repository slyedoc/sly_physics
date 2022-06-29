mod colliders;
mod types;
mod broad;
mod narrow;
mod resolve;
mod intersect;

use bevy::prelude::*;
use bevy_inspector_egui::{Inspectable, RegisterInspectable};
use broad::{BroadContact, broadphase_system};
pub use colliders::*;
use narrow::{Contact, narrow_system};
use resolve::resolve_system;

pub use self::types::*;

#[derive(Inspectable, Deref, DerefMut, Debug)]
pub struct Gravity(pub Vec3);
impl Default for Gravity {
    fn default() -> Self {
        Gravity(Vec3::new(0.0, -9.8, 0.0))
    }
}

#[derive(Inspectable)]
pub struct PhysicsConfig {    
    pub detection: CollisionDetection,
}

#[derive(Inspectable)]
pub enum CollisionDetection {
    Static,
    Continuous,
}

impl Default for PhysicsConfig {
    fn default() -> Self {
        PhysicsConfig {
            detection: CollisionDetection::Static,
        }
    }
}

#[derive(Default)]
pub struct PhysicsTime {
    pub time: f32,
}

pub struct PhysicsPlugin;


#[derive(Debug, Clone, PartialEq, Eq, Hash, Copy)]
pub enum PhysicsState {
    Running,
    Paused,
}



impl Plugin for PhysicsPlugin {
    fn build(&self, app: &mut App) {
        app
            .add_state(PhysicsState::Running)
            .init_resource::<Gravity>()
            .add_event::<BroadContact>()
            .add_event::<Contact>()
            .init_resource::<PhysicsTime>()
            .init_resource::<PhysicsConfig>()
            .register_inspectable::<RigidBody>()
            .register_inspectable::<Static>()
            .register_inspectable::<LinearVelocity>()
            .register_inspectable::<AngularVelocity>()
            .register_inspectable::<Elasticity>()
            .register_inspectable::<Friction>()
            .register_inspectable::<Mass>()
            .register_inspectable::<InverseMass>()
            .register_inspectable::<CenterOfMass>()
            .register_inspectable::<CenterOfMassWorld>()
            .register_inspectable::<InertiaTensor>()
            .register_inspectable::<InverseInertiaTensorWorld>()
            .register_inspectable::<Collider>()
            .add_system_to_stage(CoreStage::PreUpdate, update_time_system)
            .add_system(toggle_physics)
            .add_system_set(
                SystemSet::on_update(PhysicsState::Running)
                    .with_system(spawn)
                    .with_system(update_world_info.after(spawn))
                    .with_system(gravity_system.after(update_world_info))
                    .with_system(broadphase_system.after(gravity_system))    
                    .with_system(narrow_system.after(broadphase_system))   
                    .with_system(resolve_system.after(narrow_system))
                    .with_system(update_transform.after(resolve_system))                
                    // .with_system(
                    //     update_body_system
                    //         .label(Phases::UpdatePosition)
                    //         .after(Phases::Resolve),
                    // ),
            );
    }
}

 fn toggle_physics( input: Res<Input<KeyCode>>, mut state: ResMut<State<PhysicsState>>) {
    if input.just_pressed(KeyCode::Space) {
        match state.current() {
            PhysicsState::Paused => state.set(PhysicsState::Running).unwrap(),
            PhysicsState::Running => state.set(PhysicsState::Paused).unwrap(),
        }
    }
 }

// TODO: This is taking component based to and extreme and this could be a bundle, but makes it awark to add  to existing bundles
// THe only required component is Collider component
pub fn spawn(
    mut commands: Commands,
    query: Query<
        (
            Entity,
            &Collider,
            &Transform,
            Option<&RigidBody>,
            Option<&Mass>,
            Option<&InverseMass>,
            Option<&LinearVelocity>,
            Option<&AngularVelocity>,
            Option<&Elasticity>,
            Option<&Friction>,
            Option<&CenterOfMass>,
            Option<&CenterOfMassWorld>,
            Option<&InertiaTensor>,
            Option<&InverseInertiaTensor>,
            Option<&InverseInertiaTensorWorld>,
        ),
        Added<Collider>,
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
        inertia_tensor,
        inv_inertia_tensor,
        inv_inertia_tensor_world,
    ) in query.iter()
    {
        // add rigid body marker so we can query easyer
        if rigid_body.is_none() {
            commands.entity(e).insert(RigidBody);
        }

        // add mass
        if mass.is_none() {
            commands.entity(e).insert(Mass(1.0));
        }

        // add and save inv_mass for later
        let inv_mass = if inv_mass.is_none() {
            let value = if let Some(mass) = mass {
                1.0 / mass.0
            } else {
                1.0
            };
            commands.entity(e).insert(InverseMass(value));
            value
        } else {
            inv_mass.unwrap().0
        };

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

        // add and save center of mass
        let com = if center_of_mass.is_none() {
            let value = collider.get_center_of_mass();
            commands.entity(e).insert(CenterOfMass(value));
            value
        } else {
            center_of_mass.unwrap().0
        };

        if center_of_mass_world.is_none() {
            commands.entity(e).insert(CenterOfMassWorld(
                transform.translation + transform.rotation * com,
            ));
        }

        // add and save inertia_tensor
        let tensor = collider.get_inertia_tensor();
        if inertia_tensor.is_none() {
            commands.entity(e).insert(InertiaTensor(
                tensor
            ));
        }

        // add and save inv_inertia_tensor
        let inv_inertia_tensor = if inv_inertia_tensor.is_none() {
            let value = tensor.inverse() * inv_mass;
            commands.entity(e).insert(InverseInertiaTensor(value));
            value
        } else {
            inv_inertia_tensor.unwrap().0
        };

        if inv_inertia_tensor_world.is_none() {
            let orientation = Mat3::from_quat(transform.rotation);
            commands.entity(e).insert(InverseInertiaTensorWorld(
                orientation * inv_inertia_tensor * orientation.transpose(),
            ));
        }

        

        // add collider to world
        commands.entity(e).insert(collider.get_aabb());
        commands.entity(e).insert(AabbWorld::default());
    }
}

pub fn update_world_info(
    mut query: Query<(
        &mut CenterOfMassWorld,
        &mut InverseInertiaTensorWorld,
        &mut AabbWorld,
        &Transform,
        &CenterOfMass,
        &InverseInertiaTensor,
        &Aabb,
    )>,
) {
    for (
        mut center_of_mass_world,
        mut inv_inertia_tensor_wrold,
        mut aabb_world,
        trans,
        center_of_mass,
        inverse_inertia_tensor,
        aabb,
    ) in query.iter_mut()
    {
        // update com
        center_of_mass_world.0 = trans.translation + trans.rotation * center_of_mass.0;

        // update inv inertia_tensor
        
        let orientation = Mat3::from_quat(trans.rotation);
        inv_inertia_tensor_wrold.0 =
            orientation * inverse_inertia_tensor.0 * orientation.transpose();

        //update aabbworld
        aabb_world.minimums = trans.translation + aabb.minimums;
        aabb_world.maximums = trans.translation + aabb.maximums;
    }
}

fn update_time_system(time: Res<Time>, mut pt: ResMut<PhysicsTime>) {
    pt.time = time.delta_seconds();
}



pub fn gravity_system(mut query: Query<(&mut LinearVelocity, &Mass, &InverseMass), Without<Static>>, gravity: Res<Gravity>, pt: Res<PhysicsTime>) {
    for (mut linear_velocity, mass, inv_mass) in query.iter_mut(){

        // Apply Gravity, it needs to be an impluse        
        let gravey_impluse = gravity.0 * mass.0 * pt.time;

        // since rb is not static, inv mass shouldnt be 0 or less
        debug_assert!(inv_mass.0 > 0.0);

        // apply impluse
        linear_velocity.0 += gravey_impluse * inv_mass.0;
    }
}

pub fn update_transform(
    mut query: Query<
        (
            &mut Transform,
            &LinearVelocity,
            &mut AngularVelocity,
            &CenterOfMassWorld,
            &InertiaTensor,
        ),
        Without<Static>,
    >,
    pt: Res<PhysicsTime>,
) {

    for (mut transform,  linear_velocity,  mut angular_velocity, center_of_mass_world, inertia_tensor) in query.iter_mut() {    
        transform.translation += linear_velocity.0 * pt.time;

        // we have an angular velocity around the centre of mass, this needs to be converted to
        // relative body translation. This way we can properly update the rotation of the model
        let com_to_position = transform.translation - center_of_mass_world.0;

        // total torque is equal to external applied torques + internal torque (precession)
        // T = T_external + omega x I * omega
        // T_external = 0 because it was applied in the collision response function
        // T = Ia = w x I * w
        // a = I^-1 (w x I * w)
        let orientation = Mat3::from_quat(transform.rotation); // again, bevy already calc this, reuse
        let inertia_tensor_world = orientation * inertia_tensor.0 * orientation.transpose();
        let alpha = inertia_tensor_world.inverse()
            * (angular_velocity.0
                .cross(inertia_tensor_world * angular_velocity.0));
        angular_velocity.0 += alpha * pt.time;

        // update orientation
        let d_angle = angular_velocity.0 * pt.time;
        let angle = d_angle.length();
        let inv_angle = angle.recip();
        let dq = if inv_angle.is_finite() {
            Quat::from_axis_angle(d_angle * inv_angle, angle)
        } else {
            Quat::IDENTITY
        };
        transform.rotation = (dq * transform.rotation).normalize();

        // now get the new body position
        transform.translation = center_of_mass_world.0 + dq * com_to_position;
    }
}


