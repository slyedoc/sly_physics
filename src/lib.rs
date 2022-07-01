mod broad;
mod colliders;
mod intersect;
mod narrow;
mod resolve;
mod types;

use bevy::{core::FixedTimestep, prelude::*};
use bevy_inspector_egui::{Inspectable, RegisterInspectable};
use broad::{broadphase_system, BroadContact};
pub use colliders::*;
use narrow::{narrow_system, Contact};
use resolve::resolve_system;

pub use self::types::*;

const PHYSISCS_TIMESTEP: f64 = 1.0 / 60.0;

// 30 rad/s is fast enough for us
const MAX_ANGULAR_SPEED: f32 = 30.0;
const MAX_ANGULAR_SPEED_SQ: f32 = MAX_ANGULAR_SPEED * MAX_ANGULAR_SPEED;

const EPSILON: f32 = 0.001;
const EPSILON_SQ: f32 = EPSILON * EPSILON;

#[derive(Inspectable, Deref, DerefMut, Debug)]
pub struct Gravity(pub Vec3);
impl Default for Gravity {
    fn default() -> Self {
        Gravity(Vec3::new(0.0, -9.8, 0.0))
    }
}

#[derive(Inspectable)]
pub struct PhysicsConfig {
    //pub detection: CollisionDetection,
    pub time: f32,
}


impl Default for PhysicsConfig {
    fn default() -> Self {
        PhysicsConfig {
            //detection: CollisionDetection::Continuous,
            time: PHYSISCS_TIMESTEP as f32,
        }
    }
}


// #[derive(Inspectable, Debug, PartialEq, Eq)]
// pub enum CollisionDetection {
//     Continuous,
// }


#[derive(Debug, Clone, PartialEq, Eq, Hash, Copy)]
pub enum PhysicsState {
    Running,
    Paused
}

pub struct PhysicsPlugin;


impl Plugin for PhysicsPlugin {
    fn build(&self, app: &mut App) {
        app.add_state(PhysicsState::Running)
            .init_resource::<Gravity>()
            .add_event::<BroadContact>()
            .add_event::<Contact>()
            .init_resource::<PhysicsConfig>()
            .register_inspectable::<RBHelper>()
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
            .register_inspectable::<Aabb>()
            .register_inspectable::<AabbWorld>()
            .add_system_set(
                SystemSet::on_update(PhysicsState::Running)                
                    .with_run_criteria(FixedTimestep::step(PHYSISCS_TIMESTEP))
                    .with_system(spawn)
                    .with_system(update_world_info.after(spawn))
                    .with_system(gravity_system.after(update_world_info))
                    .with_system(broadphase_system.after(gravity_system))
                    .with_system(narrow_system.after(broadphase_system))
                    .with_system(resolve_system.after(narrow_system)),
            );
    }
}


#[derive(Bundle, Default)]
pub struct RigidbodyBundle {
    pub collider: Collider,
    pub mode: RigidBodyMode,
    pub mass: Mass,
    pub linear_velocity: LinearVelocity,
    pub angular_velocity: AngularVelocity,
    pub elasticity: Elasticity,
    pub friction: Friction,
    pub center_of_mass: CenterOfMass,
    pub inertia_tensor: InertiaTensor,
}

// I wanted to make using the bundle optional, this is setup will work if only a collider is
// added, but if you want to add more components you can, or just use the bundle
pub fn spawn(
    mut commands: Commands,
    mut query: Query<
        (

            Entity,
            &Transform,
            &Collider,
            &RigidBodyMode,
            &mut Mass,
            &mut CenterOfMass,
            &mut InertiaTensor,
        ),
        Added<Collider>,
    >,
) {
    for (
        e,
        transform,
        collider,
        rb_mode,
        mut mass,
        mut center_of_mass,
        mut inertia_tensor,
    ) in query.iter_mut()
    {
        let orientation = Mat3::from_quat(transform.rotation);

        let is_static = match rb_mode {
            RigidBodyMode::Static => {
                commands.entity(e).insert(Static);
                true
            }
            _ => false,
        };

           let inv_mass = if is_static {
            if mass.0 != 0.0 {
                //warn!("Static rigidbody with non-zero mass");
                mass.0 = 0.0;
            }
            0.0
        } else {
            1.0 / mass.0
        };

        commands.entity(e).insert(InverseMass(inv_mass));

        center_of_mass.0 = collider.get_center_of_mass();

        let tensor = collider.get_inertia_tensor();
        inertia_tensor.0 = tensor;


        let inv_inertia_tensor = tensor.inverse() * inv_mass;
        commands
            .entity(e)
            .insert(InverseInertiaTensor(inv_inertia_tensor));

        // add aabb and world aabb
        commands.entity(e).insert(collider.get_aabb());
        commands.entity(e).insert(AabbWorld::default());
    }
}

pub fn update_world_info(
    mut query: Query<(
        &mut AabbWorld,
        &Transform,
        &Aabb,
    )>,
) {
    for (mut aabb_world, trans,aabb) in query.iter_mut()
    {
        
        // update inv inertia_tensor

        // let orientation = Mat3::from_quat(trans.rotation);
        // inv_inertia_tensor_wrold.0 =
        //     orientation * inverse_inertia_tensor.0 * orientation.transpose();

        //update aabbworld
        aabb_world.minimums = trans.translation + aabb.minimums;
        aabb_world.maximums = trans.translation + aabb.maximums;
    }
}



pub fn gravity_system(
    mut query: Query<(&mut LinearVelocity, &Mass, &InverseMass), Without<Static>>,
    gravity: Res<Gravity>,
    config: Res<PhysicsConfig>,
) {
    for (mut linear_velocity, mass, inv_mass) in query.iter_mut() {
        // since rb is not static, inv mass shouldnt be 0 or less
        debug_assert!(inv_mass.0 > 0.0);

        // Apply Gravity, it needs to be an impluse
        let gravey_impluse = gravity.0 * mass.0 * config.time;
        linear_velocity.0 += gravey_impluse * inv_mass.0;
    }
}