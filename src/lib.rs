mod broad;
mod colliders;
mod constraints;
mod debug;
mod gravity;
mod intersect;
mod math;
mod narrow;
mod resolve;
mod types;

use bevy::{core::FixedTimestep, ecs::schedule::ShouldRun, prelude::*};
use bevy_inspector_egui::{Inspectable, RegisterInspectable};
use broad::{broadphase_system, BroadContact};
pub use colliders::*;
use constraints::{cleanpup_contraints, manifold::ManifoldArena, solve_contraints};
use debug::PhysicsDebugPlugin;
use gravity::GravityPlugin;
use narrow::narrow_system;
use resolve::resolve_system;

pub use self::types::*;
pub use debug::DebugState;

pub const PHYSISCS_TIMESTEP: f64 = 1.0 / 60.0;

const MAX_MANIFOLD_CONTACTS: usize = 4;
const MAX_SOLVE_ITERS: u32 = 5;

// 30 rad/s is fast enough for us
const MAX_ANGULAR_SPEED: f32 = 30.0;
const MAX_ANGULAR_SPEED_SQ: f32 = MAX_ANGULAR_SPEED * MAX_ANGULAR_SPEED;

const EPSILON: f32 = 0.001;
const EPSILON_SQ: f32 = EPSILON * EPSILON;

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
    // Added for you
    // InverseMass, InverseInertiaTensor,
}

#[derive(Debug, Clone, PartialEq, Eq, Hash, Copy)]
pub enum PhysicsState {
    Running,
    Paused,
}

#[derive(Inspectable)]
pub struct PhysicsConfig {
    pub time: f32,
}

impl Default for PhysicsConfig {
    fn default() -> Self {
        PhysicsConfig {
            time: PHYSISCS_TIMESTEP as f32,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Hash, SystemLabel)]
enum PhysicsSystems {
    Resolved,
}

pub struct PhysicsPlugin;

impl Plugin for PhysicsPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugin(GravityPlugin)
            .add_state(PhysicsState::Running)
            .add_event::<BroadContact>()
            .add_event::<Contact>()
            .init_resource::<PhysicsConfig>()
            .init_resource::<ManifoldArena>()
            .register_inspectable::<RBHelper>()
            .register_inspectable::<Static>()
            .register_inspectable::<LinearVelocity>()
            .register_inspectable::<AngularVelocity>()
            .register_inspectable::<Elasticity>()
            .register_inspectable::<Friction>()
            .register_inspectable::<Mass>()
            .register_inspectable::<InverseMass>()
            .register_inspectable::<CenterOfMass>()
            .register_inspectable::<InertiaTensor>()
            .register_inspectable::<InverseInertiaTensor>()
            .register_inspectable::<Collider>()
            .register_inspectable::<Aabb>()
            .register_inspectable::<AabbWorld>()
            .add_system_set_to_stage(
                CoreStage::PostUpdate,
                SystemSet::new()
                    .label(PhysicsSystems::Resolved)
                    // workaround since you cant chain with_run_criteria, see https://github.com/bevyengine/bevy/issues/1839
                    .with_run_criteria(FixedTimestep::step(PHYSISCS_TIMESTEP as f64).chain(
                        |In(input): In<ShouldRun>, state: Res<State<PhysicsState>>| {
                            if state.current() == &PhysicsState::Running {
                                input
                            } else {
                                ShouldRun::No
                            }
                        },
                    ))
                    .with_system(cleanpup_contraints.before(solve_contraints))
                    .with_system(spawn)
                    .with_system(update_world_info.after(spawn))
                    .with_system(broadphase_system.after(update_world_info))
                    .with_system(narrow_system.after(broadphase_system))
                    .with_system(solve_contraints.after(narrow_system))
                    .with_system(resolve_system.after(solve_contraints)),
            )
            .add_plugin(PhysicsDebugPlugin);
    }
}

pub fn spawn(
    mut commands: Commands,
    mut query: Query<
        (
            Entity,
            &Collider,
            &RigidBodyMode,
            &mut Mass,
            &mut CenterOfMass,
            &mut InertiaTensor,
        ),
        (Added<Collider>, With<Transform>),
    >,
) {
    for (e, collider, rb_mode, mut mass, mut center_of_mass, mut inertia_tensor) in query.iter_mut()
    {
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

pub fn update_world_info(mut query: Query<(&Transform, &Aabb, &mut AabbWorld)>) {
    for (trans, aabb, mut aabb_world) in query.iter_mut() {
        //update aabbworld

        let b = aabb.get_world_aabb(trans);
        aabb_world.0.mins = b.mins;
        aabb_world.0.maxs = b.maxs;
    }
}
