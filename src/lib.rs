#![allow(clippy::too_many_arguments, clippy::type_complexity)]

mod bvh;
mod colliders;
mod constraints;
mod debug;
mod dynamics;
mod intersect;
mod math;
mod phases;
mod types;
mod utils;

use bevy::{ecs::query::WorldQuery, math::vec3, prelude::*};
use bevy_inspector_egui::prelude::*;

use iyes_loopless::prelude::*;
use phases::*;

use bvh::*;
use colliders::*;
use prelude::PhysicsConstraintsPlugin;
use types::*;

pub const PHYSICS_TIMESTEP: f64 = 1.0 / 60.0;

const MAX_MANIFOLD_CONTACTS: usize = 4;
const BVH_BIN_COUNT: usize = 8;

// 30 rad/s is fast enough for us
const MAX_ANGULAR_SPEED: f32 = 30.0;
const MAX_ANGULAR_SPEED_SQ: f32 = MAX_ANGULAR_SPEED * MAX_ANGULAR_SPEED;

// TODO: still learning to even use epsilon, most likely using it wrong, should I swap out with f32::EPSILON?
const EPSILON: f32 = 0.001;
const EPSILON_SQ: f32 = EPSILON * EPSILON;

const BOUNDS_EPS: f32 = 0.01;

pub mod prelude {
    pub use crate::{
        bvh::Ray, bvh::Tlas, colliders::*, constraints::*, debug::BvhCamera,
        debug::DebugBvhCameraPlugin, debug::DebugEntityAabbPlugin, debug::PhysicsDebugPlugin,
        debug::PhysicsDebugState, dynamics::*, types::Aabb, types::CenterOfMass, types::Elasticity,
        types::Friction, types::InertiaTensor, types::InverseInertiaTensor, types::InverseMass,
        types::Mass, types::RBHelper, types::RigidBody, types::Velocity, PhysicsConfig,
        PhysicsFixedUpdate, PhysicsPlugin, PhysicsState, PhysicsSystem, RigidBodyBundle,
        PHYSICS_TIMESTEP,
    };
}

#[derive(Bundle, Default)]
pub struct RigidBodyBundle {
    pub mode: RigidBody,
    pub collider: Handle<Collider>,
    pub mass: Mass,
    pub velocity: Velocity,
    pub elasticity: Elasticity,
    pub friction: Friction,
    pub center_of_mass: CenterOfMass,
    pub inertia_tensor: InertiaTensor,
    pub inverse_inertia_tensor: InverseInertiaTensor,
    pub inverse_transform: InverseTransform,
    pub aabb: Aabb,
    // Will be added

    // Static - if mode is static static
    // InverseMass,
}

#[derive(WorldQuery)]
#[world_query(mutable, derive(Debug))]
pub struct RBQuery {
    entity: Entity,
    transform: &'static mut Transform,
    velocity: &'static mut Velocity,
    inv_mass: &'static InverseMass,
    inv_inertia_tensor: &'static InverseInertiaTensor,
    center_of_mass: &'static CenterOfMass,
}

#[derive(WorldQuery)]
#[world_query(derive(Debug))]
pub struct TlasQuery {
    entity: Entity,
    transform: &'static GlobalTransform,    
    collider: &'static Handle<Collider>,
    inv_trans: &'static InverseTransform,
    aabb: &'static Aabb,    
}

#[derive(Debug, Clone, PartialEq, Eq, Hash, Copy)]
pub enum PhysicsState {
    Running,
    Paused,
}

#[derive(Inspectable)]
pub struct PhysicsConfig {
    pub time: f32,
    pub solver_iterations: u32,
}

impl Default for PhysicsConfig {
    fn default() -> Self {
        PhysicsConfig {
            time: PHYSICS_TIMESTEP as f32,
            solver_iterations: 5,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Hash, SystemLabel)]
pub enum PhysicsSystem {
    Setup,
    Update,
    UpdateTlas,
    Dynamics,
    Broad,
    Narrow,
    Resolve,
    #[cfg(feature = "step")]
    Step,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, StageLabel)]
pub struct PhysicsFixedUpdate;

pub struct PhysicsPlugin;

impl Plugin for PhysicsPlugin {
    fn build(&self, app: &mut App) {
        app.add_loopless_state(PhysicsState::Running)
            .add_stage_after(
                CoreStage::PostUpdate,
                PhysicsFixedUpdate,
                SystemStage::parallel(),
            )
            .add_plugin(PhysicsConstraintsPlugin)
            .add_asset::<Collider>()
            .add_event::<BroadContact>()
            .add_event::<Contact>()
            .add_event::<ManifoldContact>()
            .init_resource::<PhysicsConfig>()
            .init_resource::<Tlas>()
            .add_system_set_to_stage(
                PhysicsFixedUpdate,
                ConditionSet::new()
                    .run_in_state(PhysicsState::Running)
                    .label(PhysicsSystem::Setup)
                    .with_system(spawn)
                    .into(),
            )
            .add_system_set_to_stage(
                PhysicsFixedUpdate,
                ConditionSet::new()
                    .run_in_state(PhysicsState::Running)
                    .label(PhysicsSystem::Update)
                    .after(PhysicsSystem::Setup)
                    .with_system(update_aabb)
                    .into(),
            )
            .add_system_set_to_stage(
                PhysicsFixedUpdate,
                ConditionSet::new()
                    .run_in_state(PhysicsState::Running)
                    .after(PhysicsSystem::Update)
                    .with_system(update_tlas)
                    .into(),
            )
            // Dynamic Plugins add systems here
            .add_system_set_to_stage(
                PhysicsFixedUpdate,
                ConditionSet::new()
                    .run_in_state(PhysicsState::Running)
                    .label(PhysicsSystem::Broad)
                    .after(PhysicsSystem::Update)
                    .with_system(broad_phase)
                    //.with_system(broad_phase_bvh)
                    .into(),
            )
            .add_system_set_to_stage(
                PhysicsFixedUpdate,
                ConditionSet::new()
                    .run_in_state(PhysicsState::Running)
                    .label(PhysicsSystem::Narrow)
                    .after(PhysicsSystem::Broad)
                    .with_system(narrow_phase)
                    .into(),
            )
            // Constraints Plugin adds systems here
            .add_system_set_to_stage(
                PhysicsFixedUpdate,
                ConditionSet::new()
                    .run_in_state(PhysicsState::Running)
                    .label(PhysicsSystem::Resolve)
                    .after(PhysicsSystem::Broad)
                    .with_system(resolve_phase)
                    .into(),
            )
            .register_type::<RigidBody>()
            .register_type::<Velocity>()
            .register_type::<Aabb>()
            .register_type::<Mass>()
            .register_type::<InverseMass>()
            .register_type::<Elasticity>()
            .register_type::<Friction>()
            .register_type::<CenterOfMass>()
            .register_type::<InertiaTensor>()
            .register_type::<InverseInertiaTensor>();

        #[cfg(feature = "step")]
        app.add_system_set_to_stage(
            PhysicsFixedUpdate,
            ConditionSet::new()
                .run_in_state(PhysicsState::Running)
                .label(PhysicsSystem::Step)
                .after(PhysicsSystem::Resolve)
                .with_system(stop_step)
                .into(),
        );
    }
}

// Note: Assuming meshes are loaded
#[allow(clippy::type_complexity)]
pub fn spawn(
    mut commands: Commands,
    mut query: Query<
        (
            Entity,
            &Handle<Collider>,
            &RigidBody,
            &mut Mass,
            &mut CenterOfMass,
            &mut InertiaTensor,
            &mut InverseInertiaTensor,
        ),
        Added<Handle<Collider>>,
    >,
    colliders: Res<Assets<Collider>>,
) {
    for (
        e,
        collider_handle,
        rb_mode,
        mut mass,
        mut center_of_mass,
        mut inertia_tensor,
        mut inverse_inertia_tensor,
    ) in query.iter_mut()
    {
        let collider = colliders.get(collider_handle).unwrap();

        // Setup RigidBody components
        let is_static = match rb_mode {
            RigidBody::Static => {
                commands.entity(e).insert(Static);
                true
            }
            _ => false,
        };

        let inv_mass = if is_static {
            if mass.0 != 0.0 {
                mass.0 = 0.0;
            }
            0.0
        } else {
            1.0 / mass.0
        };

        commands.entity(e).insert(InverseMass(inv_mass));

        center_of_mass.0 = collider.get_center_of_mass();
        inertia_tensor.0 = collider.get_inertia_tensor();
        inverse_inertia_tensor.0 = inertia_tensor.0.inverse() * inv_mass;
    }
}

#[cfg(feature = "step")]
fn stop_step(mut commands: Commands) {
    commands.insert_resource(NextState(PhysicsState::Paused));
}

pub fn update_aabb(
    mut query: Query<(&GlobalTransform, &mut InverseTransform,  &mut Aabb, &Handle<Collider>, &Velocity)>,
    config: Res<PhysicsConfig>,
    colliders: Res<Assets<Collider>>,
) {
    for (trans, mut inv_trans, mut aabb, col, lin_vel) in query.iter_mut() {
        let collider = colliders.get(col).unwrap();
        *aabb = collider.get_world_aabb(trans, lin_vel, config.time);
        inv_trans.0 = trans.compute_matrix().inverse();
    }
}

// TODO: this update system are incomplete, for now rebuilding every frame
#[allow(dead_code)]
fn update_tlas(
    query: Query<TlasQuery>,
    mut tlas: ResMut<Tlas>,
) {

    // Build Tlas
    tlas.nodes.clear();

    // reserve root node
    tlas.nodes.push(TlasNode::default());

    let mut node_index = vec![0u32;  query.iter().count() + 1];
    let mut node_indices = query.iter().count() as i32;

    // assign a TLASleaf node for each BLAS and build node indexes
    for (i, blas) in query.iter().enumerate() {
        node_index[i] = i as u32 + 1;
        tlas.nodes.push(TlasNode::Leaf(Leaf {
            entity: Some(blas.entity),
        }));
    }

    // use agglomerative clustering to build the TLAS
    let mut a = 0i32;
    let mut b = tlas.find_best_match(&node_index, node_indices, a, &query);
    while node_indices > 1 {
        let c = tlas.find_best_match(&node_index, node_indices, b, &query);
        if a == c {
            let node_index_a = node_index[a as usize];
            let node_index_b = node_index[b as usize];
            let node_a = &tlas.nodes[node_index_a as usize];
            let node_b = &tlas.nodes[node_index_b as usize];
            let aabb_a = node_a.get_aabb(&query);
            let aabb_b = node_b.get_aabb(&query);
            let aabb = aabb_a + aabb_b;

            tlas.nodes.push(TlasNode::Trunk(Trunk {
                aabb,                
                left: node_index_a as u16,
                right: node_index_b as u16,
            }));
            node_index[a as usize] = tlas.nodes.len() as u32 - 1;
            node_index[b as usize] = node_index[node_indices as usize - 1];
            node_indices -= 1;
            b = tlas.find_best_match(&node_index, node_indices, a, &query);
        } else {
            a = b;
            b = c;
        }
    }
    // update root node
    tlas.nodes.swap(0, node_index[a as usize] as usize);   
    tlas.nodes.remove(node_index[a as usize] as usize);
}

// TODO: We don't really want to copy the all tris, find better way
pub fn parse_verts(mesh: &Mesh) -> Vec<Vec3> {
    match mesh.primitive_topology() {
        bevy::render::mesh::PrimitiveTopology::TriangleList => {
            let verts = match mesh
                .attribute(Mesh::ATTRIBUTE_POSITION)
                .expect("No Position Attribute")
            {
                bevy::render::mesh::VertexAttributeValues::Float32x3(vec) => {
                    vec.iter().map(|vec| vec3(vec[0], vec[1], vec[2]))
                }
                _ => todo!(),
            }
            .collect::<Vec<_>>();

            verts
        }
        _ => todo!(),
    }
}