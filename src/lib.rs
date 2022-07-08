mod broad;
mod bvh;
mod colliders;
mod constraints;
mod debug;
mod gravity;
mod intersect;
mod math;
mod narrow;
mod ray;
mod resolve;
mod tlas;
mod tri;
mod types;

use bevy::{core::FixedTimestep, ecs::schedule::ShouldRun, prelude::*, transform::TransformSystem, math::vec3, asset::LoadState};
use bevy_inspector_egui::{Inspectable, RegisterInspectable};
use broad::{broadphase_system, BroadContact};
use constraints::{cleanpup_contraints, manifold::ManifoldArena, solve_contraints};
use debug::PhysicsDebugPlugin;
use gravity::GravityPlugin;
use narrow::narrow_system;
use resolve::resolve_system;

pub use tlas::*;
pub use types::*;
pub use colliders::*;
pub use debug::DebugState;
pub use bvh::*;
pub use tri::Tri;

pub const PHYSISCS_TIMESTEP: f64 = 1.0 / 60.0;

const MAX_MANIFOLD_CONTACTS: usize = 4;
const MAX_SOLVE_ITERS: u32 = 5;

const BVH_BIN_COUNT: usize = 8;

// 30 rad/s is fast enough for us
const MAX_ANGULAR_SPEED: f32 = 30.0;
const MAX_ANGULAR_SPEED_SQ: f32 = MAX_ANGULAR_SPEED * MAX_ANGULAR_SPEED;

const EPSILON: f32 = 0.001;
const EPSILON_SQ: f32 = EPSILON * EPSILON;

pub mod prelude {
    pub use crate::{
        RigidBodyBundle,
        Collider,
        RigidBody,
        Mass,
        LinearVelocity,
        AngularVelocity,
        Elasticity,
        Friction,
        CenterOfMass,
        InertiaTensor,
    };
    

}

#[derive(Bundle, Default)]
pub struct RigidBodyBundle {
    pub collider: Collider,
    pub mode: RigidBody,
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

#[derive(Debug, Clone, PartialEq, Eq, Hash, SystemLabel)]
pub enum BvhSystems {
    Setup
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
            .init_resource::<Tlas>()
            //.register_inspectable::<Static>()
            //.register_inspectable::<LinearVelocity>()
            // .register_inspectable::<AngularVelocity>()
            // .register_inspectable::<Elasticity>()
            // .register_inspectable::<Friction>()
            // .register_inspectable::<Mass>()
            // .register_inspectable::<InverseMass>()
            // .register_inspectable::<CenterOfMass>()
            // .register_inspectable::<InertiaTensor>()
            // .register_inspectable::<InverseInertiaTensor>()
            // .register_inspectable::<Collider>()
            // .register_inspectable::<Aabb>()
            // .register_inspectable::<AabbWorld>()
            
            // .register_inspectable::<Bvh>()
            // .register_inspectable::<BvhCamera>()
            // .register_inspectable::<Tlas>()
            // .register_inspectable::<TlasNode>()
            // .register_inspectable::<Tri>()
            // .register_inspectable::<Aabb>()
            .add_system_set_to_stage(
                CoreStage::PostUpdate,
                SystemSet::new()
                    .label(BvhSystems::Setup)
                    .after(TransformSystem::TransformPropagate)
                    .with_system(spawn_bvh)
                    .with_system(spawn_bvh_with_children)
                    .with_system(
                        update_bvh
                            .after(spawn_bvh)
                            .after(spawn_bvh_with_children),
                    )
                    .with_system(update_tlas.after(update_bvh)),
            )
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
            &RigidBody,
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

fn spawn_bvh(
    mut commands: Commands,
    meshes: Res<Assets<Mesh>>,
    query: Query<(Entity, &Handle<Mesh>), With<BvhInit>>,
    mut tlas: ResMut<Tlas>,
) {
    for (e, handle) in query.iter() {
        // let loaded = server.get_load_state(handle.id);
        let mesh = meshes.get(handle).expect("Mesh not found");
        let tris = parse_mesh(mesh);
        // mesh..ins(
        //     ATTRIBUTE_BLEND_COLOR,
        //     // The cube mesh has 24 vertices (6 faces, 4 vertices per face), so we insert one BlendColor for each
        //     vec![[1.0, 0.0, 0.0, 1.0]; 24],
        // );

        let bvh_index = tlas.add_bvh(Bvh::new(tris));
        tlas.add_instance(BvhInstance::new(e, bvh_index));
        commands.entity(e).remove::<BvhInit>();
    }
}

#[allow(clippy::type_complexity)]
fn spawn_bvh_with_children(
    mut commands: Commands,
    meshes: Res<Assets<Mesh>>,
    query: Query<(Entity, &BvhInitWithChildren)>,
    children: Query<(Entity, Option<&Children>, Option<&Handle<Mesh>>)>,
    server: Res<AssetServer>,
    mut tlas: ResMut<Tlas>,
) {
    for (root, scene) in query.iter() {
        let load_state = server.get_load_state(scene.0.id);
        if load_state != LoadState::Loaded {
            continue;
        }

        let mut stack = vec![root];
        while let Some(e) = stack.pop() {
            let (e, opt_children, opt_mesh) = children.get(e).unwrap();
            if let Some(children) = opt_children {
                for child in children.iter() {
                    stack.push(*child);
                }
            }
            if let Some(h_mesh) = opt_mesh {
                let mesh = meshes.get(h_mesh).expect("Mesh not found");
                let tris = parse_mesh(mesh);

                let bvh_index = tlas.add_bvh(Bvh::new(tris));
                tlas.add_instance(BvhInstance::new(e, bvh_index));
            }
        }

        commands.entity(root).remove::<BvhInitWithChildren>();
    }
}

// TODO: both of these update system are incomplete, for now we are rebuilding every frame
// for now working on speeding up ray intersection
// will come back to this
 fn update_bvh(query: Query<&GlobalTransform>, mut tlas: ResMut<Tlas>) {
    // Note: moved fn into tlas self to since it needed 2 mutable refs within the tlas
    tlas.update_bvh_instances(&query);
    
}

fn update_tlas(mut tlas: ResMut<Tlas>) {
    tlas.build();
}

// Markers
#[derive(Component)]
pub struct BvhInit;
#[derive(Component)]
pub struct BvhInitWithChildren(pub Handle<Scene>);

// TODO: We dont really want to copy the all tris, find better way
pub fn parse_mesh(mesh: &Mesh) -> Vec<Tri> {
    match mesh.primitive_topology() {
        bevy::render::mesh::PrimitiveTopology::TriangleList => {
            let indexes = match mesh.indices().expect("No Indices") {
                bevy::render::mesh::Indices::U32(vec) => vec,
                _ => todo!(),
            };

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

            let mut triangles = Vec::with_capacity(indexes.len() / 3);
            for tri_indexes in indexes.chunks(3)
             {
                let v0 = verts[tri_indexes[0] as usize];
                let v1 = verts[tri_indexes[1] as usize];
                let v2 = verts[tri_indexes[2] as usize];
                triangles.push(Tri::new(
                    vec3(v0[0], v0[1], v0[2]),
                    vec3(v1[0], v1[1], v1[2]),
                    vec3(v2[0], v2[1], v2[2]),
                ));
            }
            triangles
        }
        _ => todo!(),
    }
}
