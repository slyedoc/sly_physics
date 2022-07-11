mod bvh;
mod colliders;
mod constraints;
mod debug;
mod gravity;
mod intersect;
mod math;
mod phases;
mod ray;
mod tlas;
mod tri;
mod types;

use bevy::{
    core::FixedTimestep, ecs::schedule::ShouldRun, math::vec3, prelude::*,
    transform::TransformSystem,
};
use bevy_inspector_egui::prelude::*;
use constraints::{cleanpup_contraints, solve_contraints, ManifoldArena};
use debug::PhysicsDebugPlugin;
use gravity::GravityPlugin;
use phases::{broadphase_system, narrow_system, resolve_system};

use bvh::*;
use colliders::*;
use ray::Ray;
use tlas::*;
use tri::BvhTri;
use types::*;

pub const PHYSISCS_TIMESTEP: f64 = 1.0 / 60.0;

const MAX_MANIFOLD_CONTACTS: usize = 4;
const MAX_SOLVE_ITERS: u32 = 5;

const BVH_BIN_COUNT: usize = 8;

// 30 rad/s is fast enough for us
const MAX_ANGULAR_SPEED: f32 = 30.0;
const MAX_ANGULAR_SPEED_SQ: f32 = MAX_ANGULAR_SPEED * MAX_ANGULAR_SPEED;

// TODO: swap out with f32::EPSILON
const EPSILON: f32 = 0.001;
const EPSILON_SQ: f32 = EPSILON * EPSILON;

pub mod prelude {
    pub use crate::{
        colliders::Collider, constraints::ManifoldArena, debug::bvh_camera::BvhCamera,
        debug::bvh_camera::PhysicsBvhCameraPlugin, debug::DebugState, ray::Ray, tlas::Tlas,
        types::AngularVelocity, types::CenterOfMass, types::Elasticity, types::Friction,
        types::InertiaTensor, types::LinearVelocity, types::Mass, types::RigidBodyMode,
        PhysicsConfig, PhysicsPlugin, PhysicsState, PhysicsSystems, RigidBodyBundle,
        PHYSISCS_TIMESTEP,
    };
}

#[derive(Bundle, Default)]
pub struct RigidBodyBundle {
    pub mode: RigidBodyMode,
    pub collider: Collider,
    pub mass: Mass,
    pub linear_velocity: LinearVelocity,
    pub angular_velocity: AngularVelocity,
    pub elasticity: Elasticity,
    pub friction: Friction,
    pub center_of_mass: CenterOfMass,
    pub inertia_tensor: InertiaTensor,
    // Will be added
    // Option<Static>, need to test removing and using filter on mode vs new component
    // Aabb,
    // AabbWorld,
    // InverseMass,
    // InverseInertiaTensor,
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
pub enum PhysicsSystems {
    Setup,
    Resolved,
}

pub struct GJKVerts(Vec<Vec3>);

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
            //.register_inspectable::<AngularVelocity>()
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
                    .label(PhysicsSystems::Setup)
                    .after(TransformSystem::TransformPropagate),
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
                    .with_system(update_bvh.after(update_world_info))
                    .with_system(update_tlas.after(update_bvh))
                    .with_system(broadphase_system.after(update_world_info))
                    .with_system(narrow_system.after(broadphase_system))
                    .with_system(solve_contraints.after(narrow_system))
                    .with_system(resolve_system.after(solve_contraints)),
            )
            .add_plugin(PhysicsDebugPlugin);
    }
}

// Note: Assuming meshes are loaded
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
    children: Query<(Option<&Children>, Option<&Handle<Mesh>>)>,
    mut tlas: ResMut<Tlas>,
    meshes: Res<Assets<Mesh>>,
    //mut convex_meshes: ResMut<Assets<ConvexMesh>>,
) {
    for (e, collider, rb_mode, mut mass, mut center_of_mass, mut inertia_tensor) in query.iter_mut()
    {
        // Setup RigidBody components
        let is_static = match rb_mode {
            RigidBodyMode::Static => {
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

        match collider {
            Collider::Sphere { radius: _ } | Collider::Cuboid { size: _ } => {
                center_of_mass.0 = collider.get_center_of_mass();

                let tensor = collider.get_inertia_tensor();
                inertia_tensor.0 = tensor;

                commands
                    .entity(e)
                    .insert(collider.get_aabb())
                    .insert(AabbWorld::default())
                    .insert(InverseInertiaTensor(tensor.inverse() * inv_mass));
            }
            Collider::ConvexHull => {
                // find all children with mesh and add them to the tlas
                let mut stack = vec![e];
                let mut verts = vec![];
                while let Some(e) = stack.pop() {
                    let (opt_children, opt_mesh) = children.get(e).unwrap();
                    if let Some(children) = opt_children {
                        for child in children.iter() {
                            stack.push(*child);
                        }
                    }
                    if let Some(h_mesh) = opt_mesh {
                        let mesh = meshes.get(h_mesh).expect("Mesh not found");

                        let mut local_verts = parse_verts(mesh);
                        verts.append(&mut local_verts);
    
                        // TODO: Could had children here, or look for child colliders here
                        // let bvh_index = tlas.add_bvh(Bvh::new(bvh_tris));
                        // tlas.add_instance(BvhInstance::new(e, bvh_index));
                    }
                }
   


                    let convex_mesh = create_mesh_from_verts(&verts);                
                    let bvh_tris = parse_bvh_tri(&convex_mesh);

                    let bvh_index = tlas.add_bvh(Bvh::new(bvh_tris));
                    tlas.add_instance(BvhInstance::new(e, bvh_index));

                    let aabb = Aabb::from_points(&verts);
                    commands.entity(e).insert(aabb);
                    commands.entity(e).insert(AabbWorld::default());

                    center_of_mass.0 = calculate_center_of_mass(&hull_points, &hull_tris);

                    let tensor =
                        calculate_inertia_tensor(&hull_points, &hull_tris, center_of_mass.0);
                    inertia_tensor.0 = tensor;
                    commands
                        .entity(e)
                        .insert(InverseInertiaTensor(tensor.inverse() * inv_mass));

            }
            //commands.entity(e).insert(collider.get_aabb());
            //commands.entity(e).insert(AabbWorld::default());
        }

        // TODO: We need a convex mesh all shapes right now, except for sphere vs sphere
        // create a convex mesh each

        // assuming a child meshes are loaded, loop though and build a convex mesh for each
        // we dont really handle children yet, for now collapse the mesh data

        // let handle = convex_meshes.add(ConvexMesh {
        //     verts: all_verts,
        //     mesh: all_bvh_tris,
        // });
        //commands.entity(e).insert(handle);
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

// TODO: both of these update system are incomplete, for now we are rebuilding every frame
// for now working on speeding up ray intersection
// will come back to this
fn update_bvh(query: Query<&GlobalTransform>, mut tlas: ResMut<Tlas>) {
    // Note: moved this into tlas since it needed 2 mutable refs within tlas, is ther another way?
    tlas.update_bvh_instances(&query);
}

fn update_tlas(mut tlas: ResMut<Tlas>) {
    tlas.build();
}

// TODO: We dont really want to copy the all tris, find better way
pub fn parse_tris(mesh: &Mesh) -> (Vec<Vec3>, Vec<TriIndexed>, Vec<BvhTri>) {
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
            for tri_indexes in indexes.chunks(3) {
                let v0 = verts[tri_indexes[0] as usize];
                let v1 = verts[tri_indexes[1] as usize];
                let v2 = verts[tri_indexes[2] as usize];
                triangles.push(BvhTri::new(
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


// TODO: We dont really want to copy the all tris, find better way
pub fn parse_verts(mesh: &Mesh) -> Vec<Vec3> {
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

            verts
        }
        _ => todo!(),
    }
}
