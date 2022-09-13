mod bvh;

mod colliders;
mod constraints;
mod drag;
mod debug;
mod dynamics;
mod intersect;
mod math;
mod phases;
mod types;
mod utils;

use bevy::{math::vec3, prelude::*};
use bevy_inspector_egui::prelude::*;

use constraints::PenetrationArena;
use iyes_loopless::prelude::*;
use phases::{broad_phase, narrow_phase, resolve_phase};

use bvh::*;
use colliders::*;
use types::*;

pub const PHYSISCS_TIMESTEP: f64 = 1.0 / 60.0;

const MAX_MANIFOLD_CONTACTS: usize = 4;
const MAX_SOLVE_ITERS: u32 = 5; // TODO: only valid 1-5

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
        bvh::Ray, bvh::Tlas, colliders::*, constraints::PenetrationArena, debug::BvhCamera,
        debug::PhysicsBvhCameraPlugin, debug::PhysicsDebugPlugin, debug::PhysicsDebugState,
        dynamics::*, types::Aabb, types::RBHelper, types::CenterOfMass, types::Elasticity,
        types::Friction, types::InertiaTensor, types::Velocity, types::Mass, types::InverseMass, types::InverseInertiaTensor,
        types::RigidBody, PhysicsConfig, PhysicsFixedUpdate, PhysicsPlugin, PhysicsState,
        PhysicsSystems, RigidBodyBundle, PHYSISCS_TIMESTEP,
    };
}

#[derive(Bundle, Default)]
pub struct RigidBodyBundle {
    pub mode: RigidBody,
    pub collider: Collider,
    pub mass: Mass,
    pub velocity: Velocity,
    pub elasticity: Elasticity,
    pub friction: Friction,
    pub center_of_mass: CenterOfMass,
    pub inertia_tensor: InertiaTensor,
    pub damping: Drag,
    pub aabb: Aabb,
    pub inverse_inertia_tensor: InverseInertiaTensor,
    // Will be added
    // Static - if mode is static static

    // InverseMass,
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
    SetupConvex,
    Update,
    UpdateBvh,
    Dynamics,
    Broad,
    Narrow,
    ConstraintPreSolve,
    ConstraintSolve0,
    ConstraintSolve1,
    ConstraintSolve2,
    ConstraintSolve3,
    ConstraintSolve4,
    ConstraintPostSolve,
    Drag,
    Resolve,
    Camera,
    Step,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, StageLabel)]
pub struct PhysicsFixedUpdate;

pub struct PhysicsPlugin;

impl Plugin for PhysicsPlugin {
    fn build(&self, app: &mut App) {
        app.add_loopless_state(PhysicsState::Running)
            .add_event::<BroadContact>()
            .add_event::<Contact>()
            .init_resource::<PhysicsConfig>()
            .init_resource::<ColliderResources>()
            .init_resource::<PenetrationArena>()
            .init_resource::<Tlas>()
            .add_stage_after(
                CoreStage::Update,
                PhysicsFixedUpdate,
                SystemStage::parallel(),
            )
            .add_system_set_to_stage(
                PhysicsFixedUpdate,
                ConditionSet::new()
                    .run_in_state(PhysicsState::Running)
                    .label(PhysicsSystems::Setup)
                    .with_system(spawn)
                    .into(),
            )
            .add_system_set_to_stage(
                PhysicsFixedUpdate,
                ConditionSet::new()
                    .run_in_state(PhysicsState::Running)
                    .label(PhysicsSystems::Update)
                    .after(PhysicsSystems::Setup)
                    .with_system(update_aabb)
                    .into(),
            )
            .add_system_set_to_stage(
                PhysicsFixedUpdate,
                ConditionSet::new()
                    .run_in_state(PhysicsState::Running)
                    .label(PhysicsSystems::UpdateBvh)
                    .after(PhysicsSystems::Update)
                    .with_system(update_bvh)
                    .into(),
            )
            // phases
            .add_system_set_to_stage(
                PhysicsFixedUpdate,
                ConditionSet::new()
                    .run_in_state(PhysicsState::Running)
                    .label(PhysicsSystems::Broad)
                    .after(PhysicsSystems::UpdateBvh)
                    .with_system(broad_phase)
                    .into(),
            )
            .add_system_set_to_stage(
                PhysicsFixedUpdate,
                ConditionSet::new()
                    .run_in_state(PhysicsState::Running)
                    .label(PhysicsSystems::Narrow)
                    .after(PhysicsSystems::Broad)
                    .with_system(narrow_phase)
                    .into(),
            )
            // Contraints
            .add_system_set_to_stage(
                PhysicsFixedUpdate,
                ConditionSet::new()
                    .run_in_state(PhysicsState::Running)
                    .label(PhysicsSystems::ConstraintPreSolve)
                    .after(PhysicsSystems::Narrow)
                    .with_system(constraints::penetration_manifold::pre_solve)
                    .with_system(constraints::distance::pre_solve)
                    .into(),
            );

        // TODO: find better way to reuse systems
        for i in 0..MAX_SOLVE_ITERS {
            app.add_system_set_to_stage(
                PhysicsFixedUpdate,
                ConditionSet::new()
                    .run_in_state(PhysicsState::Running)
                    .label(match i {
                        0 => PhysicsSystems::ConstraintSolve0,
                        1 => PhysicsSystems::ConstraintSolve1,
                        2 => PhysicsSystems::ConstraintSolve2,
                        3 => PhysicsSystems::ConstraintSolve3,
                        4 => PhysicsSystems::ConstraintSolve4,
                        _ => unreachable!(),
                    })
                    .after(match i {
                        0 => PhysicsSystems::ConstraintPreSolve,
                        1 => PhysicsSystems::ConstraintSolve0,
                        2 => PhysicsSystems::ConstraintSolve1,
                        3 => PhysicsSystems::ConstraintSolve2,
                        4 => PhysicsSystems::ConstraintSolve3,
                        _ => unreachable!(),
                    })
                    .with_system(constraints::penetration_manifold::solve)
                    .with_system(constraints::distance::solve)
                    .into(),
            );
        }
        app.add_system_set_to_stage(
            PhysicsFixedUpdate,
            ConditionSet::new()
                .run_in_state(PhysicsState::Running)
                .label(PhysicsSystems::ConstraintPostSolve)
                .after(match MAX_SOLVE_ITERS {
                    0 => unreachable!(),
                    1 => PhysicsSystems::ConstraintSolve0,
                    2 => PhysicsSystems::ConstraintSolve1,
                    3 => PhysicsSystems::ConstraintSolve2,
                    4 => PhysicsSystems::ConstraintSolve3,
                    5 => PhysicsSystems::ConstraintSolve4,
                    _ => unreachable!(),
                })
                .with_system(constraints::penetration_manifold::post_solve)
                .with_system(constraints::distance::post_solve)
                .into(),
        )
        .add_system_set_to_stage(
            PhysicsFixedUpdate,
            ConditionSet::new()
                .run_in_state(PhysicsState::Running)
                .label(PhysicsSystems::Drag)
                .after(PhysicsSystems::ConstraintPostSolve)
                .with_system(drag::drag_system)
                .into(),
        )
        .add_system_set_to_stage(
            PhysicsFixedUpdate,
            ConditionSet::new()
                .run_in_state(PhysicsState::Running)
                .label(PhysicsSystems::Resolve)
                .after(PhysicsSystems::Drag)
                .with_system(resolve_phase)
                .into(),
        );

        app.register_type::<Velocity>();
        // registry.register::<RigidBodyMode>();
        // registry.register::<LinearVelocity>();
        // registry.register::<Static>();
        // registry.register::<AngularVelocity>();
        // registry.register::<Elasticity>();
        // registry.register::<Friction>();
        // registry.register::<Mass>();
        // registry.register::<InverseMass>();
        // registry.register::<CenterOfMass>();
        // registry.register::<InertiaTensor>();
        // registry.register::<InverseInertiaTensor>();
        // registry.register::<Collider>();
        // registry.register::<Drag>();
        // registry.register::<Aabb>();
        // registry.register::<AabbWorld>();
        // // .register_inspectable::<Bvh>()
        // // .register_inspectable::<debug::BvhCamera>()
        // // .register_inspectable::<Tlas>()
        // // .register_inspectable::<TlasNode>()
        // //.register_inspectable::<Tri>()
        // registry.register::<Aabb>();

        #[cfg(feature = "step")]
        app.add_system_set_to_stage(
            PhysicsFixedUpdate,
            ConditionSet::new()
                .run_in_state(PhysicsState::Running)
                .label(PhysicsSystems::Step)
                .after(PhysicsSystems::Resolve)
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
            &Collider,
            &RigidBody,
            &mut Mass,
            &mut CenterOfMass,
            &mut InertiaTensor,
            &mut InverseInertiaTensor,
        ),
        Added<Collider>,
    >,
    mut tlas: ResMut<Tlas>,
    collider_resources: Res<ColliderResources>,
) {
    for (
        e,
        collider,
        rb_mode,
        mut mass,
        mut center_of_mass,
        mut inertia_tensor,
        mut inverse_inertia_tensor,
    ) in query.iter_mut()
    {
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

        match collider {
            Collider::Sphere(index) => {
                let sphere = collider_resources.get_sphere(*index);
                center_of_mass.0 = sphere.get_center_of_mass();
                inertia_tensor.0 = sphere.get_inertia_tensor();
                inverse_inertia_tensor.0 = inertia_tensor.0.inverse() * inv_mass;

                let bvh_mesh = Mesh::from(shape::UVSphere {
                    radius: sphere.radius,
                    sectors: 6,
                    stacks: 6,
                });

                let bvh_tri = parse_bvh_mesh(&bvh_mesh);
                //info!("e: {:?} bvh_tri: {:?}", e, bvh_tri);
                let bvh_index = tlas.add_bvh(Bvh::new(bvh_tri));
                tlas.add_instance(BvhInstance::new(e, bvh_index));
            }
            Collider::Box(index) => {
                let cube = collider_resources.get_cube(*index);
                center_of_mass.0 = cube.get_center_of_mass();
                inertia_tensor.0 = cube.get_inertia_tensor();
                inverse_inertia_tensor.0 = inertia_tensor.0.inverse() * inv_mass;

                let bvh_mesh = Mesh::from(shape::Box::new(cube.size.x, cube.size.y, cube.size.z));
                let bvh_tri = parse_bvh_mesh(&bvh_mesh);
                let bvh_index = tlas.add_bvh(Bvh::new(bvh_tri));
                tlas.add_instance(BvhInstance::new(e, bvh_index));
            }
            Collider::Convex(index) => {
                let convex = collider_resources.get_convex(*index);
                center_of_mass.0 = convex.get_center_of_mass();
                inertia_tensor.0 = convex.get_inertia_tensor();
                inverse_inertia_tensor.0 = inertia_tensor.0.inverse() * inv_mass;

                let bvh_mesh = Mesh::from(convex);
                let bvh_tri = parse_bvh_mesh(&bvh_mesh);
                let bvh_index = tlas.add_bvh(Bvh::new(bvh_tri));
                tlas.add_instance(BvhInstance::new(e, bvh_index));
            }
        }
    }
}

#[cfg(feature = "step")]
fn stop_step(mut commands: Commands) {
    commands.insert_resource(NextState(PhysicsState::Paused));
}

pub fn update_aabb(
    mut query: Query<(&Transform, &mut Aabb, &Collider, &Velocity)>,
    config: Res<PhysicsConfig>,
    collider_resources: Res<ColliderResources>,
) {

    for (trans, mut aabb, collider, lin_vel) in query.iter_mut() {
        //update aabbworld
        *aabb = match collider {
            Collider::Sphere(index) => {
                collider_resources
                    .get_sphere(*index)
                    .get_world_aabb(trans, lin_vel, config.time)
            }
            Collider::Box(index) => {
                collider_resources
                    .get_cube(*index)
                    .get_world_aabb(trans, lin_vel, config.time)
            }
            Collider::Convex(index) => {
                collider_resources
                    .get_convex(*index)
                    .get_world_aabb(trans, lin_vel, config.time)
            }
        };
    }
}

// TODO: both of these update system are incomplete, for now we are rebuilding every frame
fn update_bvh(query: Query<(&Transform, &Aabb)>, mut tlas: ResMut<Tlas>) {
    tlas.update_bvh_instances(&query);
    tlas.build();


}

// TODO: We dont really want to copy the all tris twice, find better way
pub fn parse_mesh(mesh: &Mesh) -> (Vec<Vec3>, Vec<TriIndexed>, Vec<BvhTri>) {
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

            let mut tri_indexed = Vec::with_capacity(indexes.len() / 3);
            let mut tri_bvh = Vec::with_capacity(indexes.len() / 3);
            for tri_indexes in indexes.chunks(3) {
                let v0 = verts[tri_indexes[0] as usize];
                let v1 = verts[tri_indexes[1] as usize];
                let v2 = verts[tri_indexes[2] as usize];
                tri_bvh.push(BvhTri::new(
                    vec3(v0[0], v0[1], v0[2]),
                    vec3(v1[0], v1[1], v1[2]),
                    vec3(v2[0], v2[1], v2[2]),
                ));
                tri_indexed.push(TriIndexed {
                    a: tri_indexes[0],
                    b: tri_indexes[1],
                    c: tri_indexes[2],
                });
            }

            (verts, tri_indexed, tri_bvh)
        }
        _ => todo!(),
    }
}

// TODO: We dont really want to copy the all tris, find better way
pub fn parse_bvh_mesh(mesh: &Mesh) -> Vec<BvhTri> {
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
