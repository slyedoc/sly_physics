mod bvh;
mod colliders;
mod constraints;
mod damping;
mod debug;
mod dynamics;
mod intersect;
mod math;
mod phases;
mod utils;
mod types;


use bevy::{math::vec3, prelude::*};
use bevy_inspector_egui::prelude::*;

use constraints::{PenetrationArena};
use iyes_loopless::prelude::*;
use phases::{broadphase_system, narrow_system, resolve_system};

use bvh::*;
use colliders::*;
use prelude::GravityPlugin;
use types::*;

pub const PHYSISCS_TIMESTEP: f64 = 1.0 / 60.0;

const MAX_MANIFOLD_CONTACTS: usize = 4;
const MAX_SOLVE_ITERS: u32 = 5; // TODO: will need to update systems if you change this currently

const BVH_BIN_COUNT: usize = 8;

// 30 rad/s is fast enough for us
const MAX_ANGULAR_SPEED: f32 = 30.0;
const MAX_ANGULAR_SPEED_SQ: f32 = MAX_ANGULAR_SPEED * MAX_ANGULAR_SPEED;

// TODO: still learning to even use epsilon, most likely using it wrong, should I swap out with f32::EPSILON?
const EPSILON: f32 = 0.001;
const EPSILON_SQ: f32 = EPSILON * EPSILON;

pub mod prelude {
    pub use crate::{
        bvh::Tlas, colliders::Collider, colliders::ColliderResources, constraints::PenetrationArena,
        debug::BvhCamera, debug::PhysicsBvhCameraPlugin, debug::PhysicsDebugPlugin,
        debug::PhysicsDebugState, bvh::Ray, types::AngularVelocity, types::CenterOfMass,
        types::Elasticity, types::Friction, types::InertiaTensor, types::LinearVelocity,
        types::Mass, types::RigidBodyMode, PhysicsConfig, PhysicsFixedUpdate, PhysicsPlugin,
        PhysicsState, PhysicsSystems, dynamics::*, RigidBodyBundle, PHYSISCS_TIMESTEP,
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
    pub damping: Damping,
    // Will be added
    // Static - if mode is static static
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
    Damping,
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
            .add_plugin(GravityPlugin)

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
                    .label(PhysicsSystems::SetupConvex)
                    .with_system(setup_convex_load)
                    .into(),
            )
            .add_system_set_to_stage(
                PhysicsFixedUpdate,
                ConditionSet::new()
                    .run_in_state(PhysicsState::Running)
                    .label(PhysicsSystems::UpdateBvh)
                    .after(PhysicsSystems::SetupConvex)
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
                    //.with_system(broadphase_system_bvh)
                    .with_system(broadphase_system)
                    .into(),
            )
            .add_system_set_to_stage(
                PhysicsFixedUpdate,
                ConditionSet::new()
                    .run_in_state(PhysicsState::Running)
                    .label(PhysicsSystems::Narrow)
                    .after(PhysicsSystems::Broad)
                    .with_system(narrow_system)
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
                .after(PhysicsSystems::ConstraintSolve0)
                .with_system(constraints::penetration_manifold::post_solve)
                .with_system(constraints::distance::post_solve)
                .into(),
        )
        .add_system_set_to_stage(
            PhysicsFixedUpdate,
            ConditionSet::new()
                .run_in_state(PhysicsState::Running)
                .label(PhysicsSystems::Damping)
                .after(PhysicsSystems::ConstraintPostSolve)
                .with_system(damping::damping_system)
                .into(),
        )
        .add_system_set_to_stage(
            PhysicsFixedUpdate,
            ConditionSet::new()
                .run_in_state(PhysicsState::Running)
                .label(PhysicsSystems::Resolve)
                .after(PhysicsSystems::Damping)
                .with_system(resolve_system)
                .into(),
        )
        .add_system_set_to_stage(
            PhysicsFixedUpdate,
            ConditionSet::new()
                .run_in_state(PhysicsState::Running)
                .label(PhysicsSystems::Update)
                .after(PhysicsSystems::Resolve)
                .with_system(update_world_info)
                .into(),
        );
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

        app
        .register_inspectable::<RigidBodyMode>()
        .register_inspectable::<LinearVelocity>()
        .register_inspectable::<Static>()
        .register_inspectable::<AngularVelocity>()
        .register_inspectable::<Elasticity>()
        .register_inspectable::<Friction>()
        .register_inspectable::<Mass>()
        .register_inspectable::<InverseMass>()
        .register_inspectable::<CenterOfMass>()
        .register_inspectable::<InertiaTensor>()
        .register_inspectable::<InverseInertiaTensor>()
        .register_inspectable::<Collider>()
        .register_inspectable::<Damping>()
        .register_inspectable::<Aabb>()
        .register_inspectable::<AabbWorld>()
        //.register_inspectable::<Bvh>()
        //.register_inspectable::<debug::BvhCamera>()
        // .register_inspectable::<Tlas>()
        // .register_inspectable::<TlasNode>()
        // .register_inspectable::<Tri>()
        .register_inspectable::<Aabb>();
    }

    fn name(&self) -> &str {
        std::any::type_name::<Self>()
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
    mut tlas: ResMut<Tlas>,
    collider_resources: Res<ColliderResources>,
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
            Collider::Sphere(index) => {
                let sphere = collider_resources.get_sphere(*index);
                center_of_mass.0 = sphere.get_center_of_mass();
                let tensor = sphere.get_inertia_tensor();
                inertia_tensor.0 = tensor;

                commands
                    .entity(e)
                    .insert(sphere.get_aabb())
                    .insert(AabbWorld::default())
                    .insert(InverseInertiaTensor(tensor.inverse() * inv_mass));

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
                let b = collider_resources.get_box(*index);
                center_of_mass.0 = b.get_center_of_mass();
                let tensor = b.get_inertia_tensor();
                inertia_tensor.0 = tensor;

                commands
                    .entity(e)
                    .insert(b.get_aabb())
                    .insert(AabbWorld::default())
                    .insert(InverseInertiaTensor(tensor.inverse() * inv_mass));

                let bvh_mesh = Mesh::from(shape::Box::new(b.size.x, b.size.y, b.size.z));
                let bvh_tri = parse_bvh_mesh(&bvh_mesh);
                let bvh_index = tlas.add_bvh(Bvh::new(bvh_tri));
                tlas.add_instance(BvhInstance::new(e, bvh_index));
            }
            Collider::Convex(_index) => {
                commands.entity(e).insert(InitConvex);
            }
        }
    }
}

#[cfg(feature = "step")]
fn stop_step(
    mut commands: Commands,
) {
    commands.insert_resource(NextState(PhysicsState::Paused));
}

#[derive(Component)]
pub struct InitConvex;

// when you added a convext mesh it may not be loaded yet, so we will loop on it till its is
fn setup_convex_load(
    mut query: Query<
        (Entity, &mut CenterOfMass, &mut InertiaTensor, &InverseMass),
        With<InitConvex>,
    >,
    children: Query<(Option<&Children>, Option<&Handle<Mesh>>)>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut commands: Commands,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut _tlas: ResMut<Tlas>,
) {
    for (e, mut center_of_mass, mut inertia_tensor, inv_mass) in query.iter_mut() {
        // find all children with mesh and save there verts
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

        // TODO: test if loaded, not ideal
        if verts.len() > 3 {
            // create a simple mesh from all verts including children
            let convex_mesh = create_convex_mesh_from_verts(&verts);
            // TODO: our bvh and, gjk sysetms use different representations of triangles,
            // that is why we have 2 representations here, should collapse this information
            let (verts, tri_indexed, _bvh_tri) = parse_mesh(&convex_mesh);
            let convex_handle = meshes.add(convex_mesh);
            commands.entity(e).with_children(|parent| {
                parent
                    .spawn_bundle(PbrBundle {
                        mesh: convex_handle,
                        material: materials.add(StandardMaterial {
                            base_color: Color::rgba(0.5, 0.5, 0.5, 1.0),
                            ..default()
                        }),
                        ..default()
                    })
                    .insert(Name::new("Collider Mesh"));
            });

            // TODO: Add bvh
            // let bvh_index = tlas.add_bvh(Bvh::new(bvh_tri));
            // tlas.add_instance(BvhInstance::new(e, bvh_index));

            let aabb = Aabb::from_points(&verts);
            commands.entity(e).insert(aabb);
            commands.entity(e).insert(AabbWorld::default());
            center_of_mass.0 = calculate_center_of_mass(&verts, &tri_indexed);
            let tensor = calculate_inertia_tensor(&verts, &tri_indexed, center_of_mass.0);
            inertia_tensor.0 = tensor;
            commands
                .entity(e)
                .insert(InverseInertiaTensor(tensor.inverse() * inv_mass.0));

            commands.entity(e).remove::<InitConvex>();
        } else {
            warn!("No verts found for convex mesh");
        }
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
fn update_bvh(query: Query<&Transform>, mut tlas: ResMut<Tlas>) {
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
