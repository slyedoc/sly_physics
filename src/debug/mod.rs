mod bvh_camera;
pub use bvh_camera::*;

use crate::{Aabb, AabbWorld, PhysicsFixedUpdate, PhysicsState};
use bevy::{
    prelude::*,
    render::mesh::{Indices, PrimitiveTopology},
};
use bevy_inspector_egui::Inspectable;
use iyes_loopless::prelude::*;

#[derive(Debug, Inspectable)]
pub struct DebugConfig {
    aabb: bool,
    aabb_world: bool,
    manifold: bool,
    linear_velocity: bool,
}

impl Default for DebugConfig {
    fn default() -> Self {
        Self {
            aabb: true,
            aabb_world: true,
            manifold: true,
            linear_velocity: true,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Hash, Copy)]
pub enum PhysicsDebugState {
    Running,
    Paused,
}

pub struct PhysicsDebugPlugin;
impl Plugin for PhysicsDebugPlugin {
    fn build(&self, app: &mut App) {
        app.add_loopless_state(PhysicsDebugState::Running)
            .init_resource::<DebugConfig>()
            .add_system_set_to_stage(
                PhysicsFixedUpdate,
                ConditionSet::new()
                    .run_in_state(PhysicsDebugState::Running)
                    .run_in_state(PhysicsState::Running)
                    .with_system(update_debug)
                    .into(),
            )
            .add_system_set_to_stage(
                PhysicsFixedUpdate,
                ConditionSet::new()
                    .run_in_state(PhysicsDebugState::Running)
                    .with_system(spawn_debug)
                    .into(),
            )
            .add_enter_system(PhysicsDebugState::Paused, remove_debug);
    }
}

#[derive(Component)]
pub struct ManifoldDebug;

#[derive(Component)]
pub struct AabbDebug(pub Entity);

#[derive(Component)]
pub struct AabbWorldDebug(pub Entity);

#[derive(Component)]
pub struct PhysicsDebug;

pub fn spawn_debug(
    mut commands: Commands,
    query: Query<(Entity, &AabbWorld, &Transform), Without<AabbWorldDebug>>,
    mut meshes: ResMut<Assets<Mesh>>,
) {
    for (e, aabb_world, _trans) in query.iter() {
        let id = commands
            .spawn_bundle(PbrBundle {
                //transform: Transform::from_translation(trans.translation),
                mesh: meshes.add(Mesh::from(&aabb_world.0)),
                visibility: Visibility { is_visible: true },
                ..Default::default()
            })
            .insert(PhysicsDebug)
            .insert(Name::new("Aabb World Debug"))
            .id();

        commands.entity(e).insert(AabbWorldDebug(id));
    }
}

pub fn update_debug(
    query: Query<(&AabbWorldDebug, &AabbWorld)>,
    mut mesh_query: Query<&mut Handle<Mesh>>,
    mut meshes: ResMut<Assets<Mesh>>,
) {
    for (target_aabb_world, aabb_world) in query.iter() {
        if let Ok(mut aabb_world_handle) = mesh_query.get_mut(target_aabb_world.0) {
            *aabb_world_handle = meshes.add(Mesh::from(&aabb_world.0));
        }
        //trans_target.translation = trans_parent.translation;
    }
}

pub fn remove_debug(
    mut commands: Commands,
    query: Query<Entity, With<AabbWorldDebug>>,
    remove_query: Query<Entity, With<PhysicsDebug>>,
) {
    // Remove debug components
    for e in query.iter() {
        //commands.entity(e).remove::<AabbDebug>();
        commands.entity(e).remove::<AabbWorldDebug>();
    }

    // remove debug entities
    for e in remove_query.iter() {
        commands.entity(e).despawn_recursive();
    }
}

impl From<&Aabb> for Mesh {
    fn from(aabb: &Aabb) -> Self {
        /*
              (2)-----(3)               Y
               | \     | \              |
               |  (1)-----(0) MAX       o---X
               |   |   |   |             \
          MIN (6)--|--(7)  |              Z
                 \ |     \ |
                  (5)-----(4)
        */
        let verts = vec![
            [aabb.mins.x, aabb.maxs.y, aabb.mins.z],
            [aabb.maxs.x, aabb.maxs.y, aabb.mins.z],
            [aabb.mins.x, aabb.maxs.y, aabb.maxs.z],
            [aabb.maxs.x, aabb.maxs.y, aabb.maxs.z],
            [aabb.mins.x, aabb.mins.y, aabb.mins.z],
            [aabb.maxs.x, aabb.mins.y, aabb.mins.z],
            [aabb.mins.x, aabb.mins.y, aabb.maxs.z],
            [aabb.maxs.x, aabb.mins.y, aabb.maxs.z],
        ];

        //let mut normals = Vec::with_capacity(8);
        let uvs = vec![[0.0, 0.0]; 8];
        let normals = vec![[0.0, 0.0, 1.0]; 8];

        let indices = Indices::U32(vec![
            0, 1, 1, 2, 2, 3, 3, 0, // Top ring
            4, 5, 5, 6, 6, 7, 7, 4, // Bottom ring
            0, 4, 1, 5, 2, 6, 3, 7, // Verticals
        ]);

        let mut mesh = Mesh::new(PrimitiveTopology::LineList);
        mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, verts);
        mesh.insert_attribute(Mesh::ATTRIBUTE_UV_0, uvs);
        mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, normals);
        mesh.set_indices(Some(indices));
        mesh
    }
}
