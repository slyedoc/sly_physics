use crate::{Aabb, AabbWorld, PhysicsSystems};
use bevy::{
    prelude::*,
    render::mesh::{Indices, PrimitiveTopology},
};

#[derive(Debug, Clone, PartialEq, Eq, Hash, Copy)]
pub enum DebugState {
    Running,
    Paused,
}

pub struct PhysicsDebugPlugin;
impl Plugin for PhysicsDebugPlugin {
    fn build(&self, app: &mut App) {
        app.add_state(DebugState::Running)
            .add_system_set(
                SystemSet::on_update(DebugState::Running)
                    .after(PhysicsSystems::Resolved)
                    .with_system(spawn_debug)
                    .with_system(update_debug.after(spawn_debug)),
            )
            .add_system_set(SystemSet::on_enter(DebugState::Paused).with_system(remove_debug));
    }
}

#[derive(Component)]
pub struct PhysicsDebug(pub Entity);

#[derive(Component)]
pub struct Debug;

pub fn spawn_debug(
    mut commands: Commands,
    query: Query<(Entity, &AabbWorld, &Transform), Without<PhysicsDebug>>,
    mut meshes: ResMut<Assets<Mesh>>,
) {
    for (e, aabb, trans) in query.iter() {
        // Spawn bounding box, creating new entity seen we dont want rotation
        let id = commands
            .spawn_bundle(PbrBundle {
                //transform: Transform::from_translation(trans.translation),
                mesh: meshes.add(Mesh::from(aabb)),
                visibility: Visibility {
                    is_visible: true,
                },
                ..Default::default()
            })
            .insert(Debug)
            .insert(Name::new("AabbDebug"))
            .id();

        commands.entity(e).insert(PhysicsDebug(id));
    }
}

pub fn update_debug(
    query: Query<( &PhysicsDebug, &AabbWorld)>,
    mut debug_query: Query<&mut Handle<Mesh>, With<Debug>>,
    mut meshes: ResMut<Assets<Mesh>>,
) {
    for (target, aabb_world) in query.iter() {
        let mut mesh_handle = debug_query.get_mut(target.0).unwrap();
        *mesh_handle = meshes.add(Mesh::from(aabb_world));
        //trans_target.translation = trans_parent.translation;
    }
}

pub fn remove_debug(mut commands: Commands, query: Query<(Entity, &PhysicsDebug)>) {
    for (e, pd) in query.iter() {
        commands.entity(pd.0).despawn_recursive();
        commands.entity(e).remove::<PhysicsDebug>();
    }
}

impl From<&AabbWorld> for Mesh {
    fn from(aabb: &AabbWorld) -> Self {
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

        let mut normals = Vec::with_capacity(8);
        let mut uvs = Vec::with_capacity(8);

        for _ in 0..8 {
            normals.push([0.0, 0.0, 1.0]);
            uvs.push([0.0, 0.0]);
        }

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
