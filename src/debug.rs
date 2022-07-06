use crate::{Aabb, AabbWorld, PhysicsSystems, PHYSISCS_TIMESTEP, PhysicsState};
use bevy::{
    prelude::*,
    render::mesh::{Indices, PrimitiveTopology}, core::FixedTimestep, ecs::schedule::ShouldRun,
};
use bevy_inspector_egui::Inspectable;

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
pub enum DebugState {
    Running,
    Paused,
}

pub struct PhysicsDebugPlugin;
impl Plugin for PhysicsDebugPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<DebugConfig>()
            .add_state(DebugState::Running)
            .add_system_set_to_stage(
                CoreStage::PostUpdate,
                                // workaround since you cant chain with_run_criteria, see https://github.com/bevyengine/bevy/issues/1839
                                SystemSet::new()
                                .with_run_criteria(FixedTimestep::step(PHYSISCS_TIMESTEP as f64).chain(
                                    |In(input): In<ShouldRun>, state: Res<State<PhysicsState>>| {
                                        if state.current() == &PhysicsState::Running {
                                            input
                                        } else {
                                            ShouldRun::No
                                        }
                                    },
                                ))
                
                    .after(PhysicsSystems::Resolved)
                    .with_system(spawn_debug)
                    .with_system(update_debug.after(spawn_debug)),
            )
            .add_system_set(SystemSet::on_enter(DebugState::Paused).with_system(remove_debug));
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
    query: Query<(Entity, &Aabb, &AabbWorld, &Transform), Without<AabbDebug>>,
    mut meshes: ResMut<Assets<Mesh>>,
) {
    for (e, aabb, aabb_world, _trans) in query.iter() {
        // Spawn aabb, creating new entity seen we dont want rotation
        {
            let id = commands
                .spawn_bundle(PbrBundle {
                    //transform: Transform::from_translation(trans.translation),
                    mesh: meshes.add(Mesh::from(aabb)),
                    visibility: Visibility { is_visible: true },
                    ..Default::default()
                })
                .insert(PhysicsDebug)
                .insert(Name::new("Aabb Debug"))
                .id();

            commands.entity(e).insert(AabbDebug(id));
        }

        {
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
}

pub fn update_debug(
    query: Query<(&AabbDebug, &Aabb, &AabbWorldDebug, &AabbWorld)>,
    mut mesh_query: Query<&mut Handle<Mesh>>,
    mut meshes: ResMut<Assets<Mesh>>,
) {
    for (target_aabb, aabb, target_aabb_world, aabb_world) in query.iter() {
        let mut aabb_handle = mesh_query.get_mut(target_aabb.0).unwrap();
        *aabb_handle = meshes.add(Mesh::from(aabb));

        let mut aabb_world_handle = mesh_query.get_mut(target_aabb_world.0).unwrap();
        *aabb_world_handle = meshes.add(Mesh::from(&aabb_world.0));
        //trans_target.translation = trans_parent.translation;
    }
}

pub fn remove_debug(
    mut commands: Commands, 
    query: Query<Entity, (With<AabbDebug>, With<AabbWorldDebug>)>,
    remove_query: Query<Entity, With<PhysicsDebug>>,
) {
    // Remove debug components
    for e in query.iter() {
        commands.entity(e).remove::<AabbDebug>();
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
