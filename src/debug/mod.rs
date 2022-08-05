mod bvh_camera;
pub use bvh_camera::*;

use crate::{bvh::Tlas, prelude::ManifoldArena, AabbWorld, PhysicsFixedUpdate, PhysicsState};
use bevy::prelude::*;
use iyes_loopless::prelude::*;

#[derive(Component)]
pub struct PhysicsDebug;

#[derive(Debug, Clone, PartialEq, Eq, Hash, Copy)]
pub enum PhysicsDebugState {
    Running,
    Paused,
}

#[derive(Component)]
pub struct ManifoldDebug;

#[derive(Component)]
pub struct AabbDebug(pub Entity);

#[derive(Component)]
pub struct AabbWorldDebug(pub Entity);

#[derive(Component)]
pub struct DebugContactMaterial(Handle<StandardMaterial>);

pub struct PhysicsDebugPlugin;
impl Plugin for PhysicsDebugPlugin {
    fn build(&self, app: &mut App) {
        app.add_loopless_state(PhysicsDebugState::Paused)
            .add_startup_system(setup)
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
                    .with_system(spawn_bvh_debug)
                    .with_system(spawn_contacts)
                    .into(),
            )
            .add_enter_system(PhysicsDebugState::Paused, remove_debug);
    }
}

fn setup(mut commands: Commands, mut materials: ResMut<Assets<StandardMaterial>>) {
    commands.insert_resource::<DebugContactMaterial>(DebugContactMaterial(
        materials.add(StandardMaterial {
            base_color: Color::rgb(0.5, 0.5, 0.5),
            ..default()
        }),
    ));
}

fn spawn_debug(
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
        // commands.entity(e).remove::<AabbDebug>();
        commands.entity(e).remove::<AabbWorldDebug>();
    }

    // remove debug entities
    for e in remove_query.iter() {
        commands.entity(e).despawn_recursive();
    }
}

#[derive(Component)]
struct BvhAabbDebug;

fn spawn_bvh_debug(
    mut commands: Commands,
    query: Query<Entity, With<BvhAabbDebug>>,
    mut meshes: ResMut<Assets<Mesh>>,
    tlas: Res<Tlas>,
) {
    //remove old
    for e in query.iter() {
        commands.entity(e).despawn_recursive();
    }

    //add current
    for node in &tlas.tlas_nodes {
        if !node.is_leaf() {
            commands
                .spawn_bundle(PbrBundle {
                    //transform: Transform::from_translation(trans.translation),
                    mesh: meshes.add(Mesh::from(&node.aabb)),
                    visibility: Visibility { is_visible: true },
                    ..Default::default()
                })
                .insert(BvhAabbDebug)
                .insert(PhysicsDebug)
                .insert(Name::new("Bvh Aabb Debug"));
        }
    }
}

#[derive(Component)]
struct ContactDebug;

fn spawn_contacts(
    mut commands: Commands,
    query: Query<Entity, With<ContactDebug>>,
    mut meshes: ResMut<Assets<Mesh>>,
    debug_contact_material: Res<DebugContactMaterial>,
    mut contact_manifold: Res<ManifoldArena>,
) {
    //remove old
    for e in query.iter() {
        commands.entity(e).despawn_recursive();
    }

    //add contact
    for ((_e1, _e2), manifold) in &contact_manifold.manifolds {    
            commands
                .spawn_bundle(PbrBundle {
                    transform: Transform::from_translation(manifold.contacts[0].world_point_a),
                    mesh: meshes.add(Mesh::from(shape::UVSphere {
                        radius: 0.1,
                        ..default()
                    })),
                    material: debug_contact_material.0.clone(),
                    visibility: Visibility { is_visible: true },
                    ..Default::default()
                })
                .insert(ContactDebug)
                .insert(PhysicsDebug)
                .insert(Name::new("Contact Debug"));
        
    }
}
