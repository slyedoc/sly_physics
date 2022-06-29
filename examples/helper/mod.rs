use bevy::{prelude::*,};
use sly_physics::*;

pub fn setup_sandbox(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    asset_server: Res<AssetServer>,
) {
    // ground
    commands
        .spawn_bundle(PbrBundle {
            transform: Transform::from_xyz(0.0, -100.0, 0.0),
            mesh: meshes.add(Mesh::from(shape::UVSphere { radius: 100.0, ..default() })),
            material: materials.add(StandardMaterial {
                base_color: Color::DARK_GREEN,
                ..default()
            }),
            ..default()
        })
        .insert(Collider::sphere(100.0))
        .insert(Static)
        .insert(Name::new("Ground"));



        let radius = 0.5;
        commands
            .spawn_bundle(PbrBundle {
                mesh: meshes.add(Mesh::from(shape::UVSphere {
                    radius,
                    ..default()
                })),
                material: materials.add(StandardMaterial {
                    base_color_texture: Some(asset_server.load("checker_red.png")),
                    ..default()
                }),
                transform: Transform::from_xyz(
                    0.0,
                    radius,
                    0.0,
                ),
                ..default()
            })
            .insert(Collider::sphere(radius))
            //.insert(Static)
            //.insert(ActiveEvents::COLLISION_EVENTS)
            //.insert(LockedAxes::ROTATION_LOCKED)
            //.insert(ColliderMassProperties::Density(2.0))
            .insert(Name::new("Sphere"));
}
