use bevy::{app::PluginGroupBuilder, math::vec3, prelude::*, window::PresentMode};
use criterion::{criterion_group, criterion_main, BenchmarkId, Criterion, Throughput};

use sly_physics::prelude::*;
use std::{f32::consts::FRAC_PI_2, time::Duration};

fn stack_benchmark(c: &mut Criterion) {
    for item in [StackItem::Cube, StackItem::Sphere] {
        let mut group = c.benchmark_group(format!("stack_{}", item));

        for (size, time) in [(100, 15), (500, 40)].iter() {
            //(1000, 120), (2000, 300)
            group.measurement_time(Duration::from_secs(*time));
            group.throughput(Throughput::Elements(*size as u64));
            group.sample_size(10);
            group.bench_with_input(BenchmarkId::from_parameter(size), size, |bencher, &size| {
                bencher.iter(|| {
                    let mut app = App::new();
                    app.insert_resource(WindowDescriptor {
                        present_mode: PresentMode::Immediate,
                        ..default()
                    })
                    .add_plugins(BenchPlugins)
                    // our plugins
                    .add_plugin(PhysicsPlugin)
                    .add_plugin(GravityPlugin)
                    .insert_resource(StackConfig {
                        size,
                        item,
                    })
                    // setup environment
                    .add_startup_system(spawn_camera)
                    .add_startup_system(spawn_room)
                    .add_startup_system(spawn_stack);

                    // run for what would be 5 seconds at 60 fps
                    for _ in 0..(60 * 5) {
                        app.update();
                    }
                });
            });
        }

        group.finish();
    }
}

fn spawn_camera(mut commands: Commands) {
    commands.spawn_bundle(Camera3dBundle {
        transform: Transform::from_xyz(0.0, 10.0, -30.0).looking_at(Vec3::ZERO, Vec3::Y),
        ..default()
    });

    // light
    commands.spawn_bundle(DirectionalLightBundle {
        directional_light: DirectionalLight {
            shadows_enabled: true,
            ..default()
        },
        transform: Transform::from_xyz(50.0, 50.0, 50.0).looking_at(Vec3::ZERO, Vec3::Y),
        ..default()
    });
}

#[derive(Copy, Clone)]
enum StackItem {
    Sphere,
    Cube,
}

impl std::fmt::Display for StackItem {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            StackItem::Sphere => write!(f, "sphere"),
            StackItem::Cube => write!(f, "cube"),
        }
    }
}


struct StackConfig {
    size: usize,
    item: StackItem,
}

fn spawn_stack(
    mut commands: Commands,
    mut colliders: ResMut<Assets<Collider>>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    config: Res<StackConfig>,
) {
    // Stack
    let radius = 0.5;
    let spacing = 1.0;

    let collider = match config.item {
        StackItem::Sphere => colliders.add(Collider::from(Sphere::new(radius))),
        StackItem::Cube => colliders.add(Collider::from(Box::new(Vec3::splat(radius * 1.9)))),
    };

    let mat = materials.add(StandardMaterial {
        base_color: Color::rgb(1.0, 0.0, 0.0).into(),
        ..default()
    });

    let mesh = match config.item {
        StackItem::Sphere => meshes.add(Mesh::from(shape::UVSphere {
            radius,
            ..default()
        })),
        StackItem::Cube => meshes.add(Mesh::from(shape::Cube { size: radius * 2.0 })),
    };

    let count = 0;
    let size = (config.size as f32).powf(1.0 / 3.0).ceil() as usize;

    for i in 0..size {
        for j in 0..size {
            for k in 0..size {
                let pos = vec3(
                    i as f32 + radius - size as f32 / 2.0,
                    j as f32 + radius,
                    k as f32 + radius - (radius * 2.0 * size as f32 / 2.0),
                ) * spacing;
                commands
                    .spawn_bundle(PbrBundle {
                        mesh: mesh.clone(),
                        material: mat.clone(),
                        transform: Transform::from_translation(pos),
                        ..default()
                    })
                    .insert_bundle(RigidBodyBundle {
                        collider: collider.clone(),
                        ..default()
                    });
                if count == config.size {
                    return;
                }
            }
        }
    }
}

pub fn spawn_room(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut colliders: ResMut<Assets<Collider>>,
) {
    let floor_size = 100.0;
    let wall_height = 10.0;
    let floor_half = floor_size * 0.5;
    let wall_height_half = wall_height * 0.5;
    // floor
    commands
        .spawn_bundle(PbrBundle {
            transform: Transform::from_xyz(0.0, -0.5, 0.0),
            mesh: meshes.add(Mesh::from(shape::Box::new(floor_size, 1.0, floor_size))),
            material: materials.add(StandardMaterial {
                base_color: Color::DARK_GREEN,
                ..default()
            }),
            ..default()
        })
        .insert_bundle(RigidBodyBundle {
            collider: colliders.add(Collider::from(Box::new(vec3(floor_size, 1.0, floor_size)))),
            mode: RigidBody::Static,
            ..default()
        })
        .insert(Name::new("Floor"));

    // walls
    let wall_mat = materials.add(StandardMaterial {
        base_color: Color::ALICE_BLUE,
        ..default()
    });

    for wall in 0..4 {
        let mut transform = Transform::from_xyz(0.0, wall_height_half, floor_half);
        transform.rotate_around(
            Vec3::ZERO,
            Quat::from_axis_angle(Vec3::Y, wall as f32 * FRAC_PI_2),
        );

        commands
            .spawn_bundle(PbrBundle {
                transform,
                mesh: meshes.add(Mesh::from(shape::Box::new(floor_size, wall_height, 1.0))),
                material: wall_mat.clone(),
                ..default()
            })
            .insert_bundle(RigidBodyBundle {
                collider: colliders.add(Collider::from(Box::new(vec3(floor_size, wall_height, 1.0)))),
                mode: RigidBody::Static,
                ..default()
            })
            .insert(Name::new("Wall"));
    }
}

struct BenchPlugins;

impl PluginGroup for BenchPlugins {
    fn build(&mut self, group: &mut PluginGroupBuilder) {
        //group.add(bevy::log::LogPlugin::default());
        group.add(bevy::core::CorePlugin::default());
        group.add(bevy::time::TimePlugin::default());
        group.add(bevy::transform::TransformPlugin::default());
        group.add(bevy::hierarchy::HierarchyPlugin::default());
        group.add(bevy::diagnostic::DiagnosticsPlugin::default());
        group.add(bevy::input::InputPlugin::default());
        group.add(bevy::window::WindowPlugin::default());
        group.add(bevy::asset::AssetPlugin::default());
        //group.add(bevy::asset::debug_asset_server::DebugAssetServerPlugin::default());
        group.add(bevy::scene::ScenePlugin::default());
        group.add(bevy::winit::WinitPlugin::default());
        group.add(bevy::render::RenderPlugin::default());
        group.add(bevy::core_pipeline::CorePipelinePlugin::default());
        group.add(bevy::sprite::SpritePlugin::default());
        group.add(bevy::text::TextPlugin::default());
        group.add(bevy::ui::UiPlugin::default());
        group.add(bevy::pbr::PbrPlugin::default());
        group.add(bevy::gltf::GltfPlugin::default());
        //group.add(bevy_audio::AudioPlugin::default());
        group.add(bevy::gilrs::GilrsPlugin::default());
        //group.add(bevy_animation::AnimationPlugin::default());
    }
}

criterion_group!(benches, stack_benchmark);
criterion_main!(benches);
