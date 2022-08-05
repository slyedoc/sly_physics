use bevy::{math::vec3, prelude::*, window::PresentMode};
use bevy_inspector_egui::{Inspectable, InspectorPlugin, WorldInspectorPlugin};
use helper::{AppState, HelperPlugin};
use iyes_loopless::prelude::*;
use sly_camera_controller::*;
use sly_physics::prelude::*;
mod helper;



fn main() {
    App::new()
        .insert_resource(WindowDescriptor {
            present_mode: PresentMode::Fifo,
            ..default()
        })
        .add_plugins(DefaultPlugins)
        .add_plugin(HelperPlugin)
        .add_plugin(WorldInspectorPlugin::default())
        .add_plugin(CameraControllerPlugin)
        // our phsycis plugin
        .add_plugin(PhysicsPlugin)
        .add_plugin(PhysicsDebugPlugin)
        .add_plugin(PhysicsBvhCameraPlugin)

        .add_enter_system(AppState::Playing, helper::setup_room)
        .add_startup_system(helper::setup_camera)
        .add_startup_system(setup_text)        
        .add_enter_system(AppState::Playing, setup)
        .run();
}

fn setup_text(
    mut commands: Commands, 
    asset_server: ResMut<AssetServer>,
) {
  
        let ui_font = asset_server.load("fonts/FiraSans-Bold.ttf");
    
        commands
            .spawn_bundle(TextBundle {
                style: Style {
                    position_type: PositionType::Absolute,
                    position: UiRect::<Val> {
                        left: Val::Px(10.0),
                        top: Val::Px(10.0),
                        ..Default::default()
                    },
                    align_self: AlignSelf::FlexEnd,
                    ..Default::default()
                },
                // Use `Text` directly
                text: Text {
                    // Construct a `Vec` of `TextSection`s
                    sections: vec![
                        TextSection {
                            value: "Gravity Drop Test from 5.0 meters".to_string(),
                            style: TextStyle {
                                font: ui_font.clone(),
                                font_size: 30.0,
                                color: Color::WHITE,
                            },
                        },
                        TextSection {
                            value: "".to_string(),
                            style: TextStyle {
                                font: ui_font.clone(),
                                font_size: 30.0,
                                color: Color::GOLD,
                            },
                        },
                    ],
                    ..Default::default()
                },
                ..Default::default()
            })
            .insert(Name::new("ui FPS"))
            .insert(helper::Keep);
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    asset_server: Res<AssetServer>,
    mut collider_resources: ResMut<ColliderResources>,
) {
    let radius = 1.0;
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
            transform: Transform::from_translation(Vec3::Y * 5.0),
            ..default()
        })
        .insert_bundle(RigidBodyBundle {
            collider: collider_resources.add_sphere(radius),
            mode: RigidBodyMode::Dynamic,
            elasticity: Elasticity(1.0),
            ..default()
        })
        .insert(Name::new("Sphere A"));
}
