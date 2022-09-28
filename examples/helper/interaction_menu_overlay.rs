use bevy::{math::vec3, prelude::*};
use iyes_loopless::{
    prelude::AppLooplessStateExt,
    state::{CurrentState, NextState},
};
use sly_physics::prelude::*;

use super::{assets::*, Keep};

pub const CLEAR: Color = Color::rgba(0.0, 0.0, 0.0, 0.0);

pub struct InteractionMenuOverlayPlugin;

#[derive(Clone, Eq, PartialEq, Debug, Hash, Component)]
pub enum Equipment {
    Stick,
    Push,
}

impl Plugin for InteractionMenuOverlayPlugin {
    fn build(&self, app: &mut App) {
        app.add_loopless_state(Equipment::Stick)
            .add_startup_system(setup_menu)
            .add_system(button_click)
            .add_system(change_equipment);
    }
}

pub fn change_equipment(
    mut commands: Commands,
    state: ResMut<CurrentState<Equipment>>,
    camera_query: Query<(Entity, Option<&Children>), With<Camera>>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut colliders: ResMut<Assets<Collider>>,
) {
    if state.is_changed() {
        let (camera_entity, camera_children) = camera_query.single();
        // remove existing children
        if camera_children.is_some() {
            for e in camera_children.unwrap() {
                commands.entity(*e).despawn_recursive();
            }
        }

        commands
            .entity(camera_entity)
            .with_children(|parent| match state.0 {
                Equipment::Stick => {
                    let stick_size = vec3(0.2, 0.2, 1.0);
                    let stick_offset = vec3(0.0, 0.0, -1.0);
                    parent
                        .spawn_bundle(PbrBundle {
                            transform: Transform::from_translation(stick_offset),
                            mesh: meshes.add(Mesh::from(shape::Box::new(
                                stick_size.x,
                                stick_size.y,
                                stick_size.z,
                            ))),
                            material: materials.add(StandardMaterial {
                                base_color: Color::rgb(0.0, 0.0, 1.0),
                                ..default()
                            }),

                            ..default()
                        })
                        .insert_bundle(RigidBodyBundle {
                            mode: RigidBody::Static,
                            collider: colliders.add(Collider::from(Box::new(stick_size))),
                            ..default()
                        })
                        .insert(Name::new("Stick"));
                }
                Equipment::Push => todo!(),
            });
    }
}

fn setup_menu(
    mut commands: Commands,
    button_colors: Res<ButtonColors>,
    fonts: Res<FontAssets>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut colliders: ResMut<Assets<Collider>>,
) {
    let stick_size = vec3(0.2, 0.2, 1.0);
    let stick_offset = vec3(0.0, 0.0, -1.0);
    commands
        .spawn_bundle(PbrBundle {
            transform: Transform::from_translation(stick_offset),
            mesh: meshes.add(Mesh::from(shape::Box::new(
                stick_size.x,
                stick_size.y,
                stick_size.z,
            ))),
            material: materials.add(StandardMaterial {
                base_color: Color::rgb(0.0, 0.0, 1.0),
                ..default()
            }),

            ..default()
        })
        .insert_bundle(RigidBodyBundle {
            mode: RigidBody::Static,
            collider: colliders.add(Collider::from(Box::new(stick_size))),
            ..default()
        });

    commands
        .spawn_bundle(NodeBundle {
            style: Style {
                position_type: PositionType::Absolute,
                position: UiRect {
                    top: Val::Percent(40.0),
                    left: Val::Percent(30.0),
                    ..Default::default()
                },
                size: Size::new(Val::Percent(40.0), Val::Percent(40.0)),
                flex_direction: FlexDirection::ColumnReverse,
                align_content: AlignContent::Stretch,
                justify_content: JustifyContent::Center,
                ..Default::default()
            },
            color: CLEAR.into(),
            ..Default::default()
        })
        .insert(Keep)
        .with_children(|parent| {
            for b in Equipment::iter() {
                parent
                    .spawn_bundle(ButtonBundle {
                        style: Style {
                            margin: UiRect::all(Val::Px(10.0)),
                            align_items: AlignItems::Center,
                            align_content: AlignContent::Center,
                            justify_content: JustifyContent::Center,
                            ..Default::default()
                        },
                        color: button_colors.normal,
                        ..Default::default()
                    })
                    .with_children(|parent| {
                        parent.spawn_bundle(TextBundle {
                            text: Text {
                                sections: vec![
                                    fonts.h1(b.clone().into(), Color::rgb(0.9, 0.9, 0.9))
                                ],
                                alignment: Default::default(),
                            },
                            ..Default::default()
                        });
                    })
                    .insert(b);
            }
        });
}

impl From<Equipment> for String {
    fn from(b: Equipment) -> Self {
        match b {
            Equipment::Stick => "Stick".to_string(),
            Equipment::Push => "Push".to_string(),
        }
    }
}

impl Equipment {
    pub fn iter() -> impl Iterator<Item = Self> {
        [Equipment::Stick, Equipment::Push].into_iter()
    }
}

pub fn button_click(
    mut commands: Commands,
    interaction_query: Query<(&Interaction, &Equipment), (Changed<Interaction>, With<Button>)>,
) {
    for (interaction, btn) in interaction_query.iter() {
        if *interaction == Interaction::Clicked {
            commands.insert_resource(NextState(btn.clone()));
        }
    }
}
