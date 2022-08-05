use bevy::{
    diagnostic::{Diagnostics, FrameTimeDiagnosticsPlugin},
    prelude::*,
};
use iyes_loopless::prelude::*;
use sly_physics::prelude::*;

use super::Keep;

pub struct OverlayPlugin;

impl Plugin for OverlayPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugin(FrameTimeDiagnosticsPlugin::default())
            .add_startup_system(setup_overlay)
            .add_system(update_fps)
            .add_system(update_state)
            .add_system(update_debug);
        //.add_system(update_bvh_tri_count)
        //.add_system(update_render_time)
        //.add_system(update_ray_count);
    }
}

#[derive(Component)]
struct FpsText;

#[derive(Component)]
struct PhysicsStateText;

#[derive(Component)]
struct DebugStateText;

pub const UI_SIZE: f32 = 30.0;

fn setup_overlay(mut commands: Commands, asset_server: ResMut<AssetServer>) {
    let ui_font = asset_server.load("fonts/FiraSans-Bold.ttf");

    let mut offset = 10.0;
    let offset_change = 25.0;

    commands
        .spawn_bundle(TextBundle {
            style: Style {
                position_type: PositionType::Absolute,
                position: UiRect::<Val> {
                    left: Val::Px(10.0),
                    bottom: Val::Px(offset),
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
                        value: "FPS: ".to_string(),
                        style: TextStyle {
                            font: ui_font.clone(),
                            font_size: UI_SIZE,
                            color: Color::WHITE,
                        },
                    },
                    TextSection {
                        value: "".to_string(),
                        style: TextStyle {
                            font: ui_font.clone(),
                            font_size: UI_SIZE,
                            color: Color::GOLD,
                        },
                    },
                ],
                ..Default::default()
            },
            ..Default::default()
        })
        .insert(Name::new("ui FPS"))
        .insert(Keep)
        .insert(FpsText);

        offset += offset_change;


        commands
        .spawn_bundle(TextBundle {
            style: Style {
                position_type: PositionType::Absolute,
                position: UiRect::<Val> {
                    left: Val::Px(10.0),
                    bottom: Val::Px(offset),
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
                        value: "State: ".to_string(),
                        style: TextStyle {
                            font: ui_font.clone(),
                            font_size: UI_SIZE,
                            color: Color::WHITE,
                        },
                    },
                    TextSection {
                        value: "".to_string(),
                        style: TextStyle {
                            font: ui_font.clone(),
                            font_size: UI_SIZE,
                            color: Color::GOLD,
                        },
                    },
                ],
                ..Default::default()
            },
            ..Default::default()
        })
        .insert(Name::new("ui State"))
        .insert(Keep)
        .insert(PhysicsStateText);

        offset += offset_change;


        commands
        .spawn_bundle(TextBundle {
            style: Style {
                position_type: PositionType::Absolute,
                position: UiRect::<Val> {
                    left: Val::Px(10.0),
                    bottom: Val::Px(offset),
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
                        value: "Debug: ".to_string(),
                        style: TextStyle {
                            font: ui_font.clone(),
                            font_size: UI_SIZE,
                            color: Color::WHITE,
                        },
                    },
                    TextSection {
                        value: "".to_string(),
                        style: TextStyle {
                            font: ui_font.clone(),
                            font_size: UI_SIZE,
                            color: Color::GOLD,
                        },
                    },
                ],
                ..Default::default()
            },
            ..Default::default()
        })
        .insert(Name::new("ui Debug State"))
        .insert(Keep)
        .insert(DebugStateText);
}

fn update_fps(diagnostics: Res<Diagnostics>, mut query: Query<&mut Text, With<FpsText>>) {
    for mut text in query.iter_mut() {
        if let Some(fps) = diagnostics.get(FrameTimeDiagnosticsPlugin::FPS) {
            if let Some(average) = fps.average() {
                // Update the value of the second section
                text.sections[1].value = format!("{:.0}", average);
                text.sections[1].style.color = match average {
                    x if x >= 50.0 => Color::GREEN,
                    x if x > 40.0 && x < 50.0 => Color::YELLOW,
                    x if x <= 40.0 => Color::RED,
                    _ => Color::WHITE,
                };
            }
        }
    }
}


fn update_state(state: Res<CurrentState<PhysicsState>>, mut query: Query<&mut Text, With<PhysicsStateText>>) {
    for mut text in query.iter_mut() {
        text.sections[1].value = format!("{:?}", state.0);
        text.sections[1].style.color = match state.0 {
            PhysicsState::Running => Color::GREEN,
            PhysicsState::Paused => Color::RED,                        
        };
    }
}


fn update_debug(state: Res<CurrentState<PhysicsDebugState>>, mut query: Query<&mut Text, With<DebugStateText>>) {
    for mut text in query.iter_mut() {
        text.sections[1].value =  match state.0 {
            PhysicsDebugState::Running => "Enabled".to_string(),
            PhysicsDebugState::Paused => "Disabled".to_string(),
        };
        text.sections[1].style.color = match state.0 {
            PhysicsDebugState::Running => Color::GREEN,
            PhysicsDebugState::Paused => Color::RED,                        
        };
    }
}