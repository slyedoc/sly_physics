use bevy::prelude::*;

pub struct ButtonColors {
    pub normal: UiColor,
    pub hovered: UiColor,
}

impl Default for ButtonColors {
    fn default() -> Self {
        ButtonColors {
            normal: Color::rgb(0.15, 0.15, 0.15).into(),
            hovered: Color::rgb(0.25, 0.25, 0.25).into(),
        }
    }
}

pub struct FontAssets {
    pub ui_font: Handle<Font>,
}

impl FromWorld for FontAssets {
    fn from_world(world: &mut World) -> Self {
        let asset_server = world.get_resource::<AssetServer>().unwrap();
        let ui_font = asset_server.load("fonts/FiraSans-Bold.ttf");
        Self { ui_font }
    }
}

impl FontAssets {
    #[allow(dead_code)]
    pub fn h1(&self, text: String, color: Color) -> TextSection {
        TextSection {
            value: text,
            style: TextStyle {
                font: self.ui_font.clone(),
                font_size: 30.0,
                color,
            },
        }
    }

    #[allow(dead_code)]
    pub fn title(&self, text: String, color: Color) -> TextSection {
        TextSection {
            value: text,
            style: TextStyle {
                font: self.ui_font.clone(),
                font_size: 90.0,
                color,
            },
        }
    }

    #[allow(dead_code)]
    pub fn sub_title(&self, text: String, color: Color) -> TextSection {
        TextSection {
            value: text,
            style: TextStyle {
                font: self.ui_font.clone(),
                font_size: 16.0,
                color,
            },
        }
    }
}
