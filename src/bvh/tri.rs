use bevy::prelude::*;
use bevy_inspector_egui::Inspectable;

// TODO: Will be replaced by bevy mesh data
//, stop gap to get things working
#[derive(Default, Inspectable, Debug, Copy, Clone)]
pub struct BvhTri {
    pub vertex0: Vec3,
    pub vertex1: Vec3,
    pub vertex2: Vec3,
    pub centroid: Vec3,
}

impl BvhTri {
    pub fn new(v0: Vec3, v1: Vec3, v2: Vec3) -> Self {
        BvhTri {
            vertex0: v0,
            vertex1: v1,
            vertex2: v2,
            centroid: (v0 + v1 + v2) / 3.0,
        }
    }
}
