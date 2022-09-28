mod bvh_aabb;
mod bvh_camera;
mod constraints;
mod contacts;
mod entity_aabb;

pub use bvh_aabb::*;
pub use bvh_camera::*;
pub use constraints::*;
pub use contacts::*;
pub use entity_aabb::*;

use bevy::{math::vec3, prelude::*};
use iyes_loopless::prelude::*;

#[derive(Component)]
pub struct PhysicsDebug;

#[derive(Debug, Clone, PartialEq, Eq, Hash, Copy)]
pub enum PhysicsDebugState {
    Running,
    Paused,
}

// these values are not used, but need to set the vertex buffer to something
const AABB_VERTEX_POSITIONS: [Vec3; 8] = [
    vec3(0.5, 0.5, -0.5),
    vec3(-0.5, 0.5, -0.5),
    vec3(-0.5, 0.5, 0.5),
    vec3(0.5, 0.5, 0.5),
    vec3(0.5, -0.5, -0.5),
    vec3(-0.5, -0.5, -0.5),
    vec3(-0.5, -0.5, 0.5),
    vec3(0.5, -0.5, 0.5),
];

const AABB_INDICES: [u32; 24] = [
    0, 1, 1, 2, 2, 3, 3, 0, // Top ring
    4, 5, 5, 6, 6, 7, 7, 4, // Bottom ring
    0, 4, 1, 5, 2, 6, 3, 7, // Verticals
];

const AABB_INDICES_LEN: u32 = AABB_INDICES.len() as u32;

pub struct PhysicsDebugPlugin;
impl Plugin for PhysicsDebugPlugin {
    fn build(&self, app: &mut App) {
        app.add_loopless_state(PhysicsDebugState::Paused)
            // Rendering debug systems
            // TODO: These are some of the first pipelines I have made, and all 3 use different techniques
            // as I was learning, should most likely switch them to draw_indexed instanced
            // but these work and all are far faster than what i was doing, creating meshes for each every frame
            .add_plugin(DebugEntityAabbPlugin) // draw call per entity with dynamic uniform
            .add_plugin(DebugBvhAabbPlugin) // expanded index buffer, one draw call
            .add_plugin(DebugContactsPlugin) // draw_indexed instanced, one draw call
            .add_plugin(DebugConstraintsPlugin); // draw_indexed instanced,

        // raycast test plugin
        //.add_plugin(PhysicsBvhCameraPlugin);
    }
}
