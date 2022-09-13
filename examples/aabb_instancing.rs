//! A shader that renders a mesh multiple times in one draw call.
mod helper;
use bevy::{
    core_pipeline::core_3d::Transparent3d,
    ecs::system::{lifetimeless::*, SystemParamItem},
    math::prelude::*,
    pbr::{MeshPipeline, MeshPipelineKey, MeshUniform, SetMeshBindGroup, SetMeshViewBindGroup},
    prelude::*,
    reflect::TypeUuid,
    render::{
        extract_component::{ExtractComponent, ExtractComponentPlugin},
        mesh::{GpuBufferInfo, MeshVertexBufferLayout},
        render_asset::RenderAssets,
        render_phase::{
            AddRenderCommand, DrawFunctions, EntityRenderCommand, RenderCommandResult, RenderPhase,
            SetItemPipeline, TrackedRenderPass, DrawFunctionId, PhaseItem, EntityPhaseItem,
        },
        render_resource::*,
        renderer::{RenderDevice, RenderQueue},
        view::{ComputedVisibility, ExtractedView, Msaa, NoFrustumCulling, Visibility},
        Extract, RenderApp, RenderStage,
    },
};
use bevy_inspector_egui::WorldInspectorPlugin;
use bytemuck::{cast_slice, Pod, Zeroable};
use sly_camera_controller::CameraControllerPlugin;
use sly_physics::prelude::*;

fn main() {
    let mut app = App::new();
    app.add_plugins(DefaultPlugins)
        .add_plugin(WorldInspectorPlugin::default())
        // out plugins
        .add_plugin(PhysicsPlugin)
        .add_plugin(GravityPlugin)
        .add_plugin(PhysicsDebugPlugin)
        .add_plugin(PhysicsBvhCameraPlugin)
        // Out testing plugin
        // example stuff
        .add_plugin(helper::HelperPlugin)
        .add_plugin(CameraControllerPlugin)
        .add_startup_system(helper::setup_camera)
        .add_startup_system(helper::setup_room);

    app.run();
}


