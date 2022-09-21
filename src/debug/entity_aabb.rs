use bevy::{
    core_pipeline::core_3d::Transparent3d,
    ecs::system::{
        lifetimeless::{Read, SQuery, SRes},
        SystemParamItem,
    },
    prelude::*,
    render::{
        render_phase::*,
        render_resource::*,
        renderer::{RenderDevice, RenderQueue},
        texture::BevyDefault,
        view::{ViewUniform, ViewUniformOffset, ViewUniforms},
        Extract, RenderApp, RenderStage,
    }, reflect::TypeUuid, asset::load_internal_asset,
};
use bytemuck::cast_slice;
use iyes_loopless::state::CurrentState;

use crate::{ types::Aabb};
use super::{PhysicsDebugState, AABB_VERTEX_POSITIONS, AABB_INDICES, AABB_INDICES_LEN};

const ENTITY_AABBS_SHADER: HandleUntyped =
    HandleUntyped::weak_from_u64(Shader::TYPE_UUID, 13235301471009851039);

pub struct DebugEntityAabbPlugin;

impl Plugin for DebugEntityAabbPlugin {
    fn build(&self, app: &mut App) {


        load_internal_asset!(app, ENTITY_AABBS_SHADER, "entity_aabb.wgsl", Shader::from_wgsl);

        if let Ok(render_app) = app.get_sub_app_mut(RenderApp) {
            render_app
                .init_resource::<AabbsPipeline>()
                .init_resource::<AabbsMeta>()
                .init_resource::<AabbUniforms>()
                .add_render_command::<Transparent3d, DrawAabb>()
                .add_system_to_stage(RenderStage::Extract, extract_entity_aabbs)
                .add_system_to_stage(RenderStage::Prepare, prepare_aabbs)
                .add_system_to_stage(RenderStage::Queue, queue_aabbs);
        }
    }
}



fn extract_entity_aabbs(
    mut commands: Commands,
    current_state: Extract<Res<CurrentState<PhysicsDebugState>>>,
    query: Extract<Query<(Entity, &Aabb)>>,
) {
    if current_state.0 == PhysicsDebugState::Running {
        for (entity, aabb) in query.iter() {
            commands.get_or_spawn(entity).insert(aabb.clone());
        }
    }
}

fn prepare_aabbs(
    mut commands: Commands,
    query: Query<(Entity, &Aabb)>,
    render_device: Res<RenderDevice>,
    render_queue: Res<RenderQueue>,
    mut aabb_uniforms: ResMut<AabbUniforms>,
) {
    aabb_uniforms.uniforms.clear();
    for (entity, aabb) in query.iter() {
        commands.entity(entity).insert(AabbUniformOffset {
            offset: aabb_uniforms.uniforms.push(AabbUniform {
                mins: aabb.mins,
                maxs: aabb.maxs,
            }),
        });
    }
    aabb_uniforms
        .uniforms
        .write_buffer(&*render_device, &*render_queue);
}

fn queue_aabbs(
    draw_functions: Res<DrawFunctions<Transparent3d>>,
    render_device: Res<RenderDevice>,
    aabb_pipeline: Res<AabbsPipeline>,
    view_uniforms: Res<ViewUniforms>,
    aabb_uniforms: Res<AabbUniforms>,
    mut aabb_meta: ResMut<AabbsMeta>,
    mut views: Query<&mut RenderPhase<Transparent3d>>,
    aabb_query: Query<Entity, With<AabbUniformOffset>>,
) {
    if let (Some(view_binding), Some(aabb_binding)) = (
        view_uniforms.uniforms.binding(),
        aabb_uniforms.uniforms.binding(),
    ) {
        aabb_meta.view_bind_group = Some(render_device.create_bind_group(&BindGroupDescriptor {
            entries: &[BindGroupEntry {
                binding: 0,
                resource: view_binding,
            }],
            label: Some("aabb_view_bind_group"),
            layout: &aabb_pipeline.view_layout,
        }));

        aabb_meta.aabbs_bind_group = Some(render_device.create_bind_group(&BindGroupDescriptor {
            entries: &[BindGroupEntry {
                binding: 0,
                resource: aabb_binding,
            }],
            label: Some("aabb_bind_group"),
            layout: &aabb_pipeline.aabbs_layout,
        }));

        let draw_aabb_function = draw_functions.read().get_id::<DrawAabb>().unwrap();

        for mut transparent_phase in &mut views {
            for entity in &aabb_query {
                transparent_phase.add(Transparent3d {
                    draw_function: draw_aabb_function,
                    pipeline: aabb_pipeline.pipeline_id,
                    entity,
                    distance: 0.0, // TODO: Distance from camera
                });
            }
        }
    }
}

#[derive(Component)]
pub struct AabbUniformOffset {
    pub offset: u32,
}

#[derive(Default)]
pub(crate) struct AabbUniforms {
    pub uniforms: DynamicUniformBuffer<AabbUniform>,
}

#[derive(Component, ShaderType)]
pub(crate) struct AabbUniform {
    mins: Vec3,
    maxs: Vec3,
}

struct AabbsPipeline {
    pub pipeline_id: CachedRenderPipelineId,
    view_layout: BindGroupLayout,
    aabbs_layout: BindGroupLayout,
}

impl FromWorld for AabbsPipeline {
    fn from_world(world: &mut World) -> Self {
        let render_device = world.resource::<RenderDevice>();

        let view_layout = render_device.create_bind_group_layout(&BindGroupLayoutDescriptor {
            entries: &[
                // View
                BindGroupLayoutEntry {
                    binding: 0,
                    visibility: ShaderStages::VERTEX | ShaderStages::FRAGMENT,
                    ty: BindingType::Buffer {
                        ty: BufferBindingType::Uniform,
                        has_dynamic_offset: true,
                        min_binding_size: Some(ViewUniform::min_size()),
                    },
                    count: None,
                },
            ],
            label: Some("aabb_view_layout"),
        });

        let aabbs_layout = render_device.create_bind_group_layout(&BindGroupLayoutDescriptor {
            // aabb
            label: Some("aabb_layout"),
            entries: &[
                BindGroupLayoutEntry {
                    binding: 0,
                    visibility: ShaderStages::VERTEX | ShaderStages::FRAGMENT,
                    ty: BindingType::Buffer {
                        ty: BufferBindingType::Uniform,
                        has_dynamic_offset: true,
                        min_binding_size: Some(AabbUniform::min_size()),
                    },
                    count: None,
                },
                // BindGroupLayoutEntry {
                //     binding: 1,
                //     visibility: ShaderStages::VERTEX | ShaderStages::FRAGMENT,
                //     ty: BindingType::Buffer {
                //         ty: BufferBindingType::Uniform,
                //         has_dynamic_offset: false,
                //         min_binding_size: Some(Color::min_size()),
                //     },
                //     count: None,
                // },
            ],
        });

        let vertex_format = vec![
            // Position
            VertexFormat::Float32x4,
            // Uv
            //VertexFormat::Float32x4,
            // Color
            //VertexFormat::Float32x4,
        ];

        let vertex_layout =
            VertexBufferLayout::from_vertex_formats(VertexStepMode::Vertex, vertex_format);

        // TODO: should be 1 default, but fails, using 4 for now
        let sample_count = world.get_resource::<Msaa>().map(|m| m.samples).unwrap_or(4);

        info!("Sample count: {}", sample_count);

        let pipeline_descriptor = RenderPipelineDescriptor {
            vertex: VertexState {
                shader: ENTITY_AABBS_SHADER.typed(),
                entry_point: "vertex".into(),
                shader_defs: vec![],
                buffers: vec![vertex_layout],
            },
            fragment: Some(FragmentState {
                shader: ENTITY_AABBS_SHADER.typed(),
                shader_defs: vec![],
                entry_point: "fragment".into(),
                targets: vec![Some(ColorTargetState {
                    format: TextureFormat::bevy_default(),
                    blend: Some(BlendState::REPLACE),
                    write_mask: ColorWrites::ALL,
                })],
            }),
            layout: Some(vec![view_layout.clone(), aabbs_layout.clone()]),
            primitive: PrimitiveState {
                front_face: FrontFace::Ccw,
                cull_mode: None,
                unclipped_depth: false,
                polygon_mode: PolygonMode::Line,
                conservative: false,
                topology: PrimitiveTopology::LineList,
                strip_index_format: None,
            },
            depth_stencil: Some(DepthStencilState {
                format: TextureFormat::Depth32Float,
                depth_compare: CompareFunction::Greater,
                stencil: StencilState {
                    front: StencilFaceState::IGNORE,
                    back: StencilFaceState::IGNORE,
                    read_mask: 0,
                    write_mask: 0,
                },
                bias: DepthBiasState {
                    constant: 0,
                    slope_scale: 0.0,
                    clamp: 0.0,
                },
                depth_write_enabled: true,
            }),
            multisample: MultisampleState {
                count: sample_count,
                mask: !0,
                alpha_to_coverage_enabled: false,
            },
            label: Some("aabb_pipeline".into()),
        };

        let mut pipeline_cache = world.resource_mut::<PipelineCache>();
        let pipeline_id = pipeline_cache.queue_render_pipeline(pipeline_descriptor);

        Self {
            pipeline_id,
            view_layout,
            aabbs_layout,
        }
    }
}

#[derive(Debug, Component, Clone, Copy, PartialEq, Eq, Hash)]
pub struct AabbPipelineKey;

pub(crate) struct AabbsMeta {
    vertex_buffer: Buffer,
    index_buffer: Buffer,
    view_bind_group: Option<BindGroup>,
    aabbs_bind_group: Option<BindGroup>,
}

impl FromWorld for AabbsMeta {
    fn from_world(world: &mut World) -> Self {
        let render_device = world.resource::<RenderDevice>();

        let vertex_buffer = render_device.create_buffer_with_data(&BufferInitDescriptor {
            usage: BufferUsages::VERTEX,
            label: Some("aabb_vertex_buffer"),
            contents: cast_slice(&AABB_VERTEX_POSITIONS),
        });

        let index_buffer = render_device.create_buffer_with_data(&BufferInitDescriptor {
            usage: BufferUsages::INDEX,
            contents: cast_slice(&AABB_INDICES),
            label: Some("aabb_index_buffer"),
        });

        Self {
            vertex_buffer,
            index_buffer,
            view_bind_group: None,
            aabbs_bind_group: None,
        }
    }
}

type DrawAabb = (
    SetAabbPipeline,
    SetAabbViewBindGroup<0>,
    SetAabbBindGroup<1>,
    DrawAabbInstance,
);


struct SetAabbPipeline;
impl<P: PhaseItem> RenderCommand<P> for SetAabbPipeline {
    type Param = (SRes<PipelineCache>, SRes<AabbsPipeline>);
    #[inline]
    fn render<'w>(
        _view: Entity,
        _item: &P,
        params: SystemParamItem<'w, '_, Self::Param>,
        pass: &mut TrackedRenderPass<'w>,
    ) -> RenderCommandResult {
        let (pipeline_cache, aabb_pipeline) = params;
        if let Some(pipeline) = pipeline_cache
            .into_inner()
            .get_render_pipeline(aabb_pipeline.pipeline_id)
        {
            pass.set_render_pipeline(pipeline);
            RenderCommandResult::Success
        } else {
            RenderCommandResult::Failure
        }
    }
}

struct SetAabbViewBindGroup<const I: usize>;
impl<const I: usize> EntityRenderCommand for SetAabbViewBindGroup<I> {
    type Param = (SRes<AabbsMeta>, SQuery<Read<ViewUniformOffset>>);

    #[inline]
    fn render<'w>(
        view: Entity,
        _item: Entity,
        (aabb_meta, view_query): SystemParamItem<'w, '_, Self::Param>,
        pass: &mut TrackedRenderPass<'w>,
    ) -> RenderCommandResult {
        let view_uniform = view_query.get(view).unwrap();
        pass.set_bind_group(
            I,
            aabb_meta.into_inner().view_bind_group.as_ref().unwrap(),
            &[view_uniform.offset],
        );
        RenderCommandResult::Success
    }
}

struct SetAabbBindGroup<const I: usize>;
impl<const I: usize> EntityRenderCommand for SetAabbBindGroup<I> {
    type Param = (SRes<AabbsMeta>, SQuery<Read<AabbUniformOffset>>);

    #[inline]
    fn render<'w>(
        _view: Entity,
        item: Entity,
        (aabb_meta, aabb_query): SystemParamItem<'w, '_, Self::Param>,
        pass: &mut TrackedRenderPass<'w>,
    ) -> RenderCommandResult {
        let aabb_uniform = aabb_query.get(item).unwrap();
        pass.set_bind_group(
            I,
            aabb_meta.into_inner().aabbs_bind_group.as_ref().unwrap(),
            &[aabb_uniform.offset],
        );
        RenderCommandResult::Success
    }
}

struct DrawAabbInstance;
impl EntityRenderCommand for DrawAabbInstance {
    type Param = SRes<AabbsMeta>;

    #[inline]
    fn render<'w>(
        _view: Entity,
        _item: Entity,
        aabbs_meta: SystemParamItem<'w, '_, Self::Param>,
        pass: &mut TrackedRenderPass<'w>,
    ) -> RenderCommandResult {
        let aabbs_meta = aabbs_meta.into_inner();

        pass.set_vertex_buffer(0, aabbs_meta.vertex_buffer.slice(..));
        pass.set_index_buffer(aabbs_meta.index_buffer.slice(..), 0, IndexFormat::Uint32);
        pass.draw_indexed(0..AABB_INDICES_LEN, 0, 0..1);
        RenderCommandResult::Success
    }
}
