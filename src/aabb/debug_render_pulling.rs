use super::Aabb;

use bevy::{
    ecs::system::{
        lifetimeless::{Read, SQuery, SRes},
        SystemParamItem,
    },
    math::vec3,
    pbr::LightMeta,
    prelude::*,
    reflect::TypeUuid,
    render::{
        extract_resource::{ExtractResource, ExtractResourcePlugin},
        render_graph::{self, NodeRunError, RenderGraphContext, SlotInfo, SlotType, RenderGraph},
        render_phase::{
            AddRenderCommand, DrawFunctionId, DrawFunctions, PhaseItem, RenderCommand,
            RenderCommandResult, RenderPhase, TrackedRenderPass,
        },
        render_resource::{
            BindGroup, BindGroupDescriptor, BindGroupEntry, BindGroupLayout,
            BindGroupLayoutDescriptor, BindGroupLayoutEntry, BindingType, BlendState, Buffer,
            BufferBindingType, BufferInitDescriptor, BufferSize, BufferUsages,
            CachedRenderPipelineId, ColorTargetState, ColorWrites, CompareFunction, DepthBiasState,
            DepthStencilState, Face, FragmentState, FrontFace, IndexFormat, LoadOp,
            MultisampleState, Operations, PipelineCache, PolygonMode, PrimitiveState,
            PrimitiveTopology, RenderPassDepthStencilAttachment, RenderPassDescriptor,
            RenderPipelineDescriptor, ShaderStages, ShaderType, StencilFaceState, StencilState,
            StorageBuffer, TextureFormat, VertexState,
        },
        renderer::{RenderContext, RenderDevice, RenderQueue},
        texture::BevyDefault,
        view::{ExtractedView, ViewDepthTexture, ViewTarget, ViewUniform, ViewUniformOffset},
        Extract, RenderApp, RenderStage,
    }, core_pipeline::core_3d,
};
use bytemuck::cast_slice;

const AABBS_SHADER_HANDLE: HandleUntyped =
    HandleUntyped::weak_from_u64(Shader::TYPE_UUID, 15395570175632164571);

pub struct DebugAabbPlugin;

impl Plugin for DebugAabbPlugin {
    fn build(&self, app: &mut App) {

        info!("DebugAabbPlugin::build");

        app.add_startup_system(setup);
        app.world.resource_mut::<Assets<Shader>>().set_untracked(
            AABBS_SHADER_HANDLE,
            Shader::from_wgsl(include_str!("aabbs.wgsl")),
        );
        app.add_plugin(ExtractResourcePlugin::<Aabbs>::default());

        let render_app = app.sub_app_mut(RenderApp);
        render_app
            .init_resource::<DrawFunctions<AabbsPhaseItem>>()            
            .add_render_command::<AabbsPhaseItem, DrawAabbs>()            
            .init_resource::<AabbsPipeline>()
            .add_system_to_stage(RenderStage::Extract, extract_aabbs_phase)
            .add_system_to_stage(RenderStage::Prepare, prepare_aabbs)
            .add_system_to_stage(RenderStage::Queue, queue_aabbs);

            let aabbs_pass_node = AabbsPassNode::new(&mut render_app.world);
            let mut graph = render_app.world.resource_mut::<RenderGraph>();
            let draw_3d_graph = graph.get_sub_graph_mut(core_3d::graph::NAME).unwrap();
            draw_3d_graph.add_node(node::AABBS_PASS, aabbs_pass_node);
            draw_3d_graph
                .add_node_edge(core_3d::graph::node::MAIN_PASS, node::AABBS_PASS)
                .unwrap();
            draw_3d_graph
                .add_slot_edge(
                    draw_3d_graph.input_node().unwrap().id,
                    core_3d::graph::input::VIEW_ENTITY,
                    node::AABBS_PASS,
                    AabbsPassNode::IN_VIEW,
                )
                .unwrap();
    }
}

fn setup(mut commands: Commands) {
    // add aabb list
    commands.insert_resource(Aabbs {
        data: vec![
            Aabb::new(vec3(0.0, 0.0, 0.0), vec3(1.0, 1.0, 1.0)),
            Aabb::new(vec3(1.0, 1.0, 1.0), vec3(2.0, 2.0, 2.0)),
            Aabb::new(vec3(2.0, 2.0, 2.0), vec3(3.0, 3.0, 3.0)),
        ],
    });
}

fn extract_aabbs_phase(mut commands: Commands, cameras: Extract<Query<Entity, With<Camera3d>>>) {
    for entity in cameras.iter() {
        commands
            .get_or_spawn(entity)
            .insert(RenderPhase::<AabbsPhaseItem>::default());
    }
}

#[derive(Clone, Debug, Default, ExtractResource)]
struct Aabbs {
    data: Vec<Aabb>,
}



#[derive(Clone, Copy, Debug, Default, ShaderType)]
struct GpuAabb {
    // mins: Vec3,
    // maxs: Vec3,
    // color: [f32; 4],
    center: Vec3,
    flags: u32,
    half_extents: Vec4,
    color: [f32; 4],
}

impl From<&Aabb> for GpuAabb {
    fn from(aabb: &Aabb) -> Self {
        Self {
            mins: aabb.mins,
            maxs: aabb.maxs,
            color: Color::RED.as_rgba_f32(),
        }
    }
}

struct GpuAabbs {
    index_buffer: Option<Buffer>,
    index_count: u32,
    instances: StorageBuffer<GpuAabbsArray>,
    bind_group: Option<BindGroup>,
}

impl Default for GpuAabbs {
    fn default() -> Self {
        let mut instances = StorageBuffer::<GpuAabbsArray>::default();
        instances.set_label(Some("gpu_aabbs_array"));
        Self {
            index_buffer: None,
            index_count: 0,
            instances,
            bind_group: None,
        }
    }
}

#[derive(Default, ShaderType)]
struct GpuAabbsArray {
    #[size(runtime)]
    array: Vec<GpuAabb>,
}

fn prepare_aabbs(
    mut commands: Commands,
    aabbs: Option<Res<Aabbs>>,
    render_device: Res<RenderDevice>,
    render_queue: Res<RenderQueue>,
    gpu_aabbs: Option<ResMut<GpuAabbs>>,
) {
    if let Some(aabbs) = aabbs {
        if aabbs.is_changed() {
            let mut new_gpu_aabbs = None;
            let gpu_aabbs = if let Some(gpu_aabbs) = gpu_aabbs {
                gpu_aabbs.into_inner()
            } else {
                new_gpu_aabbs = Some(GpuAabbs::default());
                new_gpu_aabbs.as_mut().unwrap()
            };
            for aabb in aabbs.data.iter() {
                gpu_aabbs
                    .instances
                    .get_mut()
                    .array
                    .push(GpuAabb::from(aabb));
            }
            let n_instances = gpu_aabbs.instances.get().array.len();
            info!("n_instances: {}", n_instances);
            gpu_aabbs.index_count = n_instances as u32 * 6;

            let mut indices = Vec::with_capacity(gpu_aabbs.index_count as usize);
            for i in 0..n_instances {
                let base = (i * 4) as u32;
                indices.push(base + 2);
                indices.push(base);
                indices.push(base + 1);
                indices.push(base + 1);
                indices.push(base + 3);
                indices.push(base + 2);
            }
            gpu_aabbs.index_buffer = Some(render_device.create_buffer_with_data(
                &BufferInitDescriptor {
                    label: Some("gpu_aabbs_index_buffer"),
                    contents: cast_slice(&indices),
                    usage: BufferUsages::INDEX,
                },
            ));

            gpu_aabbs
                .instances
                .write_buffer(&*render_device, &*render_queue);

            if let Some(new_gpu_aabbs) = new_gpu_aabbs {
                commands.insert_resource(new_gpu_aabbs);
            }
        }
    }
}

pub struct AabbsPhaseItem {
    pub draw_function: DrawFunctionId,
}

impl PhaseItem for AabbsPhaseItem {
    type SortKey = u32;

    #[inline]
    fn sort_key(&self) -> Self::SortKey {
        0
    }

    #[inline]
    fn draw_function(&self) -> DrawFunctionId {
        self.draw_function
    }
}

fn queue_aabbs(
    opaque_3d_draw_functions: Res<DrawFunctions<AabbsPhaseItem>>,
    aabbs_pipeline: Res<AabbsPipeline>,
    render_device: Res<RenderDevice>,
    gpu_aabbs: Option<ResMut<GpuAabbs>>,
    mut views: Query<&mut RenderPhase<AabbsPhaseItem>>,
) {
    if let Some(mut gpu_aabbs) = gpu_aabbs {
        let draw_aabbs = opaque_3d_draw_functions
            .read()
            .get_id::<DrawAabbs>()
            .unwrap();

        if gpu_aabbs.is_changed() {
            println!("GpuAabbs changed");
            gpu_aabbs.bind_group = Some(render_device.create_bind_group(&BindGroupDescriptor {
                label: Some("gpu_aabbs_bind_group"),
                layout: &aabbs_pipeline.aabbs_layout,
                entries: &[BindGroupEntry {
                    binding: 0,
                    resource: gpu_aabbs.instances.buffer().unwrap().as_entire_binding(),
                }],
            }));
        }

        for mut opaque_phase in views.iter_mut() {
            opaque_phase.add(AabbsPhaseItem {
                draw_function: draw_aabbs,
            });
        }
    }
}

mod node {
    pub const AABBS_PASS: &str = "aabbs_pass";
}

pub struct AabbsPassNode {
    query: QueryState<
        (
            &'static RenderPhase<AabbsPhaseItem>,
            &'static ViewTarget,
            &'static ViewDepthTexture,
        ),
        With<ExtractedView>,
    >,
}

impl AabbsPassNode {
    pub const IN_VIEW: &'static str = "view";

    pub fn new(world: &mut World) -> Self {
        Self {
            query: QueryState::new(world),
        }
    }
}

impl render_graph::Node for AabbsPassNode {
    fn input(&self) -> Vec<SlotInfo> {
        vec![SlotInfo::new(AabbsPassNode::IN_VIEW, SlotType::Entity)]
    }

    fn update(&mut self, world: &mut World) {
        self.query.update_archetypes(world);
    }

    fn run(
        &self,
        graph: &mut RenderGraphContext,
        render_context: &mut RenderContext,
        world: &World,
    ) -> Result<(), NodeRunError> {
        let view_entity = graph.get_input_entity(Self::IN_VIEW)?;
        let (aabbs_phase, target, depth) = match self.query.get_manual(world, view_entity) {
            Ok(query) => query,
            Err(_) => return Ok(()), // No window
        };

        #[cfg(feature = "trace")]
        let _main_aabbs_pass_span = info_span!("main_aabbs_pass").entered();
        let pass_descriptor = RenderPassDescriptor {
            label: Some("main_aabbs_pass"),
            // NOTE: The quads pass loads the color
            // buffer as well as writing to it.
            color_attachments: &[Some(target.get_color_attachment(Operations {
                load: LoadOp::Load,
                store: true,
            }))],
            depth_stencil_attachment: Some(RenderPassDepthStencilAttachment {
                view: &depth.view,
                // NOTE: The quads main pass loads the depth buffer and possibly overwrites it
                depth_ops: Some(Operations {
                    load: LoadOp::Load,
                    store: true,
                }),
                stencil_ops: None,
            }),
        };

        let draw_functions = world.resource::<DrawFunctions<AabbsPhaseItem>>();

        let render_pass = render_context
            .command_encoder
            .begin_render_pass(&pass_descriptor);
        let mut draw_functions = draw_functions.write();
        let mut tracked_pass = TrackedRenderPass::new(render_pass);
        for item in &aabbs_phase.items {
            let draw_function = draw_functions.get_mut(item.draw_function).unwrap();
            draw_function.draw(world, &mut tracked_pass, view_entity, item);
        }

        Ok(())
    }
}

struct AabbsPipeline {
    pipeline_id: CachedRenderPipelineId,
    aabbs_layout: BindGroupLayout,
}

impl FromWorld for AabbsPipeline {
    fn from_world(world: &mut World) -> Self {
        let view_layout =
            world
                .resource::<RenderDevice>()
                .create_bind_group_layout(&BindGroupLayoutDescriptor {
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
                    label: Some("shadow_view_layout"),
                });

        let aabbs_layout =
            world
                .resource::<RenderDevice>()
                .create_bind_group_layout(&BindGroupLayoutDescriptor {
                    label: None,
                    entries: &[BindGroupLayoutEntry {
                        binding: 0,
                        visibility: ShaderStages::VERTEX,
                        ty: BindingType::Buffer {
                            ty: BufferBindingType::Storage { read_only: true },
                            has_dynamic_offset: false,
                            min_binding_size: BufferSize::new(0),
                        },
                        count: None,
                    }],
                });

        let mut pipeline_cache = world.resource_mut::<PipelineCache>();
        let pipeline_id = pipeline_cache.queue_render_pipeline(RenderPipelineDescriptor {
            label: Some("aabbs_pipeline".into()),
            layout: Some(vec![view_layout, aabbs_layout.clone()]),
            vertex: VertexState {
                shader: AABBS_SHADER_HANDLE.typed(),
                shader_defs: vec![],
                entry_point: "vertex".into(),
                buffers: vec![],
            },
            fragment: Some(FragmentState {
                shader: AABBS_SHADER_HANDLE.typed(),
                shader_defs: vec![],
                entry_point: "fragment".into(),
                targets: vec![Some(ColorTargetState {
                    format: TextureFormat::bevy_default(),
                    blend: Some(BlendState::REPLACE),
                    write_mask: ColorWrites::ALL,
                })],
            }),
            primitive: PrimitiveState {
                front_face: FrontFace::Ccw,
                cull_mode: Some(Face::Back),
                unclipped_depth: false,
                polygon_mode: PolygonMode::Fill,
                conservative: false,
                topology: PrimitiveTopology::TriangleList,
                strip_index_format: None,
            },
            depth_stencil: Some(DepthStencilState {
                format: TextureFormat::Depth32Float,
                depth_write_enabled: true,
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
            }),
            multisample: MultisampleState {
                count: Msaa::default().samples,
                mask: !0,
                alpha_to_coverage_enabled: false,
            },
        });

        Self {
            pipeline_id,
            aabbs_layout,
        }
    }
}

type DrawAabbs = (
    SetAabbsPipeline,
    SetAabbsShadowViewBindGroup<0>,
    SetGpuAabbsBindGroup<1>,
    DrawVertexPulledAabbs,
);

struct SetAabbsPipeline;

impl<P: PhaseItem> RenderCommand<P> for SetAabbsPipeline {
    type Param = (SRes<PipelineCache>, SRes<AabbsPipeline>);
    #[inline]
    fn render<'w>(
        _view: Entity,
        _item: &P,
        params: SystemParamItem<'w, '_, Self::Param>,
        pass: &mut TrackedRenderPass<'w>,
    ) -> RenderCommandResult {
        let (pipeline_cache, aabbs_pipeline) = params;
        if let Some(pipeline) = pipeline_cache
            .into_inner()
            .get_render_pipeline(aabbs_pipeline.pipeline_id)
        {
            pass.set_render_pipeline(pipeline);
            RenderCommandResult::Success
        } else {
            RenderCommandResult::Failure
        }
    }
}

pub struct SetAabbsShadowViewBindGroup<const I: usize>;
impl<const I: usize, P: PhaseItem> RenderCommand<P> for SetAabbsShadowViewBindGroup<I> {
    type Param = (SRes<LightMeta>, SQuery<Read<ViewUniformOffset>>);
    #[inline]
    fn render<'w>(
        view: Entity,
        _item: &P,
        (light_meta, view_query): SystemParamItem<'w, '_, Self::Param>,
        pass: &mut TrackedRenderPass<'w>,
    ) -> RenderCommandResult {
        let view_uniform_offset = view_query.get(view).unwrap();
        pass.set_bind_group(
            I,
            light_meta
                .into_inner()
                .shadow_view_bind_group
                .as_ref()
                .unwrap(),
            &[view_uniform_offset.offset],
        );

        RenderCommandResult::Success
    }
}

struct SetGpuAabbsBindGroup<const I: usize>;
impl<const I: usize, P: PhaseItem> RenderCommand<P> for SetGpuAabbsBindGroup<I> {
    type Param = SRes<GpuAabbs>;

    #[inline]
    fn render<'w>(
        _view: Entity,
        _item: &P,
        gpu_aabbs: SystemParamItem<'w, '_, Self::Param>,
        pass: &mut TrackedRenderPass<'w>,
    ) -> RenderCommandResult {
        let gpu_aabbs = gpu_aabbs.into_inner();
        pass.set_bind_group(I, gpu_aabbs.bind_group.as_ref().unwrap(), &[]);

        RenderCommandResult::Success
    }
}

struct DrawVertexPulledAabbs;
impl<P: PhaseItem> RenderCommand<P> for DrawVertexPulledAabbs {
    type Param = SRes<GpuAabbs>;

    #[inline]
    fn render<'w>(
        _view: Entity,
        _item: &P,
        gpu_aabbs: SystemParamItem<'w, '_, Self::Param>,
        pass: &mut TrackedRenderPass<'w>,
    ) -> RenderCommandResult {
        let gpu_aabbs = gpu_aabbs.into_inner();
        pass.set_index_buffer(
            gpu_aabbs.index_buffer.as_ref().unwrap().slice(..),
            0,
            IndexFormat::Uint32,
        );
        pass.draw_indexed(0..gpu_aabbs.index_count, 0, 0..1);
        RenderCommandResult::Success
    }
}
