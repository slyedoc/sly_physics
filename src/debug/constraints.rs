use std::mem;

use bevy::{
    asset::load_internal_asset,
    core_pipeline::core_3d::Transparent3d,
    ecs::system::{
        lifetimeless::{Read, SQuery, SRes},
        SystemParamItem,
    },
    math::vec3,
    prelude::*,
    reflect::TypeUuid,
    render::{
        mesh::Indices,
        render_phase::*,
        render_resource::*,
        renderer::RenderDevice,
        texture::BevyDefault,
        view::{ViewUniform, ViewUniformOffset, ViewUniforms},
        Extract, RenderApp, RenderStage,
    },
};
use bytemuck::{cast_slice, Pod, Zeroable};
use iyes_loopless::state::CurrentState;

use super::PhysicsDebugState;
use crate::constraints::*;

const CONSTRAINTS_SHADER: HandleUntyped =
    HandleUntyped::weak_from_u64(Shader::TYPE_UUID, 10740125454777370035);

pub struct DebugConstraintsPlugin;

impl Plugin for DebugConstraintsPlugin {
    fn build(&self, app: &mut App) {
        load_internal_asset!(
            app,
            CONSTRAINTS_SHADER,
            "constraints.wgsl",
            Shader::from_wgsl
        );

        if let Ok(render_app) = app.get_sub_app_mut(RenderApp) {
            render_app
                .init_resource::<ConstraintMeshes>()
                .init_resource::<ConstraintsPipeline>()
                .init_resource::<ConstraintMeta>()
                .init_resource::<GpuLines>()
                .add_render_command::<Transparent3d, DrawConstraints>()
                .add_system_to_stage(RenderStage::Extract, extract_anchors::<MotorConstraint>)
                .add_system_to_stage(
                    RenderStage::Extract,
                    extract_anchors::<OrientationConstraint>,
                )
                .add_system_to_stage(RenderStage::Extract, extract_anchors::<HingeConstraint>)
                .add_system_to_stage(
                    RenderStage::Extract,
                    extract_anchors::<HingeLimitedConstraint>,
                )
                .add_system_to_stage(RenderStage::Extract, extract_anchors::<DistanceConstraint>)
                .add_system_to_stage(RenderStage::Prepare, prepare_constraints)
                .add_system_to_stage(RenderStage::Queue, queue_constraints)
                .add_system_to_stage(RenderStage::Cleanup, cleanup);
        }
    }
}

#[derive(Deref)]
struct ConstraintMeshes {
    line_mesh: Mesh,
}

impl Default for ConstraintMeshes {
    fn default() -> Self {
        let mut line_mesh = Mesh::new(PrimitiveTopology::LineList);
        line_mesh.insert_attribute(
            Mesh::ATTRIBUTE_POSITION,
            vec![Vec3::ZERO.to_array(), vec3(0.0, 5.0, 0.0).to_array()],
        );
        line_mesh.set_indices(Some(Indices::U32(vec![0, 1])));

        Self { line_mesh }
    }
}

#[derive(Clone, Debug, Default)]
struct GpuLines {
    lines: Vec<Line>,
}

#[derive(Clone, Copy, Debug, Default, Pod, Zeroable)]
#[repr(C)]
struct Line {
    start: Vec3,
    end: Vec3,
}

struct ConstraintsPipeline {
    pub pipeline_id: CachedRenderPipelineId,
    view_layout: BindGroupLayout,
}

impl FromWorld for ConstraintsPipeline {
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
            label: Some("constrains_view_layout"),
        });

        // TODO: should be 1 default, but fails, using 4 for now
        let sample_count = world.get_resource::<Msaa>().map(|m| m.samples).unwrap_or(4);
        //let mesh = &world.get_resource::<ContactMesh>().unwrap().mesh;
        //let vertex_layout = mesh.get_mesh_vertex_buffer_layout().layout().clone();

        let formats = vec![VertexFormat::Float32x3];

        let vertex_layout =
            VertexBufferLayout::from_vertex_formats(VertexStepMode::Vertex, formats);

        let vec3_size = mem::size_of::<[f32; 3]>() as u64;
        let instance_vertex_layout = VertexBufferLayout {
            array_stride: vec3_size * 2,
            step_mode: VertexStepMode::Instance,
            attributes: vec![
                VertexAttribute {
                    format: VertexFormat::Float32x3,
                    offset: 0,
                    shader_location: 3, // shader locations 0-2 are taken up by Position, Normal and UV attributes
                },
                VertexAttribute {
                    format: VertexFormat::Float32x3,
                    offset: vec3_size,
                    shader_location: 4, //
                },
            ],
        };

        let pipeline_descriptor = RenderPipelineDescriptor {
            vertex: VertexState {
                shader: CONSTRAINTS_SHADER.typed(),
                entry_point: "vertex".into(),
                shader_defs: vec![],
                buffers: vec![vertex_layout, instance_vertex_layout],
            },
            fragment: Some(FragmentState {
                shader: CONSTRAINTS_SHADER.typed(),
                shader_defs: vec![],
                entry_point: "fragment".into(),
                targets: vec![Some(ColorTargetState {
                    format: TextureFormat::bevy_default(),
                    blend: Some(BlendState::REPLACE),
                    write_mask: ColorWrites::ALL,
                })],
            }),
            layout: Some(vec![view_layout.clone()]),
            primitive: PrimitiveState {
                front_face: FrontFace::Ccw,
                cull_mode: None,
                unclipped_depth: false,
                polygon_mode: PolygonMode::Fill,
                conservative: false,
                topology: PrimitiveTopology::LineList,
                strip_index_format: None,
            },
            depth_stencil: Some(DepthStencilState {
                format: TextureFormat::Depth32Float,
                depth_compare: CompareFunction::Always,
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
            label: Some("constraints_pipeline".into()),
        };

        let mut pipeline_cache = world.resource_mut::<PipelineCache>();
        let pipeline_id = pipeline_cache.queue_render_pipeline(pipeline_descriptor);

        Self {
            pipeline_id,
            view_layout,
        }
    }
}

struct ConstraintMeta {
    instance_buffer: Buffer,
    instance_count: u32,
    vertex_buffer: Buffer,
    index_buffer: Buffer,
    index_count: u32,
    view_bind_group: Option<BindGroup>,
}

impl FromWorld for ConstraintMeta {
    fn from_world(world: &mut World) -> Self {
        let meshes = &world.resource::<ConstraintMeshes>();
        let render_device = world.resource::<RenderDevice>();

        let vertex_buffer = render_device.create_buffer_with_data(&BufferInitDescriptor {
            usage: BufferUsages::VERTEX,
            label: Some("constraint_line_vertex_buffer"),
            contents: &meshes.line_mesh.get_vertex_buffer_data(),
        });

        let index_buffer = render_device.create_buffer_with_data(&BufferInitDescriptor {
            usage: BufferUsages::INDEX,
            label: Some("constraint_line_index_buffer"),
            contents: meshes.line_mesh.get_index_buffer_bytes().unwrap(),
        });

        let instance_buffer = render_device.create_buffer_with_data(&BufferInitDescriptor {
            label: Some("constraint_line_instance_buffer"),
            contents: &[],
            usage: BufferUsages::VERTEX,
        });
        let index_count = meshes.line_mesh.indices().unwrap().iter().count() as u32;

        Self {
            instance_buffer,
            instance_count: 0,
            vertex_buffer,
            index_buffer,
            index_count,
            view_bind_group: None,
        }
    }
}

fn extract_anchors<T: Component + Constrainable>(
    current_state: Extract<Res<CurrentState<PhysicsDebugState>>>,
    constraint_query: Extract<Query<(Entity, &T)>>,
    rb_query: Extract<Query<&Transform>>,
    mut gpu_lines: ResMut<GpuLines>,
) {
    if current_state.0 == PhysicsDebugState::Running {
        for (e, constraint) in constraint_query
            .iter()
            .filter(|(_e, c)| c.get_b().is_some())
        {
            if let Ok([trans_a, trans_b]) = rb_query.get_many([e, constraint.get_b().unwrap()])
            {
                gpu_lines.lines.push(Line {
                    start: trans_a.translation,
                    end: trans_a.translation + (trans_a.rotation * constraint.get_anchor_a()),
                });

                gpu_lines.lines.push(Line {
                    start: trans_b.translation,
                    end: trans_b.translation + (trans_b.rotation * constraint.get_anchor_b()),
                });
            }
        }
    }
}

fn prepare_constraints(
    gpu_lines: Res<GpuLines>,
    render_device: Res<RenderDevice>,
    mut constraints_meta: ResMut<ConstraintMeta>,
) {
    constraints_meta.instance_buffer =
        render_device.create_buffer_with_data(&BufferInitDescriptor {
            label: Some("contact_instance_buffer"),
            contents: cast_slice(&gpu_lines.lines),
            usage: BufferUsages::VERTEX,
        });
    constraints_meta.instance_count = gpu_lines.lines.len() as u32;
}

fn queue_constraints(
    draw_functions: Res<DrawFunctions<Transparent3d>>,
    contacts_pipeline: Res<ConstraintsPipeline>,
    render_device: Res<RenderDevice>,
    mut aabb_meta: ResMut<ConstraintMeta>,
    view_uniforms: Res<ViewUniforms>,
    mut views: Query<&mut RenderPhase<Transparent3d>>,
) {
    let draw_aabb_function = draw_functions.read().get_id::<DrawConstraints>().unwrap();

    if let Some(view_binding) = view_uniforms.uniforms.binding() {
        aabb_meta.view_bind_group = Some(render_device.create_bind_group(&BindGroupDescriptor {
            entries: &[BindGroupEntry {
                binding: 0,
                resource: view_binding,
            }],
            label: Some("contact_view_bind_group"),
            layout: &contacts_pipeline.view_layout,
        }));
    }

    for mut transparent3d_phase in views.iter_mut() {
        transparent3d_phase.add(Transparent3d {
            draw_function: draw_aabb_function,
            pipeline: contacts_pipeline.pipeline_id,
            entity: Entity::from_raw(0),
            distance: 0.0,
        });
    }
}

fn cleanup(mut gpu_lines: ResMut<GpuLines>) {
    gpu_lines.lines.clear();
}

type DrawConstraints = (
    SetConstraintsPipeline,
    SetConstraintsViewBindGroup<0>,
    DrawConstraintsInstances,
);

struct SetConstraintsPipeline;
impl<P: PhaseItem> RenderCommand<P> for SetConstraintsPipeline {
    type Param = (SRes<PipelineCache>, SRes<ConstraintsPipeline>);
    #[inline]
    fn render<'w>(
        _view: Entity,
        _item: &P,
        params: SystemParamItem<'w, '_, Self::Param>,
        pass: &mut TrackedRenderPass<'w>,
    ) -> RenderCommandResult {
        let (pipeline_cache, pipeline) = params;
        if let Some(pipeline) = pipeline_cache
            .into_inner()
            .get_render_pipeline(pipeline.pipeline_id)
        {
            pass.set_render_pipeline(pipeline);
            RenderCommandResult::Success
        } else {
            RenderCommandResult::Failure
        }
    }
}

struct SetConstraintsViewBindGroup<const I: usize>;
impl<const I: usize> EntityRenderCommand for SetConstraintsViewBindGroup<I> {
    type Param = (SRes<ConstraintMeta>, SQuery<Read<ViewUniformOffset>>);

    #[inline]
    fn render<'w>(
        view: Entity,
        _item: Entity,
        (contact_meta, view_query): SystemParamItem<'w, '_, Self::Param>,
        pass: &mut TrackedRenderPass<'w>,
    ) -> RenderCommandResult {
        let view_uniform = view_query.get(view).unwrap();
        pass.set_bind_group(
            I,
            contact_meta.into_inner().view_bind_group.as_ref().unwrap(),
            &[view_uniform.offset],
        );
        RenderCommandResult::Success
    }
}

struct DrawConstraintsInstances;
impl EntityRenderCommand for DrawConstraintsInstances {
    type Param = SRes<ConstraintMeta>;

    #[inline]
    fn render<'w>(
        _view: Entity,
        _item: Entity,
        meta: SystemParamItem<'w, '_, Self::Param>,
        pass: &mut TrackedRenderPass<'w>,
    ) -> RenderCommandResult {
        let meta = meta.into_inner();
        pass.set_vertex_buffer(0, meta.vertex_buffer.slice(..));
        pass.set_vertex_buffer(1, meta.instance_buffer.slice(..));
        pass.set_index_buffer(meta.index_buffer.slice(..), 0, IndexFormat::Uint32);
        pass.draw_indexed(0..meta.index_count, 0, 0..meta.instance_count);
        RenderCommandResult::Success
    }
}
