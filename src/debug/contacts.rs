use std::mem;

use bevy::{
    asset::load_internal_asset,
    core_pipeline::core_3d::Transparent3d,
    ecs::system::{
        lifetimeless::{Read, SQuery, SRes},
        SystemParamItem,
    },
    prelude::*,
    reflect::TypeUuid,
    render::{
        render_phase::*,
        render_resource::*,
        renderer::{RenderDevice},
        texture::BevyDefault,
        view::{ViewUniform, ViewUniformOffset, ViewUniforms},
        Extract, RenderApp, RenderStage,
    },
};
use bytemuck::{cast_slice, Pod, Zeroable};
use iyes_loopless::state::CurrentState;

use super::PhysicsDebugState;
use crate::prelude::PenetrationArena;

const CONTACTS_SHADER: HandleUntyped =
    HandleUntyped::weak_from_u64(Shader::TYPE_UUID, 12103330032048703586);

pub struct DebugContactsPlugin;

impl Plugin for DebugContactsPlugin {
    fn build(&self, app: &mut App) {
        load_internal_asset!(app, CONTACTS_SHADER, "contacts.wgsl", Shader::from_wgsl);

        if let Ok(render_app) = app.get_sub_app_mut(RenderApp) {
            render_app
                .init_resource::<ContactMesh>()
                .init_resource::<ContactsPipeline>()
                .init_resource::<ContactsMeta>()
                .init_resource::<Contacts>()
                .add_render_command::<Transparent3d, DrawContacts>()
                .add_system_to_stage(RenderStage::Extract, extract_contacts)
                .add_system_to_stage(RenderStage::Prepare, prepare_contacts)
                .add_system_to_stage(RenderStage::Queue, queue_contacts);
        }
    }
}


struct ContactMesh {
    mesh: Mesh
}

impl Default for ContactMesh {
    fn default() -> Self {
        Self { 
            mesh: Mesh::from(shape::UVSphere {
                radius: 0.1,
                sectors: 5,
                stacks: 5,
            })
        }
    }
}


#[derive(Clone, Debug, Default)]
struct Contacts {
    data: Vec<GpuContact>,
}

#[derive(Clone, Copy, Debug, Default, Pod, Zeroable)]
#[repr(C)]
struct GpuContact {
    position: Vec3,
}

struct ContactsPipeline {
    pub pipeline_id: CachedRenderPipelineId,
    view_layout: BindGroupLayout,
}

impl FromWorld for ContactsPipeline {
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
            label: Some("contacts_view_layout"),
        });

        // TODO: should be 1 default, but fails, using 4 for now
        let sample_count = world.get_resource::<Msaa>().map(|m| m.samples).unwrap_or(4);
        //let mesh = &world.get_resource::<ContactMesh>().unwrap().mesh;
        //let vertex_layout = mesh.get_mesh_vertex_buffer_layout().layout().clone();

        let formats = vec![
            // position
            VertexFormat::Float32x3,
            // normal
            VertexFormat::Float32x3,
            // uv
            VertexFormat::Float32x2,
        ];

        let vertex_layout = VertexBufferLayout::from_vertex_formats(VertexStepMode::Vertex, formats);        

        let instance_vertex_layout = VertexBufferLayout {
            array_stride: mem::size_of::<[f32;3]>() as u64,
            step_mode: VertexStepMode::Instance,
            attributes: vec![
                VertexAttribute {
                    format: VertexFormat::Float32x3,
                    offset: 0,
                    shader_location: 3, // shader locations 0-2 are taken up by Position, Normal and UV attributes
                },
            ],
        };

        let pipeline_descriptor = RenderPipelineDescriptor {
            vertex: VertexState {
                shader: CONTACTS_SHADER.typed(),
                entry_point: "vertex".into(),
                shader_defs: vec![],
                buffers: vec![
                    vertex_layout,
                    instance_vertex_layout
                ],
            },
            fragment: Some(FragmentState {
                shader: CONTACTS_SHADER.typed(),
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
                topology: PrimitiveTopology::TriangleList,
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
            label: Some("contacts_pipeline".into()),
        };

        let mut pipeline_cache = world.resource_mut::<PipelineCache>();
        let pipeline_id = pipeline_cache.queue_render_pipeline(pipeline_descriptor);

        Self {
            pipeline_id,
            view_layout,
        }
    }
}

struct ContactsMeta {
    instances: Vec<GpuContact>,
    instance_buffer: Buffer,
    vertex_buffer: Buffer,
    index_buffer: Buffer,
    index_count: u32,
    view_bind_group: Option<BindGroup>,
}

impl FromWorld for ContactsMeta {
    fn from_world(world: &mut World) -> Self {
        // Setup our mesh for contacts

        let mesh = &world.resource::<ContactMesh>().mesh;

        let indices = mesh.indices().unwrap();
        
        println!("indices: {:?}", indices.iter().count());
        

        let render_device = world.resource::<RenderDevice>();

        let vertex_buffer = render_device.create_buffer_with_data(&BufferInitDescriptor {
            usage: BufferUsages::VERTEX,
            label: Some("contact_vertex_buffer"),
            contents: &mesh.get_vertex_buffer_data(),
        });

        let index_buffer = render_device.create_buffer_with_data(&BufferInitDescriptor {
            usage: BufferUsages::INDEX,
            label: Some("contact_index_buffer"),
            contents: mesh.get_index_buffer_bytes().unwrap(),
        });

        let instance_buffer = render_device.create_buffer_with_data(
            &BufferInitDescriptor {
                label: Some("contact_instance_buffer"),
                contents: &[],
                usage: BufferUsages::VERTEX,
            }
        );
        let index_count = mesh.indices().unwrap().iter().count() as u32;
        
        Self {
            instances: Vec::new(),
            instance_buffer,
            vertex_buffer,
            index_buffer,
            index_count,
            view_bind_group: None,
        }
    }
}

fn extract_contacts(
    current_state: Extract<Res<CurrentState<PhysicsDebugState>>>,
    contact_manifold: Extract<Res<PenetrationArena>>,
    mut contacts: ResMut<Contacts>,
) {

    contacts.data.clear();
    if current_state.0 == PhysicsDebugState::Running {
        for (_pair, manifold) in &contact_manifold.manifolds {
            for contact in &manifold.contacts {
                contacts.data.push(GpuContact {
                    position: contact.world_point_a,
                });
                contacts.data.push(GpuContact {
                    position: contact.world_point_b,
                });
            }
        }
    }
}

fn prepare_contacts(
    contacts: Res<Contacts>,
    render_device: Res<RenderDevice>,
    mut contacts_meta: ResMut<ContactsMeta>,
) {
    
    contacts_meta.instances.clear();
    for contact in contacts.data.iter() {
        contacts_meta.instances.push(GpuContact {
            position: contact.position,
        });
    }


    contacts_meta.instance_buffer = render_device.create_buffer_with_data(
        &BufferInitDescriptor {
            label: Some("contact_instance_buffer"),
            contents: cast_slice(&contacts_meta.instances),
            usage: BufferUsages::VERTEX,
        }
    );
}

fn queue_contacts(
    draw_functions: Res<DrawFunctions<Transparent3d>>,
    contacts_pipeline: Res<ContactsPipeline>,
    render_device: Res<RenderDevice>,
    mut aabb_meta: ResMut<ContactsMeta>,
    view_uniforms: Res<ViewUniforms>,
    mut views: Query<&mut RenderPhase<Transparent3d>>,
) {

        let draw_aabb_function = draw_functions.read().get_id::<DrawContacts>().unwrap();

        if let Some(view_binding) = view_uniforms.uniforms.binding() {
            aabb_meta.view_bind_group =
                Some(render_device.create_bind_group(&BindGroupDescriptor {
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

type DrawContacts = (
    SetContactsPipeline,
    SetContactViewBindGroup<0>,
    DrawContactInstances,
);


struct SetContactsPipeline;
impl<P: PhaseItem> RenderCommand<P> for SetContactsPipeline {
    type Param = (SRes<PipelineCache>, SRes<ContactsPipeline>);
    #[inline]
    fn render<'w>(
        _view: Entity,
        _item: &P,
        params: SystemParamItem<'w, '_, Self::Param>,
        pass: &mut TrackedRenderPass<'w>,
    ) -> RenderCommandResult {
        let (pipeline_cache, contacts_pipeline) = params;
        if let Some(pipeline) = pipeline_cache
            .into_inner()
            .get_render_pipeline(contacts_pipeline.pipeline_id)
        {
            pass.set_render_pipeline(pipeline);
            RenderCommandResult::Success
        } else {
            RenderCommandResult::Failure
        }
    }
}

struct SetContactViewBindGroup<const I: usize>;
impl<const I: usize> EntityRenderCommand for SetContactViewBindGroup<I> {
    type Param = (SRes<ContactsMeta>, SQuery<Read<ViewUniformOffset>>);

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


struct DrawContactInstances;
impl EntityRenderCommand for DrawContactInstances {
    type Param = SRes<ContactsMeta>;

    #[inline]
    fn render<'w>(
        _view: Entity,
        _item: Entity,
        contacts_meta: SystemParamItem<'w, '_, Self::Param>,
        pass: &mut TrackedRenderPass<'w>,
    ) -> RenderCommandResult {
        let contacts_meta = contacts_meta.into_inner();

        //info!("indexes {}", contacts_meta.index_count);
        pass.set_vertex_buffer(0, contacts_meta.vertex_buffer.slice(..));
        pass.set_vertex_buffer(1, contacts_meta.instance_buffer.slice(..));
        pass.set_index_buffer(contacts_meta.index_buffer.slice(..), 0,
            IndexFormat::Uint32,
        );
        pass.draw_indexed(0..contacts_meta.index_count, 0, 0..contacts_meta.instances.len() as u32);
        //info!("draw {}", contacts_meta.instances.len());
        RenderCommandResult::Success
    }
}
