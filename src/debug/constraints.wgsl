struct View {
    view_proj: mat4x4<f32>,
    view: mat4x4<f32>,
    inverse_view: mat4x4<f32>,
    projection: mat4x4<f32>,
    world_position: vec3<f32>,
    near: f32,
    far: f32,
    width: f32,
    height: f32,
};

@group(0) @binding(0)
var<uniform> view: View;

struct Vertex {
    @builtin(vertex_index) vertex_index: u32,
    @location(0) position: vec3<f32>,
};

struct AnchorLine {
    @location(3) start: vec3<f32>,
    @location(4) end: vec3<f32>,
};

struct VertexOutput {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) world_position: vec4<f32>,
    @location(1) world_normal: vec3<f32>,
    @location(2) uvw: vec3<f32>,
    @location(3) color: vec4<f32>,
};

@vertex
fn vertex(
    vertex: Vertex,
    anchor: AnchorLine,
) -> VertexOutput {
    var out: VertexOutput;

    //out.world_position = vec4<f32>(vertex.position, 1.0);
    switch vertex.vertex_index {
        case 0u: {
            //out.world_position = vec4<f32>(0.0, 0.0 , 0.0, 1.0);
            out.world_position = vec4<f32>(anchor.start, 1.0);
        }
        case 1u: {
            //out.world_position = vec4<f32>(0.0, 5.0 , 0.0, 1.0);
            out.world_position = vec4<f32>(anchor.end, 1.0);
        }
        default {
           
        }
    }
    out.clip_position = view.view_proj * out.world_position;
    out.color = vec4<f32>(1.0, 1.0, 1.0, 1.0);
    return out;
}

struct FragmentInput {
    @builtin(front_facing) is_front: bool,
    @location(0) world_position: vec4<f32>,
    @location(1) world_normal: vec3<f32>,
    @location(2) uvw: vec3<f32>,
    @location(3) color: vec4<f32>,
}

@fragment
fn fragment(in: FragmentInput) -> @location(0) vec4<f32> {
    return in.color;
}

