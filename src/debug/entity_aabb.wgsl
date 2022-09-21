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

struct Aabb {
    mins: vec3<f32>,
    maxs: vec3<f32>,
};

struct Color {
    value: vec4<f32>
}

@group(0) @binding(0)
var<uniform> view: View;

@group(1) @binding(0)
var<uniform> aabb: Aabb;

// @group(1) @binding(1)
// var<uniform> color: Color;

struct Vertex {
    @builtin(vertex_index) vertex_index: u32,
    //@location(0) position: vec3<f32>,
};

struct VertexOutput {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) world_position: vec4<f32>,
    @location(1) world_normal: vec3<f32>,
    @location(2) uvw: vec3<f32>,
    @location(3) color: vec4<f32>,
};

@vertex
fn vertex(vertex: Vertex) -> VertexOutput {
    var out: VertexOutput;

   switch vertex.vertex_index {
        case 0u { out.world_position = vec4<f32>(aabb.maxs.x, aabb.maxs.y, aabb.mins.z, 1.0); }
        case 1u { out.world_position = vec4<f32>(aabb.mins.x, aabb.maxs.y, aabb.mins.z, 1.0); }
        case 2u { out.world_position = vec4<f32>(aabb.mins.x, aabb.maxs.y, aabb.maxs.z, 1.0); }
        case 3u { out.world_position = vec4<f32>(aabb.maxs.x, aabb.maxs.y, aabb.maxs.z, 1.0); }
        case 4u { out.world_position = vec4<f32>(aabb.maxs.x, aabb.mins.y, aabb.mins.z, 1.0); }
        case 5u { out.world_position = vec4<f32>(aabb.mins.x, aabb.mins.y, aabb.mins.z, 1.0); }
        case 6u { out.world_position = vec4<f32>(aabb.mins.x, aabb.mins.y, aabb.maxs.z, 1.0); }
        case 7u { out.world_position = vec4<f32>(aabb.maxs.x, aabb.mins.y, aabb.maxs.z, 1.0); }
        default: { out.world_position = vec4<f32>(0.0, 0.0, 0.0, 1.0); }
    }

    out.world_normal = vec3<f32>(0.0, 0.0, 1.0);
    out.clip_position = view.view_proj * out.world_position;
    out.color = vec4<f32>(1.0, 0.0, 0.0, 0.1);
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

