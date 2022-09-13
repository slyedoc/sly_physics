#import bevy_pbr::mesh_types
#import bevy_pbr::mesh_view_bindings

@group(1) @binding(0)
var<uniform> mesh: Mesh;

// NOTE: Bindings must come before functions that use them!
#import bevy_pbr::mesh_functions

struct Vertex {
    [[builtin(vertex_index)]] vertex_index: u32;
    @location(0) position: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) uv: vec2<f32>,

    @location(3) mins: vec3<f32>,
    @location(4) maxs: vec3<f32>,
    @location(4) color: vec4<f32>,
};

struct VertexOutput {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) color: vec4<f32>,
};

@vertex
fn vertex(vertex: Vertex) -> VertexOutput {
            // [aabb.maxs.x, aabb.maxs.y, aabb.mins.z],
            // [aabb.mins.x, aabb.maxs.y, aabb.mins.z],
            // [aabb.mins.x, aabb.maxs.y, aabb.maxs.z],
            // [aabb.maxs.x, aabb.maxs.y, aabb.maxs.z],
            // [aabb.maxs.x, aabb.mins.y, aabb.mins.z],
            // [aabb.mins.x, aabb.mins.y, aabb.mins.z],
            // [aabb.mins.x, aabb.mins.y, aabb.maxs.z],
            // [aabb.maxs.x, aabb.mins.y, aabb.maxs.z],
    let position = if vertex.vertex_index == 0 {
        vec3<f32>(vertex.maxs.x, vertex.maxs.y, vertex.mins.z)
    } else if vertex.vertex_index == 1 {
        vec3<f32>(vertex.mins.x, vertex.maxs.y, vertex.mins.z)
    } else {
        vertex.position;
    };
    
    var out: VertexOutput;
    out.clip_position = mesh_position_local_to_clip(mesh.model, vec4<f32>(position, 1.0));
    out.color = vertex.color;
    return out;
}

@fragment
fn fragment(in: VertexOutput) -> @location(0) vec4<f32> {
    return in.color;
}
