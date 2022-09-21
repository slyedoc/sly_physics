use bevy_render::{prelude::*};
fn main() {

    let mesh = Mesh::from(shape::UVSphere {
        radius: 0.1,
        sectors: 5,
        stacks: 5,
    });
    //let attr = mesh.attributes();
    let verts = mesh.attribute(Mesh::ATTRIBUTE_POSITION).unwrap();
    let indices = mesh.indices().unwrap();
    
    for a in mesh.attributes() {
        println!("{:?}", a);
    }
    // for i in indices.iter() {
    //     println!("i: {:?}", i);
    // }

    // println!("mesh: {:?}", mesh);
}