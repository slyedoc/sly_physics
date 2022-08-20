// use bevy::{
//     math::{vec3},
//     prelude::*,
// };
// use sly_physics::prelude::*;
// use bevy::prelude::*;

// #[test]
// fn test_epa() {    

    
//     let box_a = BoxCollider::new(Vec3::ONE);
//     let trans_a = Transform {
//         translation: vec3(8.00000667, 9.4973526, 2.5000124),
//         rotation: Quat::from_xyzw(-1.19166614E-7, -0.00000143082571, 0.00000248814399, 1.0),
//         ..default()
//     };
//     let box_b = BoxCollider::new(Vec3::ONE);
//     let trans_b = Transform {
//         translation: vec3(8.00001144, 8.49734306, 3.50001287),
//         rotation: Quat::from_xyzw(9.16530154E-7, 5.70654834E-7, 0.00000295896189, 1.0),
//         ..default()
//     };

//     let simplex_points = [
//         Point {
//             pt_a: vec3(7.49911594, 8.99734973, 1.99956369),
//             pt_b: vec3(8.50090217, 8.99734687, 3.00046062),
//             xyz: vec3(-1.00178623, 0.00000286102295, -1.00089693),
//         },
//         Point {
//             pt_a: vec3(8.50090217, 8.99735546, 3.0004611),
//             pt_b: vec3(7.49911356, 8.99734116, 2.99956703),
//             xyz: vec3(1.00178862, 0.0000143051147, 0.000894069671),
//         },
//         Point {
//             pt_a: vec3(7.49911308, 8.99735069, 3.00045824),
//             pt_b: vec3(8.50090217, 8.99734687, 2.99956608),
//             xyz: vec3(-1.00178909, 0.00000381469727, 0.000892162322),
//         },
//         Point {
//             pt_a: vec3(8.50090503, 8.9973545, 1.99956667),
//             pt_b: vec3(7.49911356, 8.99734116, 3.00046158),
//             xyz: vec3(1.00179148, 0.0000133514404, -1.0008949),
//         },
//     ];

//     let (a, b) = epa_expand(&box_a, &trans_a, &box_b, &trans_b, 0.00100000005, &simplex_points);
// }