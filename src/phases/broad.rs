use bevy::prelude::*;

use crate::{bvh::Tlas, types::*};




// Sweep and Prune
// The board phase is responsible for pruning the search space of possible collisions
// I have tried different approaches, and I am sure I will try a few more
// So far this simple approach has been the fastest
#[allow(dead_code)]
pub fn broad_phase(
    mut broad_contacts: EventWriter<BroadContact>,
    query: Query<(Entity, &Aabb, Option<&Static>), With<RigidBody>>,
) {
    // TODO: Yes, we are copying the array out here, only way to sort it
    // Ideally we would keep the array around, it should already near sorted
    let mut list = query.iter().collect::<Vec<_>>();

    // Sort the array on currently selected sorting axis
    // Note: Update inter loop if you change the axis
    list.sort_unstable_by(cmp_y_axis);

    //let t1 = Instant::now();
    // Sweep the array for collisions
    for (i, (a, aabb_a, static_a)) in list.iter().enumerate() {
        // Test collisions against all possible overlapping AABBs following current one
        for (b, aabb_b, static_b) in list.iter().skip(i + 1) {
            // Stop when tested AABBs are beyond the end of current AABB
            if aabb_b.mins.y > aabb_a.maxs.y {
                break;
            }

            // SAT test
            if aabb_a.mins.x >= aabb_b.maxs.x {
                continue;
            }
            if aabb_a.maxs.x <= aabb_b.mins.x {
                continue;
            }

            if aabb_a.mins.y >= aabb_b.maxs.y {
                continue;
            }
            if aabb_a.maxs.y <= aabb_b.mins.y {
                continue;
            }

            if aabb_a.mins.z >= aabb_b.maxs.z {
                continue;
            }
            if aabb_a.maxs.z <= aabb_b.mins.z {
                continue;
            }

            // stop if both are static
            if static_a.is_none() || static_b.is_none() {
                broad_contacts.send(BroadContact { a: *a, b: *b });
            }

        }
    }
}

#[allow(dead_code)]
fn cmp_x_axis(
    a: &(Entity, &Aabb, Option<&Static>),
    b: &(Entity, &Aabb, Option<&Static>),
) -> std::cmp::Ordering {
    // Sort on minimum value along either x, y, or z axis
    let min_a = a.1.mins.x;
    let min_b = b.1.mins.x;
    if min_a < min_b {
        return std::cmp::Ordering::Less;
    }
    if min_a > min_b {
        return std::cmp::Ordering::Greater;
    }
    std::cmp::Ordering::Equal
}

#[allow(dead_code)]
fn cmp_y_axis(
    a: &(Entity, &Aabb, Option<&Static>),
    b: &(Entity, &Aabb, Option<&Static>),
) -> std::cmp::Ordering {
    // Sort on minimum value along either x, y, or z axis
    let min_a = a.1.mins.y;
    let min_b = b.1.mins.y;
    if min_a < min_b {
        return std::cmp::Ordering::Less;
    }
    if min_a > min_b {
        return std::cmp::Ordering::Greater;
    }
    std::cmp::Ordering::Equal
}

#[allow(dead_code)]
fn cmp_z_axis(
    a: &(Entity, Aabb, Option<&Static>),
    b: &(Entity, Aabb, Option<&Static>),
) -> std::cmp::Ordering {
    // Sort on minimum value along either x, y, or z axis
    let min_a = a.1.mins.z;
    let min_b = b.1.mins.z;
    if min_a < min_b {
        return std::cmp::Ordering::Less;
    }
    if min_a > min_b {
        return std::cmp::Ordering::Greater;
    }
    std::cmp::Ordering::Equal
}

// This works but is slower, tried quite a few different approaches
// to beat sweep and prune, compute task to break it up, but this is 
// the fastest form so far and but still slower
#[allow(dead_code)]
pub fn broad_phase_bvh(
    tlas: Res<Tlas>,
    mut broad_contacts: EventWriter<BroadContact>,
    mut stack: Local<Vec<(u16, u16)>>,
    mut static_query: Query<Entity, With<Static>>,
) {    
    for node in tlas.nodes.iter().filter(|n| !n.is_leaf()) {
        stack.push((node.left, node.right));        
    }

    while let Some((a, b)) = stack.pop() {

        let node_a = &tlas.nodes[a as usize];
        let node_b = &tlas.nodes[b as usize];
        if node_a.aabb.intersection(&node_b.aabb) {
            if node_a.is_leaf() && node_b.is_leaf() {
                // possible collision
                let entity_a = tlas.blas[node_a.blas as usize].entity.unwrap();
                let entity_b = tlas.blas[node_b.blas as usize].entity.unwrap();

                // if both are not static send broad collision
                if static_query.get_many_mut([entity_a, entity_b]).is_err() {
                    //info!("Broad contact: {} {}", a, b);
                    broad_contacts.send(BroadContact {
                        a: entity_a,
                        b: entity_b,
                    });
                }
            } else {
                // ‘Descend A’ descent rule
                if !node_a.is_leaf() {
                    stack.push((node_a.right, b));
                    stack.push((node_a.left, b));

                } else {
                    stack.push((a, node_b.right));
                    stack.push((a, node_b.left));
                }
            }
        }
    }
}