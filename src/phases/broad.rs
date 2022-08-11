use bevy::prelude::*;

use crate::{
    bvh::Tlas,
    types::{AabbWorld, BroadContact, RigidBodyMode, Static},
};

// Sweep and Prune
// The board phase is responsible for pruning the search space of possable collisions
// I have tried different approaches, and I am sure I will try a few more
// So far this simple approach has been the faster
#[allow(dead_code)]
pub fn broadphase_system(
    mut broad_contacts: EventWriter<BroadContact>,
    query: Query<(Entity, &AabbWorld, Option<&Static>), With<RigidBodyMode>>,
) {
    // TODO: Yes, we are copying the array out here, only way to sort it
    // Ideally we would keep the array around, it should already near sorted
    let mut list = query.iter().collect::<Vec<_>>();

    // Sort the array on currently selected sorting axis
    // Note: Update inter loop if you change the axis
    list.sort_unstable_by(cmp_x_axis);

    //let t1 = Instant::now();
    // Sweep the array for collisions
    for (i, (a, aabb_a, static_a)) in list.iter().enumerate() {
        // Test collisions against all possible overlapping AABBs following current one
        for (b, aabb_b, static_b) in list.iter().skip(i + 1) {

            // Stop when tested AABBs are beyond the end of current AABB
            if aabb_b.mins.x > aabb_a.maxs.x {
                break;
            }

            // stop if both are static
            if static_a.is_some() && static_b.is_some() {
                continue;
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

            if a.id() < b.id() {
                broad_contacts.send(BroadContact {
                    a: *a,
                    b: *b,
                });
            } else {
                broad_contacts.send(BroadContact {
                    a: *b,
                    b: *a,
                });
            }
            
        }
    }
}

#[allow(dead_code)]
fn cmp_x_axis(a: &(Entity, &AabbWorld, Option<&Static>), b: &(Entity, &AabbWorld, Option<&Static>)) -> std::cmp::Ordering {
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
fn cmp_y_axis(a: &(Entity, &AabbWorld, Option<&Static>), b: &(Entity, &AabbWorld, Option<&Static>)) -> std::cmp::Ordering {
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
fn cmp_z_axis(a: &(Entity, AabbWorld, Option<&Static>), b: &(Entity, AabbWorld, Option<&Static>)) -> std::cmp::Ordering {
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


// TODO: The we should be able to traverse the bvh tree, 'tlas' in our case, for collision detection
// This was as the one of the main reason i added bvh
// I am missing something here, and this is broken
#[allow(dead_code)]
pub fn broadphase_system_bvh(tlas: Res<Tlas>, mut broad_contacts: EventWriter<BroadContact>) {
    let mut a = tlas.tlas_nodes[0].left();
    let mut b = tlas.tlas_nodes[0].right();
    let mut stack = Vec::new();
    loop {
        info!("test a: {}, b: {}", a, b);
        if tlas.tlas_nodes[a].aabb.intersection(&tlas.tlas_nodes[b].aabb) {
            if tlas.tlas_nodes[a].is_leaf() && tlas.tlas_nodes[b].is_leaf() {
                // At leaf nodes. Perform collision tests on leaf node contents
                broad_contacts.send(BroadContact {
                    a: tlas.blas[tlas.tlas_nodes[a].blas as usize].entity,
                    b: tlas.blas[tlas.tlas_nodes[b].blas as usize].entity,
                });
                info!("broad contact a: {}, b: {}", a, b);
                // Could have an exit rule here (eg. exit on first hit)

            } else {
                if !tlas.tlas_nodes[a].is_leaf() { // ‘Descend A’ descent rule
                    stack.push((tlas.tlas_nodes[a].right(), b));
                    a = tlas.tlas_nodes[a].left();
                    continue;
                } else {
                    stack.push((a, tlas.tlas_nodes[b].right()));
                    b = tlas.tlas_nodes[b].left();
                    continue;
                }
            }
        }
        if let Some((new_a, new_b)) = stack.pop() {
            a = new_a;
            b = new_b;
        } else {
            break;
        }
    }
}
