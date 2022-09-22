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
                broad_contacts.send(BroadContact { a: *a, b: *b });
            } else {
                broad_contacts.send(BroadContact { a: *b, b: *a });
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

// TODO: The we should be able to traverse the bvh tree, 'tlas' in our case, for possible collision detection
// This was as the one of the main reason i added bvh
// I am missing something here, and this is broken

#[allow(dead_code)]
pub fn broad_phase_bvh(
    tlas: Res<Tlas>,
    mut broad_contacts: EventWriter<BroadContact>,
    mut stack: Local<Vec<(usize, usize)>>,
    mut completed: Local<Vec<(usize, usize)>>,
    mut static_query: Query<Entity, With<Static>>,
) {
    let tlas = tlas.into_inner();

    stack.clear();
    completed.clear();

    let root = &tlas.nodes[0];
    stack.push((root.left(), root.right()));

    let mut count = 0;
    let mut skipped = 0;
    let mut static_count = 0;

    while let Some((a, b)) = stack.pop() {
        if completed.contains(&(a, b)) {
            skipped += 1;
            continue;
        }

        let node_a = &tlas.nodes[a];
        let node_b = &tlas.nodes[b];

        if node_a.aabb.intersection(&node_b.aabb) {
            let leaf_a = node_a.is_leaf();
            let leaf_b = node_b.is_leaf();

            if leaf_a && leaf_b {
                // possible collision
                let entity_a = tlas.blas[node_a.blas as usize].entity;
                let entity_b = tlas.blas[node_b.blas as usize].entity;

                // if both are not static send broad collision
                if let Err(_) = static_query.get_many_mut([entity_a, entity_b]) {
                    broad_contacts.send(BroadContact {
                        a: tlas.blas[node_a.blas as usize].entity,
                        b: tlas.blas[node_b.blas as usize].entity,
                    });
                } else {
                    static_count += 1;
                }
            } else {
                // ‘Descend A’ descent rule
                if !leaf_a {
                    stack.push((node_a.right(), b));
                    stack.push((node_a.left(), b));
                    
                    //children
                    if completed.contains(&(node_a.left(), node_a.right())) {
                        info!("should not happen");
                    }
                    stack.push((node_a.left(), node_a.right()));

                } else {
                    stack.push((a, node_b.right()));
                    stack.push((a, node_b.left()));

                    //children
                    //stack.push((node_b.left(), node_b.right()));
                }
            }
        }
        completed.push((a, b));
        count += 1;
    }
    info!(
        "count: {}, skipped {}, static {}",
        count, skipped, static_count
    );
}
