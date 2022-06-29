use bevy::prelude::*;

use crate::AabbWorld;




#[derive(Debug)]
pub struct BroadContact {
    pub a: Entity,
    pub b: Entity,
}


// The board phase is responsible for pruning the search space of possable collisions
// I have tried different approaches, and I am sure I will try a few more
// So far this simple approach has been the fastest
// TODO: Figure out way to search two axis thats actually faster or bite the bullet and try some space partitioning
pub fn broadphase_system(
    mut broad_contacts: EventWriter<BroadContact>,
    query: Query<(Entity, &AabbWorld)>,
) {
    // TODO: Yes, we are copying the array out here, only way to sort it
    // Ideally we would keep the array around, it should already near sorted
    let mut list = query.iter().collect::<Vec<_>>();


    // Sort the array on currently selected sorting axis
    // Note: Update inter loop if you change the axis
    list.sort_unstable_by(cmp_x_axis);

    //let t1 = Instant::now();
    // Sweep the array for collisions
    for (i, (a, aabb_a)) in list.iter().enumerate() {
        // Test collisions against all possible overlapping AABBs following current one
        for (b, aabb_b) in list.iter().skip(i + 1) {
            // Stop when tested AABBs are beyond the end of current AABB
            if aabb_b.minimums.x > aabb_a.maximums.x {
                break;
            }

            // SAT test
            if aabb_a.minimums.x >= aabb_b.maximums.x {
                continue;
            }
            if aabb_a.maximums.x <= aabb_b.minimums.x {
                continue;
            }

            if aabb_a.minimums.y >= aabb_b.maximums.y {
                continue;
            }
            if aabb_a.maximums.y <= aabb_b.minimums.y {
                continue;
            }

            if aabb_a.minimums.z >= aabb_b.maximums.z {
                continue;
            }
            if aabb_a.maximums.z <= aabb_b.minimums.z {
                continue;
            }

            // Overlap on all three axes, so their intersection must be non-empty
            broad_contacts.send(BroadContact { a: *a, b: *b });
        }
    }
}


#[allow(dead_code)]
fn cmp_x_axis(a: &(Entity, &AabbWorld), b: &(Entity, &AabbWorld)) -> std::cmp::Ordering {
    // Sort on minimum value along either x, y, or z axis
    let min_a = a.1.minimums.x;
    let min_b = b.1.minimums.x;
    if min_a < min_b {
        return std::cmp::Ordering::Less;
    }
    if min_a > min_b {
        return std::cmp::Ordering::Greater;
    }
    std::cmp::Ordering::Equal
}

#[allow(dead_code)]
fn cmp_y_axis(a: &(Entity, &AabbWorld), b: &(Entity, &AabbWorld)) -> std::cmp::Ordering {
    // Sort on minimum value along either x, y, or z axis
    let min_a = a.1.minimums.y;
    let min_b = b.1.minimums.y;
    if min_a < min_b {
        return std::cmp::Ordering::Less;
    }
    if min_a > min_b {
        return std::cmp::Ordering::Greater;
    }
    std::cmp::Ordering::Equal
}

#[allow(dead_code)]
fn cmp_z_axis(a: &(Entity, AabbWorld), b: &(Entity, AabbWorld)) -> std::cmp::Ordering {
    // Sort on minimum value along either x, y, or z axis
    let min_a = a.1.minimums.z;
    let min_b = b.1.minimums.z;
    if min_a < min_b {
        return std::cmp::Ordering::Less;
    }
    if min_a > min_b {
        return std::cmp::Ordering::Greater;
    }
    std::cmp::Ordering::Equal
}
