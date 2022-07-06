use bevy::{core::FixedTimestep, ecs::schedule::ShouldRun, prelude::*};
use bevy_inspector_egui::Inspectable;

use crate::{update_world_info, PhysicsState, PHYSISCS_TIMESTEP, LinearVelocity, Mass, InverseMass, Static, PhysicsConfig};

pub struct GravityPlugin;

impl Plugin for GravityPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<Gravity>().add_system_set_to_stage(
            CoreStage::PostUpdate,
            SystemSet::new()
                // workaround since you cant chain with_run_criteria, see https://github.com/bevyengine/bevy/issues/1839
                .with_run_criteria(FixedTimestep::step(PHYSISCS_TIMESTEP as f64).chain(
                    |In(input): In<ShouldRun>, state: Res<State<PhysicsState>>| {
                        if state.current() == &PhysicsState::Running {
                            input
                        } else {
                            ShouldRun::No
                        }
                    },
                ))
                .with_system(
                    gravity_system.before(update_world_info)
                ),
        );
    }
}

#[derive(Inspectable, Deref, DerefMut, Debug)]
pub struct Gravity(pub Vec3);
impl Default for Gravity {
    fn default() -> Self {
        Gravity(Vec3::new(0.0, -9.8, 0.0))
    }
}

pub fn gravity_system(
    mut query: Query<(&mut LinearVelocity, &Mass, &InverseMass), Without<Static>>,
    gravity: Res<Gravity>,
    config: Res<PhysicsConfig>,
) {
    for (mut linear_velocity, mass, inv_mass) in query.iter_mut() {
        // since rb is not static, inv mass shouldnt be 0 or less
        debug_assert!(inv_mass.0 > 0.0);

        // Apply Gravity, it needs to be an impluse
        let gravey_impluse = gravity.0 * mass.0 * config.time;
        linear_velocity.0 += gravey_impluse * inv_mass.0;
    }
}
