use bevy::prelude::*;
use bevy_inspector_egui::Inspectable;
use iyes_loopless::prelude::*;

use crate::{
    InverseMass, LinearVelocity, Mass, PhysicsConfig, PhysicsFixedUpdate, PhysicsState,
    PhysicsSystems, Static,
};

#[derive(Debug, Clone, PartialEq, Eq, Hash, Copy)]
pub enum GravityState {
    Running,
    Paused,
}

pub struct GravityPlugin;

impl Plugin for GravityPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<Gravity>()
            .add_loopless_state(GravityState::Running)
            .add_system_set_to_stage(
                PhysicsFixedUpdate,
                ConditionSet::new()
                    .run_in_state(PhysicsState::Running)
                    .run_in_state(GravityState::Running)
                    .label(PhysicsSystems::Setup)
                    .with_system(gravity_system)
                    .into(),
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
