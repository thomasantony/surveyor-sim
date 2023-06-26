use bevy_ecs::prelude::*;

use crate::{config::RcsSubsystemConfig, models::rcs::RcsThruster};
use crate::subsystems::DynamicsOutput;

#[derive(Debug, Component)]
pub struct RcsSubsystem {
    pub thrusters: Vec<RcsThruster>,
}
impl RcsSubsystem {
    pub fn from_config(config: &RcsSubsystemConfig) -> Self {
        let mut thrusters = Vec::new();
        for thruster_config in &config.thrusters {
            thrusters.push(RcsThruster::from_config(thruster_config));
        }
        Self { thrusters }
    }
    pub fn spawn_entity(config: &RcsSubsystemConfig, commands: &mut Commands) -> Entity {
        let entity = commands.spawn((Self::from_config(config), DynamicsOutput::default())).id();
        entity
    }
    // TODO: Add time resource here instead of passing in 0.0
    pub fn discrete_update_system(mut query: Query<&mut Self>)
    {
        // Nothing to do here for now
    }
}
impl RcsSubsystem {
    pub fn handle_commands(&mut self, _commands: &super::EngineCommands) {}
    pub fn update_dynamics(&mut self, _outputs: &mut super::OrbitalDynamicsInputs) {
        // Iterate over thrusters and call their update_dynamics method
        for thruster in &mut self.thrusters {
            thruster.update_dynamics(_outputs);
        }
    }
    pub fn update_discrete(&mut self, _dt: f64) {}
    pub fn update_continuous(&mut self, _dt: f64) {}
}
