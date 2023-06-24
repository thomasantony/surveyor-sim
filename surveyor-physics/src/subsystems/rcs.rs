use crate::{config::RcsSubsystemConfig, models::rcs::RcsThruster};

#[derive(Debug)]
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
