use std::ops::Sub;

use crate::{
    config::SubsystemConfig,
    integrators::NewDynamicSystem,
    spacecraft::{OrbitalDynamicsInputs, SpacecraftDiscreteState},
};
pub mod propulsion;
pub mod rcs;

use self::propulsion::EngineCommands;

#[derive(Debug)]
pub enum Subsystem {
    Propulsion(propulsion::SurveyorPropulsion),
    Rcs(rcs::RcsSubsystem),
}
impl Subsystem {
    pub fn from_config(config: &SubsystemConfig) -> Self {
        match config {
            SubsystemConfig::Propulsion(engine_subsystem_config) => Subsystem::Propulsion(
                propulsion::SurveyorPropulsion::from_config(engine_subsystem_config),
            ),
            SubsystemConfig::Rcs(rcs_subsystem_config) => {
                Subsystem::Rcs(rcs::RcsSubsystem::from_config(rcs_subsystem_config))
            }
            _ => panic!("Invalid subsystem config"),
        }
    }
    pub fn handle_commands(&mut self, commands: &EngineCommands) {
        match self {
            Subsystem::Propulsion(engine_subsystem) => {
                engine_subsystem.handle_commands(commands);
            }
            Subsystem::Rcs(rcs_subsystem) => {
                // rcs_subsystem.handle_commands(commands);
            }
        }
    }
    /// Represents a collection of models that make up a subsystem
    pub fn update_discrete(&mut self, dt: f64, _discrete_state: &SpacecraftDiscreteState) {
        match self {
            Subsystem::Propulsion(engine_subsystem) => {
                // Create discrete input for propulsion subsystem using SpacecraftDiscreteState
                engine_subsystem.update_discrete(dt);
            }
            Subsystem::Rcs(rcs_subsystem) => {
                rcs_subsystem.update_discrete(dt);
            }
        }
    }
    pub fn update_continuous(&mut self, dt: f64) {
        match self {
            Subsystem::Propulsion(engine_subsystem) => {
                engine_subsystem.update_continuous(dt);
            }
            Subsystem::Rcs(rcs_subsystem) => {
                rcs_subsystem.update_continuous(dt);
            }
        }
    }
    pub fn update_dynamics(&mut self, outputs: &mut OrbitalDynamicsInputs) {
        match self {
            Subsystem::Propulsion(engine_subsystem) => {
                engine_subsystem.update_dynamics(outputs);
            }
            Subsystem::Rcs(rcs_subsystem) => {
                rcs_subsystem.update_dynamics(outputs);
            }
        }
    }
}
impl<'a> NewDynamicSystem<'a> for Subsystem {
    type DerivativeInputs = ();
    fn get_state(&self) -> &[f64] {
        match self {
            Subsystem::Propulsion(engine_subsystem) => engine_subsystem.get_state(),
            _ => &[],
        }
    }

    fn set_state(&mut self, t: f64, state: &[f64]) {
        match self {
            Subsystem::Propulsion(engine_subsystem) => {
                engine_subsystem.set_state(t, state);
            }
            _ => {}
        }
    }

    fn get_num_states(&self) -> usize {
        match self {
            Subsystem::Propulsion(engine_subsystem) => engine_subsystem.get_num_states(),
            _ => 0,
        }
    }

    fn get_t(&self) -> f64 {
        match self {
            Subsystem::Propulsion(engine_subsystem) => engine_subsystem.get_t(),
            _ => 0.0,
        }
    }

    fn get_derivatives(
        &mut self,
        t: f64,
        d_state: &mut [f64],
        _inputs: &'a Self::DerivativeInputs,
    ) {
        match self {
            Subsystem::Propulsion(engine_subsystem) => {
                engine_subsystem.get_derivatives(t, d_state, &mut ());
            }
            _ => {}
        }
    }
}
