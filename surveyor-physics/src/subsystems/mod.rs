use surveyor_types::config::SubsystemConfig;
use enum_as_inner::EnumAsInner;

use crate::{
    integrators::DynamicSystem,
    spacecraft::{OrbitalDynamicsInputs, SpacecraftDiscreteState}, universe::Observation
};
use bevy_ecs::prelude::*;
use bevy_enum_filter::prelude::*;
pub mod propulsion;
pub mod rcs;
pub mod imu;
pub mod star_tracker;
pub mod star_sensor;

#[derive(Debug, EnumAsInner, Component, EnumFilter)]
pub (crate) enum Subsystem {
    Propulsion(propulsion::SurveyorPropulsion),
    Rcs(rcs::RcsSubsystem),
    Imu(imu::IMUSubsystem),
    StarTracker(star_tracker::StarTrackerSubsystem),
    StarSensor(star_sensor::StarSensorSubsystem),
}

impl Subsystem {
    pub fn from_config(config: &SubsystemConfig) -> Self {
        match config {
            SubsystemConfig::Propulsion(engine_subsystem_config) => Subsystem::Propulsion(
                propulsion::SurveyorPropulsion::from_config(engine_subsystem_config),
            ),
            SubsystemConfig::Rcs(rcs_subsystem_config) => {
                Subsystem::Rcs(rcs::RcsSubsystem::from_config(rcs_subsystem_config))
            },
            SubsystemConfig::Imu(imu_subsystem_config) => {
                Subsystem::Imu(imu::IMUSubsystem::from_config(imu_subsystem_config))
            },
            SubsystemConfig::StarTracker(star_tracker_subsystem_config) => {
                Subsystem::StarTracker(star_tracker::StarTrackerSubsystem::from_config(star_tracker_subsystem_config))
            },
            SubsystemConfig::StarSensor(star_sensor_subsystem_config) => {
                Subsystem::StarSensor(star_sensor::StarSensorSubsystem::from_config(star_sensor_subsystem_config))
            },
            // _ => panic!("Invalid subsystem config"),
        }
    }
    // Represents a collection of models that make up a subsystem
    pub fn update_discrete(&mut self, dt: f64, discrete_state: &SpacecraftDiscreteState, observation: &Observation) {
        match self {
            Subsystem::Propulsion(engine_subsystem) => {
                // Create discrete input for propulsion subsystem using SpacecraftDiscreteState
                engine_subsystem.update_discrete(dt, discrete_state);
            }
            Subsystem::Rcs(rcs_subsystem) => {
                rcs_subsystem.update_discrete(dt, discrete_state);
            }
            Subsystem::Imu(imu_subsystem) => {
                imu_subsystem.update_discrete(dt, discrete_state);
            }
            Subsystem::StarTracker(star_tracker_subsystem) => {
                star_tracker_subsystem.update_discrete(dt, discrete_state);
            }
            Subsystem::StarSensor(star_sensor_subsystem) => {
                star_sensor_subsystem.update_discrete(dt, discrete_state);
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
            Subsystem::Imu(_) => {}
            Subsystem::StarTracker(star_tracker_subsystem) => {
                star_tracker_subsystem.update_continuous(dt);
            }
            Subsystem::StarSensor(star_sensor_subsystem) => {
                star_sensor_subsystem.update_continuous(dt);
            }
        }
    }
    pub fn update_dynamics(&self, outputs: &mut OrbitalDynamicsInputs) {
        match self {
            Subsystem::Propulsion(engine_subsystem) => {
                engine_subsystem.update_dynamics(outputs);
            }
            Subsystem::Rcs(rcs_subsystem) => {
                rcs_subsystem.update_dynamics(outputs);
            }
            Subsystem::Imu(_) => {}
            Subsystem::StarTracker(star_tracker_subsystem) => {
                star_tracker_subsystem.update_dynamics(outputs);
            }
            Subsystem::StarSensor(star_sensor_subsystem) => {
                star_sensor_subsystem.update_dynamics(outputs);
            }
        }
    }
}

impl<'a> DynamicSystem<'a> for Subsystem {
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
        &self,
        t: f64,
        state: &[f64],
        d_state: &mut [f64],
        _inputs: &'a Self::DerivativeInputs,
    ) {
        match self {
            Subsystem::Propulsion(engine_subsystem) => {
                engine_subsystem.get_derivatives(t, state, d_state, &mut ());
            }
            _ => {}
        }
    }
}
