//! Star Tracker Simulation

use surveyor_types::{config::{StarTrackerConfig, StarTrackerSubsystemConfig}, math::UnitQuaternion};

use crate::spacecraft::SpacecraftDiscreteState;

#[derive(Debug)]
pub (crate) struct StarTrackerSensor {
    pub config: StarTrackerConfig,
    pub q_i2cf: UnitQuaternion,
}

impl StarTrackerSensor {
    pub fn from_config(config: &StarTrackerConfig) -> Self {
        Self {
            config: config.clone(),
            q_i2cf: UnitQuaternion(nalgebra::UnitQuaternion::<f64>::identity()),
        }
    }
}

#[derive(Debug)]
pub (crate) struct StarTrackerSubsystem {
    pub star_trackers: Vec<StarTrackerSensor>,
}
pub (crate) struct StarTrackerOutput {
    pub q_i2cf: UnitQuaternion,
}


impl StarTrackerSubsystem {
    pub fn from_config(config: &StarTrackerSubsystemConfig) -> Self {
        let mut star_trackers = Vec::new();
        for star_tracker_config in &config.sensors {
            star_trackers.push(StarTrackerSensor::from_config(star_tracker_config));
        }
        Self { star_trackers }
    }
    pub fn update_dynamics(&self, _outputs: &mut super::OrbitalDynamicsInputs) {

    }
    pub fn update_discrete(&mut self, _dt: f64, discrete_state: &SpacecraftDiscreteState) {
        // Store omega and accel from discrete state
        for st in &mut self.star_trackers.iter_mut() {
            let q_i2b = discrete_state.q_i2b();
            st.q_i2cf = UnitQuaternion(q_i2b.0 * st.config.geometry.q_cf2b.inverse());
        }
    }
    pub fn update_continuous(&mut self, _dt: f64) {}
}


impl StarTrackerSensor {
    pub fn get_model_output(&self) -> StarTrackerOutput {
        StarTrackerOutput {
            q_i2cf: self.q_i2cf.clone(),
        }
    }
}
