use surveyor_types::config::StarSensorSubsystemConfig;
use crate::{models::star_sensor::StarSensor, spacecraft::SpacecraftDiscreteState};


#[derive(Debug)]
pub (crate) struct StarSensorSubsystem {
    pub star_sensors: Vec<StarSensor>,
}

impl StarSensorSubsystem {
    pub fn from_config(config: &StarSensorSubsystemConfig) -> Self {
        let mut star_sensors = Vec::new();
        for star_sensor_config in &config.sensors {
            star_sensors.push(StarSensor::from_config(star_sensor_config));
        }
        Self { star_sensors }
    }
    pub fn update_dynamics(&self, _outputs: &mut super::OrbitalDynamicsInputs) {
    }
    pub fn update_discrete(&mut self, t: f64, discrete_state: &SpacecraftDiscreteState) {
        for st in &mut self.star_sensors.iter_mut() {
            st.update_discrete(t, discrete_state);
        }
    }
    pub fn update_continuous(&mut self, _dt: f64) {}
}
