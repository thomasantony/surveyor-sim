use crate::{config::ThrusterConfig, spacecraft::OrbitalDynamicsInputs};

#[derive(Debug)]
pub struct RcsThruster {
    pub config: ThrusterConfig,
    pub thrust: f64,
}

impl RcsThruster {
    pub fn from_config(config: &ThrusterConfig) -> Self {
        Self {
            config: config.clone(),
            thrust: 0.0,
        }
    }
    pub fn update_dynamics(&mut self, outputs: &mut OrbitalDynamicsInputs) {
        // Assume thrust is in Z direction in the component frame
        let thrust_cf = nalgebra::Vector3::<f64>::new(0.0, 0.0, self.thrust);
        // Rotate the thrust vector into the spacecraft body frame
        let thrust_b = self.config.geometry.q_cf2b.transform_vector(&thrust_cf);
        // Compute the torque vector
        let torque_b = self.config.geometry.cf_offset_com_b.cross(&thrust_b);
        // Add the thrust and torque to the spacecraft dynamics
        outputs.total_force_b += thrust_b;
        outputs.total_torque_b += torque_b;
    }
    pub fn handle_commands(&mut self, duty_cycle: f64) {
        // Compute the thrust based on the duty cycle
        self.thrust = self.config.max_thrust * duty_cycle;
    }
}
