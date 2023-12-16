//! The IMU model on the simulation side
use surveyor_types::config::ImuConfig;
use nalgebra::Vector3;

use crate::spacecraft::SpacecraftDiscreteState;

#[derive(Debug, Default)]
pub (crate) struct IMUSensor{
    q_cf2b: nalgebra::UnitQuaternion<f64>,
    pub omega_cf: Vector3<f64>,
    pub accel_cf: Vector3<f64>,
}

impl IMUSensor {
    pub fn from_config(config: &ImuConfig) -> Self {
        Self{
            q_cf2b: *config.geometry.q_cf2b,
            omega_cf: Vector3::zeros(),
            accel_cf: Vector3::zeros(),
        }
    }
    pub fn get_model_output(&self) -> IMUSensorOutput {
        IMUSensorOutput {
            omega_cf: self.omega_cf,
            accel_cf: self.accel_cf,
        }
    }

    pub fn update_discrete(&mut self, _dt: f64, discrete_state: &SpacecraftDiscreteState) {
        // Store omega and accel from discrete state
        let omega_b = discrete_state.omega_b();
        self.omega_cf = self.q_cf2b.inverse_transform_vector(&omega_b);
        // self.accel_cf = ...
    }
}

#[derive(Debug, Clone)]
pub (crate) struct IMUSensorOutput {
    pub omega_cf: Vector3<f64>,
    pub accel_cf: Vector3<f64>,
}
