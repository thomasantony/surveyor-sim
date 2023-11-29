//! The IMU model on the simulation side

use bevy_ecs::component::Component;
use nalgebra::Vector3;

use crate::spacecraft::SpacecraftDiscreteState;
use crate::config::{ImuSubsystemConfig, ImuConfig};

#[derive(Debug, Default)]
pub struct IMUSensor{
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
}

#[derive(Debug, Clone)]
pub struct IMUSensorOutput {
    pub omega_cf: Vector3<f64>,
    pub accel_cf: Vector3<f64>,
}

#[derive(Debug)]
pub struct IMUSubsystem {
    pub imus: Vec<IMUSensor>,
}

impl IMUSubsystem {
    pub fn from_config(config: &ImuSubsystemConfig) -> Self {
        let mut imus = Vec::new();
        for imu_config in &config.sensors {
            // imus.push(IMUSensor::from_config(imu_config));
            imus.push(IMUSensor::from_config(imu_config));
        }
        Self { imus }
    }
}

impl IMUSubsystem {
    pub fn update_dynamics(&mut self, _outputs: &mut super::OrbitalDynamicsInputs) {
    }
    pub fn update_discrete(&mut self, _dt: f64, discrete_state: &SpacecraftDiscreteState) {
        // Store omega and accel from discrete state
        for imu in &mut self.imus.iter_mut() {
            let omega_b = Vector3::from_column_slice(discrete_state.omega_b);
            imu.omega_cf = imu.q_cf2b.inverse_transform_vector(&omega_b);;
            // imu.accel_cf = ...
        }
    }
    pub fn update_continuous(&mut self, _dt: f64) {}
}
