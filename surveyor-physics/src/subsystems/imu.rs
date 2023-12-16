use crate::spacecraft::SpacecraftDiscreteState;
use crate::models::imu::IMUSensor;
use surveyor_types::config::ImuSubsystemConfig;

#[derive(Debug)]
pub (crate) struct IMUSubsystem {
    pub imus: Vec<IMUSensor>,
}

impl IMUSubsystem {
    pub fn from_config(config: &ImuSubsystemConfig) -> Self {
        let mut imus = Vec::new();
        for imu_config in &config.sensors {
            imus.push(IMUSensor::from_config(imu_config));
        }
        Self { imus }
    }
}

impl IMUSubsystem {
    pub fn update_dynamics(&mut self, _outputs: &mut super::OrbitalDynamicsInputs) {
    }
    pub fn update_discrete(&mut self, t: f64, discrete_state: &SpacecraftDiscreteState) {
        // Store omega and accel from discrete state
        for imu in &mut self.imus.iter_mut() {
            imu.update_discrete(t, discrete_state)
        }
    }
    pub fn update_continuous(&mut self, _dt: f64) {}
}
