//! The IMU Subsystem

#[derive(Debug)]
pub struct IMUSubsystem {
    pub imus: Vec<IMU>,
}
impl IMUSubsystem {
    pub fn from_config(config: &IMUSubsystemConfig) -> Self {
        let mut imus = Vec::new();
        for imu_config in &config.thrusters {
            imus.push(IMUThruster::from_config(imu_config));
        }
        Self { imus }
    }
}
pub struct IMUCommands {
    pub duty_cycles: Vec<f64>,
}
impl IMUSubsystem {
    pub fn update_dynamics(&mut self, _outputs: &mut super::OrbitalDynamicsInputs) {
    }
    pub fn update_discrete(&mut self, _dt: f64) {}
    pub fn update_continuous(&mut self, _dt: f64) {}
}
