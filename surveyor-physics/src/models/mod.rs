use crate::config::TVCConfig;

pub mod rcs;
pub mod surveyor_engines;

// Trait for a model of an actuator (e.g. a servo)
pub trait ActuatorModel<'a> {
    type ContinuousInputs;
    type DiscreteInputs;
    type ContinuousOutputs;
    type DiscreteOutputs;
    type Command;

    // TODO: Make these return Result<(), Error>
    fn handle_commands(&'a mut self, command: &Self::Command);
    fn update_continuous(&'a mut self, dt: f64, inputs: &Self::ContinuousInputs);
    fn update_discrete(&'a mut self, dt: f64, inputs: &Self::DiscreteInputs);
    fn get_discrete_outputs(&'a self) -> &Self::DiscreteOutputs;
    fn get_continuous_outputs(&'a self) -> &Self::ContinuousOutputs;
}

// Model for a TVC servo system with two degrees of freedom
#[derive(Debug)]
pub struct TVC {
    // Configuration parameters
    pub config: TVCConfig,
    // Current angle of deflection of the X servo
    deflection: f64,
    // Outputs
    outputs: TVCContinuousOutputs,
}

impl TVC {
    pub fn from_config(config: &TVCConfig) -> Self {
        Self {
            config: config.clone(),
            deflection: 0.0f64.to_radians(),
            outputs: TVCContinuousOutputs::default(),
        }
    }
}
#[derive(Debug, Default)]
pub struct TVCContinuousOutputs {
    // Rotation from TVC nominal direction to the nozzle direction
    pub q_tvc2nozzle: nalgebra::UnitQuaternion<f64>,
}

impl<'a> ActuatorModel<'a> for TVC {
    type ContinuousInputs = ();
    type DiscreteInputs = ();
    type ContinuousOutputs = TVCContinuousOutputs;
    type DiscreteOutputs = ();
    type Command = f64;

    // This will be triggered by the FSW with the desired deflection angle
    fn handle_commands(&'a mut self, command: &Self::Command) {
        self.deflection = *command;
    }
    fn update_continuous(&'a mut self, _dt: f64, _inputs: &Self::ContinuousInputs) {
        // Compute the rotation from the TVC nominal direction to the nozzle direction
        self.outputs.q_tvc2nozzle =
            nalgebra::UnitQuaternion::from_axis_angle(&self.config.axis_cf.0, self.deflection);
    }
    fn update_discrete(&'a mut self, _dt: f64, _inputs: &Self::DiscreteInputs) {}
    fn get_discrete_outputs(&'a self) -> &Self::DiscreteOutputs {
        &()
    }
    fn get_continuous_outputs(&'a self) -> &Self::ContinuousOutputs {
        &self.outputs
    }
}
