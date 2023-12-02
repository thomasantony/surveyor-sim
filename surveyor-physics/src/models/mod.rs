pub mod tvc;
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
