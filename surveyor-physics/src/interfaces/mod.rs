use crate::spacecraft::SpacecraftDiscreteState;

/// Defines the interfaces between the truth-side models and the flight-software/GNC components

pub struct AvionicsInterface {}

pub trait SensorAvionicsInterface {
    fn set_input(&mut self, input: &SpacecraftDiscreteState);
}
