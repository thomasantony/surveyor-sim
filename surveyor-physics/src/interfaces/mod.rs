// use crate::spacecraft::SpacecraftDiscreteState;

// /// Defines the interfaces between the truth-side models and the flight-software/GNC components

// pub struct AvionicsInterface {}

// pub trait SensorAvionicsInterface {
//     fn set_input(&mut self, input: &SpacecraftDiscreteState);
// }

use bevy_ecs::prelude::{EventReader, EventWriter};

use crate::subsystems::rcs::RcsCommands;

pub enum SensorEvents {
    IMU(surveyor_gnc::sensors::IMUInput),
    StarTracker(surveyor_gnc::sensors::StarTrackerInput),
}

/// Send sensor events from the simulation to the GNC system
pub fn send_sensor_events(mut events: EventReader<SensorEvents>,
    mut imu_input: EventWriter<surveyor_gnc::sensors::IMUInput>,
    mut star_tracker_input: EventWriter<surveyor_gnc::sensors::StarTrackerInput>,
) {
    for event in events.iter() {
        match event {
            SensorEvents::IMU(input) => {
                imu_input.send(input.clone());
            }
            SensorEvents::StarTracker(input) => {
                star_tracker_input.send(input.clone());
            }
        }
    }
}

/// Receive actuator events from the GNC system and send them to the simulation
/// We convert it into a truth-side type before passing it through
pub enum ActuatorEvents {
    RCS(RcsCommands),
    // TVC(surveyor_gnc::control::TVCControllerOutput),
}

impl From<&surveyor_gnc::control::RCSControllerOutput> for RcsCommands {
    fn from(output: &surveyor_gnc::control::RCSControllerOutput) -> Self {
        RcsCommands {
            duty_cycles: output.duty_cycles.clone(),
        }
    }
}

pub fn recv_actuator_events(
    mut event_writer: EventWriter<ActuatorEvents>,
    mut rcs_commands: EventReader<surveyor_gnc::control::RCSControllerOutput>,
) {
    // If there are multiple events, only process the last one
    if let Some(event) = rcs_commands.iter().last() {
        event_writer.send(ActuatorEvents::RCS(event.into()));
    }
}
