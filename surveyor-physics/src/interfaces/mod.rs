// use crate::spacecraft::SpacecraftDiscreteState;

// /// Defines the interfaces between the truth-side models and the flight-software/GNC components

// pub struct AvionicsInterface {}

// pub trait SensorAvionicsInterface {
//     fn set_input(&mut self, input: &SpacecraftDiscreteState);
// }

use bevy::hierarchy::Children;
use bevy_ecs::prelude::*;
use surveyor_gnc::sensors::IMUInput;

use crate::{subsystems::{rcs::RcsCommands, Subsystem}, spacecraft::SpacecraftModel};

#[derive(Debug, Clone, Event)]
pub enum SensorEvent {
    IMU(surveyor_gnc::sensors::IMUInput),
    StarTracker(surveyor_gnc::sensors::StarTrackerInput),
}

/// Conversion from truth-side data to GNC-side events
pub fn imu_event_generator(
    mut sc_query: Query<(&SpacecraftModel, &Children)>,
    mut q_imu: Query<&Subsystem>,
    mut imu_input_events: EventWriter<surveyor_gnc::sensors::IMUInput>)
{
    let (_, children) = sc_query.single_mut();
    for subsystem_id in children.iter() {
        let subsystem = q_imu.get_mut(*subsystem_id).unwrap();
        match subsystem {
            // Send RCS commands to the RCS subsystem
            Subsystem::Imu(imu_subsystem) => {
                for (idx, sensor) in imu_subsystem.imus.iter().enumerate() {
                    let imu_data = sensor.get_model_output();
                    let imu_input = IMUInput::new(idx, imu_data.omega_cf, imu_data.accel_cf);
                    imu_input_events.send(imu_input);
                }
            }
            _ => {}
        }
    }
}

/// Receive actuator events from the GNC system and send them to the simulation
/// We convert it into a truth-side type before passing it through
#[derive(Debug, Clone, Event)]
pub enum ActuatorEvent {
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

pub fn rcs_event_receiver(
    mut rcs_commands: EventReader<surveyor_gnc::control::RCSControllerOutput>,
    mut q_rcs: Query<&mut Subsystem>,

) {
    // If there are multiple events, only process the last one
    if let Some(event) = rcs_commands.read().last() {
        for  subsystem in q_rcs.iter_mut() {
            match subsystem.into_inner() {
                Subsystem::Rcs(rcs_subsystem) => {
                    rcs_subsystem.handle_commands(&RcsCommands::from(event));
                    break;
                }
                _ => {}
            }
        }
    }
}
