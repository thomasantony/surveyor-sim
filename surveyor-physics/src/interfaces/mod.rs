// use crate::spacecraft::SpacecraftDiscreteState;

// /// Defines the interfaces between the truth-side models and the flight-software/GNC components

// pub struct AvionicsInterface {}

// pub trait SensorAvionicsInterface {
//     fn set_input(&mut self, input: &SpacecraftDiscreteState);
// }

use bevy_ecs::prelude::*;
use surveyor_gnc::sensors::{IMUInput, StarTrackerInput};
use surveyor_gnc::clock::TimeTickEvent;
use bevy_enum_filter::prelude::*;

// ! === subsystem_filters is auto-generated by bevy_enum_filters === ! //
use crate::{subsystems::{rcs::RcsCommands, Subsystem, subsystem_filters}, SimulationTime};

pub fn time_event_generator(
    mut time_tick_events: EventWriter<surveyor_gnc::clock::TimeTickEvent>,
    mut q_sim_time: Query<&SimulationTime>,
) {
    let sim_time = q_sim_time.single_mut();

    let time_tick_event = TimeTickEvent {
        time: sim_time.time,
    };
    time_tick_events.send(time_tick_event);
}



#[derive(Debug, Clone, Event)]
pub enum SensorEvent {
    IMU(surveyor_gnc::sensors::IMUInput),
    StarTracker(surveyor_gnc::sensors::StarTrackerInput),
}

/// Conversion from truth-side data to GNC-side events
pub (crate) fn imu_event_generator(
    mut q_imu: Query<&mut Subsystem, With<Enum![Subsystem::Imu]>>,
    mut imu_input_events: EventWriter<surveyor_gnc::sensors::IMUInput>)
{
    // The Enum filter does not work on the very first update
    if let Some(mut subsystem) = q_imu.iter_mut().next()
    {
        let imu_subsystem = subsystem.as_imu_mut().unwrap();
        for (idx, sensor) in imu_subsystem.imus.iter().enumerate() {
            let imu_data = sensor.get_model_output();
            let imu_input = IMUInput::new(idx, imu_data.omega_cf, imu_data.accel_cf);
            imu_input_events.send(imu_input);
        }
    }
}

pub (crate) fn star_tracker_event_generator(
    mut q_st: Query<&mut Subsystem, With<Enum![Subsystem::StarTracker]>>,
    mut st_input_events: EventWriter<surveyor_gnc::sensors::StarTrackerInput>)
{
    // The Enum filter does not work on the very first update
    if let Some(mut subsystem) = q_st.iter_mut().next()
    {
        let st_subsystem = subsystem.as_star_tracker_mut().unwrap();
        for (idx, sensor) in st_subsystem.star_trackers.iter().enumerate() {
            let st_data = sensor.get_model_output();
            let st_input = StarTrackerInput{
                q_i2cf: st_data.q_i2cf.0,
                sensor_id: idx,
            };
            st_input_events.send(st_input);
        }
    }
}


/// Receive actuator events from the GNC system and send them to the simulation
/// We convert it into a truth-side type before passing it through
#[derive(Debug, Clone, Event)]
pub (crate) enum ActuatorEvent {
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

pub (crate) fn rcs_event_receiver(
    mut rcs_commands: EventReader<surveyor_gnc::control::RCSControllerOutput>,
    mut q_rcs: Query<&mut Subsystem, With<Enum!(Subsystem::Rcs)>>,

) {
    if let Some(mut subsystem) = q_rcs.iter_mut().next()
    {
        // If there are multiple events, only process the last one
        if let Some(event) = rcs_commands.read().last() {
            let rcs_subsystem = subsystem.as_rcs_mut().unwrap();
            rcs_subsystem.handle_commands(&RcsCommands::from(event));
        }
    }
}
