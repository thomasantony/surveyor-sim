use bevy_debug_text_overlay::screen_print;
use bevy_ecs::prelude::*;
use hifitime::prelude::*;
use nalgebra as na;
use crate::{sensors::{StarTrackerOutput, IMUOutput}, clock::SystemClock};


#[derive(Debug, Clone, Default)]
pub struct Measurement<T: Default> {
    pub value: T,
    pub time: Epoch,
    pub valid: bool,
}


#[derive(Debug, Clone, Event)]
pub struct SensorData {
    pub imus: [Measurement<IMUOutput>; 2],
    pub star_trackers: [Measurement<StarTrackerOutput>; 2],
}

impl Default for SensorData {
    fn default() -> Self {
        Self {
            imus: [Measurement::default(), Measurement::default()],
            star_trackers: [Measurement::default(), Measurement::default()],
        }
    }
}
#[derive(Debug, Clone, Event)]
pub struct AttitudeEstimatorOutput {
    pub q_i2b: na::UnitQuaternion<f64>,
    pub omega_b: na::Vector3<f64>,
}
impl Default for AttitudeEstimatorOutput {
    fn default() -> Self {
        Self {
            q_i2b: na::UnitQuaternion::identity(),
            omega_b: na::Vector3::zeros(),
        }
    }
}

pub fn update_sensor_aggregator(mut imu_query: EventReader<IMUOutput>,
                                mut str_query: EventReader<StarTrackerOutput>,
                                mut sensor_data_writer: EventWriter<SensorData>,
                                _clock: Res<SystemClock>
)
{
    // TODO: Pass through "valid" flag and timestamp from the sensor data
    // Assume there is only a single sensor data entity
    let mut sensor_data = SensorData::default();
    // Iterate over all imu outputs and add them to the sensor data
    for (imu_idx, imu_output) in imu_query.read().enumerate() {
        let meas = Measurement {
            value: imu_output.clone(),
            time: imu_output.measurement_time,
            valid: true,
        };
        sensor_data.imus[imu_idx] = meas;
    }
    // Iterate over all star tracker outputs and add them to the sensor data
    for (st_idx, star_tracker_output) in str_query.read().enumerate() {
        let meas = Measurement {
            value: star_tracker_output.clone(),
            time: star_tracker_output.measurement_time,
            valid: true,
        };
        sensor_data.star_trackers[st_idx] = meas;
    }
    // TODO: Mark measurements as invalid if they are too old
    sensor_data_writer.send(sensor_data);
}

/// A simple attitude estimator that uses the data from the IMU and Star Tracker directly
pub fn update_simple_attitude_estimator(mut sensor_data_reader: EventReader<SensorData>,
                                        mut attitude_estimate_writer: EventWriter<AttitudeEstimatorOutput>) {
    let mut attitude_estimator_output = AttitudeEstimatorOutput::default();
    if let Some(sensor_data) = sensor_data_reader.read().last()
    {
        // Find first IMU with valid data
        if let Some(imu) = sensor_data.imus.iter().filter(|x| x.valid).next()
        {
            attitude_estimator_output.omega_b = imu.value.omega_b;
        }

        if let Some(star_tracker) = sensor_data.star_trackers.iter().filter(|x| x.valid).next()
        {
            attitude_estimator_output.q_i2b = star_tracker.value.q_i2b;
        }
        // screen_print!("Omega_b: {:.2?}", attitude_estimator_output.omega_b);
    }
    attitude_estimate_writer.send(attitude_estimator_output);
}
