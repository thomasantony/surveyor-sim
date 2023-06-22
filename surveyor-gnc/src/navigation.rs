use bevy_ecs::prelude::*;
use nalgebra as na;
// TODO: Switch to using hifitime
use chrono::prelude::*;
use crate::sensors::{self, StarTrackerOutput, IMUOutput};


#[derive(Debug, Clone, Default)]
pub struct Measurement<T: Default> {
    pub value: T,
    pub time: DateTime<Utc>,
    pub valid: bool,
}


#[derive(Debug, Clone)]
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
#[derive(Debug, Clone)]
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
                                mut sensor_data_writer: EventWriter<SensorData>)
{
    // TODO: Pass through "valid" flag and timestamp from the sensor data
    // Assume there is only a single sensor data entity
    let mut sensor_data = SensorData::default();
    // Iterate over all imu outputs and add them to the sensor data
    for (imu_idx, imu_output) in imu_query.iter().enumerate() {
        let meas = Measurement {
            value: imu_output.clone(),
            time: chrono::Utc::now(),
            valid: true,
        };
        sensor_data.imus[imu_idx] = meas;
    }
    // Iterate over all star tracker outputs and add them to the sensor data
    for (st_idx, star_tracker_output) in str_query.iter().enumerate() {
        let meas = Measurement {
            value: star_tracker_output.clone(),
            time: chrono::Utc::now(),
            valid: true,
        };
        sensor_data.star_trackers[st_idx] = meas;
    }
    sensor_data_writer.send(sensor_data);
}

/// A simple attitude estimator that uses the data from the IMU and Star Tracker directly
pub fn update_simple_attitude_estimator(mut sensor_data_reader: EventReader<SensorData>,
                                        mut attitude_estimate_writer: EventWriter<AttitudeEstimatorOutput>) {
    let mut attitude_estimator_output = AttitudeEstimatorOutput::default();
    if let Some(sensor_data) = sensor_data_reader.iter().last()
    {
        let imu = sensor_data.imus[0].value.clone();
        let star_tracker = sensor_data.star_trackers[0].value.clone();
        attitude_estimator_output.q_i2b = star_tracker.q_i2b;
        attitude_estimator_output.omega_b = imu.omega_b;
    }
    attitude_estimate_writer.send(attitude_estimator_output);
}
