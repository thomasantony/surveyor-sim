use bevy_ecs::prelude::*;
use nalgebra as na;

use crate::sensors;

#[derive(Debug, Clone, Component)]
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

pub fn update_sensor_aggregator(mut imu_query: EventReader<sensors::IMUOutput>,
                                mut str_query: EventReader<sensors::StarTrackerOutput>,
                                mut sensor_data: Query<(&mut sensors::SensorData)>)
{
    // TODO: Pass through "valid" flag and timestamp from the sensor data
    // Assume there is only a single sensor data entity
    let mut sensor_data = sensor_data.single_mut();
    // Iterate over all imu outputs and add them to the sensor data
    for (imu_idx, imu_output) in imu_query.iter().enumerate() {
        let mut meas = sensors::Measurement::default();
        meas.value = imu_output.clone();
        meas.time = chrono::Utc::now();
        meas.valid = true;
        sensor_data.imus[imu_idx] = meas;
    }
    // Iterate over all star tracker outputs and add them to the sensor data
    for (st_idx, star_tracker_output) in str_query.iter().enumerate() {
        let mut meas = sensors::Measurement::default();
        meas.value = star_tracker_output.clone();
        meas.time = chrono::Utc::now();
        meas.valid = true;
        sensor_data.star_trackers[st_idx] = meas;
    }
}

/// A simple attitude estimator that uses the data from the IMU and Star Tracker directly
pub fn update_simple_attitude_estimator(sensor_data_query: Query<&sensors::SensorData>,
                                        mut estimator_query: Query<&mut AttitudeEstimatorOutput>) {
    let mut attitude_estimator_output = estimator_query.single_mut();
    let sensor_data = sensor_data_query.single();
    let imu = sensor_data.imus[0].value.clone();
    let star_tracker = sensor_data.star_trackers[0].value.clone();
    attitude_estimator_output.q_i2b = star_tracker.q_i2b;
    attitude_estimator_output.omega_b = imu.omega_b;
}
