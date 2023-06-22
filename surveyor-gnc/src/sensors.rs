use bevy_ecs::prelude::*;
use nalgebra as na;
use chrono::prelude::*;

use crate::GeometryConfig;

#[derive(Debug, Clone, Component, Default)]
pub struct Measurement<T: Default> {
    pub value: T,
    pub time: DateTime<Utc>,
    pub valid: bool,
}

#[derive(Debug, Clone, Component)]
pub struct StarTrackerInput {
    pub sensor_id: usize,
    pub q_j20002cf: na::UnitQuaternion<f64>,
}
impl Default for StarTrackerInput {
    fn default() -> Self {
        Self {
            sensor_id: 0,
            q_j20002cf: na::UnitQuaternion::identity(),
        }
    }
}

#[derive(Debug, Clone, Component)]
pub struct StarTrackerOutput
{
    pub q_i2b: na::UnitQuaternion<f64>,
}
impl Default for StarTrackerOutput {
    fn default() -> Self {
        Self {
            q_i2b: na::UnitQuaternion::identity(),
        }
    }
}

#[derive(Debug, Clone, Component)]
pub struct IMUInput
{
    pub sensor_id: usize,
    pub omega_cf: na::Vector3<f64>,
    pub acc_cf: na::Vector3<f64>
}
impl Default for IMUInput {
    fn default() -> Self {
        Self {
            sensor_id: 0,
            omega_cf: na::Vector3::zeros(),
            acc_cf: na::Vector3::zeros(),
        }
    }
}

// TODO:  Add fields to show the time of the measurement being ingested
// and if it is stale
#[derive(Debug, Clone, Component)]
pub struct IMUOutput
{
    pub omega_b: na::Vector3<f64>,
    pub acc_b: na::Vector3<f64>,
}
impl Default for IMUOutput {
    fn default() -> Self {
        Self {
            omega_b: na::Vector3::zeros(),
            acc_b: na::Vector3::zeros(),
        }
    }
}

/// This is set by an ephemeris estimator, radar sensor or other source
/// For a start, this component could be set directly from the simulation
#[derive(Debug, Clone, Component)]
pub struct EphemerisOutput
{
    pub pos_i: na::Vector3<f64>,
    pub vel_i: na::Vector3<f64>,
}
impl Default for EphemerisOutput {
    fn default() -> Self {
        Self {
            pos_i: na::Vector3::zeros(),
            vel_i: na::Vector3::zeros(),
        }
    }
}


#[derive(Debug, Clone, Component)]
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

/// System to update the IMU output
/// Generalize this later to apply to any sensor with a vector input in component frame
pub fn update_imu(mut imu_input: EventReader<IMUInput>,  mut query: Query<(&GeometryConfig, &mut IMUOutput)>) {
    // Rotate the angular velocity from the CF frame to the body frame and
    // assign it to the IMU output
    let imu_input = imu_input.iter().last().unwrap();
    // Get IMU and geometry by sensor id
    if let Some((geometry,
            mut imu_output)) = query.iter_mut().nth(imu_input.sensor_id)
    {
        imu_output.omega_b = geometry.vec_cf2b(&imu_input.omega_cf);
        imu_output.acc_b = geometry.vec_cf2b(&imu_input.acc_cf);
    }else{
        log::error!("IMU sensor id {} not found", imu_input.sensor_id);
    }
}

pub fn update_star_tracker(mut star_tracker_input: EventReader<StarTrackerInput>,
                           mut query: Query<(&GeometryConfig, &mut StarTrackerOutput)>) {
    let star_tracker_input = star_tracker_input.iter().last().unwrap();
    // Get star tracker and geometry by sensor id
    if let Some((geometry,
            mut star_tracker_output)) = query.iter_mut().nth(star_tracker_input.sensor_id)
    {
        star_tracker_output.q_i2b = geometry.q_cf2b * star_tracker_input.q_j20002cf;
    }else{
        log::error!("Star tracker sensor id {} not found", star_tracker_input.sensor_id);
    }
}
