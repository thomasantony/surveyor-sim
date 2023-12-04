
use bevy_ecs::prelude::*;
use nalgebra as na;

use surveyor_types::config::GeometryConfig;

use crate::clock::SystemClock;

#[derive(Debug, Clone, Component)]
pub struct StarTracker;

// TODO: Switch to using entity IDs instead of usize
#[derive(Debug, Clone, Event)]
pub struct StarTrackerInput {
    pub sensor_id: usize,
    pub q_i2cf: na::UnitQuaternion<f64>,
}
impl Default for StarTrackerInput {
    fn default() -> Self {
        Self {
            sensor_id: 0,
            q_i2cf: na::UnitQuaternion::identity(),
        }
    }
}

#[derive(Debug, Clone, Event)]
pub struct StarTrackerOutput
{
    pub q_i2b: na::UnitQuaternion<f64>,
    pub measurement_time: hifitime::Epoch,
    pub valid: bool,
}
impl Default for StarTrackerOutput {
    fn default() -> Self {
        Self {
            q_i2b: na::UnitQuaternion::identity(),
            measurement_time: hifitime::Epoch::default(),
            valid: false,
        }
    }
}


#[derive(Debug, Clone, Component)]
pub struct IMU;

#[derive(Debug, Clone, Event)]
pub struct IMUInput
{
    pub sensor_id: usize,
    pub omega_cf: na::Vector3<f64>,
    pub acc_cf: na::Vector3<f64>
}
impl IMUInput {
    pub fn new(sensor_id: usize, omega_cf: na::Vector3<f64>, acc_cf: na::Vector3<f64>) -> Self {
        Self {
            sensor_id,
            omega_cf,
            acc_cf,
        }
    }
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
#[derive(Debug, Clone, Event)]
pub struct IMUOutput
{
    pub omega_b: na::Vector3<f64>,
    pub acc_b: na::Vector3<f64>,
    pub measurement_time: hifitime::Epoch,
}
impl Default for IMUOutput {
    fn default() -> Self {
        Self {
            omega_b: na::Vector3::zeros(),
            acc_b: na::Vector3::zeros(),
            measurement_time: hifitime::Epoch::default(),
        }
    }
}

/// This is set by an ephemeris estimator, radar sensor or other source
/// For a start, this component could be set directly from the simulation
#[derive(Debug, Clone, Component, Event)]
pub struct EphemerisOutput
{
    pub pos_i: na::Vector3<f64>,
    pub vel_i: na::Vector3<f64>,
    pub time: hifitime::Epoch,
}
impl Default for EphemerisOutput {
    fn default() -> Self {
        Self {
            pos_i: na::Vector3::zeros(),
            vel_i: na::Vector3::zeros(),
            time: hifitime::Epoch::default(),
        }
    }
}


/// System to update the IMU output
/// Generalize this later to apply to any sensor with a vector input in component frame
pub fn update_imu(
    mut imu_input: EventReader<IMUInput>,
    mut query: Query<(&IMU, &GeometryConfig)>,
    mut output: EventWriter<IMUOutput>,
    clock: Res<SystemClock>,
) {
    // Rotate the angular velocity from the CF frame to the body frame and
    // assign it to the IMU output

    // Get IMU and geometry by sensor id
    imu_input.read().last().map(|imu_input|
    {
        if let Some((_, geometry)) = query.iter_mut().nth(imu_input.sensor_id)
        {
            let imu_output = IMUOutput{
                omega_b: geometry.q_cf2b * imu_input.omega_cf,
                acc_b: geometry.q_cf2b * imu_input.acc_cf,
                measurement_time: clock.time,
            };
            output.send(imu_output);
        }else{
            log::error!("IMU sensor id {} not found", imu_input.sensor_id);
        }
    });
}

pub fn update_star_tracker(
    mut star_tracker_input: EventReader<StarTrackerInput>,
    mut query: Query<(&StarTracker, &GeometryConfig)>,
    mut output: EventWriter<StarTrackerOutput>,
    clock: Res<SystemClock>,
) {
    star_tracker_input.read().last().map(| star_tracker_input|{
        // Get star tracker and geometry by sensor id
        if let Some((_, geometry,
                )) = query.iter_mut().nth(star_tracker_input.sensor_id)
        {
            let star_tracker_output = StarTrackerOutput{
                q_i2b: geometry.q_cf2b * star_tracker_input.q_i2cf,
                measurement_time: clock.time,
                valid: true,
            };
            output.send(star_tracker_output);
        }else{
            log::error!("Star tracker sensor id {} not found", star_tracker_input.sensor_id);
        }
    });
}

#[cfg(test)]
mod tests
{
    use bevy_app::{App, Update};
    use bevy_ecs::prelude::*;
    use crate::{sensors::{IMUOutput, StarTrackerOutput}, Name};

    use super::*;

    fn create_app() -> App
    {
        let mut app = App::new();
        app.add_event::<IMUInput>()
            .add_event::<IMUOutput>()
            .add_event::<StarTrackerOutput>()
            .add_event::<StarTrackerInput>()
            .add_systems(Update, update_imu)
            .add_systems(Update, update_star_tracker);
        app.world.spawn((Name::new("IMU_A"), IMU, GeometryConfig::default()));
        app.world.spawn((Name::new("ST_A"), StarTracker, GeometryConfig::default()));
        app
    }
    #[test]
    fn test_imu_sensor() {
        let mut app = create_app();
        app.update();

        // Set some dummy inputs to the sensor
        let omega_b = nalgebra::Vector3::new(0.1, 0.2, 0.3);
        let imu_input = crate::sensors::IMUInput {
            sensor_id: 0,
            acc_cf: nalgebra::Vector3::zeros(),
            omega_cf: omega_b,
        };
        // Send events
        app.world.send_event(imu_input);
        app.update();

        // Read IMU output event
        let evt = app.world.get_resource::<Events<IMUOutput>>().unwrap();
        let mut reader = evt.get_reader();
        let imu_output = reader.read(&evt).next().unwrap();
        assert_eq!(imu_output.omega_b, omega_b);
    }
    #[test]
    fn test_star_tracker()
    {
        let mut app = create_app();
        app.update();

        let q_i2cf = nalgebra::UnitQuaternion::from_euler_angles(0.1, 0.2, 0.3);

        let st_input = crate::sensors::StarTrackerInput {
            sensor_id: 0,
            q_i2cf: q_i2cf,
        };
        app.world.send_event(st_input);

        app.update();

        // Read Star Tracker output event
        let evt = app.world.get_resource::<Events<StarTrackerOutput>>().unwrap();
        let mut reader = evt.get_reader();
        let st_output = reader.read(&evt).next().unwrap();
        assert_eq!(st_output.q_i2b, q_i2cf);
    }
}
