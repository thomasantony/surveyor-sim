
/// ECS "Components" are used to mark the type of FSW interface that is being
/// implemented. This is used to query for dependencies from different modules in the FSW.
///
/// TODO: Use Events instead of Components for transmitting data between modules
pub mod sensors;
pub mod navigation;
pub mod guidance;
pub mod control;

use bevy_app::{Plugin, App};
use bevy_ecs::{prelude::*};
use control::{update_attitude_controller, update_control_allocator, update_rcs_controller};
use guidance::{update_guidance};
use navigation::{update_simple_attitude_estimator, update_sensor_aggregator};
use sensors::{update_imu, update_star_tracker};
use nalgebra as na;

/// Structure defining geometry of any spacecraft component
#[derive(Debug, Clone, Component)]
pub struct GeometryConfig {
    pub q_cf2b: na::UnitQuaternion<f64>,
    pub cf_b: na::Vector3<f64>,
}
impl Default for GeometryConfig {
    fn default() -> Self {
        Self {
            q_cf2b: na::UnitQuaternion::identity(),
            cf_b: na::Vector3::zeros(),
        }
    }
}
impl GeometryConfig {
    pub fn vec_cf2b(&self, vec_cf: &na::Vector3<f64>) -> na::Vector3<f64> {
        self.q_cf2b * vec_cf
    }
    pub fn vec_b2cf(&self, vec_b: &na::Vector3<f64>) -> na::Vector3<f64> {
        self.q_cf2b.inverse_transform_vector(vec_b)
    }
}

/// Bevy Plugin for the FSW
pub struct GNC;
impl Plugin for GNC {
    fn build(&self, app: &mut App) {
        app.add_startup_system(build_fsw)
            .add_event::<GncCommand>()
            .add_event::<sensors::IMUInput>()
            .add_event::<sensors::StarTrackerInput>()
            .add_system(process_gnc_command)
            .add_system(update_imu.before(update_sensor_aggregator))
            .add_system(update_star_tracker.before(update_sensor_aggregator))
            .add_system(update_sensor_aggregator)
            .add_system(update_simple_attitude_estimator.after(update_sensor_aggregator))
            .add_system(update_guidance.after(update_simple_attitude_estimator))
            .add_system(update_attitude_controller.after(update_guidance))
            .add_system(update_control_allocator.after(update_attitude_controller))
            .add_system(update_rcs_controller.after(update_control_allocator));
    }
}
pub fn build_fsw(mut commands: Commands) {
    let spacecraft_state = commands.spawn((TrajectoryPhase::BeforeRetroBurn,)).id();
    let imu = commands.spawn((sensors::IMUOutput::default(), GeometryConfig::default())).id();
    let star_tracker = commands.spawn((sensors::StarTrackerOutput::default(), GeometryConfig::default())).id();
    let ephemeris = commands.spawn((sensors::EphemerisOutput::default())).id();
    let sensor_data = commands.spawn((sensors::SensorData::default())).id();
    let estimator = commands.spawn((navigation::AttitudeEstimatorOutput::default())).id();
    let attitude_target = commands.spawn((guidance::AttitudeTarget::default())).id();
    let guidance = commands.spawn((guidance::GuidanceMode::Idle, control::AttitudeTorqueRequest::default())).id();
    let control_allocator = commands.spawn((control::ControlAllocator::default())).id();
    let rcs_controller = commands.spawn((control::RCSController::default(), control::RCSControllerInput::default(), control::RCSControllerOutput::default())).id();
}

pub enum GncCommand {
    SetGuidanceMode(guidance::GuidanceMode),
}

/// Refactor to move this out of the main FSW module
/// Maybe make it part of the guidance module?
#[derive(Debug, Clone, Component)]
pub enum TrajectoryPhase {
    BeforeRetroBurn,
    DescentContour,
    TerminalDescent,
    Landed,
}

// System to process commands
pub fn process_gnc_command(mut command: EventReader<GncCommand>,
                           mut guidance_query: Query<&mut guidance::GuidanceMode>)
{
    let mut guidance_mode = guidance_query.single_mut();
    for command in command.iter() {
        match command {
            GncCommand::SetGuidanceMode(new_mode) => {
                *guidance_mode = new_mode.clone();
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    /// Test that checks if the guidance mode gets set correctly by the command processor
    #[test]
    fn test_command_handler()
    {
        let mut fsw = World::new();
        fsw.spawn((guidance::GuidanceMode::Idle,));

        let gnc_command = GncCommand::SetGuidanceMode(guidance::GuidanceMode::Manual);
        process_gnc_command(gnc_command, &mut fsw);

        let guidance_mode = fsw.query::<&mut guidance::GuidanceMode>().single_mut(&mut fsw);
        assert_eq!(*guidance_mode, guidance::GuidanceMode::Manual);
    }
}
//     fn spawn_attitude_sensor_and_estimator() -> World {
//         let mut fsw = World::new();
//         let sensor = sensors::PerfectAttitudeSensor::spawn_entity(&mut fsw);
//         let estimator = navigation::AttitudeEstimator::spawn_entity(&mut fsw);
//         fsw.insert_one(sensor, sensors::PerfectAttitudeSensor {}).unwrap();
//         fsw.insert_one(estimator, navigation::AttitudeEstimator::default()).unwrap();
//         fsw
//     }
//     #[test]
//     fn test_attitude_estimator() {
//         let mut fsw = spawn_attitude_sensor_and_estimator();

//         // Set some dummy inputs to the sensor
//         let q_i2b = nalgebra::UnitQuaternion::from_euler_angles(0.1, 0.2, 0.3);
//         let omega_b = nalgebra::Vector3::new(0.1, 0.2, 0.3);
//         sensors::PerfectAttitudeSensor::set_input(&mut fsw, q_i2b, omega_b);

//         // Update the estimator
//         println!("Updating attitude estimator");
//         navigation::AttitudeEstimator::system_update_attitude_estimator(&mut fsw);

//         // Verify that the attitude estimate and angular velocity propagated correctly
//         let mut query = fsw.query::<&navigation::AttitudeEstimator>();
//         for (_id, estimator) in query.iter() {
//             assert_eq!(estimator.get_q_i2b(), q_i2b);
//             assert_eq!(estimator.get_omega_b(), omega_b);
//         }
//     }
// }
