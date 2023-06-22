
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
            .add_system(process_gnc_command)
            .add_event::<sensors::IMUInput>()
            .add_event::<sensors::IMUOutput>()
            .add_event::<sensors::StarTrackerOutput>()
            .add_event::<sensors::StarTrackerInput>()
            .add_system(update_imu.before(update_sensor_aggregator))
            .add_system(update_star_tracker.before(update_sensor_aggregator))
            .add_event::<navigation::SensorData>()
            .add_system(update_sensor_aggregator)
            .add_event::<navigation::AttitudeEstimatorOutput>()
            .add_system(update_simple_attitude_estimator.after(update_sensor_aggregator))
            .add_event::<guidance::AttitudeTarget>()
            .add_system(update_guidance.after(update_simple_attitude_estimator))
            .add_event::<control::AttitudeTorqueRequest>()
            .add_system(update_attitude_controller.after(update_guidance))
            .add_event::<control::RCSControllerInput>()
            .add_system(update_control_allocator.after(update_attitude_controller))
            .add_system(update_rcs_controller.after(update_control_allocator));
    }
}
pub fn build_fsw(mut commands: Commands) {
    let spacecraft_state = commands.spawn((TrajectoryPhase::BeforeRetroBurn,)).id();
    let imu = commands.spawn((sensors::IMU, GeometryConfig::default())).id();
    let star_tracker = commands.spawn((sensors::StarTracker, GeometryConfig::default())).id();
    let ephemeris = commands.spawn((sensors::EphemerisOutput::default())).id();
    let guidance = commands.spawn((guidance::GuidanceMode::Idle)).id();
    let control_allocator = commands.spawn((control::ControlAllocator::default())).id();
    let rcs_controller = commands.spawn((control::RCSController::default(), control::RCSControllerOutput::default())).id();
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
        let mut app = App::new();
        app.add_plugin(GNC);
        app.update();
        {
            let guidance_mode = app.world.query::<&mut guidance::GuidanceMode>().single(&app.world);
            assert_eq!(*guidance_mode, guidance::GuidanceMode::Idle);
        }

        // Send a command to the FSW using GncCommand event
        app.world.send_event(GncCommand::SetGuidanceMode(guidance::GuidanceMode::Manual));
        app.update();
        let guidance_mode = app.world.query::<&mut guidance::GuidanceMode>().single(&app.world);
        assert_eq!(*guidance_mode, guidance::GuidanceMode::Manual);
    }
    #[test]
    fn test_imu_sensor() {
        let mut app = App::new();
        app.add_plugin(GNC);
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
        let evt = app.world.get_resource::<Events<sensors::IMUOutput>>().unwrap();
        let mut reader = evt.get_reader();
        let imu_output = reader.iter(&evt).next().unwrap();
        assert_eq!(imu_output.omega_b, omega_b);
    }
    #[test]
    fn test_star_tracker()
    {
        let mut app = App::new();
        app.add_plugin(GNC);
        app.update();

        let q_j20002cf = nalgebra::UnitQuaternion::from_euler_angles(0.1, 0.2, 0.3);

        let st_input = crate::sensors::StarTrackerInput {
            sensor_id: 0,
            q_j20002cf: q_j20002cf,
        };
        app.world.send_event(st_input);

        app.update();

        // Read Star Tracker output event
        let evt = app.world.get_resource::<Events<sensors::StarTrackerOutput>>().unwrap();
        let mut reader = evt.get_reader();
        let st_output = reader.iter(&evt).next().unwrap();
        assert_eq!(st_output.q_i2b, q_j20002cf);
    }
}
