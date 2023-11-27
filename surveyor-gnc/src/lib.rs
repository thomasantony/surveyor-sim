
/// ECS "Components" are used to mark the type of FSW interface that is being
/// implemented. This is used to query for dependencies from different modules in the FSW.
///
/// TODO: Use Events instead of Components for transmitting data between modules
pub mod sensors;
pub mod navigation;
pub mod guidance;
pub mod control;

use bevy_ecs::{prelude::*};
use control::{update_attitude_controller, update_control_allocator, update_rcs_controller};
use guidance::{update_guidance};
use navigation::{update_simple_attitude_estimator, update_sensor_aggregator};
use sensors::{update_imu, update_star_tracker};
use nalgebra as na;

use dashmap::DashMap;


use bevy_app::{prelude::*};
use bevy_ecs::prelude::Entity;
use bevy_ecs::schedule::IntoSystemConfigs;

pub struct SurveyorGNC {
    pub entities: DashMap<&'static str, Entity>,
}

#[derive(SystemSet, Debug, Hash, PartialEq, Eq, Clone)]
pub enum SurveyorGncSystemSet {
    Sensors,
    Navigation,
    Guidance,
    Control,
}

impl Plugin for SurveyorGNC {
    fn build(&self, app: &mut App) {
        app.add_event::<GncCommand>()
            .add_systems(Update, process_gnc_command.before(SurveyorGncSystemSet::Sensors))
            // Sensors
            .add_event::<sensors::EphemerisOutput>()
            .add_event::<sensors::IMUInput>()
            .add_event::<sensors::IMUOutput>()
            .add_event::<sensors::StarTrackerOutput>()
            .add_event::<sensors::StarTrackerInput>()
            .add_systems(Update, update_imu.in_set(SurveyorGncSystemSet::Sensors))
            .add_systems(Update, update_star_tracker.in_set(SurveyorGncSystemSet::Sensors))

            // Navigation
            .add_event::<navigation::SensorData>()
            .add_event::<navigation::AttitudeEstimatorOutput>()
            .add_systems(Update,
                (update_sensor_aggregator, update_simple_attitude_estimator)
                    .chain().in_set(SurveyorGncSystemSet::Navigation)
            )

            // Guidance
            .add_event::<guidance::AttitudeTarget>()
            .add_systems(Update, update_guidance.in_set(SurveyorGncSystemSet::Guidance))

            // Control
            .add_event::<control::AttitudeTorqueRequest>()
            .add_event::<control::RCSControllerInput>()
            .add_event::<control::RCSControllerOutput>()
            .add_systems(Update, (update_attitude_controller, update_control_allocator, update_rcs_controller).chain()
                .in_set(SurveyorGncSystemSet::Control)
            );

        app.configure_set(Update, SurveyorGncSystemSet::Sensors.before(SurveyorGncSystemSet::Navigation));
        app.configure_set(Update, SurveyorGncSystemSet::Navigation.before(SurveyorGncSystemSet::Guidance));
        app.configure_set(Update, SurveyorGncSystemSet::Guidance.before(SurveyorGncSystemSet::Control));

        let traj = app.world.spawn((Name("TrajectoryPhase"), TrajectoryPhase::BeforeRetroBurn,)).id();
        let imu = app.world.spawn((Name("IMU_A"), sensors::IMU, GeometryConfig::default())).id();
        let star_tracker = app.world.spawn((Name("ST_A"), sensors::StarTracker, GeometryConfig::default())).id();
        let guidance = app.world.spawn((Name("SurveyorGNCMode"), guidance::GuidanceMode::Idle)).id();
        let control_allocator = app.world.spawn((Name("ControlAllocator"), control::ControlAllocator::default())).id();
        let rcs_controller = app.world.spawn((Name("RCSController"), control::RCSController::default(), control::RCSControllerOutput::default())).id();

        self.entities.insert("TrajectoryPhase", traj);
        self.entities.insert("IMU_A", imu);
        self.entities.insert("ST_A", star_tracker);
        self.entities.insert("Guidance", guidance);
        self.entities.insert("ControlAllocator", control_allocator);
        self.entities.insert("RCSController", rcs_controller);
    }
}

impl SurveyorGNC {
    pub fn new() -> Self {

        Self {
            entities: DashMap::new(),
        }
    }
}

#[derive(Component)]
pub struct Name(pub &'static str);

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

#[derive(Event)]
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
        app.add_simple_outer_schedule().add_plugin(SurveyorGNC::new());
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
}
