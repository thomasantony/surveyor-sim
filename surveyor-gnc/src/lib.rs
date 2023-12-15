
/// ECS "Components" are used to mark the type of FSW interface that is being
/// implemented. This is used to query for dependencies from different modules in the FSW.
///
/// TODO: Use Events instead of Components for transmitting data between modules
pub mod sensors;
pub mod navigation;
pub mod guidance;
pub mod control;
pub mod clock;

use bevy::core::Name;
use bevy_ecs::prelude::*;
use clock::TimeTickEvent;
use control::{update_attitude_controller, update_control_allocator, update_rcs_controller, RCSController};
use guidance::update_guidance;

use navigation::{update_simple_attitude_estimator, update_sensor_aggregator};
use sensors::{update_imu, update_star_tracker};

use dashmap::DashMap;

use bevy_app::prelude::*;
use bevy_ecs::prelude::Entity;
use bevy_ecs::schedule::IntoSystemConfigs;

pub struct SurveyorGNC {
    pub entities: DashMap<String, Entity>,
}

#[derive(SystemSet, Debug, Hash, PartialEq, Eq, Clone)]
pub enum SurveyorGncSystemSet {
    Sensors,
    Navigation,
    Guidance,
    Control,
}

pub fn setup_gnc(mut commands: Commands)
{
    commands.insert_resource(clock::SystemClock::default());
}

impl Plugin for SurveyorGNC {
    fn build(&self, app: &mut App) {
        app.add_event::<GncCommand>()
            // Time
            .add_event::<TimeTickEvent>()
            .add_systems(Startup, setup_gnc)
            .add_systems(Update, clock::update_sys_clock)

            // Main GNC command processor
            .add_systems(Update, process_gnc_command)
            // Sensors
            .add_event::<sensors::EphemerisOutput>()
            .add_event::<sensors::IMUInput>()
            .add_event::<sensors::IMUOutput>()
            .add_event::<sensors::StarTrackerOutput>()
            .add_event::<sensors::StarTrackerInput>()
            .add_event::<sensors::StarSensorOutput>()
            .add_event::<sensors::StarSensorInput>()
            .add_systems(Update, update_imu.in_set(SurveyorGncSystemSet::Sensors))
            .add_systems(Update, update_star_tracker.in_set(SurveyorGncSystemSet::Sensors))
            .add_systems(Update, update_star_sensor.in_set(SurveyorGncSystemSet::Sensors))

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
            .add_event::<control::RCSTorqueRequest>()
            .add_event::<control::TVCTorqueRequest>()
            .add_event::<control::TVCControllerOutput>()
            .add_event::<control::VernierTorqueRequest>()
            .add_event::<control::RCSControllerOutput>()
            .add_systems(Update, (update_attitude_controller, update_control_allocator, update_rcs_controller).chain()
                .in_set(SurveyorGncSystemSet::Control)
            );

        // Configure the system sets
        app.configure_sets(Update, SurveyorGncSystemSet::Sensors.after(process_gnc_command));
        app.configure_sets(Update, SurveyorGncSystemSet::Sensors.before(SurveyorGncSystemSet::Navigation));
        app.configure_sets(Update, SurveyorGncSystemSet::Navigation.before(SurveyorGncSystemSet::Guidance));
        app.configure_sets(Update, SurveyorGncSystemSet::Guidance.before(SurveyorGncSystemSet::Control));

        let traj = app.world.spawn((Name::new("TrajectoryPhase"), TrajectoryPhase::BeforeRetroBurn,)).id();

        let imu_config_xml = vec![r#"
            <Imu name="A">
                <geometry>
                    <q_cf2b>[1.0, 0.0, 0.0, 0.0]</q_cf2b>
                    <cf_offset_com_b>[0.0, 0.0, 0.0]</cf_offset_com_b>
                </geometry>
            </Imu>
        "#, r#"
        <Imu name="B">
                <geometry>
                    <q_cf2b>[0.0, 0.0, 0.0, 1.0]</q_cf2b>
                    <cf_offset_com_b>[0.0, 0.0, 0.0]</cf_offset_com_b>
                </geometry>
            </Imu>
        "#];

        let imu_configs = imu_config_xml.iter().map(|c| ImuConfig::from_str(c).unwrap()).collect::<Vec<_>>();
        for imu_config in imu_configs {
            let imu = app.world.spawn((
                Name::new(imu_config.name.clone()),
                sensors::IMU,
                GeometryConfig::from_geometry_params(&imu_config.geometry)
            )).id();
            self.entities.insert(imu_config.name.to_string(), imu);
        }

        let st_config_xml = r#"<StarTracker name="A">
                <geometry>
                    <!-- Points boresight along -Y (body frame) -->
                    <q_cf2b>[0.7071068, 0.0, 0.7071068, 0.0]</q_cf2b>
                    <!-- Random -->
                    <cf_offset_com_b>[0.0, 0.0, 0.0]</cf_offset_com_b>
                </geometry>
            </StarTracker>"#;
        let st_config = StarTrackerConfig::from_str(st_config_xml).unwrap();
        let star_tracker = app.world.spawn((
            Name::new(st_config.name.clone()),
            sensors::StarTracker,
            GeometryConfig::from_geometry_params(&st_config.geometry)
        )).id();


        let star_sensor_config_xml = r#"
            <StarSensor name="Canopus">
                <geometry>
                    <!-- Points boresight along -Y (body frame) -->
                    <q_cf2b>[0.7071068, 0.0, 0.7071068, 0.0]</q_cf2b>
                    <!-- Random -->
                    <cf_offset_com_b>[0.0, 0.0, 0.0]</cf_offset_com_b>
                </geometry>
                <!-- 45 deg FoV (22.5 deg half angle)-->
                <fov_deg>22.5</fov_deg>
                <!-- Canopus:
                    Right ascension	06h 23m 57.10988s
                    Declination	−52° 41' 44.3810"
                -->
                <right_ascension_deg>95.9875</right_ascension_deg>
                <declinaton_deg>52.695661389</declinaton_deg>
            </StarSensor>
        "#;
        let st_config = StarSensorConfig::from_str(star_sensor_config_xml).unwrap();
        let star_sensor = app.world.spawn((
            Name::new(st_config.name.clone()),
            sensors::StarSensor,
            GeometryConfig::from_geometry_params(&st_config.geometry)
        )).id();

        let guidance = app.world.spawn((Name::new("SurveyorGNCMode"), guidance::GuidanceMode::Idle)).id();

        // todo: fix this to use correct config
        let rcs_config_xml = vec![r#"
            <thruster type="RCS" name="roll1">
                <min_thrust>0.0</min_thrust>
                <max_thrust>0.25</max_thrust>
                <geometry>
                    <!-- thruster pointed in the +X direction -->
                    <q_cf2b>[0.7071068, 0.0, 0.7071068, 0.0]</q_cf2b>
                    <cf_offset_com_b>[-0.1, 1.0, -0.5]</cf_offset_com_b>
                </geometry>
            </thruster>
            "#, r#"
            <thruster type="RCS" name="roll2">
                <min_thrust>0.0</min_thrust>
                <max_thrust>0.25</max_thrust>
                <geometry>
                    <!-- thruster pointed in the -X direction -->
                    <q_cf2b>[0.7071068, 0.0, -0.7071068, 0.0]</q_cf2b>
                    <cf_offset_com_b>[-0.1, 1.0, -0.5]</cf_offset_com_b>
                </geometry>
            </thruster>
            "#, r#"
            <thruster type="RCS" name="leg2A">
                <min_thrust>0.0</min_thrust>
                <max_thrust>0.25</max_thrust>
                <geometry>
                    <!-- thruster pointed in the +Z direction -->
                    <q_cf2b>[1.0, 0.0, 0.0, 0.0]</q_cf2b>
                    <cf_offset_com_b>[0.8660254037844386, -0.5, -0.6]</cf_offset_com_b>
                </geometry>
            </thruster>
            "#, r#"
            <thruster type="RCS" name="leg2B">
                <min_thrust>0.0</min_thrust>
                <max_thrust>0.25</max_thrust>
                <geometry>
                    <!-- thruster pointed in the -Z direction -->
                    <q_cf2b>[0.0, 1.0, 0.0, 0.0]</q_cf2b>
                    <cf_offset_com_b>[0.8660254037844386, -0.5, -0.4]</cf_offset_com_b>
                </geometry>
            </thruster>
            "#, r#"
            <thruster type="RCS" name="leg3A">
                <min_thrust>0.0</min_thrust>
                <max_thrust>0.25</max_thrust>
                <geometry>
                    <!-- thruster pointed in the +Z direction -->
                    <q_cf2b>[1.0, 0.0, 0.0, 0.0]</q_cf2b>
                    <cf_offset_com_b>[-0.8660254037844386, -0.5, -0.6]</cf_offset_com_b>
                </geometry>
            </thruster>
            "#, r#"
            <thruster type="RCS" name="leg3B">
                <min_thrust>0.0</min_thrust>
                <max_thrust>0.25</max_thrust>
                <geometry>
                    <!-- thruster pointed in the -Z direction -->
                    <q_cf2b>[0.0, 1.0, 0.0, 0.0]</q_cf2b>
                    <cf_offset_com_b>[-0.8660254037844386, -0.5, -0.4]</cf_offset_com_b>
                </geometry>
            </thruster>
            "#,
        ];
        use hard_xml::XmlRead;
        let rcs_config = rcs_config_xml.iter().map(|c| ThrusterConfig::from_str(c).unwrap()).collect::<Vec<_>>();
        let rcs_controller = RCSController::new(&rcs_config);

        let control_allocator = app.world.spawn((Name::new("ControlAllocator"), control::ControlAllocator::default())).id();
        let rcs_controller = app.world.spawn((Name::new("RCSController"), rcs_controller, control::RCSControllerOutput::default())).id();

        self.entities.insert("TrajectoryPhase".to_string(), traj);

        self.entities.insert("ST_A".to_string(), star_tracker);
        self.entities.insert("Guidance".to_string(), guidance);
        self.entities.insert("ControlAllocator".to_string(), control_allocator);
        self.entities.insert("RCSController".to_string(), rcs_controller);
        self.entities.insert("StarSensor_A".to_string(), star_sensor);
    }
}

impl SurveyorGNC {
    pub fn new() -> Self {

        Self {
            entities: DashMap::new(),
        }
    }
}
use surveyor_types::config::*;
pub use surveyor_types::config::GeometryConfig;

use crate::sensors::update_star_sensor;

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
    for command in command.read() {
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
        app.add_plugins(SurveyorGNC::new());
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
