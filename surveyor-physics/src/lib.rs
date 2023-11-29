// pub mod plot;
pub mod integrators;
pub mod simulation;
pub mod spacecraft;
pub mod universe;
#[cfg(target_arch = "wasm32")]
pub mod visualization;

pub mod models;
pub mod subsystems;

pub mod config;

pub mod math;

pub mod interfaces;



use bevy_app::prelude::*;
use bevy_ecs::prelude::*;
use bevy_enum_filter::prelude::AddEnumFilter;
use config::SystemConfig;
use simulation::{SimulationResults, run_simulation_system};
use spacecraft::{InitialState, SpacecraftModel, build_spacecraft_entity, OrbitalDynamics, do_discrete_update};
use hard_xml::XmlRead;
use subsystems::Subsystem;
use universe::Universe;
use bevy_ecs::schedule::IntoSystemConfigs;

// Hardocde the timestep for now
pub const DT: f64 = 0.1; // s

#[derive(Debug, Clone, Eq, PartialEq, Hash, Default, States)]
pub enum SimulationState
{
    #[default]
    Paused,
    Running,
    Finished,
}

#[derive(Debug, Clone, Copy, Component, Default)]
pub struct SimulationTime(pub f64);


pub fn build_sim_ecs(mut commands: Commands)
{
    // Orbit with: a = 500 km, 0 degree inclination, 0 degree RAAN, 0 degree argument of perigee, 0 degree true anomaly
    let initial_state: InitialState = InitialState::from_str(include_str!("../initial_state.xml")).unwrap();
    // Create new spacecraft with engine subsystem
    let spacecraft_config_xml = include_str!("../simulation.xml");

    let config = SystemConfig::from_str(spacecraft_config_xml).unwrap();

    // Create new bevy ECS entity for spacecraft
    build_spacecraft_entity(&mut commands, &config.spacecraft, &initial_state);
    commands.spawn(Universe::from_config(config.universe));

    // let a: f64 = 6378.14 + 500.0;
    // // let period = 2.0 * PI * (a.powi(3) / 398600.0).sqrt();
    // let period = 0.05 * PI * (a.powi(3) / 398600.0).sqrt();
    // let sim_params: SimulationParams = SimulationParams::new(DT, initial_state, vec![SimStoppingCondition::MaxDuration(period)]);
    commands.insert_resource(config.simulation);
}

// System used to initalize the simulation
fn initialize_simulation(mut query: Query<(&SpacecraftModel, &mut OrbitalDynamics, &mut SimulationResults)>)
{
    let initial_state: InitialState = InitialState::from_str(include_str!("../initial_state.xml")).unwrap();
    let (_, mut orbital_dynamics, mut sim_results) = query.single_mut();
    *orbital_dynamics = OrbitalDynamics::from_initial_state(&initial_state);
    sim_results.history.clear();
}

pub struct SurveyorPhysicsPlugin;
impl Plugin for SurveyorPhysicsPlugin {
    fn build(&self, app: &mut App) {
        // Split this up into two systems - one that actually builds the ECS and one that
        // initializes the simulation. The latter can be run anytime we trigger a new simulation
        app.add_systems(Startup, build_sim_ecs)
            .add_enum_filter::<Subsystem>()
            .add_state::<SimulationState>()
            // Run `run_simulation_system` when we are in the `Running` state
            .add_systems(Update,
                (
                    spacecraft::step_spacecraft_model,
                    do_discrete_update,
                    run_simulation_system,
                ).chain().run_if(in_state(SimulationState::Running))
            )
            // Pass sensor data to GNC after all the models have been updated
            // and receive actuator events from GNC to be used in next update
            .add_systems(Update,
                (
                    crate::interfaces::imu_event_generator,
                    crate::interfaces::rcs_event_receiver
                ).chain().after(do_discrete_update)
            )
            // Run `initialize_simulation` when we enter the `Running` state
            .add_systems(OnEnter(SimulationState::Running),
                initialize_simulation
            );
    }
}
