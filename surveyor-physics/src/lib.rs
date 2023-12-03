// pub mod plot;
pub mod integrators;
pub mod simulation;
pub mod spacecraft;
pub mod universe;
#[cfg(target_arch = "wasm32")]
pub mod visualization;

pub mod models;
pub mod subsystems;

pub use surveyor_types::math;
use surveyor_types::config::Config;
pub mod interfaces;


use bevy_app::prelude::*;
use bevy_ecs::prelude::*;
use bevy_enum_filter::prelude::AddEnumFilter;

use simulation::{initialize_simulation, tick_sim_clock, update_simulation_state_and_time, simulation_should_step, SimClock, SimulationParams};
use spacecraft::{InitialState, build_spacecraft_entity, do_discrete_update_from_event, DiscreteUpdateEvent};
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

    let config = Config::from_str(spacecraft_config_xml).unwrap();

    // Create new bevy ECS entity for spacecraft
    build_spacecraft_entity(&mut commands, &config, &initial_state);
    commands.spawn(Universe::from_config(config.universe));

    let sim_params = SimulationParams::new(config.simulation, config.gnc.update_rate_hz);
    commands.spawn(SimClock::new((sim_params.get_update_period_secs()) as f32));
    commands.insert_resource(sim_params);
    // commands.insert_resource(config.gnc);
}


pub struct SurveyorPhysicsPlugin;
impl Plugin for SurveyorPhysicsPlugin {
    fn build(&self, app: &mut App) {
        // Split this up into two systems - one that actually builds the ECS and one that
        // initializes the simulation. The latter can be run anytime we trigger a new simulation
        app.add_systems(Startup, build_sim_ecs)
            .add_enum_filter::<Subsystem>()
            .add_event::<DiscreteUpdateEvent>()
            .add_state::<SimulationState>()

            // Run simulation when we are in the `Running` state
            .add_systems(Update,
                tick_sim_clock.run_if(in_state(SimulationState::Running))
            )
            .add_systems(Update,
                (
                    spacecraft::step_spacecraft_model,
                    do_discrete_update_from_event,
                    update_simulation_state_and_time,
                ).chain().run_if(in_state(SimulationState::Running).and_then(simulation_should_step))
            )
            // Pass sensor data to GNC after all the models have been updated
            // and receive actuator events from GNC to be used in next update
            .add_systems(Update,
                (
                    crate::interfaces::imu_event_generator,
                    crate::interfaces::rcs_event_receiver
                ).chain().after(do_discrete_update_from_event)
            )
            // Add a Timer that keeps track of time since the simulation started
            // Run `initialize_simulation` when we enter the `Running` state
            .add_systems(OnEnter(SimulationState::Running),
                initialize_simulation
            );
    }
}
