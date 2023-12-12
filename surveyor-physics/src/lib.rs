// pub mod plot;
pub mod integrators;
pub mod simulation;
pub mod spacecraft;
pub mod universe;
// #[cfg(target_arch = "wasm32")]
// pub mod visualization;

pub mod models;
pub mod subsystems;

use hifitime::Epoch;
pub use surveyor_types::math;
use surveyor_types::config::Config;
pub mod interfaces;


use bevy_app::prelude::*;
use bevy_ecs::prelude::*;
use bevy_enum_filter::prelude::AddEnumFilter;
use bevy::{prelude::AssetApp, asset::Assets};

use simulation::*;
use spacecraft::{InitialState, build_spacecraft_entity, do_discrete_update_from_event, DiscreteUpdateEvent};
use hard_xml::XmlRead;
use subsystems::Subsystem;
use universe::{Universe, Ephemerides, update_universe};
use bevy_ecs::schedule::IntoSystemConfigs;
use bevy::asset::AssetServer;

// Hardocde the timestep for now
pub const DT: f64 = 0.1; // s

#[derive(Debug, Clone, Eq, PartialEq, Hash, Default, States)]
pub enum SimulationState
{
    #[default]
    Paused,
    Running,
    Finished,
    Resetting,
}

#[derive(Debug, Clone, Copy, Component, Default)]
pub struct SimulationTime
{
    start_time: Epoch,
    pub time: Epoch,
}

impl SimulationTime {
    pub fn new(start_time: Epoch) -> Self {
        Self {
            start_time,
            time: start_time,
        }
    }
    pub fn reset(&mut self) {
        self.time = self.start_time;
    }
    pub fn now(&self) -> Epoch {
        self.time
    }
    pub fn get_monotonic_time(&self) -> f64 {
        (self.time - self.start_time).to_seconds()
    }
    pub fn get_unix_time_s(&self) -> f64 {
        self.time.to_unix(hifitime::Unit::Second)
    }
    pub fn get_unix_time_ms(&self) -> f64 {
        self.time.to_unix(hifitime::Unit::Millisecond)
    }
}


pub fn build_sim_ecs(mut commands: Commands, server: Res<AssetServer>, eph_loader: Res<Assets<Ephemerides>>)
{
    // Orbit with: a = 500 km, 0 degree inclination, 0 degree RAAN, 0 degree argument of perigee, 0 degree true anomaly
    let initial_state: InitialState = InitialState::from_str(include_str!("../initial_state.xml")).unwrap();
    // Create new spacecraft with engine subsystem
    let spacecraft_config_xml = include_str!("../simulation.xml");

    let config = Config::from_str(spacecraft_config_xml).unwrap();

    // Create new bevy ECS entity for spacecraft
    build_spacecraft_entity(&mut commands, &config, &initial_state);
    commands.spawn(Universe::from_config(config.universe, &server, &eph_loader));

    let sim_params = SimulationParams::new(config.simulation, config.gnc.update_rate_hz);
    commands.spawn(SimClock::new((sim_params.get_update_period_secs()) as f32));
    commands.insert_resource(sim_params);

    commands.insert_resource(initial_state);
}


pub struct SurveyorPhysicsPlugin;
impl Plugin for SurveyorPhysicsPlugin {
    fn build(&self, app: &mut App) {
        // Split this up into two systems - one that actually builds the ECS and one that
        // initializes the simulation. The latter can be run anytime we trigger a new simulation
        app.add_systems(Startup, build_sim_ecs)
            .init_asset::<crate::universe::Ephemerides>()
            .init_asset_loader::<crate::universe::AlmanacLoader>()
            .add_enum_filter::<Subsystem>()
            .add_event::<DiscreteUpdateEvent>()
            .add_state::<SimulationState>()
            .add_event::<SetSimulationRate>()

            // Run simulation when we are in the `Running` state
            .add_systems(Update,
                (tick_sim_clock.run_if(in_state(SimulationState::Running)),
                        set_simulation_rate)
            )
            .add_systems(Update,
                (
                    crate::universe::update_universe,
                    spacecraft::step_spacecraft_model,
                    do_discrete_update_from_event,
                    update_simulation_state_and_time,
                ).chain().run_if(in_state(SimulationState::Running).and_then(simulation_should_step))
            )
            // Pass sensor data to GNC after all the models have been updated
            // and receive actuator events from GNC to be used in next update
            .add_systems(Update,
                (
                    crate::interfaces::time_event_generator,
                    crate::interfaces::imu_event_generator,
                    crate::interfaces::star_tracker_event_generator,
                    crate::interfaces::rcs_event_receiver
                ).chain().after(do_discrete_update_from_event)
            )
            // Add a Timer that keeps track of time since the simulation started
            // Run `initialize_simulation` when we enter the `Running` state
            .add_systems(OnEnter(SimulationState::Running),
                initialize_simulation
            )
            // Reset the simulation to the initial state when we enter the `Resetting` state
            .add_systems(OnEnter(SimulationState::Resetting),
                reset_simulation
            );
    }
}
