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

use std::f64::consts::PI;

use bevy_app::prelude::*;
use bevy_ecs::system::Commands;
use config::SimulationConfig;
use nalgebra::SMatrix;
use simulation::{SimulationResults, SimulationParams, run_simulation_system};
use spacecraft::{InitialState, SpacecraftModel, OrbitalDynamicsInputs, SpacecraftProperties};
use hard_xml::XmlRead;
use universe::Universe;

// Hardocde the timestep for now
pub const DT: f64 = 0.1; // s

pub fn build_sim_ecs(mut commands: Commands)
{
    // Orbit with: a = 500 km, 0 degree inclination, 0 degree RAAN, 0 degree argument of perigee, 0 degree true anomaly
    let initial_state = InitialState::from_str(include_str!("../initial_state.xml")).unwrap();
    // Create new spacecraft with engine subsystem
    let spacecraft_config_xml = include_str!("../simulation.xml");

    let sim_config = SimulationConfig::from_str(spacecraft_config_xml).unwrap();
    let spacecraft_model =
        SpacecraftModel::from_config(sim_config.spacecraft, initial_state.clone());

    // Create new bevy ECS entity for spacecraft
    commands.spawn(
        (OrbitalDynamicsInputs::new(),
        SpacecraftProperties::new(
            1.0,
            SMatrix::from_vec(vec![1., 0., 0., 0., 1., 0., 0., 0., 1.]),
        ),
        spacecraft_model,
        SimulationResults::default())
    );
    let universe = Universe::from_config(sim_config.universe);
    commands.spawn(universe);

    let a: f64 = 6378.14 + 500.0;
    // let period = 2.0 * PI * (a.powi(3) / 398600.0).sqrt();
    let period = 0.1 * PI * (a.powi(3) / 398600.0).sqrt();
    let sim_params: SimulationParams = SimulationParams::new(DT, 0.0, period, initial_state);
    commands.insert_resource(sim_params);

}
pub struct SurveyorPhysicsPlugin;
impl Plugin for SurveyorPhysicsPlugin {
    fn build(&self, app: &mut App) {
        // Split this up into two systems - one that actually builds the ECS and one that
        // initializes the simulation. The latter can be run anytime we trigger a new simulation
        app.add_startup_system(build_sim_ecs)
            .add_system(run_simulation_system);
    }
}
