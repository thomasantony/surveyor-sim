//! New module for spacecraft model
use std::ops::Range;
use nalgebra as na;
use bevy::prelude::*;
use bevy_ecs::system::Commands;

use crate::spacecraft::{OrbitalDynamics, SpacecraftProperties};;

// Component
pub struct SpacecraftModel {
    // subsystems, orbital_dynamics, ode_state
}

/// A component defining a continuous system that can be stepped over time
pub struct ContinuousSystemState {
    /// Indices of the state vector that this system operates on within a larger state vector
    state_vector: na::DVector<f64>,
    /// Number of states in the system
    num_states: usize,
}

impl ContinuousSystemState {
    /// Create a new continuous system
    pub fn new(initial_state: &[f64]) -> Self {
        let num_states = initial_state.len();
        let state_vector = na::DVector::from_vec(initial_state.to_vec());
        Self {
            state_vector,
            num_states,
        }
    }
}


/// Have a startup system that initializes all the continuous systems
/// Queries for the size of all continuous systems and creates a new strate vector component that
/// holds all the states for all the continuous systems in a single vector

fn build_spacecraft_entity(mut commands: Commands) {
    let orbitdyn_initial_state = [0.0; 13];
    let orbital_dynamics = commands.spawn((OrbitalDynamics::default(), ContinuousSystemState::new(&initial_state))).id();
    let spacecraft_ent = commands.spawn((SpacecraftProperties::default(), SpacecraftModel::default(), OrbitalDynamics::default())).id();
}

// System that steps the spacecraft model over one timestep
fn step_spacecraft_model() {
}

pub struct SpacecraftPlugin;
impl Plugin for SpacecraftPlugin {
    fn build(&self, app: &mut App) {
        app.add_startup_system(build_spacecraft_entity);
    }
}
