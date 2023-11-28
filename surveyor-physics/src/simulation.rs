use std::fmt::Formatter;
use std::str::FromStr;

use crate::{SimulationState, SimulationTime};
use crate::spacecraft::{
    InitialState, OrbitalDynamics, SpacecraftModel,
};
use crate::universe::Universe;
use hard_xml::XmlRead;
use nalgebra::{Dyn, U13};
use bevy_ecs::prelude::*;

// Enum defining stopping conditions for simulation
#[derive(Resource, Clone, PartialEq)]
#[derive(XmlRead)]
pub enum SimStoppingCondition {
    #[xml(tag="MaxDuration")]
    MaxDuration(#[xml(text)] f64)
    // Custom(Box<dyn Fn(&OrbitalDynamics, &Universe) -> bool + Sync + Send + 'static>),
}

impl FromStr for SimStoppingCondition {
    type Err = &'static str;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s {
            // "Custom" => Ok(SimStoppingCondition::Custom(Box::new(|_, _| false))),
            "MaxDuration" => Ok(SimStoppingCondition::MaxDuration(0.0)),
            _ => Err("Invalid stopping condition"),
        }
    }
}

impl std::fmt::Debug for SimStoppingCondition {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            SimStoppingCondition::MaxDuration(t) => write!(f, "MaxDuration({})", t),
            // SimStoppingCondition::Custom(_) => write!(f, "Custom"),
        }
    }
}

impl SimStoppingCondition {
    pub fn check(&self, state: &OrbitalDynamics, universe: &Universe) -> bool {
        match self {
            SimStoppingCondition::MaxDuration(t) => {
                state.time >= *t
            },
            // SimStoppingCondition::Custom(f) => f(state, universe),
        }
    }
}

// Struct holding parameters for simulation
#[derive(Debug, Resource, PartialEq)]
#[derive(XmlRead)]
#[xml(tag = "SimulationConfig")]
pub struct SimulationParams {
    #[xml(default, flatten_text="StepSize")]
    pub dt: f64,
    #[xml(child="StoppingCondition", child="MaxDuration")]
    pub stopping_conditions: Vec<SimStoppingCondition>,
}
impl SimulationParams {
    pub fn new(dt: f64, stopping_conditions: Vec<SimStoppingCondition>) -> Self {
        Self {
            dt,
            stopping_conditions,
        }
    }
}
// Implement a sane default for SimulationParams
impl Default for SimulationParams {
    fn default() -> Self {
        Self {
            dt: 1.0 / 100.0,
            stopping_conditions: vec![SimStoppingCondition::MaxDuration(100.0)],
        }
    }
}

#[derive(Component, Debug, Default)]
pub struct SimulationResults {
    pub history: Vec<OrbitalDynamics>,
}

type StateHistoryStorage = nalgebra::VecStorage<f64, U13, Dyn>;
type StateHistory = nalgebra::Matrix<f64, U13, Dyn, StateHistoryStorage>;

impl SimulationResults {
    // Return the state history as a DMatrix with 13 rows
    pub fn get_state_history(&self) -> StateHistory {
        let mut state_history = StateHistory::zeros(self.history.len());
        for (i, state) in self.history.iter().enumerate() {
            state_history.column_mut(i).copy_from(&state.state);
        }
        state_history
    }
    pub fn get_time_history(&self) -> Vec<f64> {
        self.history.iter().map(|state| state.time).collect()
    }
}

// System that updates simulation state and the time after stepping the dynamics
pub fn run_simulation_system(
    sim_params: Res<SimulationParams>,
    mut query: Query<(&mut SimulationTime, &OrbitalDynamics), With<SpacecraftModel>>,
    mut set_sim_state: ResMut<NextState<SimulationState>>,
) {
    // Use query to extract references to the spacecraft model and orbital dynamics inputs
    let (mut t, state) = query.single_mut();
    if sim_params.stopping_conditions.iter().any(|c| c.check(&state, &Universe::default())) {
        log::info!("Simulation has finished");
        set_sim_state.set(SimulationState::Finished);
        return;
    }
    // log::info!("Sim time: {}", t.0);

    t.0 += sim_params.dt;
}
