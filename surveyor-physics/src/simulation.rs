use std::fmt::Formatter;
use std::ops::{DerefMut, Deref};
use std::str::FromStr;

use crate::{SimulationState, SimulationTime};
use crate::spacecraft::{
    InitialState, OrbitalDynamics, SpacecraftModel,
};
use crate::universe::{Universe, Observation};
use hard_xml::XmlRead;
use nalgebra::{Dyn, U13};
use bevy_ecs::prelude::*;

// Enum defining stopping conditions for simulation
#[derive(Resource, Clone, PartialEq)]
#[derive(XmlRead)]
pub enum SimStoppingCondition {
    #[xml(tag="MaxDuration")]
    MaxDuration(#[xml(text)] f64),
    #[xml(tag="CollisionWith")]
    CollisionWith(#[xml(text)] String),
    // Custom(Box<dyn Fn(&OrbitalDynamics, &Universe) -> bool + Sync + Send + 'static>),
}

#[derive(Debug, Resource, Clone, PartialEq)]
#[derive(XmlRead)]
#[xml(tag="StoppingConditions")]
pub struct StoppingConditionVec(
    #[xml(child="MaxDuration", child="CollisionWith")] pub Vec<SimStoppingCondition>
);
impl Deref for StoppingConditionVec {
    type Target = Vec<SimStoppingCondition>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
impl DerefMut for StoppingConditionVec {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl std::fmt::Debug for SimStoppingCondition {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            SimStoppingCondition::MaxDuration(t) => write!(f, "MaxDuration({})", t),
            SimStoppingCondition::CollisionWith(body) => write!(f, "CollisionWith({})", body),
            // SimStoppingCondition::Custom(_) => write!(f, "Custom"),
        }
    }
}

impl SimStoppingCondition {
    pub fn check(&self, state: &OrbitalDynamics, observation: &Observation) -> bool {
        match self {
            SimStoppingCondition::MaxDuration(t) => {
                state.time >= *t
            },
            SimStoppingCondition::CollisionWith(body) => {
                let body = observation.get_body_by_name(&body).unwrap();
                let sc_pos = state.state.fixed_rows::<3>(0);
                let r = sc_pos - body.position;
                let r_mag = r.norm();
                r_mag < body.radius
            },
            _ => false,
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
    #[xml(child="StoppingConditions")]
    pub stopping_conditions: StoppingConditionVec,
}
impl SimulationParams {
    pub fn new(dt: f64, stopping_conditions: Vec<SimStoppingCondition>) -> Self {
        Self {
            dt,
            stopping_conditions: StoppingConditionVec(stopping_conditions),
        }
    }
}
// Implement a sane default for SimulationParams
impl Default for SimulationParams {
    fn default() -> Self {
        Self {
            dt: 1.0 / 100.0,
            stopping_conditions: StoppingConditionVec(vec![SimStoppingCondition::MaxDuration(100.0)]),
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
    universe_query: Query<&Universe>,
    mut set_sim_state: ResMut<NextState<SimulationState>>,
) {
    let universe = universe_query.single();
    let obs = Observation::new(&universe);
    // Use query to extract references to the spacecraft model and orbital dynamics inputs
    let (mut t, state) = query.single_mut();
    if sim_params.stopping_conditions.iter().any(|c| c.check(&state, &obs)) {
        log::info!("Simulation has finished");
        set_sim_state.set(SimulationState::Finished);
        return;
    }
    // log::info!("Sim time: {}", t.0);

    t.0 += sim_params.dt;
}
