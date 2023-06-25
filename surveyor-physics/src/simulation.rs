use crate::{SimulationState, SimulationTime};
use crate::integrators::do_rk4_step;
use crate::spacecraft::{
    InitialState, OrbitalDynamics, OrbitalDynamicsInputs, SpacecraftModel, SpacecraftProperties,
};
use crate::subsystems::propulsion::EngineCommands;
use crate::universe::Universe;
use nalgebra::{Dyn, SVectorView, U13};
use bevy_ecs::prelude::*;

// // Enum defining stopping conditions for simulation
// pub enum SimStoppingCondition {
//     Time(f64),
//     Custom(Box<dyn Fn(&OrbitalDynamics) -> bool>),
// }
// impl std::fmt::Debug for SimStoppingCondition {
//     fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
//         match self {
//             SimStoppingCondition::Time(t) => write!(f, "Time({})", t),
//             SimStoppingCondition::Custom(_) => write!(f, "Custom"),
//         }
//     }
// }

// Struct holding parameters for simulation
#[derive(Debug, Resource)]
pub struct SimulationParams {
    pub dt: f64,
    pub t0: f64,
    pub tf: f64,
    pub initial_state: InitialState,
    // pub stopping_condition: Option<SimStoppingCondition>,
}
impl SimulationParams {
    pub fn new(dt: f64, t0: f64, tf: f64, initial_state: InitialState) -> Self {
        Self {
            dt,
            t0,
            tf,
            initial_state,
            // stopping_condition: None,
        }
    }
}
// Implement a sane default for SimulationParams
impl Default for SimulationParams {
    fn default() -> Self {
        Self {
            dt: 1.0 / 100.0,
            t0: 0.0,
            tf: 100.0,
            initial_state: InitialState::default(), // stopping_condition: None,
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

// System holding time history of simulation of spacecraft

pub fn run_simulation_system(
    sim_params: Res<SimulationParams>,
    mut query: Query<(&mut SimulationTime, &mut SpacecraftModel, &SpacecraftProperties, &mut OrbitalDynamicsInputs, &mut SimulationResults)>,
    mut universe: Query<(&mut Universe)>,
    mut set_sim_state: ResMut<NextState<SimulationState>>,
) {
    // let mut t = sim_params.t0;
    // let mut state = sim_params.initial_state.clone();
    // Use query to extract references to the spacecraft model and orbital dynamics inputs
    let (mut t, mut spacecraft_model, sc_props, mut orbital_dynamics_input, mut results) = query.single_mut();

    if t.0 >= sim_params.tf {
        log::info!("Simulation has finished");
        set_sim_state.set(SimulationState::Finished);
        return;
    }
    log::info!("Sim time: {}", t.0);

    let mut universe = universe.single_mut();

    {
        let trajectory = spacecraft_model.get_trajectory();
        let r = SVectorView::from_slice(trajectory.get_position());
        // Apply all gravity model forces to external force
        orbital_dynamics_input.total_torque_b.fill(0.0);
        orbital_dynamics_input.total_force_b = universe.compute_force(&r, &sc_props);
    }

    spacecraft_model.update_discrete(sim_params.dt);
    {
        do_rk4_step(
            sim_params.dt,
            &mut *spacecraft_model,
            &mut (&sc_props, &orbital_dynamics_input),
        );
        results
            .history
            .push(spacecraft_model.get_trajectory().clone());
    }
    t.0 += sim_params.dt;
}
