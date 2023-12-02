use std::time::Duration;


use crate::{SimulationState, SimulationTime};
use crate::spacecraft::{
    OrbitalDynamics, SpacecraftModel,
};
use crate::universe::{Universe, Observation};
use crate::InitialState;
use hard_xml::XmlRead;
use nalgebra::{Dyn, U13};
use bevy_ecs::prelude::*;
use bevy::time::{Time, Timer, TimerMode};
use surveyor_types::simulation::{SimStoppingCondition, SimulationConfig};


pub fn check_stopping_condition(cond: &SimStoppingCondition, state: &OrbitalDynamics, observation: &Observation) -> bool {
    match cond {
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
        // _ => false,
        // SimStoppingCondition::Custom(f) => f(state, universe),
    }
}


#[derive(Debug, Resource, PartialEq, Default)]
pub struct SimulationParams {
    pub config: SimulationConfig,
    pub dt: f64,
    // Number of simulation steps for each GNC update
    pub num_steps_per_gnc_update: u64,
}
impl SimulationParams {
    pub fn new(config: SimulationConfig, gnc_update_rate_hz: f64) -> Self {
        // Sim update rate must be evenly divisible by GNC update rate
        assert!(config.sim_rate_hz % gnc_update_rate_hz == 0.0);
        // GNC update rate must be less than half the sim update rate
        assert!(gnc_update_rate_hz < config.sim_rate_hz);

        Self {
            dt: 1.0/config.sim_rate_hz,
            num_steps_per_gnc_update: (config.sim_rate_hz / gnc_update_rate_hz) as u64,
            config,
        }
    }
    pub fn get_update_period_secs(&self) -> f64 {
        self.dt / self.config.time_acceleration
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

// A timer that keeps track of the progress of the simulation and also the number of steps so far
#[derive(Component)]
pub struct SimClock {
    timer: Timer,
    pub num_steps: u64,
    pub dt: f32
}
impl SimClock {
    pub fn new(dt: f32) -> Self {
        Self {
            timer: Timer::from_seconds(dt as f32, TimerMode::Repeating),
            dt,
            num_steps: 0,
        }
    }
    pub fn tick(&mut self, delta: Duration) {
        self.timer.tick(delta);
        // Increment the number of steps if the timer has just finished
        if self.timer.just_finished() {
            self.num_steps += 1;
        }
    }
    pub fn reset_timer(&mut self) {
        self.timer.reset();
    }
    pub fn just_finished(&self) -> bool {
        self.timer.just_finished()
    }
    pub fn elapsed_secs(&self) -> f32 {
        self.timer.elapsed().as_secs_f32()
    }
}


// System used to initalize the simulation
pub fn initialize_simulation(mut query: Query<(&SpacecraftModel, &mut OrbitalDynamics, &mut SimulationResults)>,
mut clock_query: Query<&mut SimClock>)
{
    let initial_state: InitialState = InitialState::from_str(include_str!("../initial_state.xml")).unwrap();
    let (_, mut orbital_dynamics, mut sim_results) = query.single_mut();
    *orbital_dynamics = OrbitalDynamics::from_initial_state(&initial_state);
    sim_results.history.clear();

    let sim_clock = clock_query.single_mut();
    sim_clock.into_inner().reset_timer();
}

// System that updates simulation state and the time after stepping the dynamics
pub fn update_simulation_state_and_time(
    sim_params: Res<SimulationParams>,
    mut query: Query<(&mut SimulationTime, &OrbitalDynamics), With<SpacecraftModel>>,
    universe_query: Query<&Universe>,
    mut set_sim_state: ResMut<NextState<SimulationState>>,
) {
    let universe = universe_query.single();
    let obs = Observation::new(&universe);
    // Use query to extract references to the spacecraft model and orbital dynamics inputs
    let (mut t, state) = query.single_mut();
    if sim_params.config.stopping_conditions.iter().any(|c| check_stopping_condition(&c, &state, &obs)) {
        log::info!("Simulation has finished");
        set_sim_state.set(SimulationState::Finished);
        return;
    }
    // log::info!("Sim time: {}", t.0);

    t.0 += sim_params.dt;
}
pub fn tick_sim_clock(time: Res<Time>, mut query: Query<&mut SimClock>) {
    let mut timer = query.single_mut();
    timer.tick(time.delta());
}

// We will step the simulation only when the timer has finished
pub fn simulation_should_step(
    q_timer: Query<&SimClock>,
) -> bool {
    let timer = q_timer.single();
    timer.just_finished()
}
