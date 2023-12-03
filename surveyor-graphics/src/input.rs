use bevy::prelude::*;
use surveyor_physics::{SimulationState, simulation::{SetSimulationRate, SimulationParams}};

pub fn keyboard_input(
    keys: Res<Input<KeyCode>>,
    sim_state: Res<State<SimulationState>>,
    mut set_sim_state: ResMut<NextState<SimulationState>>,
    sim_params: Res<SimulationParams>,
    mut set_simulation_rate: EventWriter<SetSimulationRate>,
) {
    if keys.just_pressed(KeyCode::P) {
        match sim_state.get() {
            SimulationState::Running => set_sim_state.set(SimulationState::Paused),
            SimulationState::Paused => set_sim_state.set(SimulationState::Running),
            _ => (),
        }
    }else if keys.just_pressed(KeyCode::R) {
        set_sim_state.set(SimulationState::Resetting);
    }else if keys.just_pressed(KeyCode::S) {
        let current_rate = sim_params.config.time_acceleration;
        let new_rate = (current_rate * 2.0).clamp(0.25, 10.0);
        set_simulation_rate.send(SetSimulationRate{multiplier: new_rate});
    }else if keys.just_pressed(KeyCode::A) {
        let current_rate = sim_params.config.time_acceleration;
        let new_rate = (current_rate * 0.5).clamp(0.25, 10.0);
        set_simulation_rate.send(SetSimulationRate{multiplier: new_rate});
    }
}
