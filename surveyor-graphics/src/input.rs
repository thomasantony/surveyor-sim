use bevy::prelude::*;
use surveyor_physics::{
    simulation::{SetSimulationRate, SimulationParams},
    SimulationState,
};

pub fn keyboard_input(
    keys: Res<ButtonInput<KeyCode>>,
    sim_state: Res<State<SimulationState>>,
    mut set_sim_state: ResMut<NextState<SimulationState>>,
    sim_params: Res<SimulationParams>,
    mut set_simulation_rate: EventWriter<SetSimulationRate>,
) {
    if keys.just_pressed(KeyCode::KeyP) {
        match sim_state.get() {
            SimulationState::Running => set_sim_state.set(SimulationState::Paused),
            SimulationState::Paused => set_sim_state.set(SimulationState::Running),
            _ => (),
        }
    } else if keys.just_pressed(KeyCode::KeyR) {
        set_sim_state.set(SimulationState::Resetting);
    } else if keys.just_pressed(KeyCode::KeyS) {
        let current_rate = sim_params.config.time_acceleration;
        let new_rate = (current_rate * 2.0).clamp(0.25, 10.0);
        set_simulation_rate.send(SetSimulationRate {
            multiplier: new_rate,
        });
    } else if keys.just_pressed(KeyCode::KeyA) {
        let current_rate = sim_params.config.time_acceleration;
        let new_rate = (current_rate * 0.5).clamp(0.25, 10.0);
        set_simulation_rate.send(SetSimulationRate {
            multiplier: new_rate,
        });
    }
}
