use bevy::prelude::*;
use surveyor_physics::SimulationState;

pub fn keyboard_input(
    keys: Res<Input<KeyCode>>,
    sim_state: Res<State<SimulationState>>,
    mut set_sim_state: ResMut<NextState<SimulationState>>,
) {
    if keys.just_pressed(KeyCode::P) {
        match sim_state.get() {
            SimulationState::Running => set_sim_state.set(SimulationState::Paused),
            SimulationState::Paused => set_sim_state.set(SimulationState::Running),
            _ => (),
        }
    }else if keys.just_pressed(KeyCode::R) {
        set_sim_state.set(SimulationState::Resetting);
    }
}
