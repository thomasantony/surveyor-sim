use surveyor_graphics::SurveyorGraphicsPlugin;
use bevy::{prelude::*};
use surveyor_physics::SimulationState;

fn start_sim(mut set_sim_state: ResMut<NextState<SimulationState>>)
{
    set_sim_state.set(SimulationState::Running);
}


fn main() {
    App::new()
        .add_plugins(DefaultPlugins.build().disable::<TransformPlugin>())
        .add_plugins(SurveyorGraphicsPlugin)
        .add_plugins(surveyor_physics::SurveyorPhysicsPlugin)
        .add_plugins(surveyor_gnc::SurveyorGNC::new())
        .add_systems(Startup, start_sim)
        .run();
}
