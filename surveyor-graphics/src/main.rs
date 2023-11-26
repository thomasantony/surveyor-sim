use surveyor_graphics::SurveyorGraphicsPlugin;
use bevy::{prelude::*};
use surveyor_physics::SimulationState;

fn start_sim(mut set_sim_state: ResMut<NextState<SimulationState>>)
{
    set_sim_state.set(SimulationState::Running);
}


fn main() {
    let gnc = surveyor_gnc::SurveyorGNC::new();


    App::new()
        .add_plugins(DefaultPlugins.build().disable::<TransformPlugin>())
        .add_plugin(SurveyorGraphicsPlugin)
        .add_plugin(surveyor_physics::SurveyorPhysicsPlugin)
        .add_plugin(gnc)
        .add_startup_system(start_sim)
        .run();
}
