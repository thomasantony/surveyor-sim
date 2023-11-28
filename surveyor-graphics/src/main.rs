use surveyor_graphics::SurveyorGraphicsPlugin;
use bevy::{prelude::*};
use surveyor_physics::SimulationState;
use bevy_debug_text_overlay::{screen_print, OverlayPlugin};

fn show_sim_time(phy_query: Query<& surveyor_physics::SimulationTime>) {
    let time = phy_query.single();
    screen_print!("Sim Time: {:.2}", time.0);
}

fn start_sim(mut set_sim_state: ResMut<NextState<SimulationState>>)
{
    set_sim_state.set(SimulationState::Running);
}


fn main() {
    App::new()
        .add_plugins(DefaultPlugins.build().disable::<TransformPlugin>())
        .add_plugins(OverlayPlugin{ font_size: 32.0, ..Default::default() })
        .add_plugins(SurveyorGraphicsPlugin)
        .add_plugins(surveyor_physics::SurveyorPhysicsPlugin)
        .add_plugins(surveyor_gnc::SurveyorGNC::new())
        .add_systems(Startup, start_sim)
        .add_systems(Update, show_sim_time)
        .run();
}
