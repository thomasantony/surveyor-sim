use std::time::Duration;

use bevy_app::{prelude::*};
use bevy::{DefaultPlugins};
use bevy_app::ScheduleRunnerPlugin;


pub enum AppState {
    MainMenu,
    RunningSimulation,
    PausedSimulation,
    // EditingSimulation,
    // EditingSpacecraft,
    // EditingUniverse,
    // ShowingResult
}
fn main() {
    let gnc_app_plugin = surveyor_gnc::SurveyorGNC::new();
    let mut app = App::new();
    app.add_plugins(gnc_app_plugin)
        .add_plugins(ScheduleRunnerPlugin::run_loop(Duration::from_secs_f64(
        1.0 / 60.0,
        )))
        .add_plugins(DefaultPlugins)
        .run();
}
