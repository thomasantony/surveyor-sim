use std::time::Duration;

use bevy_app::{prelude::*};
use bevy::MinimalPlugins;
use bevy_app::ScheduleRunnerSettings;


fn main() {
    let gnc_app_plugin = surveyor_gnc::SurveyorGNC::new();
    let mut app = App::new();
    app.add_plugin(gnc_app_plugin)
        // .add_simple_outer_schedule()
        .insert_resource(ScheduleRunnerSettings::run_loop(Duration::from_secs_f64(
        1.0 / 60.0,
        )))
        .add_plugins(MinimalPlugins)
        .run();
}
