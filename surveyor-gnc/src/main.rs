use bevy_app::App;
use surveyor_gnc::SurveyorGNC;

fn main() {
    App::new()
        .add_plugins(SurveyorGNC::new())
        // .add_simple_outer_schedule()
        .run();
}
