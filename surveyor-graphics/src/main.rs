use surveyor_graphics::SurveyorGraphicsPlugin;
use bevy::{prelude::*};


fn main() {
    App::new()
        .add_plugins(DefaultPlugins.build().disable::<TransformPlugin>())
        .add_plugin(SurveyorGraphicsPlugin)
        .run();
}
