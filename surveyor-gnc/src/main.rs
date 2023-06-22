use bevy_app::prelude::*;
use surveyor_gnc::{GNC};

fn main() {
    App::new()
        .add_plugin(GNC)
        .run();
}
