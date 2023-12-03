
use surveyor_physics::SimulationState;
use surveyor_types::config::Config;


// use bevy::prelude::*;
use bevy_ecs::prelude::*;
use hard_xml::XmlRead;
// use bevy_app::IntoSystemAppConfig;


#[cfg(target_arch = "wasm32")]
pub fn main() {
    use bevy::MinimalPlugins;
    use surveyor_physics::{simulation::run_simulation_system, visualization::run_plotting_system};

    console_error_panic_hook::set_once();
    console_log::init_with_level(log::Level::Debug).expect("console_log::init_with_level");

    // Create new centered div element called "plotly_box"
    let document = web_sys::window()
        .expect("window")
        .document()
        .expect("document");
    let body = document.body().expect("body");
    let div = document
        .create_element("div")
        .unwrap();
    div.set_id("plotly_box");
    div.set_attribute("style", "position: absolute; top: 0; left: 0; right: 0; bottom: 0; margin: auto; width: 50%; height: 50%;")
        .expect("setting div style");
    body.append_child(&div)
        .unwrap();

    let gnc = surveyor_gnc::SurveyorGNC::new();
    App::new()
        .add_plugins(surveyor_physics::SurveyorPhysicsPlugin)
        .add_plugins(gnc)
        .add_system(run_plotting_system.in_schedule(OnExit(surveyor_physics::SimulationState::Running)))
        .add_plugins(MinimalPlugins)
        .add_startup_system(start_sim)
        .run();
}

fn start_sim(mut set_sim_state: ResMut<NextState<SimulationState>>)
{
    set_sim_state.set(SimulationState::Running);
}

#[cfg(not(target_arch = "wasm32"))]
pub fn main() {
    // Parse spacecraft config and initialize spacecraft model object
    let spacecraft_config_xml = include_str!("../simulation.xml");
    let spacecraft_model = Config::from_str(spacecraft_config_xml).unwrap();
    println!("{:?}\n", spacecraft_model);

    let initial_state_xml = include_str!("../initial_state.xml");
    let initial_state = surveyor_physics::spacecraft::InitialState::from_str(initial_state_xml).unwrap();
    println!("{:?}", initial_state);
}
