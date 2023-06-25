use bevy_app::App;
use surveyor_physics::config::SimulationConfig;
use surveyor_physics::universe::Universe;
use nalgebra::SMatrix;
use std::f64;
use std::f64::consts::PI;

use surveyor_physics::simulation::{SimulationParams, SimulationResults};
use surveyor_physics::spacecraft::{OrbitalDynamicsInputs, SpacecraftModel, SpacecraftProperties, InitialState};

// use bevy::prelude::*;
use bevy_ecs::prelude::*;
use hard_xml::XmlRead;



#[cfg(target_arch = "wasm32")]
pub fn main() {
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

    App::new()
        .add_plugin(SurveyorPhysicsPlugin)
        .set_runner(my_runner)
        .run();
}

fn my_runner(mut app: App) {
    'mainloop: loop {
        // first handle inputs (which we have none of)
        // then update ECS systems
        // Check if we have finished the simulation
        app.update();
        let ecs = &mut app.world;
        let tf = ecs.get_resource::<SimulationParams>().unwrap().tf;
        let sim_result = ecs.query::<&SimulationResults>().single(&ecs);

        if sim_result.history[sim_result.history.len() - 1].time >= tf {
            break 'mainloop;
        }

        // Log current time
        let time = sim_result.history[sim_result.history.len() - 1].time;
        log::info!("Current time: {}", time);
    }
}
#[cfg(not(target_arch = "wasm32"))]
pub fn main() {
    // Parse spacecraft config and initialize spacecraft model object

    let spacecraft_config_xml = include_str!("../simulation.xml");
    let spacecraft_model = SimulationConfig::from_str(spacecraft_config_xml).unwrap();
    println!("{:?}\n", spacecraft_model);

    let initial_state_xml = include_str!("../initial_state.xml");
    let initial_state = surveyor_physics::spacecraft::InitialState::from_str(initial_state_xml).unwrap();
    println!("{:?}", initial_state);
}
