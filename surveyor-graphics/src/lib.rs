mod planet;
mod camera;
mod lander;
mod input;

use bevy::prelude::*;
use big_space::FloatingOriginPlugin;
use camera::*;
use planet::*;
use lander::*;
use input::*;

// use smooth_bevy_cameras::{controllers::orbit::OrbitCameraPlugin, LookTransformPlugin};

pub type GridCellType = i64;
pub struct SurveyorGraphicsPlugin;

impl Plugin for SurveyorGraphicsPlugin{
    fn build(&self, app: &mut App)
    {
        app.add_plugins((
            FloatingOriginPlugin::<GridCellType>::default(),
            // big_space::debug::FloatingOriginDebugPlugin::<GridCellType>::default(),

        ))

        .add_systems(Update, (
            camera_inputs,
        ))
        .insert_resource(ClearColor(Color::BLACK))
        .add_systems(Startup, spawn_camera)

        // Planets
        .add_systems(Startup, setup_planet)

        // Spacecraft
        .add_systems(Startup, spawn_lander)
        .add_event::<LanderStateUpdate>()
        .add_systems(Update,
            (
                compute_lander_state_from_simulation,
                render_lander_state,
                render_celbody_position,
                sync_camera
            ).chain().after(surveyor_physics::simulation::update_simulation_state_and_time)
        )
        // inputs
        .add_systems(Update, keyboard_input);
        // app.add_systems(Startup, setup);
    }
}
