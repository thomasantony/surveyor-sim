mod planet;
mod camera;
mod lander;

use bevy::{prelude::*};
use bevy_panorbit_camera::PanOrbitCameraPlugin;
use big_space::FloatingOriginPlugin;
use camera::*;
use planet::*;
use lander::*;

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
        // .add_plugins(big_space::camera::CameraControllerPlugin::<GridCellType>::default())

        // Camera plugins
        // .add_plugins(LookTransformPlugin)
        .add_plugins(PanOrbitCameraPlugin)
        .insert_resource(ClearColor(Color::BLACK))
        // .add_plugins(OrbitCameraPlugin::new(true))
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
                sync_camera
            ).chain().after(surveyor_physics::simulation::update_simulation_state_and_time)
        );
        // app.add_systems(Startup, setup);
    }
}
