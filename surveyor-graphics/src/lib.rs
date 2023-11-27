mod planet;
mod camera;
mod lander;

use bevy::{prelude::*};
use big_space::FloatingOriginPlugin;
use camera::*;
use planet::*;
use lander::*;
use smooth_bevy_cameras::{controllers::orbit::OrbitCameraPlugin, LookTransformPlugin};

pub type GridCellType = i64;
pub struct SurveyorGraphicsPlugin;

impl Plugin for SurveyorGraphicsPlugin{
    fn build(&self, app: &mut App)
    {
        app.add_plugins(FloatingOriginPlugin::<GridCellType>::default())
        // Camera plugins
        // .add_plugins(big_space::camera::CameraControllerPlugin::<GridCellType>::default())
        .add_plugins(LookTransformPlugin)
        .insert_resource(ClearColor(Color::BLACK))
        .add_plugins(OrbitCameraPlugin::new(true))
        .add_systems(Startup, spawn_camera)
        .add_systems(Update, camera_input_map)
        // .add_system(sync_camera)

        // Planets
        .add_systems(Startup, setup_planet)

        // Spacecraft
        .add_systems(Startup, spawn_lander);
        // .add_systems(Update, update_lander_pos);
        // app.add_systems(Startup, setup);
    }
}
