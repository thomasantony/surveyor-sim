mod planet;
mod camera;

use bevy::{prelude::*};
use big_space::FloatingOriginPlugin;
use camera::*;
use planet::*;
use smooth_bevy_cameras::{controllers::orbit::OrbitCameraPlugin, LookTransformPlugin};

pub type GridCellType = i64;
pub struct SurveyorGraphicsPlugin;

impl Plugin for SurveyorGraphicsPlugin{
    fn build(&self, app: &mut App)
    {
        app.add_plugin(FloatingOriginPlugin::<GridCellType>::default())
        // Camera plugins
        .add_plugin(big_space::camera::CameraControllerPlugin::<GridCellType>::default())
        .add_plugin(LookTransformPlugin)
        .insert_resource(ClearColor(Color::BLACK))
        .add_plugin(OrbitCameraPlugin::new(true))
        .add_startup_system(spawn_camera)
        .add_system(camera_input_map)
        // Planets
        .add_startup_system(setup_planet);

        // app.add_startup_system(setup);
    }
}
