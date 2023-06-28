use bevy::{prelude::*, math::DVec3};
use big_space::{FloatingOriginSettings, FloatingOrigin};

use crate::planet::MOON_RADIUS;

pub fn spawn_lander(mut commands: Commands,
    asset_server: Res<AssetServer>,
    settings: Res<FloatingOriginSettings>)
{
    let lander_pos = DVec3::new(MOON_RADIUS + 100e3, 0.0, 0.0);
    let (grid_cell, lander_translation) = settings.translation_to_grid::<i128>(lander_pos);
    // in the SceneBundle
    commands.spawn((SceneBundle {
        scene: asset_server.load("Surveyor/Surveyor-Lander.gltf#Scene0"),
        transform: Transform::from_translation(lander_translation.clone()),
        ..default()
    },
    grid_cell,
    Name::new("Lander")));
    println!("Lander Spawned")
}
