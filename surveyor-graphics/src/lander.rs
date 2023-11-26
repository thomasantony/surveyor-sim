use bevy::{prelude::*, math::DVec3};
use big_space::{FloatingOriginSettings, FloatingOrigin, GridCell};
use surveyor_physics::{SimulationTime, spacecraft::{SpacecraftModel, OrbitalDynamics}};

use crate::planet::MOON_RADIUS;

#[derive(Component)]
pub struct Lander;

pub fn spawn_lander(mut commands: Commands,
    asset_server: Res<AssetServer>,
    settings: Res<FloatingOriginSettings>)
{
    let lander_pos = DVec3::new(MOON_RADIUS + 100e3, 0.0, 0.0);
    println!("Lander position: {:?}", lander_pos);
    let (grid_cell, lander_translation) = settings.translation_to_grid::<i128>(lander_pos);
    // in the SceneBundle
    commands.spawn((SceneBundle {
        scene: asset_server.load("Surveyor/Surveyor-Lander.gltf#Scene0"),
        transform: Transform::from_translation(lander_translation.clone()),
        ..default()
    },
    grid_cell,
    Name::new("Lander"),
    Lander,));
    println!("Lander Spawned")
}

pub fn update_lander_pos(
    mut phy_query: Query<(&mut SimulationTime, &mut OrbitalDynamics)>,
    mut gfx_query: Query<(&Lander, &mut GridCell<i128>, &mut Transform)>,
    settings: Res<FloatingOriginSettings>
)
{
    // Get gfx component from query
    let (_, mut grid_cell, mut transform) = gfx_query.single_mut();
    let (_, sc) = phy_query.single_mut();


    // let lander_pos = sc.state.rows(0, 3);
    // let lander_pos: DVec3 = DVec3::new(lander_pos[0], lander_pos[1], lander_pos[2]);
    // let (new_grid_cell, new_translation) = settings.translation_to_grid::<i128>(lander_pos);
    // if new_grid_cell != *grid_cell {
    //     *grid_cell = new_grid_cell;
    //     transform.translation = new_translation;
    // }

    // println!("Lander position: {:?}", lander_pos);
    let q = sc.state.fixed_rows::<4>(6);
    let lander_quat: Quat = Quat::from_xyzw(q[0] as f32, q[1] as f32, q[2] as f32, q[3] as f32);
    transform.rotation = lander_quat;
}
