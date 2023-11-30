use bevy::{prelude::*, math::DVec3};
use big_space::{FloatingOriginSettings, GridCell};
use surveyor_physics::{SimulationTime, spacecraft::{OrbitalDynamics, SpacecraftModel}, simulation::SimClock};

use crate::planet::MOON_RADIUS;
use crate::GridCellType;

// This event contains the internal state of the lander computed  by "update_lander_state_from_simulation"
// This will be used by downstream systems to update the graphics and camera
#[derive(Event)]
pub struct LanderStateUpdate{
    pub pos: DVec3,
    pub vel: DVec3,
    pub quat: Quat,
}
#[derive(Component)]
pub struct Lander;

pub fn spawn_lander(mut commands: Commands,
    asset_server: Res<AssetServer>,
    settings: Res<FloatingOriginSettings>)
{
    let lander_pos = DVec3::new(MOON_RADIUS + 100e3, 0.0, 0.0);
    let (grid_cell, lander_translation) = settings.translation_to_grid::<GridCellType>(lander_pos);
    // in the SceneBundle
    commands.spawn(
        (
            SceneBundle {
                scene: asset_server.load("Surveyor/Surveyor-Lander.gltf#Scene0"),
                transform: Transform::from_translation(lander_translation.clone()),
                ..default()
            },
            grid_cell,
            Name::new("Lander"),
            Lander,
        )
    );
    println!("Lander Spawned")
}

// Computes the internal state of the lander from the simulation state
// and sends it as an event
pub fn compute_lander_state_from_simulation(
    phy_query: Query<(& SimulationTime, & OrbitalDynamics), With<SpacecraftModel>>,
    clock_query: Query<& SimClock>,
    mut lander_state_event_writer: EventWriter<LanderStateUpdate>,
)
{
    let sim_clock = clock_query.single();
    let (sim_time, sc) = phy_query.single();
    if (sim_time.0 as f32) <= sim_clock.dt || sim_clock.just_finished() {
        return;
    }
    let sub_step =  sim_clock.elapsed_secs()/sim_clock.dt;
    let lander_pos_prev = sc.prev_state.rows(0, 3);
    let lander_pos_next = sc.state.rows(0, 3);
    let lander_pos = lander_pos_prev.lerp(&lander_pos_next, sub_step as f64);
    let lander_pos = DVec3::new(lander_pos[0], lander_pos[1], lander_pos[2]);

    let q_prev = sc.prev_state.fixed_rows::<4>(6);
    let q_next = sc.state.fixed_rows::<4>(6);
    let lander_quat_prev = Quat::from_xyzw(q_prev[1] as f32, q_prev[2] as f32, q_prev[3] as f32, q_prev[0] as f32);
    let lander_quat_next = Quat::from_xyzw(q_next[1] as f32, q_next[2] as f32, q_next[3] as f32, q_next[0] as f32);
    let lander_quat = lander_quat_prev.slerp(lander_quat_next, sub_step as f32);

    let lander_vel_prev = sc.prev_state.fixed_rows::<3>(3);
    let lander_vel_next = sc.state.fixed_rows::<3>(3);
    let lander_vel = lander_vel_prev.lerp(&lander_vel_next, sub_step as f64);
    let lander_vel = DVec3::new(lander_vel[0], lander_vel[1], lander_vel[2]);

    let lander_state_update = LanderStateUpdate{
        pos: lander_pos,
        vel: lander_vel,
        quat: lander_quat,
    };
    lander_state_event_writer.send(lander_state_update);

}

// Receives the lander state update event and updates the graphics
pub fn render_lander_state(mut lander_state: EventReader<LanderStateUpdate>,
    mut lander_query: Query<(&mut GridCell<GridCellType>, &mut Transform), With<Lander>>,
    settings: Res<FloatingOriginSettings>
) {
    if let Some(lander_state) = lander_state.read().last()
    {
        // Get gfx component
        let (mut grid_cell, mut transform) = lander_query.single_mut();
        let (new_grid_cell, new_translation) = settings.translation_to_grid::<GridCellType>(lander_state.pos);

        if new_grid_cell != *grid_cell {
            *grid_cell = new_grid_cell;
        }
        transform.translation = new_translation;
        transform.rotation = lander_state.quat;
    }
}
