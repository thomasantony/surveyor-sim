use bevy::{prelude::*, math::DVec3};
use bevy_panorbit_camera::PanOrbitCamera;
use big_space::{FloatingOriginSettings, FloatingOrigin, GridCell};
// use smooth_bevy_cameras::{controllers::orbit::{OrbitCameraBundle, OrbitCameraController, ControlEvent}, LookTransformBundle, LookTransform, Smoother};


use crate::{GridCellType, planet::MOON_RADIUS, lander::{Lander, LanderStateUpdate}};

#[derive(Default, Debug, Component)]
pub struct CameraRelativePosition {
    pub position: Vec3,
}

pub fn spawn_camera(
    mut commands: Commands,
    _lander_query: Query<(&Lander, &mut GridCell<GridCellType>, &mut Transform)>,
    settings: Res<FloatingOriginSettings>,
) {
    let lander_pos = DVec3::new(MOON_RADIUS + 100e3, 0.0, 0.0);
    let (_, lander_translation) = settings.translation_to_grid::<GridCellType>(lander_pos);

    let camera_rel_pos: DVec3 = DVec3::new(10.0, 0.0, 0.0);
    let camera_pos: DVec3 = lander_pos + camera_rel_pos;
    let (grid_cell, camera_translation) = settings.translation_to_grid::<GridCellType>(camera_pos.clone());
    let camera_transform = Transform::from_translation(camera_translation)
                                            .looking_at(lander_translation, Vec3::Y);

    println!("Initially we have {:?} looking at {:?}", camera_translation, lander_translation);
    commands.spawn((
        Camera3dBundle {
            transform: camera_transform.clone(),
            ..default()
        },
        grid_cell,
        FloatingOrigin,
        CameraRelativePosition {
            position: Vec3::new(camera_rel_pos.x as f32, camera_rel_pos.y as f32, camera_rel_pos.z as f32)
        },
        bevy_panorbit_camera::PanOrbitCamera::default(),
    ));
    // .insert(
    //     OrbitCameraBundle::new(
    //     OrbitCameraController{
    //             mouse_rotate_sensitivity: Vec2::splat(0.16),
    //             mouse_wheel_zoom_sensitivity: 0.1,
    //             ..Default::default()
    //         },
    //     camera_translation.clone(),
    //     lander_translation.clone(), Vec3::Y
    // )
    // );
    println!("camera Spawned");
}

// Receives the lander state update event and updates the graphics
pub fn sync_camera(
    mut lander_state: EventReader<LanderStateUpdate>,
    mut camera_query: Query<(&mut Transform, &mut GridCell<GridCellType>, &CameraRelativePosition, &mut PanOrbitCamera), With<Camera>>,
    settings: Res<FloatingOriginSettings> )
{
    if let Some(lander_state) = lander_state.read().last() {
        let (mut camera_transform, mut camera_cell, camera_relpos, _pano) = camera_query.single_mut();

        let lander_pos = lander_state.pos;
        let (lander_cell, lander_translation) = settings.translation_to_grid::<GridCellType>(lander_pos);

        let new_camera_translation: Vec3 = lander_translation + camera_relpos.position;
        *camera_cell = lander_cell;
        camera_transform.translation = new_camera_translation;

        // let pano_delta = lander_translation - pano.target_focus;
        // pano.target_focus = lander_translation;
        // pano.target_radius = 10.0;
        // pano.target_alpha += 0.001;
    }

}
