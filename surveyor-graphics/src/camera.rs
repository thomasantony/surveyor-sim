use bevy::{prelude::*, math::DVec3, input::mouse::{MouseWheel, MouseMotion, MouseScrollUnit}};
use big_space::{FloatingOriginSettings, FloatingOrigin, camera::CameraController, GridCell, camera::CameraInput};
use smooth_bevy_cameras::{controllers::orbit::{OrbitCameraBundle, OrbitCameraController, ControlEvent}, LookTransformBundle, LookTransform, Smoother};
use surveyor_physics::spacecraft::OrbitalDynamics;

use crate::{GridCellType, planet::MOON_RADIUS, lander::{self, Lander}};

#[derive(Default, Debug, Component)]
pub struct CameraTarget;

pub fn spawn_camera(
    mut commands: Commands,
    mut lander_query: Query<(&Lander, &mut GridCell<GridCellType>, &mut Transform)>,
    settings: Res<FloatingOriginSettings>,
) {
    let lander_pos = DVec3::new(MOON_RADIUS + 100e3, 0.0, 0.0);
    let (_, lander_translation) = settings.translation_to_grid::<GridCellType>(lander_pos);

    let camera_pos = DVec3::new(MOON_RADIUS + 100e3 +10.0, 0.0, 0.0);
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
    ),
    )
    .insert(
        OrbitCameraBundle::new(
        OrbitCameraController{
                mouse_rotate_sensitivity: Vec2::splat(0.16),
                mouse_wheel_zoom_sensitivity: 0.1,
                ..Default::default()
            },
        camera_translation.clone(),
        lander_translation.clone(), Vec3::Y
    ))
    .insert(
        LookTransformBundle {
            transform: LookTransform::new(camera_translation.clone(), lander_translation.clone(), Vec3::Y),
            smoother: Smoother::new(0.9), // Value between 0.0 and 1.0, higher is smoother.
        }
    );
    println!("camera Spawned");
}

pub fn camera_input_map(
    mut events: EventWriter<ControlEvent>,
    mut mouse_wheel_reader: EventReader<MouseWheel>,
    mut mouse_motion_events: EventReader<MouseMotion>,
    mouse_buttons: Res<Input<MouseButton>>,
    keyboard: Res<Input<KeyCode>>,
    controllers: Query<&OrbitCameraController>,
) {
    // Can only control one camera at a time.
    let controller = if let Some(controller) = controllers.iter().find(|c| c.enabled) {
        controller
    } else {
        return;
    };
    let OrbitCameraController {
        mouse_rotate_sensitivity,
        mouse_translate_sensitivity,
        mouse_wheel_zoom_sensitivity,
        pixels_per_line,
        ..
    } = *controller;

    let mut cursor_delta = Vec2::ZERO;
    for event in mouse_motion_events.iter() {
        cursor_delta += event.delta;
    }

    // if keyboard.pressed(KeyCode::LControl) {
    if mouse_buttons.pressed(MouseButton::Left) {
        events.send(ControlEvent::Orbit(mouse_rotate_sensitivity * cursor_delta));
    }

    if mouse_buttons.pressed(MouseButton::Right) {
        events.send(ControlEvent::TranslateTarget(
            mouse_translate_sensitivity * cursor_delta,
        ));
    }

    let mut scalar = 1.0;
    for event in mouse_wheel_reader.iter() {
        // scale the event magnitude per pixel or per line
        let scroll_amount = match event.unit {
            MouseScrollUnit::Line => event.y,
            MouseScrollUnit::Pixel => event.y / pixels_per_line,
        };
        scalar *= 1.0 - scroll_amount * mouse_wheel_zoom_sensitivity;
    }
    events.send(ControlEvent::Zoom(scalar));
}

pub fn sync_camera(
    phy_query: Query<&OrbitalDynamics>,
    mut camera_query: Query<(&mut Transform, &mut GridCell<GridCellType>), With<Camera>>,
    settings: Res<FloatingOriginSettings> ) {

    let lander = phy_query.single();
    let lander_pos = lander.state.rows(0, 3);
    // Offset camera by 10m in the direction of the lander
    use surveyor_physics::math::Vector3;
    // Position camera to be between the lander and the planet
    let (mut look, mut camera_cell) = camera_query.single_mut();

    let (lander_cell, lander_translation) = settings.translation_to_grid::<GridCellType>(DVec3::new(lander_pos[0], lander_pos[1], lander_pos[2]));

    let new_camera_translation = lander_translation + Vec3::new(10.0, 0.0, 0.0);
    *camera_cell = lander_cell;

    // let camera_transform = Transform::from_translation(new_camera_translation).
                                            // .looking_at(lander_translation, Vec3::Y);
    // look.eye = new_camera_translation;
    // look.eye = lander_translation;
    // *look = camera_transform;
    look.translation = new_camera_translation;
}

// pub fn sync_camera(
//     phy_query: Query<(&OrbitalDynamics)>,
//     mut camera_query: Query<(&mut Transform, &mut GridCell<GridCellType>), With<Camera>>,
//     settings: Res<FloatingOriginSettings> ) {

//     let lander = phy_query.single();
//     let lander_pos = lander.state.rows(0, 3);
//     // Offset camera by 10m in the direction of the lander
//     use surveyor_physics::math::Vector3;
//     // Position camera to be between the lander and the planet
//     let (mut camera_transform, mut camera_cell) = camera_query.single_mut();

//     let (lander_cell, lander_translation) = settings.translation_to_grid::<GridCellType>(DVec3::new(lander_pos[0], lander_pos[1], lander_pos[2]));

//     let new_camera_translation = lander_translation + Vec3::new(10.0, 0.0, 0.0);
//     *camera_cell = lander_cell;

//     let delta = new_camera_translation - camera_transform.translation;
//     camera_transform.translation += delta;

// }


// pub fn camera_input_map(
//     mut events: EventWriter<ControlEvent>,
//     mut mouse_wheel_reader: EventReader<MouseWheel>,
//     mut mouse_motion_events: EventReader<MouseMotion>,
//     mut cam: ResMut<CameraInput>,
//     mouse_buttons: Res<Input<MouseButton>>,
//     keyboard: Res<Input<KeyCode>>,
//     controllers: Query<&CameraController>,
// ) {
//     cam.defaults_disabled = true;
// }
