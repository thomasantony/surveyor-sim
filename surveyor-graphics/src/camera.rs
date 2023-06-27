use bevy::{prelude::*, math::DVec3, input::mouse::{MouseWheel, MouseMotion, MouseScrollUnit}};
use big_space::{FloatingOriginSettings, FloatingOrigin};
use smooth_bevy_cameras::{controllers::orbit::{OrbitCameraBundle, OrbitCameraController, ControlEvent}};

use crate::GridCellType;


pub fn spawn_camera(
    mut commands: Commands,
    settings: Res<FloatingOriginSettings>,
) {
    // commands.spawn((
    //     Camera3dBundle {
    //         transform:camera_transform.clone(),
    //         ..default()
    //     },
    //     grid_cell,
    //     FloatingOrigin, // Important: marks this as the entity to use as the floating origin
    //     CameraController::default().with_max_speed(1e35), // Built-in camera controller
    // ));
    let earth_pos = DVec3::new(386000e3, 0.0, 0.0);
    let (grid_cell, earth_translation) = settings.translation_to_grid::<GridCellType>(earth_pos);

    let camera_pos = DVec3::new(1737e3 + 100e3 +10.0, 0.0, 0.0);
    let (grid_cell, camera_translation) = settings.translation_to_grid::<GridCellType>(camera_pos.clone());
    let camera_transform = Transform::from_translation(camera_translation)
                                            .looking_at(earth_translation, Vec3::Y);

    commands
        .spawn((
            Camera3dBundle {
            transform: camera_transform.clone(),
            ..default()
        },
        grid_cell,
        ), // Important: marks this as the entity to use as the floating origin
        )
        .insert(OrbitCameraBundle::new(
        OrbitCameraController{
                    mouse_rotate_sensitivity: Vec2::splat(0.16),
                    mouse_wheel_zoom_sensitivity: 0.01,
                    ..Default::default()
            },
        camera_translation.clone(),
        earth_translation.clone(), Vec3::Y),
    );


    println!("Scene Spawned");

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
