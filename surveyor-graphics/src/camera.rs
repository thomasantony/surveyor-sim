use bevy::{prelude::*, math::DVec3, input::mouse::{MouseWheel, MouseMotion, MouseScrollUnit}};
use big_space::{FloatingOriginSettings, FloatingOrigin, GridCell};


use crate::{GridCellType, planet::MOON_RADIUS, lander::{Lander, LanderStateUpdate}};

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

     commands.spawn((
        Camera3dBundle {
            transform: camera_transform.clone(),
            ..default()
        },
        grid_cell,
        FloatingOrigin,
        FollowCamera::default(),
    ));
    println!("camera Spawned");
}
/// Custom camera controller
#[derive(Component)]
pub struct FollowCamera {
    pub focus: Vec3,
    pub alpha: f32,
    pub beta: f32,
    pub radius: f32,
    pub is_upside_down: bool,
}
impl Default for FollowCamera {
    fn default() -> Self {
        Self {
            focus: Vec3::ZERO,
            is_upside_down: false,
            alpha: 0.0,
            beta: 0.0,
            radius: 10.0,
        }
    }
}

pub fn apply_limits(value: f32, upper_limit: Option<f32>, lower_limit: Option<f32>) -> f32 {
    let mut new_val = value;
    if let Some(zoom_upper) = upper_limit {
        new_val = f32::min(new_val, zoom_upper);
    }
    if let Some(zoom_lower) = lower_limit {
        new_val = f32::max(new_val, zoom_lower);
    }
    new_val
}


/// Pan the camera with middle mouse click, zoom with scroll wheel, orbit with right mouse click.
pub fn camera_inputs(
    time: Res<Time>,
    mut mouse_wheel_reader: EventReader<MouseWheel>,
    mut mouse_motion_events: EventReader<MouseMotion>,
    mouse_buttons: Res<Input<MouseButton>>,
    _keyboard: Res<Input<KeyCode>>,
    input_mouse: Res<Input<MouseButton>>,
    mut cameras: Query<&mut FollowCamera>,
) {
    // change input mapping for orbit and panning here
    let orbit_button = MouseButton::Left;
    let mouse_delta = mouse_motion_events.read().map(|event| event.delta).sum::<Vec2>();

    let mut rotation_move = Vec2::ZERO;
    let mouse_zoom_sensitivity = 0.2;
    let mouse_rotate_sensitivity = Vec2::splat(0.8);
    let mut scroll_line = 0.0;
    let mut scroll_pixel = 0.0;
    let mut orbit_button_changed = false;

    // Can only control one camera at a time.
    let mut camera =
    if let Some(camera) = cameras.iter_mut().next() {
        camera
    } else {
        return;
    };

    if input_mouse.pressed(MouseButton::Left) {
        rotation_move += mouse_delta * mouse_rotate_sensitivity;
    }

    for ev in mouse_wheel_reader.read() {
        let delta_scroll = ev.y * mouse_zoom_sensitivity;
        match ev.unit {
            MouseScrollUnit::Line => {
                scroll_line += delta_scroll;
            }
            MouseScrollUnit::Pixel => {
                scroll_pixel += delta_scroll * 0.005;
            }
        };
    }

    if mouse_buttons.just_pressed(orbit_button)
        || mouse_buttons.just_released(orbit_button)
    {
        orbit_button_changed = true;
    }

    use std::f32::consts::{TAU, PI};
    let dt = time.delta_seconds();
    if  orbit_button_changed {
        let wrapped_beta = (camera.beta % TAU).abs();
        camera.is_upside_down = wrapped_beta > TAU / 4.0 && wrapped_beta < 3.0 * TAU / 4.0;
    }
    if rotation_move.length_squared() > 0.0 {
        let delta_x = {
            // let delta = rotation_move.x / win_size.x * PI * 2.0;
            let delta = rotation_move.x * dt;
            if camera.is_upside_down {
                -delta
            } else {
                delta
            }
        };
        let delta_y = rotation_move.y * dt;
        camera.alpha -= delta_x;
        camera.beta += delta_y;
    } else if (scroll_line + scroll_pixel).abs() > 0.0 {
        // Choose different reference values based on the current projection
        let mut target_value = camera.radius;
        // Calculate the impact of scrolling on the reference value
        let line_delta = -scroll_line * target_value * 0.2;
        let pixel_delta = -scroll_pixel * target_value * 0.2;

        // Update the target value
        target_value += line_delta + pixel_delta;

        // If it is pixel-based scrolling, add it directly to the current value
        target_value += pixel_delta;

        camera.radius = apply_limits(target_value, Some(100.0), Some(0.1));
    }

    // Disallow upside-down
    camera.beta = apply_limits(camera.beta, Some(PI / 2.0), Some(-PI / 2.0));

    // consume any remaining events, so they don't pile up if we don't need them
    // (and also to avoid Bevy warning us about not checking events every frame update)
    mouse_motion_events.clear();
}

// Receives the lander state update event and updates the graphics
pub fn sync_camera(
    mut lander_state: EventReader<LanderStateUpdate>,
    mut camera_query: Query<(&mut Transform, &mut GridCell<GridCellType>, &mut FollowCamera), With<Camera>>,
    settings: Res<FloatingOriginSettings> )
{
    if let Some(lander_state) = lander_state.read().last() {
        let (mut camera_transform, mut camera_cell, camera) = camera_query.single_mut();

        let lander_pos = lander_state.pos;
        let (lander_cell, lander_translation) = settings.translation_to_grid::<GridCellType>(lander_pos);

        // Rotate the position around focus by yaw and pitch.
        let yaw = Quat::from_rotation_y(camera.alpha);
        let pitch = Quat::from_rotation_x(-camera.beta);
        let rotation = yaw * pitch;
        let rel_pos = rotation * Vec3::new(0.0, 0.0, camera.radius);
        let new_camera_translation: Vec3 = lander_translation + rel_pos;
        *camera_cell = lander_cell;
        camera_transform.translation = new_camera_translation;
        camera_transform.look_at(lander_translation, Vec3::Y);

        // let pano_delta = lander_translation - pano.target_focus;
        // pano.target_focus = lander_translation;
        // pano.target_radius = 10.0;
        // pano.target_alpha += 0.001;
    }

}
