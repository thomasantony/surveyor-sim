use bevy_ecs::prelude::*;

// pub struct AttitudeControllerOutput {
//     pub torque_b: Option<na::Vector3<f64>>,
// }

// pub fn update_attitude_controller(){}

#[derive(Debug, Clone, Copy, Component, PartialEq)]
pub enum GuidanceMode {
    Idle,
    Manual,
}

#[derive(Debug, Clone, Component)]
pub enum AttitudeTarget {
    None,
    Attitude(nalgebra::UnitQuaternion<f64>),
    BodyRate(nalgebra::Vector3<f64>),
}
impl Default for AttitudeTarget {
    fn default() -> Self {
        Self::None
    }
}

pub fn update_guidance(query: Query<&GuidanceMode>, attitude_target_query: Query<&mut AttitudeTarget>) {
    let mode = query.single();
    match mode {
        GuidanceMode::Idle => update_idle_mode(attitude_target_query),
        GuidanceMode::Manual => update_manual_mode(attitude_target_query),
    }
}
// TODO: Make it so that it only changes the target when the mode changes
pub fn update_idle_mode(mut attitude_target_query: Query<&mut AttitudeTarget>) {
    let mut attitude_target = attitude_target_query.single_mut();
    *attitude_target = AttitudeTarget::None;
}

pub fn update_manual_mode(mut attitude_target_query: Query<&mut AttitudeTarget>) {
    let mut attitude_target = attitude_target_query.single_mut();
    *attitude_target = AttitudeTarget::None;
    // TODO: Use user input to set the attitude target
    // todo!();
}
