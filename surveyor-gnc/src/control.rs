use bevy_ecs::prelude::*;
use nalgebra as na;

use crate::guidance::AttitudeTarget;

#[derive(Debug, Component, Default)]
pub struct ControlAllocator{
    pub use_diff_thrust: bool,
    pub use_rcs: bool,
    pub use_tvc: bool,
}

#[derive(Debug, Component)]
pub struct RCSController {
    /// Pseudo-Inverse of distribution matrix for the RCS thrusters in body frame
    /// Each column of the distribution matrix is a thruster direction
    /// Maps a N-dimensional thrust vector to a 3-dimensional torque vector
    /// The inverse maps a 3-dimensional torque vector to a N-dimensional thrust vector
    pub distribution_matrix_inv: na::MatrixXx3<f64>,
    /// Maximum thrust of each RCS thruster in Newtons
    pub max_thrusts: Vec<f64>,
}

impl Default for RCSController {
    fn default() -> Self {
        // TODO: Get rid of this and configure using a config struct
        Self {
            distribution_matrix_inv: na::MatrixXx3::zeros(3),
            max_thrusts: Vec::new(),
        }
    }
}

#[derive(Debug)]
pub struct RCSControllerInput {
    /// Torque vector in body frame
    pub torque_b: na::Vector3<f64>,
}

impl Default for RCSControllerInput {
    fn default() -> Self {
        Self {
            torque_b: na::Vector3::zeros(),
        }
    }
}

/// Contains the duty cycles of each RCS thruster
#[derive(Debug, Component, Clone)]
pub struct RCSControllerOutput {
    /// Thrust vector in body frame
    pub duty_cycles: Vec<f64>,
}

impl Default for RCSControllerOutput {
    fn default() -> Self {
        Self {
            duty_cycles: Vec::new(),
        }
    }
}

#[derive(Debug)]
pub struct AttitudeTorqueRequest {
    pub torque_b: na::Vector3<f64>,
}

impl Default for AttitudeTorqueRequest {
    fn default() -> Self {
        Self {
            torque_b: na::Vector3::zeros(),
        }
    }
}

pub fn update_attitude_controller(
    mut attitude_target_reader: EventReader<AttitudeTarget>,
    mut torque_request_query: EventWriter<AttitudeTorqueRequest>
)
{
    let attitude_target = attitude_target_reader.iter().last().unwrap_or(&AttitudeTarget::None);
    let torque_request = match attitude_target {
        AttitudeTarget::None => {
            AttitudeTorqueRequest::default()
        },
        AttitudeTarget::Attitude(_q_i2b) => {
            // Implement quaternion feedback control
            todo!("Not implemented yet");
        }
        AttitudeTarget::BodyRate(_omega_b) => {
            // Implement body rate feedback control
            todo!("Not implemented yet");
        }
    };
    torque_request_query.send(torque_request);
}

pub fn update_control_allocator(
    control_allocator_query: Query<&ControlAllocator>,
    mut torque_request_reader: EventReader<AttitudeTorqueRequest>,
    mut rcs_torque_request_writer: EventWriter<RCSControllerInput>,
)
{
    // The control allocator gets configured elsewhere
    let control_allocator = control_allocator_query.single();
    torque_request_reader.iter().last().map(
        |torque_request|{
            if control_allocator.use_rcs {
                // Pass through the torque request to the RCS controller
                let rcs_torque_request = RCSControllerInput {
                    torque_b: torque_request.torque_b,
                };
                rcs_torque_request_writer.send(rcs_torque_request);
            }
            if control_allocator.use_tvc {
                // Pass through the torque request to the TVC controller
                todo!("Not implemented yet");
            }
            if control_allocator.use_diff_thrust {
                // Pass through the torque request to the differential thrust allocator
                todo!("Not implemented yet");
            }

        }
    );

}

pub fn update_rcs_controller(
    mut rcs_controller_input_reader: EventReader<RCSControllerInput>,
    mut query: Query<(&RCSController, &mut RCSControllerOutput)>,
) {
    let (rcs_controller, mut output) = query.single_mut();
    if let Some(rcs_controller_input) = rcs_controller_input_reader.iter().last()
    {
        let torque_rcs = rcs_controller.distribution_matrix_inv.clone() * rcs_controller_input.torque_b;
        // Compute duty cycles and assign them to the output
        let mut duty_cycles = Vec::new();
        for (thrust, max_thrust) in torque_rcs.iter().zip(rcs_controller.max_thrusts.iter()) {
            // Clip duty cycles to [0, 1]
            let duty_cycle = (thrust.abs() / max_thrust).clamp(0., 1.);
            duty_cycles.push(duty_cycle);
        }
        output.duty_cycles = duty_cycles;
    }
}
