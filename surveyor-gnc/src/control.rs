

use bevy_ecs::prelude::*;
use nalgebra as na;
use surveyor_types::config::ThrusterConfig;

use crate::{guidance::AttitudeTarget};

#[derive(Debug, Component)]
pub struct ControlAllocator{
    pub use_diff_thrust: bool,
    pub use_rcs: bool,
    pub use_tvc: bool,
}
impl Default for ControlAllocator {
    fn default() -> Self {
        Self {
            use_diff_thrust: false,
            use_rcs: true,
            use_tvc: false,
        }
    }
}

/* RCS */

/// RCS Controller
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
            distribution_matrix_inv: na::MatrixXx3::identity(3),
            max_thrusts: vec![1.0, 1.0, 1.0]
        }
    }
}

#[derive(Debug, Event)]
pub struct RCSTorqueRequest {
    /// Torque vector in body frame
    pub torque_b: na::Vector3<f64>,
}

impl Default for RCSTorqueRequest {
    fn default() -> Self {
        Self {
            torque_b: na::Vector3::zeros(),
        }
    }
}

/// Contains the duty cycles of each RCS thruster
#[derive(Debug, Component, Clone, Event)]
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

#[derive(Debug, Event)]
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
    let attitude_target = attitude_target_reader.read().last().unwrap_or(&AttitudeTarget::None);
    let torque_request = match attitude_target {
        AttitudeTarget::None => {
            AttitudeTorqueRequest::default()
        },
        AttitudeTarget::Attitude(_q_i2b) => {
            // Implement quaternion feedback control
            AttitudeTorqueRequest::default()
            // todo!("Not implemented yet");
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
    mut rcs_torque_request_writer: EventWriter<RCSTorqueRequest>,
    mut tvc_torque_request_writer: EventWriter<TVCTorqueRequest>,
)
{
    // The control allocator gets configured elsewhere
    let control_allocator = control_allocator_query.single();
    torque_request_reader.read().last().map(
        |torque_request|{
            if control_allocator.use_rcs {
                // Pass through the torque request to the RCS controller
                let rcs_torque_request = RCSTorqueRequest {
                    torque_b: torque_request.torque_b,
                };
                rcs_torque_request_writer.send(rcs_torque_request);
            }
            if control_allocator.use_tvc {
                // Pass through the torque request to the TVC controller
                let tvc_torque_request = TVCTorqueRequest {
                    torque_b: torque_request.torque_b,
                };
                tvc_torque_request_writer.send(tvc_torque_request);
            }
            if control_allocator.use_diff_thrust {
                // Pass through the torque request to the differential thrust allocator
                todo!("Not implemented yet");
            }
        }
    );

}

pub fn update_rcs_controller(
    mut rcs_controller_input_reader: EventReader<RCSTorqueRequest>,
    mut query: Query<(&RCSController, &mut RCSControllerOutput)>,
    mut rcs_output_writer: EventWriter<RCSControllerOutput>,
) {
    let (rcs_controller, mut output) = query.single_mut();
    if let Some(rcs_controller_input) = rcs_controller_input_reader.read().last()
    {
        let torque_rcs = rcs_controller.distribution_matrix_inv.clone() * rcs_controller_input.torque_b;
        // Compute duty cycles and assign them to the output
        let mut duty_cycles = Vec::new();
        for (thrust, max_thrust) in torque_rcs.iter().zip(rcs_controller.max_thrusts.iter()) {
            // Clip duty cycles to [0, 1]
            let duty_cycle = (thrust.abs() / max_thrust).clamp(0., 1.);
            duty_cycles.push(duty_cycle);
        }
        duty_cycles[0] = 0.01;
        output.duty_cycles = duty_cycles;
        rcs_output_writer.send(output.clone());
    }
}

/* Vernier Engines and TVC */
#[derive(Debug, Component)]
pub struct TVCController {
    config: Vec<ThrusterConfig>,
    // Matrix with 3 rows and N columns (for N thrusters)
    // Each column contains the direction of a thruster in body frame
    pub distribution_matrix: na::Matrix3xX<f64>,
    pub max_angles: Vec<f64>,
    pub current_angles: Vec<f64>,
}

impl TVCController {
    pub fn new(thrusters: Vec<ThrusterConfig>) -> Self {
        // Find number of thrusters with TVC
        let num_thrusters = thrusters.iter().filter(|thruster| thruster.tvc.is_some()).count();

        // Fill distribution matrix with zeros
        let mut distribution_matrix = na::Matrix3xX::zeros(num_thrusters);

        // Each column represents the torque generated by a gimballing the thruster
        // We do not care about torque from nominal position of thruster here
        let mut column_index = 0;
        let mut max_angles = Vec::new();
        for thruster in thrusters.iter() {
            if let Some(tvc) = thruster.tvc.as_ref() {
                let gimbal_cf = tvc.axis_cf.as_ref();
                let gimbal_b = thruster.geometry.q_cf2b.transform_vector(gimbal_cf);

                max_angles.push(tvc.max_deflection);
                distribution_matrix.set_column(column_index, &gimbal_b);
                column_index += 1;
            }
        }

        Self {
            config: thrusters.clone(),
            distribution_matrix,
            current_angles: vec![0.0; num_thrusters],
            max_angles,
        }
    }
}

#[derive(Debug, Event)]
pub struct TVCTorqueRequest {
    pub torque_b: na::Vector3<f64>,
}

#[derive(Debug, Event)]
pub struct TVCControllerOutput {
    pub angles: Vec<f64>,
}

/// Receives torque request and maps it to a TVC angle
pub fn update_tvc_controller(mut tvc_torque_request_reader: EventReader<TVCTorqueRequest>,
    // mut engine_query: Query<&EngineController>,  // will be used to read current engine thrust and scale the distribution matrix
    mut tvc_query: Query<&mut TVCController>,
    mut tvc_output_writer: EventWriter<TVCControllerOutput>,)
{
    let mut tvc_controller = tvc_query.single_mut();
    if let Some(tvc_torque_request) = tvc_torque_request_reader.read().last()
    {
        // TODO: Gimbal through CoM first

        // Compute the angle from the torque request
        let angles = tvc_torque_request.torque_b.transpose() * &tvc_controller.distribution_matrix;

        // Clip the angles to the maximum
        let angles: Vec<_> = angles.as_slice()
                                    .iter()
                                    .zip(tvc_controller.max_angles.iter())
                                    .map(|(angle, max_angle)| angle.clamp(-*max_angle, *max_angle))
                                    .collect();

        // Will be used by the engine controller to compute the thrust vector
        tvc_controller.current_angles = angles.clone();

        // Send the angle to the TVC controller
        let tvc_output = TVCControllerOutput {
            angles
        };

        tvc_output_writer.send(tvc_output);
    }
}


// Controller for the vernier engine system
// Will output thrust levels for each engine
#[derive(Debug, Component)]
pub struct VernierEngineController {
    // pub config: Vec<ThrusterConfig>,
    pub current_thrust: Vec<f64>,
}
#[derive(Debug, Event)]
pub struct VernierTorqueRequest {
    pub torque_b: na::Vector3<f64>,
}

pub struct VernierEngineControllerOutput {
    pub thrust_levels: Vec<f64>,
}


/// Receives torque request and maps it to thrust levels for each engine
pub fn update_diff_thrust_controller(mut diff_thrust_control_event: EventReader<VernierTorqueRequest>,
    mut query: Query<(&RCSController, &mut RCSControllerOutput)>,
    mut rcs_output_writer: EventWriter<RCSControllerOutput>,)
{

}
