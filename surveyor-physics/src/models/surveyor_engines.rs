use crate::spacecraft::OrbitalDynamicsInputs;
use surveyor_types::config::ThrusterConfig;

use super::ActuatorModel;

/// Engine types specific to the Surveyor mission

pub enum VerierRocketCommand {
    Ignite,
    Throttle(f64),
    Shutdown,
}

/// Vernier rocket engine
///
/// Produced 30-104lbf of thrust, for 4.8 minutes
/// Ref: https://www.si.edu/object/rocket-engine-vernier-surveyor-spacecraft%3Anasm_A19790173000
#[derive(Debug)]
pub struct VernierRocket {
    config: ThrusterConfig,
    /// Current thrust level
    thrust: f64,
    /// Thrust vector in the body frame
    thrust_b: nalgebra::Vector3<f64>,
    /// Torque vector in the body frame
    torque_b: nalgebra::Vector3<f64>,
}

impl VernierRocket {
    pub fn new(config: &ThrusterConfig) -> Self {
        Self {
            config: config.clone(),
            thrust: 0.0,
            thrust_b: nalgebra::Vector3::<f64>::zeros(),
            torque_b: nalgebra::Vector3::<f64>::zeros(),
        }
    }
    pub fn from_config(config: &ThrusterConfig) -> Self {
        Self::new(config)
    }
    pub fn handle_commands(&mut self, thrust_value: &f64) {
        // Clamp thrust value to min/max
        self.thrust = thrust_value
            .max(self.config.min_thrust)
            .min(self.config.max_thrust);
    }
    pub fn update_dynamics(&self, outputs: &mut OrbitalDynamicsInputs) {
        // Return a "DynamicOutput" struct that then gets added to the OrbitalDynamicsInputs in simulation.rs
        outputs.total_force_b += self.thrust_b;
        outputs.total_torque_b += self.torque_b;
    }
}

#[derive(Debug, Default)]
pub struct VernierRocketContinuousInputs {
    pub q_tvc2nozzle: Option<nalgebra::UnitQuaternion<f64>>,
}

impl<'a> ActuatorModel<'a> for VernierRocket {
    type ContinuousInputs = VernierRocketContinuousInputs;
    type DiscreteInputs = ();
    type ContinuousOutputs = ();
    type DiscreteOutputs = ();
    type Command = f64;

    // This will be triggered by the FSW with the desired deflection angle
    fn handle_commands(&'a mut self, command: &Self::Command) {
        self.thrust = *command;
    }
    fn update_continuous(&'a mut self, _dt: f64, inputs: &Self::ContinuousInputs) {
        // Thrust is aligned with "z" axis of the component frame, we rotate it to the spacecraft frame
        let thrust_tvc_frame = nalgebra::Vector3::<f64>::new(0.0, 0.0, self.thrust);
        // If the nozzle is gimbaled, rotate the thrust vector into the TVC frame first
        let thrust_cf = inputs
            .q_tvc2nozzle
            .map(|q| q.inverse_transform_vector(&thrust_tvc_frame))
            .unwrap_or(thrust_tvc_frame);
        // Rotate the thrust vector into the spacecraft body frame
        self.thrust_b = self.config.geometry.q_cf2b.transform_vector(&thrust_cf);
        // Compute the torque vector
        self.torque_b = self.config.geometry.cf_offset_com_b.cross(&self.thrust_b);
    }
    fn update_discrete(&'a mut self, _dt: f64, _inputs: &Self::DiscreteInputs) {}

    fn get_discrete_outputs(&'a self) -> &Self::DiscreteOutputs {
        &()
    }
    fn get_continuous_outputs(&'a self) -> &Self::ContinuousOutputs {
        &()
    }
}

// Unit tests
#[cfg(test)]
mod tests {
    use approx::assert_abs_diff_eq;
    use hard_xml::XmlRead;
    use nalgebra::Vector3;

    use super::*;
    use surveyor_types::config::{GeometryParams, ThrusterConfig};

    fn create_test_engine() -> VernierRocket {
        let config = ThrusterConfig {
            min_thrust: 0.0,
            max_thrust: 100.0,
            geometry: GeometryParams::from_str(
                r#"<geometry>
                <q_cf2b>[0.0, 0.0, 0.0, 1.0]</q_cf2b>
                <cf_offset_com_b>[0.0, 0.0, 0.0]</cf_offset_com_b>
            </geometry>"#,
            )
            .unwrap(),
            tvc: None,
        };
        VernierRocket::new(&config)
    }
    fn create_test_engine_aligned_with_x() -> VernierRocket {
        let config = ThrusterConfig {
            min_thrust: 0.0,
            max_thrust: 100.0,
            geometry: GeometryParams::from_str(
                r#"<geometry>
                <q_cf2b>[0.7071068, 0, 0.7071068, 0]</q_cf2b>
                <cf_offset_com_b>[0.0, 0.0, 0.0]</cf_offset_com_b>
            </geometry>"#,
            )
            .unwrap(),
            tvc: None,
        };
        VernierRocket::new(&config)
    }
    #[test]
    fn test_no_thrust() {
        let mut engine = create_test_engine();
        let inputs = VernierRocketContinuousInputs::default();
        engine.update_continuous(0.0, &inputs);
        assert_eq!(engine.thrust_b, Vector3::new(0.0, 0.0, 0.0));
        assert_eq!(engine.torque_b, Vector3::new(0.0, 0.0, 0.0));

        let mut engine = create_test_engine_aligned_with_x();
        let inputs = VernierRocketContinuousInputs::default();
        engine.update_continuous(0.0, &inputs);
        assert_eq!(engine.thrust_b, Vector3::new(0.0, 0.0, 0.0));
        assert_eq!(engine.torque_b, Vector3::new(0.0, 0.0, 0.0));
    }
    #[test]
    fn test_min_thrust() {
        let mut engine = create_test_engine();
        let inputs = VernierRocketContinuousInputs::default();
        engine.handle_commands(&10.0);
        engine.update_continuous(0.0, &inputs);
        assert_abs_diff_eq!(
            engine.thrust_b,
            Vector3::new(0.0, 0.0, 10.0),
            epsilon = 1e-10
        );
        assert_abs_diff_eq!(engine.torque_b, Vector3::new(0.0, 0.0, 0.0));

        let mut engine = create_test_engine_aligned_with_x();
        let inputs = VernierRocketContinuousInputs::default();
        engine.handle_commands(&10.0);
        engine.update_continuous(0.0, &inputs);
        assert_abs_diff_eq!(
            engine.thrust_b,
            Vector3::new(10.0, 0.0, 0.0),
            epsilon = 1e-10
        );
        assert_abs_diff_eq!(engine.torque_b, Vector3::new(0.0, 0.0, 0.0));
    }
    #[test]
    fn test_thrust_vectoring() {
        let mut engine = create_test_engine();
        // Test with no TVC
        let inputs = VernierRocketContinuousInputs {
            q_tvc2nozzle: Some(nalgebra::UnitQuaternion::from_euler_angles(0.0, 0.0, 0.0)),
        };
        engine.handle_commands(&10.0);
        engine.update_continuous(0.0, &inputs);
        assert_eq!(engine.thrust_b, Vector3::new(0.0, 0.0, 10.0));
        assert_eq!(engine.torque_b, Vector3::new(0.0, 0.0, 0.0));

        // Test with a 10 degree deflection about the x axis
        let deflection = 10.0f64.to_radians();
        let inputs = VernierRocketContinuousInputs {
            q_tvc2nozzle: Some(nalgebra::UnitQuaternion::from_euler_angles(
                deflection.to_radians(),
                0.0,
                0.0,
            )),
        };
        engine.update_continuous(0.0, &inputs);
        // Thrust should have rotated about the x axis by 10 degrees in the opposite direction
        let expected_thrust_b = Vector3::new(
            0.0,
            (-deflection).to_radians().sin() * 10.0,
            (-deflection).to_radians().cos() * 10.0,
        );
        assert_abs_diff_eq!(engine.thrust_b, expected_thrust_b);
    }
}
