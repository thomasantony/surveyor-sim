use bevy_ecs::prelude::*;

use crate::config::EngineSubsystemConfig;
use crate::spacecraft::SpacecraftDiscreteState;
use crate::{
    integrators::DynamicSystem,
    models::{
        surveyor_engines::{VernierRocket, VernierRocketContinuousInputs},
        ActuatorModel, tvc::TVC,
    },
    spacecraft::OrbitalDynamicsInputs,
};

#[derive(Component, Debug)]
pub struct SurveyorPropulsion {
    vernier_a: VernierRocket,
    vernier_b: VernierRocket,
    vernier_c: VernierRocket,
    tvc_a: TVC,
}

impl SurveyorPropulsion {
    pub fn from_config(config: &EngineSubsystemConfig) -> Self {
        // index thrusters to match 0..2 -> a..c
        let vernier_a = VernierRocket::from_config(&config.thrusters[0]);
        let vernier_b = VernierRocket::from_config(&config.thrusters[1]);
        let vernier_c = VernierRocket::from_config(&config.thrusters[2]);
        let tvc_a = TVC::from_config(&config.thrusters[0].tvc.as_ref().unwrap());
        Self {
            vernier_a,
            vernier_b,
            vernier_c,
            tvc_a,
        }
    }
}

pub struct EngineCommands {
    // This command sets the throttle for the vernier thrusters
    pub vernier_thrust_a: f64,
    pub vernier_thrust_b: f64,
    pub vernier_thrust_c: f64,
}

impl SurveyorPropulsion {
    pub fn update_discrete(&mut self, dt: f64, _discrete_state: &SpacecraftDiscreteState) {
        self.tvc_a.update_discrete(dt, &());
        self.vernier_a.update_discrete(dt, &());
        self.vernier_b.update_discrete(dt, &());
        self.vernier_c.update_discrete(dt, &());
    }
    pub fn update_continuous(&mut self, dt: f64) {
        self.tvc_a.update_continuous(dt, &());
        let tvc_outputs = self.tvc_a.get_continuous_outputs();

        let vernier_a_inputs = VernierRocketContinuousInputs {
            q_tvc2nozzle: Some(tvc_outputs.q_tvc2nozzle.clone()),
        };
        let vernier_inputs = VernierRocketContinuousInputs::default();
        self.vernier_a.update_continuous(dt, &vernier_a_inputs);
        self.vernier_b.update_continuous(dt, &vernier_inputs);
        self.vernier_c.update_continuous(dt, &vernier_inputs);
    }
    pub fn handle_commands(&mut self, commands: &EngineCommands) {
        self.vernier_a.handle_commands(&commands.vernier_thrust_a);
        self.vernier_b.handle_commands(&commands.vernier_thrust_b);
        self.vernier_c.handle_commands(&commands.vernier_thrust_c);
    }
    pub fn update_dynamics(&self, outputs: &mut OrbitalDynamicsInputs) {
        self.vernier_a.update_dynamics(outputs);
        self.vernier_b.update_dynamics(outputs);
        self.vernier_c.update_dynamics(outputs);
    }
}

impl<'a> DynamicSystem<'a> for SurveyorPropulsion {
    type DerivativeInputs = ();
    fn get_state(&self) -> &[f64] {
        &[]
    }

    fn set_state(&mut self, _t: f64, _state: &[f64]) {}

    fn get_num_states(&self) -> usize {
        0
    }

    fn get_t(&self) -> f64 {
        0.0
    }

    fn get_derivatives(
        &self,
        _t: f64,
        _state: &[f64],
        _d_state: &mut [f64],
        _inputs: &'a Self::DerivativeInputs,
    ) {
    }
}
// // TODO: Add tests for engine subsystem
// #[cfg(test)]
// mod tests {
//     use super::*;
//     use approx::assert_relative_eq;

//     #[test]
//     fn test_engine_subsystem() {
//         let mut engine = EngineSubsystem::new();
//         let commands = EngineCommands {
//             vernier_thrust_a: 1.0,
//             vernier_thrust_b: 2.0,
//             vernier_thrust_c: 3.0,
//         };
//         let mut outputs = OrbitalDynamicsInputs::new();
//         engine.handle_commands(&commands);
//         engine.update_dynamics(1.0, &mut outputs);

//         // Test that the computed thrusts are correct in output
//         let expected_thrust = nalgebra::Vector3::new(0.0, 0.0, 6.0);
//         assert_relative_eq!(outputs.total_force_b, expected_thrust, epsilon = 1e-6);

//         // Test that the computed torques are correct
//         let expected_torque = nalgebra::Vector3::new(0.0, 0.0, 0.0);
//         assert_relative_eq!(outputs.total_torque_b, expected_torque, epsilon = 1e-6);
//     }
// }
