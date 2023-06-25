use std::collections::HashMap;

use crate::{
    config::{SpacecraftConfig, RcsSubsystemConfig},
    integrators::NewDynamicSystem,
    math::{UnitQuaternion, Vector3},
    subsystems::Subsystem, interfaces::ActuatorEvents,
};
use hard_xml::XmlRead;
use nalgebra::{DVector, SMatrix, SVector};
use bevy_ecs::prelude::*;

#[derive(Debug, XmlRead, Clone)]
#[xml(tag = "InitialState")]
pub struct InitialState {
    #[xml(flatten_text = "position")]
    pub position: Vector3,
    #[xml(flatten_text = "velocity")]
    pub velocity: Vector3,
    #[xml(flatten_text = "q_i2b")]
    pub q_i2b: UnitQuaternion,
    #[xml(flatten_text = "omega_b")]
    pub omega_b: Vector3,
}

impl Default for InitialState {
    fn default() -> Self {
        Self {
            position: Vector3(nalgebra::Vector3::zeros()),
            velocity: Vector3(nalgebra::Vector3::zeros()),
            q_i2b: UnitQuaternion(nalgebra::UnitQuaternion::identity()),
            omega_b: Vector3(nalgebra::Vector3::zeros()),
        }
    }
}
// Component defining a dynamic state of arbitrary length
#[derive(Component, Debug, Clone, Default)]
pub struct OrbitalDynamics {
    // State includes position, velocity, quaternion, and angular velocity
    pub state: SVector<f64, 13>,
    pub time: f64,
}
impl OrbitalDynamics {
    pub fn new(t: f64, state: SVector<f64, 13>) -> Self {
        Self { state, time: t }
    }
    pub fn from_initial_state(initial_state: &InitialState) -> Self {
        let y0 = SVector::<f64, 13>::from_vec(vec![
            initial_state.position.x,
            initial_state.position.y,
            initial_state.position.z,
            initial_state.velocity.x,
            initial_state.velocity.y,
            initial_state.velocity.z,
            initial_state.q_i2b.w,
            initial_state.q_i2b.i,
            initial_state.q_i2b.j,
            initial_state.q_i2b.k,
            initial_state.omega_b.x,
            initial_state.omega_b.y,
            initial_state.omega_b.z,
        ]);
        Self {
            state: y0,
            time: 0.0,
        }
    }
    pub fn get_t(&self) -> f64 {
        self.time
    }
    pub fn get_trajectory(&self) -> &[f64] {
        self.state.as_slice()
    }
    // Return a view of the state vector without copying
    pub fn get_position(&self) -> &[f64] {
        &self.state.as_slice()[0..3]
    }
    pub fn get_velocity(&self) -> &[f64] {
        &self.state.as_slice()[3..6]
    }
    pub fn get_quaternion(&self) -> &[f64] {
        &self.state.as_slice()[6..10]
    }
    pub fn get_angular_velocity(&self) -> &[f64] {
        &self.state.as_slice()[10..13]
    }
    fn dynamics(
        &self,
        _t: f64,
        (sc, orbital_dynamics_inputs): (&SpacecraftProperties, &OrbitalDynamicsInputs),
    ) -> SVector<f64, 13> {
        let mut new_state = SVector::<f64, 13>::from_vec(vec![0.0; 13]);

        // Translational dynamics
        let v = self.state.fixed_rows::<3>(3);

        let mut dx = new_state.fixed_rows_mut::<3>(0);
        dx.copy_from(&v);

        let mut dv = new_state.fixed_rows_mut::<3>(3);
        dv.copy_from(&(orbital_dynamics_inputs.total_force_b / sc.mass));

        // Rotational dynamics
        // Quaternion is in the order [w, x, y, z]
        let q = self.state.fixed_rows::<4>(6);
        let w = self.state.fixed_rows::<3>(10);

        let mut q_dot = new_state.fixed_rows_mut::<4>(6);
        q_dot[0] = 0.5 * (-q[1] * w[0] - q[2] * w[1] - q[3] * w[2]);
        q_dot[1] = 0.5 * (q[0] * w[0] + q[2] * w[2] - q[3] * w[1]);
        q_dot[2] = 0.5 * (q[0] * w[1] - q[1] * w[2] + q[3] * w[0]);
        q_dot[3] = 0.5 * (q[0] * w[2] + q[1] * w[1] - q[2] * w[0]);

        let h = sc.inertia * w;
        let dwdt = sc.inertia_inv * (orbital_dynamics_inputs.total_torque_b - w.cross(&h));

        let mut w_dot = new_state.fixed_rows_mut::<3>(10);
        w_dot.copy_from(&dwdt);

        new_state
    }
}

// Separate out OrbitalDynamicsInputs and ExternalTorque components

// Component defining inputs to the orbital dynamics model
// This includes the total force and torque acting on the spacecraft center-of-mass
#[derive(Component, Debug, Clone)]
pub struct OrbitalDynamicsInputs {
    pub total_force_b: SVector<f64, 3>,
    pub total_torque_b: SVector<f64, 3>,
}

impl OrbitalDynamicsInputs {
    pub fn new() -> Self {
        Self {
            total_force_b: SVector::<f64, 3>::zeros(),
            total_torque_b: SVector::<f64, 3>::zeros(),
        }
    }
}

#[derive(Component, Debug)]
pub struct SpacecraftProperties {
    pub mass: f64,
    pub inertia: SMatrix<f64, 3, 3>,
    pub inertia_inv: SMatrix<f64, 3, 3>,
}
impl SpacecraftProperties {
    pub fn new(mass: f64, inertia: SMatrix<f64, 3, 3>) -> Self {
        Self {
            mass,
            inertia,
            inertia_inv: inertia.try_inverse().unwrap(),
        }
    }
}

#[derive(Component, Debug)]
pub struct SpacecraftModel {
    subsystems: HashMap<String, Subsystem>,
    orbital_dynamics: OrbitalDynamics,
    ode_state: DVector<f64>,
    // fsw: FlightSoftware,
}

impl SpacecraftModel {
    pub fn new() -> Self {
        Self {
            subsystems: HashMap::new(),
            orbital_dynamics: OrbitalDynamics::new(0.0, SVector::<f64, 13>::zeros()),
            ode_state: DVector::<f64>::zeros(13),
            // fsw: FlightSoftware::new(),
        }
    }
    pub fn from_config(config: SpacecraftConfig, initial_state: InitialState) -> Self {
        // Iterate over subsystems and create subsystems
        let subsystems: Vec<_> = config
            .subsystems
            .iter()
            .map(|subsystem_config| (subsystem_config.to_string(), Subsystem::from_config(subsystem_config)))
            .collect();
        let orbital_dynamics = OrbitalDynamics::from_initial_state(&initial_state);
        let mut ode_state = DVector::<f64>::zeros(
            13 + subsystems
                .iter()
                .map(|(_, subsystem)| subsystem.get_num_states())
                .sum::<usize>(),
        );
        // Set orbital dynamics state
        ode_state.rows_mut(0, 13).copy_from(&orbital_dynamics.state);
        Self {
            subsystems: HashMap::from_iter(subsystems),
            orbital_dynamics,
            ode_state,
            // fsw: FlightSoftware::new(),
        }
    }
    // Loads and initializes subsystems from XML
    pub fn load_from_file(filename: &str, initial_state_xml_file: &str) -> std::io::Result<Self> {
        let xml_text = std::fs::read_to_string(filename)?;
        let initial_state_xml = std::fs::read_to_string(initial_state_xml_file)?;
        Self::load_from_xml(&xml_text, &initial_state_xml).map_err(|xml_err| {
            std::io::Error::new(std::io::ErrorKind::InvalidData, xml_err.to_string())
        })
    }
    pub fn load_from_xml(
        xml_text: &str,
        initial_state_xml: &str,
    ) -> Result<Self, hard_xml::XmlError> {
        let spacecraft_config = SpacecraftConfig::from_str(xml_text)?;
        let initial_state = InitialState::from_str(initial_state_xml)?;
        Ok(Self::from_config(spacecraft_config, initial_state))
    }
    // Builder pattern to add subsystem
    pub fn with_subsystem(mut self, name: String, subsystem: Subsystem) -> Self {
        self.subsystems.insert(name, subsystem);
        self
    }
    pub fn get_trajectory(&self) -> &OrbitalDynamics {
        &self.orbital_dynamics
    }
    pub fn get_trajectory_mut(&mut self) -> &mut OrbitalDynamics {
        &mut self.orbital_dynamics
    }

    pub fn update_discrete(&mut self, dt: f64) {
        // Set up the generic input to be used by all subsystems
        let spacecraft_discrete_state =
            SpacecraftDiscreteState::new(self.get_t(), &self.orbital_dynamics.state);

        for (_, subsystem) in self.subsystems.iter_mut() {
            subsystem.update_discrete(dt, &spacecraft_discrete_state);
        }
    }
    pub fn update_dynamics(&mut self, outputs: &mut OrbitalDynamicsInputs) {
        for (_, subsystem) in self.subsystems.iter_mut() {
            subsystem.update_dynamics(outputs);
        }
    }
    pub fn handle_actuator_commands(&mut self, commands: &ActuatorEvents) {
        match commands {
            ActuatorEvents::RCS(rcs_command) => {
                self.subsystems.get_mut("RCS").unwrap().as_rcs_mut().unwrap().handle_commands(rcs_command);
            }
        }
    }
}

// Need some way of passing in scprops to OrbitalDynamics
impl<'a> NewDynamicSystem<'a> for OrbitalDynamics {
    type DerivativeInputs = (&'a SpacecraftProperties, &'a OrbitalDynamicsInputs);
    fn get_num_states(&self) -> usize {
        13
    }
    fn get_state(&self) -> &[f64] {
        self.state.as_slice()
    }
    fn set_state(&mut self, t: f64, state: &[f64]) {
        self.state.copy_from_slice(state);
        self.time = t;
    }

    fn get_t(&self) -> f64 {
        self.time
    }
    fn get_derivatives(
        &mut self,
        t: f64,
        d_state: &mut [f64],
        (sc_props, orbital_dynamics_inputs): &Self::DerivativeInputs,
    ) {
        // Compute derivatives
        let dynamics = self.dynamics(t, (sc_props, orbital_dynamics_inputs));
        d_state.copy_from_slice(dynamics.as_slice());
    }
}
impl<'a> NewDynamicSystem<'a> for SpacecraftModel {
    type DerivativeInputs = (&'a SpacecraftProperties, &'a OrbitalDynamicsInputs);
    fn get_num_states(&self) -> usize {
        self.subsystems
            .iter()
            .map(|(_, subsystem)| subsystem.get_num_states())
            .sum::<usize>()
            + 13
    }
    fn get_state(&self) -> &[f64] {
        // Combine state vectors from all subsystems and orbital dynamics
        self.ode_state.as_slice()
    }
    fn set_state(&mut self, t: f64, state: &[f64]) {
        self.ode_state.copy_from_slice(state);
        self.orbital_dynamics.set_state(t, &state[0..13]);
        // Set state for all subsystems while stepping through state vector
        let mut offset = 13;
        for (_, subsystem) in self.subsystems.iter_mut() {
            subsystem.set_state(t, &state[offset..offset + subsystem.get_num_states()]);
            offset += subsystem.get_num_states();
        }
    }

    fn get_t(&self) -> f64 {
        self.orbital_dynamics.get_t()
    }
    fn get_derivatives(&mut self, t: f64, d_state: &mut [f64], inputs: &'a Self::DerivativeInputs) {
        // This is hte only place where update_continuous() is called as part of get_derivative()
        //
        // In all the other subsystems, update_continuous() is called first (from here) which sets the values that are
        // then used by get_derivatives() which is called later

        // Call update_continuous on all subsystems
        for (_, subsystem) in self.subsystems.iter_mut() {
            subsystem.update_continuous(t);
        }

        let mut a: OrbitalDynamicsInputs = inputs.1.clone();
        // Update dynamic forces
        self.update_dynamics(&mut a);

        // We assume that d_state has been initialized to the correct length
        self.orbital_dynamics
            .get_derivatives(t, &mut d_state[0..13], &(inputs.0, &a));

        let mut offset = 13;
        // Iterate over subsystems and get derivatives
        // TODO: Use wasm-bindgen-rayon to parallelize this
        for (_, subsystem) in self.subsystems.iter_mut() {
            subsystem.get_derivatives(
                t,
                &mut d_state[offset..offset + subsystem.get_num_states()],
                &mut (),
            );
            offset += subsystem.get_num_states();
        }
    }
}

/// Struct used to pass spacecraft state information to subsystems without copying
pub struct SpacecraftDiscreteState<'a> {
    pub time: f64,
    pub pos: &'a [f64],
    pub vel: &'a [f64],
    pub q_i2b: &'a [f64],
    pub omega_b: &'a [f64],
}
impl<'a> SpacecraftDiscreteState<'a> {
    pub fn new(time: f64, state_vector: &'a SVector<f64, 13>) -> Self {
        Self {
            time,
            pos: &state_vector.as_slice()[0..3],
            vel: &state_vector.as_slice()[3..6],
            q_i2b: &state_vector.as_slice()[6..10],
            omega_b: &state_vector.as_slice()[10..13],
        }
    }
}
