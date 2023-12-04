//! New module for spacecraft model
//!

use hifitime::{Epoch, Duration};
use surveyor_types::config::Config;

use na::SVectorView;
use nalgebra as na;
use bevy::prelude::*;
use bevy_ecs::system::Commands;

/// A component defining a continuous system that can be stepped over time
#[derive(Debug, Component)]
#[allow(dead_code)]
pub struct ContinuousSystemState {
    /// Indices of the state vector that this system operates on within a larger state vector
    state_vector: na::DVector<f64>,
    /// Number of states in the system
    num_states: usize,
}

impl ContinuousSystemState {
    /// Create a new continuous system
    pub fn new(initial_state: &[f64]) -> Self {
        let num_states = initial_state.len();
        let state_vector = na::DVector::from_vec(initial_state.to_vec());
        Self {
            state_vector,
            num_states,
        }
    }
}



use crate::subsystems::Subsystem;
use crate::universe::Universe;
use crate::{
    integrators::DynamicSystem,
    math::{UnitQuaternion, Vector3},
};
use hard_xml::XmlRead;
use nalgebra::{DVector, SMatrix, SVector};


#[derive(Debug, XmlRead, Clone, PartialEq, Resource)]
#[xml(tag = "InitialState")]
pub struct InitialState {
    #[xml(flatten_text = "time")]
    pub time_str: String,
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
            time_str: "2020-01-01T00:00:00Z".to_string(),
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
    pub prev_state: SVector<f64, 13>,
    pub start_time: Epoch,
    pub time: Duration,
}
impl OrbitalDynamics {
    pub fn new(t: f64, state: SVector<f64, 13>) -> Self {
        let start_time = hifitime::UNIX_REF_EPOCH;
        Self { state, start_time, prev_state: SVector::<f64, 13>::zeros(), time: Duration::from_seconds(t) }
    }
    pub fn from_initial_state(initial_state: &InitialState) -> Self {

        let sim_time = Epoch::from_gregorian_str(&initial_state.time_str).expect("Invaid time string");
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
            prev_state: SVector::<f64, 13>::zeros(),
            time: Duration::from_seconds(0.0),
            start_time: sim_time,
        }
    }
    pub fn get_t(&self) -> f64 {
        self.time.to_seconds()
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

impl Default for OrbitalDynamicsInputs {
    fn default() -> Self {
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
pub struct SpacecraftModel;


// Need some way of passing in scprops to OrbitalDynamics
impl<'a> DynamicSystem<'a> for OrbitalDynamics {
    type DerivativeInputs = (&'a SpacecraftProperties, &'a OrbitalDynamicsInputs);
    fn get_num_states(&self) -> usize {
        13
    }
    fn get_state(&self) -> &[f64] {
        self.state.as_slice()
    }
    fn set_state(&mut self, t: f64, state: &[f64]) {
        self.prev_state = self.state.clone();
        self.state.copy_from_slice(state);
        self.time = Duration::from_seconds(t);
    }

    fn get_t(&self) -> f64 {
        self.time.to_seconds()
    }
    fn get_derivatives(
        &self,
        t: f64,
        _state: &[f64],
        d_state: &mut [f64],
        (sc_props, orbital_dynamics_inputs): &Self::DerivativeInputs,
    ) {
        // Compute derivatives
        let dynamics: na::Matrix<f64, na::Const<13>, na::Const<1>, na::ArrayStorage<f64, 13, 1>> = self.dynamics(t, (sc_props, orbital_dynamics_inputs));
        d_state.copy_from_slice(dynamics.as_slice());
    }
}

/// Struct used to pass spacecraft state information to subsystems without copying
pub struct SpacecraftDiscreteState {
    pub time: f64,
    pub state: SVector<f64, 13>,
}
impl SpacecraftDiscreteState {
    pub fn new(time: f64, state: &SVector<f64, 13>) -> Self {
        Self {
            time,
            state: state.clone(),
        }
    }
    pub fn pos(&self) -> Vector3 {
        Vector3::from_column_slice(&self.state.as_slice()[0..3])
    }
    pub fn vel(&self) -> Vector3 {
        Vector3::from_column_slice(&self.state.as_slice()[3..6])
    }
    pub fn q_i2b(&self) -> UnitQuaternion {
        UnitQuaternion::from_parts(
            self.state[6],
            self.state[7],
            self.state[8],
            self.state[9],
        )
    }
    pub fn omega_b(&self) -> Vector3 {
        Vector3::from_column_slice(&self.state.as_slice()[10..13])
    }
}

#[derive(Event)]
pub struct DiscreteUpdateEvent(pub SpacecraftDiscreteState);
impl std::ops::Deref for DiscreteUpdateEvent{
    type Target = SpacecraftDiscreteState;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}


// Have a startup system that initializes all the continuous systems
// Queries for the size of all continuous systems and creates a new strate vector component that
// holds all the states for all the continuous systems in a single vector

pub fn build_spacecraft_entity(commands: &mut Commands, config: &Config, initial_state: &InitialState) {
    let spacecraft_ent = commands.spawn((
        SimulationTime::new(hifitime::Epoch::from_gregorian_str(&initial_state.time_str).unwrap()),
        OrbitalDynamics::from_initial_state(&initial_state),
        SpacecraftProperties::new(
            1.0,
            SMatrix::from_vec(vec![1., 0., 0., 0., 1., 0., 0., 0., 1.]),
        ),
        SpacecraftModel,
        SimulationResults::default(),
    )).id();

    // commands.entity(spacecraft_ent).push_children(&[orbital_dynamics]);
    commands.entity(spacecraft_ent).with_children(|parent| {
        config.spacecraft.subsystems.iter().for_each(|subsystem_config| {
            parent.spawn((Subsystem::from_config(subsystem_config), Name::new(subsystem_config.to_string())));
        });
    });
}

/// Computes the derivatives of the spacecraft state from the current state and inputs
/// and all the subsystems
fn dydt(_t: f64, state: &[f64], universe: &Universe, subsystems: &[&Subsystem], orb: &mut OrbitalDynamics, sc_props: &SpacecraftProperties) -> DVector<f64>
{
    // None of the subsystems have internal continuous states for now
    let mut d_state = DVector::zeros(13);

    // First call update_dynamics on all subsystems
    let mut orbital_dynamics_input = OrbitalDynamicsInputs::default();
    {
        let r = SVectorView::from_slice(orb.get_position());
        // Apply all gravity model forces to external force
        orbital_dynamics_input.total_torque_b.fill(0.0);
        orbital_dynamics_input.total_force_b = universe.compute_force(&r, &sc_props);
    }

    for subsystem in subsystems.iter() {
        subsystem.update_dynamics(&mut orbital_dynamics_input);
    }

    let deriv_inputs = (sc_props, &orbital_dynamics_input);
    orb.get_derivatives(0.0, &state, d_state.as_mut_slice(), &deriv_inputs);
    d_state
}

use crate::SimulationTime;
use crate::simulation::{SimulationParams, SimulationResults, SimClock};
// System that steps the spacecraft model over one timestep and updates orbital dynamics component
pub (crate) fn step_spacecraft_model(
    mut q_universe: Query<&mut Universe>,
    mut q_spacecrafts: Query<(&mut SpacecraftModel, &SimulationTime, &SpacecraftProperties, &mut OrbitalDynamics, &mut SimulationResults, &Children)>,
    q_subsystems: Query<&mut Subsystem>,
    q_sim_clock: Query<&SimClock>,
    sim_params: Res<SimulationParams>,
    mut discrete_update_event: EventWriter<DiscreteUpdateEvent>)
{
    let dt = sim_params.dt;

    let universe = q_universe.single_mut();

    // Iterate over all spacecrafts
    for (_, t, sc_props, mut orb, mut results, children) in q_spacecrafts.iter_mut() {
        let t = t.get_monotonic_time();

        // Iterate over all subsystems
        let subsystems = children.iter().map(|child| q_subsystems.get(*child).unwrap()).collect::<Vec<_>>();

        // TODO: Also call update_continuous on all subsystems each with its own state
        // Call RK4 over all subsystems and orbital dynamics
        let state = DVector::from_column_slice(orb.get_state());
        let k1 = dydt(t, state.as_slice(), &universe, &subsystems, &mut orb, &sc_props);
        let k2 = dydt(t+dt/2.0, state.as_slice(), &universe, &subsystems, &mut orb, &sc_props);
        let k3 = dydt(t+dt/2.0, state.as_slice(), &universe, &subsystems, &mut orb, &sc_props);
        let k4 = dydt(t+dt, state.as_slice(), &universe, &subsystems, &mut orb, &sc_props);
        let new_state = state + (k1 + 2.0 * k2 + 2.0 * k3 + k4) * dt / 6.0;
        orb.set_state(t + dt, new_state.as_slice());

        // TODO: Move to a separate logging system. Convert the "new state" into an event
        results
            .history
            .push(orb.clone());
    }

    // If we have a discrete update, send it as an event
    let sim_clock = q_sim_clock.single();
    if sim_clock.num_steps % sim_params.num_steps_per_gnc_update == 0 {
        for (_, t, _, orb, _, _) in q_spacecrafts.iter_mut() {
            let t = t.get_monotonic_time();
            let spacecraft_discrete_state =
                SpacecraftDiscreteState::new(t, &orb.state);
            // Send event
            discrete_update_event.send(DiscreteUpdateEvent(spacecraft_discrete_state));
        }
    }
}

// System that updates the discrete state of all subsystems
// Will be called only when a discrete update event is received (which is every num_steps_per_gnc_update)
pub (crate) fn do_discrete_update_from_event(mut discrete_update_event: EventReader<DiscreteUpdateEvent>,
    mut q_subsystems: Query<&mut Subsystem>)
{
    for discrete_update in discrete_update_event.read() {
        for subsystem in q_subsystems.iter_mut() {
            subsystem.into_inner().update_discrete(discrete_update.time, &discrete_update);
        }
    }

}
