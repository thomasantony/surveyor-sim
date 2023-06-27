use nalgebra::{DVector, DVectorView, SVector};

pub type DynamicsFn<const N: usize, OtherParams> =
    fn(f64, &SVector<f64, N>, &OtherParams) -> SVector<f64, N>;
pub fn rk4_fixed_step<const N: usize, OtherParams>(
    rhs: DynamicsFn<N, OtherParams>,
    dt: f64,
    t: f64,
    state: &SVector<f64, N>,
    params: &OtherParams,
) -> SVector<f64, N> {
    let k1 = rhs(t, &state, &params);
    let k2 = rhs(t + dt / 2.0, &(state + dt / 2.0 * k1), &params);
    let k3 = rhs(t + dt / 2.0, &(state + dt / 2.0 * k2), &params);
    let k4 = rhs(t + dt, &(state + dt * k3), &params);

    state + dt / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
}

pub trait DynamicSystem<'a> {
    type DerivativeInputs: 'a;
    fn get_state(&self) -> &[f64];
    fn set_state(&mut self, t: f64, state: &[f64]);
    fn get_num_states(&self) -> usize;
    fn get_t(&self) -> f64;
    fn get_derivatives(&self, t: f64, state: &[f64], d_state: &mut [f64], inputs: &'a Self::DerivativeInputs);
}

// pub fn do_rk4_step<'a, T: DynamicSystem<'a>>(
//     dt: f64,
//     dynamics: &'a mut T,
//     inputs: &'a T::DerivativeInputs,
// ) {
//     let t = dynamics.get_t();
//     let num_states = dynamics.get_num_states();
//     let mut d_state = DVector::<f64>::zeros(num_states);
//     {
//         dynamics.get_derivatives(t, d_state.as_mut_slice(), inputs);
//     }

//     let k1 = d_state.clone();
//     {
//         let state = DVectorView::from_slice(dynamics.get_state(), dynamics.get_num_states());
//         dynamics.set_state(t + dt / 2.0, &(state + dt / 2.0 * &k1).as_slice());
//     }
//     {
//         dynamics.get_derivatives(t + dt / 2.0, d_state.as_mut_slice(), inputs);
//     }
//     let k2 = d_state.clone();
//     {
//         let state = DVectorView::from_slice(dynamics.get_state(), dynamics.get_num_states());
//         dynamics.set_state(t + dt / 2.0, &(state + dt / 2.0 * &k2).as_slice());
//     }
//     {
//         dynamics.get_derivatives(t + dt / 2.0, d_state.as_mut_slice(), inputs);
//     }
//     let k3 = d_state.clone();
//     {
//         let state = DVectorView::from_slice(dynamics.get_state(), dynamics.get_num_states());
//         dynamics.set_state(t + dt, &(state + dt * &k3).as_slice());
//     }
//     {
//         dynamics.get_derivatives(t + dt, d_state.as_mut_slice(), inputs);
//     }
//     let k4 = d_state.clone();

//     let state = DVectorView::from_slice(dynamics.get_state(), dynamics.get_num_states());
//     let new_state = state + dt / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
//     dynamics.set_state(t + dt, new_state.as_slice());
// }
