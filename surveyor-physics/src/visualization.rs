use crate::simulation::{SimulationParams, SimulationResults};
use plotly::{
    common::{Mode, Title},
    layout::{Axis, GridPattern, LayoutGrid},
    Layout, Plot, Scatter, Scatter3D,
};
use bevy_ecs::prelude::*;

pub fn create_plots(results: &SimulationResults, _params: &SimulationParams) {
    // Plot x, y, and z position histories in 3D
    let x = results
        .get_state_history()
        .row(0)
        .iter()
        .cloned()
        .collect::<Vec<_>>();
    let y = results
        .get_state_history()
        .row(1)
        .iter()
        .cloned()
        .collect::<Vec<_>>();
    let z = results
        .get_state_history()
        .row(2)
        .iter()
        .cloned()
        .collect::<Vec<_>>();

    let _traj3d = Scatter3D::new(x, y, z).mode(Mode::Lines);

    // Plot altitude vs time
    let altitudes = results
        .get_state_history()
        .rows(0, 3)
        .column_iter()
        .map(|col| col.norm() / 1e3 - 6378.14)
        .collect::<Vec<_>>();
    let time = results
        .get_time_history()
        .iter()
        .cloned()
        .collect::<Vec<_>>();

    let _velocities = results
        .get_state_history()
        .rows(3, 6)
        .column_iter()
        .map(|c| {
            // Compute norm from the components
            (c[0].powi(2) + c[1].powi(2) + c[2].powi(2)).sqrt() / 1e3
        })
        .collect::<Vec<_>>();

    let id = "plot-div";

    let alt_vs_time = Scatter::new(time.clone(), altitudes.clone())
        .x_axis("x1")
        .y_axis("y1")
        .name("");

    // Plot three components of velocity vs time
    let vx = results
        .get_state_history()
        .row(3)
        .iter()
        .cloned()
        .map(|v| v / 1e3)
        .collect::<Vec<_>>();
    let vy = results
        .get_state_history()
        .row(4)
        .iter()
        .cloned()
        .map(|v| v / 1e3)
        .collect::<Vec<_>>();
    let vz = results
        .get_state_history()
        .row(5)
        .iter()
        .cloned()
        .map(|v| v / 1e3)
        .collect::<Vec<_>>();

    let vx_plot = Scatter::new(time.clone(), vx.clone())
        .x_axis("x2")
        .y_axis("y2")
        .name("vx");
    let vy_plot = Scatter::new(time.clone(), vy.clone())
        .x_axis("x2")
        .y_axis("y2")
        .name("vy");
    let vz_plot = Scatter::new(time.clone(), vz.clone())
        .x_axis("x2")
        .y_axis("y2")
        .name("vz");

    // Plot body rate components vs time
    let wx = results
        .get_state_history()
        .row(10)
        .iter()
        .cloned()
        .collect::<Vec<_>>();
    let wy = results
        .get_state_history()
        .row(11)
        .iter()
        .cloned()
        .collect::<Vec<_>>();
    let wz = results
        .get_state_history()
        .row(12)
        .iter()
        .cloned()
        .collect::<Vec<_>>();
    let wx_plot = Scatter::new(time.clone(), wx.clone())
        .x_axis("x3")
        .y_axis("y3")
        .name("wx");
    let wy_plot = Scatter::new(time.clone(), wy.clone())
        .x_axis("x3")
        .y_axis("y3")
        .name("wy");
    let wz_plot = Scatter::new(time.clone(), wz.clone())
        .x_axis("x3")
        .y_axis("y3")
        .name("wz");

    let mut plot = Plot::new();
    plot.add_trace(alt_vs_time);
    plot.add_trace(vx_plot);
    plot.add_trace(vy_plot);
    plot.add_trace(vz_plot);
    plot.add_trace(wx_plot);
    plot.add_trace(wy_plot);
    plot.add_trace(wz_plot);
    let layout = Layout::new()
        .grid(
            LayoutGrid::new()
                .rows(2)
                .columns(2)
                .pattern(GridPattern::Independent),
        )
        .x_axis(Axis::new().title(Title::new("Time (s)")))
        .y_axis(Axis::new().title(Title::new("Altitude (km)")).anchor("x1"))
        .x_axis2(Axis::new().title(Title::new("Time (s)")))
        .y_axis2(
            Axis::new()
                .title(Title::new("Velocity (km/s)"))
                .anchor("x2"),
        )
        .x_axis3(Axis::new().title(Title::new("Time (s)")))
        .y_axis3(Axis::new().title(Title::new("Angular Velocity (rad/s)")));

    plot.set_layout(layout);

    wasm_bindgen_futures::spawn_local(async move {
        // Plot the trajectory
        plotly::bindings::react("plotly_box", &plot).await;
        log::info!("Plotted trajectory");
    });
}

// System for plotting the spacecraft stats using plotters
// fn run_plotting_system(_entities, sim_params, sim_results): Self::SystemData) {
pub fn run_plotting_system(
    sim_params: Res<SimulationParams>,
    sim_results: Query<&SimulationResults>
) {
    let sim_result = &sim_results.single();
    // Only plot of simulation has finished
    if sim_result.history.len() == 0 {
        return;
    }
    if sim_result.history[sim_result.history.len() - 1].time < sim_params.tf {
        return;
    }
    create_plots(&sim_result, &sim_params);
}
