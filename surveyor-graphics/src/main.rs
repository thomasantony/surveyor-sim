use surveyor_graphics::SurveyorGraphicsPlugin;
use bevy::prelude::*;
use surveyor_physics::{SimulationState, math::Vector3};
use bevy_debug_text_overlay::{screen_print, OverlayPlugin};
use bevy::asset::AssetMetaCheck;

fn show_sim_time(phy_query: Query<& surveyor_physics::SimulationTime>) {
    let time = phy_query.single();
    screen_print!("Sim Time: {:.2}", time.get_monotonic_time());
}

fn start_sim(mut set_sim_state: ResMut<NextState<SimulationState>>,
            mut command_writer: EventWriter<surveyor_gnc::GncCommand>)
{
    set_sim_state.set(SimulationState::Running);
    // command_writer.send(surveyor_gnc::GncCommand::SetGuidanceMode(surveyor_gnc::guidance::GuidanceMode::Manual));
    // command_writer.send(surveyor_gnc::GncCommand::SetGuidanceMode(surveyor_gnc::guidance::GuidanceMode::Pointing(
    //     surveyor_gnc::guidance::AttitudeTarget::Attitude(
    //         nalgebra::UnitQuaternion::from_euler_angles(0.0, 0.0, 0.0)
    //     ))
    // ));
    command_writer.send(surveyor_gnc::GncCommand::SetGuidanceMode(surveyor_gnc::guidance::GuidanceMode::Pointing(
        surveyor_gnc::guidance::AttitudeTarget::Align{
            align_with_b: Vector3::new(nalgebra::Vector3::new(1.0, 0.0, 0.0)),
            align_to_i: Vector3::new(nalgebra::Vector3::new(0.0, 0.0, 1.0)),
        })
    ));
    // command_writer.send(surveyor_gnc::GncCommand::SetGuidanceMode(surveyor_gnc::guidance::GuidanceMode::Pointing(
    //     surveyor_gnc::guidance::AttitudeTarget::BodyRate(nalgebra::Vector3::new(0.1, 0.0, 0.0))
    // )));
}


#[cfg(target_arch = "wasm32")]
pub fn main() {

    App::new()
        .insert_resource(AssetMetaCheck::Never)
        .add_plugins(DefaultPlugins.build().disable::<TransformPlugin>())
        .add_plugins(OverlayPlugin{ font_size: 32.0, ..Default::default() })
        .add_plugins(SurveyorGraphicsPlugin)
        .add_plugins(surveyor_physics::SurveyorPhysicsPlugin)
        .add_plugins(surveyor_gnc::SurveyorGNC::new())
        .add_systems(Startup, start_sim)
        .add_systems(Update, show_sim_time)
        .run();
}


#[cfg(not(target_arch = "wasm32"))]
fn main() {
    App::new()
        .insert_resource(AssetMetaCheck::Never)
        .add_plugins(DefaultPlugins.build().disable::<TransformPlugin>())
        .add_plugins(OverlayPlugin{ font_size: 32.0, ..Default::default() })
        .add_plugins(SurveyorGraphicsPlugin)
        // .add_plugins(bevy_inspector_egui::quick::WorldInspectorPlugin::new())
        .add_plugins(surveyor_physics::SurveyorPhysicsPlugin)
        .add_plugins(surveyor_gnc::SurveyorGNC::new())
        .add_systems(Startup, start_sim)
        .add_systems(Update, show_sim_time)
        // .add_plugins(bevy::diagnostic::LogDiagnosticsPlugin::default())
        // .add_plugins(bevy::diagnostic::FrameTimeDiagnosticsPlugin::default())
        .add_plugins(bevy_screen_diagnostics::ScreenDiagnosticsPlugin::default())
        .add_plugins(bevy_screen_diagnostics::ScreenFrameDiagnosticsPlugin)
        .run();
}
