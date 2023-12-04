use bevy_ecs::prelude::*;
use hard_xml::XmlRead;
use nalgebra as na;

use crate::math::{UnitQuaternion, UnitVector3, Vector3};
use crate::CelestialBodyType;

use crate::simulation::SimulationConfig;

#[derive(Debug, XmlRead, PartialEq)]
#[xml(tag = "Config")]
pub struct Config {
    #[xml(child = "UniverseConfig")]
    pub universe: UniverseConfig,
    #[xml(child = "SpacecraftConfig")]
    pub spacecraft: SpacecraftConfig,
    #[xml(child = "SimulationConfig")]
    pub simulation: SimulationConfig,
    #[xml(child = "GncConfig")]
    pub gnc: GncConfig,
}


#[derive(Debug, PartialEq, Resource, XmlRead)]
#[xml(tag = "GncConfig")]
pub struct GncConfig {
    #[xml(flatten_text = "UpdateRateHz")]
    pub update_rate_hz: f64,
}

#[derive(Debug, XmlRead, PartialEq)]
#[xml(tag = "UniverseConfig")]
pub struct UniverseConfig {
    #[xml(child = "CelestialBodies", child = "CelestialBody")]
    pub celestial_bodies: Vec<CelestialBodyConfig>,
}

#[derive(Debug, XmlRead, PartialEq)]
#[xml(tag = "CelestialBody")]
pub struct CelestialBodyConfig {
    #[xml(attr = "name")]
    pub body_type: CelestialBodyType,
    #[xml(child = "gravity")]
    pub gravity_model: GravityModelConfig,
    #[xml(flatten_text = "radius")]
    pub radius: f64,
    #[xml(flatten_text = "position")]
    pub position: Vector3,
}

#[derive(Debug, XmlRead, PartialEq)]
#[xml(tag = "gravity")]
pub struct GravityModelConfig {
    #[xml(flatten_text = "mu")]
    pub mu: f64,
}

// Define SpacecraftConfig struct
// This will be used to load spacecraft configuration from XML
// This will be used to initialize the SpacecraftModel component
#[derive(Debug, XmlRead, PartialEq)]
#[xml(tag = "SpacecraftConfig")]
pub struct SpacecraftConfig {
    // #[xml(attr = "mass")]
    // pub mass: f64,
    // #[xml(attr = "inertia_com_b")]
    // pub inertia: SMatrix<f64, 3, 3>,
    #[xml(
        child = "Subsystems",
        child = "EngineSubsystem",
        child = "RcsSubsystem",
        child = "ImuSubsystem",
        child = "StarTrackerSubsystem"
    )]
    pub subsystems: Vec<SubsystemConfig>,
}

#[derive(Debug, XmlRead, PartialEq)]
pub enum SubsystemConfig {
    #[xml(tag = "EngineSubsystem")]
    Propulsion(EngineSubsystemConfig),
    #[xml(tag = "RcsSubsystem")]
    Rcs(RcsSubsystemConfig),
    #[xml(tag = "ImuSubsystem")]
    Imu(ImuSubsystemConfig),
    #[xml(tag = "StarTrackerSubsystem")]
    StarTracker(StarTrackerSubsystemConfig),
}
impl ToString for SubsystemConfig {
    fn to_string(&self) -> String {
        match self {
            SubsystemConfig::Propulsion(_) => "Propulsion".to_string(),
            SubsystemConfig::Rcs(_) => "Rcs".to_string(),
            SubsystemConfig::Imu(_) => "Imu".to_string(),
            SubsystemConfig::StarTracker(_) => "StarTracker".to_string(),
        }
    }
}

#[derive(Debug, XmlRead, PartialEq)]
#[xml(tag = "RcsSubsystem")]
pub struct RcsSubsystemConfig {
    #[xml(child = "thruster")]
    pub thrusters: Vec<ThrusterConfig>,
}

#[derive(Debug, XmlRead, PartialEq)]
#[xml(tag = "ImuSubsystem")]
pub struct ImuSubsystemConfig {
    #[xml(child = "Imu")]
    pub sensors: Vec<ImuConfig>,
}

#[derive(Debug, XmlRead, PartialEq)]
#[xml(tag = "StarTrackerSubsystem")]
pub struct StarTrackerSubsystemConfig {
    #[xml(child = "StarTracker")]
    pub sensors: Vec<StarTrackerConfig>,
}

#[derive(Debug, XmlRead, PartialEq, Clone)]
#[xml(tag = "EngineSubsystem")]
pub struct EngineSubsystemConfig {
    #[xml(child = "thruster")]
    pub thrusters: Vec<ThrusterConfig>,
}

#[derive(Debug, XmlRead, PartialEq, Clone)]
#[xml(tag = "tvc")]
// Assume single-axis TVC for now
pub struct TVCConfig {
    /// Configuration parameters for the servo
    #[xml(flatten_text = "max_deflection")]
    pub max_deflection: f64,
    /// Axis of rotation of the servo in the component frame
    #[xml(flatten_text = "axis_cf")]
    pub axis_cf: UnitVector3,
}

// A thruster with an optional TVC system
#[derive(Debug, XmlRead, PartialEq, Clone)]
#[xml(tag = "thruster")]
pub struct ThrusterConfig {
    #[xml(child = "geometry")]
    pub geometry: GeometryParams,
    #[xml(flatten_text = "max_thrust")]
    pub max_thrust: f64,
    #[xml(flatten_text = "min_thrust")]
    pub min_thrust: f64,
    #[xml(child = "tvc")]
    pub tvc: Option<TVCConfig>,
}

#[derive(Debug, XmlRead, Clone, PartialEq)]
#[xml(tag = "geometry")]
pub struct GeometryParams {
    /// Orientation of the component frame relative to the spacecraft frame
    #[xml(flatten_text = "q_cf2b")]
    pub q_cf2b: UnitQuaternion,
    /// Position of the component frame relative to the spacecraft frame center-of-mass
    #[xml(flatten_text = "cf_offset_com_b")]
    pub cf_offset_com_b: Vector3,
}

#[derive(Debug, XmlRead, Clone, PartialEq)]
#[xml(tag = "Imu")]
pub struct ImuConfig {
    #[xml(attr="name")]
    pub name: String,
    #[xml(child = "geometry")]
    pub geometry: GeometryParams,
}

#[derive(Debug, XmlRead, Clone, PartialEq)]
#[xml(tag = "StarTracker")]
pub struct StarTrackerConfig {
    #[xml(attr="name")]
    pub name: String,
    #[xml(child = "geometry")]
    pub geometry: GeometryParams,
}

/// Structure defining geometry of any spacecraft component
#[derive(Debug, Clone, Component)]
pub struct GeometryConfig {
    pub q_cf2b: na::UnitQuaternion<f64>,
    pub cf_b: na::Vector3<f64>,
}
impl Default for GeometryConfig {
    fn default() -> Self {
        Self {
            q_cf2b: na::UnitQuaternion::identity(),
            cf_b: na::Vector3::zeros(),
        }
    }
}
impl GeometryConfig {
    pub fn from_geometry_params(params: &GeometryParams) -> Self {
        Self {
            q_cf2b: params.q_cf2b.0,
            cf_b: params.cf_offset_com_b.0,
        }
    }
    pub fn vec_cf2b(&self, vec_cf: &na::Vector3<f64>) -> na::Vector3<f64> {
        self.q_cf2b * vec_cf
    }
    pub fn vec_b2cf(&self, vec_b: &na::Vector3<f64>) -> na::Vector3<f64> {
        self.q_cf2b.inverse_transform_vector(vec_b)
    }
}
