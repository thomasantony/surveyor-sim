use hard_xml::XmlRead;

use crate::{
    math::{UnitQuaternion, UnitVector3, Vector3},
    universe::CelestialBodyType,
};

#[derive(Debug, XmlRead, PartialEq)]
#[xml(tag = "Config")]
pub struct SimulationConfig {
    #[xml(child = "UniverseConfig")]
    pub universe: UniverseConfig,
    #[xml(child = "SpacecraftConfig")]
    pub spacecraft: SpacecraftConfig,
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
        child = "RcsSubsystem"
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
}
impl ToString for SubsystemConfig {
    fn to_string(&self) -> String {
        match self {
            SubsystemConfig::Propulsion(_) => "Propulsion".to_string(),
            SubsystemConfig::Rcs(_) => "Rcs".to_string(),
            SubsystemConfig::Imu(_) => "Imu".to_string(),
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
    #[xml(child = "thruster")]
    pub thrusters: Vec<ImuConfig>,
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
#[xml(tag = "imu")]
pub struct ImuConfig {
    #[xml(child = "geometry")]
    pub geometry: GeometryParams,
}
