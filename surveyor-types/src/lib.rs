use std::str::FromStr;

pub mod config;
pub mod math;
pub mod simulation;

/// Celestial body type
#[derive(Debug, PartialEq, Eq, Hash, Clone, Copy)]
pub enum CelestialBodyType {
    Sun,
    Earth,
    Moon,
}
impl FromStr for CelestialBodyType {
    type Err = &'static str;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s {
            "Sun" => Ok(Self::Sun),
            "Earth" => Ok(Self::Earth),
            "Moon" => Ok(Self::Moon),
            _ => Err("Celestial body not supported"),
        }
    }
}

impl CelestialBodyType {
    pub fn to_anise_id(&self) -> i32 {
        match self {
            Self::Sun => anise::constants::celestial_objects::SUN,
            Self::Earth => anise::constants::celestial_objects::EARTH,
            Self::Moon =>  anise::constants::celestial_objects::LUNA,
        }
    }
}
