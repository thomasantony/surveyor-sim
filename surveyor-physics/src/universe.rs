use std::{collections::HashMap, str::FromStr};

use nalgebra::{SVector, SVectorView};
use bevy_ecs::prelude::*;

use crate::{config::UniverseConfig, spacecraft::SpacecraftProperties};

/// Environment models
pub const MU: f64 = 398600.4418; // km^3/s^2

/// Gravity model
#[derive(Component, Debug)]
pub struct GravityModel {
    pub mu: f64,
}

impl GravityModel {
    pub fn new(mu: f64) -> Self {
        Self { mu }
    }
    pub fn compute_force(
        &self,
        x: &SVectorView<f64, 3>,
        sc: &SpacecraftProperties,
    ) -> SVector<f64, 3> {
        // Log the norm of the position vector
        let r3 = x.norm().powf(3.0);
        -self.mu / r3 * x * sc.mass
    }
}

/// Celestial body model
#[derive(Debug)]
pub struct CelestialBodyModel {
    /// Gravity model (TODO: Make this generic to support different gravity models)
    pub gravity_model: GravityModel,
    /// Ephemerides data (TODO: Load from JPL Horizons (maybe generate config data from HORIZONS?))
    pub ephemerides: (),
    /// Radius of the body (TODO: Make this generic to support different body models)
    pub radius: f64,
    /// Position
    pub position: SVector<f64, 3>,
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

/// Celestial body
#[derive(Debug, PartialEq, Eq, Hash)]
pub enum CelestialBodyType {
    Sun,
    Earth,
    Moon,
}

#[derive(Debug, Component)]
pub struct Universe {
    pub celestial_bodies: HashMap<CelestialBodyType, CelestialBodyModel>,
}

impl Default for Universe {
    fn default() -> Self {
        Self::new()
    }
}
impl Universe {
    pub fn new() -> Self {
        //  Add just the Earth for now at the origin
        let earth = CelestialBodyModel {
            gravity_model: GravityModel::new(MU),
            ephemerides: (),
            radius: 6378.14,
            position: SVector::<f64, 3>::zeros(),
        };
        Self {
            celestial_bodies: vec![(CelestialBodyType::Earth, earth)]
                .into_iter()
                .collect(),
        }
    }
    pub fn from_config(config: UniverseConfig) -> Self {
        let celestial_bodies = config
            .celestial_bodies
            .into_iter()
            .map(|body_config| {
                let body_type = body_config.body_type;
                let gravity_model = GravityModel::new(body_config.gravity_model.mu);
                (
                    body_type,
                    CelestialBodyModel {
                        gravity_model,
                        ephemerides: (),
                        radius: body_config.radius,
                        position: body_config.position.0,
                    },
                )
            })
            .collect();
        Self { celestial_bodies }
    }
    pub fn compute_force(
        &self,
        x: &SVectorView<f64, 3>,
        sc: &SpacecraftProperties,
    ) -> SVector<f64, 3> {
        // Compute force from each celestial body and sum
        self.celestial_bodies.iter().fold(
            SVector::<f64, 3>::zeros(),
            |force, (_body_type, body_model)| {
                let rel_pos = x - body_model.position;
                let body_force = body_model.gravity_model.compute_force(&rel_pos.fixed_rows::<3>(0), sc);
                force + body_force
            },
        )
    }
}
