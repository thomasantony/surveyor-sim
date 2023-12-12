use std::fmt::Debug;
use std::{collections::HashMap, str::FromStr};
use anise::almanac::Almanac;
use anise::astro::Aberration;
use bevy::asset::{AssetLoader, AsyncReadExt};
use bevy_derive::{Deref, DerefMut};
use bevy::utils::BoxedFuture;
use surveyor_types::CelestialBodyType;
use surveyor_types::config::UniverseConfig;

use nalgebra::{SVector, SVectorView};
use bevy::prelude::*;
use bevy::{asset::{AssetServer, io::Reader, LoadContext}, utils::thiserror::Error};

use crate::SimulationTime;
use crate::spacecraft::SpacecraftProperties;

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
    pub velocity: SVector<f64, 3>,
}

#[derive(Debug, Component)]
pub struct Universe {
    pub celestial_bodies: HashMap<CelestialBodyType, CelestialBodyModel>,
    pub ephem: Handle<Ephemerides>,
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
            velocity: SVector::<f64, 3>::zeros(),
        };
        Self {
            celestial_bodies: vec![(CelestialBodyType::Earth, earth)]
                .into_iter()
                .collect(),
            ephem: Default::default(),
        }
    }
    pub fn from_config(config: UniverseConfig, server: &Res<AssetServer>, eph_loader: &Res<Assets<Ephemerides>>) -> Self {
        let ephemerides_path = config.ephemerides_path;
        let ephemerides_handle = server.load::<Ephemerides>(ephemerides_path);

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
                        velocity: SVector::<f64, 3>::zeros(),
                    },
                )
            })
            .collect();
        Self { celestial_bodies, ephem: ephemerides_handle }
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

    pub fn observe(&self) -> Observation {
        Observation::new(self)
    }
}

// Struct representing an observation of the state of all bodies in the universe
pub struct Observation<'a>{
    pub celestial_bodies: HashMap<CelestialBodyType, &'a CelestialBodyModel>,
}
impl<'a> Observation<'a>{
    pub fn new(universe: &'a Universe) -> Self {
        Self {
            celestial_bodies: universe.celestial_bodies.iter().map(|(body_type, body_model)| (*body_type, body_model)).collect(),
        }
    }
    pub fn get_body(&self, body_type: CelestialBodyType) -> Option<&CelestialBodyModel> {
        self.celestial_bodies.get(&body_type).copied()
    }

    pub fn get_body_by_name(&self, name: &str) -> Option<&CelestialBodyModel> {
        CelestialBodyType::from_str(name).ok().and_then(|body_type| self.get_body(body_type))
    }
}


#[derive(Asset, TypePath, Default, Deref, DerefMut)]
#[deref(forward)]
pub struct Ephemerides(pub Almanac);

impl Debug for Ephemerides {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "Ephemerides")
    }
}


#[derive(Default)]
pub struct AlmanacLoader;


#[non_exhaustive]
#[derive(Debug, Error)]
pub enum AlmanacLoaderError {
    /// An [IO](std::io) Error
    #[error("Could not load asset: {0}")]
    Io(#[from] std::io::Error),

    #[error("Could not load almanac: {0}")]
    Anise(#[from] anise::errors::AlmanacError),
}

impl AssetLoader for AlmanacLoader {
    type Asset = Ephemerides;
    type Settings = ();
    type Error = AlmanacLoaderError;
    fn load<'a>(
        &'a self,
        reader: &'a mut Reader,
        _settings: &'a (),
        _load_context: &'a mut LoadContext,
    ) -> BoxedFuture<'a, Result<Self::Asset, Self::Error>> {
        Box::pin(async move {
            let mut data = Vec::new();
            reader.read_to_end(&mut data).await?;
            let almanac = Almanac::default();
            let almanac = almanac.load_from_bytes(bytes::Bytes::from(data))?;
            Ok(Ephemerides(almanac))
        })
    }

    fn extensions(&self) -> &[&str] {
        &["bsp"]
    }
}

pub fn update_universe(
    mut universe: Query<&mut Universe>,
    mut eph_loader: ResMut<Assets<Ephemerides>>,
    sim_time: Query<&SimulationTime>,
) {
    let mut universe = universe.single_mut();
    let sim_time = sim_time.single();
    if let Some(eph) = eph_loader.get_mut(&universe.ephem) {
        let frame_id = anise::constants::frames::LUNA_J2000;
        let epoch = sim_time.now();
        for (body_type, body_model) in universe.celestial_bodies.iter_mut() {
            let body_id = body_type.to_anise_id();
            let body_state = eph.state_of(body_id, frame_id, epoch, Aberration::None);
            if let Ok(body_state) = body_state {
                let posvel = body_state.to_cartesian_pos_vel() * 1000.0;
                body_model.position = posvel.fixed_rows::<3>(0).into();
                body_model.velocity = posvel.fixed_rows::<3>(3).into();
            } else {
                println!("Failed to get state of body: {:?}", body_type);
            }
        }
    }
}
