use std::{ops::{Deref, DerefMut}, fmt::Formatter};
use bevy_ecs::prelude::*;
use hard_xml::XmlRead;

// Enum defining stopping conditions for simulation
#[derive(Resource, Clone, PartialEq)]
#[derive(XmlRead)]
pub enum SimStoppingCondition {
    #[xml(tag="MaxDuration")]
    MaxDuration(#[xml(text)] f64),
    #[xml(tag="CollisionWith")]
    CollisionWith(#[xml(text)] String),
    // Custom(Box<dyn Fn(&OrbitalDynamics, &Universe) -> bool + Sync + Send + 'static>),
}

#[derive(Debug, Resource, Clone, PartialEq)]
#[derive(XmlRead)]
#[xml(tag="StoppingConditions")]
pub struct StoppingConditionVec(
    #[xml(child="MaxDuration", child="CollisionWith")] pub Vec<SimStoppingCondition>
);
impl Deref for StoppingConditionVec {
    type Target = Vec<SimStoppingCondition>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
impl DerefMut for StoppingConditionVec {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl std::fmt::Debug for SimStoppingCondition {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            SimStoppingCondition::MaxDuration(t) => write!(f, "MaxDuration({})", t),
            SimStoppingCondition::CollisionWith(body) => write!(f, "CollisionWith({})", body),
            // SimStoppingCondition::Custom(_) => write!(f, "Custom"),
        }
    }
}

// Struct holding parameters for simulation
#[derive(Debug, XmlRead, PartialEq)]
#[xml(tag = "SimulationConfig")]
pub struct SimulationConfig {
    #[xml(default, flatten_text="SimRateHz")]
    pub sim_rate_hz: f64,
    #[xml(child="StoppingConditions")]
    pub stopping_conditions: StoppingConditionVec,
    /// Time acceleration factor used to run simulation at a faster (or slower) rate
    #[xml(default, flatten_text="TimeAccel")]
    pub time_acceleration: f64,
}
impl SimulationConfig {
    pub fn new(sim_rate_hz: f64, time_acceleration: f64, stopping_conditions: Vec<SimStoppingCondition>) -> Self {
        Self {
            sim_rate_hz,
            time_acceleration,
            stopping_conditions: StoppingConditionVec(stopping_conditions),
        }
    }
}
// Implement a sane default for SimulationParams
impl Default for SimulationConfig {
    fn default() -> Self {
        Self {
            sim_rate_hz: 100.0,
            time_acceleration: 1.0,
            stopping_conditions: StoppingConditionVec(vec![SimStoppingCondition::MaxDuration(100.0)]),
        }
    }
}
