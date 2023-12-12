use rvstruct::ValueStruct;
use std::{
    num::ParseFloatError,
    str::FromStr,
};
use bevy_derive::{Deref, DerefMut};

// Wrapper type to allow parsing of quaternion from string
#[derive(ValueStruct, Debug, Clone, PartialEq, Deref, DerefMut)]
pub struct UnitQuaternion(pub nalgebra::UnitQuaternion<f64>);
impl UnitQuaternion {
    pub fn from_quaternion(q: nalgebra::Quaternion<f64>) -> Self {
        UnitQuaternion(nalgebra::UnitQuaternion::from_quaternion(q))
    }
    pub fn from_parts(w: f64, x: f64, y: f64, z: f64) -> Self {
        UnitQuaternion(nalgebra::UnitQuaternion::from_quaternion(
            nalgebra::Quaternion::new(x, y, z, w),
        ))
    }
}

impl FromStr for UnitQuaternion {
    // ParseError is the error type
    type Err = ParseFloatError;
    // Parse string of format [x,y,z,w]
    fn from_str(s: &str) -> Result<Self, Self::Err> {
        let s = s.trim_matches(|p| p == '[' || p == ']');
        let mut iter = s.split(',');
        // Map all errors to string to satisfy trait bound

        let w = iter.next().unwrap().trim().parse::<f64>()?;
        let x = iter.next().unwrap().trim().parse::<f64>()?;
        let y = iter.next().unwrap().trim().parse::<f64>()?;
        let z = iter.next().unwrap().trim().parse::<f64>()?;
        Ok(UnitQuaternion(nalgebra::UnitQuaternion::from_quaternion(
            nalgebra::Quaternion::new(w, x, y, z),
        )))
    }
}

#[derive(ValueStruct, Debug, Clone, PartialEq, Deref, DerefMut)]
pub struct Vector3(pub nalgebra::Vector3<f64>);
impl Vector3 {
    pub fn from_column_slice(slice: &[f64]) -> Self {
        Vector3(nalgebra::Vector3::from_column_slice(slice))
    }
}
impl FromStr for Vector3 {
    type Err = ParseFloatError;
    // Parse string of format [x,y,z]
    fn from_str(s: &str) -> Result<Self, Self::Err> {
        let s = s.trim_matches(|p| p == '[' || p == ']');
        let mut iter = s.split(',');
        let x = iter.next().unwrap().trim().parse::<f64>()?;
        let y = iter.next().unwrap().trim().parse::<f64>()?;
        let z = iter.next().unwrap().trim().parse::<f64>()?;
        Ok(Vector3(nalgebra::Vector3::new(x, y, z)))
    }
}

#[derive(ValueStruct, Debug, Clone, PartialEq, Deref, DerefMut)]
pub struct UnitVector3(pub nalgebra::UnitVector3<f64>);
impl FromStr for UnitVector3 {
    type Err = ParseFloatError;
    // Parse string of format [x,y,z]
    fn from_str(s: &str) -> Result<Self, Self::Err> {
        let s = s.trim_matches(|p| p == '[' || p == ']');
        let mut iter = s.split(',');
        let x = iter.next().unwrap().trim().parse::<f64>()?;
        let y = iter.next().unwrap().trim().parse::<f64>()?;
        let z = iter.next().unwrap().trim().parse::<f64>()?;
        Ok(UnitVector3(nalgebra::UnitVector3::new_normalize(
            nalgebra::Vector3::new(x, y, z),
        )))
    }
}
