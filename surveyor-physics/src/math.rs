use rvstruct::ValueStruct;
use std::{
    num::ParseFloatError,
    ops::{Deref, DerefMut},
    str::FromStr,
};

// Wrapper type to allow parsing of quaternion from string
#[derive(ValueStruct, Debug, Clone, PartialEq)]
pub struct UnitQuaternion(pub nalgebra::UnitQuaternion<f64>);

impl Deref for UnitQuaternion {
    type Target = nalgebra::UnitQuaternion<f64>;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
impl DerefMut for UnitQuaternion {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
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

#[derive(ValueStruct, Debug, Clone, PartialEq)]
pub struct Vector3(pub nalgebra::Vector3<f64>);
impl Deref for Vector3 {
    type Target = nalgebra::Vector3<f64>;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
impl DerefMut for Vector3 {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
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

#[derive(ValueStruct, Debug, Clone, PartialEq)]
pub struct UnitVector3(pub nalgebra::UnitVector3<f64>);
impl Deref for UnitVector3 {
    type Target = nalgebra::UnitVector3<f64>;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
impl DerefMut for UnitVector3 {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}
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
