//! Truth-side model for a sensor that tracks a single star and returns its unit vector
//! in the sensor frame (boresight is the Z-axis).
//! For example, Canopus (which was used for the Surveyor missions)

use nalgebra::Vector3;

use crate::spacecraft::SpacecraftDiscreteState;


#[derive(Debug)]
pub (crate) struct StarSensor {
    /// Rotation from the body frame to the camera frame
    q_cf2b: surveyor_types::math::UnitQuaternion,
    /// Field of view of the sensor in radians
    fov: f64,
    /// Unit vector to the star in the inertial frame
    star_vec_i: Vector3<f64>,

    /// Unit vector to the star in the camera frame
    star_vec_cf: Vector3<f64>,
    /// Whether the star is in the field of view of the sensor
    in_fov: bool,
}

pub (crate) struct StarSensorOutput {
    /// Unit vector to the star in the camera frame
    pub star_vec_cf: Vector3<f64>,
    /// Whether the star is in the field of view of the sensor
    pub valid: bool,
}

impl StarSensor {
    pub fn from_config(config: &surveyor_types::config::StarSensorConfig) -> Self {
        let ra = config.ra_deg.to_radians();
        let dec = config.dec_deg.to_radians();
        Self{
            q_cf2b: config.geometry.q_cf2b.clone(),
            fov: config.fov_deg.to_radians(),
            star_vec_i: Vector3::new(
                dec.cos() * ra.cos(),
                dec.cos() * ra.sin(),
                dec.sin(),
            ),
            star_vec_cf: Vector3::zeros(),
            in_fov: false,
        }
    }
    pub fn get_model_output(&self) -> StarSensorOutput {
        StarSensorOutput {
            star_vec_cf: self.star_vec_cf,
            valid: self.in_fov,
        }
    }
    pub fn is_in_fov(&self) -> bool {
        self.star_vec_cf.z >= self.fov.cos()
    }
    pub fn update_discrete(&mut self, _dt: f64, discrete_state: &SpacecraftDiscreteState) {
        // Rotate the star vector into the body frame
        let star_vec_b = discrete_state.q_i2b().inverse_transform_vector(&self.star_vec_i);
        self.in_fov = self.is_in_fov();
        if !self.in_fov {
            return;
        }
        self.star_vec_cf = self.q_cf2b.inverse_transform_vector(&star_vec_b);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;
    use nalgebra::{UnitQuaternion, SVector};

    #[test]
    fn test_star_sensor() {
        let config = surveyor_types::config::StarSensorConfig {
            name: "Canopus".to_string(),
            geometry: surveyor_types::config::GeometryParams {
                // Z-axis is boresight, pointing out along the +X body axis
                q_cf2b: surveyor_types::math::UnitQuaternion(UnitQuaternion::from_euler_angles(0.0, std::f64::consts::FRAC_PI_2, 0.0)),
                cf_offset_com_b: surveyor_types::math::Vector3(nalgebra::Vector3::zeros()),
            },
            ra_deg: 0.0,
            dec_deg: 0.0,
            fov_deg: 0.0,
        };
        let mut star_sensor = StarSensor::from_config(&config);

        // Define state with identity quaternion
        let mut state: SVector<f64, 13> = SVector::zeros();
        state[6] = 1.0;
        let discrete_state = SpacecraftDiscreteState::new(0.0, &state);
        star_sensor.update_discrete(0.0, &discrete_state);
        let output = star_sensor.get_model_output();
        assert_eq!(output.valid, true);
        assert_abs_diff_eq!(output.star_vec_cf, Vector3::new(0.0, 0.0, 1.0));

        // Now use an attitude where the star is not in the field of view
        state[6] = 0.0;
        state[8] = 1.0;
        star_sensor.update_discrete(0.0, &SpacecraftDiscreteState::new(0.0, &state));
        let output = star_sensor.get_model_output();
        assert_eq!(output.valid, false);
    }
}
