use crate::math::Real;

/// Axle clutch locking strengths and the fraction of AWD torque sent rearward.
/// All values are normalized to 0..=1. A lock of one is a rigid axle constraint.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct VehicleDifferentialConfig {
    /// Front locking under positive drivetrain power.
    pub front_accel_lock: Real,
    /// Front locking under engine braking, coasting, or disconnected drive.
    pub front_decel_lock: Real,
    /// Rear locking under positive drivetrain power.
    pub rear_accel_lock: Real,
    /// Rear locking under engine braking, coasting, or disconnected drive.
    pub rear_decel_lock: Real,
    /// Rear torque fraction when both axles are driven; ignored for FWD/RWD.
    pub center_rear_bias: Real,
}

impl Default for VehicleDifferentialConfig {
    fn default() -> Self {
        Self {
            front_accel_lock: 0.0,
            front_decel_lock: 0.0,
            rear_accel_lock: 0.0,
            rear_decel_lock: 0.0,
            center_rear_bias: 0.5,
        }
    }
}

impl VehicleDifferentialConfig {
    /// Whether every setting is finite and in the normalized range.
    pub fn is_valid(&self) -> bool {
        [
            self.front_accel_lock,
            self.front_decel_lock,
            self.rear_accel_lock,
            self.rear_decel_lock,
            self.center_rear_bias,
        ]
        .iter()
        .all(|v| v.is_finite() && (0.0..=1.0).contains(v))
    }

    pub(super) fn lock(&self, axle: usize, accelerating: bool) -> Real {
        match (axle, accelerating) {
            (0, true) => self.front_accel_lock,
            (0, false) => self.front_decel_lock,
            (_, true) => self.rear_accel_lock,
            (_, false) => self.rear_decel_lock,
        }
    }
}

// A modest clutch preload remains at zero input torque. The tuning curve is a
// capacity control, not a prescribed slip ratio: 50% gives one reference capacity,
// 99% gives 99. The exact 100% endpoint is handled algebraically, never with a
// large spring or an infinite floating-point torque. Equal static/kinetic limits
// keep the fixed-budget impulse problem convex at the sticking/sliding transition.
const CLUTCH_PRELOAD_TORQUE: Real = 50.0;

pub(super) fn clutch_impulse_limit(lock: Real, axle_torque: Real, dt: Real) -> Real {
    debug_assert!((0.0..1.0).contains(&lock));
    dt.max(0.0) * lock / (1.0 - lock) * (CLUTCH_PRELOAD_TORQUE + axle_torque.abs() * 0.5)
}

pub(super) fn acceleration_mode(
    previous: bool,
    connected: bool,
    torque: Real,
    shaft_omega: Real,
    gear_direction: Real,
) -> bool {
    if !connected {
        return false;
    }
    let direction = if shaft_omega.abs() > 0.01 {
        shaft_omega.signum()
    } else {
        gear_direction
    };
    let directed_torque = torque * direction;
    // A torque deadband retains mode through limiter/assist interruptions. Mode
    // selection is once per tick; numerical previews cannot toggle the clutch.
    if directed_torque > 0.01 {
        true
    } else if directed_torque < -0.01 {
        false
    } else {
        previous
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn differential_settings_reject_invalid_numbers() {
        assert!(VehicleDifferentialConfig::default().is_valid());
        for value in [Real::NAN, Real::INFINITY, -0.1, 1.1] {
            assert!(!VehicleDifferentialConfig {
                rear_accel_lock: value,
                ..Default::default()
            }
            .is_valid());
        }
    }

    #[test]
    fn clutch_capacity_has_open_endpoint_preload_and_timestep_scaling() {
        assert_eq!(clutch_impulse_limit(0.0, 1000.0, 0.02), 0.0);
        let mut previous = 0.0;
        for p in [0.01, 0.25, 0.5, 0.75, 0.99] {
            let value = clutch_impulse_limit(p, 200.0, 0.02);
            assert!(value > previous);
            assert_eq!(value, clutch_impulse_limit(p, -200.0, 0.02));
            assert_eq!(value, 2.0 * clutch_impulse_limit(p, 200.0, 0.01));
            assert!(clutch_impulse_limit(p, 0.0, 0.02) > 0.0);
            previous = value;
        }
    }

    #[test]
    fn drive_and_coast_follow_power_in_reverse_and_at_rest() {
        for direction in [-1.0, 1.0] {
            for speed in [0.0, 30.0] {
                assert!(acceleration_mode(
                    false,
                    true,
                    20.0 * direction,
                    speed * direction,
                    direction
                ));
                assert!(!acceleration_mode(
                    true,
                    true,
                    -20.0 * direction,
                    speed * direction,
                    direction
                ));
            }
        }
        assert!(acceleration_mode(true, true, 0.0, 30.0, 1.0));
        assert!(!acceleration_mode(true, false, 20.0, 30.0, 1.0));
        assert!(!acceleration_mode(true, true, 20.0, -30.0, 1.0));
    }
}
