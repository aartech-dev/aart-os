//! Which physical drive layout this 2-motor car uses (DESIGN.md section
//! 7.6) - selected once, at build time (see `main.rs`'s `DRIVE_LAYOUT`
//! constant), not something that changes while driving, since it reflects
//! how the two motors are actually wired into the chassis, not a steering
//! input.
//!
//! Both layouts reuse the *exact same* `DiffController`/`BemfSlipEstimator`
//! mixing and BEMF-based slip-trim machinery underneath (`diff_ctrl.rs`) -
//! only the feedforward mapping from the raw signed `steer_cmd` differs,
//! because the two layouts' geometry differs:
//!
//! - **2WD, left/right** (motor A = left wheel, motor B = right wheel, one
//!   driven axle): whichever side is on the *outside* of a turn needs to
//!   run faster, so the correction is symmetric and flips sign with turn
//!   direction - `steer_cmd` feeds `DiffController` directly, unmodified.
//! - **4WD, front/rear** (motor A = front axle, motor B = rear axle, both
//!   driven, solid axles): the front axle sweeps a longer arc than the
//!   rear in *any* turn, so the correction is a function of steer
//!   *magnitude* only, with a fixed sign - see `axle_balance::front_rear_bias`.

use crate::axle_balance::{front_rear_bias, AxleBalanceConfig};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DriveLayout {
    /// 2WD: motor A = left wheel, motor B = right wheel.
    TwoWheelLeftRight,
    /// 4WD: motor A = front axle, motor B = rear axle.
    FourWheelFrontRear,
}

impl DriveLayout {
    /// Computes the bias command `DiffController::update` expects, from the
    /// raw signed steer command. `axle_bias_gain` is only used by
    /// `FourWheelFrontRear` (see `AxleBalanceConfig::bias_gain`) - ignored
    /// for `TwoWheelLeftRight`, where `steer_cmd` is the bias, clamped to
    /// the same `[-1, 1]` range `DiffController::update` itself clamps to
    /// (so callers can compare this function's output directly without
    /// needing to know which layout is active).
    pub fn bias(&self, steer_cmd: f32, axle_bias_gain: f32) -> f32 {
        match self {
            DriveLayout::TwoWheelLeftRight => steer_cmd.clamp(-1.0, 1.0),
            DriveLayout::FourWheelFrontRear => {
                front_rear_bias(steer_cmd, AxleBalanceConfig { bias_gain: axle_bias_gain })
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn two_wheel_left_right_passes_steer_cmd_through_unmodified() {
        let layout = DriveLayout::TwoWheelLeftRight;
        assert_eq!(layout.bias(0.3, 0.5), 0.3);
        assert_eq!(layout.bias(-0.6, 0.5), -0.6);
    }

    #[test]
    fn two_wheel_left_right_clamps_at_the_bounds() {
        let layout = DriveLayout::TwoWheelLeftRight;
        assert_eq!(layout.bias(2.0, 0.5), 1.0);
        assert_eq!(layout.bias(-2.0, 0.5), -1.0);
    }

    #[test]
    fn two_wheel_left_right_ignores_the_axle_bias_gain() {
        let layout = DriveLayout::TwoWheelLeftRight;
        assert_eq!(layout.bias(0.4, 0.0), layout.bias(0.4, 99.0));
    }

    #[test]
    fn four_wheel_front_rear_delegates_to_front_rear_bias() {
        let layout = DriveLayout::FourWheelFrontRear;
        let gain = 0.7;
        assert_eq!(
            layout.bias(0.4, gain),
            front_rear_bias(0.4, AxleBalanceConfig { bias_gain: gain })
        );
        // Confirms it's really front_rear_bias's behavior, not a
        // pass-through: unlike TwoWheelLeftRight, left/right turns must
        // bias identically here.
        assert_eq!(layout.bias(0.4, gain), layout.bias(-0.4, gain));
    }
}
