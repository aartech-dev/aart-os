//! Front/rear axle-balance feedforward (DESIGN.md section 7.4).
//!
//! This is a 2-motor, solid-axle front/rear layout (motor A drives the
//! front axle, motor B drives the rear - no per-side split at all, since
//! each axle only has one motor). The two axles still need balancing in a
//! turn, but the geometry is different from left/right (`diff_ctrl.rs`):
//!
//! - Left/right: whichever side is on the *outside* of the turn needs to
//!   run faster, so the correction is symmetric and flips sign with turn
//!   direction - that's exactly what `steer_cmd`'s sign does in
//!   `DiffController`.
//! - Front/rear: the front axle sweeps a longer arc than the rear axle in
//!   *any* turn, left or right, because the yaw center sits behind the
//!   front axle regardless of which way the car turns (the same reason
//!   real AWD cars need a center differential or viscous coupling that
//!   only ever lets the front overrun the rear, never the other way).
//!   So this feedforward is a function of steer *magnitude* only (how
//!   sharp the turn is), with a fixed sign (always biases toward the
//!   front) - it never flips with `steer_cmd`'s sign the way left/right
//!   does.
//!
//! The output feeds straight into the existing `DiffController::update`'s
//! `steer_cmd`-shaped parameter (motor "a" = front, motor "b" = rear): the
//! mixing math and the BEMF-based closed-loop slip trim don't need to
//! know *why* the two motors should differ, only by how much - so that
//! machinery is reused unchanged, only this feedforward mapping is new.

#[derive(Debug, Clone, Copy)]
pub struct AxleBalanceConfig {
    /// How much front-axle duty bias to apply per unit of steer magnitude.
    /// 0.0 disables the feedforward entirely (BEMF slip trim only, no
    /// geometric prediction) - a placeholder pending real tuning on
    /// hardware, same caveat as every other tunable gain in this project.
    pub bias_gain: f32,
}

/// Maps a signed steer command (sign = turn direction, magnitude = how
/// sharp) to the front/rear bias `DiffController::update` expects:
/// negative because `DiffController` increases motor "a" (front) duty as
/// its `steer_cmd`-shaped input goes negative (`duty_a = base*(1-steer)`).
/// Always the same sign regardless of `steer_cmd`'s own sign - see the
/// module doc comment for why front/rear isn't symmetric the way
/// left/right is.
pub fn front_rear_bias(steer_cmd: f32, config: AxleBalanceConfig) -> f32 {
    (-config.bias_gain * steer_cmd.abs()).clamp(-1.0, 1.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cfg() -> AxleBalanceConfig {
        AxleBalanceConfig { bias_gain: 1.0 }
    }

    #[test]
    fn zero_steer_gives_zero_bias() {
        assert_eq!(front_rear_bias(0.0, cfg()), 0.0);
    }

    #[test]
    fn left_and_right_turns_bias_the_same_direction() {
        let left = front_rear_bias(-0.4, cfg());
        let right = front_rear_bias(0.4, cfg());
        assert!((left - right).abs() < 1e-6, "left/right turns must bias identically");
        assert!(left < 0.0, "bias must favor the front axle (negative, per DiffController's convention)");
    }

    #[test]
    fn bias_scales_with_steer_magnitude() {
        let gentle = front_rear_bias(0.2, cfg());
        let sharp = front_rear_bias(0.6, cfg());
        assert!(sharp < gentle, "a sharper turn should bias the front axle harder");
    }

    #[test]
    fn zero_gain_disables_the_feedforward() {
        let out = front_rear_bias(0.8, AxleBalanceConfig { bias_gain: 0.0 });
        assert_eq!(out, 0.0);
    }

    #[test]
    fn bias_clamps_at_the_bound() {
        let out = front_rear_bias(1.0, AxleBalanceConfig { bias_gain: 3.0 });
        assert_eq!(out, -1.0);
    }
}
