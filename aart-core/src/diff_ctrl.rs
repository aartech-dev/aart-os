//! Electronic differential + slip estimate (DESIGN.md section 7.3).
//!
//! Takes a base duty (see `DiffController::update`'s doc comment - these are
//! slot car motors, there's no throttle input to speak of, so "base duty" is
//! ~1.0/sync_max_duty most of the time, not a live speed command) plus a
//! steer command and each motor's own electrical speed, and produces a
//! skid-steer-style duty split plus a slip estimate used to cut duty on the
//! "slipping" (faster-than-commanded) side.
//!
//! `SlipEstimator` is a trait specifically so the estimator can be swapped
//! independently of the mixing/cut logic in `DiffController`. Today's only
//! implementation, `BemfSlipEstimator`, is a **motor-vs-motor proxy**: it
//! compares the two motors' own electrical speed (from BEMF timing, via
//! `Commutator::electrical_rpm`) against what the steer command implies they
//! should be doing - there is no independent wheel-speed or ground-truth
//! sensor behind it. It can't tell "this wheel is actually losing traction"
//! apart from "this wheel is simply unloaded" or gearbox backlash; it only
//! catches gross mismatch between the two motors' electrical speed and the
//! commanded ratio. A future per-wheel encoder would fit this same trait
//! shape (still two per-side speed readings + steer_cmd in); an IMU
//! yaw-rate-based estimator would need a different input shape entirely and
//! isn't something this trait was built to anticipate.

pub trait SlipEstimator {
    /// `erpm_a`/`erpm_b` are `None` when a motor doesn't have a trusted
    /// speed reading yet (still ramping, or stalled) - implementations
    /// should return 0.0 (no basis for an estimate) rather than guess.
    fn estimate(&mut self, erpm_a: Option<u32>, erpm_b: Option<u32>, steer_cmd: f32) -> f32;
}

pub struct BemfSlipEstimator;

impl BemfSlipEstimator {
    pub const fn new() -> Self {
        Self
    }
}

impl Default for BemfSlipEstimator {
    fn default() -> Self {
        Self::new()
    }
}

impl SlipEstimator for BemfSlipEstimator {
    fn estimate(&mut self, erpm_a: Option<u32>, erpm_b: Option<u32>, steer_cmd: f32) -> f32 {
        let (Some(a), Some(b)) = (erpm_a, erpm_b) else {
            return 0.0;
        };
        if a == 0 && b == 0 {
            return 0.0;
        }

        // Normalized differential ratio, -1..1: how much faster b is
        // spinning than a, symmetric around 0. Assuming duty is roughly
        // proportional to speed (a simplification - real speed also
        // depends on load/battery voltage - but a reasonable bench-level
        // starting point), this should equal steer_cmd exactly when both
        // motors are tracking the commanded split with no slip.
        let actual_ratio = (b as f32 - a as f32) / (a as f32 + b as f32);
        actual_ratio - steer_cmd
    }
}

#[derive(Debug, Clone, Copy)]
pub struct DiffControllerConfig {
    /// `|slip_estimate|` below this is treated as noise, not real slip.
    pub slip_threshold: f32,
    /// How hard to cut duty on the slipping side per unit of slip beyond
    /// the threshold. Proportional only ("a small P/PI controller" per
    /// DESIGN.md - P was enough here, so there's no integrator/windup to
    /// manage).
    pub slip_gain: f32,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DiffOutput {
    pub target_duty_a: f32,
    pub target_duty_b: f32,
    pub slip_estimate: f32,
}

pub struct DiffController<S> {
    config: DiffControllerConfig,
    slip_estimator: S,
}

impl<S: SlipEstimator> DiffController<S> {
    pub fn new(config: DiffControllerConfig, slip_estimator: S) -> Self {
        Self {
            config,
            slip_estimator,
        }
    }

    /// `base_duty` is the duty both motors would run at going straight with
    /// no correction - in practice this is `sync_max_duty` (~1.0, "no PWM")
    /// for a motor that's already Running, since there's no throttle input
    /// to vary it; it's a parameter rather than a hardcoded 1.0 here so this
    /// module doesn't need to know that convention itself, and so bench
    /// testing can still drive it directly without a real synced motor.
    pub fn update(
        &mut self,
        base_duty: f32,
        steer_cmd: f32,
        erpm_a: Option<u32>,
        erpm_b: Option<u32>,
    ) -> DiffOutput {
        let base_duty = base_duty.clamp(0.0, 1.0);
        let steer_cmd = steer_cmd.clamp(-1.0, 1.0);

        // steer_cmd biases one side down and the other up from base_duty
        // (skid-steer mixing). Clamped defensively - with base_duty/steer
        // already clamped above, only the upper bound can actually be hit
        // (e.g. base_duty=0.9, steer=0.5 -> 1.35 for the faster side).
        let mut duty_a = (base_duty * (1.0 - steer_cmd)).clamp(0.0, 1.0);
        let mut duty_b = (base_duty * (1.0 + steer_cmd)).clamp(0.0, 1.0);

        let slip = self.slip_estimator.estimate(erpm_a, erpm_b, steer_cmd);

        if slip > self.config.slip_threshold {
            // b is spinning faster than commanded - cut it.
            let excess = slip - self.config.slip_threshold;
            duty_b = (duty_b - excess * self.config.slip_gain).max(0.0);
        } else if slip < -self.config.slip_threshold {
            // a is spinning faster than commanded - cut it.
            let excess = -self.config.slip_threshold - slip;
            duty_a = (duty_a - excess * self.config.slip_gain).max(0.0);
        }

        DiffOutput {
            target_duty_a: duty_a,
            target_duty_b: duty_b,
            slip_estimate: slip,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn approx(a: f32, b: f32) -> bool {
        (a - b).abs() < 1e-4
    }

    fn test_config() -> DiffControllerConfig {
        DiffControllerConfig {
            slip_threshold: 0.05,
            slip_gain: 2.0,
        }
    }

    #[test]
    fn bemf_slip_estimator_reads_zero_when_ratio_matches_steer_cmd() {
        let mut e = BemfSlipEstimator::new();
        // (1200-800)/(800+1200) = 0.2, matching steer_cmd exactly.
        assert!(approx(e.estimate(Some(800), Some(1200), 0.2), 0.0));
    }

    #[test]
    fn bemf_slip_estimator_reads_nonzero_on_mismatch() {
        let mut e = BemfSlipEstimator::new();
        // (1400-1000)/2400 = 0.1667, but steer_cmd says it should be 0.
        assert!(approx(e.estimate(Some(1000), Some(1400), 0.0), 400.0 / 2400.0));
    }

    #[test]
    fn bemf_slip_estimator_returns_zero_without_both_readings() {
        let mut e = BemfSlipEstimator::new();
        assert_eq!(e.estimate(None, Some(1000), 0.0), 0.0);
        assert_eq!(e.estimate(Some(1000), None, 0.0), 0.0);
        assert_eq!(e.estimate(None, None, 0.0), 0.0);
    }

    #[test]
    fn bemf_slip_estimator_does_not_divide_by_zero_when_stationary() {
        let mut e = BemfSlipEstimator::new();
        assert_eq!(e.estimate(Some(0), Some(0), 0.0), 0.0);
    }

    #[test]
    fn straight_no_steer_gives_equal_duty_and_no_slip() {
        let mut c = DiffController::new(test_config(), BemfSlipEstimator::new());
        let out = c.update(0.5, 0.0, Some(1000), Some(1000));
        assert!(approx(out.target_duty_a, 0.5));
        assert!(approx(out.target_duty_b, 0.5));
        assert!(approx(out.slip_estimate, 0.0));
    }

    #[test]
    fn steer_biases_duty_as_documented() {
        let mut c = DiffController::new(test_config(), BemfSlipEstimator::new());
        // No erpm data yet - isolates the mixing math from any slip cut.
        let out = c.update(0.5, 0.3, None, None);
        assert!(approx(out.target_duty_a, 0.5 * 0.7));
        assert!(approx(out.target_duty_b, 0.5 * 1.3));
    }

    #[test]
    fn duty_split_clamps_at_the_upper_bound() {
        let mut c = DiffController::new(test_config(), BemfSlipEstimator::new());
        let out = c.update(0.9, 0.5, None, None);
        assert!(approx(out.target_duty_a, 0.9 * 0.5));
        assert!(approx(out.target_duty_b, 1.0)); // 0.9*1.5=1.35, clamped
    }

    #[test]
    fn no_cut_when_erpm_matches_the_commanded_ratio() {
        let mut c = DiffController::new(test_config(), BemfSlipEstimator::new());
        // steer=0.2 commands a 0.2 ratio; 800/1200 gives exactly that -
        // slip should be ~0 and under threshold, so no cut applied.
        let out = c.update(0.6, 0.2, Some(800), Some(1200));
        assert!(approx(out.target_duty_a, 0.6 * 0.8));
        assert!(approx(out.target_duty_b, 0.6 * 1.2));
    }

    /// The milestone's own scripted scenario: one erpm suddenly jumps
    /// relative to the other, and the controller must cut duty on the
    /// correct (faster/slipping) side.
    #[test]
    fn injected_slip_cuts_the_faster_side() {
        let mut c = DiffController::new(test_config(), BemfSlipEstimator::new());
        // steer=0 commands equal speeds; b suddenly spinning much faster
        // than a is exactly the "wheel lost traction" scenario.
        let out = c.update(0.6, 0.0, Some(1000), Some(1400));
        assert!(out.slip_estimate > test_config().slip_threshold);
        assert!(out.target_duty_b < 0.6, "slipping side should be cut");
        assert!(approx(out.target_duty_a, 0.6), "other side must be untouched");
    }

    #[test]
    fn injected_slip_cuts_the_other_side_when_reversed() {
        let mut c = DiffController::new(test_config(), BemfSlipEstimator::new());
        let out = c.update(0.6, 0.0, Some(1400), Some(1000));
        assert!(out.slip_estimate < -test_config().slip_threshold);
        assert!(out.target_duty_a < 0.6, "slipping side should be cut");
        assert!(approx(out.target_duty_b, 0.6), "other side must be untouched");
    }

    #[test]
    fn missing_erpm_reading_skips_the_slip_cut() {
        let mut c = DiffController::new(test_config(), BemfSlipEstimator::new());
        let out = c.update(0.6, 0.0, None, None);
        assert!(approx(out.slip_estimate, 0.0));
        assert!(approx(out.target_duty_a, 0.6));
        assert!(approx(out.target_duty_b, 0.6));
    }
}
