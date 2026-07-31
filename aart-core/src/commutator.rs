//! Six-step trapezoidal BLDC commutation state machine, one instance per
//! motor. Hardware-agnostic: takes tick counts and zero-cross events in,
//! produces a step index + duty out. See DESIGN.md section 7.1.
//!
//! Timing model: a BEMF zero-crossing occurs at the electrical midpoint
//! between two commutations (30 degrees after the previous one, 30 degrees
//! before the next), so the next commutation is scheduled at
//! `zero_cross_tick + period / 2`, where `period` is the time since the
//! previous zero-cross. Once running, every commutation also immediately
//! reschedules the *following* one from the current period estimate alone
//! (`deadline + period`) - this is what lets the motor keep commutating
//! through an occasional missed zero-crossing detection instead of stalling
//! on the spot; a fresh zero-cross simply overwrites the prediction with a
//! more accurate one when it arrives.
//!
//! There is no throttle/speed *input* anywhere in this module. These are
//! slot car motors: track voltage (external to this system) is what
//! actually controls speed, and once a motor is synced it is driven at a
//! fixed near-100% duty ("no PWM") permanently - the only thing that ever
//! pulls duty below that baseline again is the differential controller
//! (`diff_ctrl.rs`) cutting the inside motor's duty during a corner. Sync
//! itself always targets the same fixed configured eRPM every time it runs,
//! not a per-run command.

/// Rejects a measured period that suddenly jumped by more than this factor
/// (expressed as eighths) as an almost-certainly-missed zero-crossing rather
/// than a real slowdown, so one dropped detection doesn't corrupt the period
/// estimate the predictive scheduling relies on.
const PERIOD_JUMP_REJECT_EIGHTHS: u64 = 13; // 1.625x

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RunPhase {
    /// Open-loop: forced commutation, ramping the commanded rate (and duty
    /// in lockstep) from `sync_start_erpm` up to `sync_target_erpm`, no BEMF
    /// timing trusted yet.
    Startup,
    /// Closed-loop: commutating off zero-cross timing, duty pinned at
    /// `sync_max_duty` as a baseline (the differential controller may still
    /// pull it lower for cornering - see `diff_ctrl.rs`).
    Running,
    /// No zero-cross seen within the expected window at the current rate.
    Stalled,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PhaseState {
    High,
    Low,
    Float,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StepPattern {
    pub a: PhaseState,
    pub b: PhaseState,
    pub c: PhaseState,
}

const STEP_TABLE: [StepPattern; 6] = [
    StepPattern {
        a: PhaseState::High,
        b: PhaseState::Low,
        c: PhaseState::Float,
    },
    StepPattern {
        a: PhaseState::High,
        b: PhaseState::Float,
        c: PhaseState::Low,
    },
    StepPattern {
        a: PhaseState::Float,
        b: PhaseState::High,
        c: PhaseState::Low,
    },
    StepPattern {
        a: PhaseState::Low,
        b: PhaseState::High,
        c: PhaseState::Float,
    },
    StepPattern {
        a: PhaseState::Low,
        b: PhaseState::Float,
        c: PhaseState::High,
    },
    StepPattern {
        a: PhaseState::Float,
        b: PhaseState::Low,
        c: PhaseState::High,
    },
];

/// `step` is 1..=6; panics (array index out of bounds) outside that range,
/// same as any other internal invariant violation in this module.
pub fn step_pattern(step: u8) -> StepPattern {
    STEP_TABLE[(step - 1) as usize]
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Phase {
    A,
    B,
    C,
}

/// Which physical phase is floating (and therefore the one to sample for
/// BEMF) during `step`.
pub fn floating_phase(step: u8) -> Phase {
    let p = step_pattern(step);
    if p.a == PhaseState::Float {
        Phase::A
    } else if p.b == PhaseState::Float {
        Phase::B
    } else {
        debug_assert_eq!(p.c, PhaseState::Float);
        Phase::C
    }
}

/// Whether the floating phase should cross neutral rising (low-to-high) or
/// falling during `step`, derived from `STEP_TABLE` itself (the phase that
/// floats now is the one the *next* step drives - rising if that's a High).
fn expected_rising_edge(step: u8) -> bool {
    let next_step = if step == 6 { 1 } else { step + 1 };
    let next = step_pattern(next_step);
    match floating_phase(step) {
        Phase::A => next.a == PhaseState::High,
        Phase::B => next.b == PhaseState::High,
        Phase::C => next.c == PhaseState::High,
    }
}

/// Turns raw ADC samples into zero-cross events by watching for the
/// floating phase's reading to cross the virtual-neutral reading in the
/// direction six-step commutation expects for the current step.
///
/// No commutation blanking here (real ESCs mask out switching noise for a
/// short window right after each commutation) - a known gap, not built
/// speculatively; add it if real hardware shows false triggers right after
/// a step change.
pub struct ZeroCrossDetector {
    last_step: Option<u8>,
    last_sample_positive: Option<bool>,
}

impl ZeroCrossDetector {
    pub const fn new() -> Self {
        Self {
            last_step: None,
            last_sample_positive: None,
        }
    }

    /// `floating_sample`/`neutral_sample` are raw ADC counts for the
    /// currently-floating phase (see `floating_phase`) and the virtual
    /// neutral node, sampled at the same instant. Returns true exactly on
    /// the sample where the reading crosses neutral in the expected
    /// direction for `step`.
    pub fn check(&mut self, step: u8, floating_sample: u16, neutral_sample: u16) -> bool {
        let positive = floating_sample >= neutral_sample;

        // A different step means a different physical phase (and a
        // different expected direction) is now floating - the previous
        // reading isn't a valid reference point for it.
        if self.last_step != Some(step) {
            self.last_step = Some(step);
            self.last_sample_positive = Some(positive);
            return false;
        }

        let previous = self.last_sample_positive.replace(positive);
        match previous {
            Some(prev) if prev != positive => positive == expected_rising_edge(step),
            _ => false,
        }
    }
}

impl Default for ZeroCrossDetector {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Output {
    pub step: u8,
    pub duty: f32,
    pub phase: RunPhase,
}

#[derive(Debug, Clone, Copy)]
pub struct CommutatorConfig {
    /// Ticks per second of the timebase `poll()`/`on_zero_cross()` ticks are
    /// counted in.
    pub tick_hz: u32,
    /// Electrical RPM at the very start of the open-loop sync ramp.
    pub sync_start_erpm: u32,
    /// Electrical RPM the sync ramp targets before attempting handoff to
    /// closed-loop BEMF commutation - a fixed characteristic of the motor
    /// (where its BEMF becomes reliably detectable), the same every time
    /// sync runs, not something a command sets per-run.
    pub sync_target_erpm: u32,
    /// Duty applied at `sync_start_erpm`.
    pub sync_start_duty: f32,
    /// Duty applied by the time the ramp reaches `sync_target_erpm`, and the
    /// baseline duty used once Running (the "no PWM" duty - 1.0 for
    /// literally 100%, direct drive).
    pub sync_max_duty: f32,
    /// How fast the open-loop commanded rate accelerates during sync, in
    /// electrical RPM added each commutation step (not each tick - the
    /// step rate itself is what's actually accelerating).
    pub sync_ramp_erpm_per_step: u32,
    /// Declare a stall if no zero-cross arrives within
    /// `stall_multiplier * period_estimate` ticks of the last one.
    pub stall_multiplier: u32,
}

/// Ticks between consecutive zero-crossings (1/6 of an electrical
/// revolution) for a given electrical RPM. Inverse of the arithmetic in
/// `Commutator::electrical_rpm`. `erpm == 0` returns `u64::MAX` (never)
/// rather than dividing by zero - callers are expected to treat that as "no
/// commutation rate to schedule from" rather than actually waiting for it.
fn period_ticks_for_erpm(erpm: u32, tick_hz: u32) -> u64 {
    if erpm == 0 {
        return u64::MAX;
    }
    (tick_hz as u64 * 10) / erpm as u64
}

pub struct Commutator {
    config: CommutatorConfig,
    step: u8,
    phase: RunPhase,
    target_duty: f32,
    commanded_erpm: u32,
    last_zero_cross: Option<u64>,
    period_estimate: Option<u64>,
    period_estimate_trusted: bool,
    next_commutation: Option<u64>,
}

impl Commutator {
    pub fn new(config: CommutatorConfig) -> Self {
        let commanded_erpm = config.sync_start_erpm;
        Self {
            config,
            step: 1,
            phase: RunPhase::Startup,
            target_duty: 0.0,
            commanded_erpm,
            last_zero_cross: None,
            period_estimate: None,
            period_estimate_trusted: false,
            next_commutation: None,
        }
    }

    pub fn phase(&self) -> RunPhase {
        self.phase
    }

    pub fn step(&self) -> u8 {
        self.step
    }

    /// Sets the duty the Running phase applies (the differential controller
    /// calls this every tick - see `diff_ctrl.rs`). Has no effect while
    /// still in Startup, which computes its own ramp duty; harmless to call
    /// regardless of phase.
    pub fn set_target_duty(&mut self, duty: f32) {
        self.target_duty = duty.clamp(0.0, 1.0);
    }

    /// Electrical RPM derived from the zero-cross period estimate (six
    /// zero-crossings per electrical revolution). This is *not* mechanical
    /// RPM - that needs dividing by the motor's pole-pair count, which
    /// nothing in this system knows yet - so telemetry built on this number
    /// is electrical-rate-only until a pole-pair count exists somewhere.
    /// `None` before a trusted period estimate exists (still ramping, or no
    /// real zero-cross observed yet). For a best-effort estimate that's
    /// never `None` (including mid-ramp), see `current_erpm`.
    pub fn electrical_rpm(&self) -> Option<u32> {
        if !self.period_estimate_trusted {
            return None;
        }
        let period = self.period_estimate?;
        if period == 0 {
            return None;
        }
        Some(((self.config.tick_hz as u64 * 10) / period) as u32)
    }

    /// Best current estimate of electrical RPM regardless of phase: the
    /// open-loop commanded rate during Startup, the BEMF-measured rate
    /// during Running, or 0 once Stalled. Meant for things like PWM
    /// switching-frequency scheduling that need a live speed estimate
    /// throughout sync, not just the stricter (`None`-until-trusted)
    /// `electrical_rpm`.
    pub fn current_erpm(&self) -> u32 {
        match self.phase {
            RunPhase::Startup => self.commanded_erpm,
            RunPhase::Running => self.electrical_rpm().unwrap_or(0),
            RunPhase::Stalled => 0,
        }
    }

    /// Call when a BEMF zero-crossing is detected on the currently-floating
    /// phase (see `step_pattern`). Ignored once stalled.
    pub fn on_zero_cross(&mut self, tick: u64) {
        if self.phase == RunPhase::Stalled {
            return;
        }

        if let Some(prev) = self.last_zero_cross {
            let period = tick.saturating_sub(prev);
            let looks_like_a_missed_beat = self.period_estimate_trusted
                && self
                    .period_estimate
                    .is_some_and(|old| period * 8 > old * PERIOD_JUMP_REJECT_EIGHTHS);

            if period > 0 && !looks_like_a_missed_beat {
                self.period_estimate = Some(period);
                self.period_estimate_trusted = true;
                self.next_commutation = Some(tick + period / 2);
            }
        }

        self.last_zero_cross = Some(tick);
    }

    fn advance_step(&mut self) {
        self.step = if self.step == 6 { 1 } else { self.step + 1 };
    }

    /// Call every tick; advances the step when the scheduled commutation
    /// deadline (open-loop during sync, BEMF-derived once Running) has been
    /// reached.
    pub fn poll(&mut self, tick: u64) -> Output {
        match self.phase {
            RunPhase::Startup => self.poll_startup(tick),
            RunPhase::Running => self.poll_running(tick),
            RunPhase::Stalled => Output {
                step: self.step,
                duty: 0.0,
                phase: RunPhase::Stalled,
            },
        }
    }

    fn poll_startup(&mut self, tick: u64) -> Output {
        // Scheduled from whatever commanded_erpm was as of the *last* step
        // (or sync_start_erpm, before the first one) - commanded_erpm only
        // moves when a step actually fires, below, so this stays consistent
        // with the deadline that was set for it.
        let period = period_ticks_for_erpm(self.commanded_erpm.max(1), self.config.tick_hz).max(1);
        let deadline = *self.next_commutation.get_or_insert(tick + period);

        if tick >= deadline {
            self.advance_step();

            let target = self.config.sync_target_erpm;
            if self.commanded_erpm < target {
                self.commanded_erpm =
                    (self.commanded_erpm + self.config.sync_ramp_erpm_per_step).min(target);
            } else if self.commanded_erpm > target {
                self.commanded_erpm = self
                    .commanded_erpm
                    .saturating_sub(self.config.sync_ramp_erpm_per_step)
                    .max(target);
            }

            let next_period =
                period_ticks_for_erpm(self.commanded_erpm.max(1), self.config.tick_hz).max(1);
            self.next_commutation = Some(tick + next_period);

            // Hand off once we've reached the sync target *and* BEMF timing
            // is actually trustworthy - reaching the target rate alone
            // isn't enough if real zero-crossings never showed up.
            if self.commanded_erpm >= target && self.period_estimate_trusted {
                self.phase = RunPhase::Running;
                self.target_duty = self.config.sync_max_duty;
                if let Some(measured_period) = self.period_estimate {
                    self.next_commutation = Some(tick + measured_period);
                }
                return Output {
                    step: self.step,
                    duty: self.target_duty,
                    phase: self.phase,
                };
            }
        }

        let target = self.config.sync_target_erpm;
        let span = target.saturating_sub(self.config.sync_start_erpm).max(1);
        let progress = (self.commanded_erpm.saturating_sub(self.config.sync_start_erpm) as f32
            / span as f32)
            .min(1.0);
        let duty = self.config.sync_start_duty
            + (self.config.sync_max_duty - self.config.sync_start_duty) * progress;

        Output {
            step: self.step,
            duty,
            phase: self.phase,
        }
    }

    fn poll_running(&mut self, tick: u64) -> Output {
        if let (Some(last_zc), Some(period)) = (self.last_zero_cross, self.period_estimate) {
            let timeout = period.saturating_mul(self.config.stall_multiplier as u64);
            if tick.saturating_sub(last_zc) > timeout {
                self.phase = RunPhase::Stalled;
                return Output {
                    step: self.step,
                    duty: 0.0,
                    phase: RunPhase::Stalled,
                };
            }
        }

        if let Some(deadline) = self.next_commutation {
            if tick >= deadline {
                self.advance_step();
                if let Some(period) = self.period_estimate {
                    self.next_commutation = Some(deadline + period);
                }
            }
        }

        Output {
            step: self.step,
            duty: self.target_duty,
            phase: RunPhase::Running,
        }
    }
}

/// PWM switching-frequency scheduling (not part of `Commutator` itself -
/// this is pure interpolation math shared by both motors' hardware glue).
/// Linear between `min_khz` at `min_erpm` and `max_khz` at `max_erpm`,
/// clamped flat outside that range. Feed it `Commutator::current_erpm()` so
/// it has a live estimate throughout sync, not just once Running.
#[derive(Debug, Clone, Copy)]
pub struct PwmFrequencySchedule {
    pub min_erpm: u32,
    pub max_erpm: u32,
    pub min_khz: u32,
    pub max_khz: u32,
}

impl PwmFrequencySchedule {
    pub fn frequency_khz(&self, current_erpm: u32) -> u32 {
        if self.max_erpm <= self.min_erpm {
            return self.max_khz;
        }
        let clamped = current_erpm.clamp(self.min_erpm, self.max_erpm);
        let progress =
            (clamped - self.min_erpm) as f32 / (self.max_erpm - self.min_erpm) as f32;
        // Round-half-up via +0.5 then truncate: f32::round() needs libm,
        // which isn't available under true no_std (core alone doesn't
        // provide it) - this stays core-only. Always non-negative here
        // (min_khz/max_khz/progress all >= 0), so the "round toward zero"
        // caveat of this trick for negatives doesn't apply.
        (self.min_khz as f32 + (self.max_khz as f32 - self.min_khz as f32) * progress + 0.5) as u32
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn test_config() -> CommutatorConfig {
        CommutatorConfig {
            tick_hz: 1_000,
            sync_start_erpm: 100,
            sync_target_erpm: 400,
            sync_start_duty: 0.10,
            sync_max_duty: 1.0,
            sync_ramp_erpm_per_step: 100,
            stall_multiplier: 3,
        }
    }

    fn approx(a: f32, b: f32) -> bool {
        (a - b).abs() < 1e-4
    }

    #[test]
    fn step_pattern_table_has_exactly_one_floating_phase_per_step() {
        for step in 1..=6u8 {
            let p = step_pattern(step);
            let floats = [p.a, p.b, p.c]
                .iter()
                .filter(|s| **s == PhaseState::Float)
                .count();
            assert_eq!(floats, 1, "step {step} should float exactly one phase");
        }
    }

    #[test]
    fn startup_ramps_commanded_erpm_and_duty_towards_the_sync_target() {
        let mut c = Commutator::new(test_config());
        assert_eq!(c.current_erpm(), 100); // starts at sync_start_erpm

        // No real zero-cross ever arrives in this test, so it never hands
        // off - lets us observe the open-loop ramp in isolation.
        let out = c.poll(0);
        assert_eq!(out.phase, RunPhase::Startup);
        assert!(approx(out.duty, 0.10)); // commanded_erpm still at the start

        // commanded_erpm climbs by sync_ramp_erpm_per_step (100) each time
        // the current period's deadline is reached, not every poll() call.
        // Drive it forward until commanded_erpm has clearly advanced.
        let mut tick = 0u64;
        for _ in 0..2000 {
            tick += 1;
            c.poll(tick);
            if c.current_erpm() > 100 {
                break;
            }
        }
        assert!(c.current_erpm() > 100, "commanded erpm should have climbed");
        assert!(c.current_erpm() <= 400, "must not overshoot the sync target");
    }

    #[test]
    fn startup_never_hands_off_without_a_trusted_zero_cross() {
        let mut c = Commutator::new(test_config());
        // Drive far more ticks than the ramp could possibly need - with no
        // on_zero_cross() ever called, it must stay in Startup forever
        // rather than handing off on the open-loop schedule alone.
        let mut out = c.poll(1);
        for tick in 2..=100_000u64 {
            out = c.poll(tick);
        }
        assert_eq!(out.phase, RunPhase::Startup);
        // No real zero-cross was ever fed in, so electrical_rpm() (which
        // requires a trusted measurement) stays None - but the open-loop
        // commanded rate should still have climbed to and saturated at the
        // sync target (400, per test_config()).
        assert_eq!(c.electrical_rpm(), None);
        assert_eq!(c.current_erpm(), 400);
    }

    /// Runs the ramp to completion and returns (commutator, tick) at the
    /// moment it enters Running, so tests can drive further zero-cross
    /// scenarios through the public API only. Feeds a steady stream of
    /// zero-crosses at the *target* rate throughout (not trying to track
    /// the ramp's own instantaneous commanded rate - the ramp's step timing
    /// is entirely self-contained from commanded_erpm; the injected
    /// zero-crosses only need to (a) become trusted and (b) still be
    /// arriving once commanded_erpm reaches the target, both of which a
    /// steady rate satisfies regardless of exactly how they interleave with
    /// the ramp's own step advances).
    fn spun_up() -> (Commutator, u64) {
        let mut c = Commutator::new(test_config());
        // 1000 ticks, matching the period nearly every Running-phase test
        // scripts afterward - NOT derived from sync_target_erpm on purpose:
        // leaving behind a trusted period that matches what the next test
        // does avoids on_zero_cross's missed-beat rejection mistaking a
        // deliberate scenario change for corruption (see its doc comment).
        let period = 1_000u64;
        let mut next_fake_zc = period;
        let mut tick = 0u64;
        loop {
            tick += 1;
            let out = c.poll(tick);
            if tick >= next_fake_zc {
                c.on_zero_cross(tick);
                next_fake_zc += period;
            }
            if out.phase == RunPhase::Running {
                return (c, tick);
            }
            assert!(tick < 1_000_000, "spun_up() didn't converge - test bug");
        }
    }

    #[test]
    fn steady_period_commutates_at_half_period_after_each_zero_cross() {
        let (mut c, start) = spun_up();
        c.set_target_duty(0.5);

        let period = 1000u64;
        let mut zc = start + 200;

        // First real zero-cross only establishes a reference point.
        c.on_zero_cross(zc);

        for _ in 0..5 {
            zc += period;
            c.on_zero_cross(zc);

            // on_zero_cross schedules the commutation at zc + period / 2
            // (30 electrical degrees after *this* zero-cross).
            let commutation_tick = zc + period / 2;
            let step_before = c.poll(commutation_tick - 1).step;
            let out_at = c.poll(commutation_tick);
            assert_eq!(out_at.step, if step_before == 6 { 1 } else { step_before + 1 });
            assert!(approx(out_at.duty, 0.5));
            assert_eq!(out_at.phase, RunPhase::Running);
        }
    }

    #[test]
    fn electrical_rpm_matches_the_measured_period() {
        let (mut c, start) = spun_up();
        let mut zc = start + 200;
        c.on_zero_cross(zc);
        zc += 1000;
        c.on_zero_cross(zc); // period_estimate now trusted at 1000 ticks

        // 1kHz ticks, 1000-tick zero-cross period -> 1 zero-cross/sec ->
        // 1 electrical rev per 6 seconds -> 10 electrical RPM.
        assert_eq!(c.electrical_rpm(), Some(10));
        assert_eq!(c.current_erpm(), 10);
    }

    #[test]
    fn accelerating_period_reschedules_commutation_to_the_new_shorter_period() {
        let (mut c, start) = spun_up();

        let mut zc = start + 200;
        let mut period = 1000u64;
        c.on_zero_cross(zc);

        for _ in 0..4 {
            period = period * 9 / 10; // 10% faster each step
            zc += period;
            c.on_zero_cross(zc);

            let expected_commutation = zc + period / 2;
            let step_before = c.poll(expected_commutation - 1).step;
            let out = c.poll(expected_commutation);
            assert_eq!(out.step, if step_before == 6 { 1 } else { step_before + 1 });
        }
    }

    #[test]
    fn one_dropped_zero_cross_does_not_corrupt_the_period_estimate() {
        let (mut c, start) = spun_up();
        let period = 1000u64;
        let mut zc = start + 200;

        c.on_zero_cross(zc);
        for _ in 0..3 {
            zc += period;
            c.on_zero_cross(zc);
        }

        // The 4th zero-cross never arrives; the 5th shows up two periods
        // late, at 2x the established interval.
        let missed_zc = zc + period;
        let recovered_zc = missed_zc + period;
        c.on_zero_cross(recovered_zc);

        // The predictive schedule should still have fired a commutation
        // once around the missed interval, on the old period estimate...
        let step_before_gap = c.poll(missed_zc + period / 2 - 1).step;
        let out_during_gap = c.poll(missed_zc + period / 2);
        assert_eq!(
            out_during_gap.step,
            if step_before_gap == 6 { 1 } else { step_before_gap + 1 }
        );

        // ...and after the late zero-cross is accepted, the *next*
        // commutation is still scheduled off the original ~1000-tick
        // period, not the ~2000-tick gap that was measured.
        let next_expected = recovered_zc + period / 2;
        let step_before = c.poll(next_expected - 1).step;
        let out = c.poll(next_expected);
        assert_eq!(out.step, if step_before == 6 { 1 } else { step_before + 1 });
    }

    #[test]
    fn extended_silence_declares_a_stall_and_zeroes_duty() {
        let (mut c, start) = spun_up();
        c.set_target_duty(0.7);

        let period = 1000u64;
        let mut zc = start + 200;
        c.on_zero_cross(zc);
        for _ in 0..2 {
            zc += period;
            c.on_zero_cross(zc);
        }

        // Nothing but polling from here on - well past
        // stall_multiplier (3) * period.
        let out = c.poll(zc + period * 4);
        assert_eq!(out.phase, RunPhase::Stalled);
        assert!(approx(out.duty, 0.0));
        assert_eq!(c.current_erpm(), 0);

        // Stays stalled, doesn't recover on its own.
        let out = c.poll(zc + period * 10);
        assert_eq!(out.phase, RunPhase::Stalled);
    }

    #[test]
    fn zero_cross_is_ignored_once_stalled() {
        let (mut c, start) = spun_up();
        let period = 1000u64;
        let mut zc = start + 200;
        c.on_zero_cross(zc);
        zc += period;
        c.on_zero_cross(zc);

        c.poll(zc + period * 10); // force a stall
        assert_eq!(c.phase(), RunPhase::Stalled);

        c.on_zero_cross(zc + period * 10 + 1);
        let out = c.poll(zc + period * 10 + 2);
        assert_eq!(out.phase, RunPhase::Stalled);
    }

    #[test]
    fn expected_rising_edge_alternates_per_step() {
        // Hand-derived from STEP_TABLE: whichever phase floats now is driven
        // by the *next* step, so the expected direction alternates.
        let expected = [false, true, false, true, false, true];
        for step in 1..=6u8 {
            assert_eq!(
                expected_rising_edge(step),
                expected[(step - 1) as usize],
                "step {step}"
            );
        }
    }

    #[test]
    fn floating_phase_matches_step_table() {
        assert_eq!(floating_phase(1), Phase::C);
        assert_eq!(floating_phase(2), Phase::B);
        assert_eq!(floating_phase(3), Phase::A);
        assert_eq!(floating_phase(4), Phase::C);
        assert_eq!(floating_phase(5), Phase::B);
        assert_eq!(floating_phase(6), Phase::A);
    }

    #[test]
    fn zero_cross_detector_ignores_first_sample_on_a_new_step() {
        let mut d = ZeroCrossDetector::new();
        // Step 2 expects a rising edge; starting below neutral shouldn't
        // fire just because there's no prior reference yet.
        assert!(!d.check(2, 1000, 2048));
    }

    #[test]
    fn zero_cross_detector_fires_on_the_expected_direction() {
        let mut d = ZeroCrossDetector::new();
        assert!(!d.check(2, 1000, 2048)); // below neutral, establishes reference
        assert!(!d.check(2, 1500, 2048)); // still below neutral, no crossing
        assert!(d.check(2, 2500, 2048)); // crosses above -> rising, as expected for step 2
    }

    #[test]
    fn zero_cross_detector_rejects_the_wrong_direction() {
        let mut d = ZeroCrossDetector::new();
        // Step 1 expects a falling edge (see expected_rising_edge_alternates_per_step);
        // a rising crossing is noise, not a valid commutation event.
        assert!(!d.check(1, 1000, 2048)); // below neutral, establishes reference
        assert!(!d.check(1, 1500, 2048)); // still below, no crossing
        assert!(!d.check(1, 2500, 2048)); // crosses above -> rising, wrong direction for step 1
    }

    #[test]
    fn zero_cross_detector_fires_on_a_falling_edge_when_expected() {
        let mut d = ZeroCrossDetector::new();
        // Step 1 expects a falling edge.
        assert!(!d.check(1, 2500, 2048)); // above neutral, establishes reference
        assert!(!d.check(1, 3000, 2048)); // still above, no crossing
        assert!(d.check(1, 1000, 2048)); // crosses below -> falling, as expected for step 1
    }

    #[test]
    fn zero_cross_detector_resets_reference_on_step_change() {
        let mut d = ZeroCrossDetector::new();
        assert!(!d.check(1, 2500, 2048)); // above neutral on step 1
        // Step changes to 2 before a crossing was ever seen on step 1 - the
        // old "above neutral" reading must not be compared against step 2's
        // first sample even though it's now below neutral.
        assert!(!d.check(2, 1000, 2048));
    }

    // M3: two motors, each with its own Commutator/ZeroCrossDetector - these
    // guard that independence, so a future refactor that accidentally
    // reaches for shared/static state gets caught here instead of on
    // hardware where it'd show up as motor A's stall dragging motor B down.
    #[test]
    fn two_commutators_do_not_share_state() {
        let (mut a, start_a) = spun_up();
        let (mut b, start_b) = spun_up();

        a.set_target_duty(0.25);
        b.set_target_duty(0.75);

        let period = 1000u64;
        let mut zc_a = start_a + 200;
        let mut zc_b = start_b + 300; // deliberately different phase offset

        for _ in 0..3 {
            zc_a += period;
            a.on_zero_cross(zc_a);
            zc_b += period;
            b.on_zero_cross(zc_b);
        }

        // Starve A of further zero-crosses until it stalls, while B keeps
        // getting fed normally. If the two instances shared any state, B
        // would be dragged into A's stall (or vice versa).
        let out_a = a.poll(zc_a + period * 20);
        assert_eq!(out_a.phase, RunPhase::Stalled);
        assert!(approx(out_a.duty, 0.0));

        zc_b += period;
        b.on_zero_cross(zc_b);
        let out_b = b.poll(zc_b + period / 2);
        assert_eq!(out_b.phase, RunPhase::Running);
        assert!(approx(out_b.duty, 0.75));
    }

    #[test]
    fn two_zero_cross_detectors_do_not_share_state() {
        let mut a = ZeroCrossDetector::new();
        let mut b = ZeroCrossDetector::new();

        // Establish a's reference on step 1 (expects falling).
        assert!(!a.check(1, 2500, 2048));
        // b, on a *different* step (2, expects rising) with an unrelated
        // sample sequence, must not be influenced by a's internal state.
        assert!(!b.check(2, 1000, 2048));
        assert!(b.check(2, 2500, 2048)); // b's own rising crossing fires...
        assert!(a.check(1, 1000, 2048)); // ...independently of a's falling crossing firing here.
    }

    #[test]
    fn pwm_frequency_schedule_interpolates_linearly() {
        let schedule = PwmFrequencySchedule {
            min_erpm: 1_000,
            max_erpm: 5_000,
            min_khz: 48,
            max_khz: 96,
        };
        assert_eq!(schedule.frequency_khz(1_000), 48);
        assert_eq!(schedule.frequency_khz(5_000), 96);
        assert_eq!(schedule.frequency_khz(3_000), 72); // halfway
    }

    #[test]
    fn pwm_frequency_schedule_clamps_outside_its_range() {
        let schedule = PwmFrequencySchedule {
            min_erpm: 1_000,
            max_erpm: 5_000,
            min_khz: 48,
            max_khz: 96,
        };
        assert_eq!(schedule.frequency_khz(0), 48);
        assert_eq!(schedule.frequency_khz(50_000), 96);
    }
}
