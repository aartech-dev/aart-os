//! Drag-brake ("Latvian brake") supervisor - DESIGN.md section 7.5.
//!
//! Slot car motors need active braking, not just coasting, when the car
//! loses electrical contact with the track: either track power actually
//! went off, or the car left the slot/derailed (e.g. airborne off a jump).
//! In both cases the wheels would otherwise keep freewheeling from
//! momentum with no braking at all (`Commutator`'s existing `Stalled`
//! response, and `Bridge::disable`, are pure Hi-Z coast) - by the time the
//! car lands back on the track or gets put back in the slot, the wheels
//! are spinning at the wrong speed relative to the track, which is
//! exactly the traction-loss moment this is meant to avoid. Dead-shorting
//! the motor's phases (see `stm32_os::motor::Bridge::brake`) converts that
//! kinetic energy into heat instead, stopping the wheels quickly.
//!
//! This is a **whole-car** concern, not a per-motor one: what's being
//! detected is whether the track is still delivering power to the car at
//! all, measured once at the shared input (a dedicated current-sense
//! amplifier + shunt on the main supply rail, ahead of where it splits to
//! the two motor bridges - DESIGN.md section 6.5), not each motor's own
//! draw. So there is exactly one `DragBrakeSupervisor` for the whole car,
//! and its decision applies to both motors' `Commutator`s together.
//!
//! Detection is a single boolean in - `current_present` - and this module
//! doesn't know or care whether that came from an ADC threshold in
//! software or a dedicated hardware comparator; `eligible` is likewise
//! just a bool the caller computes however it wants (see `update`) - this
//! keeps the module decoupled from `Commutator`/`RunPhase` entirely.

/// Turns a raw bidirectional current-sense ADC sample into a
/// `current_present` bool for `DragBrakeSupervisor::update`.
///
/// The shared track-current amplifier (DESIGN.md section 6.5) is
/// bidirectional, not a simple unipolar shunt reading: current flows one
/// way while driving/accelerating (positive) and can flow the other way
/// under braking/regenerative conditions (negative) - a real track-power-
/// loss or derailment collapses the reading to zero from *either* side,
/// so what matters is the *magnitude* of the deviation from the
/// amplifier's zero-current reference point (`zero_offset`, typically its
/// mid-scale output), not the raw sample's absolute value. `min_magnitude`
/// is how far from that reference a reading has to be to still count as
/// "current present."
pub fn current_present(sample: u16, zero_offset: u16, min_magnitude: u16) -> bool {
    let deviation = (sample as i32 - zero_offset as i32).unsigned_abs();
    deviation >= min_magnitude as u32
}

#[derive(Debug, Clone, Copy)]
pub struct DragBrakeConfig {
    /// Consecutive "no current" ticks required before engaging - guards
    /// against a single noisy/glitched reading false-triggering a brake
    /// event mid-run. 1 reacts on the very first bad tick.
    ///
    /// Note: the brake *duty* actually applied isn't configured here - it
    /// lives on `CommutatorConfig::drag_brake_duty` (read by `poll()` while
    /// `Braking`), since `DragBrakeSupervisor` only ever decides *whether*
    /// to engage, never *how hard* - keeping this struct free of a field
    /// it would never itself read.
    pub debounce_ticks: u16,
}

pub struct DragBrakeSupervisor {
    config: DragBrakeConfig,
    consecutive_no_current: u16,
    engaged: bool,
}

impl DragBrakeSupervisor {
    pub fn new(config: DragBrakeConfig) -> Self {
        Self {
            config,
            consecutive_no_current: 0,
            engaged: false,
        }
    }

    /// Call once per tick with whether this reading is meaningful right
    /// now (`eligible`) and whether current is present. Returns whether
    /// the brake should be (or remain) engaged - the caller is
    /// responsible for actually calling both motors'
    /// `Commutator::engage_drag_brake`/`release_drag_brake` on a
    /// rising/falling edge of this, since only the caller has `&mut
    /// Commutator`s to call them on.
    ///
    /// `eligible` should be false whenever a low reading wouldn't mean
    /// anything yet - e.g. before either motor has finished its open-loop
    /// sync ramp, track current legitimately can be low even with power
    /// genuinely present. The caller decides that from its own state
    /// (typically "is at least one motor `Running`"); this module just
    /// reacts to the bool.
    pub fn update(&mut self, eligible: bool, current_present: bool) -> bool {
        if current_present {
            self.consecutive_no_current = 0;
            self.engaged = false;
            return self.engaged;
        }

        if !eligible {
            self.consecutive_no_current = 0;
            return self.engaged;
        }

        self.consecutive_no_current = self.consecutive_no_current.saturating_add(1);
        if self.consecutive_no_current >= self.config.debounce_ticks.max(1) {
            self.engaged = true;
        }
        self.engaged
    }

    pub fn is_engaged(&self) -> bool {
        self.engaged
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cfg(debounce_ticks: u16) -> DragBrakeConfig {
        DragBrakeConfig { debounce_ticks }
    }

    #[test]
    fn never_engages_while_ineligible_no_matter_how_long_current_is_absent() {
        let mut s = DragBrakeSupervisor::new(cfg(1));
        for _ in 0..10_000 {
            assert!(!s.update(false, false));
        }
    }

    #[test]
    fn engages_after_debounce_ticks_of_no_current_while_eligible() {
        let mut s = DragBrakeSupervisor::new(cfg(3));
        assert!(!s.update(true, false));
        assert!(!s.update(true, false));
        assert!(s.update(true, false));
        assert!(s.is_engaged());
    }

    #[test]
    fn a_single_current_present_tick_resets_the_debounce_counter() {
        let mut s = DragBrakeSupervisor::new(cfg(3));
        assert!(!s.update(true, false));
        assert!(!s.update(true, false));
        // Current blips back before the 3rd consecutive absent tick -
        // must not count towards engaging.
        assert!(!s.update(true, true));
        assert!(!s.update(true, false));
        assert!(!s.update(true, false));
        assert!(!s.is_engaged(), "counter should have reset, not carried over");
    }

    #[test]
    fn current_returning_immediately_releases_the_brake() {
        let mut s = DragBrakeSupervisor::new(cfg(1));
        assert!(s.update(true, false));
        assert!(s.is_engaged());
        assert!(!s.update(true, true));
        assert!(!s.is_engaged());
    }

    #[test]
    fn stays_engaged_while_still_eligible_and_current_is_still_absent() {
        let mut s = DragBrakeSupervisor::new(cfg(1));
        assert!(s.update(true, false));
        assert!(s.update(true, false));
        assert!(s.update(true, false));
        assert!(s.is_engaged());
    }

    #[test]
    fn debounce_of_one_reacts_on_the_first_absent_tick() {
        let mut s = DragBrakeSupervisor::new(cfg(1));
        assert!(s.update(true, false));
    }

    #[test]
    fn current_present_is_false_exactly_at_the_zero_offset() {
        assert!(!current_present(2048, 2048, 50));
    }

    #[test]
    fn current_present_detects_positive_driving_current() {
        // Well above the zero-current reference - normal accelerating draw.
        assert!(current_present(2048 + 500, 2048, 50));
    }

    #[test]
    fn current_present_detects_negative_braking_current() {
        // Well below the zero-current reference - regenerative/braking
        // current flowing the other way through the sense resistor. Must
        // be treated the same as the positive case, not ignored.
        assert!(current_present(2048 - 500, 2048, 50));
    }

    #[test]
    fn current_present_is_false_within_the_deadband_on_either_side() {
        assert!(!current_present(2048 + 20, 2048, 50));
        assert!(!current_present(2048 - 20, 2048, 50));
    }

    #[test]
    fn current_present_treats_the_deadband_boundary_as_present() {
        assert!(current_present(2048 + 50, 2048, 50));
        assert!(current_present(2048 - 50, 2048, 50));
    }

    #[test]
    fn current_present_does_not_overflow_at_the_sample_range_extremes() {
        // sample=0 with a high zero_offset, and sample=u16::MAX with a low
        // zero_offset, are the largest possible deviations - must not panic.
        assert!(current_present(0, 4095, 1));
        assert!(current_present(u16::MAX, 0, 1));
    }
}
