//! Fault aggregation (DESIGN.md M6): overcurrent, stall, and UART
//! comms-loss, rolled up into one status the hardware glue in `stm32_os`
//! acts on (disabling a faulted motor's bridge, zeroing `steer_cmd` on
//! comms-loss) and one bit - `FaultStatus::all_healthy` - that gates
//! whether the IWDG hardware watchdog gets fed at all. If a fault persists
//! uncorrected, the watchdog stops being fed and the MCU eventually resets
//! itself as a last-resort recovery path, on top of (not instead of) the
//! immediate software mitigation.
//!
//! Deliberately doesn't know about `Commutator` or ADC sampling directly -
//! it takes plain values (a raw current sample, a stalled bool) so it stays
//! composable/host-testable, same reasoning as `SlipEstimator` in
//! `diff_ctrl.rs`.

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FaultConfig {
    /// A current sample (raw ADC counts) at or above this is overcurrent.
    pub current_limit: u16,
    /// Fail-safe (zero steer_cmd) if no valid command has parsed within
    /// this many ticks of the last one - not counted from boot, since
    /// never having received a command yet isn't the same as having lost
    /// an established link.
    pub comms_timeout_ticks: u64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct FaultStatus {
    pub overcurrent_a: bool,
    pub overcurrent_b: bool,
    pub stalled_a: bool,
    pub stalled_b: bool,
    pub comms_lost: bool,
}

impl FaultStatus {
    /// The only condition under which the watchdog should be fed
    /// (DESIGN.md: "IWDG kicked only when all tasks healthy").
    pub fn all_healthy(&self) -> bool {
        !(self.overcurrent_a
            || self.overcurrent_b
            || self.stalled_a
            || self.stalled_b
            || self.comms_lost)
    }
}

pub struct FaultSupervisor {
    config: FaultConfig,
    last_valid_command_tick: Option<u64>,
}

impl FaultSupervisor {
    pub fn new(config: FaultConfig) -> Self {
        Self {
            config,
            last_valid_command_tick: None,
        }
    }

    /// Call whenever a command line parses successfully.
    pub fn note_valid_command(&mut self, tick: u64) {
        self.last_valid_command_tick = Some(tick);
    }

    pub fn is_overcurrent(&self, sample: u16) -> bool {
        sample >= self.config.current_limit
    }

    fn comms_lost(&self, tick: u64) -> bool {
        match self.last_valid_command_tick {
            None => false,
            Some(last) => tick.saturating_sub(last) > self.config.comms_timeout_ticks,
        }
    }

    /// Call every tick with each motor's raw current sample and current
    /// stall status (`Commutator::phase() == RunPhase::Stalled`) to get the
    /// aggregate fault picture.
    pub fn evaluate(
        &self,
        tick: u64,
        current_a: u16,
        current_b: u16,
        stalled_a: bool,
        stalled_b: bool,
    ) -> FaultStatus {
        FaultStatus {
            overcurrent_a: self.is_overcurrent(current_a),
            overcurrent_b: self.is_overcurrent(current_b),
            stalled_a,
            stalled_b,
            comms_lost: self.comms_lost(tick),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn test_config() -> FaultConfig {
        FaultConfig {
            current_limit: 3000,
            comms_timeout_ticks: 1000,
        }
    }

    #[test]
    fn overcurrent_trips_at_or_above_the_limit() {
        let s = FaultSupervisor::new(test_config());
        assert!(!s.is_overcurrent(2999));
        assert!(s.is_overcurrent(3000));
        assert!(s.is_overcurrent(3001));
    }

    #[test]
    fn evaluate_reports_per_motor_overcurrent_independently() {
        let s = FaultSupervisor::new(test_config());
        let status = s.evaluate(0, 3500, 100, false, false);
        assert!(status.overcurrent_a);
        assert!(!status.overcurrent_b);
        assert!(!status.all_healthy());
    }

    #[test]
    fn evaluate_reports_stall_flags_passed_in_directly() {
        let s = FaultSupervisor::new(test_config());
        let status = s.evaluate(0, 0, 0, true, false);
        assert!(status.stalled_a);
        assert!(!status.stalled_b);
        assert!(!status.all_healthy());
    }

    #[test]
    fn comms_never_established_is_not_comms_lost() {
        let s = FaultSupervisor::new(test_config());
        let status = s.evaluate(1_000_000, 0, 0, false, false);
        assert!(!status.comms_lost);
        assert!(status.all_healthy());
    }

    #[test]
    fn comms_lost_after_timeout_since_the_last_valid_command() {
        let mut s = FaultSupervisor::new(test_config());
        s.note_valid_command(100);

        let status = s.evaluate(100 + 1000, 0, 0, false, false);
        assert!(!status.comms_lost, "exactly at the timeout, not yet lost");

        let status = s.evaluate(100 + 1001, 0, 0, false, false);
        assert!(status.comms_lost);
        assert!(!status.all_healthy());
    }

    #[test]
    fn a_fresh_command_clears_comms_lost() {
        let mut s = FaultSupervisor::new(test_config());
        s.note_valid_command(100);
        assert!(s.evaluate(5000, 0, 0, false, false).comms_lost);

        s.note_valid_command(5000);
        assert!(!s.evaluate(5100, 0, 0, false, false).comms_lost);
    }

    #[test]
    fn all_healthy_is_false_if_any_single_flag_is_set() {
        let healthy = FaultStatus::default();
        assert!(healthy.all_healthy());

        let cases = [
            FaultStatus {
                overcurrent_a: true,
                ..Default::default()
            },
            FaultStatus {
                overcurrent_b: true,
                ..Default::default()
            },
            FaultStatus {
                stalled_a: true,
                ..Default::default()
            },
            FaultStatus {
                stalled_b: true,
                ..Default::default()
            },
            FaultStatus {
                comms_lost: true,
                ..Default::default()
            },
        ];
        for case in cases {
            assert!(!case.all_healthy(), "{case:?}");
        }
    }
}
