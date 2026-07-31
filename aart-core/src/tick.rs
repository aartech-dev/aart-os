//! Extends a free-running 32-bit hardware counter (e.g. the Cortex-M DWT
//! cycle counter) into a monotonic 64-bit tick, for the ISR-driven fast
//! commutation path (DESIGN.md's long-flagged "wire the fast ISR-driven
//! path" item from M2 onward).
//!
//! The 1kHz SysTick tick used everywhere else in this system (the cyclic
//! executive, `Commutator`'s tick parameter up through M6) is a placeholder
//! nowhere near fine-grained enough for real commutation timing at these
//! motors' electrical rates - a 32-bit cycle counter running at the core
//! clock (16MHz+ once the timer-frequency work's clock-resolution note is
//! addressed) gives nanosecond-scale resolution, but it wraps in seconds,
//! not never, so something has to extend it.

/// Extends 32-bit hardware-counter readings into a monotonic 64-bit tick by
/// detecting wraparound (a new reading numerically less than the last one
/// means the counter wrapped since the last call). Must be called often
/// enough that at most one wraparound can occur between calls - true by
/// construction for an ADC-ISR-driven caller firing every PWM period, many
/// orders of magnitude faster than a 32-bit counter wraps at any clock
/// speed this chip runs at.
pub struct TickExtender {
    high: u32,
    last_low: u32,
}

impl TickExtender {
    pub const fn new() -> Self {
        Self {
            high: 0,
            last_low: 0,
        }
    }

    pub fn extend(&mut self, raw: u32) -> u64 {
        if raw < self.last_low {
            self.high += 1;
        }
        self.last_low = raw;
        ((self.high as u64) << 32) | raw as u64
    }
}

impl Default for TickExtender {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn extends_monotonically_within_one_wrap_cycle() {
        let mut e = TickExtender::new();
        assert_eq!(e.extend(0), 0);
        assert_eq!(e.extend(100), 100);
        assert_eq!(e.extend(u32::MAX), u32::MAX as u64);
    }

    #[test]
    fn detects_a_wraparound_and_keeps_increasing() {
        let mut e = TickExtender::new();
        assert_eq!(e.extend(u32::MAX - 1), u32::MAX as u64 - 1);
        // Counter wrapped past u32::MAX back to a small value.
        let wrapped = e.extend(5);
        assert_eq!(wrapped, (1u64 << 32) | 5);
        assert!(wrapped > u32::MAX as u64 - 1, "must keep increasing across a wrap");
    }

    #[test]
    fn handles_several_consecutive_wraps() {
        let mut e = TickExtender::new();
        let mut previous = e.extend(0);
        for wrap in 1..=3u64 {
            // Simulate the counter climbing to near-max then wrapping,
            // once per iteration.
            let before_wrap = e.extend(u32::MAX - 1);
            assert!(before_wrap > previous);
            let after_wrap = e.extend(0);
            assert!(after_wrap > before_wrap);
            assert_eq!(after_wrap, wrap << 32);
            previous = after_wrap;
        }
    }

    #[test]
    fn repeated_identical_readings_do_not_spuriously_advance_high() {
        let mut e = TickExtender::new();
        assert_eq!(e.extend(42), 42);
        // Same reading again (e.g. ISR fired twice before the counter
        // ticked forward, if it were ever slower than the ISR - shouldn't
        // happen in practice, but must not be misread as a wrap).
        assert_eq!(e.extend(42), 42);
    }
}
