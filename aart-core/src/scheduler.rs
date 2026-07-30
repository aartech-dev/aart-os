//! Cyclic executive: a fixed table of periodic tasks driven by an external tick source.
//!
//! Hard-real-time work (motor commutation) lives directly in interrupt handlers, not
//! here — this only schedules the slower, non-time-critical work (telemetry, the
//! differential control loop, fault polling). See DESIGN.md section 5.

/// A task run every `period_ticks` ticks. `run` receives the current tick count.
pub struct Task {
    pub period_ticks: u32,
    pub run: fn(u64),
}

/// Fixed-size table of `N` tasks, advanced one tick at a time by an external tick
/// source (e.g. a 1kHz SysTick interrupt feeding ticks into the main loop).
pub struct Scheduler<const N: usize> {
    tasks: [Task; N],
    tick: u64,
}

impl<const N: usize> Scheduler<N> {
    pub const fn new(tasks: [Task; N]) -> Self {
        Self { tasks, tick: 0 }
    }

    /// Advance by one tick, running every task whose period has elapsed.
    /// A task with `period_ticks == 0` is disabled and never runs.
    pub fn tick(&mut self) {
        self.tick += 1;
        for task in &self.tasks {
            if task.period_ticks != 0 && self.tick % task.period_ticks as u64 == 0 {
                (task.run)(self.tick);
            }
        }
    }

    pub fn current_tick(&self) -> u64 {
        self.tick
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::sync::atomic::{AtomicU32, Ordering};

    static FIRES_A: AtomicU32 = AtomicU32::new(0);
    fn record_a(_tick: u64) {
        FIRES_A.fetch_add(1, Ordering::SeqCst);
    }

    #[test]
    fn fires_every_period() {
        let mut sched = Scheduler::new([Task {
            period_ticks: 4,
            run: record_a,
        }]);
        for _ in 0..16 {
            sched.tick();
        }
        assert_eq!(FIRES_A.load(Ordering::SeqCst), 4);
    }

    static FIRES_B1: AtomicU32 = AtomicU32::new(0);
    static FIRES_B2: AtomicU32 = AtomicU32::new(0);
    fn record_b1(_tick: u64) {
        FIRES_B1.fetch_add(1, Ordering::SeqCst);
    }
    fn record_b2(_tick: u64) {
        FIRES_B2.fetch_add(1, Ordering::SeqCst);
    }

    #[test]
    fn independent_periods_do_not_interfere() {
        let mut sched = Scheduler::new([
            Task {
                period_ticks: 1,
                run: record_b1,
            },
            Task {
                period_ticks: 5,
                run: record_b2,
            },
        ]);
        for _ in 0..10 {
            sched.tick();
        }
        assert_eq!(FIRES_B1.load(Ordering::SeqCst), 10);
        assert_eq!(FIRES_B2.load(Ordering::SeqCst), 2);
    }

    static FIRES_C: AtomicU32 = AtomicU32::new(0);
    fn record_c(_tick: u64) {
        FIRES_C.fetch_add(1, Ordering::SeqCst);
    }

    #[test]
    fn zero_period_task_never_fires() {
        let mut sched = Scheduler::new([Task {
            period_ticks: 0,
            run: record_c,
        }]);
        for _ in 0..100 {
            sched.tick();
        }
        assert_eq!(FIRES_C.load(Ordering::SeqCst), 0);
    }

    static FIRES_D: AtomicU32 = AtomicU32::new(0);
    fn record_d(tick: u64) {
        assert_eq!(tick % 3, 0);
        FIRES_D.fetch_add(1, Ordering::SeqCst);
    }

    #[test]
    fn task_receives_the_tick_it_fired_on() {
        let mut sched = Scheduler::new([Task {
            period_ticks: 3,
            run: record_d,
        }]);
        for _ in 0..9 {
            sched.tick();
        }
        assert_eq!(FIRES_D.load(Ordering::SeqCst), 3);
    }

    #[test]
    fn current_tick_tracks_ticks_advanced() {
        let mut sched: Scheduler<0> = Scheduler::new([]);
        for _ in 0..7 {
            sched.tick();
        }
        assert_eq!(sched.current_tick(), 7);
    }
}
