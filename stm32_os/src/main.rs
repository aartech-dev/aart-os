#![no_std]
#![no_main]

mod command;
mod motor;
mod sense;

use core::cell::RefCell;
use core::panic::PanicInfo;
use core::sync::atomic::{AtomicU32, Ordering};

use cortex_m::interrupt::{free, Mutex};
use cortex_m::peripheral::scb::SystemHandler;
use cortex_m::peripheral::syst::SystClkSource;
use cortex_m::peripheral::DWT;
use cortex_m_rt::{entry, exception};

use stm32g4xx_hal::gpio::GpioExt;
use stm32g4xx_hal::independent_watchdog::IndependentWatchdog;
use stm32g4xx_hal::pwr::PwrExt;
use stm32g4xx_hal::rcc::{Config, PllConfig, PllMDiv, PllNMul, PllRDiv, PllSrc, RccExt};
use stm32g4xx_hal::stm32::{interrupt, Interrupt};
use stm32g4xx_hal::time::ExtU32;

use aart_core::commutator::{
    floating_phase, step_pattern, Commutator, CommutatorConfig, RunPhase, ZeroCrossDetector,
};
use aart_core::diff_ctrl::{BemfSlipEstimator, DiffController, DiffControllerConfig};
use aart_core::fault::{FaultConfig, FaultSupervisor};
use aart_core::protocol::{format_error, format_telemetry, parse_line, Command, LineReader};
use aart_core::scheduler::Scheduler;
use aart_core::tick::TickExtender;
use motor::BridgeControl;
use sense::SenseIsr;

// ---------------------------------------------------------------------------
// ISR-owned per-motor state.
//
// The fast path (six-step commutation: BEMF sensing, zero-cross detection,
// stepping, applying duty) lives entirely in the ADC1_2 interrupt, triggered
// by each motor's own ADC hardware (TRGO-synced to its PWM period - tens of
// microseconds at these motors' electrical rates, nowhere near the 1kHz
// SysTick tick used everywhere else). That's the long-flagged "wire the fast
// ISR-driven path" item from M2 onward (DESIGN.md section 5).
//
// The slow 1kHz main loop still owns everything cross-cutting: the
// differential controller, fault supervision, PWM-frequency scheduling,
// telemetry, and UART - all reading/writing this same state through a
// critical section (`with_motor`), same pattern M0's scheduler used for
// its LED before that got replaced by real motor control.
// ---------------------------------------------------------------------------

struct MotorIsrState<Br, S> {
    commutator: Commutator,
    zero_cross: ZeroCrossDetector,
    bridge: Br,
    sense: S,
    tick_extender: TickExtender,
    /// Cycles through sampling the floating phase (most of the time),
    /// neutral, and current on a fixed schedule - see step_motor. 16 is a
    /// placeholder split (14/16 floating, 1/16 each of neutral/current),
    /// not derived from real timing measurements.
    rotation: u8,
    latest_neutral: u16,
    latest_current: u16,
}

const ROTATION_LEN: u8 = 16;
const NEUTRAL_SLOT: u8 = 0;
const CURRENT_SLOT: u8 = 8;

impl<Br, S> MotorIsrState<Br, S> {
    fn new(commutator: Commutator, bridge: Br, sense: S) -> Self {
        Self {
            commutator,
            zero_cross: ZeroCrossDetector::new(),
            bridge,
            sense,
            tick_extender: TickExtender::new(),
            rotation: 0,
            latest_neutral: 0,
            latest_current: 0,
        }
    }
}

type MotorAState = MotorIsrState<motor::MotorABridge, sense::MotorASense>;
type MotorBState = MotorIsrState<motor::MotorBBridge, sense::MotorBSense>;

static MOTOR_A: Mutex<RefCell<Option<MotorAState>>> = Mutex::new(RefCell::new(None));
static MOTOR_B: Mutex<RefCell<Option<MotorBState>>> = Mutex::new(RefCell::new(None));

/// Runs one ADC-triggered step for either motor - shared so the two
/// motors' fast paths can't drift out of sync with each other, the same
/// reasoning `drive_commutation` (M3) used before the ISR rework replaced
/// it with this.
fn step_motor<Br: BridgeControl, S: SenseIsr>(state: &mut MotorIsrState<Br, S>) {
    if !state.sense.eoc_pending() {
        return; // the *other* motor's ADC raised the shared interrupt
    }

    let tick = state.tick_extender.extend(DWT::cycle_count());
    let sample = state.sense.current_sample();
    state.sense.clear_eoc();

    match state.rotation {
        NEUTRAL_SLOT => state.latest_neutral = sample,
        CURRENT_SLOT => state.latest_current = sample,
        _ => {
            let step = state.commutator.step();
            if state.zero_cross.check(step, sample, state.latest_neutral) {
                state.commutator.on_zero_cross(tick);
            }
        }
    }

    let out = state.commutator.poll(tick);
    if out.phase == RunPhase::Stalled {
        // apply_step(..., 0) would still drive two phases low (0% duty is
        // a steady low, not Hi-Z) - a real safe-stop needs all three phases
        // actually disabled, which only disable() does.
        state.bridge.disable();
    } else {
        let duty_counts = (out.duty * state.bridge.max_duty() as f32) as u16;
        state.bridge.apply_step(step_pattern(out.step), duty_counts);
    }

    state.rotation = (state.rotation + 1) % ROTATION_LEN;
    match state.rotation {
        NEUTRAL_SLOT => state.sense.configure_neutral(),
        CURRENT_SLOT => state.sense.configure_current(),
        _ => state.sense.configure_phase(floating_phase(out.step)),
    }
    state.sense.rearm();
}

/// Runs `f` with exclusive access to a motor's ISR-owned state, via a
/// critical section - both the ADC1_2 ISR and the slow main loop reach this
/// same state, so anything touching it needs to be safe against the ISR
/// firing mid-access. Kept short at each call site: this briefly blocks the
/// real-time commutation path while held.
fn with_motor<T, R>(mutex: &Mutex<RefCell<Option<T>>>, f: impl FnOnce(&mut T) -> R) -> R {
    free(|cs| {
        let mut state_ref = mutex.borrow(cs).borrow_mut();
        f(state_ref
            .as_mut()
            .expect("motor ISR state installed before ADC1_2 is unmasked"))
    })
}

#[interrupt]
fn ADC1_2() {
    with_motor(&MOTOR_A, step_motor);
    with_motor(&MOTOR_B, step_motor);
}

// These are slot car motors: there is no throttle input, and once a motor
// hands off to closed-loop it runs at this duty permanently (all volts/
// current switched straight to the phases - "no PWM"). The differential
// controller is the only thing that ever pulls duty below this, and only
// for the cornering-slowed side. 1.0 = literally 100%.
const RUNNING_BASE_DUTY: f32 = 1.0;

// All the numbers below are placeholders pending real tuning on an actual
// 1106 motor/track - nothing here is derived from real motor characteristics
// yet. SYNC_TARGET_ERPM in particular (the eRPM sync ramps to before
// attempting handoff to BEMF sensing) needs to be wherever these motors'
// BEMF actually becomes reliably detectable, which isn't something to guess.
const SYNC_START_ERPM: u32 = 3_000;
const SYNC_TARGET_ERPM: u32 = 60_000;
// Nominal top speed, higher than SYNC_TARGET_ERPM: used only to scale PWM
// switching frequency (see pwm_frequency_schedule), not as an enforced
// cap - there's no closed-loop speed regulation at all in this design.
const TOP_RUNNING_ERPM: u32 = 150_000;

/// The DWT cycle counter (main.rs's fast tick source, see MotorIsrState)
/// runs at the core clock - `core_hz` is read from `rcc.clocks` at runtime
/// (main() now runs the PLL at 170MHz, not the HSI16 this was hardcoded to
/// before), not assumed as a constant, since Commutator's whole timing
/// model (period_ticks_for_erpm, electrical_rpm, stall windows) is only
/// correct if this actually matches what the DWT is really counting at.
fn motor_commutator_config(core_hz: u32) -> CommutatorConfig {
    CommutatorConfig {
        tick_hz: core_hz,
        sync_start_erpm: SYNC_START_ERPM,
        sync_target_erpm: SYNC_TARGET_ERPM,
        sync_start_duty: 0.15,
        sync_max_duty: RUNNING_BASE_DUTY,
        sync_ramp_erpm_per_step: 500,
        stall_multiplier: 8,
    }
}

/// PWM switching-frequency schedule: 48kHz at SYNC_START_ERPM, 96kHz by
/// TOP_RUNNING_ERPM, linear in between (extends past the sync ramp itself,
/// continuing to track real running speed once Running).
fn pwm_frequency_schedule() -> aart_core::commutator::PwmFrequencySchedule {
    aart_core::commutator::PwmFrequencySchedule {
        min_erpm: SYNC_START_ERPM,
        max_erpm: TOP_RUNNING_ERPM,
        min_khz: motor::PWM_FREQUENCY_MIN_KHZ,
        max_khz: motor::PWM_FREQUENCY_MAX_KHZ,
    }
}

// SysTick still drives the slow cyclic-executive-ish loop (UART, telemetry,
// differential control, fault supervision, watchdog) - just no longer the
// commutation path itself, which is now ADC1_2 above.
static PENDING_TICKS: AtomicU32 = AtomicU32::new(0);

const TICK_HZ: u32 = 1_000;
const TELEMETRY_PERIOD_TICKS: u64 = 1_000; // once per placeholder "second" (see TICK_HZ)

// M6 fault thresholds - placeholders pending real tuning, same caveat as
// the sync numbers above.
const OVERCURRENT_LIMIT: u16 = 3_500; // raw ADC counts (12-bit, max 4095)
const COMMS_TIMEOUT_TICKS: u64 = 1_000; // 1 placeholder "second" of silence, in SysTick ticks
const IWDG_TIMEOUT_MS: u32 = 200;

#[exception]
fn SysTick() {
    PENDING_TICKS.fetch_add(1, Ordering::Relaxed);
}

#[entry]
fn main() -> ! {
    let dp = stm32g4xx_hal::pac::Peripherals::take().unwrap();
    let mut cp = cortex_m::Peripherals::take().unwrap();

    // 170MHz via the PLL from HSI16 (M=/4 -> 4MHz VCO input, N=x85 ->
    // 340MHz VCO, R=/2 -> 170MHz) - the documented maximum for this whole
    // series, requiring Range1 boost mode (VOS0). Was plain HSI16/no-PLL
    // through M0-M6 (a real limitation, not a placeholder): both the PWM
    // duty-cycle resolution problem and the ADC-ISR CPU-budget question
    // flagged at the end of the ISR-driven-commutation work scale directly
    // with this clock, so this isn't just "faster for its own sake."
    let pwr_cfg = dp
        .PWR
        .constrain()
        .vos(stm32g4xx_hal::pwr::VoltageScale::Range1 { enable_boost: true })
        .freeze();
    let rcc_cfg = Config::pll()
        .pll_cfg(PllConfig {
            mux: PllSrc::HSI,
            m: PllMDiv::DIV_4,
            n: PllNMul::MUL_85,
            r: Some(PllRDiv::DIV_2),
            q: None,
            p: None,
        })
        .boost(true);
    let mut rcc = dp.RCC.freeze(rcc_cfg, pwr_cfg);
    let core_hz = rcc.clocks.sys_clk.raw();

    let gpioa = dp.GPIOA.split(&mut rcc);
    let gpiob = dp.GPIOB.split(&mut rcc);
    let gpiof = dp.GPIOF.split(&mut rcc);

    // Motor A bridge (TIM1) + sense (ADC1). Pin table: DESIGN.md section 6.1.
    let (_control_a, bridge_a) = motor::motor_a_bridge(
        dp.TIM1,
        (
            gpioa.pa8.into_alternate(),
            gpioa.pa9.into_alternate(),
            gpioa.pa10.into_alternate(),
        ),
        gpioa.pa7.into_alternate(),
        gpiob.pb0.into_alternate(),
        gpiof.pf0.into_alternate(),
        &mut rcc,
    );

    // Motor B bridge (TIM8) + sense (ADC2).
    let (_control_b, bridge_b) = motor::motor_b_bridge(
        dp.TIM8,
        (
            gpioa.pa15.into_alternate(),
            gpiob.pb8.into_alternate(),
            gpiob.pb9.into_alternate(),
        ),
        gpiob.pb3.into_alternate(),
        gpiob.pb4.into_alternate(),
        gpiob.pb5.into_alternate(),
        &mut rcc,
    );

    let adc_common = sense::claim_common(dp.ADC12_COMMON, &mut rcc);
    let mut delay = sense::CycleDelay::new(rcc.clocks.sys_clk.raw());

    let sense_a = sense::motor_a_sense(
        dp.ADC1,
        &adc_common,
        gpioa.pa0.into_analog(),
        gpioa.pa1.into_analog(),
        gpioa.pa3.into_analog(),
        gpiob.pb1.into_analog(),
        gpiob.pb11.into_analog(),
        gpiob.pb12.into_analog(),
        &mut delay,
    );

    let sense_b = sense::motor_b_sense(
        dp.ADC2,
        &adc_common,
        gpioa.pa4.into_analog(),
        gpioa.pa5.into_analog(),
        gpioa.pa6.into_analog(),
        gpiob.pb2.into_analog(),
        gpiof.pf1.into_analog(),
        &mut delay,
    );

    let mut command = command::command_channel(
        dp.USART1,
        gpiob.pb6.into_alternate(),
        gpiob.pb7.into_alternate(),
        &mut rcc,
    );

    // Both motors' PWM outputs start disabled (pwm_advanced().finalize()
    // leaves them that way) and get enabled per-phase by apply_step() (now
    // called from the ADC1_2 ISR, see step_motor) as soon as sync starts
    // driving real commutation - which happens immediately, unconditionally,
    // on power-up: there's no throttle input to wait for (see Commutator's
    // module doc comment). Two fully separate instances each - see
    // aart-core's two_commutators_do_not_share_state test for why that's the
    // thing actually worth asserting about "M3".
    free(|cs| {
        MOTOR_A.borrow(cs).replace(Some(MotorAState::new(
            Commutator::new(motor_commutator_config(core_hz)),
            bridge_a,
            sense_a,
        )));
        MOTOR_B.borrow(cs).replace(Some(MotorBState::new(
            Commutator::new(motor_commutator_config(core_hz)),
            bridge_b,
            sense_b,
        )));
    });

    // DWT cycle counter: the ISR's tick source (see MotorIsrState /
    // TickExtender) - far finer-grained than the 1kHz SysTick tick, which
    // stays reserved for the slow loop below.
    cp.DCB.enable_trace();
    cp.DWT.enable_cycle_counter();

    // ADC1_2 must preempt the slow loop's SysTick-driven work - motor state
    // is already installed above, so it's safe to unmask now. Priority
    // values are raw 8-bit (only the top 4 bits implemented on this part,
    // lower number = higher priority): ADC1_2 at the top, SysTick lowered
    // out of its way.
    unsafe {
        cp.SCB.set_priority(SystemHandler::SysTick, 0xF0);
        cp.NVIC.set_priority(Interrupt::ADC1_2, 0x00);
        cortex_m::peripheral::NVIC::unmask(Interrupt::ADC1_2);
    }

    let pwm_schedule = pwm_frequency_schedule();
    // Track the last frequency actually written per motor so set_frequency_hz
    // (a real PSC/ARR rewrite + duty rescale) only runs when the schedule's
    // output actually changes, not every single tick regardless.
    let mut last_freq_khz_a = motor::PWM_FREQUENCY_MIN_KHZ;
    let mut last_freq_khz_b = motor::PWM_FREQUENCY_MIN_KHZ;

    // slip_threshold/slip_gain are placeholders pending real tuning on
    // hardware, same as motor_commutator_config()'s sync numbers.
    let mut diff_controller = DiffController::new(
        DiffControllerConfig {
            slip_threshold: 0.05,
            slip_gain: 2.0,
        },
        BemfSlipEstimator::new(),
    );
    // steer_cmd is the only thing STEER actually changes here - THR is still
    // parsed (see the command loop below) but there is no throttle input on
    // real hardware (track voltage controls speed, not this device), so it
    // has nothing to drive.
    let mut steer_cmd = 0.0f32;
    // Overwritten every tick before the (conditional, once-per-telemetry-
    // period) read below, so this initial value is never itself observed -
    // that's expected for loop-carried state, not a real dead store.
    #[allow(unused_assignments)]
    let mut last_slip_estimate = 0.0f32;

    let mut fault_supervisor = FaultSupervisor::new(FaultConfig {
        current_limit: OVERCURRENT_LIMIT,
        comms_timeout_ticks: COMMS_TIMEOUT_TICKS,
    });
    // Same reasoning as last_slip_estimate above: always overwritten by the
    // tick loop (which always runs at least once per outer iteration, since
    // SysTick is what wakes wfi()) before the feed check near the bottom of
    // the loop reads it.
    #[allow(unused_assignments)]
    let mut system_healthy = false;

    // IWDG: a hardware reset if the main loop ever actually hangs (a real
    // bug), on top of - not instead of - the software fault handling above.
    // Fed once per outer loop iteration, but only while fault_supervisor
    // reports everything healthy (DESIGN.md: "IWDG kicked only when all
    // tasks healthy") - a persistent, uncorrected fault eventually resets
    // the MCU too, not just the affected motor's bridge. Note this is
    // independent of the ADC1_2 ISR: a hang *inside* that ISR would starve
    // this loop entirely and trip the watchdog on its own, which is exactly
    // the intended last-resort behavior.
    let mut watchdog = IndependentWatchdog::new(dp.IWDG);
    watchdog.start(IWDG_TIMEOUT_MS.millis());

    command.write(b"aart-os: ISR-driven commutation active\r\n");

    let mut syst = cp.SYST;
    syst.set_clock_source(SystClkSource::Core);
    syst.set_reload(rcc.clocks.sys_clk.raw() / TICK_HZ - 1);
    syst.clear_current();
    syst.enable_interrupt();
    syst.enable_counter();

    let mut scheduler: Scheduler<0> = Scheduler::new([]);

    let mut rx_buf = [0u8; 16];
    let mut line_reader: LineReader<32> = LineReader::new();
    let mut response_buf = [0u8; 48];

    loop {
        while PENDING_TICKS.load(Ordering::Relaxed) > 0 {
            PENDING_TICKS.fetch_sub(1, Ordering::Relaxed);
            scheduler.tick();
            let now = scheduler.current_tick();

            let (erpm_a, current_erpm_a, current_sample_a, stalled_a) =
                with_motor(&MOTOR_A, |m| {
                    (
                        m.commutator.electrical_rpm(),
                        m.commutator.current_erpm(),
                        m.latest_current,
                        m.commutator.phase() == RunPhase::Stalled,
                    )
                });
            let (erpm_b, current_erpm_b, current_sample_b, stalled_b) =
                with_motor(&MOTOR_B, |m| {
                    (
                        m.commutator.electrical_rpm(),
                        m.commutator.current_erpm(),
                        m.latest_current,
                        m.commutator.phase() == RunPhase::Stalled,
                    )
                });

            let diff_out = diff_controller.update(RUNNING_BASE_DUTY, steer_cmd, erpm_a, erpm_b);
            with_motor(&MOTOR_A, |m| m.commutator.set_target_duty(diff_out.target_duty_a));
            with_motor(&MOTOR_B, |m| m.commutator.set_target_duty(diff_out.target_duty_b));
            last_slip_estimate = diff_out.slip_estimate;

            // PWM switching frequency tracks live speed throughout sync and
            // running (current_erpm, not electrical_rpm, so it's live
            // during the open-loop ramp too, not just once BEMF is trusted).
            let freq_khz_a = pwm_schedule.frequency_khz(current_erpm_a);
            if freq_khz_a != last_freq_khz_a {
                with_motor(&MOTOR_A, |m| m.bridge.set_frequency_hz(freq_khz_a * 1_000));
                last_freq_khz_a = freq_khz_a;
            }
            let freq_khz_b = pwm_schedule.frequency_khz(current_erpm_b);
            if freq_khz_b != last_freq_khz_b {
                with_motor(&MOTOR_B, |m| m.bridge.set_frequency_hz(freq_khz_b * 1_000));
                last_freq_khz_b = freq_khz_b;
            }

            let fault_status = fault_supervisor.evaluate(
                now,
                current_sample_a,
                current_sample_b,
                stalled_a,
                stalled_b,
            );
            // Overrides whatever the ISR just wrote - stall already forces
            // disable() there, but overcurrent is an entirely separate
            // condition Commutator has no way to know about, so it's
            // enforced here instead.
            if fault_status.overcurrent_a {
                with_motor(&MOTOR_A, |m| m.bridge.disable());
            }
            if fault_status.overcurrent_b {
                with_motor(&MOTOR_B, |m| m.bridge.disable());
            }
            if fault_status.comms_lost {
                steer_cmd = 0.0; // fail-safe: go straight
            }
            system_healthy = fault_status.all_healthy();

            if now % TELEMETRY_PERIOD_TICKS == 0 {
                let speed_a = erpm_a.unwrap_or(0) as i32;
                let speed_b = erpm_b.unwrap_or(0) as i32;
                let n = format_telemetry(&mut response_buf, speed_a, speed_b, last_slip_estimate);
                command.write(&response_buf[..n]);
            }
        }

        if command.has_data() {
            let n = command.read(&mut rx_buf);
            for &byte in &rx_buf[..n] {
                let Some(line_result) = line_reader.push_byte(byte) else {
                    continue;
                };
                let Ok(line) = line_result else {
                    // Malformed bytes at the line-framing level (buffer
                    // overflow or non-UTF8) - nothing meaningful to echo
                    // back, so just resync silently and keep going.
                    continue;
                };

                let parsed = parse_line(line);
                // Any successfully parsed line counts as proof the link is
                // alive, regardless of which command it was - that's what
                // comms-loss actually means (have we heard anything lately).
                if parsed.is_ok() {
                    fault_supervisor.note_valid_command(scheduler.current_tick());
                }

                match parsed {
                    // Parsed and range-validated, but a protocol-level
                    // no-op on real hardware: these are slot car motors,
                    // track voltage controls speed, and there's no
                    // throttle input for this device to act on. Kept
                    // accepted (not rejected) since it's still useful for
                    // bench testing without a real track/motors.
                    Ok(Command::Throttle(_)) => {}
                    Ok(Command::Steer(v)) => steer_cmd = v,
                    Err(e) => {
                        let n = format_error(e, &mut response_buf);
                        command.write(&response_buf[..n]);
                    }
                }
            }
        }

        if system_healthy {
            watchdog.feed();
        }

        cortex_m::asm::wfi();
    }
}

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {}
}
