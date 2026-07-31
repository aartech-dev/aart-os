//! 3-phase complementary PWM bridge driver (motor A = TIM1, motor B = TIM8).
//!
//! One `Bridge` per motor: three high-side channels, each carrying its
//! own complementary (dead-time-inserted) low-side output automatically —
//! there is one duty register per phase, not one per FET.

use aart_core::commutator::{PhaseState, StepPattern};
use stm32g4xx_hal::gpio::{
    AF10, AF2, AF3, AF4, AF6, PA10, PA15, PA7, PA8, PA9, PB0, PB3, PB4, PB5, PB8, PB9, PF0,
};
use stm32g4xx_hal::hal_02::PwmPin;
use stm32g4xx_hal::pwm::{
    ActiveHigh, ComplementaryEnabled, FaultDisabled, Pwm, PwmAdvExt, PwmControl, C1, C2, C3,
};
use stm32g4xx_hal::rcc::{BusTimerClock, Rcc, RccBus};
use stm32g4xx_hal::stm32::tim1;
use stm32g4xx_hal::stm32::{TIM1, TIM8};
use stm32g4xx_hal::time::{ExtU32, RateExtU32};

// STM32 advanced-control timer CR2.MMS field: 0b010 selects "Update event" as
// the trigger output (TRGO). Not exposed by PwmBuilder (see the crate's own
// "future features" comment in pwm.rs), so this pokes the register directly —
// TRGO fires once per PWM period, which is what the ADC driver (sense.rs)
// syncs its BEMF/current sampling to.
const MMS_UPDATE: u8 = 0b010;

pub const DEADTIME_NANOS: u32 = 500;

// Switching frequency at the low/high ends of the eRPM range (see
// aart_core::commutator::PwmFrequencySchedule, driven from main.rs).
// Values as specified for these motors; not yet tuned against the actual
// clock resolution available - see set_frequency_hz's note on that.
pub const PWM_FREQUENCY_MIN_KHZ: u32 = 48;
pub const PWM_FREQUENCY_MAX_KHZ: u32 = 96;

/// Three duty-cycle channels driving one motor's 3-phase bridge, plus what's
/// needed to retune the PWM switching frequency at runtime (see
/// `set_frequency_hz`) - `pwm_advanced().frequency(...)` only sets it once,
/// at construction, and this needs to track eRPM afterward (DESIGN.md's
/// 48kHz-at-low-eRPM/96kHz-at-high-eRPM schedule).
///
/// `tim` is `*const tim1::RegisterBlock` rather than a generic `TIM` type
/// parameter: TIM1 and TIM8 are both `Periph<tim1::RegisterBlock, _>` (same
/// register layout, different base address) in this PAC, so one field/type
/// covers either motor's timer without needing Bridge to be generic over
/// which one it is.
pub struct Bridge<A, B, C> {
    high_a: A,
    high_b: B,
    high_c: C,
    tim: *const tim1::RegisterBlock,
    base_clock_hz: u32,
}

// SAFETY: `tim` points at a fixed peripheral base address, not real shared
// mutable aliasing that unsynchronized access could race on - callers are
// already required to serialize access themselves (the ISR/critical-section
// pattern main.rs uses to share a Bridge between the ADC1_2 interrupt and
// the main loop), same as any other embedded HAL type wrapping a raw
// peripheral pointer. Needed so `Bridge` can live in a `Mutex`-protected
// `static` at all - raw pointers aren't `Send` by default.
unsafe impl<A: Send, B: Send, C: Send> Send for Bridge<A, B, C> {}

/// What the ADC-ISR commutation step (main.rs) needs from either motor's
/// bridge - one trait so that logic can be written generically once,
/// monomorphized over `MotorABridge`/`MotorBBridge`, rather than duplicated
/// per motor (same reasoning as `sense::SenseIsr`). Delegates to the
/// concrete inherent methods below (which any external caller with a
/// concretely-typed `Bridge<A,B,C>` in hand should keep calling directly -
/// this trait exists for the generic ISR path, not to replace those).
pub trait BridgeControl {
    fn max_duty(&self) -> u16;
    fn apply_step(&mut self, pattern: StepPattern, duty: u16);
    fn disable(&mut self);
}

impl<A, B, C> BridgeControl for Bridge<A, B, C>
where
    A: PwmPin<Duty = u16>,
    B: PwmPin<Duty = u16>,
    C: PwmPin<Duty = u16>,
{
    fn max_duty(&self) -> u16 {
        Bridge::max_duty(self)
    }
    fn apply_step(&mut self, pattern: StepPattern, duty: u16) {
        Bridge::apply_step(self, pattern, duty)
    }
    fn disable(&mut self) {
        Bridge::disable(self)
    }
}

impl<A, B, C> Bridge<A, B, C>
where
    A: PwmPin<Duty = u16>,
    B: PwmPin<Duty = u16>,
    C: PwmPin<Duty = u16>,
{
    pub fn max_duty(&self) -> u16 {
        self.high_a.get_max_duty()
    }

    /// Forces all three phases to true Hi-Z at once (unlike apply_step's
    /// per-phase Float handling) - used for stall/overcurrent safe-stop
    /// (main.rs), where even the "Low" phases must stop being actively
    /// driven, not just chopped to 0% duty.
    pub fn disable(&mut self) {
        self.high_a.disable();
        self.high_b.disable();
        self.high_c.disable();
    }

    /// Drive one six-step commutation step (see `aart_core::commutator`):
    /// the phase marked High is PWM-chopped at `duty`, Low is held fully low
    /// (steady conduction on the low-side FET, not chopped), and Float is
    /// disabled entirely on both high and low sides for a true high-Z BEMF
    /// sensing window on that phase alone — the other two phases are left
    /// running, unlike `disable()` which affects all three.
    pub fn apply_step(&mut self, pattern: StepPattern, duty: u16) {
        Self::apply_phase(&mut self.high_a, pattern.a, duty);
        Self::apply_phase(&mut self.high_b, pattern.b, duty);
        Self::apply_phase(&mut self.high_c, pattern.c, duty);
    }

    fn apply_phase(channel: &mut impl PwmPin<Duty = u16>, state: PhaseState, duty: u16) {
        match state {
            PhaseState::High => {
                channel.enable();
                channel.set_duty(duty);
            }
            PhaseState::Low => {
                channel.enable();
                channel.set_duty(0);
            }
            PhaseState::Float => channel.disable(),
        }
    }

    /// Retunes the PWM switching frequency at runtime by recomputing and
    /// writing PSC/ARR directly - `pwm_advanced()`'s `.frequency(...)` only
    /// configures this once, before `.finalize()`, with no way back in
    /// afterward. Mirrors that builder's own center-aligned rounding formula
    /// (`base_freq / (freq*2)`, rounded to nearest, i.e. one counter period
    /// is a full up+down count) so this doesn't silently drift from
    /// whatever the initial construction-time frequency would have computed.
    ///
    /// Changing ARR while PWM is live can shorten or lengthen the *current*
    /// period by one count (briefly), not glitch-free - acceptable here
    /// since this is only meant to be called between commutation steps, not
    /// mid-cycle every PWM period.
    ///
    /// Duty is preserved as a *fraction* of max_duty (CCRx scales with the
    /// new ARR) so callers don't need to re-set duty right after calling
    /// this - `apply_step`'s next call still recomputes it anyway.
    ///
    /// Resolution caveat: `Config::default()` in main.rs runs TIM1/TIM8 off
    /// plain HSI (16MHz), no PLL. At 16MHz, 48kHz center-aligned gives
    /// max_duty ~166 (166 counts of duty resolution, ~0.6%); 96kHz gives
    /// ~83 (~1.2%). Workable, but coarser than the 20kHz this bridge
    /// originally ran at (max_duty ~400). Boosting SYSCLK via the PLL (G4
    /// supports up to 170MHz) would meaningfully improve this and hasn't
    /// been done - noting it here rather than quietly shipping the
    /// resolution hit.
    pub fn set_frequency_hz(&mut self, hz: u32) {
        let hz = hz.max(1);
        // Center-aligned: one PWM period is a full up-then-down count, so
        // the effective divisor is 2x what left-aligned would need for the
        // same switching frequency - same relationship
        // stm32g4xx-hal's own (private) calculate_frequency_32bit uses.
        let divisor = (hz as u64) * 2;
        let ideal_period = (self.base_clock_hz as u64 + divisor / 2) / divisor;
        let prescale = ((ideal_period.max(1) - 1) / (1 << 16)) as u32;
        let period = ((ideal_period + (prescale as u64 >> 1)) / (prescale as u64 + 1)).max(1) - 1;

        let old_max_duty = self.max_duty().max(1) as u32;
        let new_max_duty = period.min(0xFFFF) as u32;

        // SAFETY: PSC/ARR are always valid to write on an initialized,
        // enabled advanced-control timer; this only ever runs after the
        // Bridge (and thus this exact TIM instance) has been constructed.
        unsafe {
            let tim = &*self.tim;
            tim.psc().write(|w| w.psc().bits(prescale as u16));
            tim.arr().write(|w| w.arr().bits(new_max_duty));
        }

        let rescale = |duty: u16| -> u16 {
            ((duty as u32 * new_max_duty) / old_max_duty).min(new_max_duty) as u16
        };
        let duty_a = rescale(self.high_a.get_duty());
        let duty_b = rescale(self.high_b.get_duty());
        let duty_c = rescale(self.high_c.get_duty());
        self.high_a.set_duty(duty_a);
        self.high_b.set_duty(duty_b);
        self.high_c.set_duty(duty_c);
    }
}

type MotorAPins = (PA8<AF6>, PA9<AF6>, PA10<AF6>);

// Concrete (not `impl Trait`) channel types: needed so `MotorABridge` can
// be named in a `static` for the ADC-ISR ownership handoff below - opaque
// return-position `impl Trait` types can't appear in a static's type.
type MotorAChannel1 = Pwm<TIM1, C1, ComplementaryEnabled, ActiveHigh, ActiveHigh>;
type MotorAChannel2 = Pwm<TIM1, C2, ComplementaryEnabled, ActiveHigh, ActiveHigh>;
type MotorAChannel3 = Pwm<TIM1, C3, ComplementaryEnabled, ActiveHigh, ActiveHigh>;
pub type MotorABridge = Bridge<MotorAChannel1, MotorAChannel2, MotorAChannel3>;

/// Motor A bridge on TIM1: High_A/B/C = PA8/PA9/PA10, Low_A/B/C = PA7/PB0/PF0.
/// Pin assignment matches DESIGN.md section 6.1 (ESCape32's own STM32G431
/// reference pinout, used as-is for motor A).
pub fn motor_a_bridge(
    tim1: TIM1,
    pins: MotorAPins,
    low_a: PA7<AF6>,
    low_b: PB0<AF6>,
    low_c: PF0<AF6>,
    rcc: &mut Rcc,
) -> (PwmControl<TIM1, FaultDisabled>, MotorABridge) {
    let base_clock_hz = <TIM1 as RccBus>::Bus::timer_clock(&rcc.clocks).raw();

    let (control, (c1, c2, c3)) = tim1
        .pwm_advanced(pins, rcc)
        .frequency(PWM_FREQUENCY_MIN_KHZ.kHz())
        .center_aligned()
        .with_deadtime(DEADTIME_NANOS.nanos())
        .finalize();

    let high_a = c1.into_complementary(low_a);
    let high_b = c2.into_complementary(low_b);
    let high_c = c3.into_complementary(low_c);

    unsafe {
        (*TIM1::ptr())
            .cr2()
            .modify(|_, w| w.mms().bits(MMS_UPDATE));
    }

    (
        control,
        Bridge {
            high_a,
            high_b,
            high_c,
            tim: TIM1::ptr(),
            base_clock_hz,
        },
    )
}

type MotorBPins = (PA15<AF2>, PB8<AF10>, PB9<AF10>);

type MotorBChannel1 = Pwm<TIM8, C1, ComplementaryEnabled, ActiveHigh, ActiveHigh>;
type MotorBChannel2 = Pwm<TIM8, C2, ComplementaryEnabled, ActiveHigh, ActiveHigh>;
type MotorBChannel3 = Pwm<TIM8, C3, ComplementaryEnabled, ActiveHigh, ActiveHigh>;
pub type MotorBBridge = Bridge<MotorBChannel1, MotorBChannel2, MotorBChannel3>;

/// Motor B bridge on TIM8: High_A/B/C = PA15/PB8/PB9, Low_A/B/C = PB3/PB4/PB5.
/// Derived from stm32g4xx-hal's own TIM8 pin table, filtered to alternate
/// functions that exist on this Nucleo's LQFP64 package (DESIGN.md 6.1).
pub fn motor_b_bridge(
    tim8: TIM8,
    pins: MotorBPins,
    low_a: PB3<AF4>,
    low_b: PB4<AF4>,
    low_c: PB5<AF3>,
    rcc: &mut Rcc,
) -> (PwmControl<TIM8, FaultDisabled>, MotorBBridge) {
    let base_clock_hz = <TIM8 as RccBus>::Bus::timer_clock(&rcc.clocks).raw();

    let (control, (c1, c2, c3)) = tim8
        .pwm_advanced(pins, rcc)
        .frequency(PWM_FREQUENCY_MIN_KHZ.kHz())
        .center_aligned()
        .with_deadtime(DEADTIME_NANOS.nanos())
        .finalize();

    let high_a = c1.into_complementary(low_a);
    let high_b = c2.into_complementary(low_b);
    let high_c = c3.into_complementary(low_c);

    unsafe {
        (*TIM8::ptr())
            .cr2()
            .modify(|_, w| w.mms().bits(MMS_UPDATE));
    }

    (
        control,
        Bridge {
            high_a,
            high_b,
            high_c,
            tim: TIM8::ptr(),
            base_clock_hz,
        },
    )
}
