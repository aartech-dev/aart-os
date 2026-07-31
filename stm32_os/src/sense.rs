//! Phase/neutral/current/voltage sensing (motor A = ADC1, motor B = ADC2).
//!
//! Each motor gets its own dedicated ADC for phase/current sampling (option 2
//! from DESIGN.md section 6.1): a physical virtual-neutral resistor node per
//! motor instead of ESCape32's single-motor dual-ADC averaged reference, so
//! ADC1 and ADC2 can sample simultaneously with no interleaving between the
//! two motors.
//!
//! On top of that, each motor's neutral node also gets its OWN dedicated ADC
//! (ADC3 for motor A, ADC4 for motor B - both on the ADC345_COMMON clock
//! domain, only present once we retargeted from the G431 to the G474, which
//! is the whole reason for that retarget). Each runs free-running in
//! continuous mode with no external trigger, so there's always a fresh
//! neutral sample sitting in the data register - no need to interleave
//! neutral into the same rotation as phase/current on ADC1/ADC2, which used
//! to cost a full rotation slot per motor (see main.rs's old NEUTRAL_SLOT).
//!
//! Uses `DynamicAdc` (plain `&mut self` methods) rather than the crate's
//! typed `Adc<ADC, Configured/Active>` state machine: the ISR-driven fast
//! path (main.rs) repeatedly reconfigures which single channel is armed and
//! re-triggers it every interrupt, which doesn't fit a state machine that
//! wants to *consume* `self` on `start_conversion()` and hand back a
//! different type - `DynamicAdc` exposes the same underlying operations
//! without that ownership dance.

use embedded_hal::delay::DelayNs;

use aart_core::commutator::Phase;
use stm32g4xx_hal::adc::config::{ClockMode, Continuous, Eoc, ExternalTrigger12, SampleTime, Sequence, TriggerMode};
use stm32g4xx_hal::adc::{AdcClaim, AdcCommon, AdcCommonExt, DynamicAdc};
use stm32g4xx_hal::gpio::{Analog, PA0, PA1, PA3, PA4, PA5, PA6, PB1, PB11, PB12, PB14, PF1};
use stm32g4xx_hal::rcc::Rcc;
use stm32g4xx_hal::stm32::{ADC1, ADC12_COMMON, ADC2, ADC3, ADC345_COMMON, ADC4};

/// Busy-wait delay in core clock cycles. Only needed transiently during ADC
/// power-up (a few microseconds); doesn't consume a timer peripheral, since
/// SysTick is already dedicated to the scheduler tick (see main.rs).
pub struct CycleDelay {
    core_hz: u32,
}

impl CycleDelay {
    pub fn new(core_hz: u32) -> Self {
        Self { core_hz }
    }
}

impl DelayNs for CycleDelay {
    fn delay_ns(&mut self, ns: u32) {
        let cycles = ((self.core_hz as u64 * ns as u64) / 1_000_000_000).max(1) as u32;
        cortex_m::asm::delay(cycles);
    }
}

const SAMPLE_TIME: SampleTime = SampleTime::Cycles_247_5;

/// Claim the ADC1/ADC2 common block (clock config shared by both motors'
/// phase/current sensing).
///
/// AHB/HCLK now runs at 170MHz (main.rs's PLL config) - the ADC's own max
/// input clock is ~60MHz (RM0440), so HCLK/2 (85MHz) would be out of spec.
/// HCLK/4 = 42.5MHz stays under it; this was HclkDiv2 back when HCLK was
/// the original 16MHz HSI, where /2 (8MHz) was nowhere near the limit.
pub fn claim_common(adc12_common: ADC12_COMMON, rcc: &mut Rcc) -> AdcCommon<ADC12_COMMON> {
    adc12_common.claim(ClockMode::AdcHclkDiv4, rcc)
}

/// Claim the ADC3/ADC4/ADC5 common block (clock config shared by both
/// motors' dedicated free-running neutral sensing). Same divider reasoning
/// as `claim_common` above - same 170MHz AHB, same ADC input clock limit.
pub fn claim_common_345(adc345_common: ADC345_COMMON, rcc: &mut Rcc) -> AdcCommon<ADC345_COMMON> {
    adc345_common.claim(ClockMode::AdcHclkDiv4, rcc)
}

/// What the ADC1_2 ISR (main.rs) needs from either motor's sense front-end -
/// one trait so the ISR step function can be written generically once
/// rather than duplicated per motor, the same reasoning as `PhaseSense`
/// before it (removed - this supersedes it for the ISR-driven path).
pub trait SenseIsr {
    /// Reconfigures the single armed channel to whichever phase is
    /// currently floating. Takes effect on the *next* hardware trigger, not
    /// immediately - call `rearm` after.
    fn configure_phase(&mut self, phase: Phase);
    fn configure_current(&mut self);
    /// Raw value from whatever channel was last converted - doesn't
    /// trigger anything itself, just reads the data register.
    fn current_sample(&self) -> u16;
    /// Latest reading from the dedicated free-running neutral ADC (ADC3 for
    /// motor A, ADC4 for motor B) - always fresh, no rotation/rearm needed
    /// since that ADC never stops converting.
    fn neutral_sample(&self) -> u16;
    /// True if this ADC (not necessarily the other motor's) is the one
    /// that raised the shared ADC1_2 interrupt.
    fn eoc_pending(&self) -> bool;
    fn clear_eoc(&mut self);
    /// Re-arms for the next hardware trigger (TRGO auto-clears ADSTART
    /// once a single-channel conversion completes - each subsequent
    /// trigger needs this called again first).
    fn rearm(&mut self);
}

pub struct MotorASense {
    adc: DynamicAdc<ADC1>,
    phase_a: PA0<Analog>,
    phase_b: PA1<Analog>,
    phase_c: PA3<Analog>,
    current: PB11<Analog>,
    // Not sampled until brown-out detection needs it.
    #[allow(dead_code)]
    bus_voltage: PB12<Analog>,
    adc_neutral: DynamicAdc<ADC3>,
    #[allow(dead_code)]
    neutral: PB1<Analog>,
}

impl MotorASense {
    #[allow(dead_code)]
    pub fn configure_bus_voltage(&mut self) {
        self.adc.configure_channel(&self.bus_voltage, Sequence::One, SAMPLE_TIME);
    }
}

impl SenseIsr for MotorASense {
    fn configure_phase(&mut self, phase: Phase) {
        match phase {
            Phase::A => self.adc.configure_channel(&self.phase_a, Sequence::One, SAMPLE_TIME),
            Phase::B => self.adc.configure_channel(&self.phase_b, Sequence::One, SAMPLE_TIME),
            Phase::C => self.adc.configure_channel(&self.phase_c, Sequence::One, SAMPLE_TIME),
        }
    }

    fn configure_current(&mut self) {
        self.adc.configure_channel(&self.current, Sequence::One, SAMPLE_TIME);
    }

    fn current_sample(&self) -> u16 {
        self.adc.current_sample()
    }

    fn neutral_sample(&self) -> u16 {
        self.adc_neutral.current_sample()
    }

    fn eoc_pending(&self) -> bool {
        unsafe { (*ADC1::ptr()).isr().read().eoc().bit_is_set() }
    }

    fn clear_eoc(&mut self) {
        self.adc.clear_end_of_conversion_flag();
    }

    fn rearm(&mut self) {
        self.adc.start_conversion();
    }
}

#[allow(clippy::too_many_arguments)]
pub fn motor_a_sense(
    adc1: ADC1,
    common: &AdcCommon<ADC12_COMMON>,
    phase_a: PA0<Analog>,
    phase_b: PA1<Analog>,
    phase_c: PA3<Analog>,
    current: PB11<Analog>,
    bus_voltage: PB12<Analog>,
    adc3: ADC3,
    common345: &AdcCommon<ADC345_COMMON>,
    neutral: PB1<Analog>,
    delay: &mut impl DelayNs,
) -> MotorASense {
    // claim() already powers up once internally (returns Disabled, not
    // PoweredDown) - into_dynamic_adc() only exists on PoweredDown, so this
    // powers back down (type-state only) just to reach it, then powers back
    // up again on the DynamicAdc before use. A few extra microseconds at
    // boot, not a real hardware concern.
    let disabled = common.claim(adc1, delay);
    let mut adc = disabled.power_down().into_dynamic_adc();
    adc.power_up(delay);
    // Synced to motor A's own PWM period (TIM1 TRGO, see motor.rs) rather
    // than free-running, so BEMF/current samples land in the PWM off-time.
    adc.set_external_trigger((TriggerMode::RisingEdge, ExternalTrigger12::Tim_1_trgo));
    adc.set_default_sample_time(SAMPLE_TIME);
    adc.set_end_of_conversion_interrupt(Eoc::Conversion);
    adc.enable(); // calibrates + applies config internally
    adc.configure_channel(&phase_a, Sequence::One, SAMPLE_TIME);
    adc.start_conversion(); // armed for the first hardware trigger

    let disabled_neutral = common345.claim(adc3, delay);
    let mut adc_neutral = disabled_neutral.power_down().into_dynamic_adc();
    adc_neutral.power_up(delay);
    // No external trigger (stays TriggerMode::Disabled, the default) and
    // continuous mode: once started with start_conversion() below, it
    // free-runs on its own forever, so current_sample() always has a fresh
    // neutral reading with no rearm/interrupt bookkeeping needed.
    adc_neutral.set_continuous(Continuous::Continuous);
    adc_neutral.set_default_sample_time(SAMPLE_TIME);
    adc_neutral.enable();
    adc_neutral.configure_channel(&neutral, Sequence::One, SAMPLE_TIME);
    adc_neutral.start_conversion();

    MotorASense {
        adc,
        phase_a,
        phase_b,
        phase_c,
        current,
        bus_voltage,
        adc_neutral,
        neutral,
    }
}

pub struct MotorBSense {
    adc: DynamicAdc<ADC2>,
    phase_a: PA4<Analog>,
    phase_b: PA5<Analog>,
    phase_c: PA6<Analog>,
    current: PF1<Analog>,
    adc_neutral: DynamicAdc<ADC4>,
    #[allow(dead_code)]
    neutral: PB14<Analog>,
}

impl SenseIsr for MotorBSense {
    fn configure_phase(&mut self, phase: Phase) {
        match phase {
            Phase::A => self.adc.configure_channel(&self.phase_a, Sequence::One, SAMPLE_TIME),
            Phase::B => self.adc.configure_channel(&self.phase_b, Sequence::One, SAMPLE_TIME),
            Phase::C => self.adc.configure_channel(&self.phase_c, Sequence::One, SAMPLE_TIME),
        }
    }

    fn configure_current(&mut self) {
        self.adc.configure_channel(&self.current, Sequence::One, SAMPLE_TIME);
    }

    fn current_sample(&self) -> u16 {
        self.adc.current_sample()
    }

    fn neutral_sample(&self) -> u16 {
        self.adc_neutral.current_sample()
    }

    fn eoc_pending(&self) -> bool {
        unsafe { (*ADC2::ptr()).isr().read().eoc().bit_is_set() }
    }

    fn clear_eoc(&mut self) {
        self.adc.clear_end_of_conversion_flag();
    }

    fn rearm(&mut self) {
        self.adc.start_conversion();
    }
}

#[allow(clippy::too_many_arguments)]
pub fn motor_b_sense(
    adc2: ADC2,
    common: &AdcCommon<ADC12_COMMON>,
    phase_a: PA4<Analog>,
    phase_b: PA5<Analog>,
    phase_c: PA6<Analog>,
    current: PF1<Analog>,
    adc4: ADC4,
    common345: &AdcCommon<ADC345_COMMON>,
    neutral: PB14<Analog>,
    delay: &mut impl DelayNs,
) -> MotorBSense {
    let disabled = common.claim(adc2, delay);
    let mut adc = disabled.power_down().into_dynamic_adc();
    adc.power_up(delay);
    adc.set_external_trigger((TriggerMode::RisingEdge, ExternalTrigger12::Tim_8_trgo));
    adc.set_default_sample_time(SAMPLE_TIME);
    adc.set_end_of_conversion_interrupt(Eoc::Conversion);
    adc.enable();
    adc.configure_channel(&phase_a, Sequence::One, SAMPLE_TIME);
    adc.start_conversion();

    let disabled_neutral = common345.claim(adc4, delay);
    let mut adc_neutral = disabled_neutral.power_down().into_dynamic_adc();
    adc_neutral.power_up(delay);
    adc_neutral.set_continuous(Continuous::Continuous);
    adc_neutral.set_default_sample_time(SAMPLE_TIME);
    adc_neutral.enable();
    adc_neutral.configure_channel(&neutral, Sequence::One, SAMPLE_TIME);
    adc_neutral.start_conversion();

    MotorBSense {
        adc,
        phase_a,
        phase_b,
        phase_c,
        current,
        adc_neutral,
        neutral,
    }
}
