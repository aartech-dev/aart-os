#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_semihosting as _;
use stm32g4xx_hal as hal;

// RESTORED: This is required for the #[tests] and #[init] macros!
use defmt_test::*;

// 1. Required by defmt to print timestamps
#[no_mangle]
unsafe extern "C" fn _defmt_timestamp() -> u64 { 0 }

// 2. Required by defmt-test if an assertion fails
#[no_mangle]
unsafe extern "C" fn _defmt_panic(_fmt: u32, _args: u32) -> ! {
    loop {}
}

// ==========================================
// QEMU EMULATOR TESTS
// ==========================================
#[cfg(feature = "qemu")]
#[tests]
mod qemu_tests {
    use super::*;

    struct TestContext {
        gpioa: hal::pac::GPIOA,
    }

    #[init]
    fn setup() -> TestContext {
        let dp = hal::pac::Peripherals::take().unwrap();

        unsafe {
            // SPOOF THE READ-ONLY BIT:
            // Because HSIRDY is read-only, the PAC won't let us use w.hsirdy().set_bit().
            // We bypass the PAC and write directly to the RCC CR memory address.
            // Address 0x4002_1000 is RCC. 
            // 0x03 sets Bit 0 (HSION) and Bit 1 (HSIRDY).
            let rcc_cr = 0x4002_1000u32 as *mut u32;
            core::ptr::write_volatile(rcc_cr, 0x03);

            // Switch system clock to HSI
            dp.RCC.cfgr().modify(|_, w| w.sw().hsi());
            
            // Enable GPIOA clock (AHB2 bus on G4 series)
            dp.RCC.ahb2enr().modify(|_, w| w.gpioaen().set_bit());
            
            // Manually configure PA5 as Output Push-Pull
            dp.GPIOA.moder().modify(|_, w| w.moder5().output());
            dp.GPIOA.otyper().modify(|_, w| w.ot5().clear_bit());
        }

        TestContext { gpioa: dp.GPIOA }
    }

    #[test]
    fn gpioa_clock_enabled() {
        // Only .steal() is unsafe! 
        let rcc = unsafe { hal::pac::RCC::steal() };
        // .read() is safe!
        assert!(rcc.ahb2enr().read().gpioaen().bit_is_set());
    }

    #[test]
    fn pin_5_is_output() {
        // Only .steal() is unsafe!
        let gpioa = unsafe { hal::pac::GPIOA::steal() };
        // .read() is safe!
        assert_eq!(gpioa.moder().read().moder5().bits(), 0b01);
    }

    #[test]
    fn can_toggle_led_raw(ctx: &mut TestContext) {
        // Because we already own ctx.gpioa from the setup() function,
        // writing and reading it is 100% safe! No unsafe block needed.
        ctx.gpioa.bsrr().write(|w| w.bs5().set_bit());
        assert!(ctx.gpioa.odr().read().odr5().bit_is_set());

        ctx.gpioa.bsrr().write(|w| w.br5().set_bit());
        assert!(!ctx.gpioa.odr().read().odr5().bit_is_set());
    }
}

// ==========================================
// REAL HARDWARE TESTS (Nucleo Board)
// ==========================================
#[cfg(not(feature = "qemu"))]
#[tests]
mod real_tests {
    use super::*;
    use hal::rcc::{Config, RccExt};
    use hal::pwr::PwrExt;
    use hal::gpio::GpioExt;

    struct TestContext {
        led: hal::gpio::PA5<hal::gpio::Output<hal::gpio::PushPull>>,
    }

    #[init]
    fn setup() -> TestContext {
        let dp = hal::pac::Peripherals::take().unwrap();
        let pwr_config = dp.PWR.constrain().freeze();
        let mut rcc = dp.RCC.freeze(Config::default(), pwr_config);
        let gpioa = dp.GPIOA.split(&mut rcc);
        let led = gpioa.pa5.into_push_pull_output();
        TestContext { led }
    }

    #[test]
    fn led_starts_low(ctx: &mut TestContext) {
        assert!(!ctx.led.is_set_high());
    }

    #[test]
    fn can_toggle_led(ctx: &mut TestContext) {
        ctx.led.set_high();
        assert!(ctx.led.is_set_high());
        ctx.led.set_low();
        assert!(!ctx.led.is_set_high());
    }
}
