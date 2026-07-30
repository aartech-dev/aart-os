#![no_std]
#![no_main]

use core::cell::RefCell;
use core::panic::PanicInfo;
use core::sync::atomic::{AtomicU32, Ordering};

use cortex_m::interrupt::{free, Mutex};
use cortex_m::peripheral::syst::SystClkSource;
use cortex_m_rt::{entry, exception};

use stm32g4xx_hal::gpio::{GpioExt, Output, PushPull, PA5};
use stm32g4xx_hal::pwr::PwrExt;
use stm32g4xx_hal::rcc::{Config, RccExt};

use aart_core::scheduler::{Scheduler, Task};

type Led = PA5<Output<PushPull>>;

// M0 skeleton only: the Nucleo LED (PA5) becomes motor B's BEMF_B input from M3
// onward (see DESIGN.md section 6.1) and this task goes away then. For now it's
// the cheapest possible proof that the scheduler is actually driving hardware.
static LED: Mutex<RefCell<Option<Led>>> = Mutex::new(RefCell::new(None));

// SysTick only increments this; the main loop (not the ISR) drains it into
// scheduler.tick() calls, so task code never runs at interrupt priority.
static PENDING_TICKS: AtomicU32 = AtomicU32::new(0);

const TICK_HZ: u32 = 1_000;
const BLINK_PERIOD_TICKS: u32 = 500;

fn blink(_tick: u64) {
    free(|cs| {
        if let Some(led) = LED.borrow(cs).borrow_mut().as_mut() {
            if led.is_set_high() {
                led.set_low();
            } else {
                led.set_high();
            }
        }
    });
}

#[exception]
fn SysTick() {
    PENDING_TICKS.fetch_add(1, Ordering::Relaxed);
}

#[entry]
fn main() -> ! {
    let dp = stm32g4xx_hal::pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    let pwr_cfg = dp.PWR.constrain().freeze();
    let mut rcc = dp.RCC.freeze(Config::default(), pwr_cfg);
    let gpioa = dp.GPIOA.split(&mut rcc);
    let led = gpioa.pa5.into_push_pull_output();
    free(|cs| LED.borrow(cs).replace(Some(led)));

    let mut syst = cp.SYST;
    syst.set_clock_source(SystClkSource::Core);
    syst.set_reload(rcc.clocks.sys_clk.raw() / TICK_HZ - 1);
    syst.clear_current();
    syst.enable_interrupt();
    syst.enable_counter();

    let mut scheduler = Scheduler::new([Task {
        period_ticks: BLINK_PERIOD_TICKS,
        run: blink,
    }]);

    loop {
        while PENDING_TICKS.load(Ordering::Relaxed) > 0 {
            PENDING_TICKS.fetch_sub(1, Ordering::Relaxed);
            scheduler.tick();
        }
        cortex_m::asm::wfi();
    }
}

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {}
}
