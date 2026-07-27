#![no_std]
#![no_main]

use core::panic::PanicInfo;
use cortex_m_rt::entry;

// THIS is the magic line that forces the STM32G431 interrupt vectors 
// to be included in the binary so the linker stops complaining.
extern crate stm32g4xx_hal; 

#[entry]
fn main() -> ! {
    loop {}
}

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {}
}
