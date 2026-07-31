//! Command/telemetry byte channel: USART1 on PB6 (TX) / PB7 (RX).
//!
//! Deliberately *not* USART2/PA2-PA3 (the Nucleo ST-Link VCOM pins) — see
//! DESIGN.md section 6.2 for why those are needed for motor A's BEMF sensing
//! instead. Byte-level only; the line protocol (THR/STEER/SPD, DESIGN.md
//! section 7.2) is aart-core's job in a later milestone.

use embedded_io::{Read, ReadReady, Write};

use stm32g4xx_hal::gpio::{AF7, PB6, PB7};
use stm32g4xx_hal::rcc::Rcc;
use stm32g4xx_hal::serial::{FullConfig, Serial, SerialExt};
use stm32g4xx_hal::stm32::USART1;
use stm32g4xx_hal::time::U32Ext;

pub const BAUD_RATE: u32 = 115_200;

pub struct CommandChannel {
    serial: Serial<USART1, PB6<AF7>, PB7<AF7>>,
}

impl CommandChannel {
    /// Blocks only until at least one byte can be sent, then writes as many
    /// of `bytes` as fit without further waiting. Returns the count written.
    pub fn write(&mut self, bytes: &[u8]) -> usize {
        self.serial.write(bytes).unwrap_or(0)
    }

    /// True if at least one byte is waiting to be read.
    pub fn has_data(&mut self) -> bool {
        self.serial.read_ready().unwrap_or(false)
    }

    /// Only call when `has_data()` is true — otherwise this blocks until a
    /// byte arrives.
    pub fn read(&mut self, buf: &mut [u8]) -> usize {
        self.serial.read(buf).unwrap_or(0)
    }
}

pub fn command_channel(usart1: USART1, tx: PB6<AF7>, rx: PB7<AF7>, rcc: &mut Rcc) -> CommandChannel {
    let serial = usart1
        .usart(tx, rx, FullConfig::default().baudrate(BAUD_RATE.bps()), rcc)
        .expect("USART1 baud rate unreachable at the current PCLK");

    CommandChannel { serial }
}
