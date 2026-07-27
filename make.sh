#!/bin/bash
set -e

PROJECT_NAME="stm32_os"

echo "Creating project: $PROJECT_NAME..."
mkdir -p "$PROJECT_NAME"
cd "$PROJECT_NAME"

# Initialize Rust project
cargo init --name "$PROJECT_NAME"

# Create .cargo directory and config
mkdir -p .cargo
cat << 'EOF' > .cargo/config.toml
[target.thumbv7em-none-eabihf]
runner = "qemu-system-arm -cpu cortex-m4 -machine lm3s6965evb -nographic -semihosting-config enable=on,target=native -kernel"
rustflags = [
  "-C", "link-arg=-Tlink.x",
  "-C", "link-arg=-nostartfiles",
]
EOF

# Create memory.x linker script
cat << 'EOF' > memory.x
MEMORY
{
  FLASH : ORIGIN = 0x08000000, LENGTH = 256K
  RAM : ORIGIN = 0x20000000, LENGTH = 64K
}
EOF

# Overwrite Cargo.toml with embedded dependencies
cat << 'EOF' > Cargo.toml
[package]
name = "stm32_os"
version = "0.1.0"
edition = "2021"

[dependencies]
cortex-m = "0.7.7"
cortex-m-rt = "0.7.3"
stm32f4xx-hal = { version = "0.20.0", features = ["stm32f431", "rt", "critical-section-single-core"] }

defmt = "0.3.5"
defmt-rtt = "0.4.0"

[dev-dependencies]
defmt-test = "0.3.1"
panic-semihosting = "0.6.0"
EOF

# Create a dummy main.rs (required by cargo init, but tests won't use it)
cat << 'EOF' > src/main.rs
#![no_std]
#![no_main]

use panic_semihosting as _;
use cortex_m_rt::entry;

#[entry]
fn main() -> ! {
    loop {}
}
EOF

# Create the tests directory and our test harness
mkdir -p tests
cat << 'EOF' > tests/hal_test.rs
#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_semihosting as _;
use stm32f4xx_hal as hal;

use defmt_test::*;

// A struct to hold hardware state that gets passed between tests
struct TestContext {
    led: hal::gpio::PA5<hal::gpio::Output<hal::gpio::PushPull>>,
}

#[tests]
mod integration_tests {
    use super::*;

    // This runs once before any tests
    #[init]
    fn setup() -> TestContext {
        // 1. Take ownership of the raw device peripherals
        let dp = hal::pac::Peripherals::take().unwrap();

        // 2. Setup clocks (HAL handles the complex register math)
        let rcc = dp.RCC.constrain();
        let _clocks = rcc.cfgr.freeze();

        // 3. Split GPIOA into individual pins
        let gpioa = dp.GPIOA.split();
        
        // 4. Configure Pin 5 as Push-Pull output (Nucleo-F431RB LED)
        let led = gpioa.pa5.into_push_pull_output();

        TestContext { led }
    }

    #[test]
    fn led_starts_low(ctx: &mut TestContext) {
        // The HAL `is_set_high` returns a Result, we unwrap it for the test
        assert!(!ctx.led.is_set_high().unwrap());
    }

    #[test]
    fn can_toggle_led(ctx: &mut TestContext) {
        ctx.led.set_high().unwrap();
        assert!(ctx.led.is_set_high().unwrap());
        
        ctx.led.set_low().unwrap();
        assert!(!ctx.led.is_set_high().unwrap());
    }
}
EOF

# Create the Dockerfile
cat << 'EOF' > Dockerfile
FROM rust:1.75

RUN apt-get update && apt-get install -y \
    qemu-system-arm \
    gcc-arm-none-eabi \
    && rm -rf /var/lib/apt/lists/*

RUN rustup target add thumbv7em-none-eabihf

WORKDIR /app

COPY Cargo.toml Cargo.lock ./
COPY .cargo .cargo
RUN mkdir -p src && echo "fn main() {}" > src/main.rs
RUN cargo build --target thumbv7em-none-eabihf || true
RUN rm -rf src

COPY src src
COPY tests tests
COPY memory.x memory.x

CMD ["cargo", "test", "--target", "thumbv7em-none-eabihf", "--", "--nocapture"]
EOF

echo ""
echo "=================================================="
echo "Project successfully created in './$PROJECT_NAME/'"
echo "=================================================="
echo ""
echo "To run tests LOCALLY (requires qemu-system-arm):"
echo "  cd $PROJECT_NAME"
echo "  cargo test --target thumbv7em-none-eabihf"
echo ""
echo "To run tests in DOCKER:"
echo "  cd $PROJECT_NAME"
echo "  docker build -t stm32-os ."
echo "  docker run --rm stm32-os"
