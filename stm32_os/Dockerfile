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

# FIX: the old CMD was
#   cargo test --target thumbv7em-none-eabihf -- --nocapture
# With no `--features qemu`, cargo compiled the REAL-HARDWARE test module
# (`#[cfg(not(feature = "qemu"))]`), whose setup() calls the genuine HAL
# `RccExt::freeze()`. QEMU's netduinoplus2 (an STM32F405 model) never raises the
# RCC clock-ready bits, so freeze() spun forever -> the container "ran tests but
# never responded."
#
# `cargo qemu` (alias in .cargo/config.toml) forces `--features qemu` AND
# overrides the default probe-rs runner back to QEMU, so the container always
# runs the register-spoofed path and can never reach the hanging one.
CMD ["cargo", "qemu", "--", "--nocapture"]
