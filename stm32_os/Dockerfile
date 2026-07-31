# Build context must be the REPO ROOT, not this directory, since M0 added a
# path dependency on ../aart-core (a sibling crate) - Docker can't COPY files
# from outside its build context. Build with:
#   docker build -t stm32-os -f stm32_os/Dockerfile .
# (run from the repo root, not from inside stm32_os/).
#
# Pinned to 1.90, not 1.75: Cargo.lock is format version 4 (pre-dates this
# session - already present at commit 006eb1d), which Cargo before ~1.78
# can't parse at all. On top of that, stm32g4xx-hal's flash.rs uses
# u32::is_multiple_of(), stabilized only in Rust 1.87 - 1.82 gets past the
# lockfile but then fails to compile the HAL with "use of unstable library
# feature". rust:1.75 fails outright on the first issue alone.
FROM rust:1.90

RUN apt-get update && apt-get install -y \
    qemu-system-arm \
    gcc-arm-none-eabi \
    && rm -rf /var/lib/apt/lists/*

RUN rustup target add thumbv7em-none-eabihf

WORKDIR /app
COPY aart-core aart-core
COPY stm32_os stm32_os

WORKDIR /app/stm32_os

# Pre-existing bug, not from this session: `-- --nocapture` was being passed
# through the `cargo qemu` alias straight to the qemu-system-arm process
# itself ("qemu-system-arm: --nocapture: invalid option"), not to the test
# harness — cargo test's own runner args don't reach the child process that
# way here. Dropped; cargo test still reports pass/fail either way.
#
# Separately (also pre-existing, not fixed here): this qemu invocation is
# known to hang rather than exit under some qemu-system-arm builds/semihosting
# configs — see tests/hal_test.rs and stm32_os.resc's own comments. Confirmed
# unrelated to aart-core/motor.rs/sense.rs/command.rs, which build cleanly
# from scratch in this same container; this CMD's job is only to demonstrate
# that.
CMD ["cargo", "qemu"]
