# aart-os

A small single-user embedded OS in Rust for commutating two sensorless
BLDC motors (small 1106-class slot car motors) with an electronic
front/rear axle balance driven by a BEMF-based slip estimate — no
mechanical differential, no throttle input (track voltage controls
speed; this device only balances the two motors against each other).

See [`DESIGN.md`](DESIGN.md) for the full architecture, pin allocation,
and milestone history.

## Hardware

Target board: **Nucleo-G474RE** (STM32G474RET6, Cortex-M4F, 512 KB flash
/ 128 KB SRAM). See DESIGN.md §6 for the full pin table and §6.2 for the
Nucleo solder-bridge changes required (PA0/PA5/PA2-PA3 conflict with the
board's default USER button, LED, and ST-Link VCOM UART).

## Repo layout

```
aart-core/    hardware-free logic: scheduler, commutation state machine,
              axle-balance/slip control, UART protocol, fault handling.
              no_std, zero hardware deps - runs natively on the host.
stm32_os/     firmware binary - wires aart-core to real STM32G4 peripherals
              (PWM, ADC, USART, interrupts) via stm32g4xx-hal.
DESIGN.md     architecture, pin allocation, testing strategy, milestones.
```

## Testing (three tiers — see DESIGN.md §4)

**1. Host unit tests** — pure logic, no target/emulator, fastest and most
comprehensive:
```
cd aart-core
cargo test
```

**2. Renode** — register-level peripheral tests against a hand-rolled
STM32G4 platform (Renode ships no official G4 support). `stm32_os.resc`'s
own header comment has the exact current steps (build the test ELF with
`--no-run`, point `$bin` at it, load the script, `start` in the Renode
monitor) — not a single one-line command. The `.repl`/Python shims
(`stm32g431.repl`, `adc_shim.py`, `rcc_shim.py`) are still G431-scoped
scratch work from before the G474 retarget (DESIGN.md §6.4), and Renode
has no G474 model either, so this tier is stale/unverified right now —
treat it as a starting point to extend, not a working test suite.

**3. Hardware / QEMU** — from `stm32_os/`, two `cargo` aliases (defined in
`.cargo/config.toml`) cover the full-firmware path:
```
cd stm32_os
cargo hw     # flash + run on the real Nucleo-G474RE, via probe-rs (safe default)
cargo qemu   # spoofed CPU-level tests under QEMU, no board required
```
`cargo hw` is the default runner — a bare `cargo test` with no board
attached fails fast ("no probe found") rather than hanging, which is what
happens if the real HAL clock-init path is ever run under QEMU by
accident. `cargo qemu` is known to hang rather than exit under some
`qemu-system-arm` builds/semihosting configs (pre-existing, unrelated to
firmware correctness — see the Dockerfile's own comment); if it hangs,
`cargo test --target thumbv7em-none-eabihf --features qemu --no-run`
still confirms the qemu-feature build compiles cleanly without actually
running it.

## Docker build

Build context must be the repo root (`stm32_os` depends on the sibling
`aart-core` crate, so Docker can't `COPY` from outside its context):
```
docker build -t stm32-os -f stm32_os/Dockerfile .
docker run --rm stm32-os
```
This builds `stm32_os` from scratch and runs `cargo qemu` inside the
container, without installing the Rust toolchain, `qemu-system-arm`, or
`gcc-arm-none-eabi` locally. Mainly useful for confirming the firmware
still compiles clean from scratch — the `cargo qemu` step it ends with
inherits the same known QEMU-hang caveat noted above.

## Status

Milestones M0–M6 (scheduler, drivers, commutation, dual-motor,
UART protocol, front/rear axle balance, fault handling) are implemented
and host-tested; see DESIGN.md §8 for details and open caveats. Nothing
here has been run on real hardware yet — every tunable gain/threshold in
`stm32_os/src/main.rs` is a placeholder pending real bench/track tuning.
