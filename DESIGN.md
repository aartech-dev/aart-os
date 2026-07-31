# aart-os — Design Document

Status: draft v1 (2026-07-30)
Target board: Nucleo-G474RE (STM32G474RET6, Cortex-M4F, 512 KB flash / 128 KB SRAM)
— retargeted post-M6 from the original Nucleo-G431RB (STM32G431RBT6, 128 KB
flash / 32 KB SRAM) for its extra ADC3/ADC4 peripherals; see §6.4.
Language: Rust, `no_std` / `no_main`

## 1. What "single user operating system" means here

This is **not** a POSIX-style multi-user, multi-process kernel. There is one
operator, one application, one privilege level, no MMU/MPU-enforced process
isolation, and everything links into a single flash image. "OS" means a small
set of reusable kernel services that the motor-control application is built
on top of:

- a **scheduler** (who runs when),
- a **driver layer** (PWM, ADC, UART, timers) behind small traits,
- **inter-task communication primitives** (lock-free queues, critical
  sections) so ISRs and background tasks can share data safely without
  `unsafe` sprinkled through application code,
- **fault/supervisor handling** (watchdog, stall/overcurrent detection).

Think "small cyclic-executive RTOS kernel", not "Linux". This keeps scope
achievable and keeps the hard-real-time commutation path simple to reason
about.

## 2. Goals / non-goals

**Goals**
- Run two independent sensorless BLDC commutation loops (ESCape32-style
  six-step trapezoidal, BEMF zero-cross detection) on one MCU.
- Command both motors' throttle and a steering bias over UART; run a simple
  electronic differential (skid-steer style speed split) between them.
- Estimate slip from the two motors' electrical speed (no extra sensor for
  now — see §7.3) and cut duty on the slipping side.
- Every non-hardware-specific piece of logic must be unit-testable with a
  plain `cargo test` on the host (Intel Linux), with **no emulator
  required**. Emulation (Renode) is reserved for the thin layer that
  actually touches registers.

**Non-goals (for now)**
- No FOC / field-oriented control — six-step trapezoidal only, matching
  ESCape32's baseline approach. FOC is a plausible future milestone, not v1.
- No dynamic memory allocation (`alloc`) — static/`heapless` only.
- No process isolation, no dynamic loading, no filesystem.
- No true ground-truth slip sensing yet (encoder/IMU) — left as a pluggable
  extension point, not built in v1 (see open questions).

## 3. What's already in the repo

Worth stating explicitly, since the new work builds directly on it:

- `stm32_os/` — firmware crate using `stm32g4xx-hal`, `cortex-m`,
  `cortex-m-rt`, `defmt`. Boots to `loop {}` today.
- `stm32_os/memory.x` — correct FLASH/RAM layout, now for the G474RET6
  (512K/128K) after the §6.4 retarget (was the G431RBT6's 128K/32K).
- Three already-working test paths (this turns out to be exactly the test
  pyramid this design needs, see §4):
  - `cargo hw` — real hardware over `probe-rs` (safe default runner).
  - `cargo qemu` — CPU-level tests on QEMU (`netduinoplus2`/F405 model)
    with hand-spoofed RCC/GPIO register writes in `tests/hal_test.rs`.
  - `renode stm32_os.resc` — a **hand-rolled STM32G431 platform**
    (`stm32g431.repl`) plus a Python RCC peripheral shim
    (`rcc_shim.py`) that fakes clock-ready bits, because Renode ships no
    G4 platform at all. This is the more faithful emulator of the two
    (real G431 memory map, GPIO, two basic timers) and is where new
    peripheral-level tests should land. Still G431-scoped even after the
    §6.4 retarget to the G474 — Renode has no G474 platform either, so this
    tier is stale pending someone porting the `.repl`/shims to the new
    chip's register layout (not yet done; not currently blocking anything,
    since tiers 1 and 3 cover the retarget work so far).

This project already solved the annoying part (getting *anything* to boot
under emulation for a chip Renode doesn't support). The new work is mostly
additive: a host-testable core crate, more drivers, and extending the
`.repl`/shims as new peripherals are touched.

## 4. Testing strategy (three tiers)

This is the part of the design to get right first, because "preferably
using an emulator" really means "preferably *not* using an emulator when we
don't have to."

| Tier | What runs | Where | Speed | Covers |
|---|---|---|---|---|
| 1. Host unit tests | pure logic: scheduler, commutation state machine, differential controller, UART protocol parser | `cargo test` on Intel Linux, **no emulator** | milliseconds | correctness of algorithms & edge cases |
| 2. Renode peripheral tests | HAL glue: register config, ISR wiring, PWM/ADC/UART peripheral behavior | `renode stm32_os.resc` (extends existing `.repl`/shims) | seconds | "did we program the peripheral correctly" |
| 3. Hardware validation | full firmware | `cargo hw` via probe-rs on the real Nucleo + real motors | manual/bench | timing/electrical reality that no emulator models (real BEMF waveforms, real current) |

The key architectural move that makes tier 1 possible: **split the repo
into a hardware-free core crate and a hardware-facing firmware crate.**

```
aart-os/
├── aart-core/          # NEW: no_std, ZERO hardware deps. Plain `cargo test`
│   │                     runs natively on the host — no target flag, no
│   │                     emulator.
│   ├── scheduler.rs     # cyclic executive (§5)
│   ├── commutator.rs    # six-step BLDC state machine (§7.1)
│   ├── diff_ctrl.rs     # electronic differential + slip estimate (§7.3)
│   └── protocol.rs      # UART command/telemetry line protocol (§7.2)
├── stm32_os/            # EXISTING: the firmware binary
│   ├── src/main.rs      # wires aart-core types to real peripherals + ISRs
│   ├── stm32g431.repl    # extended as new peripherals are used
│   ├── rcc_shim.py
│   └── tests/hal_test.rs
└── DESIGN.md
```

`aart-core` types take hardware-agnostic inputs (tick counts, ADC samples,
zero-cross timestamps, raw UART bytes) and produce hardware-agnostic
outputs (duty cycle 0.0–1.0, commutation step index, response bytes).
`stm32_os` is thin glue: read register → call into `aart-core` → write
register. Almost all edge-case testing happens in tier 1, at Intel Linux
`cargo test` speed, before anything ever touches Renode.

## 5. Kernel: cyclic executive

STM32G431 real-time commutation timing (tens of µs at high electrical
speed) is too tight for a generic tick-scheduled task to own directly — so,
same as ESCape32 and most small ESC firmware, **the hard-real-time
commutation step lives directly in a hardware interrupt** (timer
update / ADC watchdog), not in a scheduled task. The kernel's job is to
run everything *else*:

- A 1 kHz tick (SysTick or TIM6) drives a static cyclic executive: a fixed
  table of `(task_fn, period_ticks)` entries, checked and run in the main
  `loop`. No dynamic task creation, no heap.
- ISRs communicate with tasks (and vice versa) via `heapless::spsc`
  lock-free queues — e.g. the commutation ISR pushes the latest eRPM
  reading; the 1 kHz differential-control task pops it. No blocking, no
  priority inversion.
- Shared config (rarely-written) uses `critical-section` (already a
  dependency via `cortex-m`), not queues.
- NVIC priorities: commutation/ADC ISRs > SysTick tick > everything else,
  so the tick scheduler can never delay a commutation step.

`Scheduler` itself (the table + tick-checking loop) is pure logic living in
`aart-core`, host-tested by feeding it a synthetic tick counter and
asserting task cadence/ordering.

## 6. Driver layer

Behind small traits in `stm32_os`, backed initially by `stm32g4xx-hal`:

- **PWM**: TIM1 for motor A, TIM8 for motor B — both are STM32G4 "advanced"
  timers with complementary outputs + break input, which is what a 3-phase
  bridge with high/low-side FETs needs (same choice ESCape32-class firmware
  makes on similar parts).
- **ADC**: ADC1 for motor A, ADC2 for motor B, triggered off each motor's
  PWM TRGO, sampling phase voltage in the PWM off-time for BEMF zero-cross
  detection (standard sensorless six-step technique). Since the retarget
  to the G474 (§6.4), each motor's virtual-neutral node also gets its own
  dedicated, continuously free-running ADC (ADC3/ADC4) instead of sharing
  a rotation slot with phase/current sampling.
- **UART**: USART1 on PB6/PB7 for the command/telemetry protocol — **not**
  USART2/PA2-PA3 (the Nucleo ST-Link VCOM pins) as originally assumed; see
  §6.2 for why.
- **Tick**: SysTick or TIM6 at 1 kHz.

### 6.1 Pin allocation (from ESCape32's reference hardware)

ESCape32's [reference hardware wiki page](https://github.com/neoxic/ESCape32/wiki/ReferenceHardware)
gives a per-MCU pinout table; its STM32G431 column is:

| Function | Pin | Function | Pin |
|---|---|---|---|
| Input (DShot) | PA2 | Hall_A | PB3 |
| Telem | PB6 | Hall_B | PB5 |
| High_A | PA8 | Hall_C | PB7 |
| High_B | PA9 | Hall_XOR | PB4 |
| High_C | PA10 | ADC_Volt | PA6 |
| Low_A | PA7 | ADC_Curr | PF1 |
| Low_B | PB0 | WS2812 | PB8 |
| Low_C | PF0 | AUX | PA15 |
| BEMF_A | PA0 | | |
| BEMF_B | PA4 | | |
| BEMF_C | PA5 | | |
| BEMF_Ref | PA1 + PA3 | | |

I cross-checked every PWM pin above against `stm32g4xx-hal`'s own
`pins!` macro table for `TIM1` (`~/.cargo/registry/.../stm32g4xx-hal-0.1.0/src/pwm.rs`)
rather than trust the wiki table blind — all six (PA7/8/9/10, PB0, PF0)
are valid TIM1 alternate functions on this exact part. **This is the
motor-A pin set, used as-is.**

**This table is for one motor.** ESCape32 is single-motor ESC firmware; it
uses `BEMF_A` on ADC1 and `BEMF_B`/`BEMF_C` on ADC2 *simultaneously*
(confirmed against the HAL's ADC pin table: PA0→ADC1 ch1, PA4→ADC2 ch17,
PA5→ADC2 ch13, PA1/PA3→ADC1 ch2/ch4) — i.e. **one motor's sensing already
spans both of the G431's ADC peripherals.** The G431 (unlike the
G473/474/483/484, which have up to 5 ADCs) only has ADC1 and ADC2. A
second, independent motor cannot reuse ESCape32's exact scheme — there
is no third/fourth ADC to give it.

Two ways to resolve this, worth deciding explicitly rather than picking
silently:

1. **Time-multiplex** both motors' BEMF sampling across the same
   ADC1+ADC2, interleaved via each timer's own TRGO. Stays closer to
   ESCape32's exact reference circuit, but couples the two commutation
   loops' ADC timing and adds real firmware complexity.
2. **Give each motor its own ADC outright**, by replacing ESCape32's
   software-averaged dual-ADC reference (`PA1+PA3`) with a single
   physical virtual-neutral node (three summing resistors from the motor's
   three phases into one node, a completely standard sensorless-BLDC
   technique) sampled by one extra ADC channel. This costs 3 cheap
   resistors per motor on the board, but fully decouples the two motors:
   ADC1 belongs to motor A, ADC2 belongs to motor B, both can sample
   simultaneously, and `Commutator` (§7.1) stays hardware-symmetric between
   the two instances.

**Recommendation: option 2** — it's the simpler firmware (no interleaving
scheduler between two ISRs fighting over shared ADC hardware) and the BOM
cost is negligible. Flagging this here since it's a real deviation from
ESCape32's exact reference circuit and affects the schematic — say so if
you'd rather keep closer fidelity to ESCape32's reference and take on the
interleaving instead.

With option 2, the full two-motor pin table becomes:

| | Motor A (TIM1 / ADC1) | Motor B (TIM8 / ADC2) |
|---|---|---|
| High_A | PA8 | PA15 |
| High_B | PA9 | PB8 |
| High_C | PA10 | PB9 |
| Low_A | PA7 | PB3 |
| Low_B | PB0 | PB4 |
| Low_C | PF0 | PB5 |
| BEMF_A | PA0 | PA4 |
| BEMF_B | PA1 | PA5 |
| BEMF_C | PA3 | PA6 |
| Neutral (virtual, resistor-summed) | PB1 | PB14 |
| Current sense | PB11 | PF1 |
| Bus voltage (shared, one reading is enough) | PB12 | — |

(Motor B's neutral pin shown here already reflects the G474 retarget —
see §6.4. It was PB2 on the original G431 plan.)

**Drag-brake ("Latvian brake") track-current sense — shared, not
per-motor**: `PA2` (ADC1 channel 3) — see §6.5/7.5. This is a whole-car
reading (total current the track is delivering to the car), taken once on
the shared input rail, not duplicated per motor — physically hosted on
motor A's ADC1 purely because it's the only ADC1-reachable pin still free
on this 64-pin package once every other motor/UART pin was assigned.
Repurposes `PA2`, previously left unused/reserved for possible future
DShot input (§6.2/§9) — a real tradeoff, not a silent one: if DShot input
is ever wanted later, a different pin (or a different ADC1 channel) needs
finding for one of the two.

Motor B's TIM8 pins were derived the same way — checked against the HAL's
`TIM8` entry in `pins!`, filtered to the alternate-function options that
actually exist on the LQFP64 package this Nucleo uses (STM32G431's 64-pin
package only bonds out PC13–PC15 and PF0/PF1 from ports C/F; the wiki
table's `PC6`/`PC7`/`PC8` etc. TIM8 options don't exist on this package,
so they're excluded above). TIM8's `BRK` input didn't have a free pin left
after the rest of this table was assigned — treating motor B's hardware
break input as deferred/optional rather than forcing a further pin
reshuffle for it.

Command UART lands on USART1 (PB6/PB7, standard AF7 on this family —
confirm the exact AF number against RM0440 when the driver is written),
which is *not* PA2/PA3. This was a deliberate change from the original
assumption (§6.2).

### 6.2 Known Nucleo-G431RB conflicts — required board rework

The pin table above collides with three of this board's built-in demo
peripherals. All three are standard, documented Nucleo solder-bridge
reconfigurations (this is exactly what those bridges are for), not
anything unusual — but they need doing before power-up, so calling them
out explicitly:

- **PA0 = motor A's BEMF_A**, but PA0 is also the Nucleo **USER button**
  by default (solder bridge, commonly `SB21`). Desolder it to free PA0.
- **PA5 = motor B's BEMF_B**, but PA5 is also the Nucleo **user LED
  (LD2)** by default (commonly `SB6`). Desolder it to free PA5 — you lose
  the onboard LED as a debug indicator, which is fine since `defmt-rtt`
  already gives you logging over the SWD/ST-Link probe without needing
  any UART or GPIO pin at all.
- **PA3 = motor A's BEMF_C**, but PA2/PA3 are also wired to the ST-Link's
  onboard USB-UART bridge (USART2 VCOM) by default. This is the reason
  the command/telemetry UART was moved to USART1/PB6-PB7 above — but even
  with our own UART elsewhere, the ST-Link's UART transmitter is still
  physically driving PA3 whenever the board is USB-powered, which would
  actively corrupt sensitive BEMF analog sampling on that pin. The VCOM
  bridge (commonly `SB13`/`SB14` on Nucleo-64 boards, but **verify the
  exact bridge numbers against your board's UM2570 revision/silkscreen**
  — they've moved between revisions) needs desoldering too.

One nice side effect of moving the command UART off PA2/PA3: **PA2 ends
up completely unused** in this plan — which happens to be exactly
ESCape32's own "Input" pin slot for DShot. If DShot input is ever wanted
later, that's the pin already reserved for it, no rework needed.

Nothing else in the table above (PA1, PA4, PA6, PA7–PA10, PA15, PB0–PB5,
PB8, PB9, PB11, PB12, PF0, PF1) collides with a fixed Nucleo function —
only the three above need any board modification.

### 6.3 Renode implication

The current `stm32g431.repl` only models GPIO and two *basic* timers
(TIM2/TIM3). TIM1/TIM8 (advanced, complementary-output) and an ADC are not
modeled by any generic Renode peripheral for G4 — expect to extend the
`.repl` and very likely add a custom Python shim (same pattern as
`rcc_shim.py`) for ADC, since injected/BEMF-timing ADC behavior won't
exist in Renode's stock library. This is called out per-milestone below
rather than solved up front — build the shim when the milestone that needs
it arrives, same as was done for RCC.

### 6.4 Retarget: STM32G474RE / Nucleo-G474RE (post-M6)

Moved off the G431 to the bigger G473/G474/G483/G484 family, specifically
**STM32G474RET6** on a **Nucleo-G474RE** board — same LQFP64 package and
largely the same pinout as the Nucleo-G431RB used through M0–M6, but with
512K flash / 128K RAM (vs. 128K/32K) and five ADCs (ADC1–5) instead of two.
`Cargo.toml`'s `stm32g4xx-hal` feature and `memory.x`'s FLASH/RAM sizes were
updated accordingly; the `stm32g474` feature auto-enables the crate's
`adc3`/`adc4`/`adc5` features, so no separate feature flags were needed.

**Motivation**: §6.1's option 2 (a per-motor virtual-neutral node) already
decoupled the two motors' phase/current sensing onto ADC1/ADC2, but neutral
itself still had to share each motor's single ADC via a time-multiplexed
rotation (`NEUTRAL_SLOT`/`CURRENT_SLOT` in `main.rs`'s `step_motor`) — a
real firmware complication and a source of latency/staleness in the neutral
reference used for zero-cross detection. The G474's ADC3 and ADC4 let each
motor's neutral node get its own dedicated ADC, free-running in continuous
mode (`Continuous::Continuous`, no external trigger — see `sense.rs`), so a
fresh neutral sample is always sitting in the data register with no
rotation, no rearm, and no EOC interrupt bookkeeping needed for it.

**Pin/wiring impact — verified against the vendored `stm32g4xx-hal-0.1.0`
ADC pin tables (`src/adc/g4.rs`), which differ per chip feature**:

- Motor A's neutral pin (**PB1**) is unchanged — on the G473/474/483/484,
  PB1 maps to both ADC1 channel 12 (unused now) *and* ADC3 channel 1, so
  the existing wiring just gets a new dedicated ADC without a new wire.
- Motor B's neutral pin **moves from PB2 to PB14** — PB2 has no ADC3/4/5
  route on this family at all, so a genuinely new wire is required. PB14
  maps to ADC4 channel 4 (and ADC1 channel 5, unused). **This is the one
  real hardware change from the retarget** — everything else (both
  motors' TIM1/TIM8 PWM pins, both motors' BEMF/current/bus-voltage pins,
  the USART1 command UART on PB6/PB7) carries over unchanged, verified by
  cross-referencing ESCape32's own "STM32G431+" reference-hardware column
  against pins already in use here.
- ADC3 and ADC4 share one clock-config block, `ADC345_COMMON`, analogous
  to `ADC12_COMMON` — claimed once in `main.rs` (`claim_common_345`) and
  handed to both motors' sense setup by reference, same pattern as the
  existing `ADC12_COMMON` claim.
- §6.2's Nucleo solder-bridge conflicts (PA0/PA5/PA2-PA3) and their fixes
  are unaffected — none of those pins changed.

**Why not the G473/G483/G484 instead of G474**: all four share the same
ADC layout relevant here (adc3/adc4/adc5 alike), so any would have worked
for this specific problem. G474 (and G484) additionally have an HRTIM
purpose-built for motor control, not yet used by this project but worth
having available if TIM1/TIM8's resolution ever becomes limiting —
picking the one Nucleo board (Nucleo-G474RE) that already exists off the
shelf made this a low-cost option to keep open, not a commitment to use
HRTIM now.

**Renode**: no G474 (or G473/G483/G484) Renode platform exists any more
than a G431 one did — `stm32g431.repl`/`adc_shim.py` remain G431-specific
scratch work with no equivalent yet for this chip. This doesn't change
§6.3's conclusion (Renode ADC/TIM1/TIM8 support has to be hand-built
regardless of exact chip), it just means that work, whenever it happens,
targets the new part's register layout instead of the old one. Real
verification for this retarget so far is `cargo build --target
thumbv7em-none-eabihf` (debug and release) against the real `stm32g474`
feature, which catches pin/AF/ADC-channel mistakes at compile time even
without Renode or real hardware.

### 6.5 Power-stage reference circuit (adapted from RemoraNSR3.0)

§6.1's ESCape32 wiki table only ever covered the **MCU-side pinout** —
which GPIO does which job. It says nothing about the actual power
electronics: gate driver, FETs, gate-supply generation, reverse-battery
protection. For that, this section cross-references a real, built
single-motor ESC board's schematic —
[aartech-dev/RemoraNSR3.0](https://github.com/aartech-dev/RemoraNSR3.0/tree/main/kicad)
— read directly from its KiCad `.kicad_sch` files and fabrication netlist
(`kicad/production/netlist.ipc`), not just its rendered image, so the
part numbers and net-level topology below are traceable to source rather
than guessed from a picture.

**What RemoraNSR3.0 actually uses** (single motor):

| Function | Part | Ref | Notes |
|---|---|---|---|
| MCU | AT32F421G8U7 | U3 | Artery, pin/peripheral-compatible with STM32F0/G0-family 32-pin parts — a different MCU family from our STM32G474, but the same BEMF/virtual-neutral philosophy (see below) |
| 3-phase gate driver | **TI DRV8300D** | U4 | QFN-24EP. `HIN1-3`/`LIN1-3` inputs (named `UH/VH/WH`/`UL/VL/WL` in the schematic), `HO1-3`/`LO1-3` gate outputs, `VB1-3` bootstrap caps + `VS1-3` phase-return per leg, one `GVDD` gate-supply pin, `MODE`/`DT` (external dead-time resistor) pins. No SPI, no integrated current-sense amplifier. |
| Bridge FETs (×6) | **TI CSD16327Q3** | Q3–Q8 | 30V N-channel, SON 3.3×3.3mm. Confirmed from the netlist: Q3/Q5/Q7 drains land on the raw `VBUS` rail (high-side), Q4/Q6/Q8 drains land on the U/V/W phase nodes (low-side) — one high+low pair per leg, gated by the DRV8300D's `HOx`/`LOx` outputs. |
| Boost regulator | **Diodes AP3012** | U1 | Adjustable-output boost (external inductor `L1`, rectifier diode `D3`, FB divider). Confirmed: `D3`'s cathode is the `VCC` net, and `VCC` is what feeds the DRV8300D's `GVDD` pin — i.e. **the gate driver's supply is boosted, not taken directly off the battery/track rail**, so full gate enhancement is guaranteed even when the raw supply sags under load. |
| Logic LDO | AP2204K-3.3 | U2 | Fixed 3.3V LDO, `VIN` tied to the *boosted* `VCC` rail (not raw `VBUS`) — the MCU's 3.3V rail rides on top of the same boost that feeds the gate driver, rather than being a separate regulator off the raw input. |
| Reverse-polarity protection | **CSD16327Q3 (Q1)** + BAT40V Schottky (D1/D2) + small-signal transistors (DTC143Z, a small N-FET, a dual NPN/PNP pair, a PNP) | Q1, Q2, Q9–Q11, D1, D2 | A series power MOSFET, not a plain series diode — much lower conduction loss at this current than a diode-based scheme. The exact discrete self-bias network (how Q1's gate gets charged/held on) wasn't fully re-traceable from the flat netlist alone — confirmed fact is that a MOSFET, not a diode, carries the reverse-protection current; the discrete bias details would need the rendered schematic to pin down exactly. |
| Debug header | 6-pin connector | J1 | `VCC`/`VDD`/`GND` + 3 more (SWD-shaped) |

**Adapting this to our 2-motor front/rear layout**: the sensible split is
a **shared input/power-conditioning front end** feeding **two independent
per-motor driver+bridge stages** — duplicating the whole front end per
motor would double cost/board area for no benefit, since there's exactly
one power source (track/battery) for the whole car:

```
                    ┌─────────────────────────────────────────┐
                    │   SHARED (one power source, one car)     │
                    │                                           │
  track/battery ──► │  reverse-polarity protection              │
  in                │  (CSD16327Q3 + BAT40V, "ideal diode")     │
                    │        │                                  │
                    │        ▼  VBUS (raw, unregulated)         │
                    │        ├──────────────► to both bridges'  │
                    │        │                 high-side drains │
                    │        ▼                                  │
                    │  AP3012 boost ──► VCC (boosted gate-supply)│
                    │        │              │                   │
                    │        │              ├──► DRV8300D #1 GVDD│
                    │        │              └──► DRV8300D #2 GVDD│
                    │        ▼                                  │
                    │  AP2204K-3.3 LDO ──► VDD (3.3V)            │
                    │        └──────────────► STM32G474 + logic │
                    └─────────────────────────────────────────┘
                             │                        │
                             ▼                        ▼
              ┌─────────────────────────┐  ┌─────────────────────────┐
              │  MOTOR A (front axle)   │  │  MOTOR B (rear axle)    │
              │  DRV8300D #1            │  │  DRV8300D #2            │
              │  HIN1-3/LIN1-3 ◄─ TIM1  │  │  HIN1-3/LIN1-3 ◄─ TIM8  │
              │  6× CSD16327Q3 bridge   │  │  6× CSD16327Q3 bridge   │
              │  U/V/W ──► motor A      │  │  U/V/W ──► motor B      │
              │  virtual-neutral divider│  │  virtual-neutral divider│
              │  ──► ADC1 (phase) +     │  │  ──► ADC2 (phase) +     │
              │      ADC3 (neutral)     │  │      ADC4 (neutral)     │
              └─────────────────────────┘  └─────────────────────────┘
```

This maps directly onto what's already implemented, not just onto a new
design:

- Each `DRV8300D`'s `HIN1-3`/`LIN1-3` inputs are exactly what `motor.rs`'s
  `Bridge` already drives — TIM1's/TIM8's complementary PWM channels, one
  driver per motor, no change needed to the timer wiring in §6.1.
- Each `DRV8300D`'s phase-node virtual-neutral BEMF divider (their
  `UE`/`VE`/`WE`/`Eref` nets) is the same *concept* as our own per-motor
  virtual-neutral resistor node (§6.1 option 2, PB1/PB14) feeding the
  dedicated ADC3/ADC4 (§6.4) — this reference design independently
  arrived at the same sensing architecture we already built, which is a
  useful cross-check, not a new requirement.
- `DRV8300D` has no integrated current-sense amplifier, matching our own
  discrete current-sense pins (PB11/PF1, §6.1) rather than requiring a
  driver-side current-sense feature we'd have to newly support.
- One `AP3012` + one `AP2204K-3.3` is enough for both motors — GVDD and
  the 3.3V logic rail aren't per-motor concerns, so this doesn't scale
  with motor count.
- The reverse-polarity protection stage is sized once, for the combined
  current of both bridges, not duplicated per motor.

**What does scale with motor count**: two full `DRV8300D` + 6×
`CSD16327Q3` bridges (12 FETs total), two sets of bootstrap capacitors
(3 per driver), and two virtual-neutral divider networks — this is the
real per-motor BOM cost of the front/rear layout, and it's linear, not
quadratic, in motor count.

**Drag-brake addition**: each motor's own current-sense point (PB11/PF1)
upgrades from the bare shunt-to-ADC connection described above to a
dedicated current-sense amplifier reading a low-value shunt resistor in
that bridge's return path — better resolution for the existing overcurrent
fault check (`aart-core::fault::FaultSupervisor`). Drag-brake detection
itself, though, is deliberately **not** built from those two per-motor
readings: what it needs to know is whether the track is delivering power
to the car *at all*, which is a property of the single shared input rail,
not either motor's individual draw. So there's a third, separate
current-sense amplifier + shunt on the main input path (after
reverse-polarity protection, before it splits to the two bridges), feeding
its own ADC channel (`PA2`, ADC1 — see §6.1) sampled in software against a
configured threshold, once for the whole car.

This one is **bidirectional**, unlike the two per-motor amplifiers above:
current flows one way through the shared input while the car is driving/
accelerating (positive) and can flow the other way under braking/
regenerative conditions (negative), so detection has to collapse to zero
from *either* side, not just drop below a floor — see §7.5's
`current_present` for how the firmware handles that (magnitude of
deviation from the amplifier's zero-current reference, not the raw
sample's absolute value).

This was a deliberate choice over the alternative considered: a dedicated
hardware comparator IC (e.g. TI's INA300/INA301, current-sense amplifier +
integrated comparator + resistor-set threshold + digital alarm output)
would react without any MCU involvement at all, and would keep working
even if firmware hung — a real defense-in-depth argument for something
safety-adjacent like braking. Rejected here in favor of the amplifier +
shunt + software-threshold approach because: it's runtime-adjustable
(even over the existing UART protocol, no board rework to retune) rather
than fixed by a resistor value chosen at design time; it reuses the ADC/
ISR infrastructure already being built rather than adding a new IC; and
the latency argument for dedicated hardware matters for a *per-motor,
per-PWM-cycle* check, which this isn't — "track power gone" is a
tens-of-milliseconds event (a car going airborne), where 1kHz software
polling is plenty fast. See §7.5 for what the firmware does with it.

**Not yet done**: this section documents the *reference* circuit and how
it maps onto our pin/driver architecture — there is no schematic/PCB for
this project yet, only `aart-core`/`stm32_os` firmware targeting the
Nucleo-G474RE dev board. Turning this into an actual board (values for
the boost's inductor/FB divider, the bootstrap caps, the reverse-FET's
bias network, and the current-sense shunt) is real EE work this document
doesn't attempt to finish here.

## 7. Application modules (all in `aart-core`, hardware-agnostic)

### 7.1 Commutator (per motor)

**Revised understanding (post-M5): these are small 1106 slot car motors,
not RC/drone motors — there is no throttle input at all.** Track voltage
(external to this system) is what controls speed. This device's job is
just: sync each motor at power-on, then run it at ~100% duty ("no PWM")
permanently once synced, using PWM only for the differential's cornering
correction (see 7.3). The two-option-set decided in the M1/M2 planning
(BEMF-only slip, per-motor virtual-neutral ADC) still stands; what changed
is that "throttle" never meant a live speed command — see the open question
this raised, below.

State machine, one instance per motor:
- **Inputs**: zero-cross event timestamps, current tick. No throttle
  input — sync always ramps toward the same fixed `sync_target_erpm` every
  time, a configured motor characteristic (where BEMF becomes reliably
  detectable), not a per-run command.
- **Output**: current step index (1–6), computed next-commutation deadline
  (30° electrical after last zero-cross), duty to apply.
- **Sync (open-loop)**: commanded electrical rate ramps from
  `sync_start_erpm` to `sync_target_erpm` (rate configurable via
  `sync_ramp_erpm_per_step`), duty ramping in lockstep from
  `sync_start_duty` to `sync_max_duty` over the same range. Hands off to
  closed-loop only once the target rate is reached *and* a real
  zero-crossing has actually been measured (reaching the target rate alone
  isn't sufficient — BEMF might genuinely not be visible yet at a
  misconfigured target). If BEMF is never detected, sync holds at the
  target rate/duty indefinitely rather than forcing a handoff blind.
- **Running (closed-loop)**: duty pinned to `sync_max_duty` (~1.0, "no
  PWM") as the baseline; the differential controller (7.3) is the only
  thing that ever pulls it lower, per-motor, for cornering.
- **Fault**: stall detection (no zero-cross within an expected window at
  the current commutation rate) → `Fault::Stall`.
- **PWM switching frequency**: schedules 48kHz→96kHz linearly across an
  eRPM range (`PwmFrequencySchedule`, driven by `current_erpm()` — a
  best-effort speed estimate that's live throughout sync, unlike the
  stricter `electrical_rpm()` which stays `None` until BEMF is trusted).
  Applying this at runtime needs the timer's PSC/ARR rewritten directly
  (`Bridge::set_frequency_hz` in `stm32_os`), since `pwm_advanced()`'s
  builder only sets frequency once, before `.finalize()`.

**Known gap**: all of sync's eRPM/duty/ramp-rate numbers, and the PWM
frequency schedule's eRPM bounds, are placeholders pending real bench/track
tuning against actual 1106 motor characteristics (see `main.rs`'s
`motor_commutator_config`/`pwm_frequency_schedule`) — none of this was
derived from real motor data, just reasonable-order-of-magnitude guesses.

Host-tested by feeding synthetic zero-cross timestamp sequences (steady
RPM, accelerating, a dropped zero-cross, a stall, never-hands-off-without-
trusted-BEMF) and asserting step sequencing, timing, and fault
transitions — no hardware needed.

### 7.2 UART command/telemetry protocol

Simple line-based text protocol (bench-friendly):

```
> THR 0.35
> STEER -0.10
< SPD a=1200rpm b=980rpm slip=0.02 OK
```

Parsing/formatting is pure logic in `aart-core::protocol`, host-tested for
malformed input, out-of-range values, partial/split reads across buffer
boundaries. `stm32_os` does byte I/O against USART1 (PB6/PB7 — see 6.2,
*not* USART2/the ST-Link VCOM pins).

`THR` is still parsed and range-validated (useful for bench testing without
a real track/motors) but is a deliberate no-op on real hardware — see 7.1,
there's no throttle input for it to drive. `STEER` is the only command that
actually changes anything once a motor is Running.

### 7.3 Electronic differential + slip estimate

**Note on what "a"/"b" mean here**: `DiffController` itself is agnostic —
its mixing math and BEMF slip trim only need *a* commanded ratio and each
motor's eRPM, not what physically separates the two motors. This project
actually supports **two** selectable 2-motor layouts (motor A/B are either
left/right wheels or front/rear axles, chosen once at build time — see
§7.6), and `DiffController` is reused unchanged for either: only the
feedforward mapping from `steer_cmd` to the commanded ratio differs
between them (§7.6). The description below is written in the original
left/right framing this module was designed against, since that's still
the clearer way to explain the mixing/slip-trim math itself; §7.6 covers
what's different for front/rear.

**Inputs**: `base_duty` (in practice always `sync_max_duty`, i.e. ~1.0 —
there's no throttle command to vary it, see 7.1; kept as a parameter so
this module doesn't need to know that convention and bench tests can still
drive it directly), `steer_cmd` (from the UART protocol — folded through
`DriveLayout::bias` first, see §7.6, before it reaches this parameter),
`erpm_a`/`erpm_b` (from each `Commutator`, i.e. **electrical speed derived
from BEMF timing, not a ground-truth wheel speed** — this was the chosen
starting point, see the note below).

**Output**: `target_duty_a`, `target_duty_b` — a skid-steer-style split,
where `steer_cmd` biases one side down and the other up from `base_duty`.
Going straight at `base_duty ≈ 1.0`, this is what "no PWM once running"
actually reduces to: both sides at ~100%, PWM (a duty below that) only
ever appearing on whichever side a corner is biasing down.

**Slip estimate**: since there's no independent wheel-speed sensor,
`slip_estimate = normalize(erpm_a, erpm_b) − expected_ratio(steer_cmd)`.
When `|slip_estimate|` exceeds a threshold, cut duty on the faster
("slipping") side (basic traction-control clamp), via a small P/PI
controller.

**Known limitation, stated explicitly**: this is a *motor-vs-motor*
proxy, not a *wheel-vs-ground* measurement. It can't distinguish "this
wheel is actually losing traction" from "this wheel is simply unloaded /
off the ground" or from gearbox backlash — it only catches gross mismatch
between the two motors' electrical speed vs. what the steering command
implies. `SlipEstimator` is defined as a small trait for exactly this
reason: a future per-wheel encoder or IMU-based estimator can be dropped in
without touching `diff_ctrl.rs`'s control loop.

Host-tested with scripted `erpm_a`/`erpm_b` sequences, including an
injected slip scenario (one erpm suddenly jumps relative to the other),
asserting the controller cuts duty on the correct side.

### 7.4 Front/rear axle balance (`aart-core::axle_balance`)

One of this project's two selectable 2-motor chassis layouts (§7.6) is a
**solid-axle front/rear layout** (motor A drives the front axle, motor B
drives the rear — not left/right; there is no per-side split in hardware
at all in this layout, since each axle only has one motor). Front/rear
still needs balancing in a turn, but the geometry differs from left/right
in a way that matters for the control law, not just the labeling:

- **Left/right** (§7.3's original framing): whichever side is on the
  *outside* of the turn needs to run faster. The correction is symmetric
  and flips sign with turn direction — `steer_cmd`'s sign does this
  directly.
- **Front/rear**: the front axle sweeps a longer arc than the rear axle in
  *any* turn, left or right, because the yaw center sits behind the front
  axle regardless of which way the car turns — the same reason real AWD
  cars need a center differential or viscous coupling that only ever lets
  the front overrun the rear, never the reverse. So the correction is a
  function of steer **magnitude** only (how sharp the turn is), with a
  **fixed sign** (always biases toward the front) — it must not flip with
  `steer_cmd`'s sign the way left/right does.

Rather than change `DiffController` itself (its mixing/slip-trim math
doesn't need to know *why* the two motors should differ, only by how
much), `axle_balance::front_rear_bias(steer_cmd, AxleBalanceConfig)`
computes `-bias_gain * steer_cmd.abs()` (clamped to ±1) and feeds that
into `DiffController::update` in place of the raw `steer_cmd` — `main.rs`
wires this as a small feedforward stage in front of the existing
mixing/BEMF-slip-trim machinery, both for the commanded-ratio mix itself
and for what the slip estimator treats as the "no slip" target ratio.
`bias_gain` (`AXLE_BIAS_GAIN` in `main.rs`) is a placeholder pending real
tuning against this chassis's actual wheelbase and turn radii, same
caveat as every other tunable gain in this project.

Host-tested directly (`aart-core/src/axle_balance.rs`): zero steer gives
zero bias, a left turn and a right turn of equal sharpness bias
identically (unlike `DiffController`'s own steer-sign behavior), sharper
turns bias harder, and zero gain disables the feedforward entirely
(BEMF slip trim alone, no geometric prediction).

### 7.5 Drag brake ("Latvian brake") (`aart-core::brake`)

Slot car motors need *active* braking, not just coasting, at the one
moment they're most likely to get it wrong: when a car loses electrical
contact with the track, either because track power actually went off, or
because the car left the slot (derailed, or airborne off a jump). Until
now, the only thing that stops a motor is `Commutator`'s existing
`Stalled` response (no zero-cross seen in time) — which just calls
`Bridge::disable()`, true Hi-Z, pure coast. A coasting rotor keeps
freewheeling from momentum with no braking at all, so by the time the car
lands back on the track (or gets put back in the slot), its wheels are
spinning at the wrong speed relative to the track surface — exactly the
traction-loss moment this feature exists to prevent. Dead-shorting the
motor's phases instead converts that kinetic energy into heat, stopping
the wheels quickly.

**Detection — shared, one measurement for the whole car, not per-motor.**
What's being detected is whether the track is delivering power to the car
at all, which is a property of the single input rail, not either motor's
individual draw — so this is a *third* current-sense amplifier + shunt on
the main input path (§6.5), feeding its own ADC1 channel (`PA2`), read in
software. A dedicated hardware comparator IC (TI INA300-class: current-
sense amp + integrated comparator + resistor-set threshold + digital
alarm output) was considered and explicitly rejected in favor of this —
see §6.5 for the full reasoning (runtime-adjustability and reusing the
existing ADC/ISR infrastructure won out over the hardware comparator's
MCU-independence, since "track power gone" is a tens-of-milliseconds
event, not a per-PWM-cycle one).

**Bidirectional, not a simple above/below-floor threshold.** The shared
amplifier is bidirectional: current flows one way while driving/
accelerating (positive) and can flow the other way under braking/
regenerative conditions (negative) — a real track-power loss or
derailment collapses the reading to zero from *either* side, so
`aart_core::brake::current_present(sample, zero_offset, min_magnitude)`
checks the *magnitude* of the deviation from the amplifier's zero-current
reference point (`TRACK_CURRENT_ZERO_OFFSET` in `main.rs`, typically its
mid-scale output — unverified against a real amplifier's actual reference
design), not the raw sample's absolute value. `MIN_TRACK_CURRENT_MAGNITUDE`
(`main.rs`) is how far from that reference a reading has to be, in either
direction, to still count as "current present."

**`aart_core::brake::DragBrakeSupervisor`** (one instance for the whole
car, in `main.rs`'s slow 1kHz loop, not the ISR) turns that single
`current_present` boolean into an engage/release decision applied to
*both* motors' `Commutator`s together:

- **Deliberately decoupled from `Commutator`/`RunPhase` entirely** — it
  takes two plain bools (`eligible`, `current_present`) and returns
  whether to engage, nothing else. `main.rs` computes `eligible` as
  "either motor has gotten past its own `Startup` ramp" — before that,
  low total current is still completely normal (low commanded duty during
  the open-loop ramp), not a signal of anything; once *either* motor is
  driving for real, a low shared reading becomes meaningful.
- **Debounced** (`debounce_ticks`, `DRAG_BRAKE_DEBOUNCE_TICKS` in
  `main.rs`): requires that many *consecutive* 1kHz ticks of "no current"
  before engaging, rejecting a single noisy/glitched reading. A single
  `current_present` tick immediately resets the counter and, if already
  engaged, releases the brake right away — no debounce on the way out,
  since a false "current is back" reading is far less costly than a false
  "current is gone" one.

**`Commutator` gained a fourth `RunPhase`, `Braking`** — entered only via
an explicit `engage_drag_brake()` call from the code above (never derived
internally; `Commutator` has no current-sensing input of its own). While
`Braking`, `poll()` returns a fixed `drag_brake_duty` (a placeholder
tunable, `DRAG_BRAKE_DUTY` in `main.rs`, independent of `sync_max_duty`/
the differential controller's output — it's a dead-short chop ratio, not
a commutation duty) and ignores `on_zero_cross` entirely, since the
phases are shorted together, not floating per six-step — there's nothing
meaningful to detect. `release_drag_brake()` resets every ramp/BEMF field
and returns to `Startup`, exactly as if this were power-on: braking stops
the rotor outright, so there's no coasting speed left to resume from once
current returns.

**`stm32_os::motor::Bridge` gained `brake(duty)`**: dead-shorts all three
phases together through the low-side FETs (reusing exactly what
`apply_step`'s existing `PhaseState::Low` arm already does per-phase —
steady low-side conduction via the complementary output, high-side held
off) for `duty` of the time, coasting (true `disable()`) the rest. Full
engagement (`duty` == 1.0) is that low-side-short state held continuously.
A *tunable* brake strength needs chopping between the two states across
successive ISR firings at a fixed software-PWM resolution (16 steps) —
the hardware timer's own duty register has no further per-call knob once
a phase is already pinned "Low", so this couldn't reuse hardware PWM
chopping the way normal commutation duty does. Coarse, but this only
needs to be "hard enough to stop the wheels quickly," not switch cleanly
at a particular frequency.

**In the ISR** (`step_motor`, `main.rs`): while `Braking`, the BEMF
zero-cross check is skipped (meaningless with all phases shorted) but
current sampling continues on its usual rotation slots — both the
existing per-motor `CURRENT_SLOT` and the new shared `TRACK_CURRENT_SLOT`
(motor A's rotation only, see below) — so the slow loop keeps seeing
fresh readings throughout and can tell when current returns. The bridge
dispatch is a three-way match now (`Braking` → `bridge.brake(duty)`,
`Stalled` → `bridge.disable()`, else → normal `apply_step`) rather than
the old two-way Stalled/not-Stalled check.

**The shared track-current channel lives inside motor A's rotation, not a
new peripheral.** `SenseIsr` gained `configure_track_current()` with a
default no-op body — `MotorASense` overrides it (adds a `TRACK_CURRENT_SLOT`
to its existing 16-step rotation, reusing the same `current_sample()`
register read every other slot already uses, no new trait method needed
to read it back), `MotorBSense` doesn't need to implement anything.
`MotorIsrState` gained `latest_track_current`, generic across both
motors' state structs for simplicity, but only ever meaningfully populated
on motor A's — motor B's copy holds an unused leftover value. This
asymmetry (a "shared" reading physically hosted on one motor's ADC) is a
direct consequence of the pin-availability constraint explained in §6.5:
`ADC5`, the only otherwise-unused ADC peripheral on the G474, has no
channel reachable on this 64-pin package other than two pins already
committed to motor A's PWM, so it couldn't host this instead.

Host-tested (`aart-core/src/brake.rs`, `aart-core/src/commutator.rs`):
never engages during Startup no matter how long current is absent,
engages only after the configured debounce, a single good reading resets
the debounce counter, current returning releases immediately, stays
engaged across ticks while still Braking and still absent, `poll()`
returns the configured duty while Braking, zero-cross events are ignored
while Braking, `release_drag_brake()` actually restarts sync from scratch
(not just flips the phase flag), and `current_present` itself is tested
separately: false exactly at the zero offset, true for both a positive
(driving) and a negative (braking) deviation of equal magnitude, false
within the deadband on either side, true right at the deadband boundary,
and no overflow/panic at the raw sample range's extremes.

**Not yet done / open questions**: the debounce tick count, brake duty,
`TRACK_CURRENT_ZERO_OFFSET`, and `MIN_TRACK_CURRENT_MAGNITUDE` are all
placeholders pending real bench tuning, same caveat as every other
tunable in this project. A single shared measurement point can't
distinguish "track power actually off" from "one motor lost contact while
the other's still gripping" (e.g. a bumpy track jostling one wheel loose)
— the earlier per-motor design would have caught that case too, at the
cost of duplicated hardware for a scenario that hasn't been validated as
worth it either way; revisit if real track behavior shows it matters.

### 7.6 Drive layout selection (`aart-core::drive_layout`)

This project supports **two** selectable 2-motor chassis layouts on the
identical hardware/firmware base — a build-time choice
(`DRIVE_LAYOUT` in `main.rs`), not a runtime command, since it reflects
how the two motors are physically wired into the chassis:

- **`TwoWheelLeftRight`**: motor A = left wheel, motor B = right wheel,
  one driven axle — the original left/right differential §7.3 was
  designed against. `steer_cmd` feeds `DiffController` directly, signed
  and unmodified (clamped to `[-1, 1]`, the same range `DiffController`
  itself clamps to).
- **`FourWheelFrontRear`**: motor A = front axle, motor B = rear axle,
  both driven, solid axles (§7.4). `steer_cmd` is folded through
  `axle_balance::front_rear_bias` first — magnitude-only, fixed-direction,
  since front/rear balance doesn't flip sign with turn direction the way
  left/right does.

Both layouts reuse `DiffController`/`BemfSlipEstimator` completely
unchanged — the mixing math and BEMF-based slip trim don't need to know
*why* the two motors should differ, only by how much (same reasoning
§7.4 already established for front/rear specifically). `DriveLayout::bias
(steer_cmd, axle_bias_gain)` is the single dispatch point: `main.rs` calls
it once per tick and feeds the result into `diff_controller.update(...)`
in place of the raw `steer_cmd`, for either layout — `axle_bias_gain` is
simply ignored when `TwoWheelLeftRight` is selected.

Host-tested (`aart-core/src/drive_layout.rs`): `TwoWheelLeftRight` passes
`steer_cmd` through unmodified (and clamps at the bounds, and ignores the
axle-bias gain entirely), `FourWheelFrontRear` delegates to
`front_rear_bias` exactly (confirmed by checking that, unlike
`TwoWheelLeftRight`, a left and a right turn of equal sharpness bias
identically through it).

## 8. Milestones

Each milestone lists its acceptance criteria and which test tier(s) it
lands in.

| # | Milestone | Test tier(s) |
|---|---|---|
| M0 | Workspace split (`aart-core` + `stm32_os`); cyclic executive scheduler; SysTick @ 1kHz drives it; LED-blink task replaces `loop {}` | 1 (scheduler logic) + 2 (Renode: SysTick fires, GPIOA toggles — extends existing `hal_test.rs`) |
| M1 | PWM (TIM1/TIM8), ADC (ADC1/ADC2 w/ TRGO), USART1 drivers behind traits | 2 (Renode `.repl`/shim extension — register-level correctness) |
| M2 | `Commutator` for motor A: startup ramp, zero-cross tracking, six-step sequencing, stall detection | 1 (majority of test coverage) + 2 (PWM registers get written) |
| M3 | Second `Commutator` instance for motor B (independent, no shared state) | 1 (independence) + 2 |
| M4 | UART protocol: `THR`/`STEER` in, `SPD ... OK`/`ERR ...` out | 1 (parser) + 2 (bytes in/out via Renode UART) |
| M5 | Electronic differential + BEMF-based slip estimate + duty cut | 1 (scripted erpm sequences incl. slip scenario) |
| M6 | Faults: overcurrent, stall→safe-stop, UART comms-loss timeout, IWDG kicked only when all tasks healthy | 1 (fault transitions) + 2 (IWDG register behavior) |

Recommended order: M0 → M1 → M2 → M3 → M4 → M5 → M6. M2/M3 and M4 could be
swapped or parallelized once M1 lands, since they don't depend on each
other.

**Post-M5 addendum**: once M5 landed, it turned out the throttle-based
model M2–M5 were built against didn't match the real target hardware —
see the revised 7.1/7.3. Reworked before M6: `Commutator`'s sync ramp is
now eRPM-range-based (not step-count-based) targeting a fixed
`sync_target_erpm`, `Bridge` gained `set_frequency_hz` for the 48→96kHz PWM
switching-frequency schedule, and `DiffController`'s first parameter is
`base_duty` rather than a throttle command. M1's original PWM/ADC driver
work and M2/M3's commutation-loop wiring didn't need to change shape, only
the numbers/duty-source feeding them.

**M6 addendum**: `aart-core::fault::FaultSupervisor` aggregates overcurrent
(per-motor raw ADC sample vs. a configured limit), stall (passed in from
each `Commutator::phase()`, not duplicated), and UART comms-loss (a timeout
since the last successfully parsed line, of either command — not just
`STEER` — since comms-loss is about whether the link is alive at all) into
one `FaultStatus`, host-tested for each fault independently plus the
timeout's edges. `all_healthy()` is the single bit that gates the IWDG feed
in `stm32_os`, exactly as this table originally specified. Wiring this up
surfaced a real bug in the M2-era stall handling: `apply_step(..., 0)` on a
stalled motor left two phases actively driven low (chopped to 0% duty,
which is *not* the same as off) rather than truly disabled — fixed by
calling `Bridge::disable()` (true Hi-Z on all three phases) instead,
finally giving `disable()` its first real caller. Overcurrent gets the same
`disable()` treatment, applied every tick the condition persists (simpler
than edge-detecting when it first trips). Comms-loss resets `steer_cmd` to
0 (fail-safe straight) rather than disabling anything, since losing the
steering link isn't itself a hardware safety issue for a motor that's
otherwise running fine. `IndependentWatchdog` (already implemented in
stm32g4xx-hal, not hand-rolled) is fed once per outer loop iteration only
when `all_healthy()` — a persistent, uncorrected fault eventually resets
the whole MCU as a last-resort recovery path, on top of (not instead of)
the immediate per-fault software mitigation above. All the thresholds
(`OVERCURRENT_LIMIT`, `COMMS_TIMEOUT_TICKS`, `IWDG_TIMEOUT_MS`) are
placeholders, same caveat as every other tunable constant introduced since
M2.

**Post-M6: ISR-driven commutation.** M2 through M6 ran the entire
commutation loop (BEMF sampling, zero-cross detection, stepping, duty
application) from the slow 1kHz SysTick-driven main loop - explicitly
flagged as a placeholder every time, since a single commutation step at
these motors' real electrical rates can be tens of microseconds, far
shorter than the 1ms tick could even resolve. That's now fixed:

- The fast path moved entirely into the shared `ADC1_2` interrupt (STM32G4
  routes ADC1 and ADC2 to one NVIC vector), each motor's own ADC
  hardware-triggered off its PWM TRGO. `MotorIsrState<Br, S>` bundles a
  motor's `Commutator`/`ZeroCrossDetector`/`Bridge`/sense front-end, owned
  by a `cortex_m::interrupt::Mutex`-protected `static` per motor
  (`MOTOR_A`/`MOTOR_B` in `main.rs`) so the ISR and the slow loop can both
  reach it safely. `Bridge`'s channel types had to become concrete (type
  aliases in `motor.rs`) instead of `impl Trait`, since opaque
  return-position types can't appear in a `static`'s type.
- The 1kHz tick is far too coarse for `Commutator`'s own timing too, so its
  tick source changed to the Cortex-M DWT cycle counter (~16MHz, the core
  clock) - a 32-bit hardware counter that wraps in seconds, hence
  `aart-core::tick::TickExtender` (host-tested), which turns it into a
  monotonic 64-bit tick by detecting wraparound. Safe because the ISR reads
  it far more often than a 32-bit counter could ever wrap.
- ADC sampling changed from the M1-era blocking `.convert()` (which
  force-starts a software-triggered one-shot, ignoring any configured
  hardware trigger) to `DynamicAdc` reconfigured/re-armed once per ISR
  firing: `Adc<ADC,Disabled>` doesn't expose `into_dynamic_adc` (only
  `PoweredDown` does), so construction now goes claim → power_down (type
  state only) → `into_dynamic_adc` → power_up again. Each firing samples
  one channel on a fixed rotation - the floating phase 14/16 of the time,
  virtual-neutral and current each 1/16 - reconfiguring `Sequence::One` and
  re-arming (`ADSTART` auto-clears after each single-conversion trigger)
  before returning. The rotation split is a placeholder, not measured.
- The slow loop no longer touches commutation at all; it reads/writes
  shared state through `with_motor` (a short critical section per call) for
  the differential controller, PWM-frequency scheduling, fault supervision,
  and telemetry - matching the kernel model in section 5 that was written
  before any of M2-M6 existed.
- NVIC priority: `ADC1_2` set to the highest priority, `SysTick` explicitly
  lowered, so the fast path always preempts the slow one.

**What's unverified**: everything above compiles cleanly for the real
target (debug, release, and the `cargo qemu` test-binary path all checked),
but the actual real-time margins cannot be checked in this environment -
there's no hardware here, and Renode's `adc_shim.py` (see section 6.3) is a
passive register shim with no interrupt generation, so it can't exercise
this path even in emulation. One thing to check on real hardware before
trusting this: whether the 14/16-floating sampling rotation gives clean
enough zero-cross timing in practice - that split is a placeholder, not
measured.

**Immediately after, the clock itself got fixed**: HSI16/no-PLL (the
default the whole project ran on through M0-M6) was the root cause behind
*both* caveats this milestone flagged - PWM duty resolution (`motor.rs`)
and the ISR's CPU budget. `main.rs` now runs the PLL at 170MHz (M=÷4,
N=×85, R=÷2 from HSI16 - the documented max for this series), which needs
Range1 boost mode (`PWR`'s `vos(Range1{enable_boost:true})` plus
`Config::boost(true)`; the HAL's `freeze()` handles the full documented
transition sequence - AHB pre-halving, wait-state adjustment, switch -
internally, not hand-rolled here). Both duty resolution and ISR headroom
scale directly with this (roughly 10.6x more of each). One real bug this
surfaced: `motor_commutator_config()` had `tick_hz` hardcoded to a literal
16MHz rather than reading the actual configured clock, which would have
silently mismatched the DWT counter's real rate the moment the clock
changed - now takes `core_hz` read from `rcc.clocks.sys_clk` at runtime.
Also had to move the ADC clock divider from HCLK/2 to HCLK/4
(`sense::claim_common`): at 16MHz, /2 (8MHz) was nowhere near the ADC's
~60MHz max input clock; at 170MHz, /2 (85MHz) would have been out of spec.
Still unverified for the same reason as everything else here: no hardware
to confirm the boost-mode transition and higher clock actually behave as
the HAL's implementation (matching RM0440's documented sequence) intends.

**Post-M6: retarget to STM32G474RE.** See §6.4 for the full pin-level
detail; summarized here for the milestone history:

- `Cargo.toml`'s `stm32g4xx-hal` feature moved from `stm32g431` to
  `stm32g474` (auto-enabling the crate's `adc3`/`adc4`/`adc5` features);
  `memory.x` updated from 128K/32K to 512K/128K.
- `sense.rs` gained a dedicated, continuously free-running ADC per motor
  for virtual-neutral sensing (ADC3 for motor A reusing the existing PB1
  pin, ADC4 for motor B on a new PB14 pin — PB2, the old neutral pin, has
  no ADC3/4/5 route on this family at all) via a new `ADC345_COMMON` claim
  and a new `SenseIsr::neutral_sample()` method.
- `main.rs`'s ISR rotation (`step_motor`) simplified from a 3-way split
  (floating-phase/neutral/current) to 2-way (floating-phase/current) —
  `NEUTRAL_SLOT` and the `latest_neutral` cache are gone; the fast path
  just reads `sense.neutral_sample()` directly every firing, since that
  ADC never stops converting.
- Verified via `cargo build --target thumbv7em-none-eabihf` (debug and
  release) against the real `stm32g474` feature — no hardware or Renode
  model exists for this chip either (§6.3), so compile-time pin/AF/ADC-
  channel checking is the verification ceiling here, same as it was for
  the G431. All 66 `aart-core` host tests still pass unchanged, since none
  of this touched hardware-agnostic logic.

**Post-G474-retarget: front/rear axle balance.** Clarified that this
project's actual chassis is a 2-motor front/rear layout (motor A = front
axle, motor B = rear axle, both solid-axle), not left/right — no hardware
change from this (still the same 2 motors, same G474 PWM/ADC allocation),
but the control law needed a new piece: see §7.4 for the full reasoning.
Summary: new `aart-core::axle_balance` module (`front_rear_bias`, 5 host
tests) maps the signed `steer_cmd` to a magnitude-only, fixed-direction
bias, because unlike left/right — where the correction flips sign with
turn direction — the front axle sweeps a longer path than the rear in any
turn, so the bias must not flip. `DiffController` itself needed no
changes; `main.rs` just feeds `front_rear_bias`'s output into it instead of
raw `steer_cmd` (renamed `diff_controller` to `axle_controller` for
clarity). New `AXLE_BIAS_GAIN` constant, same placeholder caveat as every
other tunable. Verified: 71 `aart-core` host tests pass (66 + 5 new), and
`stm32_os` builds cleanly (debug + release, real `stm32g474` feature) with
no warnings.

**Post-axle-balance: drag brake ("Latvian brake").** New hardware and a
new `Commutator` phase to actively brake both motors (not just coast) when
the car loses electrical contact with the track — see §6.5/§7.5 for the
full reasoning. Summary: a third, shared current-sense amplifier + shunt
on the main input rail (not per-motor - alongside upgrading each motor's
own existing bare-shunt current sense to a proper amplifier too) feeds a
new ADC1 channel (`PA2`, repurposing the pin previously reserved for
possible DShot input), read in software against a configured threshold -
a dedicated hardware comparator IC was considered and rejected in favor of
this (runtime-adjustable, reuses existing ADC/ISR infrastructure, and the
speed argument for hardware doesn't really apply to a tens-of-milliseconds
event). New `aart-core::brake` module (`DragBrakeSupervisor`, 6 host
tests, decoupled from `Commutator`/`RunPhase` entirely - takes plain
bools) debounces that shared reading and drives both motors' brakes
together. `Commutator` gained a fourth `RunPhase::Braking`,
`engage_drag_brake()`/`release_drag_brake()`, and a `drag_brake_duty`
config field (4 new host tests). `stm32_os::motor::Bridge` gained
`brake(duty)`, dead-shorting all three phases via the existing
`PhaseState::Low` handling, duty-chopped across ISR firings at a fixed
16-step software-PWM resolution (the hardware timer's own duty register
has no further per-call knob once a phase is pinned Low). `main.rs`'s ISR
dispatch became a three-way match (Braking/Stalled/else), and motor A's
rotation gained a `TRACK_CURRENT_SLOT` alongside its existing
`CURRENT_SLOT` to host the shared reading (`SenseIsr::configure_track_current`,
default no-op so `MotorBSense` needs no changes).
Verified: 81 `aart-core` host tests pass (71 + 10 new), and `stm32_os`
builds cleanly (debug + release, real `stm32g474` feature) with no
warnings. Not verified: the debounce count, brake duty, and
`MIN_TRACK_CURRENT` threshold are all placeholders pending real bench
tuning - no hardware exists yet to tune them against.

**Post-drag-brake: bidirectional current detection + selectable drive
layout.** Two follow-ups from the same conversation - see §6.5/§7.5 and
§7.6 for the full reasoning.

Bidirectional detection: the shared track-current amplifier needed to
collapse-to-zero from *either* positive (driving/accelerating) or negative
(braking/regenerative) current, not just drop below a floor - a plain
unipolar threshold would have missed a power-loss event that happened to
occur while current was flowing backward. New `aart_core::brake::
current_present(sample, zero_offset, min_magnitude)` checks the magnitude
of deviation from the amplifier's zero-current reference instead of the
raw sample's absolute value (6 new host tests: false at the reference
point, true for positive and negative deviations of equal magnitude,
false within the deadband on both sides, true at the deadband boundary,
no overflow at the sample range's extremes). `main.rs`'s old
`MIN_TRACK_CURRENT` constant split into `TRACK_CURRENT_ZERO_OFFSET` and
`MIN_TRACK_CURRENT_MAGNITUDE`.

Selectable drive layout: the left/right differential from before the
front/rear pivot (§7.3's original framing) is back, as an equally-valid,
explicitly selectable second layout rather than something the front/rear
work silently replaced. New `aart_core::drive_layout` module
(`DriveLayout` enum, `TwoWheelLeftRight`/`FourWheelFrontRear`, 4 host
tests) is the single dispatch point between the two - both reuse
`DiffController`/`BemfSlipEstimator` completely unchanged, only the
`steer_cmd` feedforward differs. `main.rs` gained a `DRIVE_LAYOUT`
build-time constant and renamed `axle_controller`/`axle_bias` back to
`diff_controller`/`bias_cmd` (no longer implies front/rear-only).

Verified: 91 `aart-core` host tests pass (81 + 10 new), and `stm32_os`
builds cleanly (debug + release, real `stm32g474` feature) with no
warnings.

## 9. Open questions (not blocking M0, but worth revisiting)

- **Real slip ground-truth**: if/when the BEMF-only proxy proves too weak
  in practice, the next step is a per-wheel Hall/quadrature encoder (this
  was the runner-up option) — `SlipEstimator` is already shaped for that
  swap.
- **DShot input**: if this ever needs to talk to a flight
  controller/RC receiver instead of a bench UART terminal, that's a second
  implementation of the command-input side of §7.2, not a redesign.
- **FOC**: only worth it if six-step trapezoidal proves too rough/noisy for
  the target motors; not in scope for v1.

## 10. Immediate next step

Implement M0: split out `aart-core`, move nothing hardware-specific into
it yet except the `Scheduler`, write its host unit tests, then wire it into
`stm32_os/src/main.rs` behind a 1 kHz SysTick, replacing the bare `loop {}`.
This is small, fully host-testable, and gives every later milestone a place
to register tasks.
