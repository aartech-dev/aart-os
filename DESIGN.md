# aart-os — Design Document

Status: draft v1 (2026-07-30)
Target board: Nucleo-G431RB (STM32G431RBT6, Cortex-M4F, 128 KB flash / 32 KB SRAM)
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
- `stm32_os/memory.x` — correct FLASH/RAM layout for the RBT6 (128K/32K).
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
    peripheral-level tests should land.

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
- **ADC**: ADC1 for motor A, ADC2 for motor B (the G431 only has two ADC
  peripherals total — see §6.2), triggered off each motor's PWM TRGO,
  sampling phase voltage in the PWM off-time for BEMF zero-cross detection
  (standard sensorless six-step technique).
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
| Neutral (virtual, resistor-summed) | PB1 | PB2 |
| Current sense | PB11 | PF1 |
| Bus voltage (shared, one reading is enough) | PB12 | — |

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

**Inputs**: `base_duty` (in practice always `sync_max_duty`, i.e. ~1.0 —
there's no throttle command to vary it, see 7.1; kept as a parameter so
this module doesn't need to know that convention and bench tests can still
drive it directly), `steer_cmd` (from the UART protocol), `erpm_a`/`erpm_b`
(from each `Commutator`, i.e. **electrical speed derived from BEMF timing,
not a ground-truth wheel speed** — this was the chosen starting point, see
the note below).

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
