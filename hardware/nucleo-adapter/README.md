# Nucleo adapter board (KiCad)

Schematic for a power-stage adapter/daughterboard that plugs into the
Nucleo-G474RE dev board `stm32_os` targets. It's a physical realization of
the architecture already described in `DESIGN.md` §6.5 (power stage) and
§6.4/6.1 (GPIO pin allocation): reverse-polarity protection, boosted
gate-drive supply, logic LDO, shared bidirectional track-current sense, and
two independent motor driver+bridge+sense stages (front/rear axles).

## Layout

- `kicad/gen_schematic.py` — generates `adapter.kicad_sch` from a
  data-driven component/net list. Hand-authoring ~110 near-identical KiCad
  symbol-placement blocks directly is repetitive and error-prone; this
  script computes exact pin coordinates (component position + library pin
  offset, rotated) and emits both the symbol placement and a matching
  `global_label` at each net connection, so connectivity is correct by
  construction. Run `python3 gen_schematic.py`, then validate with
  `kicad-cli sch erc adapter.kicad_sch`.
- `kicad/adapter.kicad_sch` — the generated schematic. **Don't hand-edit
  this file** — it's regenerated from `gen_schematic.py` and edits will be
  lost. ERCs clean (0 errors; see "Known/accepted ERC findings" below).
- `kicad/AdapterSymbols.kicad_sym/`, `kicad/AdapterFootprints.pretty/` —
  project-specific parts (gate driver, bridge FETs, boost regulator, LDO,
  reverse-protection diode/zener) copied from
  [aartech-dev/RemoraNSR3.0](https://github.com/aartech-dev/RemoraNSR3.0),
  the real single-motor ESC board `DESIGN.md` §6.5 cross-references.
- `kicad/sym-lib-table`, `kicad/fp-lib-table` — project library tables
  (the local libraries above, plus standard KiCad system libraries).
- `kicad/bom.csv` — grouped BOM (`kicad-cli sch export bom`).

## What's a real, verified part vs. a placeholder value

**Real parts** (pin geometry read directly from library source, not
guessed from memory): TI DRV8300D (3-phase gate driver), TI CSD16327Q3
(30V N-channel MOSFET, ×12 bridge + ×1 reverse-protection), Diodes AP3012
(adjustable boost), AP2204K-3.3 (fixed 3.3V LDO), TI INA180-series
(unidirectional current-sense amp — see note below), Microchip MCP6002
(dual op-amp, used for the shared bidirectional current-sense stage).

**Placeholder values needing real EE verification before board bring-up**
(flagged inline in `gen_schematic.py` at the point each is used):
- Boost FB divider (R2/R3, targeting ~12V) and inductor/rectifier/cap
  values (L1/D2/C1/C2) — placeholders pending characterization against
  the AP3012 datasheet's actual FB reference voltage.
- DRV8300D `MODE`/`DT` bias resistors — currently pulled low via 10k to
  GND as a placeholder; the correct value (or pull direction) to select
  the intended PWM mode / dead time needs datasheet verification.
- Gate resistors (10Ω), bootstrap caps (100nF), BEMF divider ratio
  (47k/10k), virtual-neutral summing network (3×47k + 10k) — reasonable
  starting points, not yet characterized against real motor
  inductance/back-EMF or DRV8300D gate-drive current.
- Current-sense shunt values (5mΩ, both the shared and per-motor ones) —
  placeholders; real value depends on expected peak current and the
  sense amp's usable output swing.

## Deliberate simplifications vs. RemoraNSR3.0

- **Reverse-polarity protection**: RemoraNSR3.0 uses an undocumented
  multi-transistor discrete bias network (Q1/Q2/Q9-Q11/D1/D2) that
  couldn't be fully reverse-engineered from its flat netlist alone
  (`DESIGN.md` §6.5). This design uses a single N-FET "ideal diode"
  instead (low-side placement, gate biased toward the raw rail through a
  resistor, clamped by a zener) — a standard, well-understood topology
  chosen for correctness confidence over blind replication. Verified via
  explicit voltage-reasoning for both correct and reversed wiring, not
  just by copying the reference topology.
- **Shared bidirectional track-current sense**: rather than betting on an
  unfamiliar bidirectional current-sense IC, this uses a discrete
  difference amplifier (shunt + 4-resistor diff-amp, MCP6002) centered on
  a buffered VDD/2 reference — consistent with the firmware's existing
  `TRACK_CURRENT_ZERO_OFFSET` bidirectional-detection scheme.

## Known/accepted ERC findings

`kicad-cli sch erc adapter.kicad_sch` reports **0 errors**. Two categories
of warning are expected and were deliberately left rather than hacked
around:

- **`pin_not_driven` would appear on the DRV8300D `HIN1-3`/`LIN1-3` pins**
  were it not for the Nucleo-interface connectors added specifically to
  represent them — but even with those connectors present, a from-scratch
  reader should understand *why* they're not a defect: those 6 signals
  per motor are genuinely driven by the STM32G474's TIM1/TIM8 outputs on
  the Nucleo dev board, a separate physical board this single-sheet
  schematic doesn't (and can't) model. This is a normal, accepted
  ERC-visible property of any adapter/interposer board schematic, not a
  wiring defect.
- **`lib_symbol_issues` / `endpoint_off_grid` warnings**: cosmetic.
  `kicad-cli` run without a `.kicad_pro` doesn't consult `sym-lib-table`
  (harmless — every symbol used is fully embedded in the schematic's own
  `lib_symbols` cache, which is what ERC actually reads pin geometry
  from); off-grid warnings come from computing pin coordinates in exact
  mm rather than always landing on the 0.1"/2.54mm grid, and are purely
  visual.

## A non-obvious KiCad fact this generator depends on

For library symbols using `(extends "Base")` inheritance (e.g. TI's
INA180A2 extends INA180A1; the standard MCP6002-xSN extends LM2904),
`kicad-cli`'s ERC engine cannot resolve pin geometry for the *derived*
symbol from an embedded `lib_symbols` cache, even though the file loads
and looks structurally correct — every pin on a derived-symbol instance
gets silently reported as unconnected/dangling regardless of how correct
its computed coordinates are (confirmed via isolated minimal-schematic
testing, not just inferred). Since `extends`-based symbols only override
cosmetic properties and share identical pin geometry with their base, the
generator places the **base** symbol's `lib_id` directly and sets `Value`
to the real derived part name instead (see `gen_schematic.py`'s PINS table
comment above the `INA180A1`/`LM2904` entries).

## Not done here

No PCB layout, no footprint assignment (BOM's footprint column is empty —
footprints weren't the focus of this pass), no `.kicad_pro` project file.
This is a schematic-and-BOM-level design pass; turning it into an
actual board is further work.
