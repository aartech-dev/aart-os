# Nucleo adapter board (KiCad)

Schematic for a power-stage adapter/daughterboard that plugs into the
Nucleo-G474RE dev board `stm32_os` targets. It's a physical realization of
the architecture already described in `DESIGN.md` §6.5 (power stage) and
§6.4/6.1 (GPIO pin allocation): reverse-polarity protection, boosted
gate-drive supply, logic LDO, shared bidirectional track-current sense, and
two independent motor driver+bridge+sense stages (front/rear axles).

## Layout

Hierarchical project, 4 sheets:

- `RemoraNextGen.kicad_pro` — the KiCad project file; open this in KiCad.
- `RemoraNextGen.kicad_sch` — root sheet. Draws the Nucleo-G474RE itself as a
  labeled-pin block (there's no real circuit of ours inside it, so a block
  with named pins is the right level of detail — see "The Nucleo block"
  below), plus three sub-circuit sheets (`Power`, `Motor A`, `Motor B`) as
  boxes with their own named pins, wired together with real point-to-point
  wires between matching pins — open this file first.
- `power.kicad_sch` — shared front end: track power in,
  reverse-polarity protection, boost regulator, logic LDO, shared
  bidirectional track-current sense, bus-voltage divider.
- `motor_a.kicad_sch`, `motor_b.kicad_sch` — one DRV8300D +
  6×CSD16327Q3 bridge + bootstrap caps + BEMF dividers + virtual-neutral
  summing network + current-sense amp each (front/rear axles).
- `gen_schematic.py` — generates all 4 files above from a
  data-driven component/net list. Hand-authoring this many near-identical
  KiCad symbol/sheet-placement blocks directly is repetitive and
  error-prone; this script computes exact pin coordinates (component
  position + library pin offset, rotated) and emits both the placement and
  a matching label at each net connection, so connectivity is correct by
  construction. Run `python3 gen_schematic.py`, then validate with
  `kicad-cli sch erc RemoraNextGen.kicad_sch` (this walks the whole hierarchy,
  not just the root file). **Don't hand-edit the 4 generated `.kicad_sch`
  files** — they're regenerated from this script and edits will be lost.
- `AdapterSymbols.kicad_sym/`, `AdapterFootprints.pretty/` —
  project-specific parts (gate driver, bridge FETs, boost regulator, LDO,
  reverse-protection diode/zener) copied from
  [aartech-dev/RemoraNSR3.0](https://github.com/aartech-dev/RemoraNSR3.0),
  the real single-motor ESC board `DESIGN.md` §6.5 cross-references.
- `sym-lib-table`, `fp-lib-table` — project library tables
  (the local libraries above, plus standard KiCad system libraries).
- `bom.csv` — grouped BOM across the whole hierarchy
  (`kicad-cli sch export bom RemoraNextGen.kicad_sch`).

## The Nucleo block

`RemoraNextGen.kicad_sch` draws the Nucleo-G474RE as a custom black-box symbol
(authored inline in `gen_schematic.py`, not a real KiCad library part —
there's nothing about the Nucleo's own circuit that's ours to design) with
one pin per net it actually exchanges with this board: the 22 motor GPIO
signals (HIN/LIN/BEMF/neutral/current-sense ×2 motors, DESIGN.md 6.4's pin
table), the shared `PA2`/`PB12` sense signals, and `VDD`/`GND`. Each pin is
wired with a real, visible point-to-point connection to whichever sub-sheet
block owns that net — not just same-named labels left to match invisibly —
so the top-level sheet reads as an actual block diagram of how the boards
interconnect.

One consequence worth flagging explicitly: `VDD` (the adapter's own LDO
output) is wired directly to the Nucleo's 3V3 pin, matching DESIGN.md's
plan for this connection. Backfeeding a dev board's 3V3 rail from an
external regulator while its own onboard regulator is also active is not
generally safe practice (two active sources on one rail) — this likely
needs the Nucleo's own regulator disconnected (in the same spirit as the
SB21/SB6 solder-bridge cuts DESIGN.md 6.2 already documents for freeing
GPIO pins) before this connection is made on real hardware. Not resolved
here; flagged for whoever does the board bring-up.

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

## Layout notes

`RemoraNextGen.kicad_sch` is A0 (the Nucleo block plus three sheet boxes, wired,
spans well past A1); `power`/`motor_a`/`motor_b.kicad_sch` are each A2
(each sub-circuit alone still exceeds A3 - this bit the project twice,
first on the original flat single-sheet layout, then again on each
individual sub-sheet after the hierarchical split, both times as silent
clipping in page-bounded exports rather than a load/ERC failure, so it's
easy to miss - check a sheet's actual drawing extent against its declared
paper size after any layout change, don't just trust the paper size that
was already there).

Every pin gets a short stub wire before its label lands, rather than the
label sitting directly on the pin tip — needed because several parts here
(AP3012, DRV8300D, CSD16327Q3) have multiple pins within a few mm of each
other, and a label glued straight to the pin crowds into neighboring
pins/labels. `place()`'s `no_connect` parameter marks genuinely-unused
pins (AP2204K-3.3's NC, DRV8300D's two NC pins) with a proper KiCad
no-connect flag instead of leaving a stub wire dangling to nowhere.

While untangling one such overlap (three labels stacked on the shared
current-sense shunt), found and fixed a real topology bug: the comment
already documented that the shunt must sit in series before the bridges
split (DESIGN.md 6.5 — "before it splits to the two bridges"), but the
bridges' high-side FET drains were wired to the pre-shunt `VBUS` net
directly, bypassing the shunt entirely. Fixed by wiring the bridges to
`VBUS_LOAD` (the shunt's downstream side) instead, so the shared
current-sense amp actually sees load current now rather than a stray
branch current.

On `RemoraNextGen.kicad_sch`, the four rails shared between `Power` and both motor
sheets (`VCC`/`VDD`/`GND`/`VBUS_LOAD`) are **not** routed as a straight
wire down the sheets' shared edge - that corridor also carries the Nucleo
GPIO wires and the other rails' own sheet pins, and a long wire segment
picks up *any* pin its path happens to pass through even without an
explicit vertex there. That's exactly what merged `VCC`/`VDD`/`GND` into
one net the first time this was tried (confirmed via `kicad-cli sch erc`
reporting real `pin_to_pin` conflicts between totally unrelated PWR_FLAGs
once the hierarchy was assembled, even though each sheet checked out fine
standalone). Each rail instead jogs out to its own dedicated lane past
every sheet's right edge, where nothing else runs.

## Known/accepted ERC findings

`kicad-cli sch erc RemoraNextGen.kicad_sch` reports **0 errors** across the whole
hierarchy. Two categories of warning are expected and were deliberately
left rather than hacked around:

- **`lib_symbol_issues` / `endpoint_off_grid` warnings**: cosmetic.
  ERC doesn't resolve pin geometry against `sym-lib-table` (harmless —
  every symbol used is fully embedded in each file's own `lib_symbols`
  cache, which is what ERC actually reads pin geometry from); off-grid
  warnings come from computing pin coordinates in exact mm rather than
  always landing on the 0.1"/2.54mm grid, and are purely visual.
- **`multiple_net_names`**: intentional net aliasing (e.g. `VBUS` and
  `TRACK_PWR` are the same physical node by design - see the module
  comments in `gen_schematic.py`'s `build_power()`).

## Non-obvious KiCad facts this generator depends on

- **`(extends "Base")` inheritance**: for library symbols using this (e.g.
  TI's INA180A2 extends INA180A1; the standard MCP6002-xSN extends
  LM2904), `kicad-cli`'s ERC engine cannot resolve pin geometry for the
  *derived* symbol from an embedded `lib_symbols` cache, even though the
  file loads and looks structurally correct — every pin on a
  derived-symbol instance gets silently reported as unconnected/dangling
  regardless of how correct its computed coordinates are (confirmed via
  isolated minimal-schematic testing, not just inferred). Since
  `extends`-based symbols only override cosmetic properties and share
  identical pin geometry with their base, the generator places the
  **base** symbol's `lib_id` directly and sets `Value` to the real
  derived part name instead (see `gen_schematic.py`'s PINS table comment
  above the `INA180A1`/`LM2904` entries).
- **Multi-unit / multi-drawing symbol children must stay bare**: a
  symbol's nested sub-drawings (e.g. `Name_0_1` for the body graphic,
  `Name_1_1` for a unit's pins) are associated with their parent purely by
  name-prefix convention against the parent's *unqualified* name — never
  the full `Library:Name` lib_id, even though the parent itself must use
  the qualified form. Qualifying the children too looks more consistent
  but silently breaks pin-geometry lookup for every symbol that has any
  (confirmed twice: once for the `AdapterSymbols:*` parts early in this
  project, then again when hand-authoring the Nucleo block's own custom
  symbol from scratch and making the exact same mistake a second time).
- **`(embedded_fonts no)` must be the last element inside a symbol
  definition.** A hand-authored symbol missing it fails to load with no
  more specific error than "Failed to load schematic" - every symbol
  extracted from a real library file already has this because it's part
  of the original block, so it only bites when authoring one from
  scratch (as the Nucleo block's custom symbol is).
- **Hierarchical sheet pins connect like symbol pins, not like labels**:
  a `hierarchical_label` inside a child sheet only participates in that
  sheet's own local net-matching-by-name; crossing to the parent sheet
  requires an explicit, separately-declared `(pin ...)` on that sheet's
  `(sheet ...)` block in the parent, wired there like any other point (see
  `place_sheet()`). Confirmed empirically end-to-end with a minimal
  parent/child test before relying on it project-wide.
- **A wire connects to any pin its path crosses, not just its declared
  endpoints.** See "Layout notes" above — this is what caused the
  `VCC`/`VDD`/`GND` merge bug during hierarchy assembly.

## Not done here

No PCB layout, no footprint assignment (BOM's footprint column is empty —
footprints weren't the focus of this pass). This is a schematic-and-BOM-level
design pass; turning it into an actual board is further work.
