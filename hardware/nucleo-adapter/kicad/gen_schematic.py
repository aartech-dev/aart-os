#!/usr/bin/env python3
"""Generates the adapter's hierarchical KiCad project: main.kicad_sch (root -
draws the Nucleo-G474RE as a block, plus three sub-sheet blocks, wired
together with real point-to-point wires between named pins) and three child
sheets (power.kicad_sch, motor_a.kicad_sch, motor_b.kicad_sch) holding the
actual circuits.

Hand-authoring this many near-identical KiCad symbol/sheet-placement blocks
directly is repetitive and error-prone; this script computes exact pin
coordinates (component position + library pin offset, rotated) and emits
both the symbol placement and a matching label at each net connection, so
connectivity is correct by construction. Run, then validate with
`kicad-cli sch erc main.kicad_sch`.
"""
import math
import os
import uuid


def u():
    return str(uuid.uuid4())


def rot(x, y, deg):
    r = math.radians(deg)
    return (x * math.cos(r) - y * math.sin(r), x * math.sin(r) + y * math.cos(r))


# Fixed so re-running the generator doesn't change sheet identity/paths.
ROOT_UUID = "10000000-0000-0000-0000-000000000001"
POWER_UUID = "10000000-0000-0000-0000-000000000002"
MOTORA_UUID = "10000000-0000-0000-0000-000000000003"
MOTORB_UUID = "10000000-0000-0000-0000-000000000004"

# ---------------------------------------------------------------------------
# Pin geometry tables: lib_id -> {pin_number: (dx, dy, pin_rotation_deg)}
# All read directly from the real symbol library files (system KiCad
# libraries + AdapterSymbols.kicad_sym, itself copied verbatim from
# aartech-dev/RemoraNSR3.0) - see the commit message / DESIGN doc for how
# each was extracted and double-checked.
# ---------------------------------------------------------------------------
PINS = {
    "Device:R": {"1": (0, 3.81, 270), "2": (0, -3.81, 90)},
    "Device:C": {"1": (0, 3.81, 270), "2": (0, -3.81, 90)},
    "Device:L": {"1": (0, 3.81, 270), "2": (0, -3.81, 90)},
    "Device:D": {"1": (-3.81, 0, 0), "2": (3.81, 0, 180)},  # 1=K(cathode) 2=A(anode)
    "power:GND": {"1": (0, 0, 270)},
    "power:VCC": {"1": (0, 0, 90)},
    "power:PWR_FLAG": {"1": (0, 0, 90)},
    "Connector_Generic:Conn_01x01": {"1": (-5.08, 0, 0)},
    "AdapterSymbols:D_Zener_Small": {"1": (-2.54, 0, 0), "2": (2.54, 0, 180)},  # 1=K 2=A
    "AdapterSymbols:AP3012": {
        "1": (10.16, 2.54, 180), "2": (0, -7.62, 90), "3": (10.16, -2.54, 180),
        "4": (-10.16, -2.54, 0), "5": (-10.16, 2.54, 0),
    },  # 1=SW 2=GND 3=FB 4=~SHDN 5=IN
    "AdapterSymbols:AP2204K-3.3": {
        "1": (-7.62, 2.54, 0), "2": (0, -7.62, 90), "3": (-7.62, 0, 0),
        "4": (5.08, 0, 180), "5": (7.62, 2.54, 180),
    },  # 1=VIN 2=GND 3=EN 4=NC 5=VOUT
    "AdapterSymbols:CSD16327Q3": {
        "1": (2.54, -5.08, 90), "2": (2.54, -5.08, 90), "3": (2.54, -5.08, 90),
        "4": (-5.08, 0, 0), "5": (2.54, 5.08, 270),
    },  # 1,2,3=S 4=G 5=D
    "AdapterSymbols:DRV8300D": {
        "1": (-10.16, 12.7, 0), "2": (-10.16, 3.81, 0), "3": (-10.16, -5.08, 0),
        "4": (0, 22.86, 270), "5": (-10.16, -10.16, 0), "6": (0, -25.4, 90),
        "7": (-10.16, -12.7, 0), "8": (-10.16, -15.24, 0), "9": (10.16, -16.51, 180),
        "10": (10.16, -3.81, 180), "11": (10.16, 8.89, 180), "12": (10.16, -13.97, 180),
        "13": (10.16, -11.43, 180), "14": (10.16, -8.89, 180), "15": (10.16, -1.27, 180),
        "16": (10.16, 1.27, 180), "17": (10.16, 3.81, 180), "18": (10.16, 11.43, 180),
        "19": (10.16, 13.97, 180), "20": (10.16, 16.51, 180), "21": (-10.16, -17.78, 0),
        "22": (-10.16, 15.24, 0), "23": (-10.16, 6.35, 0), "24": (-10.16, -2.54, 0),
        "25": (-5.08, -25.4, 90),
    },  # LIN1,LIN2,LIN3,GVDD,MODE,GND,NC,NC,LO3,LO2,LO1,VS3,HO3,VB3,VS2,HO2,VB2,VS1,HO1,VB1,DT,HIN1,HIN2,HIN3,PAD
    # INA180A2 and MCP6002-xSN are library symbols with `(extends "Base")`
    # links (to INA180A1 / LM2904 respectively) rather than owning pin
    # geometry themselves. Confirmed empirically (minimal isolated test,
    # not just theory): kicad-cli's ERC engine cannot resolve pin geometry
    # for a *derived* extends-symbol from an embedded lib_symbols cache -
    # placing the base symbol directly works cleanly (proven: a bare-base
    # placement's labels connect with zero dangling/not-connected errors),
    # placing the derived symbol's own lib_id leaves every one of its pins
    # "dangling" regardless of how correct the computed coordinates are.
    # Since extends-derived symbols only override cosmetic properties
    # (footprint filters, description) and share IDENTICAL pin geometry
    # with their base (verified directly against both library files), the
    # fix is to place the BASE's lib_id and set Value to the real part
    # name instead - see the place() calls below (value="INA180A2" /
    # value="MCP6002-xSN" on a lib_id of the *_1 base symbol).
    "Amplifier_Current:INA180A1": {
        "1": (7.62, 0, 180), "2": (-2.54, -7.62, 90), "3": (-7.62, 2.54, 0),
        "4": (-7.62, -2.54, 0), "5": (-2.54, 7.62, 270),
    },  # 1=OUT 2=GND 3=IN+ 4=IN- 5=V+
    "Amplifier_Operational:LM2904": {
        "1": (7.62, 0, 180), "2": (-7.62, -2.54, 0), "3": (-7.62, 2.54, 0),
        "4": (-2.54, -7.62, 90), "5": (-7.62, 2.54, 0), "6": (-7.62, -2.54, 0),
        "7": (7.62, 0, 180), "8": (-2.54, 7.62, 270),
    },  # unit A: OUT1,IN1-,IN1+ / unit B: IN2+,IN2-,OUT2 / shared: V-,V+
}

# Which symbol unit each pin belongs to (only matters for multi-unit parts;
# everything else is unit 1). MCP6002: pins 1-3 = unit 1, 5-7 = unit 2,
# 4,8 = unit 3 (shared power pins) - see Amplifier_Operational.kicad_sym's
# LM2904 base symbol, which MCP6002-xSN extends.
UNIT_OF_PIN = {
    "Amplifier_Operational:LM2904": {"1": 1, "2": 1, "3": 1, "4": 3, "5": 2, "6": 2, "7": 2, "8": 3},
}

# ---------------------------------------------------------------------------
# Multi-sheet plumbing: every place()/label()/wire() call operates on
# whichever Sheet is CURRENT at the time, so the same building code can
# target different output files just by swapping CURRENT before running it.
# Reference designators are a single global counter (not per-sheet) so they
# stay unique project-wide, matching standard KiCad hierarchical practice.
# ---------------------------------------------------------------------------
ref_counters = {}
PAGE_COUNTER = [2]  # page "1" is main; children get 2,3,4...


class Sheet:
    def __init__(self, tag, path_prefix):
        self.tag = tag
        self.path_prefix = path_prefix
        self.symbols = []
        self.wires = []
        self.labels = []
        self.no_connects = []
        self.hier_labels = []
        self.sheet_blocks = []
        self.used_lib_ids = set()


CURRENT = None


def next_ref(prefix):
    ref_counters[prefix] = ref_counters.get(prefix, 0) + 1
    return f"{prefix}{ref_counters[prefix]}"


STUB_LEN = 6.0


def wire(x1, y1, x2, y2):
    CURRENT.wires.append(
        "\n".join([
            "\t(wire",
            f"\t\t(pts (xy {x1:.2f} {y1:.2f}) (xy {x2:.2f} {y2:.2f}))",
            "\t\t(stroke (width 0) (type default))",
            f'\t\t(uuid "{u()}")',
            "\t)",
        ])
    )


def wire_pts(p1, p2):
    wire(p1[0], p1[1], p2[0], p2[1])


def place(lib_id, x, y, rotation=0, ref=None, ref_prefix="U", value=None, footprint_override=None, unit=1, no_connect=(), stub=True):
    """Places one symbol instance in CURRENT sheet; returns {pin_number: (abs_x, abs_y)}."""
    if ref is None:
        ref = next_ref(ref_prefix)
    CURRENT.used_lib_ids.add(lib_id)
    sym_uuid = u()
    props = []
    props.append(f'\t\t(property "Reference" "{ref}" (at {x+6:.2f} {y-3:.2f} 0) (effects (font (size 1.27 1.27))))')
    if value:
        props.append(f'\t\t(property "Value" "{value}" (at {x+6:.2f} {y+3:.2f} 0) (effects (font (size 1.27 1.27))))')
    if footprint_override is not None:
        props.append(f'\t\t(property "Footprint" "{footprint_override}" (at {x:.2f} {y:.2f} 0) (effects (font (size 1.27 1.27)) (hide yes)))')
    pins = PINS[lib_id]
    unit_map = UNIT_OF_PIN.get(lib_id, {})
    pin_lines = []
    abs_coords = {}
    for num in pins:
        # Only declare pin-uuid stubs for pins that actually belong to this
        # unit's own sub-symbol drawing - a multi-unit part (e.g. MCP6002)
        # placed as unit=2 does not graphically have units 1/3's pins, and
        # declaring uuids for them anyway is invalid/misleading to the ERC
        # engine (confirmed as a real bug: caused dangling-label false
        # positives on otherwise-correctly-computed net coordinates).
        if unit_map.get(num, 1) != unit:
            continue
        pin_lines.append(f'\t\t(pin "{num}" (uuid "{u()}"))')
    text = [
        "\t(symbol",
        f'\t\t(lib_id "{lib_id}")',
        f"\t\t(at {x:.2f} {y:.2f} {rotation})",
        f"\t\t(unit {unit})",
        "\t\t(exclude_from_sim no)",
        "\t\t(in_bom yes)",
        "\t\t(on_board yes)",
        "\t\t(dnp no)",
        f'\t\t(uuid "{sym_uuid}")',
    ]
    text += props
    text += pin_lines
    text.append(f'\t\t(instances (project "adapter" (path "{CURRENT.path_prefix}" (reference "{ref}") (unit {unit}))))')
    text.append("\t)")
    CURRENT.symbols.append("\n".join(text))

    for num, (dx, dy, _prot) in pins.items():
        # Skip pins belonging to a *different* unit than the one actually
        # being placed here (mirrors the pin_lines filter above) - without
        # this, a multi-unit part placed as e.g. unit=2 would still compute
        # a stub wire/label point for unit=1/3's pins using unit=2's own
        # (x,y) center, producing a wire connected to nothing at either end
        # (confirmed: this is exactly what caused MCP6002's shared V+/V-
        # pins to get spurious extra stubs when placing its unit=2 half).
        if unit_map.get(num, 1) != unit:
            continue
        rdx, rdy = rot(dx, dy, rotation)
        # KiCad symbol libraries author pin (dx,dy) in a Y-up local frame,
        # but the schematic sheet is Y-down - the rotated offset gets
        # mirrored in Y when placed onto the sheet. Confirmed empirically:
        # kicad-cli ERC reports actual pin coordinates that only match
        # (x+rdx, y-rdy), not (x+rdx, y+rdy), for every placement where
        # rdy != 0 (rotation=0 placements with nonzero pin dy exposed this;
        # rotation=90 passives masked it since their rdy is always 0).
        ox, oy = rdx, -rdy
        px, py = x + ox, y + oy
        if num in no_connect:
            # Genuinely-unused pin (e.g. an IC's NC pin) - a proper
            # no_connect marker at the raw pin tip, not a stub wire to
            # nowhere (which ERC correctly flags as wire_dangling).
            CURRENT.no_connects.append(f'\t(no_connect (at {px:.2f} {py:.2f}) (uuid "{u()}"))')
            abs_coords[num] = (px, py)
            continue
        if not stub:
            abs_coords[num] = (px, py)
            continue
        mag = math.hypot(ox, oy)
        if mag > 0.01:
            scale = (mag + STUB_LEN) / mag
            sx, sy = x + ox * scale, y + oy * scale
            wire(px, py, sx, sy)
            abs_coords[num] = (sx, sy)
        else:
            abs_coords[num] = (px, py)
    return ref, abs_coords


def label(net, x, y):
    """Sheet-LOCAL label (not global) - only connects within CURRENT sheet.
    Cross-sheet connectivity is done explicitly via hier_label()/sheet pins,
    not by same-named labels leaking across files - the whole point of
    partitioning is that each sheet's internal nets are actually private."""
    CURRENT.labels.append(
        "\n".join([
            f'\t(label "{net}"',
            f"\t\t(at {x:.2f} {y:.2f} 0)",
            "\t\t(effects (font (size 1.27 1.27)))",
            f'\t\t(uuid "{u()}")',
            "\t)",
        ])
    )


def hier_label(net, x, y, shape):
    CURRENT.hier_labels.append(
        "\n".join([
            f'\t(hierarchical_label "{net}"',
            f"\t\t(shape {shape})",
            f"\t\t(at {x:.2f} {y:.2f} 0)",
            "\t\t(effects (font (size 1.27 1.27)))",
            f'\t\t(uuid "{u()}")',
            "\t)",
        ])
    )


def gnd(x, y):
    ref, coords = place("power:GND", x, y, ref_prefix="#PWR")
    return coords["1"]


def pwr_flag(x, y):
    ref, coords = place("power:PWR_FLAG", x, y, ref_prefix="#FLG")
    return coords["1"]


def net(name, *points):
    """Connects every (x,y) point to the same sheet-local net via matching labels."""
    for (x, y) in points:
        label(name, *point_xy(x, y))


def point_xy(x, y):
    return (x, y)


def place_sheet(name, filename, x, y, w, h, pins, sheet_uuid):
    """Places a sheet-symbol block (with named pins) in CURRENT (root) sheet.
    pins: list of (net_name, kicad_shape, abs_y) - all on the sheet's LEFT
    edge (x). Returns {net_name: (abs_x, abs_y)}."""
    lines = [
        "\t(sheet",
        f"\t\t(at {x:.2f} {y:.2f})",
        f"\t\t(size {w:.2f} {h:.2f})",
        "\t\t(exclude_from_sim no)",
        "\t\t(in_bom yes)",
        "\t\t(on_board yes)",
        "\t\t(dnp no)",
        "\t\t(fields_autoplaced yes)",
        "\t\t(stroke (width 0.1524) (type solid))",
        "\t\t(fill (color 0 0 0 0))",
        f'\t\t(uuid "{sheet_uuid}")',
        f'\t\t(property "Sheetname" "{name}" (at {x:.2f} {y-3:.2f} 0) (effects (font (size 1.27 1.27)) (justify left bottom)))',
        f'\t\t(property "Sheetfile" "{filename}" (at {x:.2f} {y+h+3:.2f} 0) (effects (font (size 1.27 1.27)) (justify left top)))',
    ]
    abs_coords = {}
    for net_name, shape, py in pins:
        lines.append(f'\t\t(pin "{net_name}" {shape} (at {x:.2f} {py:.2f} 180) (effects (font (size 1.27 1.27))))')
        abs_coords[net_name] = (x, py)
    lines.append(f'\t\t(instances (project "adapter" (path "/{ROOT_UUID}" (page "{PAGE_COUNTER[0]}"))))')
    PAGE_COUNTER[0] += 1
    lines.append("\t)")
    CURRENT.sheet_blocks.append("\n".join(lines))
    return abs_coords


# ---------------------------------------------------------------------------
# POWER sheet: track power in, reverse-polarity protection, boost, LDO,
# shared bidirectional track-current sense (DESIGN.md section 6.5/6.4/7.5/7.7).
# Boundary nets (exposed via hier_label + a matching sheet pin on main):
# PA2, PB12 (sense outputs to the Nucleo), VDD, GND (shared with Nucleo and
# both motor sheets), VCC, VBUS_LOAD (feed both motor sheets only).
# ---------------------------------------------------------------------------

def build_power():
    global CURRENT
    CURRENT = Sheet("power", f"/{ROOT_UUID}/{POWER_UUID}")

    # --- Track power input + reverse-polarity protection ---
    # Single N-FET "ideal diode", low-side placement: Q1 drain = TRACK_RTN (the
    # externally-wired "return" terminal - may actually be positive if the
    # installer swapped the two wires), Q1 source = GND (system ground everything
    # else references). Body diode conducts (helps current flow) only when
    # correctly wired; gate biased toward TRACK_PWR through R1, clamped by zener
    # D1, so Vgs collapses toward 0 (FET stays off) if the wires are swapped.
    # Deliberately simpler than RemoraNSR3.0's own undocumented multi-transistor
    # reverse-protection network (DESIGN.md 6.5 already flags that circuit as
    # never fully reverse-engineered) - this is a standard, well-understood
    # textbook topology instead, chosen for correctness confidence over blind
    # replication.
    _, j1 = place("Connector_Generic:Conn_01x01", 40, 160, ref_prefix="J", value="Track +")
    _, j2 = place("Connector_Generic:Conn_01x01", 40, 120, ref_prefix="J", value="Track -")
    net("TRACK_PWR", j1["1"])
    net("TRACK_RTN_RAW", j2["1"])
    pwr_flag(40, 175)
    net("TRACK_PWR", (40, 175))

    _, q1 = place("AdapterSymbols:CSD16327Q3", 85, 120, ref_prefix="Q", value="CSD16327Q3")
    net("TRACK_RTN_RAW", q1["5"])  # D
    net("GND", q1["1"])            # S (x3 pins, same coordinate)
    net("RP_GATE", q1["4"])        # G

    _, r1 = place("Device:R", 85, 155, rotation=90, ref_prefix="R", value="10k")
    net("TRACK_PWR", r1["1"])
    net("RP_GATE", r1["2"])

    _, d1 = place("AdapterSymbols:D_Zener_Small", 115, 135, ref_prefix="D", value="15V")
    net("RP_GATE", d1["1"])  # K
    net("GND", d1["2"])      # A

    net("TRACK_PWR", (40, 160))  # VBUS = TRACK_PWR directly (unprotected side - see module doc)
    label("VBUS", 40, 160)

    # --- Boost regulator: VBUS -> VCC (boosted gate-drive supply), ~12V target ---
    # AP3012 adjustable boost. R2/R3 set the FB divider - placeholder values
    # targeting ~12V assuming a ~1.25V FB reference; verify against the AP3012
    # datasheet's actual reference voltage before board bring-up. L1/D2/C1/C2
    # values are likewise placeholders pending real characterization.
    _, u1 = place("AdapterSymbols:AP3012", 160, 150, ref_prefix="U", value="AP3012")
    net("VBUS", u1["5"])   # IN
    net("VBUS", u1["4"])   # ~SHDN tied to IN - always enabled (matches RemoraNSR3.0's own wiring)
    net("GND", u1["2"])    # GND
    gnd_pt = gnd(160, 175)  # real power:GND symbol - for schematic readability
    net("GND", gnd_pt)
    _gnd_flag = pwr_flag(160, 190)  # power:GND's own pin is power_in, not power_out - still needs a PWR_FLAG
    net("GND", (160, 190))

    _, l1 = place("Device:L", 130, 165, rotation=90, ref_prefix="L", value="10uH")
    net("VBUS", l1["1"])
    net("AP3012_SW", l1["2"])
    net("AP3012_SW", u1["1"])  # SW

    _, d2 = place("Device:D", 185, 165, ref_prefix="D", value="Schottky (e.g. SS14)")
    # Device:D pin "1" = K (cathode), pin "2" = A (anode) - a boost rectifier
    # conducts switch-node -> VCC, i.e. anode at the switch node, cathode at VCC.
    net("AP3012_SW", d2["2"])  # A
    net("VCC", d2["1"])        # K

    _, c1 = place("Device:C", 160, 115, rotation=90, ref_prefix="C", value="10uF")
    net("VBUS", c1["1"])
    net("GND", c1["2"])

    _, c2 = place("Device:C", 210, 150, rotation=90, ref_prefix="C", value="22uF")
    net("VCC", c2["1"])
    net("GND", c2["2"])

    _, r2 = place("Device:R", 230, 165, rotation=90, ref_prefix="R", value="86k")
    net("VCC", r2["1"])
    net("AP3012_FB", r2["2"])
    net("AP3012_FB", u1["3"])  # FB

    _, r3 = place("Device:R", 230, 130, rotation=90, ref_prefix="R", value="10k")
    net("AP3012_FB", r3["1"])
    net("GND", r3["2"])

    _vcc_flag = pwr_flag(160, 100)  # boost output isn't power_out-typed per ERC (SW/diode pins are passive)
    net("VCC", (160, 100))
    hier_label("VCC", 160, 95, "output")
    net("VCC", (160, 95))

    # --- Logic LDO: VCC -> VDD (3.3V, matches the Nucleo's own logic rail) ---
    _, u2 = place("AdapterSymbols:AP2204K-3.3", 260, 150, ref_prefix="U", value="AP2204K-3.3", no_connect={"4"})
    net("VCC", u2["1"])   # VIN
    net("VCC", u2["3"])   # EN tied on
    net("GND", u2["2"])   # GND
    net("VDD", u2["5"])   # VOUT

    _, c3 = place("Device:C", 240, 115, rotation=90, ref_prefix="C", value="1uF")
    net("VCC", c3["1"])
    net("GND", c3["2"])

    _, c4 = place("Device:C", 280, 115, rotation=90, ref_prefix="C", value="1uF")
    net("VDD", c4["1"])
    net("GND", c4["2"])

    hier_label("VDD", 260, 95, "passive")
    net("VDD", (260, 95))

    # --- Bidirectional shared track-current sense (DESIGN.md 6.5/7.5) ---
    # Shunt in the VBUS path; MCP6002 unit A buffers a VDD/2 reference, unit B
    # is a 4-resistor difference amp centered on it - chosen over betting on an
    # unfamiliar bidirectional current-sense IC (see commit notes). Output net
    # PA2 matches the real GPIO this feeds (DESIGN.md 6.1/6.5).
    _, r_shunt = place("Device:R", 85, 90, rotation=90, ref_prefix="R", value="5mOhm 1W")
    net("VBUS", r_shunt["1"])
    # VBUS_LOAD (not VBUS) is what both bridges' high-side drains actually
    # connect to (see motor sheets) - the shunt must sit in series between
    # the raw input and the bridges for the sense amp to see real load
    # current, per DESIGN.md 6.5 ("shunt on the main input path ... before
    # it splits to the two bridges").
    net("VBUS_LOAD", r_shunt["2"])
    hier_label("VBUS_LOAD", 85, 85, "output")
    net("VBUS_LOAD", (85, 85))

    _, r4 = place("Device:R", 330, 165, rotation=90, ref_prefix="R", value="10k")
    net("VDD", r4["1"])
    net("VDD_HALF_RAW", r4["2"])
    _, r5 = place("Device:R", 330, 130, rotation=90, ref_prefix="R", value="10k")
    net("VDD_HALF_RAW", r5["1"])
    net("GND", r5["2"])

    u3_ref, u3 = place("Amplifier_Operational:LM2904", 365, 150, ref_prefix="U", value="MCP6002-xSN", unit=1)
    net("VDD_HALF_RAW", u3["3"])   # unit A: IN1+ = raw divider midpoint
    net("VDD_HALF", u3["2"])       # unit A: IN1- fed back from OUT1 (voltage follower)
    net("VDD_HALF", u3["1"])       # unit A: OUT1 = buffered VDD/2 reference

    _, u3pwr = place("Amplifier_Operational:LM2904", 365, 150, ref=u3_ref, unit=3)  # shared power pins
    net("GND", u3pwr["4"])  # V-
    net("VDD", u3pwr["8"])  # V+

    _, u3b = place("Amplifier_Operational:LM2904", 420, 150, ref=u3_ref, value=None, unit=2)
    _, r6 = place("Device:R", 400, 165, rotation=90, ref_prefix="R", value="1k")
    net("VBUS", r6["1"])
    net("DIFF_IN_PLUS", r6["2"])
    net("DIFF_IN_PLUS", u3b["5"])  # unit B: IN2+

    _, r7 = place("Device:R", 400, 130, rotation=90, ref_prefix="R", value="1k")
    net("VBUS_LOAD", r7["1"])
    net("DIFF_IN_MINUS", r7["2"])
    net("DIFF_IN_MINUS", u3b["6"])  # unit B: IN2-

    _, r8 = place("Device:R", 445, 165, rotation=90, ref_prefix="R", value="20k")
    net("DIFF_IN_PLUS", r8["1"])
    net("VDD_HALF", r8["2"])

    _, r9 = place("Device:R", 445, 130, rotation=90, ref_prefix="R", value="20k")
    net("DIFF_IN_MINUS", r9["1"])
    net("PA2", r9["2"])
    net("PA2", u3b["7"])  # unit B: OUT2 = ADC1 channel 3, shared track-current sense
    hier_label("PA2", 465, 130, "output")
    net("PA2", (465, 130))

    # --- Shared bus-voltage sense (DESIGN.md 6.1: PB12, "one reading is
    # enough" - not per-motor, just a plain divider off VBUS) ---
    _, rv1 = place("Device:R", 85, 60, rotation=90, ref_prefix="R", value="47k")
    net("VBUS", rv1["1"])
    net("PB12", rv1["2"])
    _, rv2 = place("Device:R", 85, 40, rotation=90, ref_prefix="R", value="10k")
    net("PB12", rv2["1"])
    net("GND", rv2["2"])
    hier_label("PB12", 100, 60, "output")
    net("PB12", (100, 60))

    hier_label("GND", 40, 105, "passive")
    net("GND", (40, 105))

    return CURRENT


# ---------------------------------------------------------------------------
# MOTOR sheet: DRV8300D driver + 6x CSD16327Q3 bridge + bootstrap caps +
# per-phase BEMF dividers + virtual-neutral summing network + per-motor
# unidirectional current-sense amp (DESIGN.md 6.5's "what scales with motor
# count" block, x2 for front/rear). GPIO net names match DESIGN.md 6.4's pin
# table (superseded for BEMF_C/neutral/current by the G474 retarget) and are
# exposed as hierarchical labels so the Nucleo block on main.kicad_sch can
# be wired to them directly by name.
# ---------------------------------------------------------------------------

def build_motor(suffix, sheet_uuid, hin, lin, bemf, neutral_net, curr_net):
    global CURRENT
    CURRENT = Sheet(f"motor_{suffix.lower()}", f"/{ROOT_UUID}/{sheet_uuid}")
    ox, oy = 320, 160

    _, drv = place("AdapterSymbols:DRV8300D", ox, oy, ref_prefix="U", value=f"DRV8300D ({suffix})", no_connect={"7", "8"})
    net("VCC", drv["4"])   # GVDD - boosted gate supply, shared
    net("GND", drv["6"])   # GND
    net("GND", drv["25"])  # PAD (thermal, tie to GND)
    net(hin[0], drv["22"])  # HIN1
    net(hin[1], drv["23"])  # HIN2
    net(hin[2], drv["24"])  # HIN3
    net(lin[0], drv["1"])   # LIN1
    net(lin[1], drv["2"])   # LIN2
    net(lin[2], drv["3"])   # LIN3

    # Boundary pins - one hierarchical label per GPIO/power net crossing to
    # the Nucleo block or the Power sheet (wired up on main.kicad_sch).
    # Placed well below the hin/lin loops' range (oy+40 down to oy-35) so
    # these don't land on the exact same coordinate as one of those labels
    # (confirmed as a real bug: two differently-named hierarchical labels
    # sharing one point don't merge sensibly - kicad can't resolve a net
    # name for the collision and both ends up dumped in an anonymous
    # unconnected net, which then trips "same-type pins connected").
    hier_label("VCC", ox - 90, oy - 70, "input")
    net("VCC", (ox - 90, oy - 70))
    hier_label("GND", ox - 90, oy - 85, "passive")
    net("GND", (ox - 90, oy - 85))
    hier_label("VBUS_LOAD", ox - 90, oy - 100, "input")
    net("VBUS_LOAD", (ox - 90, oy - 100))
    for i, net_name in enumerate(hin):
        hier_label(net_name, ox - 90, oy + 40 - 15 * i, "input")
        net(net_name, (ox - 90, oy + 40 - 15 * i))
    for i, net_name in enumerate(lin):
        hier_label(net_name, ox - 90, oy - 5 - 15 * i, "input")
        net(net_name, (ox - 90, oy - 5 - 15 * i))

    # MODE/DT external bias resistors - pulled low (10k to GND) as a
    # placeholder; DESIGN.md flags this needs datasheet verification before
    # board bring-up (both pins likely want a specific resistor-to-GND value
    # to select PWM mode / set dead time, not just "some pulldown").
    _, r_mode = place("Device:R", ox - 55, oy + 20, rotation=90, ref_prefix="R", value="10k (verify vs datasheet)")
    net(f"DRVMODE_{suffix}", r_mode["1"])
    net(f"DRVMODE_{suffix}", drv["5"])   # MODE
    net("GND", r_mode["2"])
    _, r_dt = place("Device:R", ox - 55, oy - 35, rotation=90, ref_prefix="R", value="10k (verify vs datasheet)")
    net(f"DRVDT_{suffix}", r_dt["1"])
    net(f"DRVDT_{suffix}", drv["21"])    # DT
    net("GND", r_dt["2"])

    # Per-leg: bootstrap cap (VBx<->VSx), high-side FET (drain=VBUS_LOAD,
    # source=phase node, gate via resistor from HOx), low-side FET
    # (drain=phase node, source=common motor-return node, gate via
    # resistor from LOx), phase connector out to the motor winding, and
    # the BEMF divider (47k/10k, midpoint to that phase's ADC pin).
    ho_pins = ("19", "16", "13")   # HO1,HO2,HO3
    lo_pins = ("11", "10", "9")    # LO1,LO2,LO3
    vb_pins = ("20", "17", "14")   # VB1,VB2,VB3
    vs_pins = ("18", "15", "12")   # VS1,VS2,VS3
    leg_dx = 95
    bemf_points = []
    neutral_points = []
    curr_point = None
    for i, leg in enumerate(("U", "V", "W")):
        lx = ox + 90
        ly = oy + 95 - i * leg_dx
        phase_net = f"PH_{leg}_{suffix}"

        _, cb = place("Device:C", lx - 25, ly + 18, rotation=90, ref_prefix="C", value="100nF")
        net(f"VB{i+1}_{suffix}", cb["1"])
        net(f"VB{i+1}_{suffix}", drv[vb_pins[i]])
        net(phase_net, cb["2"])
        net(phase_net, drv[vs_pins[i]])

        _, qh = place("AdapterSymbols:CSD16327Q3", lx + 15, ly, ref_prefix="Q", value="CSD16327Q3")
        net("VBUS_LOAD", qh["5"])  # D -> input rail, downstream of the shared shunt (high-side)
        net(phase_net, qh["1"])    # S -> phase node
        _, rgh = place("Device:R", lx - 32, ly, rotation=90, ref_prefix="R", value="10R")
        net(f"HO{i+1}_{suffix}", rgh["1"])
        net(f"HO{i+1}_{suffix}", drv[ho_pins[i]])
        net(f"HO{i+1}G_{suffix}", rgh["2"])
        net(f"HO{i+1}G_{suffix}", qh["4"])

        _, ql = place("AdapterSymbols:CSD16327Q3", lx + 15, ly - 25, ref_prefix="Q", value="CSD16327Q3")
        net(phase_net, ql["5"])          # D -> phase node
        net(f"MOTRTN_{suffix}", ql["1"])  # S -> common motor-return node
        _, rgl = place("Device:R", lx - 32, ly - 25, rotation=90, ref_prefix="R", value="10R")
        net(f"LO{i+1}_{suffix}", rgl["1"])
        net(f"LO{i+1}_{suffix}", drv[lo_pins[i]])
        net(f"LO{i+1}G_{suffix}", rgl["2"])
        net(f"LO{i+1}G_{suffix}", ql["4"])

        _, jph = place("Connector_Generic:Conn_01x01", lx + 55, ly - 12, ref_prefix="J", value=f"Motor {suffix} phase {leg}")
        net(phase_net, jph["1"])

        _, rb1 = place("Device:R", lx + 80, ly, rotation=90, ref_prefix="R", value="47k")
        net(phase_net, rb1["1"])
        net(bemf[i], rb1["2"])
        _, rb2 = place("Device:R", lx + 80, ly - 20, rotation=90, ref_prefix="R", value="10k")
        net(bemf[i], rb2["1"])
        net("GND", rb2["2"])
        bemf_points.append((bemf[i], (lx + 80, ly)))

        # Virtual-neutral summing tap (3x47k converging + 10k to GND,
        # placed after the block below). Offset down a row and given extra
        # x-clearance from the BEMF divider so the two facing labels don't
        # crowd each other.
        _, rn = place("Device:R", lx + 140, ly - 10, rotation=90, ref_prefix="R", value="47k")
        net(phase_net, rn["1"])
        net(neutral_net, rn["2"])

    _, rn_gnd = place("Device:R", ox + 215, oy - 55, rotation=90, ref_prefix="R", value="10k")
    net(neutral_net, rn_gnd["1"])
    net("GND", rn_gnd["2"])
    hier_label(neutral_net, ox + 260, oy - 55, "output")
    net(neutral_net, (ox + 260, oy - 55))

    # Per-motor unidirectional current sense: shunt in the low-side
    # return path (MOTRTN -> shunt -> GND), INA180-series amp reading
    # across it (base part INA180A1 placed directly, Value overridden to
    # the real gain-50 variant actually used - see the PINS-table comment
    # on why the derived symbol's own lib_id can't be placed directly).
    _, r_shunt = place("Device:R", ox + 90, oy - 90, rotation=90, ref_prefix="R", value="5mOhm 1W")
    net(f"MOTRTN_{suffix}", r_shunt["1"])
    net("GND", r_shunt["2"])
    _, ina = place("Amplifier_Current:INA180A1", ox + 130, oy - 90, ref_prefix="U", value="INA180A2")
    net(f"MOTRTN_{suffix}", ina["3"])  # IN+ (FET/shunt side)
    net("GND", ina["4"])               # IN- (true GND side of shunt)
    net("VDD", ina["5"])               # V+
    net("GND", ina["2"])               # amp's own GND pin
    net(curr_net, ina["1"])            # OUT -> ADC
    hier_label("VDD", ox + 170, oy - 90, "input")
    net("VDD", (ox + 170, oy - 90))
    hier_label(curr_net, ox + 170, oy - 70, "output")
    net(curr_net, (ox + 170, oy - 70))

    # BEMF sense outputs - one hierarchical label per phase, placed near
    # each divider's midpoint tap rather than bunched together.
    for net_name, (px, py) in bemf_points:
        hier_label(net_name, px + 15, py, "output")
        net(net_name, (px + 15, py))

    return CURRENT


# ---------------------------------------------------------------------------
# MAIN (root) sheet: places the real Nucleo-G474RE symbol (a genuine
# STMicroelectronics/SnapEDA part with the actual CN7/CN10 morpho-header
# pinout, provided directly rather than hand-authored), plus the three
# sub-circuit sheets, wired together with real point-to-point wires between
# matching named pins.
# ---------------------------------------------------------------------------

NUCLEO_SYM_FILE = "AdapterSymbols.kicad_sym/NUCLEO-G474RE.kicad_sym"
NUCLEO_LIB_ID = "AdapterSymbols:NUCLEO-G474RE"

# Real pin (dx, dy, rotation) for every CN7/CN10 morpho-header pin this
# design actually uses, read directly from NUCLEO-G474RE.kicad_sym (not
# guessed) - unit 1 = "_1_0" (CN7), unit 2 = "_2_0" (CN10). The symbol's
# own pin *names* don't always match our net names one-to-one (the board's
# "+3V3" pin is what we call VDD; the shared current-sense/BEMF_C pins are
# alternate-function pins silkscreened "PA2/PC4" and "PA3/PC5"), so wiring
# below goes by pin *number*, via NUCLEO_NET_TO_PIN.
# Every real pin on both units (not just the ones this design uses) - the
# no_connect markers below need every unused pin's real geometry too, or
# ERC reports them as dangling/unconnected instead of a proper no-connect.
NUCLEO_PIN_GEOM = {
    # unit 1 (CN7)
    "CN7_1": (-22.86, 22.86, 0), "CN7_2": (22.86, 22.86, 180), "CN7_3": (-22.86, 20.32, 0),
    "CN7_4": (22.86, 20.32, 180), "CN7_7": (-22.86, 15.24, 0), "CN7_12": (22.86, 10.16, 180),
    "CN7_14": (22.86, 7.62, 180), "CN7_15": (-22.86, 5.08, 0), "CN7_17": (-22.86, 2.54, 0),
    "CN7_21": (-22.86, -2.54, 0), "CN7_23": (-22.86, -5.08, 0), "CN7_25": (-22.86, -7.62, 0),
    "CN7_27": (-22.86, -10.16, 0), "CN7_28": (22.86, -10.16, 180), "CN7_29": (-22.86, -12.7, 0),
    "CN7_30": (22.86, -12.7, 180), "CN7_31": (-22.86, -15.24, 0), "CN7_32": (22.86, -15.24, 180),
    "CN7_34": (22.86, -17.78, 180), "CN7_35": (-22.86, -20.32, 0), "CN7_36": (22.86, -20.32, 180),
    "CN7_37": (-22.86, -22.86, 0), "CN7_38": (22.86, -22.86, 180), "CN7_13": (-22.86, 7.62, 0),
    "CN7_5": (-22.86, 17.78, 0), "CN7_6": (22.86, 17.78, 180), "CN7_19": (-22.86, 0.0, 0),
    "CN7_9": (-22.86, 12.7, 0), "CN7_16": (22.86, 5.08, 180), "CN7_18": (22.86, 2.54, 180),
    "CN7_24": (22.86, -5.08, 180), "CN7_33": (-22.86, -17.78, 0), "CN7_11": (-22.86, 10.16, 0),
    "CN7_8": (22.86, 15.24, 180), "CN7_10": (22.86, 12.7, 180), "CN7_20": (22.86, 0.0, 180),
    "CN7_22": (22.86, -2.54, 180), "CN7_26": (22.86, -7.62, 180),
    # unit 2 (CN10)
    "CN10_1": (-22.86, 22.86, 0), "CN10_2": (22.86, 22.86, 180), "CN10_3": (-22.86, 20.32, 0),
    "CN10_4": (22.86, 20.32, 180), "CN10_5": (-22.86, 17.78, 0), "CN10_6": (22.86, 17.78, 180),
    "CN10_11": (-22.86, 10.16, 0), "CN10_12": (22.86, 10.16, 180), "CN10_13": (-22.86, 7.62, 0),
    "CN10_14": (22.86, 7.62, 180), "CN10_15": (-22.86, 5.08, 0), "CN10_16": (22.86, 5.08, 180),
    "CN10_17": (-22.86, 2.54, 0), "CN10_18": (22.86, 2.54, 180), "CN10_19": (-22.86, 0.0, 0),
    "CN10_21": (-22.86, -2.54, 0), "CN10_22": (22.86, -2.54, 180), "CN10_23": (-22.86, -5.08, 0),
    "CN10_24": (22.86, -5.08, 180), "CN10_25": (-22.86, -7.62, 0), "CN10_26": (22.86, -7.62, 180),
    "CN10_27": (-22.86, -10.16, 0), "CN10_28": (22.86, -10.16, 180), "CN10_29": (-22.86, -12.7, 0),
    "CN10_30": (22.86, -12.7, 180), "CN10_31": (-22.86, -15.24, 0), "CN10_33": (-22.86, -17.78, 0),
    "CN10_34": (22.86, -17.78, 180), "CN10_35": (-22.86, -20.32, 0), "CN10_37": (-22.86, -22.86, 0),
    "CN10_7": (-22.86, 15.24, 0), "CN10_8": (22.86, 15.24, 180), "CN10_32": (22.86, -15.24, 180),
    "CN10_9": (-22.86, 12.7, 0), "CN10_10": (22.86, 12.7, 180), "CN10_20": (22.86, 0.0, 180),
    "CN10_36": (22.86, -20.32, 180), "CN10_38": (22.86, -22.86, 180),
}
NUCLEO_PIN_UNIT = {num: (1 if num.startswith("CN7_") else 2) for num in NUCLEO_PIN_GEOM}
NUCLEO_NET_TO_PIN = {
    "PA15": "CN7_17", "PA0": "CN7_28", "PF0": "CN7_29", "PA1": "CN7_30",
    "PF1": "CN7_31", "PA4": "CN7_32", "PB0": "CN7_34", "VDD": "CN7_16", "GND": "CN7_19",
    "PB8": "CN10_3", "PB9": "CN10_5", "PA5": "CN10_11", "PA6": "CN10_13",
    "PA7": "CN10_15", "PB12": "CN10_16", "PB11": "CN10_18", "PA9": "CN10_21",
    "PA8": "CN10_23", "PB1": "CN10_24", "PB4": "CN10_27", "PB14": "CN10_28",
    "PB5": "CN10_29", "PB3": "CN10_31", "PA10": "CN10_33", "PA2": "CN10_35",
    "PA3": "CN10_37",
}
# Every real pin not wired to one of our nets gets a proper no_connect flag
# instead of showing as a dangling/unconnected error - this Nucleo variant
# exposes far more GPIO than this particular design uses.
_USED_NUCLEO_PINS = set(NUCLEO_NET_TO_PIN.values())
NUCLEO_UNUSED_UNIT1 = {n for n in NUCLEO_PIN_GEOM if n.startswith("CN7_") and n not in _USED_NUCLEO_PINS}
NUCLEO_UNUSED_UNIT2 = {n for n in NUCLEO_PIN_GEOM if n.startswith("CN10_") and n not in _USED_NUCLEO_PINS}
PINS[NUCLEO_LIB_ID] = NUCLEO_PIN_GEOM
UNIT_OF_PIN[NUCLEO_LIB_ID] = NUCLEO_PIN_UNIT


def extract_flexible_symbol_block(source_text, bare_name):
    """Like extract_symbol_block(), but for externally-authored .kicad_sym
    files that don't use KiCad's own tab-indentation convention (this one
    uses 2-space indents) - matches by stripped content instead of an
    exact leading-tab prefix; otherwise identical (track paren depth from
    the opening line to find the true matching close, regardless of
    indentation style - the S-expression parser doesn't care, only the
    exact-prefix line search needed generalizing)."""
    lines = source_text.split("\n")
    start = None
    needle = f'(symbol "{bare_name}"'
    for i, line in enumerate(lines):
        if line.strip().startswith(needle):
            start = i
            break
    if start is None:
        raise ValueError(f"symbol {bare_name!r} not found")
    depth = 0
    end = None
    for i in range(start, len(lines)):
        depth += lines[i].count("(") - lines[i].count(")")
        if depth == 0:
            end = i + 1
            break
    if end is None:
        raise ValueError(f"symbol {bare_name!r}: unbalanced parens")
    return lines[start:end]


def nucleo_real_symbol_text():
    with open(NUCLEO_SYM_FILE) as f:
        text = f.read()
    block = list(extract_flexible_symbol_block(text, "NUCLEO-G474RE"))
    # Only the top-level symbol needs the library-qualified name; nested
    # unit drawings ("NUCLEO-G474RE_1_0"/"_2_0") stay bare.
    block[0] = block[0].replace('"NUCLEO-G474RE"', f'"{NUCLEO_LIB_ID}"', 1)
    return "\n".join(block)


def route_lane(p1, p2, lane_x):
    """3-segment jog: p1 -> (lane_x, p1.y) -> (lane_x, p2.y) -> p2. Used
    instead of a direct wire whenever p1/p2 aren't already aligned, since a
    direct diagonal isn't valid, and a naive orthogonal 2-segment route
    risks passing straight through an unrelated pin sitting on the same
    row/column (confirmed - that's what merged VCC/VDD/GND into one net
    the first time the shared rails were routed this way). Give each net
    with real crossing risk its own dedicated lane_x."""
    wire_pts(p1, (lane_x, p1[1]))
    wire(lane_x, p1[1], lane_x, p2[1])
    wire_pts((lane_x, p2[1]), p2)


def build_main():
    global CURRENT
    CURRENT = Sheet("main", "/")

    power_pins = [
        ("PA2", "output", 55), ("PB12", "output", 70),
        ("VDD", "passive", 85), ("GND", "passive", 100),
        ("VCC", "output", 115), ("VBUS_LOAD", "output", 130),
    ]
    motor_a_pins = [
        ("VCC", "input", 185), ("VDD", "input", 200), ("GND", "passive", 215), ("VBUS_LOAD", "input", 230),
        ("PA8", "input", 260), ("PA9", "input", 275), ("PA10", "input", 290),
        ("PA7", "input", 305), ("PB0", "input", 320), ("PF0", "input", 335),
        ("PA0", "output", 350), ("PA1", "output", 365), ("PA3", "output", 380),
        ("PB1", "output", 395), ("PB11", "output", 410),
    ]
    motor_b_names = ["VCC", "VDD", "GND", "VBUS_LOAD",
                      "PA15", "PB8", "PB9", "PB3", "PB4", "PB5",
                      "PA4", "PA5", "PA6", "PB14", "PF1"]
    motor_b_pins = [
        (new_name, shape, y + 440)
        for (old_name, shape, y), new_name in zip(motor_a_pins, motor_b_names)
    ]

    nucleo_symtext = nucleo_real_symbol_text()
    nuc_ref, nuc1 = place(NUCLEO_LIB_ID, 70, 190, ref_prefix="U", value="Nucleo-G474RE", unit=1, stub=False, no_connect=NUCLEO_UNUSED_UNIT1)
    _, nuc2 = place(NUCLEO_LIB_ID, 70, 300, ref=nuc_ref, unit=2, stub=False, no_connect=NUCLEO_UNUSED_UNIT2)
    nuc_by_pin = {**nuc1, **nuc2}

    def nuc(net_name):
        return nuc_by_pin[NUCLEO_NET_TO_PIN[net_name]]

    power_coords = place_sheet("Power", "power.kicad_sch", 250, 40, 140, 110, power_pins, POWER_UUID)
    motora_coords = place_sheet("Motor A", "motor_a.kicad_sch", 250, 170, 140, 400, motor_a_pins, MOTORA_UUID)
    motorb_coords = place_sheet("Motor B", "motor_b.kicad_sch", 250, 610, 140, 400, motor_b_pins, MOTORB_UUID)

    # Nucleo <-> Power/Motor A/Motor B: the real symbol's pins don't line
    # up with the sheets' pins the way the old hand-authored block's did
    # (its pin order was ours to choose; this one's is the real board's),
    # so every one of these gets its own dedicated routing lane rather
    # than a straight wire.
    lane_x = 110
    for name in ("PA2", "PB12", "VDD", "GND"):
        route_lane(nuc(name), power_coords[name], lane_x)
        lane_x += 4
    for name, _, _ in motor_a_pins[4:]:
        route_lane(nuc(name), motora_coords[name], lane_x)
        lane_x += 4
    for name, _, _ in motor_b_pins[4:]:
        route_lane(nuc(name), motorb_coords[name], lane_x)
        lane_x += 4

    # Power -> Motor A -> Motor B (shared rails). Not a direct vertical
    # wire down the shared x=250 edge: that range also carries the Nucleo
    # GPIO wires and the *other* rails' own sheet pins, and a long wire
    # segment picks up any pin its path happens to pass through even
    # without an explicit vertex there (confirmed the hard way - this is
    # exactly what merged VCC/VDD/GND into one net the first time this was
    # tried). Each rail instead jogs out to its own dedicated lane well
    # past every sheet's right edge (x=390), where nothing else runs.
    for lane_i, rail in enumerate(("VCC", "VDD", "GND", "VBUS_LOAD")):
        bus_lane_x = 420 + lane_i * 10
        route_lane(power_coords[rail], motora_coords[rail], bus_lane_x)
        route_lane(motora_coords[rail], motorb_coords[rail], bus_lane_x)

    return CURRENT, nucleo_symtext


# ---------------------------------------------------------------------------
# Symbol embedding: extract each used symbol's full definition from its
# source library file (system KiCad libraries, or the local
# AdapterSymbols.kicad_sym directory copied from aartech-dev/RemoraNSR3.0)
# and embed it in each output file's own lib_symbols, exactly how a real
# KiCad save does it - kicad-cli's ERC reads pin geometry from this embedded
# cache, not by resolving lib_id against sym-lib-table at check time
# (confirmed empirically - see commit notes).
# ---------------------------------------------------------------------------

SYSTEM_LIB_SOURCES = {
    "Device": "/usr/share/kicad/symbols/Device.kicad_sym",
    "power": "/usr/share/kicad/symbols/power.kicad_sym",
    "Connector_Generic": "/usr/share/kicad/symbols/Connector_Generic.kicad_sym",
    "Amplifier_Current": "/usr/share/kicad/symbols/Amplifier_Current.kicad_sym",
    "Amplifier_Operational": "/usr/share/kicad/symbols/Amplifier_Operational.kicad_sym",
}
LOCAL_LIB_DIR = "AdapterSymbols.kicad_sym"


def extract_symbol_block(source_text, bare_name):
    """Returns the full `(symbol "bare_name" ... )` block's inner lines
    (top-level pin_names/property/sub-unit lines), by matching top-level
    `\t(symbol "` boundaries (one tab of indent) - sub-unit symbols nest
    one level deeper and are included as part of the outer block."""
    lines = source_text.split("\n")
    start = None
    for i, line in enumerate(lines):
        if line == f'\t(symbol "{bare_name}"':
            start = i
            break
    if start is None:
        raise ValueError(f"symbol {bare_name!r} not found")
    # Track paren depth from the opening line to find its true matching
    # close, rather than guessing from indentation - a single-symbol file
    # (RemoraNSR3.0's one-symbol-per-file layout) has no following
    # top-level symbol to bound it, and naively slicing to end-of-file
    # would also capture that file's own trailing `kicad_symbol_lib` close.
    depth = 0
    end = None
    for i in range(start, len(lines)):
        depth += lines[i].count("(") - lines[i].count(")")
        if depth == 0:
            end = i + 1
            break
    if end is None:
        raise ValueError(f"symbol {bare_name!r}: unbalanced parens, no matching close found")
    return lines[start:end]


def resolve_extends(lib_nick, bare_name, full_name, cache):
    """Returns a list of one or two embeddable blocks for `bare_name`. If it
    has an `(extends "Base")` link (e.g. MCP6002-xSN extends LM2904,
    INA180A2 extends INA180A1), KiCad resolves that reference against
    *another symbol in the same lib_symbols block* - confirmed empirically
    (flattening the base's content into one merged symbol and dropping
    `extends`, tried first, reliably fails kicad-cli's loader even once
    every structural mismatch is fixed; embedding both symbols verbatim,
    side by side, with the reference rewritten to match the base's own
    renamed lib_id, loads and ERCs cleanly - simpler *and* correct once
    actually verified rather than assumed). So: rename both, point the
    child's `extends` at the base's new full name, and return both blocks -
    the base is never itself instantiated by `place()`, it only needs to
    exist in lib_symbols for this reference to resolve."""
    source = cache[lib_nick]
    block = list(extract_symbol_block(source, bare_name))
    block[0] = f'\t(symbol "{full_name}"'
    for i, line in enumerate(block):
        s = line.strip()
        if s.startswith('(extends "'):
            base = s.split('"')[1]
            base_full_name = f"{lib_nick}:{base}"
            block[i] = line.replace(f'(extends "{base}")', f'(extends "{base_full_name}")')
            base_block = list(extract_symbol_block(source, base))
            base_block[0] = f'\t(symbol "{base_full_name}"'
            return [base_block, block]
    return [block]


# Symbol name -> actual on-disk filename (KiCad 10's one-symbol-per-file
# storage uses lowercase/hyphenated filenames that don't always match the
# symbol's own internal name case).
LOCAL_LIB_FILES = {
    "AP3012": "ap3012", "AP2204K-3.3": "ap2204k-3.3", "CSD16327Q3": "csd16327q3",
    "D_Zener_Small": "d_zener_small", "DRV8300D": "DRV8300D", "BAT40V": "BAT40V",
}


def embed(lib_nick, bare_name, full_name, cache):
    """Returns a list of embeddable block strings (usually one; two if
    `bare_name` extends a base symbol - see resolve_extends)."""
    if lib_nick == "AdapterSymbols":
        filename = LOCAL_LIB_FILES[bare_name]
        path = os.path.join(LOCAL_LIB_DIR, f"{filename}.kicad_sym")
        with open(path) as f:
            text = f.read()
        block = list(extract_symbol_block(text, bare_name))
        block[0] = f'\t(symbol "{full_name}"'
        return ["\n".join(block)]
    if lib_nick not in cache:
        with open(SYSTEM_LIB_SOURCES[lib_nick]) as f:
            cache[lib_nick] = f.read()
    blocks = resolve_extends(lib_nick, bare_name, full_name, cache)
    return ["\n".join(b) for b in blocks]


def embed_lib_symbols(used_lib_ids, cache, extra_text=None):
    """Builds the `(lib_symbols ...)` inner text for one output file, given
    the set of lib_ids actually placed in it (plus, for main.kicad_sch, the
    inline-authored Nucleo symbol text)."""
    embedded = []
    seen = set()
    for lib_id in sorted(used_lib_ids):
        lib_nick, bare_name = lib_id.split(":", 1)
        for text in embed(lib_nick, bare_name, lib_id, cache):
            name = text.split('"')[1]
            if name in seen:
                continue
            seen.add(name)
            embedded.append(text)
    if extra_text:
        embedded.append(extra_text)
    return "\n".join(embedded)


def write_sheet_file(filename, sheet, lib_symbols_text, is_root):
    parts = [
        "(kicad_sch",
        "\t(version 20250114)",
        '\t(generator "eeschema")',
        '\t(generator_version "10.0")',
        f'\t(uuid "{ROOT_UUID if is_root else u()}")',
        '\t(paper "A0")' if is_root else '\t(paper "A2")',
        "\t(lib_symbols",
        lib_symbols_text,
        "\t)",
    ]
    if sheet.sheet_blocks:
        parts.append("\n".join(sheet.sheet_blocks))
    if sheet.symbols:
        parts.append("\n".join(sheet.symbols))
    if sheet.wires:
        parts.append("\n".join(sheet.wires))
    if sheet.no_connects:
        parts.append("\n".join(sheet.no_connects))
    if sheet.hier_labels:
        parts.append("\n".join(sheet.hier_labels))
    if sheet.labels:
        parts.append("\n".join(sheet.labels))
    if is_root:
        parts.append('\t(sheet_instances\n\t\t(path "/" (page "1"))\n\t)')
    parts.append(")")
    parts.append("")
    with open(filename, "w") as f:
        f.write("\n".join(parts))


cache = {}

power_sheet = build_power()
motora_sheet = build_motor(
    "A", MOTORA_UUID,
    hin=("PA8", "PA9", "PA10"), lin=("PA7", "PB0", "PF0"),
    bemf=("PA0", "PA1", "PA3"), neutral_net="PB1", curr_net="PB11",
)
motorb_sheet = build_motor(
    "B", MOTORB_UUID,
    hin=("PA15", "PB8", "PB9"), lin=("PB3", "PB4", "PB5"),
    bemf=("PA4", "PA5", "PA6"), neutral_net="PB14", curr_net="PF1",
)
main_sheet, nucleo_symtext = build_main()

write_sheet_file("power.kicad_sch", power_sheet, embed_lib_symbols(power_sheet.used_lib_ids, cache), is_root=False)
write_sheet_file("motor_a.kicad_sch", motora_sheet, embed_lib_symbols(motora_sheet.used_lib_ids, cache), is_root=False)
write_sheet_file("motor_b.kicad_sch", motorb_sheet, embed_lib_symbols(motorb_sheet.used_lib_ids, cache), is_root=False)
write_sheet_file("main.kicad_sch", main_sheet, embed_lib_symbols(set(), cache, extra_text=nucleo_symtext), is_root=True)

print(
    f"Wrote main.kicad_sch + power/motor_a/motor_b.kicad_sch: "
    f"{len(power_sheet.symbols)+len(motora_sheet.symbols)+len(motorb_sheet.symbols)+len(main_sheet.symbols)} symbols total"
)
