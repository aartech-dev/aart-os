#!/usr/bin/env python3
"""Generates adapter.kicad_sch from a data-driven component/net list.

Hand-authoring ~100 near-identical KiCad symbol-placement blocks directly
is repetitive and error-prone; this script computes exact pin coordinates
(component position + library pin offset, rotated) and emits both the
symbol placement and a matching global_label at each net connection, so
connectivity is correct by construction rather than by manual coordinate
arithmetic. Run, then validate with `kicad-cli sch erc adapter.kicad_sch`.
"""
import math
import uuid

def u():
    return str(uuid.uuid4())

def rot(x, y, deg):
    r = math.radians(deg)
    return (x * math.cos(r) - y * math.sin(r), x * math.sin(r) + y * math.cos(r))

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

sch_symbols = []   # placement sexp text
sch_labels = []    # global_label sexp text
sch_wires = []     # short pin->label stub wires (see place(), STUB_LEN)
sch_no_connects = []  # explicit no_connect markers for genuinely-unused pins
ref_counters = {}
pwr_counter = [0]

# Extra length (mm) each pin grows by via a short stub wire before a label
# lands on it. Densely-pinned parts (AP3012, DRV8300D, CSD16327Q3, etc.)
# have several pins within a few mm of each other on the symbol body itself;
# placing a global_label directly on the raw pin tip crowds its text into
# neighboring pins/labels. Extending every pin outward (away from the
# symbol's own center, same direction the pin already points) by a fixed
# stub before the label lands there gives the label room without changing
# connectivity - the wire segment is zero-impedance, same net either way.
STUB_LEN = 6.0


def wire(x1, y1, x2, y2):
    sch_wires.append(
        "\n".join([
            "\t(wire",
            f"\t\t(pts (xy {x1:.2f} {y1:.2f}) (xy {x2:.2f} {y2:.2f}))",
            "\t\t(stroke (width 0) (type default))",
            f'\t\t(uuid "{u()}")',
            "\t)",
        ])
    )


def next_ref(prefix):
    ref_counters[prefix] = ref_counters.get(prefix, 0) + 1
    return f"{prefix}{ref_counters[prefix]}"


def place(lib_id, x, y, rotation=0, ref=None, ref_prefix="U", value=None, footprint_override=None, unit=1, no_connect=()):
    """Places one symbol instance; returns {pin_number: (abs_x, abs_y)}."""
    if ref is None:
        ref = next_ref(ref_prefix)
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
    text.append(f'\t\t(instances (project "adapter" (path "/" (reference "{ref}") (unit {unit}))))')
    text.append("\t)")
    sch_symbols.append("\n".join(text))

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
            sch_no_connects.append(
                f'\t(no_connect (at {px:.2f} {py:.2f}) (uuid "{u()}"))'
            )
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


def label(net, x, y, shape="passive"):
    sch_labels.append(
        "\n".join([
            f'\t(global_label "{net}"',
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
    """Connects every (x,y) point to the same net via matching labels."""
    for (x, y) in points:
        label(name, x, y)


# ---------------------------------------------------------------------------
# SHARED: track power in, reverse-polarity protection, boost, LDO,
# bidirectional track-current sense (DESIGN.md section 6.5/6.4/7.5/7.7).
# ---------------------------------------------------------------------------

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
_, j1 = place("Connector_Generic:Conn_01x01", 20, 160, ref_prefix="J", value="Track +")
_, j2 = place("Connector_Generic:Conn_01x01", 20, 120, ref_prefix="J", value="Track -")
net("TRACK_PWR", j1["1"])
net("TRACK_RTN_RAW", j2["1"])
pwr_flag(20, 175)
net("TRACK_PWR", (20, 175))

_, q1 = place("AdapterSymbols:CSD16327Q3", 65, 120, ref_prefix="Q", value="CSD16327Q3")
net("TRACK_RTN_RAW", q1["5"])  # D
net("GND", q1["1"])            # S (x3 pins, same coordinate)
net("RP_GATE", q1["4"])        # G

_, r1 = place("Device:R", 65, 155, rotation=90, ref_prefix="R", value="10k")
net("TRACK_PWR", r1["1"])
net("RP_GATE", r1["2"])

_, d1 = place("AdapterSymbols:D_Zener_Small", 95, 135, ref_prefix="D", value="15V")
net("RP_GATE", d1["1"])  # K
net("GND", d1["2"])      # A

net("TRACK_PWR", (20, 160))  # VBUS = TRACK_PWR directly (unprotected side - see module doc)
label("VBUS", 20, 160)

# --- Boost regulator: VBUS -> VCC (boosted gate-drive supply), ~12V target ---
# AP3012 adjustable boost. R2/R3 set the FB divider - placeholder values
# targeting ~12V assuming a ~1.25V FB reference; verify against the AP3012
# datasheet's actual reference voltage before board bring-up. L1/D2/C1/C2
# values are likewise placeholders pending real characterization.
_, u1 = place("AdapterSymbols:AP3012", 140, 150, ref_prefix="U", value="AP3012")
net("VBUS", u1["5"])   # IN
net("VBUS", u1["4"])   # ~SHDN tied to IN - always enabled (matches RemoraNSR3.0's own wiring)
net("GND", u1["2"])    # GND
gnd_pt = gnd(140, 175)  # real power:GND symbol - for schematic readability
net("GND", gnd_pt)
_gnd_flag = pwr_flag(140, 190)  # power:GND's own pin is power_in, not power_out - still needs a PWR_FLAG
net("GND", (140, 190))

_, l1 = place("Device:L", 110, 165, rotation=90, ref_prefix="L", value="10uH")
net("VBUS", l1["1"])
net("AP3012_SW", l1["2"])
net("AP3012_SW", u1["1"])  # SW

_, d2 = place("Device:D", 165, 165, ref_prefix="D", value="Schottky (e.g. SS14)")
# Device:D pin "1" = K (cathode), pin "2" = A (anode) - a boost rectifier
# conducts switch-node -> VCC, i.e. anode at the switch node, cathode at VCC.
net("AP3012_SW", d2["2"])  # A
net("VCC", d2["1"])        # K

_, c1 = place("Device:C", 140, 115, rotation=90, ref_prefix="C", value="10uF")
net("VBUS", c1["1"])
net("GND", c1["2"])

_, c2 = place("Device:C", 190, 150, rotation=90, ref_prefix="C", value="22uF")
net("VCC", c2["1"])
net("GND", c2["2"])

_, r2 = place("Device:R", 210, 165, rotation=90, ref_prefix="R", value="86k")
net("VCC", r2["1"])
net("AP3012_FB", r2["2"])
net("AP3012_FB", u1["3"])  # FB

_, r3 = place("Device:R", 210, 130, rotation=90, ref_prefix="R", value="10k")
net("AP3012_FB", r3["1"])
net("GND", r3["2"])

# VCC bus-tap label deliberately omitted here - it'll be added by task #20
# wiring directly to each DRV8300D's GVDD pin, once those are placed.
_vcc_flag = pwr_flag(140, 100)  # boost output isn't power_out-typed per ERC (SW/diode pins are passive)
net("VCC", (140, 100))

# --- Logic LDO: VCC -> VDD (3.3V, matches the Nucleo's own logic rail) ---
_, u2 = place("AdapterSymbols:AP2204K-3.3", 240, 150, ref_prefix="U", value="AP2204K-3.3", no_connect={"4"})
net("VCC", u2["1"])   # VIN
net("VCC", u2["3"])   # EN tied on
net("GND", u2["2"])   # GND
net("VDD", u2["5"])   # VOUT

_, c3 = place("Device:C", 220, 115, rotation=90, ref_prefix="C", value="1uF")
net("VCC", c3["1"])
net("GND", c3["2"])

_, c4 = place("Device:C", 260, 115, rotation=90, ref_prefix="C", value="1uF")
net("VDD", c4["1"])
net("GND", c4["2"])

# VDD bus-tap label deliberately omitted here - it'll be added by task #20 /
# a later revision wiring directly to the Nucleo 3V3 pin. VDD already has a
# real power_out driver (U2 VOUT below), so no PWR_FLAG needed for it.

# --- Bidirectional shared track-current sense (DESIGN.md 6.5/7.5) ---
# Shunt in the VBUS path; MCP6002 unit A buffers a VDD/2 reference, unit B
# is a 4-resistor difference amp centered on it - chosen over betting on an
# unfamiliar bidirectional current-sense IC (see commit notes). Output net
# PA2 matches the real GPIO this feeds (DESIGN.md 6.1/6.5).
_, r_shunt = place("Device:R", 65, 90, rotation=90, ref_prefix="R", value="5mOhm 1W")
net("VBUS", r_shunt["1"])
# VBUS_LOAD (not VBUS) is what both bridges' high-side drains actually
# connect to (see motor_block below) - the shunt must sit in series
# between the raw input and the bridges for the sense amp to see real
# load current, per DESIGN.md 6.5 ("shunt on the main input path ...
# before it splits to the two bridges"). A stray duplicate net name here
# previously left the bridges tied to raw VBUS, bypassing the shunt
# entirely - fixed alongside this layout pass since it was found while
# untangling an overlapping label at this exact point.
net("VBUS_LOAD", r_shunt["2"])

_, r4 = place("Device:R", 310, 165, rotation=90, ref_prefix="R", value="10k")
net("VDD", r4["1"])
net("VDD_HALF_RAW", r4["2"])
_, r5 = place("Device:R", 310, 130, rotation=90, ref_prefix="R", value="10k")
net("VDD_HALF_RAW", r5["1"])
net("GND", r5["2"])

u3_ref, u3 = place("Amplifier_Operational:LM2904", 345, 150, ref_prefix="U", value="MCP6002-xSN", unit=1)
net("VDD_HALF_RAW", u3["3"])   # unit A: IN1+ = raw divider midpoint
net("VDD_HALF", u3["2"])       # unit A: IN1- fed back from OUT1 (voltage follower)
net("VDD_HALF", u3["1"])       # unit A: OUT1 = buffered VDD/2 reference

_, u3pwr = place("Amplifier_Operational:LM2904", 345, 150, ref=u3_ref, unit=3)  # shared power pins
net("GND", u3pwr["4"])  # V-
net("VDD", u3pwr["8"])  # V+

_, u3b = place("Amplifier_Operational:LM2904", 400, 150, ref=u3_ref, value=None, unit=2)
_, r6 = place("Device:R", 380, 165, rotation=90, ref_prefix="R", value="1k")
net("VBUS", r6["1"])
net("DIFF_IN_PLUS", r6["2"])
net("DIFF_IN_PLUS", u3b["5"])  # unit B: IN2+

_, r7 = place("Device:R", 380, 130, rotation=90, ref_prefix="R", value="1k")
net("VBUS_LOAD", r7["1"])
net("DIFF_IN_MINUS", r7["2"])
net("DIFF_IN_MINUS", u3b["6"])  # unit B: IN2-

_, r8 = place("Device:R", 425, 165, rotation=90, ref_prefix="R", value="20k")
net("DIFF_IN_PLUS", r8["1"])
net("VDD_HALF", r8["2"])

_, r9 = place("Device:R", 425, 130, rotation=90, ref_prefix="R", value="20k")
net("DIFF_IN_MINUS", r9["1"])
net("PA2", r9["2"])
net("PA2", u3b["7"])  # unit B: OUT2 = ADC1 channel 3, shared track-current sense

# --- Shared bus-voltage sense (DESIGN.md 6.1: PB12, "one reading is
# enough" - not per-motor, just a plain divider off VBUS) ---
_, rv1 = place("Device:R", 65, 60, rotation=90, ref_prefix="R", value="47k")
net("VBUS", rv1["1"])
net("PB12", rv1["2"])
_, rv2 = place("Device:R", 65, 40, rotation=90, ref_prefix="R", value="10k")
net("PB12", rv2["1"])
net("GND", rv2["2"])


# ---------------------------------------------------------------------------
# PER-MOTOR: DRV8300D driver + 6x CSD16327Q3 bridge + bootstrap caps +
# per-phase BEMF dividers + virtual-neutral summing network + per-motor
# unidirectional current-sense amp (DESIGN.md 6.5's "what scales with motor
# count" block, x2 for front/rear). GPIO net names are passed in directly
# from DESIGN.md 6.4's pin table (§6.1 superseded for BEMF_C/neutral/current
# by the G474 retarget) - connectivity to those nets is by matching
# global_label name only, so exact physical layout here doesn't need to
# mirror the Nucleo's own pin geography.
# ---------------------------------------------------------------------------

def motor_block(suffix, ox, oy, hin, lin, bemf, neutral_net, curr_net):
    """hin/lin/bemf are each a 3-tuple of net names for phases U,V,W."""
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

    # Physical Nucleo header interface: one connector pin per HIN/LIN
    # signal, so the schematic honestly represents where these 6 gate-drive
    # inputs actually enter this board. kicad-cli ERC will still flag all 6
    # as "Input pin not driven by any Output pins" - that's an expected,
    # accepted finding, not a defect: the real driver (the STM32G474's TIM1/
    # TIM8 complementary PWM outputs) is on the Nucleo dev board, a separate
    # physical board this single-sheet schematic doesn't model. See the
    # commit notes / README for this documented exclusion.
    for idx, (name, net_name) in enumerate(zip(("HIN1", "HIN2", "HIN3", "LIN1", "LIN2", "LIN3"), hin + lin)):
        _, jn = place("Connector_Generic:Conn_01x01", ox - 110, oy + 40 - 15 * idx,
                      ref_prefix="J", value=f"Nucleo {net_name} ({name}, motor {suffix})")
        net(net_name, jn["1"])

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

    # Per-leg: bootstrap cap (VBx<->VSx), high-side FET (drain=VBUS,
    # source=phase node, gate via resistor from HOx), low-side FET
    # (drain=phase node, source=common motor-return node, gate via
    # resistor from LOx), phase connector out to the motor winding, and
    # the BEMF divider (47k/10k, midpoint to that phase's ADC pin).
    ho_pins = ("19", "16", "13")   # HO1,HO2,HO3
    lo_pins = ("11", "10", "9")    # LO1,LO2,LO3
    vb_pins = ("20", "17", "14")   # VB1,VB2,VB3
    vs_pins = ("18", "15", "12")   # VS1,VS2,VS3
    leg_dx = 95
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


# Motor A: TIM1 / ADC1 / ADC3 (DESIGN.md 6.4)
motor_block(
    "A", 320, 340,
    hin=("PA8", "PA9", "PA10"), lin=("PA7", "PB0", "PF0"),
    bemf=("PA0", "PA1", "PA3"), neutral_net="PB1", curr_net="PB11",
)

# Motor B: TIM8 / ADC2 / ADC4 (DESIGN.md 6.4)
motor_block(
    "B", 320, 650,
    hin=("PA15", "PB8", "PB9"), lin=("PB3", "PB4", "PB5"),
    bemf=("PA4", "PA5", "PA6"), neutral_net="PB14", curr_net="PF1",
)

# Assembly: extract each used symbol's full definition from its source
# library file (system KiCad libraries, or the local AdapterSymbols.kicad_sym
# directory copied from aartech-dev/RemoraNSR3.0) and embed it in
# lib_symbols, exactly how a real KiCad save does it - kicad-cli's ERC reads
# pin geometry from this embedded cache, not by resolving lib_id against
# sym-lib-table at check time (confirmed empirically - see commit notes).
# ---------------------------------------------------------------------------
import os

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


def rename_child_units(block, bare_name, full_name):
    """Renames nested `(symbol "{bare_name}_N_M" ...)` sub-unit drawings to
    match a renamed parent. KiCad associates a multi-unit symbol's child
    sub-drawings to its parent purely by name-prefix convention
    (`"{parent}_{unit}_{style}"`) - renaming only the parent's own header
    line (as the lib-id-qualifying rename below does) leaves children
    orphaned under their old bare name, silently breaking pin-geometry
    lookup for every unit past the first (confirmed empirically: this is
    why single-unit parts worked immediately but MCP6002 kept reporting
    correctly-computed pin coordinates as still unconnected)."""
    prefix = f'\t\t(symbol "{bare_name}_'
    replacement = f'\t\t(symbol "{full_name}_'
    return [replacement + line[len(prefix):] if line.startswith(prefix) else line for line in block]


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


used_lib_ids = sorted(set(PINS.keys()))
cache = {}
embedded = []
seen = set()
for lib_id in used_lib_ids:
    lib_nick, bare_name = lib_id.split(":", 1)
    for text in embed(lib_nick, bare_name, lib_id, cache):
        # A base symbol pulled in for two different `extends` children
        # (not the case yet, but cheap to guard) would otherwise be
        # embedded twice under the same name.
        name = text.split('"')[1]
        if name in seen:
            continue
        seen.add(name)
        embedded.append(text)

lib_symbols_block = "\n".join(embedded)

output = "\n".join([
    "(kicad_sch",
    "\t(version 20250114)",
    "\t(generator \"eeschema\")",
    "\t(generator_version \"10.0\")",
    f'\t(uuid "{u()}")',
    '\t(paper "A0")',
    "\t(lib_symbols",
    lib_symbols_block,
    "\t)",
    "\n".join(sch_symbols),
    "\n".join(sch_wires),
    "\n".join(sch_no_connects),
    "\n".join(sch_labels),
    "\t(sheet_instances",
    '\t\t(path "/"',
    '\t\t\t(page "1")',
    "\t\t)",
    "\t)",
    ")",
    "",
])

with open("adapter.kicad_sch", "w") as f:
    f.write(output)

print(f"Wrote adapter.kicad_sch: {len(sch_symbols)} symbols, {len(sch_labels)} labels")
