# ---------------------------------------------------------------------------
# Renode PythonPeripheral shim for STM32G4 ADC1/ADC2.
#
# Renode has no STM32G4 ADC model (same gap as the RCC one - see
# rcc_shim.py). This is NOT a faithful ADC: it exists purely so the HAL's
# blocking enable()/convert() polling loops terminate under emulation instead
# of spinning forever, by echoing the two status bits they wait on:
#   * CR.ADEN (bit 0) set  -> ISR.ADRDY (bit 0) reads back set, so
#     Adc::enable()'s ready-wait returns.
#   * CR.ADSTART (bit 2) set -> ISR.EOC (bit 2) and ISR.EOS (bit 3) read back
#     set immediately, so wait_for_conversion_sequence() returns on its next
#     poll instead of waiting for real conversion timing.
#   * DR always reads back `sample_value` (mid-scale for a 12-bit ADC).
#     Edit it, or extend this shim, if a test needs a specific sequence of
#     values instead of a constant.
#
# Bit positions confirmed against the same "ADC_v2" register layout STM32G4
# shares with L4/F3 (RM0440; cross-checked against libopencm3's
# adc_common_v2.h, which uses identical bit numbers).
#
# Attach one instance per ADC: ADC1 @ 0x5000_0000, ADC2 @ 0x5000_0100.
# ADC12_COMMON (0x5000_0300) isn't polled by anything on the claim()/
# calibrate()/convert() path this shim targets, so it doesn't need one -
# a plain Memory.MappedMemory (or omitting it, if nothing reads/writes it in
# your test) is enough.
# ---------------------------------------------------------------------------

ISR = 0x00
CR = 0x08
DR = 0x40

ADEN_BIT = 1 << 0
ADSTART_BIT = 1 << 2
ADRDY_BIT = 1 << 0
EOC_BIT = 1 << 2
EOS_BIT = 1 << 3

sample_value = 2048

if request.isInit:
    regs = {}
    isr = 0

elif request.isRead:
    off = request.offset
    if off == ISR:
        request.value = isr
    elif off == DR:
        request.value = sample_value
    else:
        request.value = regs.get(off, 0)

elif request.isWrite:
    off = request.offset
    val = request.value
    regs[off] = val

    if off == CR:
        if val & ADEN_BIT:
            isr |= ADRDY_BIT
        if val & ADSTART_BIT:
            isr |= EOC_BIT | EOS_BIT
