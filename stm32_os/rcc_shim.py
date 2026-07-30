# ---------------------------------------------------------------------------
# Renode PythonPeripheral shim for the STM32G4 RCC.
#
# Purpose: never let the HAL hang on a clock-ready poll under emulation.
#   * mirror every write so read-back (e.g. AHB2ENR.GPIOAEN) returns what the
#     firmware wrote,
#   * force the oscillator/PLL ready bits set in RCC_CR,
#   * reflect the SW field into the SWS field of RCC_CFGR so the clock-switch
#     wait loop terminates.
#
# This is a shim, not a faithful RCC. It's enough for clock init + GPIO tests.
# Register offsets are for STM32G4 (RM0440).
#
# API note: Renode's PythonPeripheral request object varies slightly between
# versions (isInit/isRead/isWrite + request.value/offset). If your build uses a
# different shape, adapt these three branches; the logic is what matters.
# ---------------------------------------------------------------------------

CR    = 0x00   # RCC_CR
CFGR  = 0x08   # RCC_CFGR

if request.isInit:
    # Persists across subsequent requests because the peripheral is `initable`.
    regs = {}

elif request.isRead:
    off = request.offset
    val = regs.get(off, 0)

    if off == CR:
        val |= (1 << 10)   # HSIRDY
        val |= (1 << 17)   # HSERDY
        val |= (1 << 25)   # PLLRDY

    elif off == CFGR:
        sw = val & 0x3                      # SW  = bits [1:0]
        val = (val & ~(0x3 << 2)) | (sw << 2)  # SWS = bits [3:2] echoes SW

    request.value = val

elif request.isWrite:
    regs[request.offset] = request.value
