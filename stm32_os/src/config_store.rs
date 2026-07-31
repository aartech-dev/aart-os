//! Flash persistence for `aart_core::params::Params` - DESIGN.md section 7.7.
//!
//! Same overall shape as ESCape32's own config storage (`src/util.c`'s
//! `savecfg`/`checkcfg`, `src/main.c`'s `cfgdata`/`cfg`): a compiled-in
//! default, a RAM working copy the firmware actually reads from, and a
//! single dedicated flash page that gets erased and reprogrammed wholesale
//! on save - no wear-leveling, no ping-pong pages, matching ESCape32's own
//! choice not to bother with either for a value that's saved rarely (an
//! operator tuning settings, not a per-tick write).
//!
//! One real difference from ESCape32: this checks a magic marker
//! (`Params::from_bytes`) before trusting what's in flash, where ESCape32
//! relies solely on `checkcfg`'s per-field clamping to survive garbage -
//! `Params::from_bytes` already explains why a marker was judged
//! sufficient without a full checksum on top.
//!
//! **Known hardware caveat, not engineered around here**: erasing/
//! programming a flash page briefly stalls the CPU's ability to fetch new
//! instructions from that same flash bank (RM0440) - since the ADC1_2
//! ISR's code lives in that same bank, `save()` pauses real-time
//! commutation stepping for the duration (a 2KB page erase can take on
//! the order of tens of milliseconds). The PWM hardware itself keeps
//! running from its own timer registers regardless (it's not CPU-driven),
//! so this doesn't glitch the actual motor drive, but BEMF zero-cross
//! sampling/scheduling pauses and resumes once the write completes -
//! same tradeoff ESCape32 itself accepts (its own `savecfg` runs with
//! `__disable_irq()` held for the whole operation, and `execcmd`'s `save`
//! case is gated by `!ertm && !busy`, i.e. "not currently spinning").
//! Recommended use here is the same: save during setup/bench tuning, not
//! mid-run.

use aart_core::params::Params;
use stm32g4xx_hal::flash::{FlashSize, Parts};

/// Byte offset (from `FLASH_START`) of the reserved config page - the last
/// 2KB page of the part's real 512K, kept out of the linker's reach by
/// `memory.x`'s shortened `FLASH` region length (510K).
const CONFIG_FLASH_OFFSET: u32 = 510 * 1024;

/// STM32G4 flash page size - one full erase granularity (RM0440).
const PAGE_SIZE_BYTES: u32 = 2048;

/// Loads persisted parameters, falling back to `Params::defaults()` if
/// nothing valid has ever been saved (erased flash, or a size/magic
/// mismatch from some earlier firmware version's layout).
pub fn load(flash_parts: &mut Parts) -> Params {
    let writer = flash_parts.writer::<PAGE_SIZE_BYTES>(FlashSize::Sz512K);
    match writer.read(CONFIG_FLASH_OFFSET, Params::BYTE_LEN) {
        Ok(bytes) => Params::from_bytes(bytes).unwrap_or_else(Params::defaults),
        Err(_) => Params::defaults(),
    }
}

/// Erases and reprograms the config page with `params`. Returns `false` on
/// any flash error (erase failure, write failure, or the post-write
/// verification `FlashWriter` performs by default). See the module doc
/// comment for the real-time cost of calling this while motors are
/// actively commutating.
pub fn save(flash_parts: &mut Parts, params: &Params) -> bool {
    let mut writer = flash_parts.writer::<PAGE_SIZE_BYTES>(FlashSize::Sz512K);
    if writer.page_erase(CONFIG_FLASH_OFFSET).is_err() {
        return false;
    }
    writer.write(CONFIG_FLASH_OFFSET, &params.to_bytes(), true).is_ok()
}
