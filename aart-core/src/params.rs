//! Runtime-tunable parameters, gettable/settable over the UART protocol
//! (DESIGN.md section 7.7) and persisted to flash by `stm32_os`.
//!
//! Field names are deliberately copied from
//! [ESCape32](https://github.com/neoxic/ESCape32)'s own configuration
//! (`src/common.h`'s `Cfg` struct, `src/prog.c`'s `CFG_MAP`) wherever a
//! clear equivalent exists - `freq_min`/`freq_max`, `duty_spup`,
//! `duty_drag`, `timing`, `revdir` are ESCape32's own names, not invented
//! here. Two differences from ESCape32, both consequences of this being a
//! 2-motor firmware rather than ESCape32's single-motor one:
//!
//! - `timing`/`revdir` become `timing_a`/`timing_b`/`revdir_a`/`revdir_b`
//!   (one motor's commutation timing/direction can genuinely differ from
//!   the other's - opposite wiring orientation, for instance - where a
//!   single-motor ESC has no such split to make).
//! - `freq_min`/`freq_max`/`duty_spup`/`duty_ramp`/`duty_drag` stay singular
//!   (shared across both motors), since these are motor/track tuning
//!   parameters expected to be identical for a matched pair, not
//!   per-motor wiring quirks.
//!
//! This module is pure/hardware-agnostic: no ADC, no flash, no GPIO - just
//! the parameter table, validation, and the byte encoding `stm32_os` writes
//! to flash. `get`/`set` return `ParseError` (from `protocol.rs`) rather
//! than inventing a parallel error type, since GET/SET failures are part
//! of the same UART error vocabulary as everything else on this channel.

use crate::protocol::ParseError;

/// Marks a flash page as holding a valid `Params` encoding (see
/// `to_bytes`/`from_bytes`) rather than erased (`0xFF` fill) or garbage.
/// ESCape32 itself doesn't checksum its stored config at all (`checkcfg`'s
/// per-field clamping is its only defense against garbage) - a magic
/// marker here is already stronger than that precedent, so a full
/// checksum on top was judged not worth the extra code for what's still
/// just a corruption *sanity check*, not a safety-critical integrity
/// guarantee.
const MAGIC: u32 = 0x4141_5254; // "AART"

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Params {
    /// PWM switching frequency at low eRPM, kHz - ESCape32's `freq_min`.
    pub freq_min: u16,
    /// PWM switching frequency at high eRPM, kHz - ESCape32's `freq_max`.
    pub freq_max: u16,
    /// Startup/spin-up duty, percent - ESCape32's `duty_spup`.
    pub duty_spup: u8,
    /// Sync ramp rate, eRPM added per commutation step - ESCape32's
    /// `duty_ramp` (units differ: ESCape32's is a kERPM *threshold*, ours
    /// is a per-step *increment* - the closest analog, not an identical
    /// quantity).
    pub duty_ramp: u16,
    /// Drag-brake ("Latvian brake") chop duty, percent - ESCape32's
    /// `duty_drag`. See `aart_core::brake`/`Bridge::brake`.
    pub duty_drag: u8,
    /// Motor A commutation timing advance, 0-31 - ESCape32's `timing`
    /// (their range is 1-31; 0 here means "no advance," matching the
    /// formula's own natural zero point - see `commutator.rs`'s
    /// `on_zero_cross`).
    pub timing_a: u8,
    /// Motor B commutation timing advance, 0-31 - see `timing_a`.
    pub timing_b: u8,
    /// Motor A commutation direction reversed - ESCape32's `revdir`.
    pub revdir_a: bool,
    /// Motor B commutation direction reversed - see `revdir_a`.
    pub revdir_b: bool,
}

impl Params {
    /// Compiled-in defaults - matches what `stm32_os/src/main.rs` used as
    /// fixed constants before this module existed.
    pub const fn defaults() -> Self {
        Self {
            freq_min: 48,
            freq_max: 96,
            duty_spup: 15,
            duty_ramp: 500,
            duty_drag: 60,
            timing_a: 0,
            timing_b: 0,
            revdir_a: false,
            revdir_b: false,
        }
    }

    /// Serialized size in bytes (magic + fields) - see `to_bytes`.
    pub const BYTE_LEN: usize = 16;

    /// Fixed layout, little-endian - simple and sufficient for a struct
    /// this small; no need for a general-purpose serialization crate in
    /// `no_std`.
    pub fn to_bytes(&self) -> [u8; Self::BYTE_LEN] {
        let mut buf = [0u8; Self::BYTE_LEN];
        buf[0..4].copy_from_slice(&MAGIC.to_le_bytes());
        buf[4..6].copy_from_slice(&self.freq_min.to_le_bytes());
        buf[6..8].copy_from_slice(&self.freq_max.to_le_bytes());
        buf[8] = self.duty_spup;
        buf[9..11].copy_from_slice(&self.duty_ramp.to_le_bytes());
        buf[11] = self.duty_drag;
        buf[12] = self.timing_a;
        buf[13] = self.timing_b;
        buf[14] = self.revdir_a as u8;
        buf[15] = self.revdir_b as u8;
        buf
    }

    /// `None` if `buf` is too short, or the magic doesn't match (erased
    /// flash reads back as `0xFF` fill, which will never match `MAGIC`) -
    /// callers should fall back to `defaults()` in either case.
    pub fn from_bytes(buf: &[u8]) -> Option<Self> {
        if buf.len() < Self::BYTE_LEN {
            return None;
        }
        if u32::from_le_bytes(buf[0..4].try_into().ok()?) != MAGIC {
            return None;
        }
        Some(Self {
            freq_min: u16::from_le_bytes(buf[4..6].try_into().ok()?),
            freq_max: u16::from_le_bytes(buf[6..8].try_into().ok()?),
            duty_spup: buf[8],
            duty_ramp: u16::from_le_bytes(buf[9..11].try_into().ok()?),
            duty_drag: buf[11],
            timing_a: buf[12],
            timing_b: buf[13],
            revdir_a: buf[14] != 0,
            revdir_b: buf[15] != 0,
        })
    }
}

impl Default for Params {
    fn default() -> Self {
        Self::defaults()
    }
}

fn ranged(value: i32, min: i32, max: i32) -> Result<i32, ParseError> {
    if value < min || value > max {
        Err(ParseError::OutOfRange)
    } else {
        Ok(value)
    }
}

/// Reads a named parameter as a plain integer (booleans as 0/1) - the
/// shape `GET`'s wire response needs.
pub fn get(params: &Params, name: &str) -> Result<i32, ParseError> {
    Ok(match name {
        "freq_min" => params.freq_min as i32,
        "freq_max" => params.freq_max as i32,
        "duty_spup" => params.duty_spup as i32,
        "duty_ramp" => params.duty_ramp as i32,
        "duty_drag" => params.duty_drag as i32,
        "timing_a" => params.timing_a as i32,
        "timing_b" => params.timing_b as i32,
        "revdir_a" => params.revdir_a as i32,
        "revdir_b" => params.revdir_b as i32,
        _ => return Err(ParseError::UnknownParam),
    })
}

/// Validates and applies `value` to the named parameter, returning the
/// value actually stored (always `== value` here - out-of-range values are
/// rejected outright rather than silently clamped, unlike ESCape32's own
/// `checkcfg`, so an operator never has to wonder whether a `SET` "worked"
/// with a different value than what they asked for).
pub fn set(params: &mut Params, name: &str, value: i32) -> Result<i32, ParseError> {
    match name {
        "freq_min" => params.freq_min = ranged(value, 20, 150)? as u16,
        "freq_max" => params.freq_max = ranged(value, 20, 150)? as u16,
        "duty_spup" => params.duty_spup = ranged(value, 1, 100)? as u8,
        "duty_ramp" => params.duty_ramp = ranged(value, 1, 10_000)? as u16,
        "duty_drag" => params.duty_drag = ranged(value, 0, 100)? as u8,
        "timing_a" => params.timing_a = ranged(value, 0, 31)? as u8,
        "timing_b" => params.timing_b = ranged(value, 0, 31)? as u8,
        "revdir_a" => params.revdir_a = ranged(value, 0, 1)? != 0,
        "revdir_b" => params.revdir_b = ranged(value, 0, 1)? != 0,
        _ => return Err(ParseError::UnknownParam),
    }
    get(params, name)
}

/// All recognized parameter names, for a future `SHOW`-style listing
/// command - not wired to anything yet, kept here so the name list has one
/// source of truth alongside `get`/`set`.
pub const PARAM_NAMES: &[&str] = &[
    "freq_min",
    "freq_max",
    "duty_spup",
    "duty_ramp",
    "duty_drag",
    "timing_a",
    "timing_b",
    "revdir_a",
    "revdir_b",
];

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn get_reads_back_defaults() {
        let p = Params::defaults();
        assert_eq!(get(&p, "freq_min"), Ok(48));
        assert_eq!(get(&p, "freq_max"), Ok(96));
        assert_eq!(get(&p, "duty_spup"), Ok(15));
        assert_eq!(get(&p, "duty_ramp"), Ok(500));
        assert_eq!(get(&p, "duty_drag"), Ok(60));
        assert_eq!(get(&p, "timing_a"), Ok(0));
        assert_eq!(get(&p, "timing_b"), Ok(0));
        assert_eq!(get(&p, "revdir_a"), Ok(0));
        assert_eq!(get(&p, "revdir_b"), Ok(0));
    }

    #[test]
    fn get_unknown_name_is_rejected() {
        assert_eq!(get(&Params::defaults(), "bogus"), Err(ParseError::UnknownParam));
    }

    #[test]
    fn set_updates_the_field_and_echoes_it_back() {
        let mut p = Params::defaults();
        assert_eq!(set(&mut p, "duty_drag", 80), Ok(80));
        assert_eq!(p.duty_drag, 80);
    }

    #[test]
    fn set_rejects_out_of_range_values_rather_than_clamping() {
        let mut p = Params::defaults();
        assert_eq!(set(&mut p, "timing_a", 32), Err(ParseError::OutOfRange));
        assert_eq!(p.timing_a, 0, "rejected SET must not partially apply");
        assert_eq!(set(&mut p, "timing_a", -1), Err(ParseError::OutOfRange));
    }

    #[test]
    fn set_accepts_the_documented_boundaries() {
        let mut p = Params::defaults();
        assert_eq!(set(&mut p, "timing_a", 0), Ok(0));
        assert_eq!(set(&mut p, "timing_a", 31), Ok(31));
        assert_eq!(set(&mut p, "duty_drag", 0), Ok(0));
        assert_eq!(set(&mut p, "duty_drag", 100), Ok(100));
    }

    #[test]
    fn set_treats_revdir_as_a_zero_one_bool() {
        let mut p = Params::defaults();
        assert_eq!(set(&mut p, "revdir_a", 1), Ok(1));
        assert!(p.revdir_a);
        assert_eq!(set(&mut p, "revdir_a", 0), Ok(0));
        assert!(!p.revdir_a);
        assert_eq!(set(&mut p, "revdir_a", 2), Err(ParseError::OutOfRange));
    }

    #[test]
    fn set_unknown_name_is_rejected() {
        let mut p = Params::defaults();
        assert_eq!(set(&mut p, "bogus", 5), Err(ParseError::UnknownParam));
    }

    #[test]
    fn timing_a_and_timing_b_are_independent() {
        let mut p = Params::defaults();
        set(&mut p, "timing_a", 10).unwrap();
        assert_eq!(p.timing_a, 10);
        assert_eq!(p.timing_b, 0, "must not cross-wire the two motors' settings");
    }

    #[test]
    fn to_bytes_from_bytes_round_trips() {
        let mut p = Params::defaults();
        set(&mut p, "timing_a", 17).unwrap();
        set(&mut p, "revdir_b", 1).unwrap();
        set(&mut p, "freq_max", 120).unwrap();
        let bytes = p.to_bytes();
        assert_eq!(Params::from_bytes(&bytes), Some(p));
    }

    #[test]
    fn from_bytes_rejects_erased_flash() {
        // Erased flash reads back as all-0xFF.
        let erased = [0xFFu8; Params::BYTE_LEN];
        assert_eq!(Params::from_bytes(&erased), None);
    }

    #[test]
    fn from_bytes_rejects_a_too_short_buffer() {
        let bytes = Params::defaults().to_bytes();
        assert_eq!(Params::from_bytes(&bytes[..Params::BYTE_LEN - 1]), None);
    }

    #[test]
    fn from_bytes_rejects_a_bad_magic() {
        let mut bytes = Params::defaults().to_bytes();
        bytes[0] ^= 0xFF; // corrupt just the magic
        assert_eq!(Params::from_bytes(&bytes), None);
    }
}
