//! Command/telemetry line protocol over the UART command channel (DESIGN.md
//! section 7.2):
//!
//! ```text
//! > THR 0.35
//! > STEER -0.10
//! < SPD a=1200rpm b=980rpm slip=0.00 OK
//! ```
//!
//! Byte I/O lives in `stm32_os` (see `command.rs`); everything here is pure,
//! hardware-agnostic parsing/formatting, host-testable with plain
//! `cargo test`.

use core::fmt::Write as _;

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Command {
    Throttle(f32),
    Steer(f32),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ParseError {
    Empty,
    UnknownCommand,
    MissingValue,
    /// Either not a number, or extra tokens after the value - deliberately
    /// strict rather than silently ignoring trailing garbage on a
    /// motor-control channel.
    InvalidValue,
    OutOfRange,
}

const THROTTLE_RANGE: core::ops::RangeInclusive<f32> = 0.0..=1.0;
const STEER_RANGE: core::ops::RangeInclusive<f32> = -1.0..=1.0;

/// Parses one already-delimited line (no terminator). Use `LineReader` to
/// get there from a raw byte stream.
pub fn parse_line(line: &str) -> Result<Command, ParseError> {
    let line = line.trim();
    let mut tokens = line.split_whitespace();

    let keyword = tokens.next().ok_or(ParseError::Empty)?;
    let value_str = tokens.next().ok_or(ParseError::MissingValue)?;
    if tokens.next().is_some() {
        return Err(ParseError::InvalidValue);
    }
    let value: f32 = value_str.parse().map_err(|_| ParseError::InvalidValue)?;

    match keyword {
        "THR" => {
            if THROTTLE_RANGE.contains(&value) {
                Ok(Command::Throttle(value))
            } else {
                Err(ParseError::OutOfRange)
            }
        }
        "STEER" => {
            if STEER_RANGE.contains(&value) {
                Ok(Command::Steer(value))
            } else {
                Err(ParseError::OutOfRange)
            }
        }
        _ => Err(ParseError::UnknownCommand),
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LineError {
    /// No terminator arrived before the buffer filled up; the partial line
    /// was discarded so accumulation can resync from the next byte.
    TooLong,
    /// The accumulated bytes weren't valid UTF-8 (e.g. line noise).
    NotUtf8,
}

/// Accumulates raw bytes into terminator-delimited lines, tolerating bytes
/// arriving split across arbitrarily many reads (a UART read can return a
/// partial line, a whole line, or several lines at once).
pub struct LineReader<const N: usize> {
    buf: [u8; N],
    len: usize,
}

impl<const N: usize> LineReader<N> {
    pub const fn new() -> Self {
        Self { buf: [0; N], len: 0 }
    }

    /// Feed one byte. Returns `Some(line)` the moment `byte` completes a
    /// line (`\n`, with a preceding `\r` stripped if present), `None` if
    /// more bytes are still needed. Call once per byte received; drain every
    /// `Some` before moving on, since a single chunk can complete more than
    /// one line.
    pub fn push_byte(&mut self, byte: u8) -> Option<Result<&str, LineError>> {
        if byte == b'\n' {
            let mut end = self.len;
            if end > 0 && self.buf[end - 1] == b'\r' {
                end -= 1;
            }
            let result = core::str::from_utf8(&self.buf[..end]).map_err(|_| LineError::NotUtf8);
            self.len = 0;
            return Some(result);
        }

        if self.len == N {
            // Overflowed without ever seeing a terminator: drop what we had
            // and drop this byte too, so the next terminator we do see
            // starts a clean resync rather than reporting a truncated line.
            self.len = 0;
            return Some(Err(LineError::TooLong));
        }

        self.buf[self.len] = byte;
        self.len += 1;
        None
    }
}

impl<const N: usize> Default for LineReader<N> {
    fn default() -> Self {
        Self::new()
    }
}

struct FixedWriter<'a> {
    buf: &'a mut [u8],
    len: usize,
}

impl<'a> FixedWriter<'a> {
    fn new(buf: &'a mut [u8]) -> Self {
        Self { buf, len: 0 }
    }
}

impl core::fmt::Write for FixedWriter<'_> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let bytes = s.as_bytes();
        let remaining = self.buf.len().saturating_sub(self.len);
        if bytes.len() > remaining {
            return Err(core::fmt::Error);
        }
        self.buf[self.len..self.len + bytes.len()].copy_from_slice(bytes);
        self.len += bytes.len();
        Ok(())
    }
}

/// Formats a `SPD ... OK` telemetry line into `buf` (CRLF-terminated).
/// Returns the number of bytes written, or 0 if it didn't fit.
///
/// `slip` has no real value to report until M5's differential controller
/// exists - callers pass 0.0 until then; the field is here now because it's
/// part of the documented wire format, not because it means anything yet.
pub fn format_telemetry(buf: &mut [u8], speed_a_rpm: i32, speed_b_rpm: i32, slip: f32) -> usize {
    let mut w = FixedWriter::new(buf);
    match write!(
        w,
        "SPD a={speed_a_rpm}rpm b={speed_b_rpm}rpm slip={slip:.2} OK\r\n"
    ) {
        Ok(()) => w.len,
        Err(_) => 0,
    }
}

/// Formats an `ERR ...` line for a rejected command. Returns the number of
/// bytes written, or 0 if it didn't fit.
pub fn format_error(err: ParseError, buf: &mut [u8]) -> usize {
    let reason = match err {
        ParseError::Empty => "empty",
        ParseError::UnknownCommand => "unknown command",
        ParseError::MissingValue => "missing value",
        ParseError::InvalidValue => "invalid value",
        ParseError::OutOfRange => "out of range",
    };
    let mut w = FixedWriter::new(buf);
    match write!(w, "ERR {reason}\r\n") {
        Ok(()) => w.len,
        Err(_) => 0,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn valid_throttle_parses() {
        assert_eq!(parse_line("THR 0.35"), Ok(Command::Throttle(0.35)));
    }

    #[test]
    fn valid_steer_parses_negative() {
        assert_eq!(parse_line("STEER -0.10"), Ok(Command::Steer(-0.10)));
    }

    #[test]
    fn extra_whitespace_between_tokens_is_tolerated() {
        assert_eq!(parse_line("THR    0.5"), Ok(Command::Throttle(0.5)));
        assert_eq!(parse_line("  THR 0.5  "), Ok(Command::Throttle(0.5)));
    }

    #[test]
    fn empty_line_is_rejected() {
        assert_eq!(parse_line(""), Err(ParseError::Empty));
        assert_eq!(parse_line("   "), Err(ParseError::Empty));
    }

    #[test]
    fn unknown_command_is_rejected() {
        assert_eq!(parse_line("FOO 0.5"), Err(ParseError::UnknownCommand));
        assert_eq!(parse_line("thr 0.5"), Err(ParseError::UnknownCommand)); // case-sensitive
    }

    #[test]
    fn missing_value_is_rejected() {
        assert_eq!(parse_line("THR"), Err(ParseError::MissingValue));
    }

    #[test]
    fn non_numeric_value_is_rejected() {
        assert_eq!(parse_line("THR fast"), Err(ParseError::InvalidValue));
    }

    #[test]
    fn trailing_garbage_after_value_is_rejected() {
        assert_eq!(parse_line("THR 0.5 extra"), Err(ParseError::InvalidValue));
    }

    #[test]
    fn throttle_out_of_range_is_rejected() {
        assert_eq!(parse_line("THR 1.5"), Err(ParseError::OutOfRange));
        assert_eq!(parse_line("THR -0.1"), Err(ParseError::OutOfRange));
    }

    #[test]
    fn steer_out_of_range_is_rejected() {
        assert_eq!(parse_line("STEER 1.1"), Err(ParseError::OutOfRange));
        assert_eq!(parse_line("STEER -1.1"), Err(ParseError::OutOfRange));
    }

    #[test]
    fn throttle_and_steer_accept_their_range_boundaries() {
        assert_eq!(parse_line("THR 0.0"), Ok(Command::Throttle(0.0)));
        assert_eq!(parse_line("THR 1.0"), Ok(Command::Throttle(1.0)));
        assert_eq!(parse_line("STEER -1.0"), Ok(Command::Steer(-1.0)));
        assert_eq!(parse_line("STEER 1.0"), Ok(Command::Steer(1.0)));
    }

    fn feed<const N: usize>(reader: &mut LineReader<N>, bytes: &[u8]) -> Vec<Result<String, LineError>> {
        let mut lines = Vec::new();
        for &b in bytes {
            if let Some(result) = reader.push_byte(b) {
                lines.push(result.map(|s| s.to_string()));
            }
        }
        lines
    }

    #[test]
    fn one_shot_line_is_produced_on_the_terminator() {
        let mut r = LineReader::<32>::new();
        let lines = feed(&mut r, b"THR 0.5\n");
        assert_eq!(lines, [Ok("THR 0.5".to_string())]);
    }

    #[test]
    fn crlf_terminator_strips_the_cr() {
        let mut r = LineReader::<32>::new();
        let lines = feed(&mut r, b"THR 0.5\r\n");
        assert_eq!(lines, [Ok("THR 0.5".to_string())]);
    }

    #[test]
    fn line_split_across_multiple_feeds_still_assembles_correctly() {
        let mut r = LineReader::<32>::new();
        assert!(feed(&mut r, b"THR 0.").is_empty());
        assert!(feed(&mut r, b"3").is_empty());
        let lines = feed(&mut r, b"5\r\n");
        assert_eq!(lines, [Ok("THR 0.35".to_string())]);
    }

    #[test]
    fn one_feed_can_contain_more_than_one_line() {
        let mut r = LineReader::<32>::new();
        let lines = feed(&mut r, b"THR 0.5\nSTEER -0.1\n");
        assert_eq!(
            lines,
            [Ok("THR 0.5".to_string()), Ok("STEER -0.1".to_string())]
        );
    }

    #[test]
    fn overlong_line_without_terminator_reports_too_long_and_resyncs() {
        let mut r = LineReader::<8>::new();
        // 9 bytes, no terminator - overflows an 8-byte buffer.
        let lines = feed(&mut r, b"123456789");
        assert_eq!(lines, [Err(LineError::TooLong)]);

        // The reader should be usable again immediately afterward.
        let lines = feed(&mut r, b"THR 0.5\n");
        assert_eq!(lines, [Ok("THR 0.5".to_string())]);
    }

    #[test]
    fn telemetry_formats_the_documented_shape() {
        let mut buf = [0u8; 64];
        let n = format_telemetry(&mut buf, 1200, 980, 0.02);
        assert_eq!(&buf[..n], b"SPD a=1200rpm b=980rpm slip=0.02 OK\r\n");
    }

    #[test]
    fn telemetry_reports_zero_when_it_does_not_fit() {
        let mut buf = [0u8; 4];
        let n = format_telemetry(&mut buf, 1200, 980, 0.02);
        assert_eq!(n, 0);
    }

    #[test]
    fn error_formats_a_readable_reason() {
        let mut buf = [0u8; 32];
        let n = format_error(ParseError::OutOfRange, &mut buf);
        assert_eq!(&buf[..n], b"ERR out of range\r\n");
    }
}
