//! Integration test for the GET/SET/SAVE UART commands (DESIGN.md section
//! 7.7), driven through the exact same public functions `stm32_os/src/
//! main.rs`'s command loop calls - `LineReader` fed one byte at a time
//! (mimicking how `command.rs` actually delivers UART bytes), `parse_line`,
//! `aart_core::params::get`/`set`, and `format_param`/`format_error` for
//! the wire response - so this exercises the real wire protocol
//! byte-for-byte without needing real hardware, QEMU, or Renode (none of
//! which can currently exercise this path either - QEMU's own test target
//! doesn't cover `main.rs`'s command loop at all, and reproducibly hangs
//! on unrelated setup even for what it does cover; Renode has no ADC/UART
//! interrupt model for this chip).
//!
//! What this deliberately does *not* cover: `SAVE`'s actual flash write
//! (`stm32_os::config_store`, hardware-only) and `apply_params`'s push
//! into the live `Commutator`s (touches `main.rs`'s `MOTOR_A`/`MOTOR_B`
//! statics, also hardware-only) - both need real hardware or a working
//! QEMU/Renode ADC+flash model. What *is* covered here is everything
//! upstream of those: the exact bytes an operator's terminal would see.

use aart_core::params::{self, Params};
use aart_core::protocol::{format_error, format_param, parse_line, Command, LineReader};

/// Feeds `line` (including its own `\n` or `\r\n`) through `reader` one
/// byte at a time, dispatches whatever `Command` results exactly the way
/// `main.rs`'s command loop does for `GET`/`SET`, and returns the bytes
/// that would be written back over UART as a `String`. Panics if `line`
/// doesn't complete in one shot (every test below sends one full line per
/// call) or isn't valid UTF-8 - acceptable for a test helper, not
/// production code.
fn get_set(reader: &mut LineReader<64>, params: &mut Params, line: &str) -> String {
    let mut out = String::new();
    for &byte in line.as_bytes() {
        let Some(result) = reader.push_byte(byte) else {
            continue;
        };
        let line = result.expect("test input is valid UTF-8 and fits the buffer");
        let mut buf = [0u8; 64];
        let n = match parse_line(line) {
            Ok(Command::Get(name)) => match params::get(params, name) {
                Ok(value) => format_param(&mut buf, name, value),
                Err(e) => format_error(e, &mut buf),
            },
            Ok(Command::Set(name, value)) => match params::set(params, name, value) {
                Ok(applied) => format_param(&mut buf, name, applied),
                Err(e) => format_error(e, &mut buf),
            },
            Ok(Command::Save) => {
                // Real SAVE also calls stm32_os::config_store::save() here
                // (flash I/O, hardware-only) - the wire response itself is
                // just these two fixed strings, nothing to parse-test.
                buf[..11].copy_from_slice(b"SAVE=1 OK\r\n");
                11
            }
            Ok(Command::Throttle(_)) | Ok(Command::Steer(_)) => 0,
            Err(e) => format_error(e, &mut buf),
        };
        out.push_str(core::str::from_utf8(&buf[..n]).expect("formatters only emit ASCII"));
    }
    out
}

#[test]
fn get_reads_back_a_default() {
    let mut reader = LineReader::<64>::new();
    let mut params = Params::defaults();
    assert_eq!(get_set(&mut reader, &mut params, "GET timing_a\n"), "timing_a=0 OK\r\n");
}

#[test]
fn set_then_get_reflects_the_new_value() {
    let mut reader = LineReader::<64>::new();
    let mut params = Params::defaults();
    assert_eq!(
        get_set(&mut reader, &mut params, "SET duty_drag 75\n"),
        "duty_drag=75 OK\r\n"
    );
    assert_eq!(get_set(&mut reader, &mut params, "GET duty_drag\n"), "duty_drag=75 OK\r\n");
}

#[test]
fn set_out_of_range_is_rejected_and_leaves_the_value_unchanged() {
    let mut reader = LineReader::<64>::new();
    let mut params = Params::defaults();
    assert_eq!(get_set(&mut reader, &mut params, "SET timing_a 99\n"), "ERR out of range\r\n");
    assert_eq!(get_set(&mut reader, &mut params, "GET timing_a\n"), "timing_a=0 OK\r\n");
}

#[test]
fn get_and_set_reject_an_unknown_parameter_name() {
    let mut reader = LineReader::<64>::new();
    let mut params = Params::defaults();
    assert_eq!(get_set(&mut reader, &mut params, "GET bogus\n"), "ERR unknown parameter\r\n");
    assert_eq!(get_set(&mut reader, &mut params, "SET bogus 1\n"), "ERR unknown parameter\r\n");
}

#[test]
fn timing_a_and_timing_b_stay_independent_over_the_wire() {
    let mut reader = LineReader::<64>::new();
    let mut params = Params::defaults();
    get_set(&mut reader, &mut params, "SET timing_a 20\n");
    assert_eq!(get_set(&mut reader, &mut params, "GET timing_a\n"), "timing_a=20 OK\r\n");
    assert_eq!(
        get_set(&mut reader, &mut params, "GET timing_b\n"),
        "timing_b=0 OK\r\n",
        "setting motor A's timing must not leak into motor B's"
    );
}

#[test]
fn crlf_terminated_lines_work_the_same_as_bare_lf() {
    let mut reader = LineReader::<64>::new();
    let mut params = Params::defaults();
    assert_eq!(
        get_set(&mut reader, &mut params, "SET freq_max 120\r\n"),
        "freq_max=120 OK\r\n"
    );
}

#[test]
fn a_full_session_arriving_in_arbitrary_byte_sized_chunks() {
    // Mimics a real UART read returning however many bytes happened to be
    // in the FIFO, not one tidy line at a time - command.rs's actual
    // `has_data`/`read` loop in main.rs works the same way.
    let mut reader = LineReader::<64>::new();
    let mut params = Params::defaults();
    let session = b"SET duty_spup 30\nGET duty_spup\nSET revdir_a 1\nGET revdir_a\n";

    let mut out = String::new();
    for chunk in session.chunks(7) {
        // chunk boundaries deliberately don't line up with line boundaries
        let text = core::str::from_utf8(chunk).unwrap();
        out.push_str(&get_set(&mut reader, &mut params, text));
    }

    assert_eq!(
        out,
        "duty_spup=30 OK\r\nduty_spup=30 OK\r\nrevdir_a=1 OK\r\nrevdir_a=1 OK\r\n"
    );
}

#[test]
fn save_then_reload_round_trips_every_field_a_real_power_cycle_would_need() {
    // Doesn't touch stm32_os::config_store (flash, hardware-only), but
    // Params::to_bytes/from_bytes *is* exactly what SAVE serializes and a
    // real boot deserializes - this is the meaningful part of "does SAVE
    // actually preserve what I set" that's reachable on the host.
    let mut reader = LineReader::<64>::new();
    let mut params = Params::defaults();
    get_set(&mut reader, &mut params, "SET freq_min 60\n");
    get_set(&mut reader, &mut params, "SET duty_drag 80\n");
    get_set(&mut reader, &mut params, "SET timing_b 22\n");
    get_set(&mut reader, &mut params, "SET revdir_b 1\n");
    assert_eq!(get_set(&mut reader, &mut params, "SAVE\n"), "SAVE=1 OK\r\n");

    let reloaded = Params::from_bytes(&params.to_bytes()).expect("just-saved bytes must be valid");
    assert_eq!(reloaded, params, "a reload after SAVE must see exactly what was set");
}
