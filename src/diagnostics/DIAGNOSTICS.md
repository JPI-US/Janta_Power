# Diagnostics Flow

This document tracks the current diagnostics architecture and should be updated as diagnostics phases change.

## Current Phase

Current phase: dual-transport diagnostics with the first-wave shared executor.

This phase adds:

- MQTT diagnostics and serial diagnostics using the same command-execution path
- a shared first-wave executor for hardware checks and go-home
- workspace wiring for the old `crates/diagnostics` crate and the new standalone `serial_console` crate
- serial responses over USB Serial/JTAG
- MQTT responses over the diagnostics MQTT ack topic

## Goal

The goal is to let diagnostics commands arrive from two places:

- from the backend over MQTT
- from a technician plugged into the board over USB Serial/JTAG

The command should be answered on the same transport that received it:

- MQTT in, MQTT out
- serial in, serial out

At the same time, only the main runtime loop is allowed to touch hardware, NVS, and the live motion state.

## High-Level Design

The current design has three layers:

- transport frontends
- shared diagnostics executor
- main runtime ownership

### Transport frontends

There are now two frontends:

- [`src/diagnostics/mqtt.rs`](./mqtt.rs)
- [`src/diagnostics/serial.rs`](./serial.rs)

MQTT:

- uses a dedicated MQTT client thread
- still handles fast read-only commands directly when safe
- queues hardware-touching commands back to the main loop
- publishes final MQTT results on `tower/{device_id}/cmd/diagnostics/ack`

Serial:

- uses `serial_console::UsbSerialJtagConsole`
- is polled from the main loop instead of running as a second hardware-owning thread
- parses line-based commands using the old diagnostics crate command parser
- writes the response lines back to the serial console

### Shared executor

[`src/diagnostics/executor.rs`](./executor.rs) is now the shared first-wave execution layer.

It:

- reuses `board_diagnostics::DiagnosticCommand`
- reuses `board_diagnostics::StandardDiagnosticHandler`
- adapts current runtime-owned hardware into a board implementation
- writes output to the caller's `DiagnosticIo` as the command produces it

That last point is what makes progress reporting possible. The executor does not
own a writer; each transport supplies one:

- serial passes `ConsoleIo`, which goes straight out, so `MOVING_HOME` reaches the
  technician when homing starts rather than after it ends
- MQTT passes `TranscriptIo`, which collects, because it publishes one message
  when the command finishes

`DiagnosticsTranscript.lines` is consequently filled in by the caller that
collected them, not by the executor.

### Main runtime ownership

[`src/runtime/main.rs`](../runtime/main.rs) is still the only owner of:

- `Motion`
- NVS
- shared I2C bus access used by diagnostics
- live heading and mode state
- queued MQTT hardware diagnostics

That rule is still the main production-safety guardrail.

## Commands In This Phase

### MQTT commands

Current MQTT commands:

- `ping`
- `get_status`
- `firmware_version`
- `get_capabilities`
- `rtc_check`
- `hdc1080_read`
- `go_home`
- `request_rehome`

Notes:

- `go_home` and `request_rehome` currently map to the same first-wave go-home executor path
- `ping`, `get_status`, `firmware_version`, and `get_capabilities` can be answered without touching runtime-owned hardware
- `rtc_check`, `hdc1080_read`, and `go_home` are executed on the main loop

### LED_TEST

Drives the WS2812 on GPIO7 to one named colour and returns. `RED`, `GREEN` and
`BLUE` exercise the three channels independently — one dead channel is a real
failure a white-only test would miss — while `WHITE` proves all three together and
`OFF` proves the LED can be driven dark rather than being stuck on. `RESTORE` puts
it back to the runtime's status colour.

The colours live in `board_diagnostics::LED_TEST_COLORS` and the result reports
`rgb=` so the host can tell what was actually driven; the installer keeps a matching
table to render its preview, and both are pinned by test.

The argument also accepts a literal `R,G,B` triplet — `LED_TEST 0,0,1` — for probing
hardware by hand. A named set walks an operator through a test but cannot answer
questions like whether `0,0,1` lights when `0,0,0` does not, and that is the sort of
question that separates a firmware bug from an LED that never goes fully dark.

If colours come out scrambled and vary between runs, suspect the data line rather
than this code. A WS2812 at 5 V wants roughly 3.5 V for a logic high and an ESP32
GPIO gives 3.3 V, so without a level shifter the link is marginal by construction.

`OFF` is the most sensitive colour to test with, and the most useful: all 24 bits
are zero, so a single bit misread as a 1 shows as a faint tint. On any lit colour
that same error is invisible among the bits already set. A faint glow on `OFF` is
therefore a bit-error rate readout, not an LED that will not switch off.

**This command cannot verify itself.** A WS2812 has no readback, so a dead LED, a
broken joint and a wrong pin all look identical from the firmware's side. The
result says the colour was *written*, never that light came out — the judgement is
the operator's, which is why the host drives the sequence one colour at a time and
asks. Do not let it grow into something that reports a pass nobody witnessed.

### I2C_SCAN

Probes the addresses this board actually carries — `0x40` HDC1080 and `0x68`
DS3231 — with a zero-length write. That is the standard probe and the safe one:
reading a byte instead can disturb devices that treat a read as a pointer advance.

Deliberately not a sweep of the whole 7-bit range. Every probe of an absent device
costs a full bus timeout, so 112 of them on a dead bus runs for minutes and blocks
the main loop throughout — the first version did exactly that and timed out on the
host. Two probes answer the question the command exists for.

It passes whenever anything answers. A silent bus is the failure; a missing sensor
is a detail. That is deliberate — the command exists precisely because a sensor read
failing with `NoAcknowledge` cannot distinguish "this device is absent" from "the
bus is dead".

### Configuration commands

`SET_ENV`, `GET_ENV`, `SAVE_CONFIG` and `GET_CONFIG` are routed away from the board
path in [`executor.rs`](./executor.rs), because they need the `&mut EspNvs` the
runtime board also holds and touch no hardware at all. They run against
`NvsEnvironment` with an `UnsupportedBoard` stub in the board slot.

`SET_ENV` validates against `board_diagnostics::CONFIG_KEYS` and stages into RAM.
`SAVE_CONFIG` commits the staged set, sets the `provisioned` flag so boot stops
seeding defaults over it, and clears the buffer — on a write failure the buffer is
left intact so the commit can be retried without resending every value. `GET_ENV`
reports the staged value if one is waiting, otherwise the committed one.

Secrets never leave the board. Reads report `<set>` or `<unset>`, and `SET_ENV`
refuses to store either as a password.

### Serial commands

Current serial commands enabled in this phase:

- `PING`
- `I2C_SCAN`
- `LED_TEST <colour>`
- `OLED_TEST [pattern]`
- `FIRMWARE_VERSION`
- `GET_CAPABILITIES`
- `RTC_CHECK`
- `HDC1080_READ`
- `GO_HOME`
- `REBOOT`

`REBOOT` answers `OK Rebooting` and then returns
`SerialDiagnosticsPoll::RebootRequested` from the poll. The frontend does not reset
the board itself — the main loop does, after a short delay that lets the reply
clear the USB Serial/JTAG FIFO before the CDC device drops with the reset.

Serial still uses the old line-based command language from [`crates/diagnostics/src/lib.rs`](../../crates/diagnostics/src/lib.rs).

### OLED_TEST

Drives the HS96L03W2C03 (SSD1306-compatible, 128x64, I²C 0x3C) through
`crates/drivers/ssd1306`. With no argument it walks every pattern, holding each for
700 ms — short enough that the whole cycle stays inside the host's default silence
budget, since serial is not serviced while it runs. With an argument it sets one
pattern and holds it.

The patterns are chosen for what each reveals: `ALL_ON` finds dead pixels, `OFF`
finds one stuck on, `CHECKERBOARD` finds addressing faults through a misaligned
grid, `STRIPES` isolates a segment driver, `BORDER` proves the edges are reachable
when a centre-covering pattern would not.

The panel acknowledges its address, so absence is detected before anything is drawn
and reported as a real failure — this test is not purely operator-judged the way
`LED_TEST` is.

### Announcing what the tower is doing

`SerialDiagnosticsRuntime::announce_phase` writes `PHASE <name>` when the phase
changes; repeating the current phase is a no-op so retry loops can call it every
pass. `sleep_answering_boot_diagnostics` and `pump_serial_during_move` announce
their own phase, and the idle window announces `ready` — the one point where every
command is available.

### Structured results

Every command ends in one `RESULT <NAME> PASS|FAIL <details>` line, built by
`board_diagnostics::result_line` so serial and MQTT cannot drift. `PASS` details
are `key=value` pairs recorded by the board method that ran — `temp`/`hum` from
HDC1080, `time` from the RTC, `heading` from go-home, `sections` and `committed`
from the configuration path. They are collected off the board through
`StandardDiagnosticHandler::board` after the command, which avoids widening
`DiagnosticBoard` with a details sink.

Legacy lines are still emitted ahead of the `RESULT`. A host that predates the
structured protocol resolves on those; one that understands it ignores them and
resolves on the `RESULT`. Both on the wire at once is what lets firmware and
installer ship independently — dropping the legacy lines is a later step, and
needs the version handshake to be in the field first.

`FIRMWARE_VERSION` emits `PROTOCOL_VERSION 1` before its version line, because the
host resolves the command on `VERSION` and anything after that arrives too late to
be recorded. Boot-phase refusals terminate with a `RESULT` too; without one a
strict host would wait out its timeout on a command the board had already refused.

Phase lines and homing progress are the only output the firmware sends unprompted,
so they can arrive while an unrelated command is pending. Both formats live in
`board_diagnostics` (`phase_line`, `homing_progress_line`) and are pinned by tests
in both repositories against the host's accept rules; a phase name that happened to
read as a terminal result would resolve whatever command was in flight.

### Serial during boot

The boot waits run the ordinary `poll` with `motion: None`. NVS and the I2C bus
are built long before the motion stack — lines 78 and 103 against line 425 — so the
sensors and the entire configuration set are answered during boot exactly as they
are after it. There is no separate response path to keep in step.

`MotionContext` bundles the motion handle with the five values only `go_home` uses,
and the board holds it as an `Option`. When it is absent `go_home` refuses with
`motion is not initialised yet`, which is the actual reason; unimplemented commands
still give theirs. `ERROR BUSY <phase>` is reserved for what it describes.

`poll_minimal` covers the mid-move case. The move holds the motor — and only the
motor. The status LED is not taken by anything; the I²C bus is a `&'static` manager
locking per transaction that motion never touches; and NVS is free during a homing
search, because the search borrows nothing but the motor. So `poll_minimal` runs the
same `execute_first_wave_command` the idle path runs, with `motion: None`, rather
than serving a hand-picked subset.

What stays refused is
[`board_diagnostics::requires_exclusive_runtime`](../../crates/diagnostics/src/lib.rs):
`GO_HOME`, `MOTOR_MOVE` and `RELAY_MOTOR`, which want the motor or its power, and
`REBOOT`, which belongs to the main loop — serving it here would mean resetting from
inside the stepping loop with the motor relay still closed, or holding the request
until the move ends, which for a homing sweep can be twenty minutes later. Those
four get the same `ERROR BUSY <phase>` a boot wait sends, so nothing on the wire
changed; the set of commands that receive it shrank.

That predicate is an exhaustive `match` with no wildcard arm, so a command added
later stops the crate compiling until somebody classifies it. Getting it wrong is
otherwise invisible: a command that could have run is refused as `BUSY`, which reads
on the wire exactly like a refusal that was meant.

Two things vary with what the move can spare:

- **NVS.** A homing search lends it; a tracking pass and encoder recovery hold it
  for their own bookkeeping and pass `None`. Configuration commands then refuse with
  `configuration storage is not lendable during a tracking move` — never `BUSY`,
  because "busy" invites waiting for a move that will be followed by another.
- **Time.** `MoveTolerance::Homing` allows a command to hold the stepping loop for
  seconds; `PositionCritical` does not. Only the full `OLED_TEST` walk asks for
  that: at 10 kHz a full-screen write is about a second and the walk is five of
  them.

  Worth being precise about *why*, because the obvious answer is wrong. The tower
  is not turning fast enough for a pause to cost steps — 25,600 microsteps per motor
  revolution behind an 85:1 slew, accelerating at 200 steps/s², makes a one-degree
  homing move eleven seconds long and never gets it past about 1,100 steps/s, under
  3 rpm at the motor. Nothing restarted from rest at that speed loses anything. What
  a pause actually costs is supervision: the stall and overshoot detectors live
  inside the stepping loop and run only between `motor.poll()` calls, so a blocked
  loop is a blind one. Homing switches both off for its whole duration, which is
  what makes the walk free there and not during a tracking move.

The sensor reads never approach that limit. A whole `HDC1080_READ` is about 60 ms,
an `RTC_CHECK` about 10 ms and an `I2C_SCAN` about 3 ms, against a stall detector
that samples every 250 ms. Raising the bus clock would remove the OLED constraint
too — every device on it supports 400 kHz — but it is left at 10 kHz deliberately
and changing it is a separate decision.

`poll_minimal` takes no control-command reservation, unlike `poll`. The reservation
stops an MQTT command and a serial command running at once, and that cannot happen
here: MQTT hands its commands to the main loop through a channel, and the main loop
is the thread blocked inside this move. Taking it would only add a way for a stale
MQTT reservation to lock out the person stood at the tower.

That is why the LED is *not* part of `MotionContext`. Motion and the LED have
different availability: motion disappears during a move, the LED does not. Bundling
them made `LED_TEST` fail with `BUSY homing` for no reason other than the shape of a
struct.

`main.rs` drives the waits from `sleep_answering_boot_diagnostics`, which replaces
the bare `thread::sleep` calls with 50 ms slices. The phases are `waiting to connect
Wi-Fi`, `boot validation`, `waiting for MQTT`, `retrying MQTT boot validation`,
`checking for OTA update`, and `settling after homing`.

`RTC_CHECK` reads the DS3231 directly and reports its time as `rtc_utc` beside the
system clock as `system_local`. It fails when the read fails or the year falls
outside 2024–2100 — the same range `Rtc::init` uses to decide whether the tower can
boot, so the diagnostic and the boot path agree about what counts as a usable
clock.

### Gaps the waits do not cover

Boot also blocks inside driver calls that do not sleep — Wi-Fi association, the
clock, MQTT client setup. Serial cannot be serviced during those, because they
block the only thread that would poll it, so a command sent into one of them times
out rather than being refused.

Each announces its phase immediately *before* blocking, so a connected host sees
`PHASE connecting Wi-Fi` and knows why the link went quiet. Closing these properly
would mean moving association off the main thread, which is a larger change than
the sleep-slicing that covers the rest of boot.

### Serial during long moves

A homing search can run for tens of minutes, and a tracking pass blocks the loop
before its idle window is reached. The `motion` crate offers `MoveTick` and a
`_watched` variant of every entry point that can block; the callback fires about
ten times a second from the same place `run` already logs position.

`main.rs` passes `pump_serial_during_move`, which calls `poll_readonly`. Because
that path can only answer commands needing no hardware, running it from inside the
stepping loop does not break the single-owner rule — a command that would touch
motion, NVS, or the bus is refused with `ERROR BUSY` instead.

`go_home` uses the same hook for the other direction: it reports
`HOMING steps_remaining=... encoder_ticks=...` every five seconds, which keeps the
installer's silence budget alive across a search no total timeout could cover.
The line shape is fixed by `board_diagnostics::homing_progress_line` and pinned by
tests in both repositories — it must never equal `LIMIT` or begin with `ERROR`,
or the host resolves `GO_HOME` while the tower is still moving.

## Files Involved

### Workspace wiring

[`Cargo.toml`](../../Cargo.toml)

- adds `crates/diagnostics` to the workspace
- adds `crates/infrastructure/serial_console` to the workspace
- aliases the old diagnostics crate as `board_diagnostics`
- adds the standalone `serial_console` crate as a dependency

[`crates/diagnostics/Cargo.toml`](../../crates/diagnostics/Cargo.toml)

- now points at the new `crates/infrastructure/serial_console` path

### Old reusable diagnostics logic

[`crates/diagnostics/src/lib.rs`](../../crates/diagnostics/src/lib.rs)

- provides the line-command parser
- provides the board/environment/config traits
- provides `StandardDiagnosticHandler`

We are reusing this crate as logic, not as a second firmware runtime model.

### Serial transport

[`crates/infrastructure/serial_console/src/lib.rs`](../../crates/infrastructure/serial_console/src/lib.rs)

- owns the low-level USB Serial/JTAG read/write
- provides `SerialLineRuntime`
- provides `UsbSerialJtagConsole`

### Current firmware diagnostics modules

[`src/diagnostics/mod.rs`](./mod.rs)

- exposes the current diagnostics modules

[`src/diagnostics/executor.rs`](./executor.rs)

- shared first-wave executor
- runtime board adapter for RTC, HDC1080, and go-home
- transcript collection
- command classification helpers

[`src/diagnostics/mqtt.rs`](./mqtt.rs)

- dedicated MQTT diagnostics listener
- dedupe, rate limiting, busy-state, and timeout handling
- direct MQTT responses for safe read-only commands
- queues first-wave hardware diagnostics back to the main loop
- publishes final transcript-style MQTT results

[`src/diagnostics/serial.rs`](./serial.rs)

- serial frontend for the first-wave commands
- writes `CMD_RECEIVED`
- parses serial lines into diagnostics commands
- executes them through the shared executor on the main loop
- writes transcript lines back to serial

[`src/runtime/main.rs`](../runtime/main.rs)

- creates the shared diagnostics snapshot
- creates the MQTT diagnostics command queue
- starts the dedicated MQTT listener
- creates the serial diagnostics runtime
- services queued MQTT diagnostics commands
- polls serial diagnostics during the sliced sleep window

## Runtime Flow

### MQTT flow

1. Backend publishes to `tower/{device_id}/cmd/diagnostics`
2. [`src/diagnostics/mqtt.rs`](./mqtt.rs) receives and parses the JSON
3. Request protection runs first:
   - duplicate detection
   - rate limiting
   - busy-state check
4. Safe read-only commands may respond immediately
5. Hardware-touching first-wave commands are reserved and queued back to the main loop
6. The main loop executes them through [`executor.rs`](./executor.rs)
7. Final transcript result is published back to `tower/{device_id}/cmd/diagnostics/ack`

### Serial flow

1. A technician sends a line over USB Serial/JTAG
2. [`src/diagnostics/serial.rs`](./serial.rs) reads the line when the main loop polls it
3. The serial frontend writes `CMD_RECEIVED`
4. The line is parsed with `board_diagnostics::DiagnosticCommand::parse(...)`
5. Supported first-wave commands execute through [`executor.rs`](./executor.rs)
6. The returned transcript lines are written back to serial

## Important Safety Rules Still In Place

- The MQTT diagnostics thread still does not touch motion or NVS directly
- Serial diagnostics is polled on the main loop, not run as a second hardware-owning worker
- The main loop is still the single owner of motion, heading, NVS, and shared I2C
- One control command at a time is still enforced through the diagnostics command state

## Current Limitations

- The old diagnostics crate still contains its own serial runtime pieces, but the current firmware now prefers the standalone `serial_console` crate for the live transport
- Only the first-wave commands are integrated right now
- `MOTOR_MOVE` and the relay commands are still deferred
- `WRITE_ENV_FILE` is refused outright: there is no filesystem to write
- Serial diagnostics is not running in its own background thread; it is polled from the main loop sleep slices
- Full firmware `cargo check` is still blocked in this Windows environment by the existing ESP-IDF path-length issue

## Next Logical Phase

The next phase should expand beyond the first wave carefully:

- add more runtime-safe command adapters
- decide which old config/environment commands still make sense in this firmware
- add explicit MQTT transcript schema expectations on the backend side if needed
- tighten serial-vs-MQTT busy/timeout behavior if any long-running commands are introduced
