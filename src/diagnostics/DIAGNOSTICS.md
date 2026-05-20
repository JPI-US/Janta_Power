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
- captures output lines into a transcript so either transport can send them back

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

### Serial commands

Current serial commands enabled in this phase:

- `FIRMWARE_VERSION`
- `GET_CAPABILITIES`
- `RTC_CHECK`
- `HDC1080_READ`
- `GO_HOME`

Serial still uses the old line-based command language from [`crates/diagnostics/src/lib.rs`](../../crates/diagnostics/src/lib.rs).

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
- `SET_ENV`, `SAVE_CONFIG`, `GET_ENV`, `GET_CONFIG`, `WRITE_ENV_FILE`, `CONFIG_MODE`, `MOTOR_MOVE`, relay commands, reboot, and display commands are still deferred
- Serial diagnostics is not running in its own background thread; it is polled from the main loop sleep slices
- Full firmware `cargo check` is still blocked in this Windows environment by the existing ESP-IDF path-length issue

## Next Logical Phase

The next phase should expand beyond the first wave carefully:

- add more runtime-safe command adapters
- decide which old config/environment commands still make sense in this firmware
- add explicit MQTT transcript schema expectations on the backend side if needed
- tighten serial-vs-MQTT busy/timeout behavior if any long-running commands are introduced
