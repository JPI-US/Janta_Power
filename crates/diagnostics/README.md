# diagnostics

`diagnostics` implements the reusable text-command protocol used by the Janta installer UI and ESP32 firmware. It parses commands, dispatches them to a handler, and provides a standard handler whose hardware, persistent environment, and reported configuration are supplied by the consuming firmware.

It does not directly own GPIO pins, I2C drivers, NVS, reboot behavior, or the firmware's main loop. In this repository, the current firmware mainly reuses this crate for command parsing and protocol handling, while the live USB Serial/JTAG transport is usually framed by the separate `serial_console` crate.

## Responsibilities

- Parse text into `DiagnosticCommand` values.
- Acknowledge each complete command with `CMD_RECEIVED`.
- Dispatch commands through `DiagnosticHandler`.
- Provide `StandardDiagnosticHandler` for the standard Janta protocol.
- Separate board hardware, environment storage, and configuration through traits.
- Report `RebootRequested` so the consuming firmware controls the actual restart.
- Keep protocol behavior testable on a computer with mocks.

## Adding the Crate

From the repository root firmware, add the crate with the workspace path that matches the current layout:

```toml
[dependencies]
board_diagnostics = { package = "diagnostics", path = "crates/diagnostics" }
```

The root firmware in this repository aliases the package as `board_diagnostics` so it does not collide with the local [`src/diagnostics`](../../src/diagnostics) module.

Add `serial_console` separately only if the firmware also needs the standalone serial line runtime used by the current dual-transport integration:

```toml
[dependencies]
serial_console = { path = "crates/infrastructure/serial_console" }
```

If another crate inside the workspace depends on these packages, adjust the relative paths to match that crate's location.

## Implementing a Firmware

A standard firmware integration has four parts.

### 1. Implement `DiagnosticBoard`

The board implementation owns the real hardware drivers and defines what hardware commands do:

```rust
use diagnostics::{DiagnosticBoard, DiagnosticIo};

struct MyBoard {
    // GPIO, I2C, display, motor, relay, and sensor drivers
}

impl DiagnosticBoard for MyBoard {
    type Error = MyBoardError;

    fn hdc1080_read<IO: DiagnosticIo>(&mut self, io: &mut IO) -> Result<(), Self::Error> {
        // Read the physical sensor and write protocol response lines.
        Ok(())
    }

    // Implement rtc_check, motor_move, go_home, oled_test,
    // relay_motor, and relay_hotspot in the same way.
}
```

All required methods should perform real validation and return an error when the hardware action fails. Writing `OK` without checking the physical result makes the UI report a false pass.

`config_mode` has a default no-op implementation and can be overridden when the firmware needs explicit configuration-mode behavior.

### 2. Implement `DiagnosticEnvironment`

This implementation owns mutable configuration and persistence:

- `set_env` updates a value in memory.
- `get_env` retrieves a value.
- `save_config` persists current values, normally to ESP32 NVS.
- `write_env_file` is optional and defaults to success.
- Error-message methods translate storage failures into stable protocol responses.

The standard handler writes `OK` only after `set_env`, `save_config`, or `write_env_file` succeeds.

### 3. Provide `DiagnosticConfiguration`

Use `StaticDiagnosticConfiguration` when firmware version, capabilities, and configuration sections can be assembled at startup:

```rust
use diagnostics::{DiagnosticConfigSection, StaticDiagnosticConfiguration};

let configuration = StaticDiagnosticConfiguration::new("1.0.0")
    .with_capability("WIFI")
    .with_capability("MQTT")
    .with_section(
        DiagnosticConfigSection::new("device")
            .with_entry("tower_id", "TOWER_007"),
    );
```

Implement `DiagnosticConfiguration` directly if values must be generated dynamically.

### 4. Choose a Runtime Style

This crate supports two integration styles.

#### Option A: use the built-in diagnostics runtime

Create the transport, handler, and runtime after hardware initialization:

```rust
use diagnostics::{
    DiagnosticPoll, DiagnosticRuntime, StandardDiagnosticHandler,
    UsbSerialJtagConsole,
};
use std::thread;
use std::time::Duration;

UsbSerialJtagConsole::install_driver(1024, 1024)?;

let board = MyBoard::new(/* hardware drivers */);
let environment = MyEnvironment::new(/* NVS and shared values */);
let configuration = build_diagnostic_configuration();
let mut handler = StandardDiagnosticHandler::new(board, environment, configuration);
let mut runtime = DiagnosticRuntime::new();
let mut console = UsbSerialJtagConsole::new();

loop {
    match runtime.poll(&mut console, &mut handler) {
        Ok(DiagnosticPoll::Idle) => thread::sleep(Duration::from_millis(20)),
        Ok(DiagnosticPoll::CommandProcessed) => {}
        Ok(DiagnosticPoll::RebootRequested) => {
            // Flush or delay briefly, then call the platform restart function.
        }
        Err(error) => {
            // Log the handler error and decide whether to continue or recover.
        }
    }
}
```

The runtime never restarts the processor itself. The firmware must handle `RebootRequested`, allowing the response to be transmitted before calling `esp_restart`.

#### Option B: use an external transport/runtime

If the firmware already has its own transport loop, parse incoming text with `DiagnosticCommand::parse(...)` and then route the command through `DiagnosticHandler` or a shared executor. That is the pattern used by the current firmware:

- [`src/diagnostics/serial.rs`](../../src/diagnostics/serial.rs) uses `serial_console::SerialLineRuntime` for USB Serial/JTAG framing
- [`src/diagnostics/executor.rs`](../../src/diagnostics/executor.rs) reuses `DiagnosticCommand` and `StandardDiagnosticHandler` concepts for the shared first-wave executor
- [`src/diagnostics/mqtt.rs`](../../src/diagnostics/mqtt.rs) accepts MQTT diagnostics requests and routes hardware-touching work back to the main loop

## Supported Commands

| Command | Dispatched behavior |
| --- | --- |
| `HDC1080_READ` | `DiagnosticBoard::hdc1080_read` |
| `RTC_CHECK` | `DiagnosticBoard::rtc_check` |
| `MOTOR_MOVE <argument>` | `DiagnosticBoard::motor_move` |
| `GO_HOME` | `DiagnosticBoard::go_home` |
| `OLED_TEST <payload>` | `DiagnosticBoard::oled_test` |
| `RELAY_MOTOR <state>` | `DiagnosticBoard::relay_motor` |
| `RELAY_HOTSPOT <state>` | `DiagnosticBoard::relay_hotspot` |
| `CONFIG_MODE` | `DiagnosticBoard::config_mode` |
| `SET_ENV <key>=<value>` | Update an environment value |
| `GET_ENV <key>` | Return `<key>=<value>` |
| `SAVE_CONFIG` | Persist the environment |
| `WRITE_ENV_FILE` | Run the optional environment-file operation |
| `FIRMWARE_VERSION` | Return `VERSION <version>` |
| `GET_CAPABILITIES` | Return comma-separated capabilities |
| `GET_CONFIG` | Stream configuration sections and values |
| `REBOOT` | Return `OK Rebooting` and request a reboot |

Unknown commands return `ERROR Unknown command: <COMMAND>`. Malformed `SET_ENV` input returns `ERROR Bad SET_ENV format`.

## Unit Tests

The test module replaces every external dependency with an in-memory implementation:

- `MockTransport` supplies serial bytes and captures response lines.
- `MockBoard` replaces GPIO, motors, relays, RTC, OLED, and other hardware.
- `MockEnvironment` replaces ESP32 NVS and can simulate save failures.
- `MockConfiguration` provides deterministic version, capability, and config output.
- `MockHdcSensor` supplies known sensor identifiers and readings.

The current test suite verifies:

1. Valid, lowercase, whitespace-heavy, and malformed `SET_ENV` parsing.
2. Unknown command parsing.
3. Exact end-to-end output for environment, version, capabilities, and unknown commands.
4. Streaming configuration lines through `GET_CONFIG`.
5. The expected protocol error when `SAVE_CONFIG` fails.
6. Returning `RebootRequested` while preserving the reboot response.
7. Dispatching a board command through `StandardDiagnosticHandler`.
8. The complete successful HDC1080 helper response sequence.
9. Building version, capability, and configuration output with `StaticDiagnosticConfiguration`.

These tests validate parsing, dispatch, and protocol output. They do not communicate with a USB device or validate physical hardware.

## Running Tests

The repository-level `.cargo/config.toml` defaults to the ESP32-S3 target. Override it with the computer's host target for unit tests.

From the repository root on Windows:

```powershell
cargo test -p diagnostics --target x86_64-pc-windows-msvc
```

From inside the crate directory:

```powershell
cargo test --target x86_64-pc-windows-msvc
```

After dependencies are available locally, testing can be forced offline:

```powershell
cargo test --target x86_64-pc-windows-msvc --offline
```

On another operating system, use the host triple shown by `rustc -vV` instead of `x86_64-pc-windows-msvc`.

Run one test by adding its name:

```powershell
cargo test -p diagnostics `
  --target x86_64-pc-windows-msvc `
  parses_set_env_command_variants
```

## Adding Tests

Keep protocol tests deterministic and hardware-independent:

1. Put complete or partial command bytes in `MockTransport`.
2. Construct `StandardDiagnosticHandler` with the required mocks.
3. Call `DiagnosticRuntime::poll` or `handle_command` directly.
4. Assert the returned control/poll value.
5. Assert every response line in order.
6. Add failure flags to mocks when testing error behavior.

When adding a new command, add parser coverage, successful dispatch coverage, expected output coverage, and at least one relevant failure case.

## What Unit Tests Do Not Cover

- ESP32 USB Serial/JTAG driver installation or real COM-port traffic.
- GPIO voltage, relay switching, motor movement, limit switches, display output, or button presses.
- Real I2C communication with RTC, HDC1080, or other sensors.
- NVS behavior on an ESP32 partition.
- Timing behavior under concurrent firmware tasks.
- Compatibility with the Electron UI beyond the asserted text protocol.

Those require on-device integration tests and, for manufacturing acceptance, physical observations or feedback signals from the PCB.
