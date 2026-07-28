# Janta Power — Tower Firmware

Rust firmware for the Janta solar tracking tower. Runs on an ESP32-S3-WROOM-1
(N16R8) and drives a stepper that rotates a panel tower to follow the sun,
reporting telemetry to AWS IoT over MQTT and answering diagnostics over USB
serial.

Built on `esp-idf-svc` with the standard library (not bare-metal). The binary is
`tower`, entry point [`src/runtime/main.rs`](src/runtime/main.rs).

## Hardware

| Function | Pins |
| --- | --- |
| I²C bus, shared at 10 kHz — HDC1080 temp/humidity, DS3231 RTC | SDA 8, SCL 9 |
| Stepper driver | STEP 15, DIR 16 |
| Motor power relay (active-low) | 17 |
| Limit switch (pull-down) | 14 |
| Quadrature encoder | A 10, B 11 |
| Buttons — Maintenance, East, West | 5, 4, 6 |
| RGB status LED (RMT channel 0) | 7 |
| USB Serial/JTAG — logs, diagnostics, flashing | native D+/D− |

The I²C peripherals share one bus through `shared_bus::BusManagerStd`.

## Layout

```text
src/runtime/         Main loop, boot sequence, tracking
src/diagnostics/     Serial and MQTT diagnostics glue for the runtime
src/switchboard.rs   Single source of deployment/default values
motion/, rtc/        Motor control and timekeeping
crates/diagnostics/  Transport-agnostic diagnostics protocol (host-testable)
crates/infrastructure/   clock, wifi, network, ota, serial_console
crates/sensors/      hdc1080, ds323x, bno080, sensors
crates/drivers/      rgb_led, accel-stepper
crates/ui/buttons/   Button debouncing
```

## Build prerequisites

Four things must be in place before this repository will build. Missing any of
them produces an error that does not obviously name the cause, so check all four
first.

### 1. Toolchain

`rust-toolchain.toml` pins the `esp` channel. Install it with
[`espup`](https://github.com/esp-rs/espup):

```powershell
espup install
```

Also required on `PATH`: `ldproxy`, `espflash`, `cargo-espflash`, and Python 3.
The target (`xtensa-esp32s3-espidf`) and linker are already set in
[`.cargo/config.toml`](.cargo/config.toml).

### 2. ESP-IDF

`embuild` installs ESP-IDF v5.2.2 automatically into `.embuild/` on first build.
Expect a multi-gigabyte download and a long first build. `.embuild/` is
gitignored.

### 3. AWS IoT credentials

`crates/infrastructure/network/src/mqtt.rs` embeds three credential files at
compile time. They are gitignored and must be supplied out of band. Drop them in
`crates/infrastructure/network/`:

| File | Source |
| --- | --- |
| `AmazonRootCA1.pem` | Public — Amazon Trust Services root CA |
| `tower_{DEVICE_ID}-certificate.pem.crt` | AWS IoT thing provisioning |
| `tower_{DEVICE_ID}-private.pem.key` | AWS IoT thing provisioning |

`DEVICE_ID` comes from `.env` and defaults to `1A`, so a default build expects
`tower_1A-certificate.pem.crt`. Set `DEVICE_ID` to provision a different tower —
no source edits are needed per tower.

### 4. `.env` (optional but recommended)

`build.rs` generates `constants.rs` from `.env` at compile time; without one it
falls back to hardcoded defaults. Copy the template and edit:

```powershell
Copy-Item .env.example .env
```

`.env.example` documents the motor and mechanical constants, which change
significantly with the motor and gearing — read its notes before changing them.
Note that `build.rs` also carries fallback Wi-Fi credentials as defaults; prefer
setting those in `.env` rather than relying on or editing the fallbacks.

### Windows path length

ESP-IDF's build tooling is bound by the 260-character `MAX_PATH` limit, and
`esp-idf-sys` refuses to build when its output directory exceeds 88 characters.
Two mitigations are needed together from a normal user directory:

- `ESP_IDF_PATH_ISSUES = "warn"` is already set in `.cargo/config.toml`, which
  downgrades the length guard from a hard error to a warning.
- Point the target directory somewhere short, or the build will fail deeper in
  CMake/ninja where the errors are much less legible:

```powershell
$env:CARGO_TARGET_DIR = "C:\jt"
```

Enabling `LongPathsEnabled` in Windows does **not** avoid this — the Xtensa GCC
is a MinGW binary without a `longPathAware` manifest, and CMake enforces its own
`CMAKE_OBJECT_PATH_MAX` of 250 regardless. A short checkout path (e.g.
`C:\janta_power`) is the more portable fix and is worth preferring on CI.

## Build and flash

```powershell
cargo build --release
```

`cargo run --release` flashes and opens a monitor, using the runner configured in
`.cargo/config.toml` (`espflash flash --monitor --partition-table=partitions.csv`).

To erase a board:

```powershell
espflash erase-flash --chip esp32s3 --port COM5
```

The Janta Installer desktop app performs the same flash and erase operations
against a selected copy of this repository, and its **Check Tools** button probes
for this tooling.

## Tests

`crates/diagnostics` is deliberately dependency-free so the protocol layer can be
tested on the host in seconds, without the Xtensa toolchain. The workspace pins
the Xtensa target, so name the host target explicitly:

```powershell
cargo +esp test -p diagnostics --target x86_64-pc-windows-msvc
```

Please keep that crate free of hardware and platform dependencies. Transport
belongs in `serial_console`, board access in the consumer.

## Configuration model

Three layers, in increasing order of runtime authority:

1. **`.env`** → `constants.rs` at build time. Mechanical and site constants.
2. **`src/switchboard.rs`** — policy and defaults, reading from `constants`. The
   single place to change behaviour for a build or an OTA; the app reads only
   from the switchboard, never from `constants` directly.
3. **NVS** — runtime state and provisioned values: heading, encoder snapshot,
   tracking mode, Wi-Fi credentials, tower location.

Be aware that the boot sequence currently rewrites several NVS keys from
switchboard defaults on **every** boot, so values provisioned over serial do not
survive a power cycle. See the roadmap below.

## Serial diagnostics

The board answers a line-oriented command protocol over USB Serial/JTAG at
115200 baud, used by the Janta Installer for provisioning and hardware
verification. The contract lives with the installer, in its
`docs/serial-protocol.md`; the firmware side is `src/diagnostics/serial.rs` and
`src/diagnostics/executor.rs`.

Only five commands are implemented so far. The rest are rejected with an explicit
`ERROR ... not enabled in first-wave diagnostics`.

## Known rough edges

[`DIAGNOSTICS_ROADMAP.md`](DIAGNOSTICS_ROADMAP.md) tracks the planned work on
command coverage, serial availability, and the structured `RESULT` protocol,
including the boot-time NVS rewrite noted above.

One inconsistency worth knowing before you trust a version number: the firmware
version appears in three places with three values — `Cargo.toml` (`1.0.0`),
`switchboard::normal().default_version` (`1.0.4`), and `main.rs DEFAULT_VERSION`
(`1.1.3`). `main.rs` writes its value into NVS unconditionally on boot, so that
is what `FIRMWARE_VERSION` reports and what OTA compares against; the switchboard
field is effectively dead.

## Related

- **Janta Installer** — Electron desktop app for diagnostics, provisioning, and
  flashing over USB serial. Owns the serial protocol contract.
- [`REMOTE_DIAGNOSTICS_PARKING_LOT.md`](REMOTE_DIAGNOSTICS_PARKING_LOT.md) —
  parked ideas for MQTT-based remote diagnostics.
