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

Install in this order, and **run steps 2 and 3 from a directory outside this
repository**. `rust-toolchain.toml` pins the `esp` channel, which does not exist
until step 3 — until then every `cargo` command run inside this repository fails
with `custom toolchain 'esp' ... is not installed`, including the `cargo install`
that would have fixed it.

1. **Rust.** Install rustup and cargo from [rustup.rs](https://rustup.rs).
   Check it with `cargo --version` in a *new* shell: rustup edits your shell
   profile, so a terminal that was already open will not see it. If a new shell
   still cannot find it, something later in your profile is overwriting `PATH`
   with an absolute assignment instead of appending to it.

2. **Helper tools.**

   ```powershell
   cargo install espup ldproxy espflash cargo-espflash
   ```

3. **The `esp` toolchain**, via [`espup`](https://github.com/esp-rs/espup):

   ```powershell
   espup install
   ```

   `espup` prints what to do when it finishes. On Unix that means sourcing the
   export file it writes (`~/export-esp.sh`) in every shell you build from; it
   carries environment the build needs beyond `PATH`.

4. **Python 3** on `PATH`.

Confirm with `rustup toolchain list` — `esp` should appear alongside your default
toolchain. The target (`xtensa-esp32s3-espidf`) and linker are already set in
[`.cargo/config.toml`](.cargo/config.toml).

The Janta Installer's **Check Tools** button probes for all of the above and names
the command that fixes each missing piece.

### 2. ESP-IDF

`embuild` installs ESP-IDF v5.2.2 automatically into `.embuild/` on first build.
Expect a multi-gigabyte download and a long first build. `.embuild/` is
gitignored.

**macOS: certificate verification.** ESP-IDF downloads its tools with a bundled
Python. A Python installed from python.org does not use the system trust store, so
the download fails with:

```text
[SSL: CERTIFICATE_VERIFY_FAILED] certificate verify failed: unable to get local issuer certificate
ERROR: Failed to download, and retry count has expired
```

The message names neither Python nor certificates as the thing to fix. Either run
the installer's own one-time fix — `/Applications/Python 3.x/Install
Certificates.command` — or point the build at a CA bundle for that invocation:

```bash
export SSL_CERT_FILE="$(python3 -c 'import certifi; print(certifi.where())')"
```

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

If `DEVICE_ID` is not set in `.env`, it defaults to `1A` and the build looks for
`tower_1A-*` credential files. Pass it explicitly when the cert pair is named for a
different tower:

```powershell
$env:DEVICE_ID = "1"; cargo check
```

`cargo run --release` flashes and opens a monitor, using the runner configured in
`.cargo/config.toml` (`espflash flash --monitor --partition-table=partitions.csv`).

### Reconnecting to a board that is already running this firmware

espflash's default reset drives DTR and RTS, which only reaches the ROM while the
USB Serial/JTAG peripheral is unclaimed. This firmware installs that driver in the
first milliseconds of boot, so on a board already running it the default sequence
fails with:

```text
Error while connecting to device
╰─▶ Failed to connect to the device
```

Pass the sequence meant for this peripheral instead — the runner above and the
Janta Installer both do:

```powershell
espflash flash --before usb-reset ...
```

If a board still refuses, put it in download mode by hand: hold **BOOT**, tap
**RESET**, release **BOOT**, then flash. That path goes through the ROM and does
not depend on what the application is doing.

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

`SAVE_CONFIG` sets a `provisioned` flag in NVS. While it is clear, boot seeds the
site keys from switchboard defaults on every boot, so editing `.env` and
reflashing — or shipping an OTA — updates an unprovisioned tower. Once an
installer has written a configuration over serial, boot leaves those keys alone.
`crates/diagnostics`'s `CONFIG_KEYS` is the single table of which protocol key maps
to which NVS key and how its value is validated.

Passwords are never read back: `GET_CONFIG` reports `<set>` or `<unset>`, so a
technician can confirm a password is configured without a USB cable being able to
read it.

## Serial diagnostics

The board answers a line-oriented command protocol over USB Serial/JTAG at
115200 baud, used by the Janta Installer for provisioning and hardware
verification. The contract lives with the installer, in its
`docs/serial-protocol.md`; the firmware side is `src/diagnostics/serial.rs` and
`src/diagnostics/executor.rs`.

Implemented so far: `PING`, `FIRMWARE_VERSION`, `GET_CAPABILITIES`, `RTC_CHECK`,
`HDC1080_READ`, `GO_HOME`, `REBOOT`, and the configuration set — `SET_ENV`,
`GET_ENV`, `SAVE_CONFIG`, `GET_CONFIG`. The actuator and display commands are
still rejected with an explicit `ERROR ... not enabled in first-wave diagnostics`.

The console is installed at the top of `main`, and both the blocking waits in the
boot path and the long moves pump it, so the board answers throughout boot,
homing, and tracking rather than only from the idle window.

During boot the sensors and the whole configuration set are answered normally —
NVS and the I2C bus exist long before the motion stack. Only `GO_HOME` is refused,
and it says the motion stack is not up rather than blaming Wi-Fi. Mid-move nothing
can be lent out, so only `PING`, `FIRMWARE_VERSION` and `GET_CAPABILITIES` are
served and the rest are refused with `ERROR BUSY <phase>`.

The board also announces `PHASE <name>` whenever what it is doing changes, so the
host can tell a busy board from a wedged one before sending anything, and
`GO_HOME` reports progress every five seconds while searching.

Every command ends in a structured `RESULT <NAME> PASS|FAIL <key=value ...>` line,
and `FIRMWARE_VERSION` announces `PROTOCOL_VERSION 1` so a host knows which dialect
it is talking to. Legacy response lines are still sent alongside, so an installer
that predates this protocol keeps working.

## Behaviour without a network

The tower tracks the sun without Wi-Fi. Time comes from the DS3231 first, and only
telemetry, OTA and MQTT diagnostics need a network — so a failed association is
logged and boot continues. Serial diagnostics are unaffected, which is what makes
it possible to provision a board before the network it will use exists.

The one network-independent thing the tower cannot do without is the clock:
`Rtc::init` panics when the RTC is unreadable *and* Wi-Fi is down, because a
tracker that cannot tell the time would drive the tower to the wrong place. A dead
RTC battery therefore still stops the tower, on purpose.

## Thread stacks

ESP-IDF gives every pthread 3 KB by default, which Rust code doing logging, JSON or
TLS work overruns. `sdkconfig.defaults` raises the main task to 20 KB; the two
threads this firmware spawns — the MQTT event loop and the diagnostics listener —
set their own size through `thread::Builder`. Anything new that spawns a thread and
does more than arithmetic should do the same, or it will fail as a stack overflow
under load rather than at the point of the mistake.

## OTA is currently off

`switchboard::normal().effects.allow_ota` is `false`. With it on, a board that can
reach the network downloads whatever `firmware.jantaus.com` advertises for its
`DEVICE_ID` and reboots into it — which silently replaces anything flashed over USB,
and makes locally built firmware impossible to test once a tower has Wi-Fi
credentials.

**Turn it back on before deploying a tower**, or that tower can never be updated
remotely.

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
