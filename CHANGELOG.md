# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- **`OLED_TEST [pattern]`** and a `ssd1306` driver. The display is an HS96L03W2C03
  (LCSC C5248080) — 128x64 over I²C at `0x3C`, SSD1306-compatible. Its datasheet
  never names a controller, but the `0x00`/`0x40` control bytes and the
  initialisation sequence identify it beyond doubt.

  No argument walks `ALL_ON`, `CHECKERBOARD`, `STRIPES`, `BORDER`, `OFF`; an
  argument sets one pattern and holds it. Each is chosen for what it can reveal:
  `ALL_ON` finds dead pixels, `OFF` finds one stuck on, `CHECKERBOARD` finds
  addressing faults, `BORDER` proves the edges are reachable.

  The panel acknowledges its address, so a missing display is caught before
  anything is drawn — unlike the status LED, this test is not purely
  operator-judged. Its pattern geometry is host-tested; the crate depends on
  `embedded-hal` alone.

  Works during homing and tracking moves. The I²C bus is a `&'static` manager that
  locks per transaction and motion never touches it, so the display is reachable
  mid-move just as the status LED is — with one restriction: a pattern must be
  named *during a tracking move*. At 10 kHz a full-screen write is about a second
  and the walk is five of them, and the stall and overshoot detectors only run
  between `motor.poll()` calls — so the walk is five seconds of a moving tower going
  unwatched, and is refused with that reason. A homing search allows it, because
  homing switches both detectors off anyway. `init` also stopped clearing, which
  halved the cost of every command.

  The installer walks the patterns one at a time and asks an operator about each,
  with a ten-second countdown per pattern and a skip. The deny option reads "No
  pattern, or damaged" rather than "no pattern": a panel with dead rows or a
  misaligned grid draws *something*, and an operator needs somewhere to put that.

  Note the module's VCC is rated 2.8-3.3 V absolute maximum and the board supplies
  5 V. It responds, which is not the same as being within spec.

- **`LED_TEST <colour>`.** Drives the status LED to one named colour and returns,
  so the host can step through `RED`/`GREEN`/`BLUE`/`WHITE`/`OFF` and ask an
  operator about each. `RESTORE` returns it to the runtime's colour, so a test never
  leaves the board dark.

  Works during homing and tracking moves. The status LED is the one piece of
  hardware that stays lendable while the tower moves — the move holds the motor, the
  caller holds NVS, but nothing holds the LED — so it is passed separately from
  `MotionContext` rather than bundled with it. Bundling was what made `LED_TEST`
  answer `BUSY homing`.

  Deliberately operator-judged. A WS2812 has no readback, so the result reports that
  the colour was written and nothing more. The installer shows a modal with the
  colour under test and a ten-second countdown that records a failure if nobody
  answers — an unattended run must not bank a pass for something no one looked at.

  Also of note: `display_healthy` at main.rs:505 is still the only LED call in the
  runtime, so the LED remains a one-shot "boot got this far" marker. `display_error`,
  `display_warning` and `display_maintenance` are written and unused.

- **`I2C_SCAN`.** Reports which addresses acknowledge on the shared bus, naming the
  expected devices and listing any that are silent. A sensor read failing with
  `NoAcknowledge` says one address did not answer; it cannot say whether the bus is
  alive, which is the first thing worth knowing.

- **`CONFIG_MODE`**, which until now was parsed and then refused as "not enabled in
  first-wave diagnostics" — so the installer's Config Mode test could never pass.

  There is no mode to enter: `SET_ENV` and `SAVE_CONFIG` are accepted whenever NVS
  is lendable. A command that only replied `OK` would therefore be asserting that
  rather than showing it — another diagnostic that cannot fail, which this firmware
  has already produced twice. It instead reads NVS and reports
  `provisioned=yes|no stored=<n> staged=<n> keys=<n>`, so it fails when the
  namespace cannot be opened: the fault that otherwise stays hidden until a
  `SAVE_CONFIG` fifteen commands later reports success and loses the lot. `stored`
  against `staged` also tells a technician whether what they see in the app is
  committed or still waiting on a save.

### Changed

- **Homing no longer blocks the diagnostics that have nothing to do with the
  motor.** `I2C_SCAN`, `HDC1080_READ`, `RTC_CHECK`, `CONFIG_MODE` and the whole
  `SET_ENV`/`SAVE_CONFIG`/`GET_CONFIG` set answered `ERROR BUSY homing` for the
  length of a homing search — which can be tens of minutes, and which happens
  precisely when a technician is stood beside the tower wanting to run them.

  A move holds the motor and only the motor. The I²C bus is a `&'static` manager
  locking per transaction; the status LED belongs to nothing; and NVS is free during
  a homing search, which borrows the motor alone. So the mid-move path now runs the
  same `execute_first_wave_command` the idle path runs, with `motion: None`, instead
  of serving a hand-picked pair of commands.

  What is still refused is refused for a reason: `GO_HOME`, `MOTOR_MOVE` and
  `RELAY_MOTOR` want the motor or its power, and `REBOOT` belongs to the main loop —
  honouring it here would mean resetting from inside the stepping loop with the
  motor relay still closed, or holding the request until a homing sweep ends twenty
  minutes later. Those four still answer `ERROR BUSY <phase>`, so nothing on the
  wire changed; the set that receives it shrank.

  The classification lives in `board_diagnostics::requires_exclusive_runtime` as an
  exhaustive `match` with no wildcard arm, so a command added later stops the crate
  compiling until somebody classifies it. Getting it wrong is otherwise invisible: a
  command that could have run is refused as `BUSY`, which on the wire is
  indistinguishable from a refusal that was meant.

  Two things vary with what the move can spare. A tracking pass and encoder recovery
  hold NVS for their own bookkeeping, so configuration commands refuse there with
  `configuration storage is not lendable during a tracking move` — never `BUSY`,
  which would invite waiting for a move that is followed by another. And a homing
  search may be held for seconds where a tracking move may not — but not for the
  reason it looks like. The tower is far too slow for a pause to cost steps: 25,600
  microsteps per motor revolution behind an 85:1 slew at 200 steps/s² makes a
  one-degree homing move eleven seconds long and never gets past about 3 rpm at the
  motor. What a pause costs is supervision — the stall and overshoot detectors run
  only inside the stepping loop — and homing switches both off for its whole
  duration. Only the full `OLED_TEST` walk is expensive enough to matter:
  `HDC1080_READ` is about 60 ms, `RTC_CHECK` about 10 ms and `I2C_SCAN` about 3 ms,
  against a stall detector that samples every 250 ms.

- **OTA is disabled for bench work** (`switchboard::normal().effects.allow_ota`),
  and `main.rs` now reads that flag instead of a hardcoded `const ALLOW_OTA = true`
  that made it unreachable.

  With OTA on, a board that can reach the network downloads whatever
  `firmware.jantaus.com` advertises for its `DEVICE_ID` and reboots into it. That
  silently replaced USB-flashed firmware the moment a tower was given Wi-Fi
  credentials, so locally built images could not be tested at all — the board kept
  reverting to the published build.

  **Turn this back on before deploying.** A tower shipped with it false can never be
  updated remotely.

  Related and still open: `main.rs` rewrites NVS `version` to the compiled
  `DEFAULT_VERSION` on every boot, discarding what OTA recorded. If a published
  image's own compiled version is not above the version its metadata advertises,
  that re-downloads the same firmware on every boot.

### Fixed

- **Status LED frames were marginal.** `rgb_led` sent T0H 350 ns / T1H 700 ns. A
  WS2812 distinguishes a 1 from a 0 at roughly 500 ns of high time, so 700 ns left
  little margin on a line that is already marginal for another reason: the part
  wants about 3.5 V for a logic high at 5 V supply, and an ESP32 GPIO gives 3.3 V.
  The symptom was colours that were usually right and occasionally scrambled in
  varying ways — a misread bit lands wherever it lands.

  Timings are now chosen to maximise margin either side of that threshold rather
  than to match the datasheet's nominal figures: T0H 300, T0L 850, T1H 850, T1L 450.
  Both are well inside their permitted windows, but a pulse now has to be distorted
  much further before it is misread.

  The direction matters. A 0 misread as a 1 is what makes an LED asked to go dark
  glow faintly instead, and only a *shorter* T0H buys margin against that — an
  interim change to the nominal 400 ns helped the 1 bits and cost the 0s.

  `OFF` is the sensitive case for the same reason: with all 24 bits zero, one
  flipped bit is visible against black, where on any lit colour it is lost among the
  bits already set. A faint tint on `OFF` therefore means roughly one bit error per
  frame, and the remaining fix is electrical — a level shifter, or a series resistor
  on the data line — not firmware.

- **A cleared display kept a lit strip at one edge.** `write_pattern` covers the
  128 columns the panel shows, but controllers in this family carry wider RAM than
  the glass — an SH1106 has 132 columns behind a 128-column panel — so the columns
  past the visible edge were never written and held whatever was drawn there last.
  After the border pattern, that read as a right-hand edge that would not switch
  off.

  `clear()` now writes past the visible width, and `OLED_TEST OFF` routes through
  it. Overrunning is safe only because clearing is idempotent: the column pointer
  wraps within the page and rewrites zeros over zeros. A real pattern must never be
  written that way or the wrap would overwrite its left edge.

- **Refusals were invisible to a strict host.** `serial.rs` wrote `ERROR ...` and
  returned for a command that is not enabled, and for one refused because another
  control command is in flight — with no `RESULT` line. A host running the
  structured protocol resolves on `RESULT` and nothing else, so a refusal the board
  issued in microseconds reached it as an eight-second timeout: the failure that
  explains nothing. Both paths now terminate.

- **Spawned threads had 3 KB of stack.** `sdkconfig.defaults` raises the main task
  to 20 KB with the comment "Rust often needs a bit of an extra main task stack size
  compared to C" — but nothing was ever set for pthread, so both the MQTT event
  thread and the diagnostics listener ran on ESP-IDF's 3 KB default while doing
  logging and JSON work.

  The result was `A stack overflow in task pthread` on the first boot after a flash,
  where `first_boot == 1` adds OTA slot validation and two extra publishes on top of
  the usual traffic. Later boots skip that work, stay under the limit, and look
  fine — which is why it presented as a one-time crash that "fixed itself".

  Both threads now spawn through `thread::Builder` with 8 KB. Sized at the spawn
  site rather than raising `CONFIG_PTHREAD_TASK_STACK_SIZE_DEFAULT`, so the cost
  lands on the two threads that need it instead of every thread in the system.
  Long-standing; it only surfaced once a board got far enough to publish anything.

- **`RTC_CHECK` never read the RTC.** It reported `Local::now()` — the ESP32 system
  clock — so it tested chrono rather than the DS3231, and passed on a board whose
  RTC had lost its time. That is the fault it most needs to catch: an unreadable or
  implausible RTC with no network is what stops the tower booting at all.

  It now reads the DS3231 directly, reports its time as `rtc_utc` alongside the
  system clock, and fails when the read fails or the year falls outside 2024–2100.
  The sanity check runs *before* the legacy `TIME:` line is written, since a host
  predating the structured protocol resolves on that line and would otherwise
  record a pass for a clock about to be called invalid.

- **`HDC1080_READ` could not fail.** Every I2C call in the vendored `hdc1080`
  driver ended in `.unwrap_or_default()`, which discarded the error and left the
  read buffer zeroed. An absent or unwired sensor therefore returned `Ok(0)` for
  its identity registers and `Ok((-40.00, 0.00))` for its measurements — the exact
  values zero raw counts convert to — and the diagnostic reported PASS.

  Errors now propagate. The diagnostic also verifies the identity registers
  (`0x1050` / `0x5449`) and fails by name when they do not match, because a device
  answering something else is not the sensor we think we are reading. And it now
  calls `init()`, which writes the configuration the four-byte sequential read
  path assumes; without it the device stays in its power-on mode where that read
  is not valid.

- **A tower with no reachable Wi-Fi rebooted forever.** `main.rs` ended the
  association step with `.expect("Wi-Fi connection failed")`, and the line after
  it propagated a failed reconnect out of `main`. Either one aborts, and ESP-IDF's
  default panic behaviour is to reboot — so a board on a bench, or on site before
  the network exists, cycled roughly every twenty-five seconds.

  This also made the tower impossible to provision over serial, which is circular:
  the credentials a technician writes with `SET_ENV` are the very ones the board is
  failing to use. Both paths now log and continue, as does a diagnostics MQTT
  listener that will not start.

  Losing the clock is still fatal, deliberately. `Rtc::init` panics when the RTC is
  unreadable *and* Wi-Fi is down, because a tracker that cannot tell the time would
  drive the tower to the wrong place. Wi-Fi alone is not needed to follow the sun:
  time comes from the DS3231 first, and only telemetry and OTA need a network.

- **`CommandState` was private behind a public alias.** `SharedCommandState` is
  `pub type ... = Arc<Mutex<CommandState>>`, but `CommandState` itself was
  private. Older toolchains accepted this; the current `esp` channel
  (rustc 1.97.0-nightly) rejects it as private-in-public and the `tower` binary
  fails to build with 17 errors. The struct is now `pub`; its fields stay
  private, so nothing is exposed that the alias did not already expose.

  Worth knowing before the next toolchain bump on any machine, not just the one
  that hit it.

## [1.1.3] - 2026-04-23

Stable release of Encoder + RTC + AWS with fleet-safe tower identity.

### Added

- MQTT client ID and OTA metadata lookup now derive from `DEVICE_ID`, so
  changing towers no longer requires source edits for `tower_5` / `tower_6`
  identity.

### Fixed

- Sunset homing no longer fires an hour early during DST: the clock crate's
  `sunrise_times` / `sunset_times` now use the local civil date (DST-aware
  via `TZ`) instead of the DS3231's UTC date, which was rolling the lookup
  to "tomorrow" at 19:00 CDT / 18:00 CST.

## [1.1.2] - 2026-04-22

Fleet-identity follow-up to v1.1.1. Fixes the AWS IoT reconnect storms
observed on the Sadler T6 deployment (duplicate MQTT client IDs) and
detaches per-device TLS material from source so the same commit can
flash any tower.

### Fixed

- **MQTT client_id no longer shared across the fleet.** `main.rs` was
  passing the literal `"esp32_thing_001"` to `Mqtt::new_mqtt`, which
  AWS IoT Core treats as a single identity: every time a second tower
  connects, the first one is forcibly disconnected (`errno=119`, EOF
  on the control stream), producing the reconnect loops seen on-site.
  Client ID is now `"tower_5"` per deployment. Still a per-tower source
  edit for now — env-driving it lives on the fleet-rollout cleanup
  list alongside the OTA credentials.
- **Certificate/key filenames no longer hardcoded per tower.** The
  `include_str!` calls in `crates/infrastructure/network/src/mqtt.rs`
  now expand `env!("DEVICE_ID")` at compile time, so the same source
  tree accepts any tower's cert pair. Flashing a new tower becomes:
  drop `tower_{DEVICE_ID}-certificate.pem.crt` +
  `tower_{DEVICE_ID}-private.pem.key` into the crate, bump `DEVICE_ID`
  in `.env`, build.

### Added

- **`crates/infrastructure/network/build.rs`** — loads the repo-root
  `.env`, re-exports `DEVICE_ID` into the crate's compiler env via
  `cargo:rustc-env`, and registers rerun triggers on `.env` plus the
  cert/key files (so cert rotation or a tower swap rebuilds correctly
  without `cargo clean`). Adds `dotenv = "0.15"` as a
  `[build-dependencies]` entry — same version already used by the root
  build script.

### Security

- **`.gitignore` widened from literal names to globs.** Previously only
  files named exactly `device.pem.crt` / `private.pem.key` were
  ignored; any per-device download naming (e.g.
  `tower_5-private.pem.key`) would have slipped through. Now covered
  by `*.pem.crt` / `*.pem.key`, with `AmazonRootCA1.pem` kept as an
  explicit entry. Audited history: only the harmless public
  `fullchain.pem` (Let's Encrypt CA chain, no private material) was
  ever tracked — no key material has been committed.

## [1.1.1] - 2026-04-22

Config-hygiene and homing-reliability follow-up to v1.1.0. No behavioural
changes in the happy path for the Sadler T6 deployment (values already
matched the hardcoded literals being replaced); firmware is safe to flash
over v1.1.0 without recommissioning.

### Fixed

- **Tower coordinates now actually honour `.env`.** `main.rs` was
  hardcoding `tower_latitude = 33.75944` and `tower_longitude = -96.82722`,
  silently ignoring `TOWER_LATITUDE` / `TOWER_LONGITUDE` from `.env`
  (despite `build.rs` generating the constants and the `Switchboard`
  carrying the fields). NOAA sun-angle math now reads from
  `sw.default_tower_latitude` / `sw.default_tower_longitude`, so changing
  `.env` and reflashing actually moves the sun.
- **CCW limit-switch search radius expanded from 10° to 350°.** Boot /
  recovery homing (`motion/src/motion/homing.rs::find_limit_switch_ccw`)
  previously gave up after sweeping only 10° CCW; if the switch wasn't
  within that narrow window the device would fall through to the
  critical-error path. The sweep now covers almost a full revolution, so
  the switch is found on any valid tower orientation.

### Removed

- **`TIMEZONE_OFFSET_HOURS` env var and all downstream plumbing** — the
  build constants (`TIMEZONE_OFFSET_HOURS`, `TIMEZONE_OFFSET_HOURS_I32`),
  the `Switchboard::default_tz_offset_hours` field, and the NVS write of
  `"offset_hours"` are gone. No code reads them anywhere; `TZ_POSIX` +
  `rtc::timezone::local_time()` / `chrono::Local` own timezone conversion
  end-to-end (and do so DST-aware, unlike the fixed integer offset).
- **`TOWER_ID` env var and all plumbing** — build const, the
  `Switchboard::default_tower_id` field, and the hardcoded `tower_id`
  literal in `main.rs` are gone. The value only ever surfaced in a single
  boot `info!(…)` line; `DEVICE_ID` is now the single source of tower
  identity (it's what MQTT topics route through — `tower/{DEVICE_ID}/…`),
  and the boot log now prints `sw.device_id` instead.

## [1.1.0] - 2026-04-21

Major telemetry rework aligning the device with AWS IoT Core topic conventions
and structured JSON payloads, plus RTC-backed local time for every runtime path.

### Added

- **`network::telemetry` module** — single source of truth for MQTT topic
  builders (`topic::status`, `topic::logs_{boot,firmware_update,error,warning,info}`,
  `topic::data_{angle,encoder_error_ticks}`, `topic::component_status`),
  structured payload structs (`Heartbeat`, `BootLog`, `FirmwareUpdateLog`,
  `ErrorLog`, `WarningLog`, `InfoLog`, `Angle`, `EncoderErrorTicks`,
  `ComponentStatus`), a generic `publish_json` helper, and a convenience
  `publish_error` for the common `ErrorLog` case.
- **`Component` enum** (`motor`, `encoder`, `light_sensor`, `limit_switch`,
  `system`) attached to every log / component-status payload.
- **`Severity` enum** (`online`, `warning`, `fault`) for the
  `tower/{id}/component/*/status` topic.
- **`EncoderErrorCategory` enum** (`acceptable`, `undershoot`, `overshoot`)
  plus a human-readable `result` string on `tower/{id}/data/encoder_error_ticks`
  payloads; drift is categorised against the new build-time constant
  `HOME_ERROR_ACCEPTABLE_DEG` (2.5° default).
- **`infra::error_loop(device_id, mqtt, component, message, notes)`** —
  republishes a structured `ErrorLog` every 15 minutes until the device is
  reset; replaces `Telemetry::critical_failure_loop`.
- **`Motion::report_home_error_ticks`** — helper that consumes the stashed
  sunset-homing drift, classifies it against the threshold, and publishes a
  structured payload to `tower/{id}/data/encoder_error_ticks`.
- **Optional `value` / `unit` fields** on `ErrorLog` and `WarningLog` (skipped
  from JSON when `None`) so numeric errors carry their reading alongside the
  message.
- **`rtc` crate**: DS3231 read/write, POSIX `TZ` via `setenv`/`tzset`,
  `settimeofday`, SNTP fallback with timeout (ported from standalone bring-up).
- **Build-time `TZ_POSIX` / switchboard `default_tz_posix`**; NVS key `tz_posix`
  seeded on boot with other defaults.

### Changed

- **All MQTT topics migrated to the AWS IoT Core convention**
  (`tower/{device_id}/...`):
  - `{id}/tower/status` heartbeat → `tower/{id}/status`.
  - Boot log → `tower/{id}/logs/boot` (JSON `BootLog`).
  - Firmware update success/failure → `tower/{id}/logs/firmware_update`
    (JSON `FirmwareUpdateLog`).
  - Critical failures (10 call sites: boot homing, re-home-after-recovery,
    sunset/sleep homing, encoder probe exhaustion, switchboard misconfig) →
    `tower/{id}/logs/error` (JSON `ErrorLog`) with `component` tagging.
  - Tracking heading → `tower/{id}/data/angle` (JSON `Angle`).
  - End-of-day encoder drift → `tower/{id}/data/encoder_error_ticks`
    (JSON `EncoderErrorTicks` with `category` + `result`).
- **Payloads are now structured JSON** across every topic; the previous raw
  byte / plain-text bodies (e.g. `"Critical failure: Limit switch failure at
  the Office Tower!"`) are gone. Downstream automation should parse JSON, not
  match on body substrings.
- **Boot time**: **RTC-first** (sane year range on the DS3231) restores system
  time from the chip; otherwise **Wi-Fi + SNTP** (60s timeout) writes UTC to
  the RTC and system clock. Removed the previous **SNTP-first** blocking wait
  before MQTT.
- **Local-time consistency**: wall-clock strings, daily encoder date rollover,
  encoder-fault housekeeping timestamp, and `clock` crate
  `after_sunrise`/`after_sunset` all now use the same `rtc::timezone::local_time()`
  (or `chrono::Local` after `TZ` is set) source. No more mixed UTC/local
  boundaries across runtime paths.
- **Motion NOAA sun-angle timezone** now uses runtime local UTC offset
  (DST-aware via `TZ` / `tzset`) instead of the fixed build-time
  `TIMEZONE_OFFSET_HOURS` constant.
- **`Component::HomingSensor` renamed to `Component::LimitSwitch`** (wire
  value: `"limit_switch"`) to match the physical hardware naming.
- **Ordering invariant documented** on `Motion::force_zero_if_limit_switch_pressed`
  and `poll_limit_switch_zeroing`: the drift-capture step must run before the
  encoder re-zero or the end-of-day metric is silently destroyed.

### Removed

- `PUBLISH_MQTT` publish gate and every `if publish_mqtt { ... }` check
  threaded through `boot_diagnostic`, `encoder_fault::tick`,
  `tracking_loop::tick`, `Motion::set_tower_position`,
  `housekeeping`, and `switch_to_stepper_only_daily`. Will return behind a
  future admin mode.
- `Telemetry::critical_failure_loop`, the `Telemetry` struct, and
  `infra::topic()` — replaced by `infra::error_loop` + `network::telemetry`.
- `infra::Telemetry::publish_{status_heartbeat,boot_log,firmware_update_success,firmware_update_failure}_if`
  helpers — call sites now use `publish_json` directly with the shared payload
  structs.
- Raw byte-string critical-alert constants: `CRITICAL_TOWER_LIMIT_SWITCH`,
  `CRITICAL_FAILURE_REHOME_AFTER_ENCODER_RECOVERY`.
- Hardcoded `event_type` field from every log payload (was always
  topic-determined; redundant with the destination topic).
- **Dead MQTT username / password plumbing** — AWS IoT Core authenticates via
  TLS client certificates, so `MQTT_USER` / `MQTT_PASSWORD` env vars, the
  NVS `"mqtt_user"` / `"mqtt_pass"` keys, and the `Switchboard` fields
  `mqtt_broker_url`, `mqtt_client_id`, `default_mqtt_user`,
  `default_mqtt_pass` (and their initialisers) were all deleted.
- `src/runtime/infra/telemetry.rs` downsized from a mixed topic + publisher +
  loop module to a single `error_loop` helper.

### Fixed

- Structured log payloads now carry `current_time` and, where relevant,
  `firmware_version`, on every event — previously some paths emitted bodies
  without either.

### Known limitations

- **OTA HTTP credentials** for the updater are still duplicated between
  `switchboard` defaults and call sites (e.g. `motion`); change both or
  centralise to avoid drift.
- **MQTT client ID** is still the hardcoded `"esp32_thing_001"` in `main.rs`;
  deploying more than one tower with the same firmware will cause AWS IoT to
  disconnect the previous session on each new connect. Must be made
  per-device before fleet deploy.
- **Broker URL** is still hardcoded in `main.rs`; should move to `.env` /
  switchboard.
- **OTA security**: firmware image signature verification is still not
  implemented.
- **NVS boot writes**: when persistence is enabled, Wi-Fi and timezone keys
  are overwritten from firmware defaults each boot — field-tuned values will
  not survive unless policy is changed.
- **Version string triplicate**: the current firmware version is hardcoded in
  three places (`main.rs` `DEFAULT_VERSION`, `main.rs` unconditional NVS
  write, `switchboard::default_version`) and they disagree. OTA version
  comparison may report stale values until consolidated to
  `env!("CARGO_PKG_VERSION")`.

### Notes for operators

- Every topic is now `tower/{device_id}/...`. Topic prefix is sourced from
  the `DEVICE_ID` env var at build time; each physical tower must be flashed
  with its own value.
- Every payload is JSON; route AWS rules on payload fields
  (`component`, `category`, etc.) rather than topic substrings.
- `tower/{id}/logs/error` receives a republish every 15 minutes while the
  device is wedged on a critical failure; route alerts accordingly to avoid
  paging storms (e.g. aggregate over 30-minute windows).

## [1.0.0] - 2026-04-08

First stable release of the **tower** ESP32-S3 firmware: field-deployable baseline for sun tracking, motion control, and cloud telemetry.

### Added

- Solar tracking with configurable deadband, soft limits, and stall / encoder overshoot guardrails (`EncoderGuarded` vs `StepperOnly`).
- Limit-switch homing (CCW search), heading and encoder snapshot persistence in NVS.
- Encoder fault recovery (probes, drift handling) with daily Stepper-only fallback and MQTT alerts on `{MQTT_USER}/tower/status`.
- MQTT telemetry: firmware version, boot check, tracking data topic, OTA-related status; build-time `.env` constants via `build.rs`.
- Wi-Fi and MQTT client (TLS) with credentials seeded from NVS (defaults from build / switchboard).
- OTA version compare and update flow (infrastructure crate).

### Known limitations

- **MQTT connection**: Broker URL and MQTT client ID are still hardcoded in `main.rs` while `Switchboard` also carries `mqtt_broker_url` / `mqtt_client_id`—they must be kept in sync manually until wired to one source.
- **OTA HTTP credentials** for the updater are duplicated between `switchboard` defaults and call sites (e.g. `motion`); change both or centralize to avoid drift.
- **NVS boot writes**: When persistence is enabled, selected keys (MQTT, Wi-Fi, timezone, etc.) are overwritten from firmware defaults each boot—field-tuned values in NVS will not survive unless policy is changed.
- **OTA security**: Firmware image signature verification is not implemented; treat OTA as trusted-network / trusted-artifact until signing is added.
- **Tooling**: No CI workflow in-repo yet; validate with `cargo check` / project flash workflow before tagging releases.

### Notes for operators

- Critical alerts use topic `{MQTT_USER}/tower/status` with **payload-dependent** meaning; automation should display or route on the **full message body**, not a single hardcoded title.
- Configure per-device identity and secrets via **`.env`** at build time (`MQTT_USER`, `MQTT_PASSWORD`, etc.); see `.env.example`.
