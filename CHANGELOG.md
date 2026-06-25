# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

- Removed extraneous non-code files from certain crates

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
