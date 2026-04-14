# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- `rtc` crate: DS3231 read/write, POSIX `TZ` via `setenv`/`tzset`, `settimeofday`, SNTP fallback with timeout (ported from standalone bring-up).
- Build-time `TZ_POSIX` / switchboard `default_tz_posix`; NVS key `tz_posix` seeded on boot (with other defaults).

### Changed

- Boot time: **RTC-first** (sane year range on the DS3231) restores system time from the chip; otherwise **Wi‑Fi + SNTP** (60s timeout) writes UTC to the RTC and system clock. Removed the previous **SNTP-first** blocking wait before MQTT.
- Wall-clock strings and daily encoder date rollover use **`chrono::Local`** after `TZ` is set (no longer `SystemTime` + fixed `offset_hours` only for those paths). `offset_hours` remains in NVS for compatibility and motion/NOAA-related constants.
- Encoder-fault housekeeping timestamp and Stepper-only daily reset date now use the same local-time source (`rtc::timezone::local_time`), removing mixed UTC/local date boundaries in runtime paths.

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
