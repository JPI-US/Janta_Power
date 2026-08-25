use std::time::Duration;
use chrono::{DateTime, Local};
use clock::Clock;
use log::{error, info, warn};
use std::thread;

#[path = "../config.rs"]
mod config;
#[path = "../constants.rs"]
mod constants;
#[path = "../switchboard.rs"]
mod switchboard;
mod infra;
mod app;
#[path = "../diagnostics/mod.rs"]
mod diagnostics;

/// Firmware version reported by `FIRMWARE_VERSION` and compared against by OTA.
///
/// At module scope because the boot waits answer `FIRMWARE_VERSION` before NVS
/// has been read; PHASE 3 writes this same value into NVS on every boot, so the
/// early answer and the later one are the same string.
const DEFAULT_VERSION: &str = "1.1.3";

// Provide __pender function for embassy_executor
// This function is called by embassy_executor to wake tasks
#[no_mangle]
pub extern "C" fn __pender() {
    // Intentionally empty.
}

use esp_idf_svc::{
    eventloop::EspSystemEventLoop,
    hal::{
        gpio::PinDriver,
        i2c::{I2cConfig, I2cDriver},
        prelude::*,
    },
    log::EspLogger,
    nvs::{EspDefaultNvsPartition, EspNvs},
    ota::EspOta,
};
use rtc::Rtc;
use motion::{MoveOutcome, Motion, MotionMode};
use rgb_led::Led;
use network::mqtt::Mqtt;
use ota::OtaUpdater;
use semver::Version;
use wifi::wifi::{Wifi, WifiState};

use crate::app::encoder_fault::{Direction, EncoderRecoverySwitches};

fn main() -> anyhow::Result<()> {
    let sw = switchboard::normal();
    const MQTT_BROKER_URL: &str = "mqttS://a2exykcl6t998u-ats.iot.us-east-1.amazonaws.com:8883";

    // PHASE 1: INITIALIZATION --------------------------------------------------
    esp_idf_svc::sys::link_patches();

    // Logger and event loop
    EspLogger::initialize_default();

    // Serial diagnostics uses the USB/JTAG console but stays on the main loop so
    // the runtime keeps single ownership of motion, NVS, and shared I2C.
    //
    // Constructed here, ahead of everything that can block: installing the driver
    // depends on nothing, and leaving it until after Wi-Fi, SNTP, MQTT, and homing
    // meant the peripheral did not exist for the first minutes of boot. The waits
    // in that stretch now pump it through `sleep_answering_boot_diagnostics`.
    let mut serial_diagnostics = diagnostics::serial::SerialDiagnosticsRuntime::new()?;

    let sysloop = EspSystemEventLoop::take()?;

    // Hardware and persistent storage
    let peripherals = Peripherals::take().unwrap();
    let nvs_default = EspDefaultNvsPartition::take()?;

    let mut nvs = match EspNvs::new(nvs_default.clone(), "storage", true) {
        Ok(nvs) => {
            info!("Got namespace {:?} from default partition", "storage");
            nvs
        }
        Err(e) => panic!("Could't get namespace {:?}", e),
    };

    let last_run_normal = infra::SnapshotStore::new(&mut nvs, true)
        .load_last_run_normal_or_init(true);
    let trust_nvs_state = last_run_normal;
    info!(
        "Last run normal={} -> trust_nvs_state={} (active_mode=Normal)",
        last_run_normal, trust_nvs_state
    );

    // Encoder pins
    let encoder_a = peripherals.pins.gpio10;
    let encoder_b = peripherals.pins.gpio11;

    // I2C bus for Clock
    let sda = peripherals.pins.gpio8;
    let scl = peripherals.pins.gpio9;
    let config = I2cConfig::new().baudrate(10_u32.kHz().into());
    let i2c = I2cDriver::new(peripherals.i2c0, sda, scl, &config).unwrap();
    let bus: &'static _ = shared_bus::new_std!(I2cDriver = i2c).unwrap();

    // Declared here, alongside the bus and NVS, because the boot waits below can
    // already serve sensor and configuration commands and need both.
    //
    // SET_ENV stages into this buffer and SAVE_CONFIG commits it, so it has to
    // outlive any single command — the installer sends fifteen SET_ENVs before the
    // save.
    let mut config_staging = board_diagnostics::ConfigStaging::new();
    let diagnostics_command_state = diagnostics::mqtt::new_shared_command_state();

    // PHASE 2: NETWORK SETUP ---------------------------------------------------

    const PERSIST_NVS: bool = true;

    // A tower that SAVE_CONFIG has provisioned keeps its site configuration. Until
    // then the switchboard defaults are rewritten on every boot, so editing `.env`
    // and reflashing — or shipping an OTA — still updates an unprovisioned tower.
    //
    // Without this gate the next power cycle silently discards everything a
    // technician just wrote over serial, which made the whole Customer Data panel
    // theatre.
    let provisioned = nvs
        .get_u8(board_diagnostics::NVS_KEY_PROVISIONED)
        .ok()
        .flatten()
        .unwrap_or(0)
        == 1;
    let seed_defaults = PERSIST_NVS && !provisioned;
    if provisioned {
        info!("Tower is provisioned: leaving site configuration in NVS untouched");
    }

    if seed_defaults {
        match nvs.set_str("wifi_ssid", sw.default_wifi_ssid) {
            Ok(_) => info!("Wifi ssid updated"),
            Err(e) => error!("Wifi ssid not updated {:?}", e),
        };
    }
    if seed_defaults {
        match nvs.set_str("wifi_pass", sw.default_wifi_pass) {
            Ok(_) => info!("Wifi password updated"),
            Err(e) => error!("Wifi password not updated {:?}", e),
        };
    }

    if seed_defaults {
        match nvs.set_str("tz_posix", sw.default_tz_posix) {
            Ok(_) => info!("POSIX TZ string has been updated"),
            Err(e) => error!("tz_posix was not updated {:?}", e),
        };
    }

    // Wi-Fi
    let mut buffer = [0u8; 64];
    let real_wifi_ssid = nvs
        .get_str("wifi_ssid", &mut buffer)?
        .expect("Wifi ssid not found")
        .to_string();
    let real_wifi_pass = nvs
        .get_str("wifi_pass", &mut buffer)?
        .expect("Wifi password not found")
        .to_string();

    let mut wifi = Wifi::new(peripherals.modem, sysloop.clone(), nvs_default)?;
    // Read from the switchboard rather than hardcoded: on a board that cannot get
    // past `Rtc::init` — no clock and no network — this wait is the only stretch
    // that serves diagnostics, so being able to widen it without editing the boot
    // path is worth something.
    log::info!(
        "Waiting for {} seconds before connecting to wifi",
        sw.wifi_connect_delay_secs
    );
    sleep_answering_boot_diagnostics(
        Duration::from_secs(sw.wifi_connect_delay_secs),
        &mut BootServices {
            serial: &mut serial_diagnostics,
            command_state: &diagnostics_command_state,
            nvs: &mut nvs,
            staging: &mut config_staging,
            bus,
            firmware_version: DEFAULT_VERSION,
            led: None,
        },
        "waiting to connect Wi-Fi",
    );
    // Wi-Fi is not required to track the sun. Time comes from the DS3231 first
    // (see `Rtc::init`); only telemetry and OTA need a network. So failing to
    // associate must not take the tower down with it.
    //
    // This used to `.expect(...)`, which meant a board with no reachable network
    // panicked and rebooted roughly every twenty-five seconds — and that made the
    // tower impossible to provision over serial, because the credentials a
    // technician is trying to write are the very ones it is failing to use.
    //
    // Losing the clock is still fatal, deliberately: `Rtc::init` panics when the
    // RTC is unreadable *and* Wi-Fi is down, because a tracker that cannot tell
    // the time would drive the tower to the wrong place.
    // Announced before the call, not during it: association blocks inside the
    // driver, so the serial link genuinely cannot be serviced while it runs. The
    // phase at least tells a connected host why it has gone quiet.
    serial_diagnostics.announce_phase("connecting Wi-Fi");
    let associated = match wifi.connect(&real_wifi_ssid, &real_wifi_pass) {
        Ok(()) => true,
        Err(err) => {
            error!("Wi-Fi connection failed: {:?}. Continuing without a network.", err);
            false
        }
    };
    info!("Current wifi state: {:?}", wifi.state());

    // Only worth retrying if the first attempt got somewhere. `reconnect_if_disconnected`
    // blocks for another ten seconds, and on a board with no reachable network that is
    // ten more seconds of silence on the serial link for no possible gain.
    if associated && wifi.state() == WifiState::Disconnected {
        if let Err(err) = wifi.reconnect_if_disconnected() {
            error!("Wi-Fi reconnect failed: {:?}. Continuing without a network.", err);
        }
    }

    // Time: RTC-first, SNTP fallback (see `rtc::Rtc::init`).
    // If true, never trust DS3231 on this boot — always SNTP then write RTC (debug / bad battery).
    const FORCE_NTP_SKIP_RTC: bool = false;
    let mut tz_buf = [0u8; 96];
    let tz_posix_str = nvs
        .get_str("tz_posix", &mut tz_buf)?
        .unwrap_or(sw.default_tz_posix);
    serial_diagnostics.announce_phase("setting the clock");
    {
        let mut rtc = Rtc::new(bus);
        rtc.init(&wifi, tz_posix_str, FORCE_NTP_SKIP_RTC);
    }
    let local_time_boot = rtc::timezone::local_time();
    let formatted_time = format!("{}", local_time_boot.format("%d/%m/%Y %H:%M:%S"));
    info!("{}", formatted_time);

    // MQTT
    // AWS IoT Core uses TLS client certificates baked into the firmware for
    // authentication; no username/password plumbing is required at runtime.
    // Broker URL is still hardcoded here; client ID is derived from DEVICE_ID so
    // fleet identity stays in `.env` with the MQTT topic/cert identity.
    serial_diagnostics.announce_phase("starting MQTT");
    let mqtt_client_id = "esp32_thing_001";
    let mut mqtt = Box::new(Mqtt::new_mqtt(MQTT_BROKER_URL, mqtt_client_id)?);

    // PHASE 3: BOOT VALIDATION -------------------------------------------------
    let first_boot = nvs.get_u8("first_boot")?.unwrap_or(1);

    const ALLOW_BOOT_VALIDATION: bool = true;

    // Load firmware version early so the boot-log publish can include it.
    let mut version_buf = [0u8; 32];
    if PERSIST_NVS {
        nvs.set_str("version", DEFAULT_VERSION)?;
    }
    let current_version: Version = nvs
        .get_str("version", &mut version_buf)?
        .map(|s| s.trim().parse::<Version>())
        .transpose()?
        .unwrap_or_else(|| Version::parse(DEFAULT_VERSION).unwrap());
    let current_version_string = current_version.to_string();

    // Diagnostics phases 1 and 2 keep the main loop as the single owner of
    // hardware/NVS. A background listener answers read-only requests from this
    // cached snapshot, and queued control commands are handed back to the main loop.
    let diagnostics_snapshot = diagnostics::mqtt::new_shared_snapshot(
        diagnostics::mqtt::OwnedStatusSnapshot {
            device_id: sw.device_id.to_string(),
            firmware_version: current_version_string.clone(),
            mqtt_connected: mqtt.is_connected(),
            wifi_connected: matches!(wifi.state(), WifiState::Connected(_)),
            motion_mode: String::from("booting"),
            current_heading: sw.home_heading_deg,
            activity: String::from("booting"),
            activity_reason: String::from("Tower is booting and initializing subsystems"),
            sun_angle: None,
            target_heading: None,
            angle_offset: None,
            last_move_outcome: None,
        },
    );
    // Phase 2 adds a bounded handoff queue for diagnostics commands that must
    // execute on the main loop because they mutate live control state.
    let (diagnostics_control_tx, diagnostics_control_rx) =
        diagnostics::mqtt::new_control_channel(8);

    // Boot diagnostics: Wi-Fi + MQTT
    let boot_diagnostic_result = if ALLOW_BOOT_VALIDATION {
        boot_diagnostic(
            sw.device_id,
            &mut wifi,
            &mut mqtt,
            &current_version,
            &mut BootServices {
            serial: &mut serial_diagnostics,
            command_state: &diagnostics_command_state,
            nvs: &mut nvs,
            staging: &mut config_staging,
            bus,
            firmware_version: &current_version_string,
            led: None,
        },
        )
    } else {
        info!("Boot validation disabled");
        true
    };

    // On first OTA boot, mark slot valid only after diagnostics.
    if ALLOW_BOOT_VALIDATION && first_boot == 1 {
        info!("First boot, now performing boot diagnostics");
        let mut valid_ota = EspOta::new().expect("Failed to get OTA instance");
        let running_slot = valid_ota.get_running_slot();
        info!("This is the running boot slot {:?}", running_slot);

        if running_slot.unwrap().label == "factory" {
            info!("Running from factory partition -> skipping OTA validity marking");
            nvs.set_u8("first_boot", 0)?;
        } else {
            if boot_diagnostic_result {
                info!("Boot validation passed, now marking firmware as valid");
                valid_ota.mark_running_slot_valid()?;
                nvs.set_u8("first_boot", 0)?;

                // If a `prev_version` was stashed by the OTA path, we just
                // successfully booted into the new firmware. Publish the
                // `logs/firmware_update` success event and only clear the
                // stash on publish success (so a transient publish failure
                // doesn't lose the signal).
                let mut prev_ver_buf = [0u8; 32];
                if let Some(prev_str) = nvs.get_str("prev_version", &mut prev_ver_buf)? {
                    match prev_str.trim().parse::<Version>() {
                        Ok(prev_version) => {
                            let current_time = rtc::timezone::local_time()
                                .format(network::telemetry::TIME_FORMAT)
                                .to_string();
                            let payload = network::telemetry::FirmwareUpdateLog {
                                current_time: &current_time,
                                message: "Firmware successfully updated",
                                previous_version: &prev_version.to_string(),
                                current_version: &current_version.to_string(),
                                notes: "No errors during update",
                            };
                            let topic = network::telemetry::topic::logs_firmware_update(sw.device_id);
                            let published =
                                network::telemetry::publish_json(&mut mqtt, &topic, &payload).is_ok();
                            if published {
                                let _ = nvs.remove("prev_version");
                            }
                        }
                        Err(e) => {
                            warn!(
                                "prev_version in NVS is not valid semver ({:?}), clearing: {:?}",
                                prev_str, e
                            );
                            let _ = nvs.remove("prev_version");
                        }
                    }
                }
            } else {
                error!("Boot validation failed, rolling back firmware");
                valid_ota.mark_running_slot_invalid_and_reboot();
            }
        }
    } else {
        info!("Normal boot firmware already validated");
    }

    // Run diagnostics intake on its own MQTT client so commands are not delayed
    // behind the long tracking loop sleep. This thread reads the shared snapshot
    // directly and enqueues any hardware-affecting command back to the main loop.
    let diagnostics_mqtt_client_id = format!("{}_diagnostics", mqtt_client_id);
    // Remote diagnostics are optional. With no network there is nothing to listen
    // to, and serial diagnostics are unaffected — so a listener that will not start
    // must not stop the tower from tracking. The control queue's receiver reports
    // the closed channel once and carries on.
    if let Err(err) = diagnostics::mqtt::spawn_listener(
        MQTT_BROKER_URL,
        &diagnostics_mqtt_client_id,
        sw.device_id,
        diagnostics_snapshot.clone(),
        diagnostics_command_state.clone(),
        diagnostics_control_tx,
    ) {
        error!(
            "Diagnostics MQTT listener not started: {:?}. Serial diagnostics remain available.",
            err
        );
    }

    // PHASE 4: FIRMWARE VERSION AND OTA ---------------------------------------
    // (current_version was loaded earlier, before boot_diagnostic, so the
    // boot-log publish could include `firmware_version`.)

    // Read from the switchboard rather than hardcoded. `sw.effects.allow_ota` has
    // existed all along and was never wired up, so there was no way to turn OTA off
    // for bench work short of editing this line — and with OTA on, a board that can
    // reach the network replaces whatever was just flashed onto it.
    let allow_ota = sw.effects.allow_ota;

    {
        let payload = network::telemetry::Heartbeat {
            current_time: &formatted_time,
            firmware_version: &current_version.to_string(),
        };
        let topic = network::telemetry::topic::status(sw.device_id);
        let _ = network::telemetry::publish_json(&mut mqtt, &topic, &payload);
    }

    let mut updater = OtaUpdater::new_ota(
        current_version.clone(),
        &mut mqtt,
        sw.device_id,
        Some(sw.default_ota_updater),
        Some(sw.default_ota_password),
    ).expect("Failed to create OTA updater instance");

    if allow_ota {
        info!("Checking for new OTA update in 3 seconds...");
        sleep_answering_boot_diagnostics(
            Duration::from_secs(3),
            &mut BootServices {
            serial: &mut serial_diagnostics,
            command_state: &diagnostics_command_state,
            nvs: &mut nvs,
            staging: &mut config_staging,
            bus,
            firmware_version: &current_version_string,
            led: None,
        },
            "checking for OTA update",
        );
        if let Err(e) = updater.run_version_compare(&mut nvs) {
            let current_time = rtc::timezone::local_time()
                .format(network::telemetry::TIME_FORMAT)
                .to_string();
            let version_str = current_version.to_string();
            let payload = network::telemetry::FirmwareUpdateLog {
                current_time: &current_time,
                message: "Firmware update unsuccessful",
                previous_version: &version_str,
                current_version: &version_str,
                notes: "Did not update due to failures",
            };
            let topic = network::telemetry::topic::logs_firmware_update(sw.device_id);
            let _ = network::telemetry::publish_json(&mut mqtt, &topic, &payload);
            error!("Version compare failed: {:?}", e);
        } else {
            info!("Version compare succeeded");
        }
    } else {
        info!("OTA disabled: skipping version compare");
    }
    
    // PHASE 5: MOTION INITIALIZATION ------------------------------------------
    // Tower location — seeded from `TOWER_LATITUDE` / `TOWER_LONGITUDE` in `.env`
    // via `Switchboard`, but only until the tower is provisioned. On an
    // unprovisioned tower, editing `.env` and reflashing still updates the
    // coordinates; once an installer has written them over serial, these are left
    // alone.
    let tower_latitude: f64 = sw.default_tower_latitude;
    if seed_defaults {
        match nvs.set_str("tower_latitude", &tower_latitude.to_string()) {
            Ok(_) => info!("Tower latitude has been updated"),
            Err(e) => error!("Tower latitude was not updated {:?}", e),
        };
    }

    let tower_longitude: f64 = sw.default_tower_longitude;
    if seed_defaults {
        match nvs.set_str("tower_longitude", &tower_longitude.to_string()) {
            Ok(_) => info!("Tower longitude has been updated"),
            Err(e) => error!("Tower longitude was not updated {:?}", e),
        };
    }

    // Tower location from NVS
    let latitude = nvs
        .get_str("tower_latitude", &mut buffer)?
        .unwrap_or("0")
        .parse()
        .unwrap_or(0.0);
    let longitude = nvs
        .get_str("tower_longitude", &mut buffer)?
        .unwrap_or("0")
        .parse()
        .unwrap_or(0.0);
    let altitude: f64 = 0.0;

    info!("Retrieved latitude: {}, and longitude: {}", latitude, longitude);
    info!("Device: {}, Lat: {}, Lon: {}, Alt: {}", sw.device_id, latitude, longitude, altitude);

    // Hardware initialization
    let mut calculation = Clock::new(bus.acquire_i2c(), latitude, longitude, altitude);

    // LED status
    let mut led = Led::new(peripherals.pins.gpio7, peripherals.rmt.channel0).unwrap();

    // Motion (motor, encoder, relay, limit switch)
    let mut motion = Motion::new(
        peripherals.pins.gpio15,
        peripherals.pins.gpio16,
        peripherals.pins.gpio17,
        peripherals.pins.gpio14,
        encoder_a,
        encoder_b,
    );

    motion.init();
    led.display_healthy();
    let _ = motion.run();

    // Runtime guardrails from switchboard
    motion.set_stall_detection_enabled(sw.runtime.guardrails.stall_detection_enabled);
    motion.set_soft_limits(
        sw.runtime.guardrails.soft_limits_enabled,
        sw.runtime.guardrails.soft_limit_min_deg,
        sw.runtime.guardrails.soft_limit_max_deg,
    );

    const POWER_ON: bool = true;

    // Daily encoder mode reset before mode load
    check_daily_encoder_reset(&mut nvs, &rtc::timezone::local_time(), PERSIST_NVS);

    // Motion mode from NVS, default EncoderGuarded
    let mut motion_mode = infra::SnapshotStore::new(&mut nvs, PERSIST_NVS)
        .load_tracking_mode_or_init(MotionMode::EncoderGuarded);
    motion.set_motion_mode(motion_mode);
    motion.set_motor_power_on(POWER_ON);
    info!(
        "Motion mode: {:?}",
        match motion_mode {
            MotionMode::StepperOnly => "StepperOnly",
            MotionMode::EncoderGuarded => "EncoderGuarded",
        }
    );

    // PHASE 6: STATE RESTORATION ----------------------------------------------
    let mut actual_heading: f32 = if trust_nvs_state {
        let h = infra::SnapshotStore::new(&mut nvs, PERSIST_NVS)
            .load_heading_or_init(sw.home_heading_deg);
        info!("Restored heading from NVS: {}", h);
        h
    } else {
        info!("Skipping heading restore: NVS state untrusted");
        sw.home_heading_deg
    };

    // Restore encoder snapshot only in EncoderGuarded mode.
    let mut restored_from_snapshot = false;
    if trust_nvs_state && motion_mode == MotionMode::EncoderGuarded {
        if let Some(enc_ticks_adj) =
            infra::SnapshotStore::new(&mut nvs, PERSIST_NVS)
                .load_encoder_snapshot()
        {
            // Restore zero offset so adjusted ticks equal saved snapshot.
            let raw = motion.encoder_ticks_raw();
            motion.set_encoder_zero_offset(raw - enc_ticks_adj);
            info!("Restored encoder snapshot ticks from NVS: {}", enc_ticks_adj);
            restored_from_snapshot = true;
        } else {
            info!("No valid encoder snapshot found in NVS; will home normally.");
        }
    } else {
        if !trust_nvs_state {
            info!("Skipping encoder snapshot restore: NVS state untrusted");
        } else {
            info!("Motion mode is StepperOnly: skipping encoder snapshot restore");
        }
    }

    // Keep motion state aligned with restored heading.
    motion.update_position(actual_heading);

    // PHASE 7: NORMAL MODE BOOT ACTIONS ---------------------------------------

    // Encoder fault recovery
    let mut encoder_fault = app::encoder_fault::EncoderFaultRecovery::new();
    let encoder_daily_mode = infra::SnapshotStore::new(&mut nvs, PERSIST_NVS).load_encoder_daily_mode();
    encoder_fault.set_mode_switched_daily(encoder_daily_mode);

    // Button inputs (reserved for manual control)
    let _mb = PinDriver::input(peripherals.pins.gpio5).unwrap(); // Maintenance
    let _eb = PinDriver::input(peripherals.pins.gpio4).unwrap(); // East Button
    let _wb = PinDriver::input(peripherals.pins.gpio6).unwrap(); // West Button

    // Homing policy:
    // - StepperOnly: always home
    // - EncoderGuarded: home when snapshot restore is unavailable/untrusted
    // - EncoderGuarded: if NVS claims mechanical home (≈ home_heading_deg), require limit
    //   switch active before skipping homing; otherwise re-home like snapshot miss
    const HOMING_ENABLED: bool = true;
    const HOMING_DIRECTION: Direction = Direction::Ccw;
    /// Same tolerance as motion sunset home check (`location` vs `HOME_HEADING_DEG`).
    const HOME_HEADING_VERIFY_EPS_DEG: f32 = 0.01;

    let would_skip_homing_on_snapshot = motion_mode == MotionMode::EncoderGuarded
        && restored_from_snapshot
        && trust_nvs_state;
    let restored_claims_mechanical_home =
        (actual_heading - sw.home_heading_deg).abs() < HOME_HEADING_VERIFY_EPS_DEG;
    let home_claim_needs_limit_verify = would_skip_homing_on_snapshot
        && restored_claims_mechanical_home
        && !motion.switch_pressed();
    if home_claim_needs_limit_verify {
        log::info!(
            "Restored heading matches home ({}) but limit switch not pressed; homing to verify",
            sw.home_heading_deg
        );
    }

    let should_home_by_mode = motion_mode == MotionMode::StepperOnly
        || !restored_from_snapshot
        || !trust_nvs_state
        || home_claim_needs_limit_verify;
    if should_home_by_mode && HOMING_ENABLED {
        let mut on_tick = pump_serial_during_move(
            &mut serial_diagnostics,
            "homing",
            &current_version_string,
            &mut led,
        );
        let limit_sw_status = match HOMING_DIRECTION {
            Direction::Cw => motion.find_limit_switch_cw_watched(&mut on_tick),
            Direction::Ccw => motion.find_limit_switch_ccw_watched(&mut on_tick),
        };
        drop(on_tick);
        match limit_sw_status {
            true => log::info!(
                "Homing OK (dir={}): limit switch found",
                HOMING_DIRECTION.as_str()
            ),
            false => {
                log::error!(
                    "Homing FAILED (dir={}): limit switch could not be found",
                    HOMING_DIRECTION.as_str()
                );
                infra::error_loop(
                    sw.device_id,
                    &mut mqtt,
                    network::telemetry::Component::LimitSwitch,
                    "Limit switch not found during boot homing",
                    "Boot-time homing sweep completed without detecting the limit switch; tower orientation unknown.",
                );
            }
        }
        // Align RAM and NVS with home heading after homing.
        actual_heading = sw.home_heading_deg;
        if PERSIST_NVS {
            infra::SnapshotStore::new(&mut nvs, true).save_heading(sw.home_heading_deg);
            if motion_mode == MotionMode::EncoderGuarded {
                infra::SnapshotStore::new(&mut nvs, true)
                    .save_encoder_snapshot(motion.encoder_ticks_adjusted());
            }
        }
        sleep_answering_boot_diagnostics(
            Duration::from_secs(5),
            &mut BootServices {
            serial: &mut serial_diagnostics,
            command_state: &diagnostics_command_state,
            nvs: &mut nvs,
            staging: &mut config_staging,
            bus,
            firmware_version: &current_version_string,
            led: Some(&mut led),
        },
            "settling after homing",
        );
    } else if should_home_by_mode {
        log::warn!("Homing skipped: HOMING_ENABLED=false");
        if !trust_nvs_state {
            infra::error_loop(
                sw.device_id,
                &mut mqtt,
                network::telemetry::Component::System,
                "Homing disabled with untrusted NVS state",
                "HOMING_ENABLED=false but persisted heading could not be trusted; manual intervention required.",
            );
        }
    } else {
        log::info!("Skipping homing: restored heading+encoder snapshot from NVS");
    }

    // Mark this boot as Normal so next boot can trust NVS.
    infra::SnapshotStore::new(&mut nvs, true).save_last_run_normal(true);
    // Publish a post-boot baseline snapshot before entering the steady-state
    // tracking loop so diagnostics can report a useful initial status.
    diagnostics::mqtt::update_snapshot(
        &diagnostics_snapshot,
        diagnostics::mqtt::OwnedStatusSnapshot {
            device_id: sw.device_id.to_string(),
            firmware_version: current_version_string.clone(),
            mqtt_connected: mqtt.is_connected(),
            wifi_connected: matches!(wifi.state(), WifiState::Connected(_)),
            motion_mode: match motion_mode {
                MotionMode::StepperOnly => String::from("StepperOnly"),
                MotionMode::EncoderGuarded => String::from("EncoderGuarded"),
            },
            current_heading: actual_heading,
            activity: String::from("idle"),
            activity_reason: String::from("Tower finished boot and is ready for tracking"),
            sun_angle: None,
            target_heading: None,
            angle_offset: None,
            last_move_outcome: None,
        },
    );

    // PHASE 8: MAIN TRACKING LOOP ---------------------------------------------
    let mut previous_motion_mode = motion_mode;
    let mut need_rehome_stepper_only = false;

    loop {
        let local_time = rtc::timezone::local_time();
        let current_datetime = format!("{}", local_time.format("%d/%m/%Y %H:%M:%S"));

        // Daily encoder mode reset
        let reset_occurred = check_daily_encoder_reset(&mut nvs, &local_time, PERSIST_NVS);
        if reset_occurred {
            motion_mode = infra::SnapshotStore::new(&mut nvs, PERSIST_NVS)
                .load_tracking_mode_or_init(MotionMode::EncoderGuarded);
            motion.set_motion_mode(motion_mode);
            encoder_fault.set_mode_switched_daily(false);
            let mode_str = match motion_mode {
                MotionMode::StepperOnly => "StepperOnly",
                MotionMode::EncoderGuarded => "EncoderGuarded",
            };
            info!("Daily reset: Motion mode updated to {}", mode_str);
        }
        
        // Re-home once when transitioning into StepperOnly.
        if motion_mode == MotionMode::StepperOnly && previous_motion_mode != MotionMode::StepperOnly {
            info!("Motion mode switched to StepperOnly - re-homing required");
            need_rehome_stepper_only = true;
        }
        previous_motion_mode = motion_mode;
        
        if need_rehome_stepper_only && motion_mode == MotionMode::StepperOnly {
            info!("StepperOnly mode detected - re-homing to establish known position");
            const HOMING_DIRECTION: Direction = Direction::Ccw;
            let mut on_tick = pump_serial_during_move(
                &mut serial_diagnostics,
                "re-homing",
                &current_version_string,
                &mut led,
            );
            let limit_sw_status = match HOMING_DIRECTION {
                Direction::Cw => motion.find_limit_switch_cw_watched(&mut on_tick),
                Direction::Ccw => motion.find_limit_switch_ccw_watched(&mut on_tick),
            };
            drop(on_tick);
            match limit_sw_status {
                true => {
                    info!("Re-homing OK (dir={}): limit switch found", HOMING_DIRECTION.as_str());
                    actual_heading = sw.home_heading_deg;
                    motion.update_position(actual_heading);
                    if PERSIST_NVS {
                        infra::SnapshotStore::new(&mut nvs, PERSIST_NVS).save_heading(actual_heading);
                    }
                    need_rehome_stepper_only = false;
                }
                false => {
                    error!("Re-homing FAILED (dir={}): limit switch could not be found", HOMING_DIRECTION.as_str());
                    infra::error_loop(
                        sw.device_id,
                        &mut mqtt,
                        network::telemetry::Component::LimitSwitch,
                        "Re-home failed after encoder recovery",
                        "Encoder recovery completed but subsequent re-home could not locate the limit switch.",
                    );
                }
            }
            thread::sleep(Duration::from_secs(2));
            continue;
        }

        info!("Actual Heading: {}", motion.location());
        info!("Current datetime: {}", current_datetime.clone());

        let now = std::time::Instant::now();

        // Reload mode in case another path switched to StepperOnly.
        let current_motion_mode = infra::SnapshotStore::new(&mut nvs, PERSIST_NVS)
            .load_tracking_mode_or_init(MotionMode::EncoderGuarded);
        if current_motion_mode != motion_mode {
            motion_mode = current_motion_mode;
            motion.set_motion_mode(motion_mode);
            if motion_mode == MotionMode::StepperOnly {
                info!("Motion mode changed to StepperOnly - will re-home on next iteration");
                need_rehome_stepper_only = true;
            }
        }
        
        let encoder_recovery_cfg = EncoderRecoverySwitches {
            enabled: sw.runtime.encoder_recovery.enabled,
            probe_interval_secs: sw.runtime.encoder_recovery.probe_interval_secs,
            probe_steps: sw.runtime.encoder_recovery.probe_steps,
            max_drift_deg: sw.runtime.encoder_recovery.max_drift_deg,
            rehome_dir: match sw.runtime.encoder_recovery.rehome_dir {
                switchboard::Direction::Cw => crate::app::encoder_fault::Direction::Cw,
                switchboard::Direction::Ccw => crate::app::encoder_fault::Direction::Ccw,
            },
        };
        let mut on_tick = pump_serial_during_move(
            &mut serial_diagnostics,
            "encoder recovery",
            &current_version_string,
            &mut led,
        );
        let encoder_fault_active = encoder_fault.tick(
            &encoder_recovery_cfg,
            &mut motion,
            &mut on_tick,
            motion_mode,
            &mut actual_heading,
            &mut nvs,
            &mut mqtt,
            &mut wifi,
            &current_version,
            PERSIST_NVS,
            sw.device_id,
            sw.home_heading_deg,
        )?;
        drop(on_tick);

        if encoder_fault_active {
            continue;  // Fault active, skip tracking this iteration
        }
        
        // Re-check mode after encoder_fault.tick() in case it changed during recovery.
        let updated_motion_mode = infra::SnapshotStore::new(&mut nvs, PERSIST_NVS)
            .load_tracking_mode_or_init(MotionMode::EncoderGuarded);
        if updated_motion_mode != motion_mode {
            motion_mode = updated_motion_mode;
            motion.set_motion_mode(motion_mode);
            if motion_mode == MotionMode::StepperOnly {
                info!("Motion mode switched to StepperOnly during encoder fault recovery - will re-home");
                need_rehome_stepper_only = true;
            }
        }
        
        // Re-home before tracking if StepperOnly was activated during recovery.
        if need_rehome_stepper_only && motion_mode == MotionMode::StepperOnly {
            info!("StepperOnly mode detected - re-homing to establish known position (CCW)");
            const HOMING_DIRECTION: Direction = Direction::Ccw;
            let mut on_tick = pump_serial_during_move(
                &mut serial_diagnostics,
                "re-homing",
                &current_version_string,
                &mut led,
            );
            let limit_sw_status = match HOMING_DIRECTION {
                Direction::Cw => motion.find_limit_switch_cw_watched(&mut on_tick),
                Direction::Ccw => motion.find_limit_switch_ccw_watched(&mut on_tick),
            };
            drop(on_tick);
            match limit_sw_status {
                true => {
                    info!("Re-homing OK (dir={}): limit switch found", HOMING_DIRECTION.as_str());
                    actual_heading = sw.home_heading_deg;
                    motion.update_position(actual_heading);
                    if PERSIST_NVS {
                        infra::SnapshotStore::new(&mut nvs, PERSIST_NVS).save_heading(actual_heading);
                    }
                    need_rehome_stepper_only = false;
                }
                false => {
                    error!("Re-homing FAILED (dir={}): limit switch could not be found", HOMING_DIRECTION.as_str());
                    infra::error_loop(
                        sw.device_id,
                        &mut mqtt,
                        network::telemetry::Component::LimitSwitch,
                        "Re-home failed after encoder recovery",
                        "Encoder recovery completed but subsequent re-home could not locate the limit switch.",
                    );
                }
            }
            thread::sleep(Duration::from_secs(2));
            continue;
        }

        const TRACKING_ENABLED: bool = true;
        let (activity, activity_reason, last_sun_angle, last_target_heading, last_angle_offset, last_move_outcome) = if TRACKING_ENABLED {
            let motion_mode_str = match motion_mode {
                MotionMode::StepperOnly => "StepperOnly",
                MotionMode::EncoderGuarded => "EncoderGuarded",
            };
            let mut activity = if motion_mode == MotionMode::StepperOnly {
                String::from("stepper_only_tracking")
            } else {
                String::from("tracking")
            };
            let mut activity_reason = format!(
                "Tracking enabled in {}; tower heading is {:.2} degrees",
                motion_mode_str, actual_heading
            );
            // A tracking pass blocks this thread, and a sunset homing run inside it
            // can block for minutes. Keep answering read-only serial throughout.
            let mut on_tick = pump_serial_during_move(
                &mut serial_diagnostics,
                "tracking move",
                &current_version_string,
                &mut led,
            );
            let tick_result = app::tracking_loop::tick(
                &mut motion,
                &mut on_tick,
                &mut calculation,
                &mut actual_heading,
                &mut mqtt,
                &current_version,
                &mut nvs,
                &mut wifi,
                current_datetime.clone(),
                PERSIST_NVS,
                allow_ota,
                sw.device_id,
            );
            drop(on_tick);

            let outcome = tick_result.outcome;
            let (last_sun_angle, last_target_heading, last_angle_offset) = match tick_result.snapshot {
                Some(snapshot) => (
                    Some(snapshot.sun_angle),
                    Some(snapshot.target_heading),
                    Some(snapshot.angle_offset),
                ),
                None => (None, None, None),
            };
            let last_move_outcome = format!("{:?}", outcome);

            if outcome != MoveOutcome::Completed {
                warn!("Last move aborted: {:?}", outcome);
                activity = String::from("tracking_aborted");
                activity_reason = format!("Last tracking move aborted: {:?}", outcome);
            } else {
                activity_reason = format!("Tracking tick completed; tower heading is {:.2} degrees", actual_heading);
            }
            if let Err(e) = encoder_fault.on_move_outcome(outcome, &encoder_recovery_cfg, &mut motion, &mut nvs, PERSIST_NVS) {
                error!("Error in encoder fault recovery: {:?}", e);
            }
            (
                activity,
                activity_reason,
                last_sun_angle,
                last_target_heading,
                last_angle_offset,
                Some(last_move_outcome),
            )
        } else {
            info!("Tracking disabled");
            (
                String::from("tracking_disabled"),
                String::from("Tracking is disabled by runtime configuration"),
                None,
                None,
                None,
                None,
            )
        };

        info!("Tracking loop duration (v1.1.3): {:?}", now.elapsed());
        
        // Housekeeping
        if wifi.state() == WifiState::Disconnected {
            warn!("Wifi disconnected, attempting to reconnect...");
            wifi.reconnect_if_disconnected()?;
        }
        // Heartbeat ping: see `network::telemetry::topic::status` for the topic string.
        {
            let payload = network::telemetry::Heartbeat {
                current_time: &current_datetime,
                firmware_version: &current_version.to_string(),
            };
            let topic = network::telemetry::topic::status(sw.device_id);
            let _ = network::telemetry::publish_json(&mut mqtt, &topic, &payload);
        }

        // Refresh the shared diagnostics snapshot once per main-loop pass. The
        // diagnostics thread serves `get_status` from this cached state instead
        // of reaching into live control objects from another thread.
        diagnostics::mqtt::update_snapshot(
            &diagnostics_snapshot,
            diagnostics::mqtt::OwnedStatusSnapshot {
                device_id: sw.device_id.to_string(),
                firmware_version: current_version_string.clone(),
                mqtt_connected: mqtt.is_connected(),
                wifi_connected: matches!(wifi.state(), WifiState::Connected(_)),
                motion_mode: match motion_mode {
                    MotionMode::StepperOnly => String::from("StepperOnly"),
                    MotionMode::EncoderGuarded => String::from("EncoderGuarded"),
                },
                current_heading: actual_heading,
                activity,
                activity_reason,
                sun_angle: last_sun_angle,
                target_heading: last_target_heading,
                angle_offset: last_angle_offset,
                last_move_outcome,
            },
        );

        const LOOP_SLEEP_SECS: u64 = 300;
        // Replace the old monolithic 300-second sleep with short slices so the
        // main loop can service queued diagnostics control commands promptly.
        let sleep_deadline = std::time::Instant::now() + Duration::from_secs(LOOP_SLEEP_SECS);
        sleep_with_diagnostics_control_until(
            sleep_deadline,
            &diagnostics_control_rx,
            &diagnostics_snapshot,
            &diagnostics_command_state,
            &mut serial_diagnostics,
            &current_version_string,
            &mut motion,
            &mut led,
            &mut nvs,
            &mut config_staging,
            bus,
            &mut mqtt,
            &wifi,
            sw.device_id,
            sw.home_heading_deg,
            motion_mode,
            &mut actual_heading,
            PERSIST_NVS,
            &mut need_rehome_stepper_only,
        )?;

    }
}

/// A [`motion::MoveWatcher`] that services read-only serial diagnostics.
///
/// Homing and tracking moves block this thread for as long as they take — a
/// worst-case homing search runs for tens of minutes — and without this the board
/// is silent for all of it. The commands this can answer need no hardware, so
/// running them from inside the stepping loop does not break the rule that the
/// main loop is the single owner of motion, NVS, and the I2C bus: see
/// `SerialDiagnosticsRuntime::poll_readonly`, which refuses everything else.
///
/// Motion calls this from the same 100 ms block as its existing position log, so
/// the added work on the stepping loop is comparable to what it already does.
fn pump_serial_during_move<'a>(
    serial_diagnostics: &'a mut diagnostics::serial::SerialDiagnosticsRuntime,
    phase: &'a str,
    firmware_version: &'a str,
    // `Led<'static>` rather than a second lifetime parameter: the LED is built from
    // owned peripherals and lives for the whole program, and a borrowed inner
    // lifetime would have to appear in the return type's bounds to be captured.
    led: &'a mut Led<'static>,
) -> impl FnMut(motion::MoveTick) + 'a {
    serial_diagnostics.announce_phase(phase);
    move |_tick| {
        if let Err(err) = serial_diagnostics.poll_minimal(phase, firmware_version, Some(led)) {
            warn!("Serial diagnostics unavailable during {}: {:?}", phase, err);
        }
    }
}

/// What the runtime can lend to diagnostics during a boot wait.
///
/// Bundled because the list grew: the bus and NVS exist long before the motion
/// stack does, so a wait can serve the sensors and the whole configuration set,
/// not merely the three commands that need nothing at all.
struct BootServices<'a, T: esp_idf_svc::nvs::NvsPartitionId> {
    serial: &'a mut diagnostics::serial::SerialDiagnosticsRuntime,
    command_state: &'a diagnostics::mqtt::SharedCommandState,
    nvs: &'a mut esp_idf_svc::nvs::EspNvs<T>,
    staging: &'a mut board_diagnostics::ConfigStaging,
    bus: &'static shared_bus::BusManagerStd<I2cDriver<'static>>,
    firmware_version: &'a str,
    /// `None` for the waits that happen before the LED is built. Saying it is not
    /// initialised when it is would be exactly the sort of misleading answer these
    /// diagnostics exist to avoid.
    led: Option<&'a mut Led<'static>>,
}

/// Sleep for `duration`, answering serial diagnostics while waiting.
///
/// Every blocking wait in the boot path used to be dead air on the serial link.
/// The installer opens the port and queries `FIRMWARE_VERSION` one second later,
/// which landed in that silence and timed out — so the first thing the app did
/// always failed. `phase` names what the tower is doing, and goes out verbatim on
/// commands that cannot be served yet.
///
/// Motion is the one thing withheld: it does not exist yet at these points, so
/// `GO_HOME` is refused — by name, saying the motion stack is not up, rather than
/// blaming whatever the tower happens to be waiting for.
fn sleep_answering_boot_diagnostics<T: esp_idf_svc::nvs::NvsPartitionId>(
    duration: Duration,
    services: &mut BootServices<'_, T>,
    phase: &str,
) {
    const SLICE_MS: u64 = 50;

    services.serial.announce_phase(phase);

    let deadline = std::time::Instant::now() + duration;
    loop {
        if let Err(err) = services.serial.poll(
            services.command_state,
            services.firmware_version,
            None,
            services.led.as_deref_mut(),
            services.nvs,
            services.staging,
            services.bus,
        ) {
            warn!("Serial diagnostics unavailable during {}: {:?}", phase, err);
        }

        let now = std::time::Instant::now();
        if now >= deadline {
            return;
        }
        thread::sleep(
            deadline
                .saturating_duration_since(now)
                .min(Duration::from_millis(SLICE_MS)),
        );
    }
}

fn boot_diagnostic<T: esp_idf_svc::nvs::NvsPartitionId>(
    device_id: &str,
    wifi: &mut Wifi,
    mqtt: &mut Mqtt,
    current_version: &Version,
    services: &mut BootServices<'_, T>,
) -> bool {
    info!("Starting boot validation in 5 seconds...");
    sleep_answering_boot_diagnostics(Duration::from_secs(5), services, "boot validation");

    // Wi-Fi check
    match wifi.state() {
        WifiState::Connected(ip) => {
            info!("Wi-Fi connected with IP: {}", ip);
        }
        WifiState::Connecting => {
            warn!("Wi-Fi still connecting during validation...");
            return false;
        }
        WifiState::Disconnected => {
            error!("Wi-Fi disconnected, validation failed");
            return false;
        }
    }

    const MAX_RETRIES: u8 = 3;

    for attempt in 1..=MAX_RETRIES {
        info!("Boot diagnostic MQTT attempt {}/{}", attempt, MAX_RETRIES);

        let mut waited = 0;
        while !mqtt.is_connected() && waited < 12000 {
            sleep_answering_boot_diagnostics(
                Duration::from_millis(3000),
                services,
                "waiting for MQTT",
            );
            waited += 3000;
        }

        if !mqtt.is_connected() {
            warn!("MQTT not connected yet, retrying...");
            continue;
        }

        let current_time = rtc::timezone::local_time()
            .format(network::telemetry::TIME_FORMAT)
            .to_string();
        let payload = network::telemetry::BootLog {
            current_time: &current_time,
            message: "Tower rebooted successfully",
            firmware_version: &current_version.to_string(),
            component: network::telemetry::Component::System,
            notes: "Scheduled reboot completed without errors",
        };
        let topic = network::telemetry::topic::logs_boot(device_id);
        if network::telemetry::publish_json(mqtt, &topic, &payload).is_ok() {
            return true;
        }
        error!("MQTT publish failed immediately");
        if attempt == MAX_RETRIES {
            error!("All MQTT boot diagnostic attempts failed...");
            return false;
        }
        sleep_answering_boot_diagnostics(
            Duration::from_millis(1000),
            services,
            "retrying MQTT boot validation",
        );
        continue;
    }
    false
}

/// Reset daily encoder mode at day rollover.
fn check_daily_encoder_reset<T: esp_idf_svc::nvs::NvsPartitionId>(
    nvs: &mut esp_idf_svc::nvs::EspNvs<T>,
    local_time: &DateTime<Local>,
    persist_nvs: bool,
) -> bool {
    let mut snapshot_store = infra::SnapshotStore::new(nvs, persist_nvs);
    
    let encoder_daily_mode = snapshot_store.load_encoder_daily_mode();
    if !encoder_daily_mode {
        return false;
    }

    let current_date = local_time.format("%Y-%m-%d").to_string();

    let stored_date = snapshot_store.load_encoder_mode_reset_date();

    match stored_date {
        Some(stored) if stored != current_date => {
            info!("Daily reset: New day detected (stored={}, current={}), resetting encoder mode to EncoderGuarded", stored, current_date);
            snapshot_store.save_tracking_mode(MotionMode::EncoderGuarded);
            snapshot_store.save_encoder_daily_mode(false);
            snapshot_store.save_encoder_mode_reset_date(&current_date);
            true
        }
        Some(_stored) => false,
        None => {
            warn!("encoder_daily_mode is true but no reset_date found in NVS; initializing reset_date");
            snapshot_store.save_encoder_mode_reset_date(&current_date);
            false
        }
    }
}

fn sleep_with_diagnostics_control_until<T: esp_idf_svc::nvs::NvsPartitionId>(
    deadline: std::time::Instant,
    control_rx: &diagnostics::mqtt::ControlCommandReceiver,
    diagnostics_snapshot: &diagnostics::mqtt::SharedStatusSnapshot,
    diagnostics_command_state: &diagnostics::mqtt::SharedCommandState,
    serial_diagnostics: &mut diagnostics::serial::SerialDiagnosticsRuntime,
    firmware_version: &str,
    motion: &mut Motion<'_>,
    led: &mut Led<'_>,
    nvs: &mut esp_idf_svc::nvs::EspNvs<T>,
    config_staging: &mut board_diagnostics::ConfigStaging,
    bus: &'static shared_bus::BusManagerStd<I2cDriver<'static>>,
    mqtt: &mut Mqtt,
    wifi: &Wifi<'_>,
    device_id: &str,
    home_heading_deg: f32,
    motion_mode: MotionMode,
    actual_heading: &mut f32,
    persist_nvs: bool,
    need_rehome_stepper_only: &mut bool,
) -> anyhow::Result<()> {
    const SLEEP_SLICE_MS: u64 = 100;
    // Time allowed for a diagnostics response to clear the USB Serial/JTAG FIFO
    // before a requested reset tears the peripheral down.
    const REBOOT_FLUSH_DELAY_MS: u64 = 250;

    // Reaching the idle window is the one moment the whole command set is
    // available, so it is worth saying out loud — the phases announced everywhere
    // else all mean "not yet".
    serial_diagnostics.announce_phase("ready");

    while std::time::Instant::now() < deadline {
        // Service any queued diagnostics commands during the idle window
        // between tracking passes without giving up main-loop ownership.
        service_diagnostics_control_commands(
            control_rx,
            diagnostics_snapshot,
            diagnostics_command_state,
            firmware_version,
            motion,
            led,
            nvs,
            config_staging,
            bus,
            mqtt,
            wifi,
            device_id,
            home_heading_deg,
            motion_mode,
            actual_heading,
            persist_nvs,
            need_rehome_stepper_only,
        )?;

        let serial_poll = serial_diagnostics.poll(
            diagnostics_command_state,
            firmware_version,
            Some(&mut diagnostics::executor::MotionContext {
                motion,
                home_heading_deg,
                motion_mode,
                actual_heading,
                persist_nvs,
                need_rehome_stepper_only,
            }),
            Some(led),
            nvs,
            config_staging,
            bus,
        )?;

        if serial_poll != diagnostics::serial::SerialDiagnosticsPoll::Idle {
            diagnostics::mqtt::update_snapshot(
                diagnostics_snapshot,
                diagnostics::mqtt::OwnedStatusSnapshot {
                    device_id: device_id.to_string(),
                    firmware_version: firmware_version.to_string(),
                    mqtt_connected: mqtt.is_connected(),
                    wifi_connected: matches!(wifi.state(), WifiState::Connected(_)),
                    motion_mode: match motion_mode {
                        MotionMode::StepperOnly => String::from("StepperOnly"),
                        MotionMode::EncoderGuarded => String::from("EncoderGuarded"),
                    },
                    current_heading: *actual_heading,
                    activity: String::from("serial_diagnostics"),
                    activity_reason: String::from(
                        "Serial diagnostics command processed on the main control loop",
                    ),
                    sun_angle: None,
                    target_heading: None,
                    angle_offset: None,
                    last_move_outcome: Some(String::from("SerialDiagnostics")),
                },
            );
        }

        // REBOOT is answered here rather than in the transport: resetting is the
        // main loop's call, and `OK Rebooting` is still sitting in the USB
        // Serial/JTAG FIFO. The reset drops the CDC device with it, so give the
        // host time to read the line before the port disappears.
        if serial_poll == diagnostics::serial::SerialDiagnosticsPoll::RebootRequested {
            info!("Serial diagnostics requested a reboot");
            thread::sleep(Duration::from_millis(REBOOT_FLUSH_DELAY_MS));
            esp_idf_svc::hal::reset::restart();
        }

        let now = std::time::Instant::now();
        if now >= deadline {
            break;
        }

        let remaining = deadline.saturating_duration_since(now);
        thread::sleep(remaining.min(Duration::from_millis(SLEEP_SLICE_MS)));
    }

    Ok(())
}

fn service_diagnostics_control_commands<T: esp_idf_svc::nvs::NvsPartitionId>(
    control_rx: &diagnostics::mqtt::ControlCommandReceiver,
    diagnostics_snapshot: &diagnostics::mqtt::SharedStatusSnapshot,
    diagnostics_command_state: &diagnostics::mqtt::SharedCommandState,
    firmware_version: &str,
    motion: &mut Motion<'_>,
    led: &mut Led<'_>,
    nvs: &mut esp_idf_svc::nvs::EspNvs<T>,
    config_staging: &mut board_diagnostics::ConfigStaging,
    bus: &'static shared_bus::BusManagerStd<I2cDriver<'static>>,
    mqtt: &mut Mqtt,
    wifi: &Wifi<'_>,
    device_id: &str,
    home_heading_deg: f32,
    motion_mode: MotionMode,
    actual_heading: &mut f32,
    persist_nvs: bool,
    need_rehome_stepper_only: &mut bool,
) -> anyhow::Result<()> {
    const MAX_COMMANDS_PER_SLICE: usize = 4;

    for _ in 0..MAX_COMMANDS_PER_SLICE {
        let command = match control_rx.try_recv() {
            Ok(command) => command,
            Err(std::sync::mpsc::TryRecvError::Empty) => break,
            Err(std::sync::mpsc::TryRecvError::Disconnected) => {
                warn!("Diagnostics control queue disconnected; no further control commands will run");
                break;
            }
        };

        match command {
            diagnostics::mqtt::ControlCommand::ExecuteFirstWave {
                request_id,
                cmd,
                command,
            } => {
                // This path runs on the main loop, not the diagnostics thread,
                // so motor/NVS access stays serialized in one place.
                //
                // MQTT publishes one message when the command finishes, so unlike
                // the serial path it collects the output rather than streaming it.
                let mut io = diagnostics::executor::TranscriptIo::default();
                let mut transcript = diagnostics::executor::execute_first_wave_command(
                    &mut io,
                    command,
                    firmware_version,
                    Some(&mut diagnostics::executor::MotionContext {
                        motion,
                        home_heading_deg,
                        motion_mode,
                        actual_heading,
                        persist_nvs,
                        need_rehome_stepper_only,
                    }),
                    Some(led),
                    nvs,
                    config_staging,
                    bus,
                );
                transcript.lines = io.into_lines();

                let completion = diagnostics::mqtt::complete_control_command(
                    diagnostics_command_state,
                    &request_id,
                );
                diagnostics::mqtt::update_snapshot(
                    diagnostics_snapshot,
                    diagnostics::mqtt::OwnedStatusSnapshot {
                        device_id: device_id.to_string(),
                        firmware_version: firmware_version.to_string(),
                        mqtt_connected: mqtt.is_connected(),
                        wifi_connected: matches!(wifi.state(), WifiState::Connected(_)),
                        motion_mode: match motion_mode {
                            MotionMode::StepperOnly => String::from("StepperOnly"),
                            MotionMode::EncoderGuarded => String::from("EncoderGuarded"),
                        },
                        current_heading: *actual_heading,
                        activity: if transcript.status == "completed" {
                            format!("{}_completed", cmd)
                        } else {
                            format!("{}_failed", cmd)
                        },
                        activity_reason: transcript.message.clone(),
                        sun_angle: None,
                        target_heading: None,
                        angle_offset: None,
                        last_move_outcome: Some(cmd.clone()),
                    },
                );
                if completion == diagnostics::mqtt::ControlCompletionDisposition::PublishFinalResult {
                    if let Err(e) = diagnostics::mqtt::publish_transcript(
                        mqtt,
                        device_id,
                        &request_id,
                        &cmd,
                        &transcript,
                    ) {
                        warn!("Failed to publish diagnostics control result: {:?}", e);
                    }
                } else {
                    warn!(
                        "Suppressing late diagnostics control result for request_id={}",
                        request_id
                    );
                }
            }
        }
    }

    Ok(())
}
