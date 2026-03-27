use std::time::{Duration, Instant, SystemTime};
use chrono::{DateTime, FixedOffset, Utc};
use clock::Clock;
use log::{error, info, warn};
use std::thread;

#[path = "../config.rs"]
mod config;
#[path = "../constants.rs"]
mod constants;
#[path = "../config_manager.rs"]
mod config_manager;
#[path = "../switchboard.rs"]
mod switchboard;
mod infra;
mod app;
#[path = "../diagnostics/mod.rs"]
mod diagnostics;

// Provide __pender function for embassy_executor
// This function is called by embassy_executor to wake tasks
#[no_mangle]
pub extern "C" fn __pender() {
    // For ESP-IDF with FreeRTOS, this is typically a no-op when using
    // the embassy-time-driver feature, as the time driver handles wake-ups
    // This stub satisfies the linker requirement
}

use esp_idf_svc::{
    eventloop::EspSystemEventLoop,
    hal::{
        i2c::{I2cConfig, I2cDriver},
        prelude::*,
    },
    log::EspLogger,
    nvs::{EspDefaultNvsPartition, EspNvs},
    ota::EspOta,
    sntp::{EspSntp, SyncStatus},
};
use motion::{MoveOutcome, Motion, MotionMode};
use rgb_led::Led;
use network::mqtt::Mqtt;
use ota::OtaUpdater;
use semver::Version;
use wifi::wifi::{Wifi, WifiState};

use crate::app::encoder_fault::{Direction, EncoderRecoverySwitches};
use crate::constants::{get_active_mode, get_active_profile, RunMode};
use crate::diagnostics::{admin_mode, cmd_handler};
use config_manager::{ConfigManager, RuntimeMode};

fn main() -> anyhow::Result<()> {
    let active_profile = get_active_profile();
    let compile_time_mode = get_active_mode();
    let sw = switchboard::active(active_profile);
    let persist_nvs = sw.effects.persist_nvs;
    let publish_mqtt = sw.effects.publish_mqtt;
    let allow_ota = sw.effects.allow_ota;
    let allow_boot_validation = sw.effects.allow_boot_validation;

    // PHASE 1: INITIALIZATION--------------------------------------------------
    
    // Required for ESP-IDF patches
    esp_idf_svc::sys::link_patches();

    // Initialize logger and system event loop
    EspLogger::initialize_default();
    let sysloop = EspSystemEventLoop::take()?;

    // Initialize hardware peripherals (GPIO, I2C, etc.)
    let peripherals = Peripherals::take().unwrap();
    let nvs_default = EspDefaultNvsPartition::take()?;
    
    // Open NVS (Non-Volatile Storage) for persistent state
    let mut nvs = match EspNvs::new(nvs_default.clone(), "storage", true) {
        Ok(nvs) => {
            info!("Got namespace {:?} from default partition", "storage");
            nvs
        }
        Err(e) => panic!("Could't get namespace {:?}", e),
    };

    let mut config_manager = ConfigManager::new(&mut nvs, persist_nvs);
    let runtime_mode = config_manager.get_runtime_mode(&mut nvs);
    let active_mode = match runtime_mode {
        RuntimeMode::Admin => RunMode::Admin,
        RuntimeMode::Normal => compile_time_mode,
    };
    info!(
        "Mode: compile_time={:?}, runtime={:?}, active={:?}",
        compile_time_mode, runtime_mode, active_mode
    );

    let last_run_normal = infra::SnapshotStore::new(&mut nvs, persist_nvs)
        .load_last_run_normal_or_init(true);
    let trust_nvs_state = match active_mode {
        RunMode::Normal => last_run_normal,
        RunMode::Admin => false,
    };
    info!(
        "Last run normal={} -> trust_nvs_state={} (active_mode={:?})",
        last_run_normal, trust_nvs_state, active_mode
    );

    if active_mode == RunMode::Admin {
        infra::SnapshotStore::new(&mut nvs, persist_nvs).save_last_run_normal(false);
    }

    // Setup encoder pins
    let encoder_a = peripherals.pins.gpio10;
    let encoder_b = peripherals.pins.gpio11;

    // Setup I2C bus for Clock
    let sda = peripherals.pins.gpio8;
    let scl = peripherals.pins.gpio9;
    let config = I2cConfig::new().baudrate(10_u32.kHz().into());
    let i2c = I2cDriver::new(peripherals.i2c0, sda, scl, &config).unwrap();
    let bus: &'static _ = shared_bus::new_std!(I2cDriver = i2c).unwrap();

    // PHASE 2: NETWORK SETUP--------------------------------------------------

    if persist_nvs {
        match nvs.set_str("mqtt_user", sw.default_mqtt_user) {
            Ok(_) => info!("Mqtt username updated"),
            Err(e) => error!("Mqtt username not updated {:?}", e),
        };
    }
    if persist_nvs {
        match nvs.set_str("mqtt_pass", sw.default_mqtt_pass) {
            Ok(_) => info!("Mqtt password updated"),
            Err(e) => error!("Mqtt password not updated {:?}", e),
        };
    }

    if persist_nvs {
        match nvs.set_str("wifi_ssid", sw.default_wifi_ssid) {
            Ok(_) => info!("Wifi ssid updated"),
            Err(e) => error!("Wifi ssid not updated {:?}", e),
        };
    }
    if persist_nvs {
        match nvs.set_str("wifi_pass", sw.default_wifi_pass) {
            Ok(_) => info!("Wifi password updated"),
            Err(e) => error!("Wifi password not updated {:?}", e),
        };
    }

    if persist_nvs {
        match nvs.set_i32("offset_hours", sw.default_tz_offset_hours) {
            Ok(_) => info!("Timezone offset has been updated"),
            Err(e) => error!("Timezone offset was not updated {:?}", e),
        };
    }

    // Connect to WiFi
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
    log::info!("Waiting for 20 seconds before connecting to wifi");
    thread::sleep(Duration::from_secs(20));
	wifi.connect(&real_wifi_ssid, &real_wifi_pass).expect("Wi-Fi connection failed");
	info!("Current wifi state: {:?}", wifi.state());
    if wifi.state() == WifiState::Disconnected {
        wifi.reconnect_if_disconnected()?;
    }

    // Synchronize time with NTP server
    let ntp = EspSntp::new_default().unwrap();
    info!("Synchronizing with NTP Server");
    while ntp.get_sync_status() != SyncStatus::Completed {}
    info!("Time Sync Completed");

    // Calculate local time from UTC + timezone offset
    let st_now = SystemTime::now();
    let dt_now_utc: DateTime<Utc> = st_now.clone().into();
    let timezone_offset_hours: i32 = nvs.get_i32("offset_hours")?.unwrap_or(-5);
    let local_time: DateTime<FixedOffset> = DateTime::from_naive_utc_and_offset(
        dt_now_utc.naive_utc(),
        FixedOffset::east_opt(timezone_offset_hours * 3600).unwrap(),  
    );
    let formatted_time = format!("{}", local_time.format("%d/%m/%Y %H:%M:%S"));
    info!("{}", formatted_time);

    // Connect to MQTT broker
    let real_mqtt_user = nvs
        .get_str("mqtt_user", &mut buffer)?
        .expect("Mqtt username not found")
        .to_string();
    let real_mqtt_pass = nvs
        .get_str("mqtt_pass", &mut buffer)?
        .expect("Mqtt password not found")
        .to_string();

    let mut mqtt = Box::new(Mqtt::new_mqtt(
        sw.mqtt_broker_url,
        sw.mqtt_client_id,
        &real_mqtt_user,
        &real_mqtt_pass,
    )?);

    cmd_handler::CommandHandler::init(&mut mqtt, sw.device_id)?;

    // PHASE 3: BOOT VALIDATION--------------------------------------------------
    
    let first_boot = nvs.get_u8("first_boot")?.unwrap_or(1);

    // Run boot diagnostics (WiFi/MQTT connectivity check)
    let boot_diagnostic_result = if allow_boot_validation {
        boot_diagnostic(sw.device_id, &mut wifi, &mut mqtt, publish_mqtt)
    } else {
        info!("Boot validation disabled");
        true
    };

    // On first boot after OTA update, validate firmware before marking as valid
    // If validation fails, rollback to previous firmware version
    if allow_boot_validation && first_boot == 1 {
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
            } else {
                error!("Boot validation failed, rolling back firmware");
                valid_ota.mark_running_slot_invalid_and_reboot();
            }
        }
    } else {
        info!("Normal boot firmware already validated");
    }

    
    // PHASE 4: FIRMWARE VERSION & OTA--------------------------------------------------


    let mut version_buf = [0u8; 32];
    const DEFAULT_VERSION: &str = "1.0.6";
    
    // Store current firmware version in NVS
    if persist_nvs {
        nvs.set_str("version", "1.0.6")?;
    }

    // Read firmware version from NVS (or use default)
    let current_version: Version = nvs
        .get_str("version", &mut version_buf)?
        .map(|s| s.trim().parse::<Version>())
        .transpose()?
        .unwrap_or_else(|| Version::parse(DEFAULT_VERSION).unwrap());

    // Publish firmware version to MQTT
    infra::Telemetry::publish_firmware_version_if(
        sw.device_id,
        &mut mqtt,
        formatted_time.clone(),
        &current_version,
        publish_mqtt,
    );

    // Check for OTA updates
    let mut updater = OtaUpdater::new_ota(
        current_version.clone(),
        &mut mqtt,
        Some(sw.default_ota_updater),
        Some(sw.default_ota_password),
    ).expect("Failed to create OTA updater instance");

    if allow_ota {
        info!("Checking for new OTA update in 3 seconds...");
        thread::sleep(Duration::from_secs(3));
        if let Err(e) = updater.run_version_compare(&mut nvs) {
            mqtt.publish(&infra::topic(sw.device_id, "firmware/status"), b"OTA update failed!")?;
            error!("Version compare failed: {:?}", e);
        }
        else {
            info!("Version compare succeeded");
        }
    } else {
        info!("OTA disabled: skipping version compare");
    }
    
    // PHASE 5: MOTION INITIALIZATION--------------------------------------------------
    
    // Tower location (Sadler, TX): 33.75944 N, -96.82722 W
    let tower_latitude: f64 = 33.75944;
    if persist_nvs {
        match nvs.set_str("tower_latitude", &tower_latitude.to_string()) {
            Ok(_) => info!("Tower latitude has been updated"),
            Err(e) => error!("Tower latitude was not updated {:?}", e),
        };
    }

    let tower_longitude: f64 = -96.82722;
    if persist_nvs {
        match nvs.set_str("tower_longitude", &tower_longitude.to_string()) {
            Ok(_) => info!("Tower longitude has been updated"),
            Err(e) => error!("Tower longitude was not updated {:?}", e),
        };
    }

    // Load tower location from NVS
    let tower_id: u32 = 1;
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
    info!("Tower id: {}, Lat: {}, Lon: {}, Alt: {}", tower_id, latitude, longitude, altitude);

     
    //HARDWARE INITIALIZATION
    
    // Initialize Clock (sun position calculator) with location and current time
    let mut calculation = Clock::new(bus.acquire_i2c(), latitude, longitude, altitude);
    calculation.set_date_time(&local_time.naive_local());
    
    // Initialize LED status indicator
    let mut led = Led::new(peripherals.pins.gpio7, peripherals.rmt.channel0).unwrap();

    // Initialize Motion (motor, encoder, relay, limit switch)
    let mut motion = Motion::new(
        peripherals.pins.gpio15,   // CCW Motor
        peripherals.pins.gpio16,   // CW Motor
        peripherals.pins.gpio17,   // Relay 
        peripherals.pins.gpio14,   // Limit Switch 
        encoder_a,                 // Encoder A
        encoder_b,                 // Encoder B
    );
   
    motion.init();          // Initialize motor driver parameters
    led.display_healthy();  // Show healthy LED status
    let _ = motion.run();   // Ensure motor driver is in a ready state

    // Apply runtime guardrails (stall detection, soft limits) from switchboard
    motion.set_stall_detection_enabled(sw.runtime.guardrails.stall_detection_enabled);
    motion.set_soft_limits(
        sw.runtime.guardrails.soft_limits_enabled,
        sw.runtime.guardrails.soft_limit_min_deg,
        sw.runtime.guardrails.soft_limit_max_deg,
    );

    const POWER_ON: bool = true;  // Motor power control (future: sense input)

    // Check for daily encoder mode reset (before loading tracking mode)
    check_daily_encoder_reset(&mut nvs, &local_time, persist_nvs);

    // Select motion mode: Load from NVS, default to EncoderGuarded
    let mut motion_mode = infra::SnapshotStore::new(&mut nvs, persist_nvs)
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

    // PHASE 6: STATE RESTORATION--------------------------------------------------
    
    let mut actual_heading: f32 = if trust_nvs_state {
        let h = infra::SnapshotStore::new(&mut nvs, persist_nvs)
            .load_heading_or_init(90.0);
        info!("Restored heading from NVS: {}", h);
        h
    } else {
        info!("Skipping heading restore: NVS state untrusted");
        90.0
    };

    // Restore encoder adjusted ticks snapshot (version-gated) in EncoderGuarded mode only.
    let mut restored_from_snapshot = false;
    if trust_nvs_state && motion_mode == MotionMode::EncoderGuarded {
        if let Some(enc_ticks_adj) =
            infra::SnapshotStore::new(&mut nvs, persist_nvs)
                .load_encoder_snapshot()
        {
            // Choose the zero offset so that: adjusted = raw - offset == enc_ticks_adj
            // => offset = raw - enc_ticks_adj
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

    // Keep Motion's internal position consistent for logs/logic
    motion.update_position(actual_heading);

    // PHASE 7: MODE SELECTION (Admin vs Normal)--------------------------------------------------
    if active_mode == RunMode::Admin {
        admin_mode::run(
            sw.device_id,
            formatted_time.clone(),
            &sw.admin,
            &mut motion,
            &sw.boot.recovery,
            &sw.boot.homing,
            &mut nvs,
            &mut mqtt,
            &mut wifi,
            &current_version,
            &mut config_manager,
            publish_mqtt,
            persist_nvs,
            false,
        )?;
        return Ok(());
    }

    // PHASE 8: NORMAL MODE BOOT ACTIONS--------------------------------------------------
    // Boot recovery is disabled for production deployment

    // Initialize encoder fault recovery early (needed for homing failure handling)
    let mut encoder_fault = app::encoder_fault::EncoderFaultRecovery::new();
    let encoder_daily_mode = infra::SnapshotStore::new(&mut nvs, persist_nvs).load_encoder_daily_mode();
    encoder_fault.set_mode_switched_daily(encoder_daily_mode);

    // Buttons: maintenance toggles Normal <-> Custom this boot; East/West for future use
    let mut buttons = buttons::Buttons::new(
        peripherals.pins.gpio5,  // Maintenance
        peripherals.pins.gpio4,  // East
        peripherals.pins.gpio6,  // West
    );

    // Homing: Find limit switch to establish known home position
    // Decision logic:
    // - StepperOnly: Always home (no encoder to trust)
    // - EncoderGuarded: Home if no snapshot restored OR NVS state untrusted
    let homing_dir = match sw.boot.homing.dir {
        switchboard::Direction::Cw => Direction::Cw,
        switchboard::Direction::Ccw => Direction::Ccw,
    };

    let should_home_by_mode =
        motion_mode == MotionMode::StepperOnly || !restored_from_snapshot || !trust_nvs_state;
    if should_home_by_mode && sw.boot.homing.enabled {
        let limit_sw_status = match homing_dir {
            Direction::Cw => motion.find_limit_switch_cw(),
            Direction::Ccw => motion.find_limit_switch_ccw(),
        };
        match limit_sw_status {
            true => log::info!(
                "Homing OK (dir={}): limit switch found",
                homing_dir.as_str()
            ),
            false => {
                log::error!(
                    "Homing FAILED (dir={}): limit switch could not be found",
                    homing_dir.as_str()
                );
                infra::Telemetry::critical_failure_loop(
                    sw.device_id,
                    &mut mqtt,
                    b"Critical failure: Limit switch failure!",
                    publish_mqtt,
                );
            }
        }
        // After a successful homing run, re-seed NVS with a clean baseline.
        if persist_nvs {
            infra::SnapshotStore::new(&mut nvs, true).save_heading(90.0);
            if motion_mode == MotionMode::EncoderGuarded {
                infra::SnapshotStore::new(&mut nvs, true)
                    .save_encoder_snapshot(motion.encoder_ticks_adjusted());
            }
        }
        thread::sleep(Duration::from_secs(5));
    } else if should_home_by_mode {
        log::warn!("Homing skipped: switchboard_state.current().boot.homing.enabled=false");
        if !trust_nvs_state {
            infra::Telemetry::critical_failure_loop(
                sw.device_id,
                &mut mqtt,
                b"Critical failure: NVS state untrusted but homing disabled!",
                publish_mqtt,
            );
        }
    } else {
        log::info!("Skipping homing: restored heading+encoder snapshot from NVS");
    }

    // Mark this boot as Normal so next boot can trust NVS again (if persistence enabled)
    infra::SnapshotStore::new(&mut nvs, true).save_last_run_normal(true);

    // PHASE 9: MAIN TRACKING LOOP (Normal Mode Only)--------------------------------------------------
    // Note: encoder_fault was already initialized before homing (above)
    
    // Track previous motion mode to detect switches to StepperOnly
    let mut previous_motion_mode = motion_mode;
    let mut need_rehome_stepper_only = false;
    let mut last_cmd_poll = Instant::now();

    loop {
        buttons.tick();

        // Maintenance button: Normal <-> Custom this boot. Custom -> Normal also erases NVS and restarts.
        if buttons.is_maintenance_pressed() {
            if switchboard_state.active_profile() == Profile::Custom {
                switchboard_state.set_profile(Profile::Normal);
                info!("Switching back to Normal: erasing NVS and restarting");
                erase_nvs_and_restart();
            } else {
                switchboard_state.set_profile(Profile::Custom);
                info!("Switched to Custom mode (this boot)");
            }
        }

        // Custom mode: run Custom-specific tick and sleep (no normal tracking path)
        if switchboard_state.active_profile() == Profile::Custom {
            let st_now = SystemTime::now();
            let dt_now_utc: DateTime<Utc> = st_now.into();
            let local_time: DateTime<FixedOffset> = DateTime::from_naive_utc_and_offset(
                dt_now_utc.naive_utc(),
                FixedOffset::east_opt(timezone_offset_hours * 3600).unwrap(),
            );
            let current_datetime = format!("{}", local_time.format("%d/%m/%Y %H:%M:%S"));
            let _ = app::custom_mode::tick(
                &mut motion,
                &mut calculation,
                &mut actual_heading,
                &mut mqtt,
                &current_version,
                &mut nvs,
                &mut wifi,
                current_datetime,
                switchboard_state.current().effects.publish_mqtt,
                switchboard_state.current().effects.persist_nvs,
                switchboard_state.current().effects.allow_ota,
            );
            // Sleep in 1s chunks so maintenance button (to exit Custom) is checked often
            let sleep_secs = switchboard_state.current().runtime.tracking.loop_sleep_secs;
            for _ in 0..sleep_secs {
                buttons.tick();
                if buttons.is_maintenance_pressed() {
                    switchboard_state.set_profile(Profile::Normal);
                    info!("Switching back to Normal: erasing NVS and restarting");
                    erase_nvs_and_restart();
                }
                std::thread::sleep(Duration::from_secs(1));
            }
            continue;
        }

        let st_now = SystemTime::now();
        let dt_now_utc: DateTime<Utc> = st_now.into();
        let local_time: DateTime<FixedOffset> = DateTime::from_naive_utc_and_offset(
            dt_now_utc.naive_utc(),
            FixedOffset::east_opt(timezone_offset_hours * 3600).unwrap(),
        );
        let current_datetime = format!("{}", local_time.format("%d/%m/%Y %H:%M:%S"));

        // Responsive MQTT command drain (tracking still runs on loop_sleep interval)
        let loop_now = Instant::now();
        if last_cmd_poll.elapsed() >= Duration::from_millis(200) {
            last_cmd_poll = loop_now;
            if wifi.state() == WifiState::Disconnected {
                warn!("Wifi disconnected, attempting to reconnect...");
                let _ = wifi.reconnect_if_disconnected();
            }
            let mut commands_processed = 0u32;
            while commands_processed < 10 {
                match cmd_handler::CommandHandler::process_one(
                    &mut mqtt,
                    &mut motion,
                    &mut nvs,
                    &mut wifi,
                    &current_version,
                    &sw.admin,
                    &sw.boot.recovery,
                    &sw.boot.homing,
                    &mut config_manager,
                    &mut actual_heading,
                    publish_mqtt,
                    persist_nvs,
                    sw.device_id,
                    &current_datetime,
                ) {
                    Ok(true) => commands_processed += 1,
                    Ok(false) => break,
                    Err(e) => {
                        warn!("Command processing error: {:?}", e);
                        break;
                    }
                }
            }
        }

        // Check for daily encoder mode reset (at start of each loop iteration)
        let reset_occurred = check_daily_encoder_reset(&mut nvs, &local_time, persist_nvs);
        if reset_occurred {
            // If reset occurred, update motion mode and encoder fault state
            motion_mode = infra::SnapshotStore::new(&mut nvs, persist_nvs)
                .load_tracking_mode_or_init(MotionMode::EncoderGuarded);
            motion.set_motion_mode(motion_mode);
            encoder_fault.set_mode_switched_daily(false);
            let mode_str = match motion_mode {
                MotionMode::StepperOnly => "StepperOnly",
                MotionMode::EncoderGuarded => "EncoderGuarded",
            };
            info!("Daily reset: Motion mode updated to {}", mode_str);
        }
        
        // Detect if we just switched to StepperOnly mode (need to re-home)
        if motion_mode == MotionMode::StepperOnly && previous_motion_mode != MotionMode::StepperOnly {
            info!("Motion mode switched to StepperOnly - re-homing required");
            need_rehome_stepper_only = true;
        }
        previous_motion_mode = motion_mode;
        
        // If we need to re-home (switched to StepperOnly), do it now before tracking
        if need_rehome_stepper_only && motion_mode == MotionMode::StepperOnly {
            info!("StepperOnly mode detected - re-homing to establish known position");
            let limit_sw_status = match homing_dir {
                Direction::Cw => motion.find_limit_switch_cw(),
                Direction::Ccw => motion.find_limit_switch_ccw(),
            };
            match limit_sw_status {
                true => {
                    info!("Re-homing OK (dir={}): limit switch found", homing_dir.as_str());
                    // Update actual heading after successful re-homing
                    actual_heading = 90.0;
                    motion.update_position(actual_heading);
                    if persist_nvs {
                        infra::SnapshotStore::new(&mut nvs, persist_nvs).save_heading(actual_heading);
                    }
                    need_rehome_stepper_only = false;  // Clear flag after successful homing
                }
                false => {
                    error!("Re-homing FAILED (dir={}): limit switch could not be found", homing_dir.as_str());
                    infra::Telemetry::critical_failure_loop(
                        sw.device_id,
                        &mut mqtt,
                        b"Critical failure: Limit switch failure!",
                        publish_mqtt,
                    );
                }
            }
            thread::sleep(Duration::from_secs(2));  // Brief pause after homing attempt
            continue;  // Skip tracking this iteration, try again next loop
        }

        info!("Actual Heading: {}", motion.location());
        info!("Current datetime: {}", current_datetime.clone());

        let now = std::time::Instant::now();  // Timer to measure loop iteration duration

        // Check for encoder faults (stalls, unplugged, etc.)
        // If fault active, skip tracking and continue to next iteration
        // Also reload motion_mode from NVS in case it was switched to StepperOnly
        let current_motion_mode = infra::SnapshotStore::new(&mut nvs, persist_nvs)
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
        if encoder_fault.tick(
            &encoder_recovery_cfg,
            &mut motion,
            motion_mode,
            &mut actual_heading,
            &mut nvs,
            &mut mqtt,
            &mut wifi,
            &current_version,
            publish_mqtt,
            persist_nvs,
            sw.device_id,
            sw.home_heading_deg,
        )? {
            continue;  // Fault active, skip tracking this iteration
        }
        
        // After encoder_fault.tick(), check again if mode was switched (it might switch during tick)
        let updated_motion_mode = infra::SnapshotStore::new(&mut nvs, persist_nvs)
            .load_tracking_mode_or_init(MotionMode::EncoderGuarded);
        if updated_motion_mode != motion_mode {
            motion_mode = updated_motion_mode;
            motion.set_motion_mode(motion_mode);
            if motion_mode == MotionMode::StepperOnly {
                info!("Motion mode switched to StepperOnly during encoder fault recovery - will re-home");
                need_rehome_stepper_only = true;
            }
        }
        
        // CRITICAL: If we need to re-home (switched to StepperOnly), do it NOW before tracking
        // This must happen AFTER encoder_fault.tick() because mode switches happen during tick()
        if need_rehome_stepper_only && motion_mode == MotionMode::StepperOnly {
            info!("StepperOnly mode detected - re-homing to establish known position (CCW)");
            let limit_sw_status = match homing_dir {
                Direction::Cw => motion.find_limit_switch_cw(),
                Direction::Ccw => motion.find_limit_switch_ccw(),
            };
            match limit_sw_status {
                true => {
                    info!("Re-homing OK (dir={}): limit switch found", homing_dir.as_str());
                    // Update actual heading after successful re-homing
                    actual_heading = 90.0;
                    motion.update_position(actual_heading);
                    if persist_nvs {
                        infra::SnapshotStore::new(&mut nvs, persist_nvs).save_heading(actual_heading);
                    }
                    need_rehome_stepper_only = false;  // Clear flag after successful homing
                }
                false => {
                    error!("Re-homing FAILED (dir={}): limit switch could not be found", homing_dir.as_str());
                    infra::Telemetry::critical_failure_loop(
                        sw.device_id,
                        &mut mqtt,
                        b"Critical failure: Limit switch failure!",
                        publish_mqtt,
                    );
                }
            }
            thread::sleep(Duration::from_secs(2));  // Brief pause after homing attempt
            continue;  // Skip tracking this iteration, try again next loop
        }

        // Perform solar tracking: calculate sun position and move tower
        if sw.runtime.tracking.enabled {
            let outcome = app::tracking_loop::tick(
                &mut motion,
                &mut calculation,
                &mut actual_heading,
                &mut mqtt,
                &current_version,
                &mut nvs,
                &mut wifi,
                current_datetime.clone(),
                publish_mqtt,
                persist_nvs,
                allow_ota,
                sw.device_id,
            );

            if outcome != MoveOutcome::Completed {
                warn!("Last move aborted: {:?}", outcome);
            }
            // Update encoder fault state based on move outcome
            // This may switch to StepperOnly if 3 consecutive failures occur
            if let Err(e) = encoder_fault.on_move_outcome(outcome, &encoder_recovery_cfg, &mut motion, &mut nvs, persist_nvs) {
                error!("Error in encoder fault recovery: {:?}", e);
            }
        } else {
            info!("Tracking disabled");
        }

        info!("Tracking loop duration: {:?}", now.elapsed());
        
        // Housekeeping: Check WiFi connection and publish telemetry
        if wifi.state() == WifiState::Disconnected {
            warn!("Wifi disconnected, attempting to reconnect...");
            wifi.reconnect_if_disconnected()?;
        }
        infra::Telemetry::publish_firmware_version_if(
            sw.device_id,
            &mut mqtt,
            formatted_time.clone(),
            &current_version,
            publish_mqtt,
        );

        std::thread::sleep(Duration::from_secs(
            sw.runtime.tracking.loop_sleep_secs.max(1),
        ));

    }
}

fn boot_diagnostic(device_id: &str, wifi: &mut Wifi, mqtt: &mut Mqtt, publish_mqtt: bool) -> bool {
    // Let system settle before validation
    info!("Starting boot validation in 5 seconds...");
    thread::sleep(Duration::from_secs(5));

    // Check WiFi connectivity
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
            thread::sleep(Duration::from_millis(3000));
            waited += 3000;
        }

        if !mqtt.is_connected() {
            warn!("MQTT not connected yet, retrying...");
            continue;
        }

        // Try publishing a test message
        if infra::Telemetry::publish_boot_check_if(device_id, mqtt, publish_mqtt) {
            return true;
        }
        error!("MQTT publish failed immediately");
        if attempt == MAX_RETRIES {
            error!("All MQTT boot diagnostic attempts failed...");
            return false; // give up after max retries
        }
        thread::sleep(Duration::from_millis(1000)); // backoff
        continue;
    }
    return false; 
}

/// Check if daily encoder mode reset is needed and perform it if necessary.
/// Returns true if a reset occurred, false otherwise.
fn check_daily_encoder_reset<T: esp_idf_svc::nvs::NvsPartitionId>(
    nvs: &mut esp_idf_svc::nvs::EspNvs<T>,
    local_time: &DateTime<FixedOffset>,
    persist_nvs: bool,
) -> bool {
    let mut snapshot_store = infra::SnapshotStore::new(nvs, persist_nvs);
    
    // Check if encoder was switched to daily mode
    let encoder_daily_mode = snapshot_store.load_encoder_daily_mode();
    if !encoder_daily_mode {
        return false;  // Not in daily mode, no reset needed
    }
    
    // Get current date (YYYY-MM-DD format)
    let current_date = local_time.format("%Y-%m-%d").to_string();
    
    // Get stored reset date
    let stored_date = snapshot_store.load_encoder_mode_reset_date();
    
    match stored_date {
        Some(stored) if stored != current_date => {
            // New day detected - reset to EncoderGuarded
            info!("Daily reset: New day detected (stored={}, current={}), resetting encoder mode to EncoderGuarded", stored, current_date);
            
            // Reset tracking mode to EncoderGuarded
            snapshot_store.save_tracking_mode(MotionMode::EncoderGuarded);
            
            // Clear daily mode flag
            snapshot_store.save_encoder_daily_mode(false);
            
            // Update reset date to current date
            snapshot_store.save_encoder_mode_reset_date(&current_date);
            
            true
        }
        Some(_stored) => {
            // Same day, no reset needed
            false
        }
        None => {
            // No stored date (shouldn't happen if daily_mode is true, but handle gracefully)
            warn!("encoder_daily_mode is true but no reset_date found in NVS; initializing reset_date");
            snapshot_store.save_encoder_mode_reset_date(&current_date);
            false
        }
    }
}
