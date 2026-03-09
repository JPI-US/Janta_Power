use std::time::{Duration, SystemTime};
use chrono::{DateTime, FixedOffset, Utc};
use clock::Clock;
use log::{error, info, warn};
use std::thread;

#[path = "../config.rs"]
mod config;
#[path = "../switchboard.rs"]
mod switchboard;
#[path = "../constants.rs"]
mod constants;
#[path = "../config_manager.rs"]
mod config_manager;
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
        peripherals::Peripherals,
        prelude::*,
        //        task::block_on,
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

use crate::switchboard::{Direction, MotionModePolicy, Profile};
use crate::app::encoder_fault::EncoderFaultRecovery;
use crate::diagnostics::{admin_mode, cmd_handler};
use crate::constants::{get_active_profile, get_active_mode, RunMode};
use config_manager::{ConfigManager, RuntimeMode};

fn main() -> anyhow::Result<()> {
    
    // PHASE 1: INITIALIZATION--------------------------------------------------

    let active_profile = get_active_profile();
    let compile_time_mode = get_active_mode();
    let mut switchboard_state = switchboard::SwitchboardState::new(active_profile);
    
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

    // Initialize runtime configuration manager (handles MQTT-configurable overrides)
    let mut config_manager = ConfigManager::new(&mut nvs, switchboard_state.current().effects.persist_nvs);
    
    // This allows switching modes via MQTT commands
    let runtime_mode = config_manager.get_runtime_mode(&mut nvs);
    let active_mode = match runtime_mode {
        RuntimeMode::Admin => RunMode::Admin,
        RuntimeMode::Normal => compile_time_mode, // Use compile-time if runtime not set
    };
    info!(
        "Mode: compile_time={:?}, runtime={:?}, active={:?}",
        compile_time_mode, runtime_mode, active_mode
    );

    let last_run_normal = infra::SnapshotStore::new(&mut nvs, true)
        .load_last_run_normal_or_init(true);
    let trust_nvs_state = match active_mode {
        RunMode::Normal => last_run_normal,  // Only trust NVS if last run was Normal
        RunMode::Admin => false,             // Never trust NVS in Admin mode
    };
    info!(
        "Last run normal={} -> trust_nvs_state={} (active_mode={:?})",
        last_run_normal, trust_nvs_state, active_mode
    );

    // Admin mode was last = untrusted
    if active_mode == RunMode::Admin {
        infra::SnapshotStore::new(&mut nvs, true).save_last_run_normal(false);
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

    use crate::constants::{WIFI_SSID, WIFI_PASSWORD, TIMEZONE_OFFSET_HOURS};
    let mqtt_user = "device1A";
    if switchboard_state.current().effects.persist_nvs {
        match nvs.set_str("mqtt_user", mqtt_user){
            Ok(_) => info!("Mqtt username updated"),
            Err(e) => error!("Mqtt username not updated {:?}", e),
        };
    }
    let mqtt_pass= "device1A";
    if switchboard_state.current().effects.persist_nvs {
        match nvs.set_str("mqtt_pass", mqtt_pass){
            Ok(_) => info!("Mqtt password updated"),
            Err(e) => error!("Mqtt password not updated {:?}", e),
        };
    }

    let wifi_ssid = WIFI_SSID;
    if switchboard_state.current().effects.persist_nvs {
        match nvs.set_str("wifi_ssid", wifi_ssid){
            Ok(_) => info!("Wifi ssid updated"),
            Err(e) => error!("Wifi ssid not updated {:?}", e),
        };
    }
    let wifi_pass = WIFI_PASSWORD;
    if switchboard_state.current().effects.persist_nvs {
        match nvs.set_str("wifi_pass", wifi_pass){
            Ok(_) => info!("Wifi password updated"),
            Err(e) => error!("Wifi password not updated {:?}", e),
        };
    }

    let offset_hours = TIMEZONE_OFFSET_HOURS as i32;
    if switchboard_state.current().effects.persist_nvs {
        match nvs.set_i32("offset_hours", offset_hours){
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
        "mqttS://mqtt.jantaus.com:9443",
        "device1A_pub",
        &real_mqtt_user,
        &real_mqtt_pass,
    )?);

    // Subscribe to command topic for remote control (admin mode, tests, config changes)
    cmd_handler::CommandHandler::init(&mut mqtt)?;

    // PHASE 3: BOOT VALIDATION--------------------------------------------------
    
    let first_boot = nvs.get_u8("first_boot")?.unwrap_or(1);

    // Run boot diagnostics (WiFi/MQTT connectivity check)
    let boot_diagnostic_result = if switchboard_state.current().effects.allow_boot_validation {
        boot_diagnostic(&mut wifi, &mut mqtt, switchboard_state.current().effects.publish_mqtt)
    } else {
        info!("Boot validation disabled by switchboard");
        true
    };

    // On first boot after OTA update, validate firmware before marking as valid
    // If validation fails, rollback to previous firmware version
    if switchboard_state.current().effects.allow_boot_validation && first_boot == 1 {
        info!("First boot, now performing boot diagnostics");
        let mut valid_ota = EspOta::new().expect("Failed to get OTA instance");// Minimal OTA instance for validation

        let running_slot = valid_ota.get_running_slot();
        info!("This is the running boot slot {:?}", running_slot);

        if running_slot.unwrap().label == "factory" {
            info!("Running from factory partition -> skipping OTA validity marking");
            nvs.set_u8("first_boot", 0)?;
        } else{
            // Mark firmware valid or rollback
            if boot_diagnostic_result {
                info!("Boot validation passed, now marking firmware as valid");
                valid_ota.mark_running_slot_valid()?;
                nvs.set_u8("first_boot", 0)?;
                
            } else {
                error!("Boot validation failed, rolling back firmware");
                valid_ota.mark_running_slot_invalid_and_reboot(); // reboots immediately
            }
        }
    }else {
        info!("Normal boot firmware already validated");
    }

    
    // PHASE 4: FIRMWARE VERSION & OTA--------------------------------------------------


    let mut version_buf = [0u8; 32];
    const DEFAULT_VERSION: &str = "1.0.7";
    
    // Store current firmware version in NVS
    if switchboard_state.current().effects.persist_nvs {
        nvs.set_str("version", "1.0.7")?;
    }

    // Read firmware version from NVS (or use default)
    let current_version: Version = nvs
        .get_str("version", &mut version_buf)?
        .map(|s| s.trim().parse::<Version>())
        .transpose()?
        .unwrap_or_else(|| Version::parse(DEFAULT_VERSION).unwrap());

    // Publish firmware version to MQTT (if enabled)
    infra::Telemetry::publish_firmware_version_if(
        &mut mqtt,
        &current_version,
        switchboard_state.current().effects.publish_mqtt,
    );

    // Check for OTA updates (if enabled)
    let mut updater = OtaUpdater::new_ota(
        current_version.clone(),
        &mut mqtt,
        Some("device1A"),
        Some("device1A")
    ).expect("Failed to create OTA updater instance");

    if switchboard_state.current().effects.allow_ota {
        info!("Checking for new OTA update in 3 seconds...");
        thread::sleep(Duration::from_secs(3));
        updater.run_version_compare(&mut nvs)?;
    } else {
        info!("OTA disabled by switchboard: skipping version compare");
    }
    
    // PHASE 5: MOTION INITIALIZATION--------------------------------------------------
    
    let tower_latitude: f64 = 32.797868;
    if switchboard_state.current().effects.persist_nvs {
        match nvs.set_str("tower_latitude", &tower_latitude.to_string()) {
            Ok(_) => info!("Tower latitude has been updated"),
            Err(e) => error!("Tower latitude was not updated {:?}", e),
        };
    }

    let tower_longitude: f64 = -96.835597;
    if switchboard_state.current().effects.persist_nvs {
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

    // Apply runtime guardrails (stall detection, soft limits)
    motion.set_stall_detection_enabled(switchboard_state.current().runtime.guardrails.stall_detection_enabled);
    motion.set_soft_limits(
        switchboard_state.current().runtime.guardrails.soft_limits_enabled,
        switchboard_state.current().runtime.guardrails.soft_limit_min_deg,
        switchboard_state.current().runtime.guardrails.soft_limit_max_deg,
    );

    const POWER_ON: bool = true;  // Motor power control (future: sense input)

    // Select motion mode: StepperOnly (open-loop) vs EncoderGuarded (closed-loop)
    // Mode can come from NVS (persisted) or be forced by switchboard
    let motion_mode = match switchboard_state.current().runtime.motion_mode {
        MotionModePolicy::FromNvsDefault(default) => {
            infra::SnapshotStore::new(&mut nvs, switchboard_state.current().effects.persist_nvs)
                .load_tracking_mode_or_init(default)
        }
        MotionModePolicy::Force(forced) => forced,
    };
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
        let h = infra::SnapshotStore::new(&mut nvs, switchboard_state.current().effects.persist_nvs)
            .load_heading_or_init(90.0);
        info!("Restored heading from NVS: {}", h);
        h
    } else {
        info!("Skipping heading restore: NVS state untrusted (last run was Admin)");
        90.0
    };

    // Restore encoder adjusted ticks snapshot (version-gated) in L2 only.
    let mut restored_from_snapshot = false;
    if trust_nvs_state && motion_mode == MotionMode::EncoderGuarded {
        if let Some(enc_ticks_adj) =
            infra::SnapshotStore::new(&mut nvs, switchboard_state.current().effects.persist_nvs)
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
            info!("Skipping encoder snapshot restore: NVS state untrusted (last run was Admin)");
        } else {
            info!("Motion mode is StepperOnly: skipping encoder snapshot restore");
        }
    }

    // Keep Motion's internal position consistent for logs/logic
    motion.update_position(actual_heading);

    
    // PHASE 7: MODE SELECTION--------------------------------------------------
    
    if active_mode == RunMode::Admin {
        admin_mode::run(
            &switchboard_state.current().admin,
            &mut motion,
            &switchboard_state.current().boot.recovery,
            &switchboard_state.current().boot.homing,
            &mut nvs,
            &mut mqtt,
            &mut wifi,
            &current_version,
            &mut config_manager,
            switchboard_state.current().effects.publish_mqtt,
            switchboard_state.current().effects.persist_nvs,
            false, // bypass_enabled_check: false for full Admin mode at boot
        )?;
        // Admin mode handles its own loop; exit here to prevent falling into Normal mode
        return Ok(());
    }

    // PHASE 8: NORMAL MODE BOOT ACTIONS--------------------------------------------------
    
    if switchboard_state.current().boot.recovery.enabled {
        diagnostics::boot_recovery::run(&mut motion, &switchboard_state.current().boot.recovery);
    }

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
    let should_home_by_mode =
        motion_mode == MotionMode::StepperOnly || !restored_from_snapshot || !trust_nvs_state;
    if should_home_by_mode && switchboard_state.current().boot.homing.enabled {
        let limit_sw_status = match switchboard_state.current().boot.homing.dir {
            Direction::Cw => motion.find_limit_switch_cw(),
            Direction::Ccw => motion.find_limit_switch_ccw(),
        };
        match limit_sw_status {
            true => log::info!(
                "Homing OK (dir={}): limit switch found",
                switchboard_state.current().boot.homing.dir.as_str()
            ),
            false => {
                log::error!(
                    "Homing FAILED (dir={}): limit switch could not be found",
                    switchboard_state.current().boot.homing.dir.as_str()
                );
                infra::Telemetry::critical_failure_loop(
                    &mut mqtt,
                    b"Critical failure: Limit switch failure!",
                    switchboard_state.current().effects.publish_mqtt,
                );
            }
        }
        // After a successful homing run, re-seed NVS with a clean baseline.
        if switchboard_state.current().effects.persist_nvs {
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
                &mut mqtt,
                b"Critical failure: NVS state untrusted (last run was Admin) but homing disabled!",
                switchboard_state.current().effects.publish_mqtt,
            );
        }
    } else {
        log::info!("Skipping homing: restored heading+encoder snapshot from NVS");
    }

    // Mark this boot as Normal so next boot can trust NVS again (if persistence enabled)
    infra::SnapshotStore::new(&mut nvs, true).save_last_run_normal(true);

    // PHASE 9: MAIN TRACKING LOOP (Normal Mode Only)--------------------------------------------------
    
    let mut encoder_fault = EncoderFaultRecovery::new();

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
        FixedOffset::east_opt(timezone_offset_hours * 3600).unwrap());
        let current_datetime = format!("{}", local_time.format("%d/%m/%Y %H:%M:%S"));

        info!("Actual Heading: {}", motion.location());
        info!("Current datetime: {}", current_datetime.clone());

        let now = std::time::Instant::now();  // Timer to measure loop iteration duration

        // Check for encoder faults (stalls, unplugged, etc.)
        // If fault active, skip tracking and continue to next iteration
        if encoder_fault.tick(
            &switchboard_state.current().runtime.encoder_recovery,
            &mut motion,
            motion_mode,
            &mut actual_heading,
            &mut nvs,
            &mut mqtt,
            &mut wifi,
            &current_version,
            switchboard_state.current().effects.publish_mqtt,
            switchboard_state.current().effects.persist_nvs,
        )? {
            continue;  // Fault active, skip tracking this iteration
        }

        // Perform solar tracking: calculate sun position and move tower
        if switchboard_state.current().runtime.tracking.enabled {
            let outcome = app::tracking_loop::tick(
                &mut motion,
                &mut calculation,
                &mut actual_heading,
                &mut mqtt,
                &current_version,
                &mut nvs,
                &mut wifi,
                current_datetime.clone(),
                switchboard_state.current().effects.publish_mqtt,
                switchboard_state.current().effects.persist_nvs,
                switchboard_state.current().effects.allow_ota,
            );

            if outcome != MoveOutcome::Completed {
                warn!("Last move aborted: {:?}", outcome);
            }
            // Update encoder fault state based on move outcome
            encoder_fault.on_move_outcome(outcome, &switchboard_state.current().runtime.encoder_recovery);
        } else {
            info!("Tracking disabled by switchboard (runtime.tracking.enabled=false)");
        }

        info!("Tracking loop duration (v1.0.7): {:?}", now.elapsed());
        
        // Housekeeping: Check WiFi connection and publish telemetry
        if wifi.state() == WifiState::Disconnected {
            warn!("Wifi disconnected, attempting to reconnect...");
            wifi.reconnect_if_disconnected()?;
        }
        infra::Telemetry::publish_firmware_version_if(
            &mut mqtt,
            &current_version,
            switchboard_state.current().effects.publish_mqtt,
        );

        // Process MQTT commands (non-blocking, processes all queued commands)
        // Process up to 10 commands per iteration to avoid blocking too long
        let mut commands_processed = 0;
        while commands_processed < 10 {
            match cmd_handler::CommandHandler::process_one(
                &mut mqtt,
                &mut motion,
                &mut nvs,
                &mut wifi,
                &current_version,
                &switchboard_state.current().admin,
                &switchboard_state.current().boot.recovery,
                &switchboard_state.current().boot.homing,
                &mut config_manager,
                &mut actual_heading, // Pass actual_heading so tests can update it after re-homing
                switchboard_state.current().effects.publish_mqtt,
                switchboard_state.current().effects.persist_nvs,
            ) {
                Ok(true) => {
                    commands_processed += 1;
                    // Continue processing more commands
                }
                Ok(false) => {
                    // No more commands available
                    break;
                }
                Err(e) => {
                    warn!("Command processing error: {:?}", e);
                    break; // Stop on error to avoid infinite loop
                }
            }
        }
        if commands_processed > 0 {
            info!("Processed {} MQTT command(s) in this iteration", commands_processed);
        }

        // Check if runtime mode changed via MQTT (e.g., set_mode command)
        // Mode changes require reboot to take effect (for safety)
        let current_runtime_mode = config_manager.get_runtime_mode(&mut nvs);
        let should_be_admin = current_runtime_mode == RuntimeMode::Admin;
        if should_be_admin && active_mode == RunMode::Normal {
            warn!("Runtime mode changed to Admin via MQTT. Restart required for mode change to take effect.");
        }
        
        // Sleep before next iteration, checking maintenance button every second so clicks are not missed
        let sleep_secs = switchboard_state.current().runtime.tracking.loop_sleep_secs;
        for _ in 0..sleep_secs {
            buttons.tick();
            if buttons.is_maintenance_pressed() {
                if switchboard_state.active_profile() == Profile::Custom {
                    switchboard_state.set_profile(Profile::Normal);
                    info!("Switching back to Normal: erasing NVS and restarting");
                    erase_nvs_and_restart();
                } else {
                    switchboard_state.set_profile(Profile::Custom);
                    info!("Switched to Custom mode (this boot)");
                }
                break;
            }
            std::thread::sleep(Duration::from_secs(1));
        }
    }
}

/// Erase the default NVS partition and restart the ESP. Next boot will use env defaults.
fn erase_nvs_and_restart() -> ! {
    unsafe {
        // Erase default NVS so next boot has no persisted state
        let _ = esp_idf_svc::sys::nvs_flash_erase();
        esp_idf_svc::sys::esp_restart();
    }
}

fn boot_diagnostic(wifi: &mut Wifi, mqtt: &mut Mqtt, publish_mqtt: bool) -> bool {
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

    // MQTT check
    const MAX_RETRIES: u8 = 3;

    for attempt in 1..=MAX_RETRIES {
        info!("Boot diagnostic MQTT attempt {}/{}", attempt, MAX_RETRIES);

        // Wait until the MQTT client reports connected
        let mut waited = 0;
        while !mqtt.is_connected() && waited < 12000 {
            thread::sleep(Duration::from_millis(3000));
            waited += 3000;
        }

        if !mqtt.is_connected() {
            warn!("MQTT not connected yet, retrying...");
            continue; // next attempt
        }

        // Try publishing a test message
        if infra::Telemetry::publish_boot_check_if(mqtt, publish_mqtt) {
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