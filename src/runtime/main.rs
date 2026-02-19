use std::time::{Duration, SystemTime};
use chrono::{DateTime, FixedOffset, Utc};
use clock::Clock;
use log::{error, info, warn};
use std::thread;

#[path = "../config.rs"]
mod config;
#[path = "../constants.rs"]
mod constants;
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
        gpio::PinDriver,
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

use crate::app::encoder_fault::{EncoderFaultRecovery, Direction, EncoderRecoverySwitches};
use crate::constants::RunMode;

fn main() -> anyhow::Result<()> {
    
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

    // Production deployment: Always Normal mode, no runtime mode switching
    let active_mode = RunMode::Normal;
    
    let last_run_normal = infra::SnapshotStore::new(&mut nvs, true)
        .load_last_run_normal_or_init(true);
    let trust_nvs_state = last_run_normal;  // Always trust NVS in Normal mode
    info!(
        "Last run normal={} -> trust_nvs_state={} (active_mode=Normal)",
        last_run_normal, trust_nvs_state
    );

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

    use crate::constants::{WIFI_SSID, WIFI_PASSWORD, TIMEZONE_OFFSET_HOURS, MQTT_USER, MQTT_PASSWORD};
    const PERSIST_NVS: bool = true;  // Always enabled in production
    
    let mqtt_user = MQTT_USER;
    if PERSIST_NVS {
        match nvs.set_str("mqtt_user", mqtt_user){
            Ok(_) => info!("Mqtt username updated"),
            Err(e) => error!("Mqtt username not updated {:?}", e),
        };
    }
    let mqtt_pass = MQTT_PASSWORD;
    if PERSIST_NVS {
        match nvs.set_str("mqtt_pass", mqtt_pass){
            Ok(_) => info!("Mqtt password updated"),
            Err(e) => error!("Mqtt password not updated {:?}", e),
        };
    }

    let wifi_ssid = WIFI_SSID;
    if PERSIST_NVS {
        match nvs.set_str("wifi_ssid", wifi_ssid){
            Ok(_) => info!("Wifi ssid updated"),
            Err(e) => error!("Wifi ssid not updated {:?}", e),
        };
    }
    let wifi_pass = WIFI_PASSWORD;
    if PERSIST_NVS {
        match nvs.set_str("wifi_pass", wifi_pass){
            Ok(_) => info!("Wifi password updated"),
            Err(e) => error!("Wifi password not updated {:?}", e),
        };
    }

    let offset_hours = TIMEZONE_OFFSET_HOURS as i32;
    if PERSIST_NVS {
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

    // PHASE 3: BOOT VALIDATION--------------------------------------------------
    
    let first_boot = nvs.get_u8("first_boot")?.unwrap_or(1);

    const ALLOW_BOOT_VALIDATION: bool = true;  // Always enabled in production
    const PUBLISH_MQTT: bool = true;  // Always enabled in production
    
    // Run boot diagnostics (WiFi/MQTT connectivity check)
    let boot_diagnostic_result = if ALLOW_BOOT_VALIDATION {
        boot_diagnostic(&mut wifi, &mut mqtt, PUBLISH_MQTT)
    } else {
        info!("Boot validation disabled");
        true
    };

    // On first boot after OTA update, validate firmware before marking as valid
    // If validation fails, rollback to previous firmware version
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
    const DEFAULT_VERSION: &str = "1.0.4";
    
    // Store current firmware version in NVS
    if PERSIST_NVS {
        nvs.set_str("version", "1.0.4")?;
    }

    // Read firmware version from NVS (or use default)
    let current_version: Version = nvs
        .get_str("version", &mut version_buf)?
        .map(|s| s.trim().parse::<Version>())
        .transpose()?
        .unwrap_or_else(|| Version::parse(DEFAULT_VERSION).unwrap());

    const ALLOW_OTA: bool = true;  // Always enabled in production
    
    // Publish firmware version to MQTT
    infra::Telemetry::publish_firmware_version_if(
        &mut mqtt,
        &current_version,
        PUBLISH_MQTT,
    );

    // Check for OTA updates
    let mut updater = OtaUpdater::new_ota(
        current_version.clone(),
        &mut mqtt,
        Some("device1A"),
        Some("device1A")
    ).expect("Failed to create OTA updater instance");

    if ALLOW_OTA {
        info!("Checking for new OTA update in 3 seconds...");
        thread::sleep(Duration::from_secs(3));
        updater.run_version_compare(&mut nvs)?;
    } else {
        info!("OTA disabled: skipping version compare");
    }
    
    // PHASE 5: MOTION INITIALIZATION--------------------------------------------------
    
    let tower_latitude: f64 = 32.797868;
    if PERSIST_NVS {
        match nvs.set_str("tower_latitude", &tower_latitude.to_string()) {
            Ok(_) => info!("Tower latitude has been updated"),
            Err(e) => error!("Tower latitude was not updated {:?}", e),
        };
    }

    let tower_longitude: f64 = -96.835597;
    if PERSIST_NVS {
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

    // Apply runtime guardrails (stall detection, soft limits) - always enabled in production
    use crate::constants::{SOFT_LIMIT_MIN_DEG, SOFT_LIMIT_MAX_DEG};
    motion.set_stall_detection_enabled(true);
    motion.set_soft_limits(
        true,
        SOFT_LIMIT_MIN_DEG,
        SOFT_LIMIT_MAX_DEG,
    );

    const POWER_ON: bool = true;  // Motor power control (future: sense input)

    // Check for daily encoder mode reset (before loading tracking mode)
    check_daily_encoder_reset(&mut nvs, &local_time, PERSIST_NVS);

    // Select motion mode: Load from NVS, default to EncoderGuarded
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

    // PHASE 6: STATE RESTORATION--------------------------------------------------
    
    let mut actual_heading: f32 = if trust_nvs_state {
        let h = infra::SnapshotStore::new(&mut nvs, PERSIST_NVS)
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
            infra::SnapshotStore::new(&mut nvs, PERSIST_NVS)
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

    // PHASE 7: NORMAL MODE BOOT ACTIONS--------------------------------------------------
    // Boot recovery is disabled for production deployment

    // Initialize encoder fault recovery early (needed for homing failure handling)
    let mut encoder_fault = app::encoder_fault::EncoderFaultRecovery::new();
    let encoder_daily_mode = infra::SnapshotStore::new(&mut nvs, PERSIST_NVS).load_encoder_daily_mode();
    encoder_fault.set_mode_switched_daily(encoder_daily_mode);

    // Initialize button inputs (for future manual control)
    let _mb = PinDriver::input(peripherals.pins.gpio5).unwrap(); // Maintenance
    let _eb = PinDriver::input(peripherals.pins.gpio4).unwrap(); // East Button
    let _wb = PinDriver::input(peripherals.pins.gpio6).unwrap(); // West Button

    // Homing: Find limit switch to establish known home position
    // Decision logic:
    // - StepperOnly: Always home (no encoder to trust)
    // - EncoderGuarded: Home if no snapshot restored OR NVS state untrusted
    const HOMING_ENABLED: bool = true;  // Always enabled in production
    const HOMING_DIRECTION: Direction = Direction::Ccw;  // Default direction
    
    let should_home_by_mode =
        motion_mode == MotionMode::StepperOnly || !restored_from_snapshot || !trust_nvs_state;
    if should_home_by_mode && HOMING_ENABLED {
        let limit_sw_status = match HOMING_DIRECTION {
            Direction::Cw => motion.find_limit_switch_cw(),
            Direction::Ccw => motion.find_limit_switch_ccw(),
        };
        match limit_sw_status {
            true => log::info!(
                "Homing OK (dir={}): limit switch found",
                HOMING_DIRECTION.as_str()
            ),
            false => {
                // In EncoderGuarded mode, homing failure might be due to encoder stall - trigger recovery
                if motion_mode == MotionMode::EncoderGuarded {
                    log::warn!("Homing failed in EncoderGuarded mode - triggering encoder fault recovery");
                    let encoder_recovery_cfg = app::encoder_fault::EncoderRecoverySwitches::default();
                    if let Err(e) = encoder_fault.on_move_outcome(
                        MoveOutcome::AbortedStall,
                        &encoder_recovery_cfg,
                        &mut motion,
                        &mut nvs,
                        PERSIST_NVS,
                    ) {
                        error!("Error triggering encoder fault recovery after homing failure: {:?}", e);
                    }
                    // Don't enter critical failure loop - encoder fault recovery will handle it
                } else {
                    log::error!(
                        "Homing FAILED (dir={}): limit switch could not be found",
                        HOMING_DIRECTION.as_str()
                    );
                    infra::Telemetry::critical_failure_loop(
                        &mut mqtt,
                        b"Critical failure: Limit switch failure!",
                        PUBLISH_MQTT,
                    );
                }
            }
        }
        // After a successful homing run, re-seed NVS with a clean baseline.
        if PERSIST_NVS {
            infra::SnapshotStore::new(&mut nvs, true).save_heading(90.0);
            if motion_mode == MotionMode::EncoderGuarded {
                infra::SnapshotStore::new(&mut nvs, true)
                    .save_encoder_snapshot(motion.encoder_ticks_adjusted());
            }
        }
        thread::sleep(Duration::from_secs(5));
    } else if should_home_by_mode {
        log::warn!("Homing skipped: HOMING_ENABLED=false");
        if !trust_nvs_state {
            infra::Telemetry::critical_failure_loop(
                &mut mqtt,
                b"Critical failure: NVS state untrusted but homing disabled!",
                PUBLISH_MQTT,
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

    loop {
        let st_now = SystemTime::now();
        let dt_now_utc: DateTime<Utc> = st_now.into();
        let local_time: DateTime<FixedOffset> = DateTime::from_naive_utc_and_offset(
            dt_now_utc.naive_utc(),
            FixedOffset::east_opt(timezone_offset_hours * 3600).unwrap(),
        );
        let current_datetime = format!("{}", local_time.format("%d/%m/%Y %H:%M:%S"));

        // Check for daily encoder mode reset (at start of each loop iteration)
        let reset_occurred = check_daily_encoder_reset(&mut nvs, &local_time, PERSIST_NVS);
        if reset_occurred {
            // If reset occurred, update motion mode and encoder fault state
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
        
        // Detect if we just switched to StepperOnly mode (need to re-home)
        if motion_mode == MotionMode::StepperOnly && previous_motion_mode != MotionMode::StepperOnly {
            info!("Motion mode switched to StepperOnly - re-homing required");
            need_rehome_stepper_only = true;
        }
        previous_motion_mode = motion_mode;
        
        // If we need to re-home (switched to StepperOnly), do it now before tracking
        if need_rehome_stepper_only && motion_mode == MotionMode::StepperOnly {
            info!("StepperOnly mode detected - re-homing to establish known position");
            const HOMING_DIRECTION: Direction = Direction::Ccw;
            let limit_sw_status = match HOMING_DIRECTION {
                Direction::Cw => motion.find_limit_switch_cw(),
                Direction::Ccw => motion.find_limit_switch_ccw(),
            };
            match limit_sw_status {
                true => {
                    info!("Re-homing OK (dir={}): limit switch found", HOMING_DIRECTION.as_str());
                    // Update actual heading after successful re-homing
                    actual_heading = 90.0;
                    motion.update_position(actual_heading);
                    if PERSIST_NVS {
                        infra::SnapshotStore::new(&mut nvs, PERSIST_NVS).save_heading(actual_heading);
                    }
                    need_rehome_stepper_only = false;  // Clear flag after successful homing
                }
                false => {
                    error!("Re-homing FAILED (dir={}): limit switch could not be found", HOMING_DIRECTION.as_str());
                    // Don't enter critical failure loop - continue and try again next iteration
                    // Keep flag set to try again
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
        
        let encoder_recovery_cfg = EncoderRecoverySwitches::default();
        if encoder_fault.tick(
            &encoder_recovery_cfg,
            &mut motion,
            motion_mode,
            &mut actual_heading,
            &mut nvs,
            &mut mqtt,
            &mut wifi,
            &current_version,
            PUBLISH_MQTT,
            PERSIST_NVS,
        )? {
            continue;  // Fault active, skip tracking this iteration
        }
        
        // After encoder_fault.tick(), check again if mode was switched (it might switch during tick)
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
        
        // CRITICAL: If we need to re-home (switched to StepperOnly), do it NOW before tracking
        // This must happen AFTER encoder_fault.tick() because mode switches happen during tick()
        if need_rehome_stepper_only && motion_mode == MotionMode::StepperOnly {
            info!("StepperOnly mode detected - re-homing to establish known position (CCW)");
            const HOMING_DIRECTION: Direction = Direction::Ccw;  // Always home CCW
            let limit_sw_status = match HOMING_DIRECTION {
                Direction::Cw => motion.find_limit_switch_cw(),
                Direction::Ccw => motion.find_limit_switch_ccw(),
            };
            match limit_sw_status {
                true => {
                    info!("Re-homing OK (dir={}): limit switch found", HOMING_DIRECTION.as_str());
                    // Update actual heading after successful re-homing
                    actual_heading = 90.0;
                    motion.update_position(actual_heading);
                    if PERSIST_NVS {
                        infra::SnapshotStore::new(&mut nvs, PERSIST_NVS).save_heading(actual_heading);
                    }
                    need_rehome_stepper_only = false;  // Clear flag after successful homing
                }
                false => {
                    error!("Re-homing FAILED (dir={}): limit switch could not be found", HOMING_DIRECTION.as_str());
                    // Don't enter critical failure loop - continue and try again next iteration
                    // Keep flag set to try again
                }
            }
            thread::sleep(Duration::from_secs(2));  // Brief pause after homing attempt
            continue;  // Skip tracking this iteration, try again next loop
        }

        // Perform solar tracking: calculate sun position and move tower
        const TRACKING_ENABLED: bool = true;  // Always enabled in production
        if TRACKING_ENABLED {
            let outcome = app::tracking_loop::tick(
                &mut motion,
                &mut calculation,
                &mut actual_heading,
                &mut mqtt,
                &current_version,
                &mut nvs,
                &mut wifi,
                current_datetime.clone(),
                PUBLISH_MQTT,
                PERSIST_NVS,
                ALLOW_OTA,
            );

            if outcome != MoveOutcome::Completed {
                warn!("Last move aborted: {:?}", outcome);
            }
            // Update encoder fault state based on move outcome
            // This may switch to StepperOnly if 3 consecutive failures occur
            if let Err(e) = encoder_fault.on_move_outcome(outcome, &encoder_recovery_cfg, &mut motion, &mut nvs, PERSIST_NVS) {
                error!("Error in encoder fault recovery: {:?}", e);
            }
        } else {
            info!("Tracking disabled");
        }

        info!("Tracking loop duration (v1.0.4): {:?}", now.elapsed());
        
        // Housekeeping: Check WiFi connection and publish telemetry
        if wifi.state() == WifiState::Disconnected {
            warn!("Wifi disconnected, attempting to reconnect...");
            wifi.reconnect_if_disconnected()?;
        }
        infra::Telemetry::publish_firmware_version_if(
            &mut mqtt,
            &current_version,
            PUBLISH_MQTT,
        );

        // Sleep before next iteration (default: 300 seconds = 5 minutes)
        const LOOP_SLEEP_SECS: u64 = 300;
        std::thread::sleep(Duration::from_secs(LOOP_SLEEP_SECS));

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
