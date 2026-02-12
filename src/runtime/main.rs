use std::time::{Duration, SystemTime};
use chrono::{DateTime, FixedOffset, Utc};
use clock::Clock;
use log::*;
use std::thread;

#[path = "../config.rs"]
mod config;
#[path = "../switchboard.rs"]
mod switchboard;
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

use crate::switchboard::{Direction, MotionModePolicy, Profile, Switchboard};
use crate::app::encoder_fault::EncoderFaultRecovery;
use crate::diagnostics::{admin_mode, cmd_handler};
use crate::constants::{get_active_profile, get_active_mode, RunMode};

fn main() -> anyhow::Result<()> {
    // ================= Initialize from .env constants =================
    // Profile and mode are selected from .env file at compile time
    // Values are read from .env and converted to enums at runtime
    let active_profile = get_active_profile();
    let active_mode = get_active_mode();
    let switchboard = switchboard::active(active_profile);
    // Required for ESP-IDF patches
    esp_idf_svc::sys::link_patches();

    // Initialize logger and system loop
    EspLogger::initialize_default();
    let sysloop = EspSystemEventLoop::take()?;

    // Initialize peripherals and nvs
    let peripherals = Peripherals::take().unwrap();
    let nvs_default = EspDefaultNvsPartition::take()?;
    let mut nvs = match EspNvs::new(nvs_default.clone(), "storage", true) {
        Ok(nvs) => {
            info!("Got namespace {:?} from default partition", "storage");
            nvs
        }
        Err(e) => panic!("Could't get namespace {:?}", e),
    };

    // Track last run mode to avoid trusting stale state after Admin/bench sessions.
    // Only meaningful when NVS persistence is enabled.
    // Safety flag: we ALWAYS persist this (even if `effects.persist_nvs=false`) so that
    // a manual/admin session cannot accidentally leave Normal mode trusting stale heading/snapshot.
    let last_run_normal = infra::SnapshotStore::new(&mut nvs, true)
        .load_last_run_normal_or_init(true);
    let trust_nvs_state = match active_mode {
        RunMode::Normal => last_run_normal,
        RunMode::Admin => false,
    };
    info!(
        "Last run normal={} -> trust_nvs_state={} (active_mode={:?})",
        last_run_normal, trust_nvs_state, active_mode
    );

    // If we start Admin mode and persistence is enabled, mark NVS state as untrusted for the next Normal boot.
    if active_mode == RunMode::Admin {
        infra::SnapshotStore::new(&mut nvs, true).save_last_run_normal(false);
    }

    // Encoder pins (move them once; pass into Motion::new later)
    let encoder_a = peripherals.pins.gpio47;
    let encoder_b = peripherals.pins.gpio21;

    // Setting of sda and scl gpio pins as well as i2c
    let sda = peripherals.pins.gpio8;
    let scl = peripherals.pins.gpio9;

    // I2C configuration
    let config = I2cConfig::new().baudrate(10_u32.kHz().into());
    let i2c = I2cDriver::new(peripherals.i2c0, sda, scl, &config).unwrap();

    // Setting up i2c bus driver
    let bus: &'static _ = shared_bus::new_std!(I2cDriver = i2c).unwrap();

    // ======== Static credentials: Initialization for wifi and mqtt ========
    // Write credentials into NVS (as raw bits)
    // Note: WiFi credentials come from .env constants
    use crate::constants::{WIFI_SSID, WIFI_PASSWORD, TIMEZONE_OFFSET_HOURS};
    let mqtt_user = "device1A";
    if switchboard.effects.persist_nvs {
        match nvs.set_str("mqtt_user", mqtt_user){
            Ok(_) => info!("Mqtt username updated"),
            Err(e) => error!("Mqtt username not updated {:?}", e),
        };
    }
    let mqtt_pass= "device1A";
    if switchboard.effects.persist_nvs {
        match nvs.set_str("mqtt_pass", mqtt_pass){
            Ok(_) => info!("Mqtt password updated"),
            Err(e) => error!("Mqtt password not updated {:?}", e),
        };
    }

    let wifi_ssid = WIFI_SSID;
    if switchboard.effects.persist_nvs {
        match nvs.set_str("wifi_ssid", wifi_ssid){
            Ok(_) => info!("Wifi ssid updated"),
            Err(e) => error!("Wifi ssid not updated {:?}", e),
        };
    }
    let wifi_pass = WIFI_PASSWORD;
    if switchboard.effects.persist_nvs {
        match nvs.set_str("wifi_pass", wifi_pass){
            Ok(_) => info!("Wifi password updated"),
            Err(e) => error!("Wifi password not updated {:?}", e),
        };
    }

    let offset_hours = TIMEZONE_OFFSET_HOURS as i32;
    if switchboard.effects.persist_nvs {
        match nvs.set_i32("offset_hours", offset_hours){
            Ok(_) => info!("Timezone offset has been updated"),
            Err(e) => error!("Timezone offset was not updated {:?}", e),
        };
    }

    // ======== Wifi: Initialization ========
    let mut buffer = [0u8; 64]; // Adjust size as needed
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
    if wifi.state() == WifiState::Disconnected{
        wifi.reconnect_if_disconnected()?;
    }

    // Initializing ntp and local time
    let ntp = EspSntp::new_default().unwrap();
    info!("Synchronizing with NTP Server");
    while ntp.get_sync_status() != SyncStatus::Completed {}
    info!("Time Sync Completed");

    let st_now = SystemTime::now();
    let dt_now_utc: DateTime<Utc> = st_now.clone().into();
    let timezone_offset_hours: i32 = nvs.get_i32("offset_hours")?.unwrap_or(-5);
    //let timezone_offset_hours: i32 = -5; 
    let local_time: DateTime<FixedOffset> = DateTime::from_naive_utc_and_offset(
        dt_now_utc.naive_utc(),
        //FixedOffset::west_opt(5 * 3600).unwrap(),              // 
        FixedOffset::east_opt(timezone_offset_hours * 3600).unwrap(),  
    );

    let formatted_time = format!("{}", local_time.format("%d/%m/%Y %H:%M:%S"));
    info!("{}", formatted_time);

    // ======== Mqtt: Initialization ========
    let real_mqtt_user = nvs
        .get_str("mqtt_user", &mut buffer)?
        .expect("Mqtt username not found")
        .to_string();
    let real_mqtt_pass = nvs
        .get_str("mqtt_pass", &mut buffer)?
        .expect("Mqtt password not found")
        .to_string();

    // Create MQTT client using the Wi-Fi TCP/IP stack
    let mut mqtt = Box::new(Mqtt::new_mqtt(
        "mqttS://mqtt.jantaus.com:9443",
        "device1A_pub",
        &real_mqtt_user,
        &real_mqtt_pass,
    )?);

    // Initialize command handler (subscribe to command topic for runtime test execution)
    cmd_handler::CommandHandler::init(&mut mqtt)?;

    // ======== Boot Validation ========    
    let first_boot = nvs.get_u8("first_boot")?.unwrap_or(1);

    // Run boot_diagnostic check
    let boot_diagnostic_result = if switchboard.effects.allow_boot_validation {
        boot_diagnostic(&mut wifi, &mut mqtt, switchboard.effects.publish_mqtt)
    } else {
        info!("Boot validation disabled by switchboard");
        true
    };

    if switchboard.effects.allow_boot_validation && first_boot == 1 {
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

    // ======== OTA: Initialization ========
    // Create a version buffer large enough for the version string
    let mut version_buf = [0u8; 32]; // Adjust size as needed

    info!("Setting the firmware version..."); // RESETTING THE FIRMWARE VERSION
    if switchboard.effects.persist_nvs {
        nvs.set_str("version", "1.0.4")?; // RESETTING THE FIRMWARE VERSION
    }

    // Default version if nothing is stored
    const DEFAULT_VERSION: &str = "1.0.4";
    // Read the current version from NVS
    let current_version: Version = nvs
        .get_str("version", &mut version_buf)?
        .map(|s| s.trim().parse::<Version>())
        .transpose()? // converts Option<Result<..>> into Result<Option<..>>
        .unwrap_or_else(|| Version::parse(DEFAULT_VERSION).unwrap());

    // Publishing a message about the current running version (gated)
    infra::Telemetry::publish_firmware_version_if(
        &mut mqtt,
        &current_version,
        switchboard.effects.publish_mqtt,
    );

    // Creates an instance of OTA crate
    let mut updater = OtaUpdater::new_ota(current_version.clone(), &mut mqtt, Some("device1A"), Some("device1A")).expect("Failed to create OTA updater instance");

    // Run version compare
    if switchboard.effects.allow_ota {
        info!("Checking for new OTA update in 3 seconds...");
        thread::sleep(Duration::from_secs(3));
        updater.run_version_compare(&mut nvs)?;
    } else {
        info!("OTA disabled by switchboard: skipping version compare");
    }
    
    // Set tower configuration values
    let tower_latitude: f64 = 32.797868;
    if switchboard.effects.persist_nvs {
        match nvs.set_str("tower_latitude", &tower_latitude.to_string()){
            Ok(_) => info!("Tower latitude has been updated"),
            Err(e) => error!("Tower latitude was not updated {:?}", e),
        };
    }

    let tower_longitude: f64 = -96.835597;
    if switchboard.effects.persist_nvs {
        match nvs.set_str("tower_longitude", &tower_longitude.to_string()){
            Ok(_) => info!("Tower longitude has been updated"),
            Err(e) => error!("Tower longitude was not updated {:?}", e),
        };
    }

    // Load tower configuration values
    let tower_id: u32 = 1;
    let latitude = nvs
        .get_str("tower_latitude", &mut buffer)?
        .unwrap_or("0")
        .parse() 
        .unwrap_or(0.0);
    //let latitude: f64 = 32.797868;
    let longitude = nvs
        .get_str("tower_longitude", &mut buffer)?
        .unwrap_or("0")
        .parse()
        .unwrap_or(0.0);
    //let longitude: f64 = -96.835597;
    let altitude: f64 = 0.0; 

    info!("Retrieved latitude: {}, and longitude: {}", latitude, longitude);

    
    info!("Tower id: {}, Lat: {}, Lon: {}, Alt: {}", tower_id, latitude, longitude, altitude);
    
    // Set new instance of clock crate 
    let mut calculation = Clock::new(bus.acquire_i2c(), latitude, longitude, altitude); // Create a new clock object 
    calculation.set_date_time(&local_time.naive_local()); // Set the current date and time 
    
    //let mut relay = PinDriver::output(peripherals.pins.gpio15).unwrap();
    //let mut lmsw = PinDriver::input(peripherals.pins.gpio6).unwrap();
    //lmsw.set_pull(esp_idf_hal::gpio::Pull::Down);
    let mut led = Led::new(peripherals.pins.gpio7, peripherals.rmt.channel0).unwrap();

    // Set motion pins
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

    // Apply runtime guardrails (compile-time configured).
    motion.set_stall_detection_enabled(switchboard.runtime.guardrails.stall_detection_enabled);
    motion.set_soft_limits(
        switchboard.runtime.guardrails.soft_limits_enabled,
        switchboard.runtime.guardrails.soft_limit_min_deg,
        switchboard.runtime.guardrails.soft_limit_max_deg,
    );

    // For later: motor power sense input. For now this is just a test knob.
    const POWER_ON: bool = true;

    // ======== Select motion mode (StepperOnly vs EncoderGuarded) ========
    let motion_mode = match switchboard.runtime.motion_mode {
        MotionModePolicy::FromNvsDefault(default) => {
            infra::SnapshotStore::new(&mut nvs, switchboard.effects.persist_nvs)
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

    // Restore heading (do NOT overwrite on every boot).
    // If the previous run was Admin, treat NVS state as untrusted and force homing instead.
    let mut actual_heading: f32 = if trust_nvs_state {
        let h = infra::SnapshotStore::new(&mut nvs, switchboard.effects.persist_nvs)
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
            infra::SnapshotStore::new(&mut nvs, switchboard.effects.persist_nvs)
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

    // Keep Motion's internal position consistent for logs/logic.
    motion.update_position(actual_heading);

    // ======== Admin mode (bench / diagnostics) ========
    // Centralized runner so we don't scatter test toggles throughout `main.rs`.
    if active_mode == RunMode::Admin {
        admin_mode::run(
            &switchboard.admin,
            &mut motion,
            &switchboard.boot.recovery,
            &switchboard.boot.homing,
            &mut nvs,
            &mut mqtt,
            &mut wifi,
            &current_version,
            switchboard.effects.publish_mqtt,
            switchboard.effects.persist_nvs,
        )?;
        // Admin runner may return if configured; if so, we still don't want to fall into normal mode by accident.
        return Ok(());
    }

    // ======== Normal mode boot actions ========
    // One-shot recovery move (bench / unstuck).
    // If we're near a hard stop, we can safely back off before attempting any homing search.
    if switchboard.boot.recovery.enabled {
        diagnostics::boot_recovery::run(&mut motion, &switchboard.boot.recovery);
    }

    let _mb = PinDriver::input(peripherals.pins.gpio5).unwrap(); // Maintenance
    let _eb = PinDriver::input(peripherals.pins.gpio4).unwrap(); // East Button
    let _wb = PinDriver::input(peripherals.pins.gpio6).unwrap(); // West Button

    // --- Homing ---
    // StepperOnly always homes. EncoderGuarded can skip homing if snapshot restored.
    // If snapshot wasn't available/valid, fall back to the existing homing behavior.
    let should_home_by_mode =
        motion_mode == MotionMode::StepperOnly || !restored_from_snapshot || !trust_nvs_state;
    if should_home_by_mode && switchboard.boot.homing.enabled {
        let limit_sw_status = match switchboard.boot.homing.dir {
            Direction::Cw => motion.find_limit_switch_cw(),
            Direction::Ccw => motion.find_limit_switch_ccw(),
        };
        match limit_sw_status {
            true => log::info!(
                "Homing OK (dir={}): limit switch found",
                switchboard.boot.homing.dir.as_str()
            ),
            false => {
                log::error!(
                    "Homing FAILED (dir={}): limit switch could not be found",
                    switchboard.boot.homing.dir.as_str()
                );
                infra::Telemetry::critical_failure_loop(
                    &mut mqtt,
                    b"Critical failure: Limit switch failure!",
                    switchboard.effects.publish_mqtt,
                );
            }
        }
        // After a successful homing run, re-seed NVS with a clean baseline.
        if switchboard.effects.persist_nvs {
            infra::SnapshotStore::new(&mut nvs, true).save_heading(90.0);
            if motion_mode == MotionMode::EncoderGuarded {
                infra::SnapshotStore::new(&mut nvs, true)
                    .save_encoder_snapshot(motion.encoder_ticks_adjusted());
            }
        }
        thread::sleep(Duration::from_secs(5));
    } else if should_home_by_mode {
        log::warn!("Homing skipped: switchboard.boot.homing.enabled=false");
        if !trust_nvs_state {
            infra::Telemetry::critical_failure_loop(
                &mut mqtt,
                b"Critical failure: NVS state untrusted (last run was Admin) but homing disabled!",
                switchboard.effects.publish_mqtt,
            );
        }
    } else {
        log::info!("Skipping homing: restored heading+encoder snapshot from NVS");
    }

    // Mark this boot as Normal so next boot can trust NVS again (if persistence enabled).
    infra::SnapshotStore::new(&mut nvs, true).save_last_run_normal(true);

    // --- Encoder fault recovery ---
    // If we stall (encoder unplugged), pause normal tracking and periodically probe for recovery.
    let mut encoder_fault = EncoderFaultRecovery::new();

    loop {
        let st_now = SystemTime::now();
        let dt_now_utc: DateTime<Utc> = st_now.into();
        let local_time: DateTime<FixedOffset> = DateTime::from_naive_utc_and_offset(
        dt_now_utc.naive_utc(),
        FixedOffset::east_opt(timezone_offset_hours * 3600).unwrap());
        let current_datetime = format!("{}", local_time.format("%d/%m/%Y %H:%M:%S"));

        info!("Actual Heading: {}", motion.location());
        info!("Current datetime: {}", current_datetime.clone());
        //std::thread::sleep(Duration::from_secs(10)); // 5-minute cycle */

        let now = std::time::Instant::now();  // Timer to measure how long this tracking loop iteration takes

        if encoder_fault.tick(
            &switchboard.runtime.encoder_recovery,
            &mut motion,
            motion_mode,
            &mut actual_heading,
            &mut nvs,
            &mut mqtt,
            &mut wifi,
            &current_version,
            switchboard.effects.publish_mqtt,
            switchboard.effects.persist_nvs,
        )? {
            continue;
        }

        // Perform solar tracking (normal path) if enabled.
        if switchboard.runtime.tracking.enabled {
            let outcome = app::tracking_loop::tick(
                &mut motion,
                &mut calculation,
                &mut actual_heading,
                &mut mqtt,
                &current_version,
                &mut nvs,
                &mut wifi,
                current_datetime.clone(),
                switchboard.effects.publish_mqtt,
                switchboard.effects.persist_nvs,
                switchboard.effects.allow_ota,
            );

            if outcome != MoveOutcome::Completed {
                warn!("Last move aborted: {:?}", outcome);
            }
            encoder_fault.on_move_outcome(outcome, &switchboard.runtime.encoder_recovery);
        } else {
            info!("Tracking disabled by switchboard.runtime.tracking.enabled=false");
        }

        info!("Tracking loop duration (v1.0.4): {:?}", now.elapsed());
        if wifi.state() == WifiState::Disconnected{
            warn!("Wifi disconnected, attempting to reconnect...");
            wifi.reconnect_if_disconnected()?;
        }
        infra::Telemetry::publish_firmware_version_if(
            &mut mqtt,
            &current_version,
            switchboard.effects.publish_mqtt,
        );

        // Process incoming MQTT commands (non-blocking, returns immediately if no command)
        if let Err(e) = cmd_handler::CommandHandler::process_one(
            &mut mqtt,
            &mut motion,
            &mut nvs,
            &mut wifi,
            &current_version,
            &switchboard.admin,
            &switchboard.boot.recovery,
            &switchboard.boot.homing,
            switchboard.effects.publish_mqtt,
            switchboard.effects.persist_nvs,
        ) {
            warn!("Command processing error: {:?}", e);
        }
        
        std::thread::sleep(Duration::from_secs(
            switchboard.runtime.tracking.loop_sleep_secs,
        ));

    }
}

fn boot_diagnostic(wifi: &mut Wifi, mqtt: &mut Mqtt, publish_mqtt: bool) -> bool {
    // Let system settle
    info!("Starting boot validation in 5 seconds...");
    thread::sleep(Duration::from_secs(5));

    // Wifi check
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