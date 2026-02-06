use std::time::{Duration, SystemTime};
use chrono::{DateTime, FixedOffset, Utc};
use clock::Clock;
use log::*;
use std::thread;

mod config;
mod switchboard;
mod state;
mod infra;
mod app;

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

// ================= Compile-time switchboard =================
// Choose ONE profile here; everything else is defined in `src/switchboard/`.
const ACTIVE_PROFILE: Profile = Profile::Normal;
const SWITCHBOARD: Switchboard = switchboard::active(ACTIVE_PROFILE);

fn main() -> anyhow::Result<()> {
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
    let mqtt_user = "device1A";
    match nvs.set_str("mqtt_user", mqtt_user){
        Ok(_) => info!("Mqtt username updated"),
        Err(e) => error!("Mqtt username not updated {:?}", e),
    }; 
    let mqtt_pass= "device1A";
    match nvs.set_str("mqtt_pass", mqtt_pass){
        Ok(_) => info!("Mqtt password updated"),
        Err(e) => error!("Mqtt password not updated {:?}", e),
    }; 

    let wifi_ssid = "Power2"; //"jp5k";
    match nvs.set_str("wifi_ssid", wifi_ssid){
        Ok(_) => info!("Wifi ssid updated"),
        Err(e) => error!("Wifi ssid not updated {:?}", e),
    }; 
    let wifi_pass= "@Powerfuture22";
    match nvs.set_str("wifi_pass", wifi_pass){
        Ok(_) => info!("Wifi password updated"),
        Err(e) => error!("Wifi password not updated {:?}", e),
    }; 

    let offset_hours = -5;
    match nvs.set_i32("offset_hours", offset_hours){
        Ok(_) => info!("Timezone offset has been updated"),
        Err(e) => error!("Timezone offset was not updated {:?}", e),
    };

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

    // ======== Boot Validation ========    
    let first_boot = nvs.get_u8("first_boot")?.unwrap_or(1);

    // Run boot_diagnostic check
    let boot_diagnostic_result = boot_diagnostic(&mut wifi, &mut mqtt);

    if first_boot == 1 {
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
    nvs.set_str("version", "1.0.4")?; // RESETTING THE FIRMWARE VERSION

    // Default version if nothing is stored
    const DEFAULT_VERSION: &str = "1.0.4";
    // Read the current version from NVS
    let current_version: Version = nvs
        .get_str("version", &mut version_buf)?
        .map(|s| s.trim().parse::<Version>())
        .transpose()? // converts Option<Result<..>> into Result<Option<..>>
        .unwrap_or_else(|| Version::parse(DEFAULT_VERSION).unwrap());

    // Publishing a message about the current running version
    info!("The current firmware version is: {}", current_version.to_string());
    let payload = format!(
        "The current firmware version is: {}",
        current_version.to_string()
    );
    mqtt.publish("device1A/firmware/version", payload.as_bytes())?;

    // Creates an instance of OTA crate
    let mut updater = OtaUpdater::new_ota(current_version.clone(), &mut mqtt, Some("device1A"), Some("device1A")).expect("Failed to create OTA updater instance");

    // Run version compare
    info!("Checking for new OTA update in 3 seconds...");
    thread::sleep(Duration::from_secs(3));
    updater.run_version_compare(&mut nvs)?;
    
    // Set tower configuration values
    let tower_latitude: f64 = 32.797868;
    match nvs.set_str("tower_latitude", &tower_latitude.to_string()){
        Ok(_) => info!("Tower latitude has been updated"),
        Err(e) => error!("Tower latitude was not updated {:?}", e),
    };

    let tower_longitude: f64 = -96.835597;
    match nvs.set_str("tower_longitude", &tower_longitude.to_string()){
        Ok(_) => info!("Tower longitude has been updated"),
        Err(e) => error!("Tower longitude was not updated {:?}", e),
    };

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

    // For later: motor power sense input. For now this is just a test knob.
    const POWER_ON: bool = true;

    // ======== Select motion mode (StepperOnly vs EncoderGuarded) ========
    let motion_mode = match SWITCHBOARD.runtime.motion_mode {
        MotionModePolicy::FromNvsDefault(default) => {
            state::SnapshotStore::new(&mut nvs).load_tracking_mode_or_init(default)
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
    let mut actual_heading: f32 =
        state::SnapshotStore::new(&mut nvs).load_heading_or_init(90.0);
    info!("Restored heading from NVS(90 Degrees + Distance by encoders): {}", actual_heading);

    // Restore encoder adjusted ticks snapshot (version-gated) in L2 only.
    let mut restored_from_snapshot = false;
    if motion_mode == MotionMode::EncoderGuarded {
        if let Some(enc_ticks_adj) =
            state::SnapshotStore::new(&mut nvs).load_encoder_snapshot()
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
        info!("Motion mode is StepperOnly: skipping encoder snapshot restore");
    }

    // Keep Motion's internal position consistent for logs/logic.
    motion.update_position(actual_heading);

    // ======== One-shot recovery move (bench / unstuck) ========
    // If we're near a hard stop, we can safely back off before attempting any homing search.
    if SWITCHBOARD.boot.recovery.enabled {
        app::boot_recovery::run(&mut motion, &SWITCHBOARD.boot.recovery);
    }

    let _mb = PinDriver::input(peripherals.pins.gpio5).unwrap(); // Maintenance
    let _eb = PinDriver::input(peripherals.pins.gpio4).unwrap(); // East Button
    let _wb = PinDriver::input(peripherals.pins.gpio6).unwrap(); // West Button

    // --- Homing ---
    // StepperOnly always homes. EncoderGuarded can skip homing if snapshot restored.
    // If snapshot wasn't available/valid, fall back to the existing homing behavior.
    let should_home_by_mode = motion_mode == MotionMode::StepperOnly || !restored_from_snapshot;
    if should_home_by_mode && SWITCHBOARD.boot.homing.enabled {
        let limit_sw_status = match SWITCHBOARD.boot.homing.dir {
            Direction::Cw => motion.find_limit_switch_cw(),
            Direction::Ccw => motion.find_limit_switch_ccw(),
        };
        match limit_sw_status {
            true => log::info!(
                "Homing OK (dir={}): limit switch found",
                SWITCHBOARD.boot.homing.dir.as_str()
            ),
            false => {
                log::error!(
                    "Homing FAILED (dir={}): limit switch could not be found",
                    SWITCHBOARD.boot.homing.dir.as_str()
                );
                infra::Telemetry::publish_critical_failure_loop(
                    &mut mqtt,
                    b"Critical failure: Limit switch failure!",
                );
            }
        }
        thread::sleep(Duration::from_secs(5));
    } else if should_home_by_mode {
        log::warn!("Homing skipped: SWITCHBOARD.boot.homing.enabled=false");
    } else {
        log::info!("Skipping homing: restored heading+encoder snapshot from NVS");
    }

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
            &SWITCHBOARD.runtime.encoder_recovery,
            &mut motion,
            motion_mode,
            &mut actual_heading,
            &mut nvs,
            &mut mqtt,
            &mut wifi,
            &current_version,
        )? {
            continue;
        }

        // Perform solar tracking (normal path) if enabled.
        if SWITCHBOARD.runtime.tracking.enabled {
            let outcome = app::tracking_loop::tick(
                &mut motion,
                &mut calculation,
                &mut actual_heading,
                &mut mqtt,
                &current_version,
                &mut nvs,
                &mut wifi,
                current_datetime.clone(),
            );

            if outcome != MoveOutcome::Completed {
                warn!("Last move aborted: {:?}", outcome);
            }
            encoder_fault.on_move_outcome(outcome, &SWITCHBOARD.runtime.encoder_recovery);
        } else {
            info!("Tracking disabled by SWITCHBOARD.runtime.tracking.enabled=false");
        }

        info!("Tracking loop duration (v1.0.4): {:?}", now.elapsed());
        if wifi.state() == WifiState::Disconnected{
            warn!("Wifi disconnected, attempting to reconnect...");
            wifi.reconnect_if_disconnected()?;
        }
        infra::Telemetry::publish_firmware_version(&mut mqtt, &current_version);
        
        std::thread::sleep(Duration::from_secs(
            SWITCHBOARD.runtime.tracking.loop_sleep_secs,
        ));

    }
}

fn boot_diagnostic(wifi: &mut Wifi, mqtt: &mut Mqtt) -> bool {
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
        if infra::Telemetry::publish_boot_check(mqtt) {
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