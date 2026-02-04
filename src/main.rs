use std::{
    str::FromStr,
    time::{Duration, SystemTime, Instant},
};
use chrono::{DateTime, FixedOffset, Utc};
use clock::Clock;
use log::*;
use std::thread;
//use esp32_nimble::{enums::*, uuid128, BLEAdvertisedDevice, BLEDevice, BLEScan};
use esp_idf_hal::{sys::esp_app_desc, task::current};

mod snapshot_store;
mod telemetry;

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
use motion::{calculate_steps, MoveOutcome, Motion, MotionMode};
use rgb_led::Led;
use network::mqtt::Mqtt;
use ota::OtaUpdater;
use semver::Version;
use wifi::wifi::{Wifi, WifiState};

fn main() -> anyhow::Result<()> {
    
    // SYSTEM INITIALIZATION
    esp_idf_svc::sys::link_patches();
    EspLogger::initialize_default();
    let sysloop = EspSystemEventLoop::take()?;
    
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
    let encoderA = peripherals.pins.gpio47;
    let encoderB = peripherals.pins.gpio21;

    // Setting of sda and scl gpio pins as well as i2c
    let sda = peripherals.pins.gpio8;
    let scl = peripherals.pins.gpio9;
    let config = I2cConfig::new().baudrate(10_u32.kHz().into());
    let i2c = I2cDriver::new(peripherals.i2c0, sda, scl, &config).unwrap();
    let bus: &'static _ = shared_bus::new_std!(I2cDriver = i2c).unwrap();

     
    //CREDENTIALS CONFIGURATION 
    // todo!("Implement a .env");
    
    let mqtt_user = "device1A";
    match nvs.set_str("mqtt_user", mqtt_user) {
        Ok(_) => info!("Mqtt username updated"),
        Err(e) => error!("Mqtt username not updated {:?}", e),
    };
    
    let mqtt_pass = "device1A";
    match nvs.set_str("mqtt_pass", mqtt_pass) {
        Ok(_) => info!("Mqtt password updated"),
        Err(e) => error!("Mqtt password not updated {:?}", e),
    };
    
    let wifi_ssid = "Power2";
    match nvs.set_str("wifi_ssid", wifi_ssid) {
        Ok(_) => info!("Wifi ssid updated"),
        Err(e) => error!("Wifi ssid not updated {:?}", e),
    };
    
    let wifi_pass = "@Powerfuture22";
    match nvs.set_str("wifi_pass", wifi_pass) {
        Ok(_) => info!("Wifi password updated"),
        Err(e) => error!("Wifi password not updated {:?}", e),
    };
    
    let offset_hours = -5;
    match nvs.set_i32("offset_hours", offset_hours) {
        Ok(_) => info!("Timezone offset has been updated"),
        Err(e) => error!("Timezone offset was not updated {:?}", e),
    };

     
    // WIFI INITIALIZATION
    
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
    thread::sleep(Duration::from_secs(WIFI_CONNECT_DELAY_SECS));
	wifi.connect(&real_wifi_ssid, &real_wifi_pass).expect("Wi-Fi connection failed");
	info!("Current wifi state: {:?}", wifi.state());
    if wifi.state() == WifiState::Disconnected{
        wifi.reconnect_if_disconnected()?;
    }

     
    //TIME SYNCHRONIZATION

    let ntp = EspSntp::new_default().unwrap();
    info!("Synchronizing with NTP Server");
    while ntp.get_sync_status() != SyncStatus::Completed {}
    info!("Time Sync Completed");

    let st_now = SystemTime::now();
    let dt_now_utc: DateTime<Utc> = st_now.clone().into();
    let timezone_offset_hours: i32 = nvs.get_i32("offset_hours")?.unwrap_or(DEFAULT_TZ_OFFSET_HOURS);
    let local_time: DateTime<FixedOffset> = DateTime::from_naive_utc_and_offset(
        dt_now_utc.naive_utc(),
        FixedOffset::east_opt(timezone_offset_hours * 3600).unwrap(),
    );

    let formatted_time = format!("{}", local_time.format("%d/%m/%Y %H:%M:%S"));
    info!("{}", formatted_time);

     
    //MQTT INITIALIZATION
    
    let real_mqtt_user = nvs
        .get_str("mqtt_user", &mut buffer)?
        .expect("Mqtt username not found")
        .to_string();
    let real_mqtt_pass = nvs
        .get_str("mqtt_pass", &mut buffer)?
        .expect("Mqtt password not found")
        .to_string();

    let mut mqtt = Box::new(Mqtt::new_mqtt(
        MQTT_BROKER_URL,
        MQTT_CLIENT_ID,
        &real_mqtt_user,
        &real_mqtt_pass,
    )?);

     
    //BOOT VALIDATION
    
    let first_boot = nvs.get_u8("first_boot")?.unwrap_or(1);
    let boot_diagnostic_result = boot_diagnostic(&mut wifi, &mut mqtt);

    if first_boot == 1 {
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

     
    //OTA UPDATE SYSTEM

    let mut version_buf = [0u8; 32];
    info!("Setting the firmware version...");
    nvs.set_str("version", DEFAULT_VERSION)?;
    let current_version: Version = nvs
        .get_str("version", &mut version_buf)?
        .map(|s| s.trim().parse::<Version>())
        .transpose()?
        .unwrap_or_else(|| Version::parse(DEFAULT_VERSION).unwrap());

    info!("The current firmware version is: {}", current_version.to_string());
    let mut payload = format!("The current firmware version is: {}", current_version.to_string());
    mqtt.publish("device1A/firmware/version", payload.as_bytes())?;

    let mut updater = OtaUpdater::new_ota(
        current_version.clone(),
        &mut mqtt,
        Some("device1A"),
        Some("device1A"),
    )
    .expect("Failed to create OTA updater instance");

    info!("Checking for new OTA update in 3 seconds...");
    thread::sleep(Duration::from_secs(OTA_CHECK_DELAY_SECS));
    updater.run_version_compare(&mut nvs)?;

     
    //TOWER CONFIGURATION

    let tower_latitude: f64 = DEFAULT_TOWER_LATITUDE;
    match nvs.set_str("tower_latitude", &tower_latitude.to_string()) {
        Ok(_) => info!("Tower latitude has been updated"),
        Err(e) => error!("Tower latitude was not updated {:?}", e),
    };

    let tower_longitude: f64 = DEFAULT_TOWER_LONGITUDE;
    match nvs.set_str("tower_longitude", &tower_longitude.to_string()) {
        Ok(_) => info!("Tower longitude has been updated"),
        Err(e) => error!("Tower longitude was not updated {:?}", e),
    };

    let tower_id: u32 = DEFAULT_TOWER_ID;
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
    
    let mut calculation = Clock::new(bus.acquire_i2c(), latitude, longitude, altitude);
    calculation.set_date_time(&local_time.naive_local());
    
    let mut led = Led::new(peripherals.pins.gpio7, peripherals.rmt.channel0).unwrap();
    
    let mut motion = Motion::new(
        peripherals.pins.gpio15,   // CCW Motor
        peripherals.pins.gpio16,   // CW Motor
        peripherals.pins.gpio17,   // Relay 
        peripherals.pins.gpio14,   // Limit Switch 
        encoderA,                  // Encoder A
        encoderB,                  // Encoder B
    );
   
    motion.init();          // Initialize motor driver parameters
    led.display_healthy();  // Show healthy LED status
    let _ = motion.run();   // Ensure motor driver is in a ready state

    // For later: motor power sense input. For now this is just a test knob.
    const POWER_ON: bool = true;

    // ======== Select tracking mode (L1 legacy vs L2 current) ========
    // Default to L2 (current) if not set.
    let motion_mode = snapshot_store::SnapshotStore::new(&mut nvs)
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

    // Restore heading (do NOT overwrite on every boot).
    let mut actual_heading: f32 =
        snapshot_store::SnapshotStore::new(&mut nvs).load_heading_or_init(90.0);
    info!("Restored heading from NVS(90 Degrees + Distance by encoders): {}", actual_heading);

    // Restore encoder adjusted ticks snapshot (version-gated) in L2 only.
    let mut restored_from_snapshot = false;
    if motion_mode == MotionMode::EncoderGuarded {
        if let Some(enc_ticks_adj) =
            snapshot_store::SnapshotStore::new(&mut nvs).load_encoder_snapshot()
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
    // Convention for *current wiring*: positive step movement = physical CW.
    const RECOVERY_MOVE_CW_ON_BOOT: bool = true;
    const RECOVERY_MOVE_CW_DEG: f32 = 30.0;
    if RECOVERY_MOVE_CW_ON_BOOT {
        let steps = calculate_steps(RECOVERY_MOVE_CW_DEG);
        log::warn!(
            "RECOVERY_MOVE: moving {:.1}° CW (steps={}) before homing",
            RECOVERY_MOVE_CW_DEG,
            steps
        );
        let _ = motion.move_by(steps);
    }

    let mut mb = PinDriver::input(peripherals.pins.gpio5).unwrap();  // Maintenance 
    let mut eb = PinDriver::input(peripherals.pins.gpio4).unwrap();  // East Button
    let mut wb = PinDriver::input(peripherals.pins.gpio6).unwrap();  // West Button

    // --- Homing ---
    // StepperOnly always homes. EncoderGuarded can skip homing if snapshot restored.
    // If snapshot wasn't available/valid, fall back to the existing homing behavior.
    if motion_mode == MotionMode::StepperOnly || !restored_from_snapshot {
        // Convention for *current wiring*: positive step movement = physical CW.
        let limit_sw_status = motion.find_limit_switch_cw();
        match limit_sw_status{
            true => log::info!("Limit switch has returned true"),
            false => {
                log::error!("Limit switch has returned false, limit switch could not be found");
                telemetry::Telemetry::publish_critical_failure_loop(
                    &mut mqtt,
                    b"Critical failure: Limit switch failure!",
                );
            }
        }
        thread::sleep(Duration::from_secs(5)); // 
    } else {
        log::info!("Skipping homing: restored heading+encoder snapshot from NVS");
    }

    // ======== Stage 4: encoder-fault probe loop state ========
    // If we stall (encoder unplugged), we pause normal tracking, probe periodically,
    // and when the encoder is back we recompute heading from encoder ticks and resume.
    let mut encoder_fault = false;
    let mut next_probe_at: Option<Instant> = None;

    loop {
        let st_now = SystemTime::now();
        let dt_now_utc: DateTime<Utc> = st_now.into();
        let local_time: DateTime<FixedOffset> = DateTime::from_naive_utc_and_offset(
            dt_now_utc.naive_utc(),
            FixedOffset::east_opt(timezone_offset_hours * 3600).unwrap(),
        );
        let current_datetime = format!("{}", local_time.format("%d/%m/%Y %H:%M:%S"));

        info!("Actual Heading: {}", motion.location());
        info!("Current datetime: {}", current_datetime.clone());

        let now = std::time::Instant::now();  // Timer to measure how long this tracking loop iteration takes

        // ======== Stage 4: encoder-fault probe loop (3 min) ========
        // If we ever abort due to encoder stall, pause normal tracking and periodically probe
        // for encoder recovery by issuing a tiny movement and checking for tick change.
        const ENCODER_PROBE_INTERVAL: Duration = Duration::from_secs(180);
        // Keep probe small: ~30k steps. Tune if needed.
        const ENCODER_PROBE_STEPS: i64 = 30_000;
        const HOME_HEADING_DEG: f32 = 90.0;

        // If we're in encoder fault, skip tracking and only probe every few minutes.
        if encoder_fault {
            let now_i = Instant::now();
            let should_probe = next_probe_at.map(|t| now_i >= t).unwrap_or(true);
            if should_probe {
                info!("Encoder fault: probing for recovery...");
                let ok = motion.probe_encoder_motion(ENCODER_PROBE_STEPS);
                if ok {
                    info!("Encoder probe succeeded; recomputing heading from encoder and resuming tracking.");
                    encoder_fault = false;
                    next_probe_at = None;

                    // IMPORTANT:
                    // When encoder comes back, it may have missed ticks while unplugged.
                    // If encoder-derived heading is wildly different from our last stable heading,
                    // we should NOT persist a bogus value — instead, re-home to re-establish truth.
                    fn angle_diff_deg(a: f32, b: f32) -> f32 {
                        let mut d = (a - b).rem_euclid(360.0);
                        if d > 180.0 {
                            d = 360.0 - d;
                        }
                        d.abs()
                    }
                    const ENCODER_RECOVERY_MAX_DRIFT_DEG: f32 = 15.0;

                    let candidate_heading = motion.heading_from_encoder_ticks(HOME_HEADING_DEG);
                    let drift = angle_diff_deg(candidate_heading, actual_heading);
                    info!(
                        "Encoder recovered: candidate_heading={} prev_heading={} drift_deg={}",
                        candidate_heading, actual_heading, drift
                    );

                    if drift <= ENCODER_RECOVERY_MAX_DRIFT_DEG {
                        actual_heading = candidate_heading;
                        motion.update_position(actual_heading);
                        snapshot_store::SnapshotStore::new(&mut nvs).save_heading(actual_heading);
                        if motion_mode == MotionMode::EncoderGuarded {
                            snapshot_store::SnapshotStore::new(&mut nvs)
                                .save_encoder_snapshot(motion.encoder_ticks_adjusted());
                        }
                    } else {
                        warn!(
                            "Encoder recovered but heading drift ({:.2}°) exceeds {:.2}°; re-homing to re-establish truth.",
                            drift, ENCODER_RECOVERY_MAX_DRIFT_DEG
                        );
                        // Re-home (CW to limit switch). This is the only way to restore absolute truth
                        // after encoder disconnect/missed ticks.
                        let ok = motion.find_limit_switch_cw();
                        if !ok {
                            telemetry::Telemetry::publish_critical_failure_loop(
                                &mut mqtt,
                                b"Critical failure: re-home after encoder recovery failed!",
                            );
                        }
                        actual_heading = HOME_HEADING_DEG;
                        motion.update_position(actual_heading);
                        snapshot_store::SnapshotStore::new(&mut nvs).save_heading(actual_heading);
                        if motion_mode == MotionMode::EncoderGuarded {
                            snapshot_store::SnapshotStore::new(&mut nvs)
                                .save_encoder_snapshot(motion.encoder_ticks_adjusted());
                        }
                    }
                    // Fall through to normal tracking in this same loop iteration.
                } else {
                    warn!(
                        "Encoder probe failed; will retry in {:?}",
                        ENCODER_PROBE_INTERVAL
                    );
                    next_probe_at = Some(now_i + ENCODER_PROBE_INTERVAL);

                    // Still do housekeeping while we wait.
                    if wifi.state() == WifiState::Disconnected {
                        warn!("Wifi disconnected, attempting to reconnect...");
                        wifi.reconnect_if_disconnected()?;
                    }
                    telemetry::Telemetry::publish_firmware_version(&mut mqtt, &current_version);
                    std::thread::sleep(Duration::from_secs(30));
                    continue;
                }
            } else {
                let t = next_probe_at.unwrap();
                let remaining = t.saturating_duration_since(now_i);
                info!("Encoder fault: waiting {:?} until next probe...", remaining);

                // Housekeeping
                if wifi.state() == WifiState::Disconnected {
                    warn!("Wifi disconnected, attempting to reconnect...");
                    wifi.reconnect_if_disconnected()?;
                }
                telemetry::Telemetry::publish_firmware_version(&mut mqtt, &current_version);
                std::thread::sleep(Duration::from_secs(30));
                continue;
            }
        }

        // Perform solar tracking (normal path)
        let tracking_done = motion.set_tower_position(
            &mut calculation,
            actual_heading,
            0,
            &mut mqtt,
            current_version.clone(),
            &mut nvs,
            &mut wifi,
            current_datetime.clone(),
        );

        // Persist only when a move actually completed successfully.
        // This prevents “phantom saves” and prevents old headings from being overwritten on abort paths.
        match motion.take_last_move_outcome() {
            Some(MoveOutcome::Completed) => {
                actual_heading = motion.location();
                snapshot_store::SnapshotStore::new(&mut nvs).save_heading(actual_heading);
                if motion_mode == MotionMode::EncoderGuarded {
                    snapshot_store::SnapshotStore::new(&mut nvs)
                        .save_encoder_snapshot(motion.encoder_ticks_adjusted());
                }
            }
            Some(outcome @ (MoveOutcome::AbortedPowerMissing | MoveOutcome::AbortedStall)) => {
                warn!("Last move aborted: {:?}", outcome);
                if outcome == MoveOutcome::AbortedStall {
                    encoder_fault = true;
                    next_probe_at = Some(Instant::now() + ENCODER_PROBE_INTERVAL);
                }
            }
            None => {
                if tracking_done {
                    info!("No movement required (tracking_done=true)");
                } else {
                    // Defensive: if we ever return tracking_done=false but did not set a MoveOutcome,
                    // we do NOT persist anything to avoid corrupting the snapshot.
                    warn!("tracking_done=false but no MoveOutcome recorded; skipping NVS persist");
                }
            }
        }

        info!("Tracking loop duration (v1.0.4): {:?}", now.elapsed());
        
        if wifi.state() == WifiState::Disconnected {
            warn!("Wifi disconnected, attempting to reconnect...");
            wifi.reconnect_if_disconnected()?;
        }
        telemetry::Telemetry::publish_firmware_version(&mut mqtt, &current_version);
        
        std::thread::sleep(Duration::from_secs(300)); // 5-minute cycle  

    }
}

fn boot_diagnostic(wifi: &mut Wifi, mqtt: &mut Mqtt) -> bool {
    info!("Starting boot validation in 5 seconds...");
    thread::sleep(Duration::from_secs(5));

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
        if telemetry::Telemetry::publish_boot_check(mqtt) {
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
