// IMPORTS
use std::time::{Duration, SystemTime};
use chrono::{DateTime, FixedOffset, Utc};
use clock::Clock;
use log::*;
use std::thread;
use esp_idf_hal::peripherals::Peripherals;
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
use motion::Motion;
use rgb_led::Led;
use network::mqtt::Mqtt;
use ota::OtaUpdater;
use semver::Version;
use wifi::wifi::{Wifi, WifiState};

mod switchboard;

// This function must be provided when using embassy-sync/embassy-time-driver
//Fixes the ldproxy linker error in 'tower'
#[no_mangle]
pub extern "C" fn __pender() {
    // Keep it empty
}

// MAIN FUNCTION
fn main() -> anyhow::Result<()> {
    let sw = switchboard::normal();
    
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
    
    let mqtt_user = sw.default_mqtt_user;
    match nvs.set_str("mqtt_user", mqtt_user) {
        Ok(_) => info!("Mqtt username updated"),
        Err(e) => error!("Mqtt username not updated {:?}", e),
    };
    
    let mqtt_pass = sw.default_mqtt_pass;
    match nvs.set_str("mqtt_pass", mqtt_pass) {
        Ok(_) => info!("Mqtt password updated"),
        Err(e) => error!("Mqtt password not updated {:?}", e),
    };
    
    let wifi_ssid = sw.default_wifi_ssid;
    match nvs.set_str("wifi_ssid", wifi_ssid) {
        Ok(_) => info!("Wifi ssid updated"),
        Err(e) => error!("Wifi ssid not updated {:?}", e),
    };
    
    let wifi_pass = sw.default_wifi_pass;
    match nvs.set_str("wifi_pass", wifi_pass) {
        Ok(_) => info!("Wifi password updated"),
        Err(e) => error!("Wifi password not updated {:?}", e),
    };
    
    let offset_hours = sw.default_tz_offset_hours;
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
    thread::sleep(Duration::from_secs(sw.wifi_connect_delay_secs));
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
    let timezone_offset_hours: i32 = nvs
        .get_i32("offset_hours")?
        .unwrap_or(sw.default_tz_offset_hours);
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
        sw.mqtt_broker_url,
        sw.mqtt_client_id,
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
    nvs.set_str("version", sw.default_version)?;
    let current_version: Version = nvs
        .get_str("version", &mut version_buf)?
        .map(|s| s.trim().parse::<Version>())
        .transpose()?
        .unwrap_or_else(|| Version::parse(sw.default_version).unwrap());

    info!("The current firmware version is: {}", current_version.to_string());
    let mut payload = format!("The current firmware version is: {}", current_version.to_string());
    mqtt.publish("device1A/firmware/version", payload.as_bytes())?;

    let mut updater = OtaUpdater::new_ota(
        current_version.clone(),
        &mut mqtt,
        Some(sw.default_ota_updater),
        Some(sw.default_ota_password),
    )
    .expect("Failed to create OTA updater instance");

    info!("Checking for new OTA update in 3 seconds...");
    thread::sleep(Duration::from_secs(sw.ota_check_delay_secs));
    updater.run_version_compare(&mut nvs)?;

     
    //TOWER CONFIGURATION

    let tower_latitude: f64 = sw.default_tower_latitude;
    match nvs.set_str("tower_latitude", &tower_latitude.to_string()) {
        Ok(_) => info!("Tower latitude has been updated"),
        Err(e) => error!("Tower latitude was not updated {:?}", e),
    };

    let tower_longitude: f64 = sw.default_tower_longitude;
    match nvs.set_str("tower_longitude", &tower_longitude.to_string()) {
        Ok(_) => info!("Tower longitude has been updated"),
        Err(e) => error!("Tower longitude was not updated {:?}", e),
    };

    let tower_id: u32 = sw.default_tower_id;
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
    
    motion.init();
    led.display_healthy();
    motion.run();

     
    // HEADING INITIALIZATION

    let heading_tag = sw.heading_tag;
    let mut actual_heading: f32 = 90.0;

    match nvs.set_u32(heading_tag, actual_heading.to_bits()) {
        Ok(_) => info!("heading updated"),
        Err(e) => error!("heading not updated {:?}", e),
    };

    match nvs.get_u32(heading_tag).unwrap() {
        Some(v) => {
            info!("{:?} = {:?}", heading_tag, v);
            actual_heading = f32::from_bits(v);
        }
        None => info!("{:?} not found", heading_tag),
    };

    let mut mb = PinDriver::input(peripherals.pins.gpio5).unwrap();  // Maintenance Button
    let mut eb = PinDriver::input(peripherals.pins.gpio4).unwrap();  // East Button
    let mut wb = PinDriver::input(peripherals.pins.gpio6).unwrap();  // West Button

     
    // HOMING SEQUENCE
    // todo!("Implement an encoder to re-position in case of power failure");

    let limit_sw_status = motion.find_limit_switch_cw();
    match limit_sw_status {
        true => log::info!("Limit switch has returned true"),
        false => {
            log::error!("Limit switch has returned false, limit switch could not be found");
            loop {
                if let Err(e) = mqtt.publish("device1A/tower/status", b"Critical failure: Limit switch failure!") {
                    log::error!("Failed to publish critical error message: {:?}", e);
                }
                thread::sleep(Duration::from_secs(900)); // Loop every 15 minutes
            }
        }
    }
    thread::sleep(Duration::from_secs(5));

     
    // MAIN TRACKING LOOP

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

        let now = std::time::Instant::now();

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

        if !tracking_done {
            actual_heading = motion.location();
            match nvs.set_u32(heading_tag, actual_heading.to_bits()) {
                Ok(_) => info!("Stored stable heading in NVS: {}", actual_heading),
                Err(e) => warn!("Failed to store heading in NVS: {:?}", e),
            }

            // ======== Stage 2: persist encoder snapshot (non-backdrivable tower) ========
            // Convention: adjusted ticks are 0 at the limit switch, CW positive.
            let enc_ticks_adj = motion.encoder_ticks_adjusted();
            if let Err(e) = nvs.set_u32(sw.nvs_key_enc_snapshot_version, sw.enc_snapshot_version) {
                warn!(
                    "Failed to store encoder snapshot version in NVS ({}): {:?}",
                    sw.nvs_key_enc_snapshot_version, e
                );
            }
            if let Err(e) = nvs.set_i32(sw.nvs_key_enc_ticks_adj, enc_ticks_adj) {
                warn!(
                    "Failed to store encoder ticks in NVS ({}): {:?}",
                    sw.nvs_key_enc_ticks_adj, e
                );
            } else {
                info!(
                    "Stored encoder snapshot in NVS: {}={} (v={})",
                    sw.nvs_key_enc_ticks_adj, enc_ticks_adj, sw.enc_snapshot_version
                );
            }
        } else {
            info!("True return from set tower position");
            info!("Angle offset is less then 5");
        }

        info!("Tracking loop duration (v1.0.4): {:?}", now.elapsed());
        
        if wifi.state() == WifiState::Disconnected {
            warn!("Wifi disconnected, attempting to reconnect...");
            wifi.reconnect_if_disconnected()?;
        }
        
        payload = format!("The current firmware version is: {}", current_version.to_string());
        mqtt.publish("device1A/firmware/version", payload.as_bytes())?;
        
        std::thread::sleep(Duration::from_secs(sw.tracking_loop_sleep_secs)); // 5-minute cycle
    }
}

 
// BOOT DIAGNOSTIC FUNCTION
 
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

        match mqtt.publish("device1A/boot", b"Boot check...") {
            Ok(_) => {
                info!("MQTT boot diagnostic publish succeeded...");
                return true;
            }
            Err(e) => {
                error!("MQTT publish failed immediately: {:?}", e);
                if attempt == MAX_RETRIES {
                    error!("All MQTT boot diagnostic attempts failed...");
                    return false;
                }
                thread::sleep(Duration::from_millis(1000));
                continue;
            }
        }
    }
    return false;
}
