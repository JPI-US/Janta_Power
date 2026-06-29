use core::option::Option::None;
use std::{thread, time::{Duration, Instant}};

use chrono::{DateTime, Local};
use clock::Clock;
use esp_idf_svc::{
    eventloop::EspSystemEventLoop,
    hal::{
        delay::Ets,
        gpio::PinDriver,
        i2c::{I2cConfig, I2cDriver},
        prelude::*,
    },
    log::EspLogger,
    nvs::{EspDefaultNvsPartition, EspNvs},
    ota::EspOta,
};
use hdc1080::Hdc1080;
use rtc::Rtc;
use motion::{MoveOutcome, Motion, MotionMode};
use rgb_led::Led;
use network::mqtt::Mqtt;
use ota::OtaUpdater;
use rgb_led::Led;
use rtc::Rtc;
use semver::Version;
use wifi::wifi::{Wifi, WifiState};
use esp_idf_svc::hal::gpio::{Input, Gpio4, Gpio5, Gpio6};

use crate::app::encoder_fault::{Direction, EncoderRecoverySwitches};

/// Long-lived runtime state for the tower, owned across the main tracking loop.
///
/// Built once at the end of boot (in `main`) by gathering the initialized
/// hardware/state locals, then driven by the loop. Generic over the shared-I2C
/// proxy type `I2C` so the bus-backed devices (`calculation`, `temp_sensor`)
/// can be stored without naming the verbose `shared_bus` proxy type.
struct Tower<I2C> {
    sw: switchboard::Switchboard,
    nvs: EspNvs<esp_idf_svc::nvs::NvsDefault>,
    motion: Motion<'static>,
    mqtt: Box<Mqtt>,
    wifi: Wifi<'static>,
    calculation: Clock<I2C>,
    temp_sensor: Option<Hdc1080<I2C, Ets>>,
    encoder_fault: app::encoder_fault::EncoderFaultRecovery,
    current_version: Version,
    motion_mode: MotionMode,
    actual_heading: f32,
    allow_ota: bool,
    /// Motion mode seen on the previous loop iteration; used to detect the
    /// transition *into* `StepperOnly`, which forces a re-home.
    previous_motion_mode: MotionMode,
    /// Set when a `StepperOnly` switch needs a re-home before tracking resumes.
    need_rehome: bool,
}

/// Main-loop steps, factored out of the loop body so the loop reads as a thin
/// sequence of named operations. Each method owns one concern and mutates the
/// shared state through `&mut self` (disjoint field borrows let one method touch
/// `motion`, `nvs`, `mqtt`, ... at once). Bound to `I2C: I2c` because the
/// tracking and temperature steps drive bus-backed devices.
impl<I2C: embedded_hal::i2c::I2c> Tower<I2C> {
    /// Every loop step persists to NVS (mirrors the boot-phase `PERSIST_NVS`).
    const PERSIST_NVS: bool = true;

    /// Day-rollover reset of the daily encoder mode (see [`check_daily_encoder_reset`]).
    fn daily_reset(&mut self, local_time: &DateTime<Local>) {
        let reset_occurred = check_daily_encoder_reset(&mut self.nvs, local_time, Self::PERSIST_NVS);
        if reset_occurred {
            self.motion_mode = infra::SnapshotStore::new(&mut self.nvs, Self::PERSIST_NVS)
                .load_tracking_mode_or_init(MotionMode::EncoderGuarded);
            self.motion.set_motion_mode(self.motion_mode);
            self.encoder_fault.set_mode_switched_daily(false);
            let mode_str = match self.motion_mode {
                MotionMode::StepperOnly => "StepperOnly",
                MotionMode::EncoderGuarded => "EncoderGuarded",
            };
            info!("Daily reset: Motion mode updated to {}", mode_str);
        }
    }

    /// Flag a re-home when the mode has just transitioned into `StepperOnly`.
    fn detect_stepper_transition(&mut self) {
        if self.motion_mode == MotionMode::StepperOnly
            && self.previous_motion_mode != MotionMode::StepperOnly
        {
            info!("Motion mode switched to StepperOnly - re-homing required");
            self.need_rehome = true;
        }
        self.previous_motion_mode = self.motion_mode;
    }

    /// Reload the motion mode from NVS in case another path switched it; if it
    /// changed to `StepperOnly`, schedule a re-home. `stepper_switch_log` is the
    /// message logged on that transition.
    fn sync_motion_mode_from_nvs(&mut self, stepper_switch_log: &str) {
        let mode = infra::SnapshotStore::new(&mut self.nvs, Self::PERSIST_NVS)
            .load_tracking_mode_or_init(MotionMode::EncoderGuarded);
        if mode != self.motion_mode {
            self.motion_mode = mode;
            self.motion.set_motion_mode(self.motion_mode);
            if self.motion_mode == MotionMode::StepperOnly {
                info!("{}", stepper_switch_log);
                self.need_rehome = true;
            }
        }
    }

    /// Re-home to the limit switch if a `StepperOnly` re-home is pending.
    /// Returns `Ok(true)` when a re-home ran (caller should `continue` the loop).
    /// `intro_log` is logged when the sweep starts.
    fn rehome_if_pending(&mut self, intro_log: &str) -> anyhow::Result<bool> {
        if !(self.need_rehome && self.motion_mode == MotionMode::StepperOnly) {
            return Ok(false);
        }
        info!("{}", intro_log);
        const HOMING_DIRECTION: Direction = Direction::Ccw;
        let limit_sw_status = match HOMING_DIRECTION {
            Direction::Cw => self.motion.find_limit_switch_cw(),
            Direction::Ccw => self.motion.find_limit_switch_ccw(),
        };
        match limit_sw_status {
            true => {
                info!("Re-homing OK (dir={}): limit switch found", HOMING_DIRECTION.as_str());
                self.actual_heading = self.sw.home_heading_deg;
                self.motion.update_position(self.actual_heading);
                if Self::PERSIST_NVS {
                    infra::SnapshotStore::new(&mut self.nvs, Self::PERSIST_NVS)
                        .save_heading(self.actual_heading);
                }
                self.need_rehome = false;
            }
            false => {
                error!("Re-homing FAILED (dir={}): limit switch could not be found", HOMING_DIRECTION.as_str());
                infra::error_loop(
                    self.sw.device_id,
                    &mut self.mqtt,
                    network::telemetry::Component::LimitSwitch,
                    "Re-home failed after encoder recovery",
                    "Encoder recovery completed but subsequent re-home could not locate the limit switch.",
                );
            }
        }
        thread::sleep(Duration::from_secs(2));
        Ok(true)
    }

    /// Build the encoder-recovery config from the switchboard.
    fn encoder_recovery_cfg(&self) -> EncoderRecoverySwitches {
        EncoderRecoverySwitches {
            enabled: self.sw.runtime.encoder_recovery.enabled,
            probe_interval_secs: self.sw.runtime.encoder_recovery.probe_interval_secs,
            probe_steps: self.sw.runtime.encoder_recovery.probe_steps,
            max_drift_deg: self.sw.runtime.encoder_recovery.max_drift_deg,
            rehome_dir: match self.sw.runtime.encoder_recovery.rehome_dir {
                switchboard::Direction::Cw => crate::app::encoder_fault::Direction::Cw,
                switchboard::Direction::Ccw => crate::app::encoder_fault::Direction::Ccw,
            },
        }
    }

    /// Run one encoder-fault probe/recovery tick. Returns `Ok(true)` when a fault
    /// is active (caller should `continue` and skip tracking this iteration).
    fn run_encoder_fault(&mut self) -> anyhow::Result<bool> {
        let cfg = self.encoder_recovery_cfg();
        let fault_active = self.encoder_fault.tick(
            &cfg,
            &mut self.motion,
            self.motion_mode,
            &mut self.actual_heading,
            &mut self.nvs,
            &mut self.mqtt,
            &mut self.wifi,
            &self.current_version,
            Self::PERSIST_NVS,
            self.sw.device_id,
            self.sw.home_heading_deg,
        )?;
        Ok(fault_active)
    }

    /// Run one tracking tick (gated by the switchboard) and feed the move
    /// outcome back into encoder-fault recovery.
    fn run_tracking(&mut self, current_datetime: String) {
        if !self.sw.runtime.tracking.enabled {
            info!("Tracking disabled");
            return;
        }
        let cfg = self.encoder_recovery_cfg();
        let outcome = app::tracking_loop::tick(
            &mut self.motion,
            &mut self.calculation,
            &mut self.actual_heading,
            &mut self.mqtt,
            &self.current_version,
            &mut self.nvs,
            &mut self.wifi,
            current_datetime,
            Self::PERSIST_NVS,
            self.allow_ota,
            self.sw.device_id,
        );

        if outcome != MoveOutcome::Completed {
            warn!("Last move aborted: {:?}", outcome);
        }
        if let Err(e) = self.encoder_fault.on_move_outcome(
            outcome,
            &cfg,
            &mut self.motion,
            &mut self.nvs,
            Self::PERSIST_NVS,
        ) {
            error!("Error in encoder fault recovery: {:?}", e);
        }
    }

    /// Reconnect Wi-Fi if it has dropped.
    fn maintain_wifi(&mut self) -> anyhow::Result<()> {
        if self.wifi.state() == WifiState::Disconnected {
            warn!("Wifi disconnected, attempting to reconnect...");
            self.wifi.reconnect_if_disconnected()?;
        }
        Ok(())
    }

    /// Publish the periodic heartbeat (`tower/{id}/status`).
    fn publish_heartbeat(&mut self, current_datetime: &str) {
        let payload = network::telemetry::Heartbeat {
            current_time: current_datetime,
            firmware_version: &self.current_version.to_string(),
        };
        let topic = network::telemetry::topic::status(self.sw.device_id);
        let _ = network::telemetry::publish_json(&mut self.mqtt, &topic, &payload);
    }

    /// Publish system temperature telemetry when the HDC1080 is present.
    fn report_temperature(&mut self, current_datetime: &str) {
        if let Some(ref mut sensor) = self.temp_sensor {
            infra::temperature::report_system_temperature(
                sensor,
                &mut self.mqtt,
                self.sw.device_id,
                current_datetime,
            );
        }
    }

    /// Answer at most one queued remote command (gated by the switchboard).
    fn process_commands(&mut self) {
        if !self.sw.runtime.commands_enabled {
            return;
        }
        let motion_mode_str = match self.motion_mode {
            MotionMode::StepperOnly => "stepper_only",
            MotionMode::EncoderGuarded => "encoder_guarded",
        };
        let firmware_version = self.current_version.to_string();
        let ctx = diagnostics::commands::CmdCtx {
            device_id: self.sw.device_id,
            firmware_version: &firmware_version,
            mqtt_connected: self.mqtt.is_connected(),
            wifi_connected: matches!(self.wifi.state(), WifiState::Connected(_)),
            motion_mode: motion_mode_str,
            current_heading: self.actual_heading,
        };
        if let Err(e) = diagnostics::transport::process_one(&mut self.mqtt, self.sw.device_id, &ctx) {
            warn!("Command processing failed: {:?}", e);
        }
    }
}

fn main() -> anyhow::Result<()> {
    let sw = switchboard::active(switchboard::Profile::from_env_str(
        crate::constants::ACTIVE_PROFILE_STR,
    ));

    // PHASE 1: INITIALIZATION --------------------------------------------------
    esp_idf_svc::sys::link_patches();

    // Logger and event loop
    EspLogger::initialize_default();
    let sysloop = EspSystemEventLoop::take()?;

    // Hardware and persistent storage
    let nvs_default = EspDefaultNvsPartition::take()?;

    let mut nvs = match EspNvs::new(nvs_default.clone(), "storage", true) {
        Ok(nvs) => {
            info!("Got namespace {:?} from default partition", "storage");
            nvs
        }
        Err(e) => panic!("Could't get namespace {:?}", e),
    };

    let last_run_normal =
        infra::SnapshotStore::new(&mut nvs, true).load_last_run_normal_or_init(true);
    let trust_nvs_state = last_run_normal;
    info!(
        "Last run normal={} -> trust_nvs_state={} (active_mode=Normal)",
        last_run_normal, trust_nvs_state
    );

    let peripherals = Peripherals::take().unwrap();

    // Encoder pins
    let encoder_a = peripherals.pins.gpio10;
    let encoder_b = peripherals.pins.gpio11;

    // I2C bus for Clock
    let sda = peripherals.pins.gpio8;
    let scl = peripherals.pins.gpio9;
    let config = I2cConfig::new().baudrate(10_u32.kHz().into());
    let i2c = I2cDriver::new(peripherals.i2c0, sda, scl, &config).unwrap();
    let bus: &'static _ = shared_bus::new_std!(I2cDriver = i2c).unwrap();

    // LED status
    let mut led = Led::new(peripherals.pins.gpio7, peripherals.rmt.channel0).unwrap();
    led.display_none()?;

    // Button inputs (reserved for manual control)
    let maintenance_button = PinDriver::input(peripherals.pins.gpio5).unwrap();
    let ccw_button = PinDriver::input(peripherals.pins.gpio4).unwrap();
    let cw_button = PinDriver::input(peripherals.pins.gpio6).unwrap();

    // PHASE 1.5: MAINTENANCE MODE ----------------------------------------------

    // Init motion early so that maintenance mode can use it
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
    let _ = motion.run();

    // Runtime guardrails from switchboard
    motion.set_stall_detection_enabled(sw.runtime.guardrails.stall_detection_enabled);
    motion.set_soft_limits(
        sw.runtime.guardrails.soft_limits_enabled,
        sw.runtime.guardrails.soft_limit_min_deg,
        sw.runtime.guardrails.soft_limit_max_deg,
    );

    // Capture maintenance mode
    capture_maintenance_mode(&mut motion, &mut led, maintenance_button, ccw_button, cw_button)?;

    // PHASE 2: NETWORK SETUP ---------------------------------------------------

    const PERSIST_NVS: bool = true;

    if PERSIST_NVS {
        match nvs.set_str("wifi_ssid", sw.default_wifi_ssid) {
            Ok(_) => info!("Wifi ssid updated"),
            Err(e) => error!("Wifi ssid not updated {:?}", e),
        };
    }
    if PERSIST_NVS {
        match nvs.set_str("wifi_pass", sw.default_wifi_pass) {
            Ok(_) => info!("Wifi password updated"),
            Err(e) => error!("Wifi password not updated {:?}", e),
        };
    }

    if PERSIST_NVS {
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
    log::info!("Waiting for 20 seconds before connecting to wifi");
    thread::sleep(Duration::from_secs(20));
    wifi.connect(&real_wifi_ssid, &real_wifi_pass)
        .expect("Wi-Fi connection failed");
    info!("Current wifi state: {:?}", wifi.state());
    if wifi.state() == WifiState::Disconnected {
        wifi.reconnect_if_disconnected()?;
    }

    // Time: RTC-first, SNTP fallback (see `rtc::Rtc::init`).
    // If true, never trust DS3231 on this boot — always SNTP then write RTC (debug / bad battery).
    const FORCE_NTP_SKIP_RTC: bool = false;
    let mut tz_buf = [0u8; 96];
    let tz_posix_str = nvs
        .get_str("tz_posix", &mut tz_buf)?
        .unwrap_or(sw.default_tz_posix);
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
    let mqtt_client_id = format!("tower_{}", sw.device_id);
    let mut mqtt = Box::new(Mqtt::new_mqtt(
        "mqttS://a2exykcl6t998u-ats.iot.us-east-1.amazonaws.com:8883",
        &mqtt_client_id,
    )?);

    // PHASE 3: BOOT VALIDATION -------------------------------------------------
    let first_boot = nvs.get_u8("first_boot")?.unwrap_or(1);

    const ALLOW_BOOT_VALIDATION: bool = true;

    // Load firmware version early so the boot-log publish can include it.
    let mut version_buf = [0u8; 32];
    const DEFAULT_VERSION: &str = "1.1.5";
    if PERSIST_NVS {
        nvs.set_str("version", "1.1.5")?;
    }
    let current_version: Version = nvs
        .get_str("version", &mut version_buf)?
        .map(|s| s.trim().parse::<Version>())
        .transpose()?
        .unwrap_or_else(|| Version::parse(DEFAULT_VERSION).unwrap());

    // Boot diagnostics: Wi-Fi + MQTT
    let boot_diagnostic_result = if ALLOW_BOOT_VALIDATION {
        boot_diagnostic(sw.device_id, &mut wifi, &mut mqtt, &current_version)
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
                            let topic =
                                network::telemetry::topic::logs_firmware_update(sw.device_id);
                            let published =
                                network::telemetry::publish_json(&mut mqtt, &topic, &payload)
                                    .is_ok();
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

    // Subscribe to the remote command channel (tower/{id}/cmd/diagnostics).
    // Non-fatal: a subscribe hiccup must not block boot. Gated by the profile.
    if sw.runtime.commands_enabled {
        if let Err(e) = diagnostics::transport::subscribe(&mut mqtt, sw.device_id) {
            warn!("Failed to subscribe to command channel: {:?}", e);
        }
    }

    // PHASE 4: FIRMWARE VERSION AND OTA ---------------------------------------
    // (current_version was loaded earlier, before boot_diagnostic, so the
    // boot-log publish could include `firmware_version`.)

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
    )
    .expect("Failed to create OTA updater instance");

    if allow_ota {
        info!("Checking for new OTA update in 3 seconds...");
        thread::sleep(Duration::from_secs(3));
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
    // Tower location — seeded from `TOWER_LATITUDE` / `TOWER_LONGITUDE` in
    // `.env` via `Switchboard`. When `PERSIST_NVS` is on, the switchboard
    // defaults are (re)written into NVS on every boot, so updating `.env` and
    // reflashing updates the tower coordinates on the next boot.
    let tower_latitude: f64 = sw.default_tower_latitude;
    if PERSIST_NVS {
        match nvs.set_str("tower_latitude", &tower_latitude.to_string()) {
            Ok(_) => info!("Tower latitude has been updated"),
            Err(e) => error!("Tower latitude was not updated {:?}", e),
        };
    }

    let tower_longitude: f64 = sw.default_tower_longitude;
    if PERSIST_NVS {
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

    info!(
        "Retrieved latitude: {}, and longitude: {}",
        latitude, longitude
    );
    info!(
        "Device: {}, Lat: {}, Lon: {}, Alt: {}",
        sw.device_id, latitude, longitude, altitude
    );

    // Hardware initialization
    let calculation = Clock::new(bus.acquire_i2c(), latitude, longitude, altitude);

    let temp_sensor = match Hdc1080::new(bus.acquire_i2c(), Ets) {
        Ok(mut sensor) => {
            let _ = sensor.init();
            if sensor.get_device_id().unwrap_or(0) == 0x1050 {
                info!("HDC1080 detected");
                Some(sensor)
            } else {
                warn!("HDC1080 not detected on I2C; temp telemetry disabled");
                None
            }
        }
        Err(e) => {
            warn!("HDC1080 init failed: {:?}; temp telemetry disabled", e);
            None
        }
    };

    led.display_healthy()?;

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
    let motion_mode = infra::SnapshotStore::new(&mut nvs, PERSIST_NVS)
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
            infra::SnapshotStore::new(&mut nvs, PERSIST_NVS).load_encoder_snapshot()
        {
            // Restore zero offset so adjusted ticks equal saved snapshot.
            let raw = motion.encoder_ticks_raw();
            motion.set_encoder_zero_offset(raw - enc_ticks_adj);
            info!(
                "Restored encoder snapshot ticks from NVS: {}",
                enc_ticks_adj
            );
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
    let encoder_daily_mode =
        infra::SnapshotStore::new(&mut nvs, PERSIST_NVS).load_encoder_daily_mode();
    encoder_fault.set_mode_switched_daily(encoder_daily_mode);

    // Homing policy:
    // - StepperOnly: always home
    // - EncoderGuarded: home when snapshot restore is unavailable/untrusted
    // - EncoderGuarded: if NVS claims mechanical home (≈ home_heading_deg), require limit
    //   switch active before skipping homing; otherwise re-home like snapshot miss
    const HOMING_DIRECTION: Direction = Direction::Ccw;
    /// Same tolerance as motion sunset home check (`location` vs `HOME_HEADING_DEG`).
    const HOME_HEADING_VERIFY_EPS_DEG: f32 = 0.01;

    let would_skip_homing_on_snapshot =
        motion_mode == MotionMode::EncoderGuarded && restored_from_snapshot && trust_nvs_state;
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
    if should_home_by_mode && sw.boot.homing.enabled {
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
        thread::sleep(Duration::from_secs(5));
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

    // PHASE 8: MAIN TRACKING LOOP ---------------------------------------------
    // Gather the long-lived state into the Tower context; the loop drives it.
    let mut tower = Tower {
        sw,
        nvs,
        motion,
        mqtt,
        wifi,
        calculation,
        temp_sensor,
        encoder_fault,
        current_version,
        motion_mode,
        actual_heading,
        allow_ota,
        previous_motion_mode: motion_mode,
        need_rehome: false,
    };

    loop {
        let local_time = rtc::timezone::local_time();
        let current_datetime = format!("{}", local_time.format("%d/%m/%Y %H:%M:%S"));

        // Daily encoder-mode reset at day rollover.
        tower.daily_reset(&local_time);

        // Re-home once when transitioning into StepperOnly.
        tower.detect_stepper_transition();
        if tower.rehome_if_pending(
            "StepperOnly mode detected - re-homing to establish known position",
        )? {
            continue;
        }

        info!("Actual Heading: {}", tower.motion.location());
        info!("Current datetime: {}", current_datetime.clone());

        let now = std::time::Instant::now();

        // Reload mode in case another path switched to StepperOnly.
        tower.sync_motion_mode_from_nvs(
            "Motion mode changed to StepperOnly - will re-home on next iteration",
        );

        if tower.run_encoder_fault()? {
            continue; // Fault active, skip tracking this iteration
        }

        // Re-check mode after recovery in case it changed during the probe.
        tower.sync_motion_mode_from_nvs(
            "Motion mode switched to StepperOnly during encoder fault recovery - will re-home",
        );

        // Re-home before tracking if StepperOnly was activated during recovery.
        if tower.rehome_if_pending(
            "StepperOnly mode detected - re-homing to establish known position (CCW)",
        )? {
            continue;
        }

        tower.run_tracking(current_datetime.clone());

        info!("Tracking loop duration (v1.1.5): {:?}", now.elapsed());

        tower.maintain_wifi()?;
        tower.publish_heartbeat(&current_datetime);
        tower.report_temperature(&current_datetime);
        tower.process_commands();

        const LOOP_SLEEP_SECS: u64 = 300;
        std::thread::sleep(Duration::from_secs(LOOP_SLEEP_SECS));
    }
}

fn boot_diagnostic(
    device_id: &str,
    wifi: &mut Wifi,
    mqtt: &mut Mqtt,
    current_version: &Version,
) -> bool {
    info!("Starting boot validation in 5 seconds...");
    thread::sleep(Duration::from_secs(5));

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
            thread::sleep(Duration::from_millis(3000));
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
        thread::sleep(Duration::from_millis(1000));
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

/// Enters maintenance mode if the maintenance button is held down on startup.
fn capture_maintenance_mode(
    motion: &mut Motion,
    led: &mut Led,
    maintenance_button: PinDriver<Gpio5, Input>,
    ccw_button: PinDriver<Gpio4, Input>,
    cw_button: PinDriver<Gpio6, Input>
) -> anyhow::Result<()> {
    if maintenance_button.is_low() {
        return Ok(());
    }
    
    log::info!("[Maintenance Mode] Begin");
    led.display_maintenance()?;

    let mut move_direction = None;
    let mut last_maintenance = maintenance_button.is_high();
    let mut last_click_time: Option<Instant> = None;
    const DOUBLE_CLICK_MS: Duration = Duration::from_millis(250);

    loop {
        let ccw = ccw_button.is_high();
        let cw = cw_button.is_high();
        let maintenance = maintenance_button.is_high();
        let maintenance_pressed = maintenance && !last_maintenance;
        last_maintenance = maintenance;

        let now = Instant::now();

        // Don't move if CCW and CW are both pressed
        if ccw && cw {
            move_direction = None
        } 
        // Stop moving if maintenance button is single clicked,
        // or exit mainenance mode if double clicked
        else if maintenance_pressed {
            if let Some(last) = last_click_time {
                if now.duration_since(last) <= DOUBLE_CLICK_MS {
                    break;
                }
            }

            last_click_time = Some(now);
            move_direction = None;
            log::info!("[Maintenance Mode] Not moving");
        }
        else if ccw && !cw {
            move_direction = Some(Direction::Ccw);
            log::info!("[Maintenance Mode] Moving CCW");
        } 
        else if cw && !ccw {
            move_direction = Some(Direction::Cw);
            log::info!("[Maintenance Mode] Moving CW");
        }

        if let Some(direction) = move_direction {
            match direction {
                Direction::Ccw => {
                    led.display_maintenance_moving_ccw()?;
                    motion.move_by(-30_000);
                },
                Direction::Cw => {
                    led.display_maintenance_moving_cw()?;
                    motion.move_by(30_000);
                }
            }
            led.display_maintenance()?;
        }
    }

    log::info!("[Maintenance Mode] Exit, continuing normal boot");
    led.display_none()
}