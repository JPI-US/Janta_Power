pub mod motion {
    use accel_stepper::{Driver, OperatingSystemClock, StepAndDirection};
    use astronav::coords::noaa_sun::NOAASun;
    use chrono::{Datelike, Local, Timelike};
    use clock::Clock;
    use std::time::{Duration, Instant};
    use esp_idf_svc::hal::gpio::{Gpio10, Gpio11, Gpio14, Gpio15, Gpio16, Gpio17, Input, Output, PinDriver};
    use quadrature_encoder::{IncrementalEncoder, Rotary, QuadStep};
    use esp_idf_svc::nvs::*;
    use network::mqtt::Mqtt;
    use wifi::wifi::{Wifi, WifiState};
    use ota::OtaUpdater;
    use semver::Version;
    use std::thread;

    // Focused motion modules.
    mod encoder;
    mod homing;
    mod move_exec;

    // Build-time constants generated from .env.
    include!(concat!(env!("OUT_DIR"), "/constants.rs"));

    // ESP-IDF NVS key names are limited to 15 characters.
    const NVS_KEY_HOME_ERROR_TICKS: &str = "home_err_ticks";

    /// `tower/status` payload: limit switch not found during sunset verify or sleep homing.
    const CRITICAL_TOWER_LIMIT_SWITCH: &[u8] =
        b"Critical failure: Limit switch failure at the Office Tower!";

    // Keep step math in one place.
    pub(crate) const STEPS_PER_REV: f64 = MICROSTEPS * GEAR_REDUCTION * SLEW_BEARING;

    #[derive(PartialEq, Copy, Clone, Debug)]
    pub enum MotionMode {
        // Stepper-only movement (open-loop).
        StepperOnly,
        // Stepper movement with encoder-based guardrails (stall detection, snapshots, etc.).
        EncoderGuarded,
    }

    #[derive(Debug, PartialEq, Copy, Clone)]
    pub enum MoveOutcome {
        Completed,
        AbortedPowerMissing,
        AbortedStall,
        AbortedOvershoot,
    }

    pub fn calculate_steps(offset_deg: f32) -> i64 {
        ((offset_deg as f64 / 360.0) * STEPS_PER_REV) as i64
    }

    pub struct Motion<'a> {
        location: f32,
        motion_mode: MotionMode,
        speed: f32,
        acceleration: u16,
        motor: Driver,
        motor_device:
            StepAndDirection<PinDriver<'a, Gpio15, Output>, PinDriver<'a, Gpio16, Output>>,
        motor_clock: OperatingSystemClock,
        // Legacy field kept for compatibility.
        prev_balance: i32,
        relay: PinDriver<'a, Gpio17, Output>,
        lmsw: PinDriver<'a, Gpio14, Input>,
        encoder: IncrementalEncoder<Rotary, PinDriver<'a, Gpio10, Input>, PinDriver<'a, Gpio11, Input>, QuadStep>,
        // Encoder zero is a software offset: adjusted = raw - offset.
        encoder_zero_offset: i32,
        // Limit-switch debounce state (active-low switch).
        lmsw_last_state_pressed: bool,
        lmsw_last_change: Instant,
        lmsw_zeroed_this_press: bool,
        // captured when the limit switch is hit (before we re-zero).
        last_home_error_ticks: Option<i32>,

        // Stall detector state.
        motor_power_on: bool,
        stall_detection_enabled: bool,
        stall_last_check: Instant,
        stall_step_pos_at_last_enc_change: i64,
        stall_last_enc_ticks_seen: i32,
        stall_reported: bool,
        stall_consecutive: u8,
        
        // Ratio-based stall detection state (EncoderGuarded mode only).
        stall_check_start_encoder_ticks: i32,
        stall_check_start_step_pos: i64,
        stall_check_last_interval_step: i64,

        // Encoder overshoot protection state (EncoderGuarded mode only).
        overshoot_enc_start: Option<i32>,
        overshoot_expected_ticks: Option<i64>,

        // Last attempted move outcome, consumed by `take_last_move_outcome`.
        last_move_outcome: Option<MoveOutcome>,

        // Tracking soft limits.
        soft_limits_enabled: bool,
        soft_limit_min_deg: f32,
        soft_limit_max_deg: f32,

        // During homing, overshoot checks are disabled.
        is_homing: bool,
    }

    // Direction and step wiring notes:
    // - CW controls direction pin
    // - CCW steps by convention in this wiring
    impl Motion<'_> {
        pub fn new<'a>(
            step_pin: Gpio15,
            direction_pin: Gpio16,
            relay_pin: Gpio17,
            limit_switch_pin: Gpio14,
            encoder_a_pin: Gpio10,
            encoder_b_pin: Gpio11,
        ) -> Motion<'a> {
            let step = PinDriver::output(step_pin).unwrap();
            let direction = PinDriver::output(direction_pin).unwrap();
            // Relay is active-low: boot with relay OFF.
            let mut relay = PinDriver::output(relay_pin).unwrap();
            relay.set_high().unwrap_or_default();
            let mut lmsw = PinDriver::input(limit_switch_pin).unwrap();
            let encoder_a = PinDriver::input(encoder_a_pin).unwrap();
            let encoder_b = PinDriver::input(encoder_b_pin).unwrap();
            lmsw.set_pull(esp_idf_svc::hal::gpio::Pull::Down)
                .unwrap_or_default();

            let encoder = IncrementalEncoder::<Rotary, _, _, QuadStep>::new(encoder_a, encoder_b);

            let now = Instant::now();
            Motion {
                location: 0.0,
                motion_mode: MotionMode::EncoderGuarded,
                speed: DEFAULT_MAX_SPEED_STEPS_PER_S,
                acceleration: DEFAULT_ACCEL_STEPS_PER_S2,
                motor: Driver::new(),
                motor_device: StepAndDirection::new(step, direction),
                motor_clock: OperatingSystemClock::new(),
                prev_balance: 0,
                relay,
                lmsw,
                encoder,
                encoder_zero_offset: 0,
                lmsw_last_state_pressed: false,
                lmsw_last_change: now,
                lmsw_zeroed_this_press: false,
                last_home_error_ticks: None,

                motor_power_on: true,
                stall_detection_enabled: true,
                stall_last_check: now,
                stall_step_pos_at_last_enc_change: 0,
                stall_last_enc_ticks_seen: 0,
                stall_reported: false,
                stall_consecutive: 0,
                
                stall_check_start_encoder_ticks: 0,
                stall_check_start_step_pos: 0,
                stall_check_last_interval_step: 0,

                overshoot_enc_start: None,
                overshoot_expected_ticks: None,

                last_move_outcome: None,

                soft_limits_enabled: false,
                soft_limit_min_deg: 0.0,
                soft_limit_max_deg: 290.0,

                is_homing: false,
            }
        }

        pub fn update_position(&mut self, location: f32) {
            self.location = location;
        }

        pub fn set_stall_detection_enabled(&mut self, enabled: bool) {
            self.stall_detection_enabled = enabled;
        }

        pub fn stall_detection_enabled(&self) -> bool {
            self.stall_detection_enabled
        }

        pub fn set_soft_limits(&mut self, enabled: bool, min_deg: f32, max_deg: f32) {
            self.soft_limits_enabled = enabled;
            self.soft_limit_min_deg = min_deg;
            self.soft_limit_max_deg = max_deg;
        }

        pub fn set_motion_mode(&mut self, mode: MotionMode) {
            self.motion_mode = mode;
        }

        pub fn motion_mode(&self) -> MotionMode {
            self.motion_mode
        }

        pub fn set_motor_power_on(&mut self, power_on: bool) {
            self.motor_power_on = power_on;
        }

        pub fn location(&mut self) -> f32 {
            self.location
        }

        pub fn flip_relay(&mut self) {
            self.relay.toggle().unwrap_or_default();
        }

        #[inline]
        pub(crate) fn relay_on(&mut self) {
            // Active-low: LOW = ON
            self.relay.set_low().unwrap_or_default();
        }

        #[inline]
        pub(crate) fn relay_off(&mut self) {
            // Active-low: HIGH = OFF
            self.relay.set_high().unwrap_or_default();
        }



        pub fn set_tower_position<I2C: embedded_hal::i2c::I2c, T: NvsPartitionId>(
            &mut self,
            clock: &mut Clock<I2C>,
            location: f32,
            _balance: i32,
            mqtt: &mut Mqtt,
            current_version: Version,
            nvs: &mut EspNvs<T>,
            wifi: &mut Wifi<'_>,
            formatted_time: String,
            publish_mqtt: bool,
            persist_nvs: bool,
            allow_ota: bool,
            device_id: &str,
        ) -> bool {
            self.update_position(location);
            log::info!("{},", clock.after_sunrise());
            if clock.after_sunrise() && !clock.after_sunset() {
                // If already at home, keep encoder zeroed before daytime tracking.
                self.force_zero_if_limit_switch_pressed();
                // NOAA expects local civil date/time + tz offset. DS3231 holds UTC; use libc local time
                // (same instant as settimeofday after RTC/NTP) so h/m/s match tz_offset_h.
                let now = Local::now();
                let timezone_hours = (now.offset().local_minus_utc() as f32) / 3600.0;
                let sun = NOAASun {
                    year: now.year() as u16,
                    doy: now.ordinal() as u16,
                    long: clock.get_longitude() as f32,
                    lat: clock.get_latitude() as f32,
                    timezone: timezone_hours,
                    hour: now.hour() as u8,
                    min: now.minute() as u8,
                    sec: now.second() as u8,
                };
                let rtc_naive = clock.get_date_time();
                log::info!(
                    "NOAA inputs: year={} doy={} lat={:.6} long={:.6} tz_offset_h={:.3} | h={} m={} s={} (Local civil, libc TZ)",
                    sun.year,
                    sun.doy,
                    sun.lat,
                    sun.long,
                    timezone_hours,
                    sun.hour,
                    sun.min,
                    sun.sec
                );
                log::info!(
                    "NOAA time cross-check: Local::now={} | DS3231 UTC naive={}",
                    now.format("%Y-%m-%d %H:%M:%S %:z"),
                    rtc_naive
                );
                log::info!("Tracking in progress");
                let angle_offset_raw = sun.azimuth_in_deg() - (location as f64);

                // Clamp daytime target heading to soft limits.
                let target_raw = (location as f64) + angle_offset_raw;
                let target_clamped = if self.soft_limits_enabled {
                    let min = self.soft_limit_min_deg as f64;
                    let max = self.soft_limit_max_deg as f64;
                    if target_raw < min {
                        log::warn!(
                            "SOFT_LIMIT clamp: target_raw={:.2} < min={:.2} -> clamping",
                            target_raw,
                            min
                        );
                        min
                    } else if target_raw > max {
                        log::warn!(
                            "SOFT_LIMIT clamp: target_raw={:.2} > max={:.2} -> clamping",
                            target_raw,
                            max
                        );
                        max
                    } else {
                        target_raw
                    }
                } else {
                    target_raw
                };

                let angle_offset = target_clamped - (location as f64);
                log::info!("Actual Location: {}", location);
                log::info!(
                    "Angle Offset: {} (raw_offset={} target_raw={} target_clamped={})",
                    angle_offset,
                    angle_offset_raw,
                    target_raw,
                    target_clamped
                );
                log::info!("Sun Angle: {}", sun.azimuth_in_deg());
                // Daytime tracking: no move in deadband, otherwise step by offset.
                if angle_offset.abs() <= TRACKING_DEADBAND_DEG as f64 {
                    self.relay_off();
                    return true;
                }

                self.relay_on();
                log::info!("Tracking move (|offset| > {}°)", TRACKING_DEADBAND_DEG);
                let steps = (angle_offset / 360.0) * STEPS_PER_REV;
                        log::info!("Steps Needed: {}", steps as i64);
                let move_outcome = self.move_by(steps as i64);
                if move_outcome != MoveOutcome::Completed {
                    self.relay_off();
                    log::warn!("Tracking move aborted: {:?}", move_outcome);
                    // Return true so main does NOT persist heading/snapshot for a move that did not happen.
                    return true;
                }
                        self.update_position((location as f64 + angle_offset) as f32);
                self.relay_off();

                let tower_angle = location as f64 + angle_offset;
                let payload = serde_json::json!({
                    "current_time": formatted_time,
                    "tower_angle": tower_angle,
                })
                .to_string();
                if publish_mqtt {
                    let topic = format!("tower/{}/data/angle", device_id);
                    match mqtt.publish(&topic, payload.as_bytes()) {
                        Ok(_) => log::info!("Published tower angle to {}", topic),
                        Err(e) => log::error!("Failed to publish tower angle: {:?}", e),
                    }
                } else {
                    log::info!("MQTT publish disabled: skipping tower angle publish");
                }
                        return false;
            } 
            else {// Sunset Operation 
                if (location - HOME_HEADING_DEG).abs() < 0.01 {
                    // Verify home physically when heading says home.
                    if self.lmsw.is_high() {
                        log::warn!(
                            "Heading near home but limit switch not pressed; verifying home by homing CCW"
                        );
                        let ok = self.find_limit_switch_ccw();
                        if ok {
                            log::info!("Home verification homing succeeded");

                            // Publish and persist home error ticks if captured.
                            if let Some(home_error_ticks) = self.take_last_home_error_ticks() {
                                let payload = format!("{}", home_error_ticks);
                                if publish_mqtt {
                                    let topic = format!("{}/tower/home_error_ticks", device_id);
                                    if let Err(e) = mqtt.publish(&topic, payload.as_bytes()) {
                                        log::error!(
                                            "Failed to publish home_error_ticks: {:?}",
                                            e
                                        );
                                    } else {
                                        log::info!(
                                            "Published home_error_ticks={}",
                                            home_error_ticks
                                        );
                                    }
                                } else {
                                    log::info!(
                                        "MQTT publish disabled: skipping home_error_ticks publish"
                                    );
                                }

                                if persist_nvs {
                                    if let Err(e) =
                                        nvs.set_i32(NVS_KEY_HOME_ERROR_TICKS, home_error_ticks)
                                    {
                                        log::warn!(
                                            "Failed to store home_error_ticks in NVS: {:?}",
                                            e
                                        );
                                    } else {
                                        log::info!(
                                            "Stored home_error_ticks in NVS: {}",
                                            home_error_ticks
                                        );
                                    }
                                } else {
                                    log::warn!(
                                        "NVS persist disabled: skipping home_error_ticks store"
                                    );
                                }
                            } else {
                                log::warn!("No home_error_ticks captured on this homing run");
                            }
                        } else {
                            log::error!(
                                "Home verification failed: limit switch could not be found"
                            );
                            // Critical: sunset home verification could not find limit switch.
                            loop {
                                if publish_mqtt {
                                    let t = format!("{}/tower/status", device_id);
                                    if let Err(e) = mqtt.publish(&t, CRITICAL_TOWER_LIMIT_SWITCH) {
                                        log::warn!(
                                            "Failed to publish critical status to {}: {:?}",
                                            t,
                                            e
                                        );
                                    }
                                } else {
                                    log::info!(
                                        "MQTT publish disabled: {}",
                                        std::str::from_utf8(CRITICAL_TOWER_LIMIT_SWITCH)
                                            .unwrap_or("critical")
                                    );
                                }
                                thread::sleep(Duration::from_secs(900));
                            }
                        }
                    }

                    log::info!("At sleep position");
                    // Ensure encoder ticks are truly 0 at home while we sleep.
                    self.force_zero_if_limit_switch_pressed();

                    let mut last_check = Instant::now();
                    let check_interval = Duration::from_secs(2 * 60 * 60);

                    while clock.after_sunset() || !clock.after_sunrise() {
                        if clock.after_sunrise() && !clock.after_sunset() {
                            log::info!("Sunrise detected, exiting sleep loop");
                            break;
                        }
                        if allow_ota && last_check.elapsed() >= check_interval {
                            log::info!("2 hours elapsed, checking for OTA");

                            log::info!("Current wifi state: {:?}", wifi.state());
                            if wifi.state() == WifiState::Disconnected {
                                let _ = wifi.reconnect_if_disconnected();
                            }

                            thread::sleep(Duration::from_secs(3));
                            let mut updater = OtaUpdater::new_ota(current_version.clone(), mqtt, device_id, Some("device1A"), Some("device1A")).expect("Failed to create OTA adapter instance");

                            thread::sleep(Duration::from_secs(3));
                            let run_compare = updater.run_version_compare(nvs);

                            match run_compare {
                                Ok(_) => log::info!("Version compare succeeded"),
                                Err(e) => {
                                    log::error!("Version compare failed: {:?}", e);
                                }
                            } 

                            last_check = Instant::now();
                        } else if !allow_ota && last_check.elapsed() >= check_interval {
                            log::info!("OTA disabled: skipping periodic OTA check");
                            last_check = Instant::now();
                        }
                        log::info!("Still waiting for sunrise...");
                        std::thread::sleep(std::time::Duration::from_secs(600));
                    }

                    return true;
                } else {
                    log::info!("Moving to sleep position...");
                    let limit_sw_status = self.find_limit_switch_ccw();
                    match limit_sw_status{
                        true => {
                            log::info!("Limit switch has returned true");

                            // Publish and persist home error ticks if captured.
                            if let Some(home_error_ticks) = self.take_last_home_error_ticks() {
                                let payload = format!("{}", home_error_ticks);
                                if publish_mqtt {
                                    let topic = format!("{}/tower/home_error_ticks", device_id);
                                    if let Err(e) = mqtt.publish(&topic, payload.as_bytes()) {
                                        log::error!("Failed to publish home_error_ticks: {:?}", e);
                                    } else {
                                        log::info!("Published home_error_ticks={}", home_error_ticks);
                                    }
                                } else {
                                    log::info!("MQTT publish disabled: skipping home_error_ticks publish");
                                }

                                if persist_nvs {
                                    if let Err(e) =
                                        nvs.set_i32(NVS_KEY_HOME_ERROR_TICKS, home_error_ticks)
                                    {
                                        log::warn!("Failed to store home_error_ticks in NVS: {:?}", e);
                                    } else {
                                        log::info!("Stored home_error_ticks in NVS: {}", home_error_ticks);
                                    }
                                } else {
                                    log::warn!("NVS persist disabled: skipping home_error_ticks store");
                                }
                            } else {
                                log::warn!("No home_error_ticks captured on this homing run");
                            }
                        },
                        false => {
                            log::error!("Limit switch has returned false, limit switch could not be found");
                            // Critical: move-to-sleep homing could not find limit switch.
                            loop {
                                if publish_mqtt {
                                    let t = format!("{}/tower/status", device_id);
                                    if let Err(e) = mqtt.publish(&t, CRITICAL_TOWER_LIMIT_SWITCH) {
                                        log::warn!(
                                            "Failed to publish critical status to {}: {:?}",
                                            t,
                                            e
                                        );
                                    }
                                } else {
                                    log::info!(
                                        "MQTT publish disabled: {}",
                                        std::str::from_utf8(CRITICAL_TOWER_LIMIT_SWITCH)
                                            .unwrap_or("critical")
                                    );
                                }
                                thread::sleep(Duration::from_secs(900));
                            }
                        }
                    }
                    log::info!("Tower has reached sleep position");
                    return false;
                }
            }
        }
    }
}

pub use motion::{calculate_steps, Motion, MotionMode, MoveOutcome};
