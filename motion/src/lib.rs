pub mod motion {
    use accel_stepper::{Driver, OperatingSystemClock, StepAndDirection};
    use astronav::coords::noaa_sun::NOAASun;
    use clock::Clock;
    use std::time::{Duration, Instant};
    use esp_idf_svc::hal::gpio::{Gpio15, Gpio16, Gpio17, Gpio14, Gpio47, Gpio21, Input, Output, PinDriver};
    use quadrature_encoder::{IncrementalEncoder, Rotary, HalfStep};
    use esp_idf_svc::nvs::*;
    use network::mqtt::Mqtt;
    use wifi::wifi::{Wifi, WifiState};
    use ota::OtaUpdater;
    use semver::Version;
    use std::thread;

    // Split into focused modules: encoder, homing, move execution.
    mod encoder;
    mod homing;
    mod move_exec;

    // Motor movement constants.
    //
    // Keep these in one place so all step calculations stay consistent.
    pub(crate) const MICROSTEPS: f64 = 25_600.0;
    pub(crate) const GEAR_REDUCTION: f64 = 1.0;
    pub(crate) const SLEW_BEARING: f64 = 84.0;
    pub(crate) const STEPS_PER_REV: f64 = MICROSTEPS * GEAR_REDUCTION * SLEW_BEARING;

    // If your mechanical CW/CCW is flipped vs software commands, toggle this.
    // This inverts the sign of *all* step movements right before they are sent to the driver.
    pub(crate) const INVERT_MOTOR_DIRECTION: bool = true;

    // Stepper driver tuning knobs (steps/s and steps/s^2).
    //
    // Conservative defaults for the NEMA42 + PST2822PH combo based on your bench test.
    pub(crate) const DEFAULT_MAX_SPEED_STEPS_PER_S: f32 = 8_000.0;
    pub(crate) const DEFAULT_ACCEL_STEPS_PER_S2: u16 = 900;

    #[derive(PartialEq, Copy, Clone)]
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
        // Legacy / unused after removing fine-adjust logic; keep for now to minimize churn.
        prev_balance: i32,
        relay: PinDriver<'a, Gpio17, Output>,
        lmsw: PinDriver<'a, Gpio14, Input>,
        encoder: IncrementalEncoder<Rotary, PinDriver<'a, Gpio47, Input>, PinDriver<'a, Gpio21, Input>, HalfStep>,
        // Encoder "reset" is implemented as a software offset: displayed_position = raw - offset.
        encoder_zero_offset: i32,
        // Limit-switch edge detection / debounce state (active-low switch).
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

        // Report last attempted move outcome (read once via `take_last_move_outcome`).
        last_move_outcome: Option<MoveOutcome>,

        // Tracking soft limits (guardrail).
        soft_limits_enabled: bool,
        soft_limit_min_deg: f32,
        soft_limit_max_deg: f32,
    }

    // CW: direction
    // CCW: step
    impl Motion<'_> {
        pub fn new<'a>(p10: Gpio15, p11: Gpio16, p7: Gpio17, p6: Gpio14, p47: Gpio47, p21: Gpio21) -> Motion<'a> {
            let step = PinDriver::output(p10).unwrap();
            let direction = PinDriver::output(p11).unwrap();
            let relay = PinDriver::output(p7).unwrap();
            let mut lmsw = PinDriver::input(p6).unwrap();
            let encoder_a = PinDriver::input(p47).unwrap();
            let encoder_b = PinDriver::input(p21).unwrap();
            lmsw.set_pull(esp_idf_svc::hal::gpio::Pull::Down)
                .unwrap_or_default();

            let encoder = IncrementalEncoder::<Rotary, _, _, HalfStep>::new(encoder_a, encoder_b);

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

                last_move_outcome: None,

                soft_limits_enabled: false,
                soft_limit_min_deg: 0.0,
                soft_limit_max_deg: 290.0,
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
        ) -> bool {
            self.update_position(location);
            log::info!("{},", clock.after_sunrise());
            if clock.after_sunrise() && !clock.after_sunset() {
                // If we're starting the day already at home, ensure encoder ticks are zeroed
                // even though the motor isn't running yet.
                self.force_zero_if_limit_switch_pressed();
                let sun = NOAASun {
                    year: clock.get_year(),
                    doy: clock.get_day() as u16,
                    long: clock.get_longitude() as f32,
                    lat: clock.get_latitude() as f32,
                    timezone: -5.0,
                    hour: clock.get_hour(),
                    min: clock.get_minutes(),
                    sec: clock.get_seconds(),
                };
                log::info!("Tracking in progress");
                let angle_offset_raw = sun.azimuth_in_deg() - (location as f64);

                // Soft limits (guardrail): clamp target heading to avoid mechanical cable wrap / hard stops.
                // We only apply this in daytime tracking moves.
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
                // Single-path daytime tracking:
                // If we're within ±5°, do nothing. Otherwise execute a movement based on offset.
                if angle_offset.abs() <= 5.0 {
                    let _ = self.relay.set_low().unwrap_or_default();
                    return true;
                }

                self.relay.set_high().unwrap_or_default();
                log::info!("Tracking move (|offset| > 5°)");
                let steps = (angle_offset / 360.0) * STEPS_PER_REV;
                        log::info!("Steps Needed: {}", steps as i64);
                let move_outcome = self.move_by(steps as i64);
                if move_outcome != MoveOutcome::Completed {
                    self.relay.set_low().unwrap_or_default();
                    log::warn!("Tracking move aborted: {:?}", move_outcome);
                    // Return true so main does NOT persist heading/snapshot for a move that did not happen.
                    return true;
                }
                        self.update_position((location as f64 + angle_offset) as f32);
                self.relay.set_low().unwrap_or_default();

                // Publish message
                        let payload = format!(
                            "Current datetime: {}, and current tower angle: {}",
                            formatted_time, 
                            location as f64 + angle_offset
                        );
                if publish_mqtt {
                    match mqtt.publish("device1A/data", payload.as_bytes()) {
                        Ok(_) => log::info!("Published data payload successfully"),
                        Err(e) => log::error!("Failed to publish data payload: {:?}", e),
                    }
                } else {
                    log::info!("MQTT publish disabled: skipping tracking data payload publish");
                }
                        return false;
            } 
            else {// Sunset Operation 
                if location == 90.0 {
                    log::info!("Already reached sleep position");
                    // Ensure encoder ticks are truly 0 at home while we sleep.
                    self.force_zero_if_limit_switch_pressed();

                    // Track start time
                    let mut last_check = Instant::now();
                    let check_interval = Duration::from_secs(2 * 60 * 60); // 2 hours

                    // Wait here until sunrise
                    while clock.after_sunset() || !clock.after_sunrise() {
                        if clock.after_sunrise() && !clock.after_sunset() {
                            log::info!("Sunrise detected, exiting sleep loop");
                            break;
                        }
                        if allow_ota && last_check.elapsed() >= check_interval {
                            log::info!("2 hours elapsed, checking for OTA");

                            // Check to see if wifi is disconnected before OTA try
                            log::info!("Current wifi state: {:?}", wifi.state());
                            if wifi.state() == WifiState::Disconnected {
                                let _ = wifi.reconnect_if_disconnected();
                            }

                            // Creates an instance of OTA crate and runs version compare
                            thread::sleep(Duration::from_secs(3));
                            let mut updater = OtaUpdater::new_ota(current_version.clone(), mqtt, Some("device1A"), Some("device1A")).expect("Failed to create OTA adapter instance");

                            thread::sleep(Duration::from_secs(3));
                            let run_compare = updater.run_version_compare(nvs);

                            match run_compare {
                                Ok(_) => log::info!("Version compare succeeded"),
                                Err(e) => {
                                    log::error!("Version compare failed: {:?}", e);
                                }
                            } 

                            last_check = Instant::now(); // reset the timer
                            //break;
                        } else if !allow_ota && last_check.elapsed() >= check_interval {
                            log::info!("OTA disabled: skipping periodic OTA check");
                            last_check = Instant::now();
                        }
                        log::info!("Still waiting for sunrise...");
                        std::thread::sleep(std::time::Duration::from_secs(600)); // Prevent busy waiting
                    }

                    return true;
                } else {
                    log::info!("Moving to sleep position...");
                    let limit_sw_status = self.find_limit_switch_ccw();
                    match limit_sw_status{
                        true => {
                            log::info!("Limit switch has returned true");

                            // Publish + persist daily home error (ticks) if we captured it.
                            if let Some(home_error_ticks) = self.take_last_home_error_ticks() {
                                let payload = format!("{}", home_error_ticks);
                                if publish_mqtt {
                                    if let Err(e) = mqtt.publish("device1A/tower/home_error_ticks", payload.as_bytes()) {
                                        log::error!("Failed to publish home_error_ticks: {:?}", e);
                                    } else {
                                        log::info!("Published home_error_ticks={}", home_error_ticks);
                                    }
                                } else {
                                    log::info!("MQTT publish disabled: skipping home_error_ticks publish");
                                }

                                if persist_nvs {
                                    if let Err(e) = nvs.set_i32("home_error_ticks", home_error_ticks) {
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
                            loop{
                                if publish_mqtt {
                                    if let Err(e) = mqtt.publish("device1A/tower/status", b"Critical failure: Limit switch failure!") {
                                        log::error!("Failed to publish critical error message: {:?}", e);
                                    }
                                } else {
                                    log::error!("Critical failure: Limit switch failure! (MQTT disabled)");
                                }
                                thread::sleep(Duration::from_secs(900));// Loop every 15 minutes
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
