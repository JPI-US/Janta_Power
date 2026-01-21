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
    use std::{thread, panic};

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

    pub fn calculate_steps(offset: f32) -> i64 {
        return ((offset / 360.0) * (25600.0 * 50.0 * 84.0)) as i64;
    }

    // Encoder calibration (output shaft): 348,323 ticks per full revolution.
    const ENC_TICKS_PER_REV: f32 = 348_323.0;
    const ENC_TICKS_PER_DEG: f32 = ENC_TICKS_PER_REV / 360.0;

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

        // ======== Stage 2: stall detector (log-only for now) ========
        motor_power_on: bool,
        stall_last_check: Instant,
        stall_step_pos_at_last_enc_change: i64,
        stall_last_enc_ticks_seen: i32,
        stall_reported: bool,
        stall_consecutive: u8,

        // ======== Stage 3: report last attempted move outcome ========
        last_move_outcome: Option<MoveOutcome>,
    }

    // CW: direction
    // CCW: step
    impl Motion<'_> {
        pub fn new<'a>(p10: Gpio15, p11: Gpio16, p7: Gpio17, p6: Gpio14, p47: Gpio47, p21: Gpio21) -> Motion<'a> {
            let step = PinDriver::output(p10).unwrap();
            let direction = PinDriver::output(p11).unwrap();
            let relay = PinDriver::output(p7).unwrap();
            let mut lmsw = PinDriver::input(p6).unwrap();
            let encoderA = PinDriver::input(p47).unwrap();
            let encoderB = PinDriver::input(p21).unwrap();
            lmsw.set_pull(esp_idf_svc::hal::gpio::Pull::Down)
                .unwrap_or_default();

            let encoder = IncrementalEncoder::<Rotary, _, _, HalfStep>::new(encoderA, encoderB);

            let now = Instant::now();
            Motion {
                location: 0.0,
                motion_mode: MotionMode::EncoderGuarded,
                speed: 43000.0,
                acceleration: 25600,
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
                stall_last_check: now,
                stall_step_pos_at_last_enc_change: 0,
                stall_last_enc_ticks_seen: 0,
                stall_reported: false,
                stall_consecutive: 0,

                last_move_outcome: None,
            }
        }

        pub fn update_position(&mut self, location: f32) {
            self.location = location;
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

        pub fn switch_pressed(&mut self) -> bool {
            self.lmsw.is_low()
        }

        // Convention: CW is positive; 0 ticks corresponds to the limit switch (home) after zeroing.
        pub fn encoder_ticks_adjusted(&self) -> i32 {
            self.encoder.position() - self.encoder_zero_offset
        }

        // Raw encoder ticks from the quadrature decoder (typically resets to 0 on reboot).
        pub fn encoder_ticks_raw(&self) -> i32 {
            self.encoder.position()
        }

        // Restore the software zero offset so adjusted ticks can be reconstructed after reboot.
        pub fn set_encoder_zero_offset(&mut self, zero_offset: i32) {
            self.encoder_zero_offset = zero_offset;
        }

        /// Convert current encoder position into a heading (degrees), assuming:
        /// - The limit switch (home) corresponds to `home_heading_deg`
        /// - Positive encoder ticks correspond to increasing heading CW
        pub fn heading_from_encoder_ticks(&self, home_heading_deg: f32) -> f32 {
            let deg = (self.encoder_ticks_adjusted() as f32) / ENC_TICKS_PER_DEG;
            (home_heading_deg + deg).rem_euclid(360.0)
        }

        // retrieve and clear the most recent home error
        pub fn take_last_home_error_ticks(&mut self) -> Option<i32> {
            self.last_home_error_ticks.take()
        }

        pub fn take_last_move_outcome(&mut self) -> Option<MoveOutcome> {
            self.last_move_outcome.take()
        }

        /// Tiny diagnostic move: step a small amount and check whether encoder ticks change.
        ///
        /// Intended for "encoder might be unplugged / recovered" probing. This does NOT use
        /// ticks as a servo; it simply checks for *any* tick movement.
        pub fn probe_encoder_motion(&mut self, probe_steps: i64) -> bool {
            let start_ticks = self.encoder_ticks_adjusted();
            self.relay.set_high().unwrap_or_default();
            let outcome = self.move_by(probe_steps);
            self.relay.set_low().unwrap_or_default();

            if outcome != MoveOutcome::Completed {
                log::warn!("Encoder probe aborted: {:?}", outcome);
                return false;
            }

            let end_ticks = self.encoder_ticks_adjusted();
            let moved = end_ticks != start_ticks;
            log::info!(
                "Encoder probe complete: start_ticks={} end_ticks={} moved={}",
                start_ticks,
                end_ticks,
                moved
            );
            moved
        }

        // so `run()` may not be executing and we still want adjusted ticks to be 0 at home
        fn force_zero_if_limit_switch_pressed(&mut self) {
            if self.lmsw.is_low() {
                // Capture drift before re-zeroing (based on current offset)
                let home_error = self.encoder_ticks_adjusted();
                self.last_home_error_ticks = Some(home_error);

                // Make adjusted ticks 0 at the switch
                self.encoder_zero_offset = self.encoder.position();

                // Keep the debouncer state consistent (pressed + already zeroed for this press)
                self.lmsw_last_state_pressed = true;
                self.lmsw_last_change = Instant::now();
                self.lmsw_zeroed_this_press = true;

                log::info!(
                    "Limit switch active: forced encoder zero (home_error_ticks={}, offset={})",
                    home_error,
                    self.encoder_zero_offset
                );
            }
        }

        pub fn init(&mut self) {
            self.motor.set_max_speed(self.speed);
            self.motor.set_speed(self.speed);
            self.motor.set_acceleration(self.acceleration.into());
        }

        pub fn move_by(&mut self, location: i64) -> MoveOutcome {
            // Reset stall detector baseline for this move so we don't accidentally compare
            // against stale values from a previous run.
            let now = Instant::now();
            self.stall_last_check = now;
            self.stall_step_pos_at_last_enc_change = self.motor.current_position();
            self.stall_last_enc_ticks_seen = self.encoder_ticks_adjusted();
            self.stall_reported = false;
            self.stall_consecutive = 0;

            self.motor.move_by(location);
            let outcome = self.run();
            self.last_move_outcome = Some(outcome);
            outcome
        }

        pub fn move_by_ticks(&mut self, location: i64) -> MoveOutcome {
            let now = Instant::now();
            self.stall_last_check = now;
            self.stall_step_pos_at_last_enc_change = self.motor.current_position();
            self.stall_last_enc_ticks_seen = self.encoder_ticks_adjusted();
            self.stall_reported = false;
            self.stall_consecutive = 0;

            self.motor.move_by(location);
            let outcome = self.run();
            self.last_move_outcome = Some(outcome);
            outcome
        }
        

        pub fn run(&mut self) -> MoveOutcome {
            let mut t0 = Instant::now();
            loop {
                if self.motor.is_running() {
                    let _ = self.motor.poll(&mut self.motor_device, &self.motor_clock);
                    let _ = self.encoder.poll();

                    // ======== Stage 3: immediate abort when power is missing (EncoderGuarded only) ========
                    if self.motion_mode == MotionMode::EncoderGuarded && !self.motor_power_on {
                        log::warn!("MOVE_ABORT power_missing=true: stopping motor immediately");
                        let pos = self.motor.current_position();
                        self.motor.set_current_position(pos); // hard stop
                        return MoveOutcome::AbortedPowerMissing;
                    }

                    // ======== Stage 2: Stall detector (L2 only, log-only) ========
                    // Detect "motor stepping but encoder not moving" at a fixed cadence.
                    if self.motion_mode == MotionMode::EncoderGuarded
                        && self.stall_last_check.elapsed() >= Duration::from_millis(250)
                    {
                        // Your logs show that the encoder can take ~20k stepper steps before the first tick changes.
                        // So we detect stall based on "too many steps with NO encoder tick change", not based on
                        // whether ticks change in a short time window.
                        const MAX_STEPS_WITHOUT_ENC_CHANGE: i64 = 120_000;

                        let step_pos = self.motor.current_position();
                        let enc_pos = self.encoder_ticks_adjusted();

                        if enc_pos != self.stall_last_enc_ticks_seen {
                            // Encoder moved -> reset baseline.
                            self.stall_last_enc_ticks_seen = enc_pos;
                            self.stall_step_pos_at_last_enc_change = step_pos;
                            self.stall_reported = false;
                            self.stall_consecutive = 0;
                        }

                        let steps_since_enc_change =
                            (step_pos - self.stall_step_pos_at_last_enc_change).abs();
                        let stalled = steps_since_enc_change >= MAX_STEPS_WITHOUT_ENC_CHANGE;

                        if stalled && !self.stall_reported {
                            log::warn!(
                                "STALL_DETECTED power_on={} steps_since_enc_change={} step_pos={} enc_pos={} (threshold={})",
                                self.motor_power_on,
                                steps_since_enc_change,
                                step_pos,
                                enc_pos,
                                MAX_STEPS_WITHOUT_ENC_CHANGE
                            );
                            self.stall_reported = true;
                        }

                        // Stage 3: abort the move once we've exceeded the allowed step budget without
                        // any encoder tick change.
                        if stalled {
                            log::error!("MOVE_ABORT stall_confirmed=true: stopping motor immediately");
                            let pos = self.motor.current_position();
                            self.motor.set_current_position(pos); // hard stop
                            return MoveOutcome::AbortedStall;
                        }

                        self.stall_last_check = Instant::now();
                    }

                    // Reset encoder count to 0 when the limit switch is pressed (edge-triggered + debounced)
                    // The switch is active-low in this codebase (pressed => is_low()).
                    let pressed = self.lmsw.is_low();
                    let now = Instant::now();
                    if pressed != self.lmsw_last_state_pressed {
                        self.lmsw_last_state_pressed = pressed;
                        self.lmsw_last_change = now;
                        // Allow re-zeroing after a release.
                        if !pressed {
                            self.lmsw_zeroed_this_press = false;
                        }
                    }

                    // Simple time-based debounce: require stable pressed state for 30ms.
                    if pressed
                        && !self.lmsw_zeroed_this_press
                        && self.lmsw_last_change.elapsed() >= Duration::from_millis(30)
                    {
                        // Capture how far off we were from "perfect home" right before we re-zero.
                        let home_error = self.encoder_ticks_adjusted();
                        self.last_home_error_ticks = Some(home_error);
                        self.encoder_zero_offset = self.encoder.position();
                        self.lmsw_zeroed_this_press = true;
                        log::info!(
                            "Limit switch pressed: home_error_ticks={}, encoder zeroed (offset={})",
                            home_error,
                            self.encoder_zero_offset
                        );
                    }
                    
                    if t0.elapsed() >= Duration::from_millis(100) {
                        let position = self.encoder_ticks_adjusted();
                        let step_pos = self.motor.current_position();
                        let step_rem = self.motor.distance_to_go();
                        log::info!(
                            "Encoder Ticks: {}, Step Position: {}, Step Remaining: {}",
                            position,
                            step_pos,
                            step_rem
                        );
                        t0 = Instant::now();
                    }
                } else {
                    break;
                }
            }
            MoveOutcome::Completed
        }

        pub fn flip_relay(&mut self) {
            self.relay.toggle().unwrap_or_default();
        }

        pub fn find_limit_switch_cw(&mut self) -> bool {
           
            if self.lmsw.is_low() {
                log::info!("Found Limit Switch, Heading : 90");
                self.update_position(90.0);
                self.force_zero_if_limit_switch_pressed();
                return true;
            }

            log::info!("Move 15 Degress clockwise first");
            self.relay.set_high().unwrap_or_default();

            let correction_factor = 1.231;
            let steps = (15.0 / 360.0) * (25600.0 * 50.0 * 84.0);
            log::info!("Steps Needed: {}", steps);
            log::info!("Steps Needed: {}", steps as i64);
            if self.move_by(steps as i64) != MoveOutcome::Completed {
                self.relay.set_low().unwrap_or_default();
                return false;
            }
            log::info!("Done moving 15 Degress clockwise");
            
            log::info!("Now, looking for the limit switch");

            let mut max_steps = calculate_steps(-360.0);
            while (max_steps < 0 && self.lmsw.is_high()) {
                let step_movement = calculate_steps(-1.0);
                if self.move_by(step_movement) != MoveOutcome::Completed {
                    self.relay.set_low().unwrap_or_default();
                    return false;
                }
                max_steps -= step_movement;
            }

            self.relay.set_low().unwrap_or_default();
            if max_steps < 0 {
                log::info!("Found Limit Switch, Heading : 90");
                self.update_position(90.0);
                self.relay.set_low().unwrap_or_default();
                self.force_zero_if_limit_switch_pressed();
                return true;
            }
            log::error!("Limit Switch was not found!");
            return false;
        }


        pub fn find_limit_switch_ccw(&mut self) -> bool {
            
            if self.lmsw.is_low() {
                self.update_position(90.0);
                self.force_zero_if_limit_switch_pressed();
                return true;
            }
            
            log::info!("Move 15 Degress clockwise first");
            self.relay.set_high().unwrap_or_default();

            let correction_factor = 1.231;
            let steps = (15.0 / -360.0) * (25600.0 * 50.0 * 84.0);
            log::info!("Steps Needed: {}", steps);
            log::info!("Steps Needed: {}", steps as i64);
            if self.move_by(steps as i64) != MoveOutcome::Completed {
                self.relay.set_low().unwrap_or_default();
                return false;
            }
            log::info!("Done moving 15 Degress clockwise");
            log::info!("Now, looking for the limit switch");

            let mut max_steps = calculate_steps(360.0); // full CW
            while (max_steps > 0 && self.lmsw.is_high()) {
                let step_movement = calculate_steps(1.0); // Move 1 deg at a time
                if self.move_by(step_movement) != MoveOutcome::Completed {
                    self.relay.set_low().unwrap_or_default();
                    return false;
                }
                max_steps -= step_movement;
            }

            self.relay.set_low().unwrap_or_default();

            if max_steps > 0 {
                self.update_position(90.0);
                self.relay.set_low().unwrap_or_default();
                self.force_zero_if_limit_switch_pressed();
                return true;
            }
            false
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
                let angle_offset = sun.azimuth_in_deg() - (location as f64);
                log::info!("Actual Location: {}", location);
                log::info!("Angle Offset: {}", angle_offset);
                log::info!("Sun Angle: {}", sun.azimuth_in_deg());
                // Single-path daytime tracking:
                // If we're within ±5°, do nothing. Otherwise execute a movement based on offset.
                if angle_offset.abs() <= 5.0 {
                    let _ = self.relay.set_low().unwrap_or_default();
                    return true;
                }

                self.relay.set_high().unwrap_or_default();
                log::info!("Tracking move (|offset| > 5°)");
                let steps = (angle_offset / 360.0) * (25600.0 * 50.0 * 84.0);
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
                match mqtt.publish("device1A/data", payload.as_bytes()) {
                    Ok(_) => log::info!("Published data payload successfully"),
                    Err(e) => log::error!("Failed to publish data payload: {:?}", e),
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
                        if last_check.elapsed() >= check_interval {
                            log::info!("2 hours elapsed, checking for OTA");

                            // Check to see if wifi is disconnected before OTA try
                            log::info!("Current wifi state: {:?}", wifi.state());
                            if wifi.state() == WifiState::Disconnected{
                                wifi.reconnect_if_disconnected();
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
                        }
                        log::info!("Still waiting for sunrise...");
                        std::thread::sleep(std::time::Duration::from_secs(600)); // Prevent busy waiting
                    }

                    return true;
                } else {
                    log::info!("Moving to sleep position...");
                    let limit_sw_status = self.find_limit_switch_cw(); // change to ccw for waco
                    match limit_sw_status{
                        true => {
                            log::info!("Limit switch has returned true");

                            // Stage 4: publish + persist daily home error (ticks) if we captured it.
                            if let Some(home_error_ticks) = self.take_last_home_error_ticks() {
                                let payload = format!("{}", home_error_ticks);
                                if let Err(e) = mqtt.publish("device1A/tower/home_error_ticks", payload.as_bytes()) {
                                    log::error!("Failed to publish home_error_ticks: {:?}", e);
                                } else {
                                    log::info!("Published home_error_ticks={}", home_error_ticks);
                                }

                                if let Err(e) = nvs.set_i32("home_error_ticks", home_error_ticks) {
                                    log::warn!("Failed to store home_error_ticks in NVS: {:?}", e);
                                } else {
                                    log::info!("Stored home_error_ticks in NVS: {}", home_error_ticks);
                                }
                            } else {
                                log::warn!("No home_error_ticks captured on this homing run");
                            }
                        },
                        false => {
                            log::error!("Limit switch has returned false, limit switch could not be found");
                            loop{
                                if let Err(e) = mqtt.publish("device1A/tower/status", b"Critical failure: Limit switch failure!") {
                                    log::error!("Failed to publish critical error message: {:?}", e);
                                }
                                thread::sleep(Duration::from_secs(900));// Loop every 15 minutes
                            }
                        }
                    }
                    log::info!("Tower has reached sleep position");
                    return false;
                }
            }
            // Default fall-through (should generally be unreachable due to early returns above).
            true
            //note to self: maybe remove?
            /*else if clock.after_sunset() {
                 if false {
                    let angle_offset = 90.0 - location;
                    let steps = (angle_offset / 360.0) * (20000.0 * 50.0 * 84.0);
                    log::info!("Steps Needed: {}", steps);
                    log::info!("Steps Needed: {}", steps as i64);
                    self.move_by(steps as i64);
                    self.relay.set_high().unwrap_or_default();
                    self.run();
                    self.relay.set_low().unwrap_or_default();
                    self.update_position(90.0);
                    return true; 

                    
                }
            }
            true
            */
        }
    }
}

pub use motion::{Motion, MotionMode, MoveOutcome};
