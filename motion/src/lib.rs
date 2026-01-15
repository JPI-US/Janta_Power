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

    #[derive(PartialEq)]
    enum TrackingState {
        L1,
        L2,
        L3,
    }

    pub fn calculate_steps(offset: f32) -> i64 {
        return ((offset / 360.0) * (25600.0 * 50.0 * 84.0)) as i64;
    }

    pub struct Motion<'a> {
        location: f32,
        tracking_state: TrackingState,
        speed: f32,
        acceleration: u16,
        motor: Driver,
        motor_device:
            StepAndDirection<PinDriver<'a, Gpio15, Output>, PinDriver<'a, Gpio16, Output>>,
        motor_clock: OperatingSystemClock,
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
                tracking_state: TrackingState::L1,
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
            }
        }

        pub fn update_position(&mut self, location: f32) {
            self.location = location;
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

        // Stage 3: restore the software zero offset so adjusted ticks can be reconstructed after reboot.
        pub fn set_encoder_zero_offset(&mut self, zero_offset: i32) {
            self.encoder_zero_offset = zero_offset;
        }

        pub fn init(&mut self) {
            self.motor.set_max_speed(self.speed);
            self.motor.set_speed(self.speed);
            self.motor.set_acceleration(self.acceleration.into());
        }

        pub fn move_by(&mut self, location: i64) {
            self.motor.move_by(location);
            self.run();
        }

        pub fn move_by_ticks(&mut self, location: i64) {
            self.motor.move_by(location);
            self.run();
        }
        

        pub fn run(&mut self) {
            let mut t0 = Instant::now();
            loop {
                if self.motor.is_running() {
                    let _ = self.motor.poll(&mut self.motor_device, &self.motor_clock);
                    self.encoder.poll();

                    // Reset encoder count to 0 when the limit switch is pressed (edge-triggered + debounced).
                    //
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
                        self.encoder_zero_offset = self.encoder.position();
                        self.lmsw_zeroed_this_press = true;
                        log::info!("Limit switch pressed: encoder zeroed (offset={})", self.encoder_zero_offset);
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
        }

        pub fn flip_relay(&mut self) {
            self.relay.toggle().unwrap_or_default();
        }

        pub fn find_limit_switch_cw(&mut self) -> bool {
           
            if self.lmsw.is_low() {
                log::info!("Found Limit Switch, Heading : 90");
                self.update_position(90.0);
                return true;
            }

            log::info!("Move 15 Degress clockwise first");
            self.relay.set_high().unwrap_or_default();

            let correction_factor = 1.231;
            let steps = (15.0 / 360.0) * (25600.0 * 50.0 * 84.0);
            log::info!("Steps Needed: {}", steps);
            log::info!("Steps Needed: {}", steps as i64);
            self.move_by(steps as i64);
            self.run();    // Blocking 
            log::info!("Done moving 15 Degress clockwise");
            
            log::info!("Now, looking for the limit switch");

            let mut max_steps = calculate_steps(-360.0);
            while (max_steps < 0 && self.lmsw.is_high()) {
                let step_movement = calculate_steps(-1.0);
                self.move_by(step_movement);
                max_steps -= step_movement;
            }

            self.relay.set_low().unwrap_or_default();
            if max_steps < 0 {
                log::info!("Found Limit Switch, Heading : 90");
                self.update_position(90.0);
                self.relay.set_low().unwrap_or_default();
                return true;
            }
            log::error!("Limit Switch was not found!");
            return false;
        }


        pub fn find_limit_switch_ccw(&mut self) -> bool {
            
            if self.lmsw.is_low() {
                self.update_position(90.0);
                return true;
            }
            
            log::info!("Move 15 Degress clockwise first");
            self.relay.set_high().unwrap_or_default();

            let correction_factor = 1.231;
            let steps = (15.0 / -360.0) * (25600.0 * 50.0 * 84.0)
            log::info!("Steps Needed: {}", steps);
            log::info!("Steps Needed: {}", steps as i64);
            self.move_by(steps as i64);
            self.run();    // Blocking 
            log::info!("Done moving 15 Degress clockwise");
            log::info!("Now, looking for the limit switch");

            let mut max_steps = calculate_steps(360.0); // full CW
            while (max_steps > 0 && self.lmsw.is_high()) {
                let step_movement = calculate_steps(1.0); // Move 1 deg at a time
                self.move_by(step_movement);
                max_steps -= step_movement;
            }

            self.relay.set_low().unwrap_or_default();

            if max_steps > 0 {
                self.update_position(90.0);
                self.relay.set_low().unwrap_or_default();
                return true;
            }
            false
        }



        pub fn set_tower_position<I2C: embedded_hal::i2c::I2c, T: NvsPartitionId>(
            &mut self,
            clock: &mut Clock<I2C>,
            location: f32,
            balance: i32,
            mqtt: &mut Mqtt,
            current_version: Version,
            nvs: &mut EspNvs<T>,
            wifi: &mut Wifi<'_>,
            formatted_time: String,
        ) -> bool {
            self.update_position(location);
            log::info!("{},", clock.after_sunrise());
            if clock.after_sunrise() && !clock.after_sunset() {
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
                if angle_offset.abs() > 5.0 {
                    self.relay.set_high().unwrap_or_default();
                    self.tracking_state = TrackingState::L1;
                }
                if angle_offset.abs() <= 5.0 && self.tracking_state == TrackingState::L1 {
                    let _ = self.relay.set_low().unwrap_or_default();
                    return true; // New line
                    //self.tracking_state = TrackingState::L2;
                }
                match self.tracking_state {
                    TrackingState::L1 => {
                        let correction_factor = 1.3;
                        log::info!("Tracking state L1");
                        let steps = (angle_offset / 360.0) * (25600.0 * 50.0 * 84.0); // * correction_factor; // Change to -360 for waco 
                        log::info!("Steps Needed: {}", steps as i64);
                        self.move_by(steps as i64);
                        self.run();    // Blocking 
                        // log::info!("Angle Offset: {}", angle_offset);
                        self.update_position((location as f64 + angle_offset) as f32);
                        log::info!("Exiting Tracking state L1");
                        self.relay.set_low().unwrap_or_default(); // New line

                        //Publish message
                        let payload = format!(
                            "Current datetime: {}, and current tower angle: {}",
                            formatted_time, 
                            location as f64 + angle_offset
                        );
                        match mqtt.publish("device1A/data", payload.as_bytes()){
                            Ok(_) => log::info!("Published data payload successfully"),
                            Err(e) => log::error!("Failed to publish data payload: {:?}", e),
                        }
                        return false;
                    }
                    TrackingState::L2 => {
                        log::info!("Tracking state L2");
                        if angle_offset.abs() > 5.0 {
                            self.prev_balance = 0;
                            self.tracking_state = TrackingState::L1;
                            return false;
                        }
                        if (balance - self.prev_balance).abs() < 75 {
                            self.prev_balance = 0;
                            self.tracking_state = TrackingState::L1;
                            return true;
                        } else {
                            self.prev_balance = balance;
                        }
                        if balance <= -10 {
                            let steps = (-0.5 / 360.0) * (25600.0 * 50.0 * 84.0);
                            self.move_by(steps as i64);
                            self.run();
                            self.update_position(location - 0.5);
                            return false;
                        } else if balance >= 10 {
                            let steps = (0.5 / 360.0) * (25600.0 * 50.0 * 84.0);
                            self.move_by(steps as i64);
                            self.run();
                            self.update_position(location + 0.5);
                            return false;
                        } else {
                            self.prev_balance = 0;
                            self.tracking_state = TrackingState::L1;
                            return true;
                        }
                    }
                    TrackingState::L3 => (), // Future tracking 
                }
            } 
            else {// Sunset Operation 
                if location == 90.0 {
                    log::info!("Already reached sleep position");

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
                        true => log::info!("Limit switch has returned true"),
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
            true
            //note to self: maybe remove?
            /*else if clock.after_sunset() {
                 if self.tracking_state != TrackingState::L3 {
                    let angle_offset = 90.0 - location;
                    let steps = (angle_offset / 360.0) * (20000.0 * 50.0 * 84.0);
                    log::info!("Steps Needed: {}", steps);
                    log::info!("Steps Needed: {}", steps as i64);
                    self.move_by(steps as i64);
                    self.relay.set_high().unwrap_or_default();
                    self.run();
                    self.relay.set_low().unwrap_or_default();
                    self.update_position(90.0);
                    self.tracking_state = TrackingState::L3;
                    return true; 

                    
                }
            }
            true
            */
        }
    }
}

pub use motion::Motion;
