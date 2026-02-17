// Stepper movement execution + stall detector.
//
// Keep behavior identical to the previous monolithic implementation in `motion/src/lib.rs`.

use super::{Motion, MotionMode, MoveOutcome, INVERT_MOTOR_DIRECTION, MAX_STEPS_WITHOUT_ENC_CHANGE, ENCODER_STALL_MIN_TICKS, ENCODER_STALL_CHECK_INTERVAL_STEPS};
use std::time::{Duration, Instant};

impl Motion<'_> {
    pub fn init(&mut self) {
        self.motor.set_max_speed(self.speed);
        self.motor.set_speed(self.speed);
        self.motor
            .set_acceleration(self.acceleration.into());
    }

    pub fn take_last_move_outcome(&mut self) -> Option<MoveOutcome> {
        self.last_move_outcome.take()
    }

    pub fn move_by(&mut self, location: i64) -> MoveOutcome {
        // Turn relay ON before starting motor movement
        let _ = self.relay.set_high();
        log::info!("Relay ON - Starting motor movement");

        // Reset stall detector baseline for this move so we don't accidentally compare
        // against stale values from a previous run.
        let now = Instant::now();
        self.stall_last_check = now;
        self.stall_step_pos_at_last_enc_change = self.motor.current_position();
        self.stall_last_enc_ticks_seen = self.encoder_ticks_adjusted();
        self.stall_reported = false;
        self.stall_consecutive = 0;
        
        // Initialize ratio-based stall detection (EncoderGuarded mode only).
        if self.motion_mode == MotionMode::EncoderGuarded {
            self.stall_check_start_encoder_ticks = self.encoder_ticks_adjusted();
            self.stall_check_start_step_pos = self.motor.current_position();
            self.stall_check_last_interval_step = self.motor.current_position();
        }

        // Initialize overshoot protection (EncoderGuarded mode only).
        if self.motion_mode == MotionMode::EncoderGuarded {
            use super::{ENC_TICKS_PER_REV, STEPS_PER_REV};
            self.overshoot_enc_start = Some(self.encoder_ticks_adjusted());
            // Calculate expected encoder ticks for this move: (steps / STEPS_PER_REV) * ENC_TICKS_PER_REV
            let expected_ticks = ((location.abs() as f64 / STEPS_PER_REV) * ENC_TICKS_PER_REV as f64) as i64;
            self.overshoot_expected_ticks = Some(expected_ticks);
        } else {
            self.overshoot_enc_start = None;
            self.overshoot_expected_ticks = None;
        }

        let signed_steps = if INVERT_MOTOR_DIRECTION { -location } else { location };
        self.motor.move_by(signed_steps);
        let outcome = self.run();
        
        // Turn relay OFF after movement completes or aborts
        let _ = self.relay.set_low();
        log::info!("Relay OFF - Motor movement finished: {:?}", outcome);
        
        self.last_move_outcome = Some(outcome);
        outcome
    }

    pub fn move_by_ticks(&mut self, location: i64) -> MoveOutcome {
        // Turn relay ON before starting motor movement
        let _ = self.relay.set_high();
        log::info!("Relay ON - Starting motor movement (ticks)");

        let now = Instant::now();
        self.stall_last_check = now;
        self.stall_step_pos_at_last_enc_change = self.motor.current_position();
        self.stall_last_enc_ticks_seen = self.encoder_ticks_adjusted();
        self.stall_reported = false;
        self.stall_consecutive = 0;
        
        // Initialize ratio-based stall detection (EncoderGuarded mode only).
        if self.motion_mode == MotionMode::EncoderGuarded {
            self.stall_check_start_encoder_ticks = self.encoder_ticks_adjusted();
            self.stall_check_start_step_pos = self.motor.current_position();
            self.stall_check_last_interval_step = self.motor.current_position();
        }

        // Initialize overshoot protection (EncoderGuarded mode only).
        // For move_by_ticks, we use the location directly as expected ticks.
        if self.motion_mode == MotionMode::EncoderGuarded {
            self.overshoot_enc_start = Some(self.encoder_ticks_adjusted());
            self.overshoot_expected_ticks = Some(location.abs());
        } else {
            self.overshoot_enc_start = None;
            self.overshoot_expected_ticks = None;
        }

        let signed_steps = if INVERT_MOTOR_DIRECTION { -location } else { location };
        self.motor.move_by(signed_steps);
        let outcome = self.run();
        
        // Turn relay OFF after movement completes or aborts
        let _ = self.relay.set_low();
        log::info!("Relay OFF - Motor movement finished (ticks): {:?}", outcome);
        
        self.last_move_outcome = Some(outcome);
        outcome
    }

    pub fn run(&mut self) -> MoveOutcome {
        let mut t0 = Instant::now();
        loop {
            if self.motor.is_running() {
                let _ = self.motor.poll(&mut self.motor_device, &self.motor_clock);
                let _ = self.encoder.poll();

                // Immediate abort when power is missing (EncoderGuarded only).
                if self.motion_mode == MotionMode::EncoderGuarded && !self.motor_power_on {
                    log::warn!("MOVE_ABORT power_missing=true: stopping motor immediately");
                    let pos = self.motor.current_position();
                    self.motor.set_current_position(pos); // hard stop
                    let _ = self.relay.set_low(); // Turn relay OFF on abort
                    return MoveOutcome::AbortedPowerMissing;
                }

                // Stall detector.
                // Detect "motor stepping but encoder not moving" at a fixed cadence.
                if self.motion_mode == MotionMode::EncoderGuarded
                    && self.stall_detection_enabled
                    && self.stall_last_check.elapsed() >= Duration::from_millis(250)
                {
                    // Stall threshold: loaded from .env file at compile time.
                    // Previously this was tuned for a drivetrain where encoder ticks could take ~20k+ steps
                    // to change. With the 50:1 gearbox removed, encoder ticks should change *much sooner*,
                    // so we use a smaller budget to detect "motor stepping but encoder not moving".
                    //
                    // If you see false stalls due to slack/noise, increase the value in .env file.

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

                    // Abort the move once we've exceeded the allowed step budget without
                    // any encoder tick change.
                    if stalled {
                        log::error!("MOVE_ABORT stall_confirmed=true: stopping motor immediately");
                        let pos = self.motor.current_position();
                        self.motor.set_current_position(pos); // hard stop
                        let _ = self.relay.set_low(); // Turn relay OFF on stall abort
                        return MoveOutcome::AbortedStall;
                    }

                    self.stall_last_check = Instant::now();
                }

                // Ratio-based stall detection (EncoderGuarded mode only).
                // Check if encoder has moved at least MIN_TICKS over every INTERVAL_STEPS.
                if self.motion_mode == MotionMode::EncoderGuarded
                    && self.stall_detection_enabled
                {
                    let current_step_pos = self.motor.current_position();
                    let current_encoder_ticks = self.encoder_ticks_adjusted();
                    
                    let total_steps_moved = (current_step_pos - self.stall_check_start_step_pos).abs();
                    
                    // Check every INTERVAL_STEPS
                    if total_steps_moved >= ENCODER_STALL_CHECK_INTERVAL_STEPS {
                        let encoder_ticks_moved = (current_encoder_ticks - self.stall_check_start_encoder_ticks).abs();
                        
                        if encoder_ticks_moved < ENCODER_STALL_MIN_TICKS {
                            log::error!(
                                "MOVE_ABORT ratio_stall_detected: total_steps={} encoder_ticks={} (minimum required={})",
                                total_steps_moved,
                                encoder_ticks_moved,
                                ENCODER_STALL_MIN_TICKS
                            );
                            let pos = self.motor.current_position();
                            self.motor.set_current_position(pos); // hard stop
                            let _ = self.relay.set_low(); // Turn relay OFF on stall abort
                            return MoveOutcome::AbortedStall;
                        }
                        
                        // Reset for next interval
                        self.stall_check_start_encoder_ticks = current_encoder_ticks;
                        self.stall_check_start_step_pos = current_step_pos;
                        self.stall_check_last_interval_step = current_step_pos;
                        log::debug!(
                            "Ratio-based stall check passed: {} ticks over {} steps, resetting for next interval",
                            encoder_ticks_moved,
                            total_steps_moved
                        );
                    }
                }

                // Encoder overshoot protection (EncoderGuarded mode only).
                // Skip overshoot protection during homing (we don't know expected ticks).
                // Check if encoder moved more than expected + tolerance.
                if self.motion_mode == MotionMode::EncoderGuarded
                    && !self.is_homing  // Skip overshoot protection during homing
                    && self.overshoot_enc_start.is_some()
                    && self.overshoot_expected_ticks.is_some()
                {
                    use super::ENCODER_OVERSHOOT_TOLERANCE_TICKS;
                    let enc_start = self.overshoot_enc_start.unwrap();
                    let enc_current = self.encoder_ticks_adjusted();
                    let enc_delta = (enc_current - enc_start).abs() as i64;
                    let expected = self.overshoot_expected_ticks.unwrap();
                    let tolerance = ENCODER_OVERSHOOT_TOLERANCE_TICKS;

                    if enc_delta > expected + tolerance {
                        log::error!(
                            "MOVE_ABORT overshoot_detected: enc_delta={} expected={} tolerance={} (exceeded by {})",
                            enc_delta,
                            expected,
                            tolerance,
                            enc_delta - expected - tolerance
                        );
                        let pos = self.motor.current_position();
                        self.motor.set_current_position(pos); // hard stop
                        let _ = self.relay.set_low(); // Turn relay OFF on overshoot abort
                        // Clear overshoot tracking
                        self.overshoot_enc_start = None;
                        self.overshoot_expected_ticks = None;
                        return MoveOutcome::AbortedOvershoot;
                    }
                }

                // Home-error capture + zeroing on limit switch.
                self.poll_limit_switch_zeroing();

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
        // Clear overshoot tracking on successful completion
        self.overshoot_enc_start = None;
        self.overshoot_expected_ticks = None;
        MoveOutcome::Completed
    }
}

