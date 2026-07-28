// Stepper movement execution and safety checks.

use std::time::{Duration, Instant};

use anyhow::{Context, Result};

use super::{
    Motion, MotionMode, MoveOutcome, ENCODER_STALL_CHECK_INTERVAL_STEPS, ENCODER_STALL_MIN_TICKS,
    INVERT_MOTOR_DIRECTION, MAX_STEPS_WITHOUT_ENC_CHANGE,
};

impl Motion<'_> {
    pub fn init(&mut self) {
        self.motor.set_max_speed(self.speed);
        self.motor.set_speed(self.speed);
        self.motor.set_acceleration(self.acceleration.into());
    }

    pub fn take_last_move_outcome(&mut self) -> Option<MoveOutcome> {
        self.last_move_outcome.take()
    }

    pub fn move_by(&mut self, location: i64) -> Result<MoveOutcome> {
        // Start move.
        self.relay_on();
        log::info!("Relay ON - Starting motor movement");

        // Reset stall baselines for this move.
        let now = Instant::now();
        self.stall_last_check = now;
        self.stall_step_pos_at_last_enc_change = self.motor.current_position();
        self.stall_last_enc_ticks_seen = self.encoder_ticks_adjusted();
        self.stall_reported = false;
        self.stall_consecutive = 0;

        // Initialize ratio-based stall state (EncoderGuarded only).
        if self.motion_mode == MotionMode::EncoderGuarded {
            self.stall_check_start_encoder_ticks = self.encoder_ticks_adjusted();
            self.stall_check_start_step_pos = self.motor.current_position();
            self.stall_check_last_interval_step = self.motor.current_position();
        }

        // Initialize overshoot state (EncoderGuarded only).
        if self.motion_mode == MotionMode::EncoderGuarded {
            use super::{ENC_TICKS_PER_REV, STEPS_PER_REV};
            self.overshoot_enc_start = Some(self.encoder_ticks_adjusted());
            let expected_ticks =
                ((location.abs() as f64 / STEPS_PER_REV) * ENC_TICKS_PER_REV as f64) as i64;
            self.overshoot_expected_ticks = Some(expected_ticks);
        } else {
            self.overshoot_enc_start = None;
            self.overshoot_expected_ticks = None;
        }

        let signed_steps = if INVERT_MOTOR_DIRECTION {
            -location
        } else {
            location
        };
        self.motor.move_by(signed_steps);
        let outcome = self.run().context("Failed to run motor")?;

        // End move.
        self.relay_off();
        log::info!("Relay OFF - Motor movement finished: {:?}", outcome);

        self.last_move_outcome = Some(outcome.clone());
        Ok(outcome)
    }

    pub fn move_by_ticks(&mut self, location: i64) -> Result<MoveOutcome> {
        // Start move in tick space.
        self.relay_on();
        log::info!("Relay ON - Starting motor movement (ticks)");

        let now = Instant::now();
        self.stall_last_check = now;
        self.stall_step_pos_at_last_enc_change = self.motor.current_position();
        self.stall_last_enc_ticks_seen = self.encoder_ticks_adjusted();
        self.stall_reported = false;
        self.stall_consecutive = 0;

        // Initialize ratio-based stall state (EncoderGuarded only).
        if self.motion_mode == MotionMode::EncoderGuarded {
            self.stall_check_start_encoder_ticks = self.encoder_ticks_adjusted();
            self.stall_check_start_step_pos = self.motor.current_position();
            self.stall_check_last_interval_step = self.motor.current_position();
        }

        // Initialize overshoot state (EncoderGuarded only).
        if self.motion_mode == MotionMode::EncoderGuarded {
            self.overshoot_enc_start = Some(self.encoder_ticks_adjusted());
            self.overshoot_expected_ticks = Some(location.abs());
        } else {
            self.overshoot_enc_start = None;
            self.overshoot_expected_ticks = None;
        }

        let signed_steps = if INVERT_MOTOR_DIRECTION {
            -location
        } else {
            location
        };
        self.motor.move_by(signed_steps);
        let outcome = self.run().context("Failed to run motor")?;

        // End move.
        self.relay_off();
        log::info!("Relay OFF - Motor movement finished (ticks): {:?}", outcome);

        self.last_move_outcome = Some(outcome.clone());
        Ok(outcome)
    }

    pub fn run(&mut self) -> Result<MoveOutcome> {
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
                    self.relay_off(); // Turn relay OFF on abort
                    return Ok(MoveOutcome::AbortedPowerMissing);
                }

                // Absolute stall detector: encoder must change within a step budget.
                if self.motion_mode == MotionMode::EncoderGuarded
                    && self.stall_detection_enabled
                    && self.stall_last_check.elapsed() >= Duration::from_millis(250)
                {
                    let step_pos = self.motor.current_position();
                    let enc_pos = self.encoder_ticks_adjusted();

                    if enc_pos != self.stall_last_enc_ticks_seen {
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

                    if stalled {
                        log::error!("MOVE_ABORT stall_confirmed=true: stopping motor immediately");
                        let pos = self.motor.current_position();
                        self.motor.set_current_position(pos); // hard stop
                        self.relay_off(); // Turn relay OFF on stall abort
                        return Ok(MoveOutcome::AbortedStall);
                    }

                    self.stall_last_check = Instant::now();
                }

                // Ratio stall detector: every interval must produce minimum tick movement.
                if self.motion_mode == MotionMode::EncoderGuarded && self.stall_detection_enabled {
                    let current_step_pos = self.motor.current_position();
                    let current_encoder_ticks = self.encoder_ticks_adjusted();

                    let total_steps_moved =
                        (current_step_pos - self.stall_check_start_step_pos).abs();

                    if total_steps_moved >= ENCODER_STALL_CHECK_INTERVAL_STEPS {
                        let encoder_ticks_moved =
                            (current_encoder_ticks - self.stall_check_start_encoder_ticks).abs();

                        if encoder_ticks_moved < ENCODER_STALL_MIN_TICKS {
                            log::error!(
                                "MOVE_ABORT ratio_stall_detected: total_steps={} encoder_ticks={} (minimum required={})",
                                total_steps_moved,
                                encoder_ticks_moved,
                                ENCODER_STALL_MIN_TICKS
                            );
                            let pos = self.motor.current_position();
                            self.motor.set_current_position(pos); // hard stop
                            self.relay_off(); // Turn relay OFF on stall abort
                            return Ok(MoveOutcome::AbortedStall);
                        }

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

                // Overshoot protection (EncoderGuarded only, disabled during homing).
                if self.motion_mode == MotionMode::EncoderGuarded
                    && !self.is_homing
                    && self.overshoot_enc_start.is_some()
                    && self.overshoot_expected_ticks.is_some()
                {
                    use super::ENCODER_OVERSHOOT_TOLERANCE_TICKS;
                    let enc_start = self
                        .overshoot_enc_start
                        .ok_or_else(|| anyhow::anyhow!("overshoot_enc_start not initialized"))?;
                    let enc_current = self.encoder_ticks_adjusted();
                    let enc_delta = (enc_current - enc_start).abs() as i64;
                    let expected = self.overshoot_expected_ticks.ok_or_else(|| {
                        anyhow::anyhow!("overshoot_expected_ticks not initialized")
                    })?;
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
                        self.motor.set_current_position(pos);
                        self.relay_off();
                        self.overshoot_enc_start = None;
                        self.overshoot_expected_ticks = None;
                        return Ok(MoveOutcome::AbortedOvershoot);
                    }
                }

                // Capture home error and zero encoder on switch events.
                self.poll_limit_switch_zeroing();

                // During homing, stop immediately once switch is pressed.
                if self.is_homing && self.lmsw.is_low() {
                    log::info!(
                        "Limit switch pressed during homing - found home, stopping immediately"
                    );
                    let pos = self.motor.current_position();
                    self.motor.set_current_position(pos);
                    self.relay_off();
                    return Ok(MoveOutcome::Completed);
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
        // Clear overshoot state on normal completion.
        self.overshoot_enc_start = None;
        self.overshoot_expected_ticks = None;
        Ok(MoveOutcome::Completed)
    }
}
