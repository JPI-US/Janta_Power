// Stepper movement execution + stall detector.
//
// Keep behavior identical to the previous monolithic implementation in `motion/src/lib.rs`.

use super::{Motion, MotionMode, MoveOutcome, INVERT_MOTOR_DIRECTION};
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
        // Reset stall detector baseline for this move so we don't accidentally compare
        // against stale values from a previous run.
        let now = Instant::now();
        self.stall_last_check = now;
        self.stall_step_pos_at_last_enc_change = self.motor.current_position();
        self.stall_last_enc_ticks_seen = self.encoder_ticks_adjusted();
        self.stall_reported = false;
        self.stall_consecutive = 0;

        let signed_steps = if INVERT_MOTOR_DIRECTION { -location } else { location };
        self.motor.move_by(signed_steps);
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

        let signed_steps = if INVERT_MOTOR_DIRECTION { -location } else { location };
        self.motor.move_by(signed_steps);
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

                // Immediate abort when power is missing (EncoderGuarded only).
                if self.motion_mode == MotionMode::EncoderGuarded && !self.motor_power_on {
                    log::warn!("MOVE_ABORT power_missing=true: stopping motor immediately");
                    let pos = self.motor.current_position();
                    self.motor.set_current_position(pos); // hard stop
                    return MoveOutcome::AbortedPowerMissing;
                }

                // Stall detector.
                // Detect "motor stepping but encoder not moving" at a fixed cadence.
                if self.motion_mode == MotionMode::EncoderGuarded
                    && self.stall_detection_enabled
                    && self.stall_last_check.elapsed() >= Duration::from_millis(250)
                {
                    // Stall threshold (no gearbox):
                    // Previously this was tuned for a drivetrain where encoder ticks could take ~20k+ steps
                    // to change. With the 50:1 gearbox removed, encoder ticks should change *much sooner*,
                    // so we use a smaller budget to detect "motor stepping but encoder not moving".
                    //
                    // If you see false stalls due to slack/noise, increase to ~10k–20k.
                    const MAX_STEPS_WITHOUT_ENC_CHANGE: i64 = 20_000;

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
                        return MoveOutcome::AbortedStall;
                    }

                    self.stall_last_check = Instant::now();
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
        MoveOutcome::Completed
    }
}

