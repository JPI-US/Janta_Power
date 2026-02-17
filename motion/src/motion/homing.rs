// Limit-switch and homing related helpers for Motion.
//
// Keep behavior identical to the previous monolithic implementation in `motion/src/lib.rs`.

use super::{calculate_steps, Motion, MoveOutcome};
use std::time::{Duration, Instant};

impl Motion<'_> {
    pub fn switch_pressed(&mut self) -> bool {
        self.lmsw.is_low()
    }

    // Retrieve and clear the most recent home error (captured when we hit the limit switch).
    pub fn take_last_home_error_ticks(&mut self) -> Option<i32> {
        self.last_home_error_ticks.take()
    }

    // `set_tower_position()` may call this when the motor isn't running yet; we still want
    // adjusted ticks to be 0 at home.
    pub(crate) fn force_zero_if_limit_switch_pressed(&mut self) {
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

    // Called from `run()` while the motor is moving: edge detection + debounce + capture home error + zero.
    pub(crate) fn poll_limit_switch_zeroing(&mut self) {
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
    }

    pub fn find_limit_switch_cw(&mut self) -> bool {
        use super::HOME_HEADING_DEG;
        
        // Set homing flag to disable overshoot protection during homing
        self.is_homing = true;
        
        // If limit switch is already pressed, do nothing (skip pre-move and homing)
        if self.lmsw.is_low() {
            log::info!("Limit switch already pressed - skipping pre-move and homing");
            log::info!("Found Limit Switch, Heading : {}", HOME_HEADING_DEG);
            self.update_position(HOME_HEADING_DEG);
            self.force_zero_if_limit_switch_pressed();
            self.is_homing = false;  // Clear homing flag
            return true;
        }
        
        // Pre-homing move: move 15 degrees CW before searching for limit switch
        log::info!("Pre-homing: moving 15° CW before limit switch search");
        let pre_move_steps = calculate_steps(15.0);
        let pre_move_outcome = self.move_by(pre_move_steps);
        if pre_move_outcome != MoveOutcome::Completed {
            // In EncoderGuarded mode, stall/overshoot means encoder failure - abort homing
            if self.motion_mode == super::MotionMode::EncoderGuarded
                && (pre_move_outcome == super::MoveOutcome::AbortedStall
                    || pre_move_outcome == super::MoveOutcome::AbortedOvershoot)
            {
                log::error!("Pre-homing move stalled/overshot in EncoderGuarded mode - aborting homing");
                self.is_homing = false;  // Clear homing flag
                return false;  // Abort homing, trigger encoder fault recovery
            }
            // In StepperOnly mode, continue with homing (legacy behavior)
            log::warn!("Pre-homing move failed: {:?}, continuing with homing search anyway", pre_move_outcome);
        } else {
            log::info!("Pre-homing move completed successfully");
        }
        
        self.relay.set_high().unwrap_or_default();
        // Convention for *current wiring*: positive step movement = physical CW.
        log::info!("Looking for the limit switch (CW search)");

        let mut max_steps = calculate_steps(360.0);
        while max_steps > 0 && self.lmsw.is_high() {
            let step_movement = calculate_steps(1.0);
            if self.move_by(step_movement) != MoveOutcome::Completed {
                self.relay.set_low().unwrap_or_default();
                self.is_homing = false;  // Clear homing flag on failure
                return false;
            }
            max_steps -= step_movement;
        }

        self.relay.set_low().unwrap_or_default();
        if max_steps > 0 {
            log::info!("Found Limit Switch, Heading : {}", HOME_HEADING_DEG);
            self.update_position(HOME_HEADING_DEG);
            self.relay.set_low().unwrap_or_default();
            self.force_zero_if_limit_switch_pressed();
            self.is_homing = false;  // Clear homing flag on success
            return true;
        }
        log::error!("Limit Switch was not found!");
        self.is_homing = false;  // Clear homing flag on failure
        return false;
    }

    pub fn find_limit_switch_ccw(&mut self) -> bool {
        use super::HOME_HEADING_DEG;
        
        // Set homing flag to disable overshoot protection during homing
        self.is_homing = true;
        
        // If limit switch is already pressed, do nothing (skip pre-move and homing)
        if self.lmsw.is_low() {
            log::info!("Limit switch already pressed - skipping pre-move and homing");
            self.update_position(HOME_HEADING_DEG);
            self.force_zero_if_limit_switch_pressed();
            self.is_homing = false;  // Clear homing flag
            return true;
        }
        
        // Pre-homing move: move 15 degrees CW before searching for limit switch
        log::info!("Pre-homing: moving 15° CW before limit switch search");
        let pre_move_steps = calculate_steps(15.0);
        let pre_move_outcome = self.move_by(pre_move_steps);
        if pre_move_outcome != MoveOutcome::Completed {
            log::warn!("Pre-homing move failed: {:?}, continuing with homing search anyway", pre_move_outcome);
        } else {
            log::info!("Pre-homing move completed successfully");
        }
        
        self.relay.set_high().unwrap_or_default();
        // Convention for *current wiring*: negative step movement = physical CCW.
        log::info!("Looking for the limit switch (CCW search)");

        let mut max_steps = calculate_steps(-360.0);
        while max_steps < 0 && self.lmsw.is_high() {
            let step_movement = calculate_steps(-1.0);
            if self.move_by(step_movement) != MoveOutcome::Completed {
                self.relay.set_low().unwrap_or_default();
                self.is_homing = false;  // Clear homing flag on failure
                return false;
            }
            max_steps -= step_movement;
        }

        self.relay.set_low().unwrap_or_default();

        if max_steps < 0 {
            self.update_position(HOME_HEADING_DEG);
            self.relay.set_low().unwrap_or_default();
            self.force_zero_if_limit_switch_pressed();
            self.is_homing = false;  // Clear homing flag on success
            return true;
        }
        self.is_homing = false;  // Clear homing flag on failure
        false
    }
}

