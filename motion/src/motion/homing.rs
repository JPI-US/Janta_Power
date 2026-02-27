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
        // Stage 2: CCW-only homing (Spec A). Keep the CW API as a wrapper so existing
        // call sites remain safe.
        log::warn!("Homing requested CW, but firmware is configured for CCW-only homing; using CCW search");
        self.find_limit_switch_ccw()
    }

    pub fn find_limit_switch_ccw(&mut self) -> bool {
        use super::HOME_HEADING_DEG;
        
        // Set homing flag to disable overshoot protection during homing
        self.is_homing = true;

        // During homing, we want old-style behavior: keep searching until the switch is found
        // or we hit the travel budget. Stall detection can cause confusing abort cascades here,
        // so we disable it temporarily and restore it afterwards.
        let stall_prev = self.stall_detection_enabled();
        self.set_stall_detection_enabled(false);
        
        // If limit switch is already pressed, do nothing
        if self.lmsw.is_low() {
            log::info!("Limit switch already pressed - skipping homing");
            self.update_position(HOME_HEADING_DEG);
            self.force_zero_if_limit_switch_pressed();
            self.set_stall_detection_enabled(stall_prev);
            self.is_homing = false;  // Clear homing flag
            return true;
        }
        
        self.relay_on();
        // Convention for *current wiring*: negative step movement = physical CCW.
        log::info!("Looking for the limit switch (CCW search, max 350°)");

        let mut max_steps = calculate_steps(-350.0);
        while max_steps < 0 && self.lmsw.is_high() {
            let step_movement = calculate_steps(-1.0);
            if self.move_by(step_movement) != MoveOutcome::Completed {
                self.relay_off();
                self.set_stall_detection_enabled(stall_prev);
                self.is_homing = false;  // Clear homing flag on failure
                return false;
            }
            max_steps -= step_movement;
        }

        self.relay_off();

        if max_steps < 0 {
            self.update_position(HOME_HEADING_DEG);
            self.relay_off();
            self.force_zero_if_limit_switch_pressed();
            self.set_stall_detection_enabled(stall_prev);
            self.is_homing = false;  // Clear homing flag on success
            return true;
        }
        self.set_stall_detection_enabled(stall_prev);
        self.is_homing = false;  // Clear homing flag on failure
        false
    }
}

