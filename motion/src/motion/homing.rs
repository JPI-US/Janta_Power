// Limit-switch and homing helpers.

use super::{calculate_steps, Motion, MoveOutcome};
use std::time::{Duration, Instant};

impl Motion<'_> {
    pub fn switch_pressed(&mut self) -> bool {
        self.lmsw.is_low()
    }

    // Retrieve and clear the most recent home error.
    pub fn take_last_home_error_ticks(&mut self) -> Option<i32> {
        self.last_home_error_ticks.take()
    }

    // Ensure adjusted ticks are zeroed when we are physically on the switch.
    pub(crate) fn force_zero_if_limit_switch_pressed(&mut self) {
        if self.lmsw.is_low() {
            // Capture drift before re-zeroing.
            let home_error = self.encoder_ticks_adjusted();
            self.last_home_error_ticks = Some(home_error);

            // Make adjusted ticks 0 at the switch.
            self.encoder_zero_offset = self.encoder.position();

            // Keep debounce state consistent with a pressed/zeroed switch.
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

    // Called from `run()` while moving: edge-detect, debounce, and zero on press.
    pub(crate) fn poll_limit_switch_zeroing(&mut self) {
        // Switch is active-low.
        let pressed = self.lmsw.is_low();
        let now = Instant::now();
        if pressed != self.lmsw_last_state_pressed {
            self.lmsw_last_state_pressed = pressed;
            self.lmsw_last_change = now;

            if !pressed {
                self.lmsw_zeroed_this_press = false;
            }
        }

        // Time-based debounce: pressed and stable for 30 ms.
        if pressed
            && !self.lmsw_zeroed_this_press
            && self.lmsw_last_change.elapsed() >= Duration::from_millis(30)
        {
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
        // Firmware is currently CCW-only for homing.
        log::warn!("Homing requested CW, but firmware is configured for CCW-only homing; using CCW search");
        self.find_limit_switch_ccw()
    }

    pub fn find_limit_switch_ccw(&mut self) -> bool {
        use super::HOME_HEADING_DEG;
        
        // Disable overshoot checks while homing.
        self.is_homing = true;

        // Keep searching until switch is found or travel budget is exhausted.
        // Stall detection is temporarily disabled to avoid abort cascades.
        let stall_prev = self.stall_detection_enabled();
        self.set_stall_detection_enabled(false);

        if self.lmsw.is_low() {
            log::info!("Limit switch already pressed - skipping homing");
            self.update_position(HOME_HEADING_DEG);
            self.force_zero_if_limit_switch_pressed();
            self.set_stall_detection_enabled(stall_prev);
            self.is_homing = false;
            return true;
        }

        self.relay_on();
        // Current wiring convention: negative step command moves CCW.
        log::info!("Looking for the limit switch (CCW search, max 350°)");

        let mut max_steps = calculate_steps(-350.0);
        while max_steps < 0 && self.lmsw.is_high() {
            let step_movement = calculate_steps(-1.0);
            if self.move_by(step_movement) != MoveOutcome::Completed {
                self.relay_off();
                self.set_stall_detection_enabled(stall_prev);
                self.is_homing = false;
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
            self.is_homing = false;
            return true;
        }
        self.set_stall_detection_enabled(stall_prev);
        self.is_homing = false;
        false
    }
}

