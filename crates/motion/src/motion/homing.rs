// Limit-switch and homing helpers.

use std::time::{Duration, Instant};

use super::Motion;

impl Motion<'_> {
    // Retrieve and clear the most recent home error.
    pub fn take_last_home_error_ticks(&mut self) -> Option<i32> {
        self.last_home_error_ticks.take()
    }

    // Ensure adjusted ticks are zeroed when we are physically on the switch.
    // IF YOU SEEING THIS NEEZ OR SALLA THEN PLEASE READ THIS VERY CAREFULLY
    // INVARIANT — DO NOT REORDER THE THREE NUMBERED STEPS BELOW.
    // The end-of-day encoder drift metric (published to
    // `tower/{id}/data/encoder_error_ticks`) depends on this exact sequence:
    //   1. Read `encoder_ticks_adjusted()` — this is the cumulative drift
    //      (daytime CW ticks minus sunset CCW ticks) relative to the previous
    //      zero reference. It is ONLY meaningful BEFORE we update the offset.
    //   2. Stash that value into `last_home_error_ticks`. The publish site
    //      later reads this stashed value via `take_last_home_error_ticks()`.
    //   3. Update `encoder_zero_offset` so future `encoder_ticks_adjusted()`
    //      reads start from zero.
    // If step 3 runs before step 1/2, the stashed value becomes ~0 and the
    // drift metric is silently destroyed. The same invariant applies to
    // `poll_limit_switch_zeroing` below.
    pub fn force_zero_if_limit_switch_pressed(&mut self) {
        if self.lmsw_active() {
            // 1. Read drift relative to the previous zero reference.
            let home_error = self.encoder_ticks_adjusted();
            // 2. Stash it for the publish site to consume later.
            self.last_home_error_ticks = Some(home_error);

            // 3. Re-zero: adjusted ticks now read 0 at the switch.
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
        let pressed = self.lmsw_active();
        let now = Instant::now();
        if pressed != self.lmsw_last_state_pressed {
            self.lmsw_last_state_pressed = pressed;
            self.lmsw_last_change = now;

            if !pressed {
                self.lmsw_zeroed_this_press = false;
            }
        }

        // Time-based debounce: pressed and stable for 30 ms.
        //
        // INVARIANT — see the ordering note on `force_zero_if_limit_switch_pressed`.
        // Steps 1 → 2 → 3 must stay in this order or the drift metric is lost.
        if pressed
            && !self.lmsw_zeroed_this_press
            && self.lmsw_last_change.elapsed() >= Duration::from_millis(30)
        {
            // 1. Read drift relative to the previous zero reference.
            let home_error = self.encoder_ticks_adjusted();
            // 2. Stash it for the publish site to consume later.
            self.last_home_error_ticks = Some(home_error);
            // 3. Re-zero: adjusted ticks now read 0 at the switch.
            self.encoder_zero_offset = self.encoder.position();
            self.lmsw_zeroed_this_press = true;
            log::info!(
                "Limit switch pressed: home_error_ticks={}, encoder zeroed (offset={})",
                home_error,
                self.encoder_zero_offset
            );
        }
    }
}
