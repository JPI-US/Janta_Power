// Encoder helpers for Motion.

use super::{
    Motion, MoveOutcome, ENCODER_PROBE_MIN_TICKS, ENCODER_PROBE_STEPS,
    ENCODER_STALL_CHECK_INTERVAL_STEPS, ENCODER_STALL_MIN_TICKS, ENC_TICKS_PER_REV,
};

// Output-shaft encoder calibration from build-time constants.
pub(super) const ENC_TICKS_PER_DEG: f32 = ENC_TICKS_PER_REV / 360.0;

impl Motion<'_> {
    // CW is positive; 0 ticks is limit-switch home after zeroing.
    pub fn encoder_ticks_adjusted(&self) -> i32 {
        self.encoder.position() - self.encoder_zero_offset
    }

    // Raw quadrature ticks.
    pub fn encoder_ticks_raw(&self) -> i32 {
        self.encoder.position()
    }

    // Restore software zero offset after reboot.
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

    /// Convert a degrees delta into expected encoder ticks (output shaft).
    /// Positive degrees correspond to positive encoder ticks (CW).
    pub fn encoder_ticks_for_deg(&self, deg: f32) -> i32 {
        (deg * ENC_TICKS_PER_DEG).round() as i32
    }

    /// Diagnostic probe: move and verify encoder ticks changed.
    pub fn probe_encoder_motion(&mut self, probe_steps: i64) -> bool {
        let start_ticks = self.encoder_ticks_adjusted();
        let outcome = self.move_by(probe_steps);

        if outcome != MoveOutcome::Completed {
            log::warn!("Encoder probe aborted: {:?}", outcome);
            return false;
        }

        let end_ticks = self.encoder_ticks_adjusted();
        let encoder_ticks_moved = (end_ticks - start_ticks).abs();

        // Probe threshold is intentionally looser than runtime stall checks.
        let min_expected_ticks = if probe_steps.abs() == ENCODER_PROBE_STEPS {
            ENCODER_PROBE_MIN_TICKS
        } else {
            ((probe_steps.abs() as f64 / ENCODER_STALL_CHECK_INTERVAL_STEPS as f64)
                * ENCODER_STALL_MIN_TICKS as f64)
                .ceil() as i32
        };

        let moved = encoder_ticks_moved >= min_expected_ticks;
        log::info!(
            "Encoder probe complete: start_ticks={} end_ticks={} ticks_moved={} min_expected={} passed={}",
            start_ticks,
            end_ticks,
            encoder_ticks_moved,
            min_expected_ticks,
            moved
        );
        moved
    }
}
