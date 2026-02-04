// Encoder-related helpers for Motion (Stage 1 extraction).
//
// Keep behavior identical to the previous monolithic implementation in `motion/src/lib.rs`.

use super::{Motion, MoveOutcome};

// Encoder calibration (output shaft): 348,323 ticks per full revolution.
const ENC_TICKS_PER_REV: f32 = 348_323.0;
const ENC_TICKS_PER_DEG: f32 = ENC_TICKS_PER_REV / 360.0;

impl Motion<'_> {
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
}

