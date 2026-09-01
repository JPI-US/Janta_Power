//! Config settings pertaining to standstill power saving mode.

#[derive(Clone, Copy)]
/// Config settings pertaining to standstill power saving mode.
pub struct Standstill {
    /// Controls how quickly the current falls from run current to holding current.
    ///
    /// Each step takes `fall_time` milliseconds. Values are clamped to the
    /// 4-bit hardware range of 0 to 15.
    ///
    /// Corresponds to the CTRL12 `TSTSL_FALL` field.
    pub fall_time: u8,

    /// Controls the delay between the last STEP pulse and activation of
    /// standstill power saving mode.
    ///
    /// Each unit represents 16 ms. Values are clamped to the hardware range
    /// of 1 to 63.
    ///
    /// Corresponds to the CTRL13 `TSTSL_DLY` field.
    pub delay: u8,
}

impl Default for Standstill {
    fn default() -> Self {
        Self {
            fall_time: 0b_0100,
            delay: 0b_000100,
        }
    }
}
