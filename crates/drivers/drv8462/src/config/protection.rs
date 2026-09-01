//! Config settings pertaining to built-in fault protection.

#[derive(Default, Copy, Clone)]
/// Config settings pertaining to built-in fault protection.
pub struct Protection {
    /// Sets the time the driver waits to confirm an overcurrent condition.
    ///
    /// Corresponds to the CTRL3 `TOCP` field.
    pub overcurrent_deglitch_time: TOcp,

    /// Controls whether an overcurrent condition latches a fault or auto-retries.
    ///
    /// Corresponds to the CTRL3 `OCP_MODE` field.
    pub overcurrent_auto_retry: bool,

    /// Controls whether an overtemperature condition latches a fault or auto-retries.
    ///
    /// Corresponds to the CTRL3 `OTSD_MODE` field.
    pub overtemperature_auto_retry: bool,

    /// Controls whether temperature warnings are reported on `nFAULT`.
    ///
    /// Corresponds to the CTRL3 `TW_REP` field.
    pub report_temperature_warning: bool,
}

#[repr(u8)]
#[derive(Default, Copy, Clone)]
/// Overcurrent protection deglitch time.
pub enum TOcp {
    /// 2.2 μs.
    #[default]
    Us2_2 = 0b_1,

    /// 1.2 μs.
    Us1_2 = 0b_0,
}
