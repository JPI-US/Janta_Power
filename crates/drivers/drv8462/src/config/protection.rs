#[derive(Default, Copy, Clone)]
pub struct Protection {
    /// Sets the time the driver waits to confirm an overcurrent condition before triggering overcurrent protection.
    ///
    /// Corresponds to the CTRL3 `TOCP` field.
    pub overcurrent_protection_deglitch_time: TOcp,

    /// Controls whether reaching an overcurrent condition raises a latched fault or auto-retries.
    ///
    /// Corresponds to the CTRL3 `OCP_MODE` field.
    pub overcurrent_condition_auto_retry: bool,

    /// Controls whether reaching an overtemperature condition raises a latched fault or auto-retries.
    ///
    /// Corresponds to the CTRL3 `OTSD_MODE` field.
    pub overtemperature_condition_auto_retry: bool,

    /// Controls whether overtemperature or undertemperature warnings are reported on `nFAULT`.
    ///
    /// Corresponds to the CTRL3 `TW_REP` field.
    pub report_temperature_warning: bool,
}

#[repr(u8)]
#[derive(Default, Copy, Clone)]
/// Overcurrent protection time.
pub enum TOcp {
    #[default]
    /// 2.2 μs.
    Us2_2 = 0b_1,
    /// 1.2 μs.
    Us1_2 = 0b_0,
}
