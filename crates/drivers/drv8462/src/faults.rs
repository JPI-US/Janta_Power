/// Fault conditions reported by the DRV8462.
///
/// Each field corresponds to a fault condition represented by a bit in the
/// DRV8462 fault register. Multiple fault conditions may be active
/// simultaneously.
///
/// Use [`Drv8462::get_faults`] to read and inspect all currently reported
/// fault conditions.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct Faults {
    /// `true` fault is active, `false` otherwise.
    pub fault_active: bool,

    /// SPI communication error.
    pub spi_error: bool,

    /// Undervoltage lockout.
    pub undervoltage_lockout: bool,

    /// Charge-pump undervoltage.
    pub charge_pump_undervoltage: bool,

    /// Over-current protection fault.
    pub over_current: bool,

    /// Motor stall detected.
    pub stall: bool,

    /// Temperature fault.
    pub temperature: bool,

    /// Open-load condition detected.
    pub open_load: bool,
}
