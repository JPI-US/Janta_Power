//! DRV8462 fault representation.

/// Fault conditions reported by the DRV8462.
///
/// Multiple fault conditions may be active simultaneously.
///
/// Use [`Drv8462::get_faults`] to read the currently reported fault conditions.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct Faults {
    /// Whether any fault is currently active.
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
