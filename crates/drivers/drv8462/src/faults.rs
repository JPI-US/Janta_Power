//! DRV8462 fault representation.

/// Fault conditions reported by the DRV8462.
///
/// Multiple fault conditions may be active simultaneously.
///
/// Use [`Drv8462::get_faults`] to read the currently reported fault conditions.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct Faults {
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

impl core::fmt::Display for Faults {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "DRV8462 fault")?;

        if self.spi_error {
            write!(f, ": SPI error")?;
        }
        if self.undervoltage_lockout {
            write!(f, ": undervoltage lockout")?;
        }
        if self.charge_pump_undervoltage {
            write!(f, ": charge-pump undervoltage")?;
        }
        if self.over_current {
            write!(f, ": over-current")?;
        }
        if self.stall {
            write!(f, ": stall")?;
        }
        if self.temperature {
            write!(f, ": temperature")?;
        }
        if self.open_load {
            write!(f, ": open load")?;
        }

        Ok(())
    }
}

impl std::error::Error for Faults {}
