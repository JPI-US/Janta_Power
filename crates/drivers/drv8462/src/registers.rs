/// DRV8462 SPI register addresses.
#[repr(u8)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum Register {
    /// Fault status register.
    Fault = 0x00,

    /// Control register 1.
    Ctrl1 = 0x04,

    /// Control register 2.
    Ctrl2 = 0x05,

    /// Control register 3.
    Ctrl3 = 0x06,

    /// Control register 4.
    Ctrl4 = 0x07,

    /// Control register 6.
    Ctrl6 = 0x09,

    /// Control register 9.
    Ctrl9 = 0x0C,

    /// Control register 10.
    Ctrl10 = 0x0D,

    /// Control register 11.
    Ctrl11 = 0x0E,

    /// Control register 12.
    Ctrl12 = 0x0F,

    /// Control register 13.
    Ctrl13 = 0x10,
}
