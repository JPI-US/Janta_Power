#![deny(unsafe_code)]
#![no_std]

pub use rtcc::{
    DateTimeAccess, Datelike, Hours, NaiveDate, NaiveDateTime, NaiveTime, Rtcc, Timelike,
};

/// All possible errors in this crate
#[derive(Debug)]
pub enum Error {
    /// I²C/SPI bus error
    Comm,
    /// Pin setting error
    Pin,
    /// Invalid input data provided
    InvalidInputData,
    /// Internal device state is invalid.
    ///
    /// It was not possible to read a valid date and/or time.
    /// The device is probably missing initialization.
    InvalidDeviceState,
}

/// Square-wave output frequency
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SqWFreq {
    /// 1 Hz (default)
    _1Hz,
    /// 1.024 Hz
    _1_024Hz,
    /// 4.096 Hz
    _4_096Hz,
    /// 8.192 Hz
    _8_192Hz,
}

/// Temperature conversion rate
///
/// This is only available on the DS3232 and DS3234 devices.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TempConvRate {
    /// Once every 64 seconds (default)
    _64s,
    /// Once every 128 seconds
    _128s,
    /// Once every 256 seconds
    _256s,
    /// Once every 512 seconds
    _512s,
}

struct Register;

impl Register {
    const SECONDS: u8 = 0x00;
    const MINUTES: u8 = 0x01;
    const HOURS: u8 = 0x02;
    const DOW: u8 = 0x03;
    const DOM: u8 = 0x04;
    const MONTH: u8 = 0x05;
    const YEAR: u8 = 0x06;
    const ALARM1_SECONDS: u8 = 0x07;
    const ALARM2_MINUTES: u8 = 0x0B;
    const CONTROL: u8 = 0x0E;
    const STATUS: u8 = 0x0F;
    const AGING_OFFSET: u8 = 0x10;
    const TEMP_MSB: u8 = 0x11;
    const TEMP_CONV: u8 = 0x13;
}

struct BitFlags;

impl BitFlags {
    const H24_H12: u8 = 0b0100_0000;
    const AM_PM: u8 = 0b0010_0000;
    const CENTURY: u8 = 0b1000_0000;
    const EOSC: u8 = 0b1000_0000;
    const BBSQW: u8 = 0b0100_0000;
    const TEMP_CONV: u8 = 0b0010_0000;
    const RS2: u8 = 0b0001_0000;
    const RS1: u8 = 0b0000_1000;
    const INTCN: u8 = 0b0000_0100;
    const ALARM2_INT_EN: u8 = 0b0000_0010;
    const ALARM1_INT_EN: u8 = 0b0000_0001;
    const OSC_STOP: u8 = 0b1000_0000;
    const BB32KHZ: u8 = 0b0100_0000;
    const CRATE1: u8 = 0b0010_0000;
    const CRATE0: u8 = 0b0001_0000;
    const EN32KHZ: u8 = 0b0000_1000;
    const BUSY: u8 = 0b0000_0100;
    const ALARM2F: u8 = 0b0000_0010;
    const ALARM1F: u8 = 0b0000_0001;
    const TEMP_CONV_BAT: u8 = 0b0000_0001;
    const ALARM_MATCH: u8 = 0b1000_0000;
    const WEEKDAY: u8 = 0b0100_0000;
}

const DEVICE_ADDRESS: u8 = 0b110_1000;
const CONTROL_POR_VALUE: u8 = 0b0001_1100;

/// IC markers
pub mod ic {
    /// DS3231 IC marker
    pub struct DS3231;
    /// DS3232 IC marker
    pub struct DS3232;
    /// DS3234 IC marker
    pub struct DS3234;
}

/// DS3231, DS3232 and DS3234 RTC driver
#[derive(Debug, Default)]
pub struct Ds323x<I2C> {
    i2c: I2C,
    control: u8,
    status: u8,
}

mod ds323x;
pub use crate::ds323x::{
    Alarm1Matching, Alarm2Matching, DayAlarm1, DayAlarm2, WeekdayAlarm1, WeekdayAlarm2,
};
mod ds3231;

mod private {
    use super::ic;
    pub trait Sealed {}

    impl Sealed for ic::DS3231 {}
    impl Sealed for ic::DS3232 {}
    impl Sealed for ic::DS3234 {}
}
