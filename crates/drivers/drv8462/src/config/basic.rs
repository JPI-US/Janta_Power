//! Basic configuration settings that should almost always be explicitly set.

/// Basic configuration settings that should almost always be explicitly set.
///
/// The default values will probably not work for you out of the box!
///
/// # Fields
///
/// - `microstep_mode` (`MicrostepMode`) - Controls the microstepping configuration,
///   corresponding to the CTRL2 `MICROSTEP_MODE` field.
/// - `holding_current` (`u8`) - Determines the current when the motor is idle.
/// - `run_current` (`u8`) - Determines the current when the motor is running.
/// - `enable_internal_voltage_reference` (`bool`) - Enables the device's internal
///   3.3V reference for current regulation.
///
/// # Examples
///
/// ```
/// use crate::...;
///
/// let s = Basic {
///     microstep_mode: value,
///     holding_current: value,
///     run_current: value,
///     enable_internal_voltage_reference: value,
/// };
/// ```
#[derive(Copy, Clone)]
pub struct Basic {
    pub microstep_mode: MicrostepMode,

    /// Determines the current when the motor is idle.
    ///
    /// `255` = 256/256 x 100%
    ///
    /// `254` = 255/256 x 100%
    ///
    /// `253` = 254/256 x 100%
    ///
    /// `252` = 253/256 x 100%
    ///
    /// ...
    ///
    /// `0` = 1/256 x 100%
    ///
    /// Corresponds to the CTRL10 `ISTSL` field.
    pub holding_current: u8,

    /// Determines the current when the motor is running.
    ///
    /// `255` = 256/256 x 100%
    ///
    /// `254` = 255/256 x 100%
    ///
    /// `253` = 254/256 x 100%
    ///
    /// `252` = 253/256 x 100%
    ///
    /// ...
    ///
    /// `0` = 1/256 x 100%
    ///
    /// Corresponds to the CTRL11 `TRQ_DAC` field.
    pub run_current: u8,

    /// Enables the device's internal 3.3V reference for current regulation,
    /// ignoring the voltage on the VREF pin.
    ///
    /// Corresponds to the CTRL13 `VREF_INT_EN` field.
    pub enable_internal_voltage_reference: bool,
}

impl Default for Basic {
    fn default() -> Self {
        Self {
            microstep_mode: MicrostepMode::default(),
            holding_current: 0b_10000000,
            run_current: 0b_11111111,
            enable_internal_voltage_reference: bool::default(),
        }
    }
}

#[repr(u8)]
#[derive(Default, Copy, Clone)]
/// Microstep mode.
///
/// Controls the number of commanded positions per full motor step. Higher
/// resolutions generally provide smoother motion and lower vibration, at the
/// cost of a higher required step-pulse rate.
pub enum MicrostepMode {
    /// Full-step operation with 100% current on the active phase.
    ///
    /// Provides maximum available torque per commanded step.
    FullStep100 = 0b_0000,

    /// Full-step operation with approximately 71% phase current.
    ///
    /// Reduces current, power consumption, and heat at the cost of torque.
    FullStep71 = 0b_0001,

    /// 1/2 step operation using a non-circular current waveform.
    HalfStepNonCircular = 0b_0010,

    /// 1/2 step operation.
    ///
    /// Provides smoother motion than full-step operation while retaining
    /// relatively high torque.
    HalfStep = 0b_0011,

    /// 1/4 step operation.
    QuarterStep = 0b_0100,

    /// 1/8 step operation.
    EighthStep = 0b_0101,

    /// 1/16 step operation.
    ///
    /// A practical balance between smooth motion and step-pulse rate.
    #[default]
    SixteenthStep = 0b_0110,

    /// 1/32 step operation.
    ThirtySecondStep = 0b_0111,

    /// 1/64 step operation.
    SixtyFourthStep = 0b_1000,

    /// 1/128 step operation.
    OneTwentyEighthStep = 0b_1001,

    /// 1/256 step operation.
    ///
    /// Provides the finest available command resolution.
    TwoFiftySixthStep = 0b_1010,
}
