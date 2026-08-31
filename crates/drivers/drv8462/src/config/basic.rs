#[derive(Copy, Clone)]
pub struct Basic {
    /// Controls the microstepping configuration.
    ///
    /// Corresponds to the CTRL2 `MICROSTEP_MODE` field.
    pub microstep_mode: MicrostepMode,

    /// Determines the current when the motor is idle.
    ///
    /// 255 = 256/256 x 100%
    ///
    /// 254 = 255/256 x 100%
    ///
    /// 253 = 254/256 x 100%
    ///
    /// 252 = 253/256 x 100%
    ///
    /// ....................
    ///
    /// 0 = 1/256 x 100%
    ///
    /// Corresponds to the CTRL10 `ISTSL` field.
    pub holding_current: u8,

    /// Determines the current when the motor is running.
    ///
    /// 255 = 256/256 x 100%
    ///
    /// 254 = 255/256 x 100%
    ///
    /// 253 = 254/256 x 100%
    ///
    /// 252 = 253/256 x 100%
    ///
    /// ....................
    ///
    /// 0 = 1/256 x 100%
    ///
    /// Corresponds to the CTRL11 `TRQ_DAC` field.
    pub run_current: u8,

    /// Controls whether the device uses its internal 3.3V reference for current regulation
    /// and ignores the voltage on the VREF pin.
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
pub enum MicrostepMode {
    /// Full-step operation with 100% current on the active phase.
    ///
    /// Provides maximum available torque per commanded step and the most
    /// direct relationship between commanded steps and rotor position.
    /// Useful when maximum torque is more important than smooth motion or
    /// reduced vibration.
    FullStep100 = 0b_0000,

    /// Full-step operation with approximately 71% phase current.
    ///
    /// Reduces motor current, power consumption, and heat compared with
    /// 100% full-step operation. Useful when the application can tolerate
    /// reduced torque and wants lower power or quieter operation.
    FullStep71 = 0b_0001,

    /// 1/2 step operation using a non-circular current waveform.
    ///
    /// Provides twice the positional resolution of full-step operation while
    /// maintaining a relatively simple drive pattern. The non-circular
    /// waveform prioritizes the intended current levels rather than keeping
    /// the motor current magnitude constant.
    HalfStepNonCircular = 0b_0010,

    /// 1/2 step operation.
    ///
    /// Doubles the number of commanded positions per motor revolution
    /// compared with full-step operation, reducing vibration and improving
    /// motion smoothness while retaining relatively high torque.
    HalfStep = 0b_0011,

    /// 1/4 step operation.
    ///
    /// Provides four commanded positions per full motor step, giving smoother
    /// motion and reducing vibration compared with full-step and half-step
    /// operation. A useful compromise between motion smoothness and the
    /// number of step pulses required by the controller.
    QuarterStep = 0b_0100,

    /// 1/8 step operation.
    ///
    /// Provides finer positional control and smoother rotation than
    /// quarter-step operation. Useful for reducing vibration and audible
    /// noise while avoiding the higher step-pulse rate of finer microstepping.
    EighthStep = 0b_0101,

    /// 1/16 step operation.
    ///
    /// Provides a good balance between smooth motion, reduced vibration, and
    /// controller step rate. Often a practical default when finer motion is
    /// desired without requiring very high step frequencies.
    #[default]
    SixteenthStep = 0b_0110,

    /// 1/32 step operation.
    ///
    /// Produces very fine commanded motion with further reductions in
    /// mechanical vibration and step-induced noise. Useful when smooth
    /// movement is more important than maximizing the available full-step
    /// torque per commanded position.
    ThirtySecondStep = 0b_0111,

    /// 1/64 step operation.
    ///
    /// Provides very fine microstepping for applications that benefit from
    /// exceptionally smooth commanded motion and low vibration. Requires a
    /// correspondingly higher step-pulse rate from the controller.
    SixtyFourthStep = 0b_1000,

    /// 1/128 step operation.
    ///
    /// Provides extremely fine command resolution and very smooth motion.
    /// Useful when high-resolution motion commands are beneficial, provided
    /// the controller can generate the required step rate.
    OneTwentyEighthStep = 0b_1001,

    /// 1/256 step operation.
    ///
    /// Provides the finest available microstep resolution. Primarily useful
    /// for maximizing commanded position resolution and minimizing
    /// step-induced vibration. It requires a very high step-pulse rate and
    /// does not provide the same practical increase in absolute positioning
    /// accuracy as the increase in command resolution might suggest.
    TwoFiftySixthStep = 0b_1010,
}
