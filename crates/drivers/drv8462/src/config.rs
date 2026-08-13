#[derive(Copy, Clone)]
/// Abstraction over the raw DRv8462 config registers. Each struct field maps to a register field on the chip.
/// For more information on the specifics of each field, see Page 74 of the [datasheet](https://www.ti.com/lit/ds/symlink/drv8452.pdf).
pub struct Drv8462Config {
    /// Controls the how quickly the output transitions between high and low, resulting in faster or slower edges.
    ///
    /// Corresponds to the CTRL1 `SR` field.
    pub output_rise_fall_time: Sr,

    /// Sets the fixed off-time used by the motor current regulation. This determines how long the output remains off between cycles.
    ///
    /// Corresponds to the CTRL1 `T_OFF` field.
    pub time_off: TOff,

    /// Controls how motor winding current decays during current regulation.
    ///
    /// Corresponds to the CTRL1 `DECAY` field.
    pub decay: Decay,

    /// Controls the microstepping configuration.
    ///
    /// Corresponds to the CTRL2 `MICROSTEP_MODE` field.
    pub microstep_mode: MicrostepMode,

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

    /// Controls the current sense blanking time.
    ///
    /// Corresponds to the CTRL4 `TBLANK_TIME` field.
    pub current_sense_blanking_time: TblankTime,

    // /// Enables/disables stall detection.
    // ///
    // /// Corresponds to the CTRL4 `STL_LRN` field.
    // pub enable_stall_detection: bool,

    // /// Controls whether stall detection os reported on `nFAULT`.
    // ///
    // /// Corresponds to the CTRL4 `STL_REP` field.
    // pub report_stall_detection: bool,
    /// Enables or disables STEP input filtering as per [`Self::step_frequency_tolerance`].
    ///
    /// Corresponds to the CTRL4 `FRQ_CHG` field.
    pub enable_step_input_filtering: bool,

    /// Programs the filter setting for the STEP input. Controls how much noise to tolerate before
    /// the STEP input is considered outside the expected frequency.
    ///
    /// Corresponds to the CTRL4 `STEP_FRQ_TOL` field.
    pub step_frequency_tolerance: StepFrqTol,

    /// Stall threshold count. Values are clamped between 0 and 4095 because the hardware field is 12 bits.
    ///
    /// Corresponds to the CTRL5 `STALL_TH` and CTRL6 `STALL_TH` fields.
    // pub stall_threshold: u16,

    /// Controls the current ripple in smart tune ripple control decay mode.
    ///
    /// Corresponds to the CTRL6 `RC_RIPPLE` field.
    pub current_ripple: RcRipple,

    /// Controls spread-spectrum operation to reduce EMI.
    /// Enable when lower electromagnetic emissions are desired, such as when meeting
    /// EMC requirements or reducing interference with nearby electronics. Disable when
    /// fixed-frequency operation is preferred.
    ///
    /// Corresponds to the CTRL6 `DIS_SSC` field.
    pub enable_spread_spectrum: bool,

    /// Controls whether to scale to scale the torque count by a factor of 8.
    /// Corresponds to the CTRL6 `TRQ_SCALE` field.
    pub enable_torque_scaling: bool,

    /// Enables or disables open load detection faults.
    ///
    /// Corresponds to the CTRL9 `EN_OL` field.
    pub enable_open_load_detection: bool,

    /// Controls whether nFAULT is released after latched OL fault is cleared using
    /// CLR_FLT bit or nSLEEP reset pulse, or whether nFAULT is released immediately
    /// after OL fault condition is removed.
    ///
    /// Corresponds to the CTRL9 `OL_MODE` field.
    pub open_load_immediate_release: bool,

    /// Controls the time between when an open load is detected and when it is registered
    /// as a fault.
    ///
    /// Corresponds to the CTRL9 `OL_T` field.
    pub open_load_detection_time: OlT,

    /// Controls whether the STEP edge is active on only the rising edge (false) or both
    /// the rising and falling edge (true).
    ///
    /// Corresponds to the CTRL9 `STEP_EDGE` field.
    pub dual_step_edge: bool,

    /// Controls the resolution for auto microstepping mode.
    ///
    /// Corresponds to the CTRL9 `RES_AUTO` field.
    pub auto_microstepping_resolution: ResAuto,

    /// Controls whether auto microstepping mode is enabled.
    /// Corresponds to the CTRL9 `EN_AUTO` field.
    pub enable_auto_microstepping: bool,

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

    /// Determines whether the driver will enter a low-power state when completely idle.
    ///
    /// Corresponds to the CTRL11 `EN_STSL` field.
    pub standstill_power_saving_mode: bool,

    /// Controls the time it takes the current to reduce from the run current to the holding
    /// current after TSTSL_DLY time has elapsed.
    ///
    /// The hardware register is only 4 bits wide. Values >= 15 are effectively the same.
    ///
    /// 0b_000: fall time = 0
    ///
    /// 0b_0001: fall time for each current step = 1 ms
    ///
    /// ............
    ///
    /// 0b_0100: fall time for each current step = 4 ms
    ///
    /// ............
    ///
    /// 0b_1111: fall time for each current step = 15 ms
    ///
    /// Corresponds to the CTRL12 `TSTSL_FALL` field.
    pub standstill_fall_time: u8,

    /// Controls the delay between last STEP pulse and activation of standstill power saving mode.
    ///
    /// 0b_000000: Reserved
    ///
    /// 0b_000001: Delay = 1 x 16 ms = 16 ms
    ///
    /// ............
    ///
    /// 0b_000100: Delay = 4 x 16 ms = 64 ms
    ///
    /// ............
    ///
    /// 0b_111111: Delay = 63 x 16 ms = 1.008 s
    ///
    /// The hardware register is only 6 bits. Values >= 63 are effectively the same.
    ///
    /// Corresponds to the CTRL13 `TSTSL_DLY` field.
    pub standstill_delay: u8,

    /// Controls whether the device uses its internal 3.3V reference for current regulation
    /// and ignores the voltage on the VREF pin.
    ///
    /// Corresponds to the CTRL13 `VREF_INT_EN` field.
    pub enable_internal_voltage_reference: bool,
}

impl Default for Drv8462Config {
    fn default() -> Self {
        Self {
            output_rise_fall_time: Sr::default(),
            time_off: TOff::default(),
            decay: Decay::default(),
            microstep_mode: MicrostepMode::default(),
            overcurrent_protection_deglitch_time: TOcp::default(),
            overcurrent_condition_auto_retry: bool::default(),
            overtemperature_condition_auto_retry: bool::default(),
            report_temperature_warning: bool::default(),
            current_sense_blanking_time: TblankTime::default(),
            // enable_stall_detection: bool::default(),
            // report_stall_detection: bool::default(),
            enable_step_input_filtering: bool::default(),
            step_frequency_tolerance: StepFrqTol::default(),
            // stall_threshold: 0b_000000000011,
            current_ripple: RcRipple::default(),
            enable_spread_spectrum: bool::default(),
            enable_torque_scaling: bool::default(),
            enable_open_load_detection: bool::default(),
            open_load_immediate_release: bool::default(),
            open_load_detection_time: OlT::default(),
            dual_step_edge: bool::default(),
            auto_microstepping_resolution: ResAuto::default(),
            enable_auto_microstepping: bool::default(),
            holding_current: 0b_10000000,
            run_current: 0b_11111111,
            standstill_power_saving_mode: bool::default(),
            standstill_fall_time: 0b_0100,
            standstill_delay: 0b_000100,
            enable_internal_voltage_reference: bool::default(),
        }
    }
}

impl Drv8462Config {
    /// Serializes the CTRL1 fields to be written to the register.
    ///
    /// # Note:
    ///
    /// This always set the `EN_OUT` value in bit index 7 to 0, which will
    /// disable output when written. If you don't want to disable output,
    /// Re-enable the chip output after writing (recommended) or set this bit
    /// set this bit high yourself before writing.
    pub fn as_ctrl1(&self) -> u8 {
        (self.output_rise_fall_time as u8) << 6 | (self.time_off as u8) << 3 | self.decay as u8
    }

    /// Serializes the CTRL2 fields to be written to the register.
    pub fn as_ctrl2(&self) -> u8 {
        self.microstep_mode as u8
    }

    /// Serializes the CTRL3 fields to be written to the register.
    pub fn as_ctrl3(&self) -> u8 {
        (self.overcurrent_protection_deglitch_time as u8) << 3
            | (self.overcurrent_condition_auto_retry as u8) << 2
            | (self.overtemperature_condition_auto_retry as u8) << 1
            | self.report_temperature_warning as u8
    }

    /// Serializes the CTRL4 fields to be written to the register.
    pub fn as_ctrl4(&self) -> u8 {
        (self.current_sense_blanking_time as u8) << 6
            | (self.enable_step_input_filtering as u8) << 2
            | self.step_frequency_tolerance as u8
    }

    /// Serializes the CTRL6 fields to be written to the register.
    pub fn as_ctrl6(&self) -> u8 {
        (self.current_ripple as u8) << 6
            | (self.enable_spread_spectrum as u8) << 5
            | (self.enable_torque_scaling as u8) << 4
    }

    /// Serializes the CTRL9 fields to be written to the register.
    pub fn as_ctrl9(&self) -> u8 {
        (self.enable_open_load_detection as u8) << 7
            | (self.open_load_immediate_release as u8) << 6
            | (self.open_load_detection_time as u8) << 4
            | (self.dual_step_edge as u8) << 3
            | (self.auto_microstepping_resolution as u8) << 1
            | self.enable_auto_microstepping as u8
    }

    /// Serializes the CTRL10 fields to be written to the register.
    pub fn as_ctrl10(&self) -> u8 {
        self.holding_current as u8
    }

    /// Serializes the CTRL11 fields to be written to the register.
    pub fn as_ctrl11(&self) -> u8 {
        self.run_current as u8
    }

    /// Serializes the CTRL12 fields to be written to the register.
    pub fn as_ctrl12(&self) -> u8 {
        (self.standstill_power_saving_mode as u8) << 7 | (self.standstill_fall_time as u8) << 3
    }

    /// Serializes the CTRL13 fields to be written to the register.
    pub fn as_ctrl13(&self) -> u8 {
        (self.standstill_delay as u8) << 2 | (self.enable_internal_voltage_reference as u8) << 1
    }
}

#[repr(u8)]
#[derive(Default, Copy, Clone)]
/// Output rise fall time.
pub enum Sr {
    #[default]
    /// 140 ns
    Ns140 = 0b_0,
    /// 70 ns
    Ns70 = 0b_1,
}

#[repr(u8)]
#[derive(Default, Copy, Clone)]
/// Time off.
pub enum TOff {
    /// 9.5 μs.
    Us9_5 = 0b_00,
    #[default]
    /// 19 μs.
    Us19 = 0b_01,
    /// 27 μs.
    Us27 = 0b_10,
    /// 35 μs.
    Us35 = 0b_11,
}

#[repr(u8)]
#[derive(Default, Copy, Clone)]
/// Current regulation decay.
pub enum Decay {
    /// Slow decay mode. Generally results in smooth, quiet operation at lower speeds.
    SlowDecay = 0b_000,

    /// Mixed decay with approximately 30% fast decay.
    /// This can improve current regulation and reduce current distortion compared
    /// with pure slow decay, particularly as motor speed increases.
    Mixed30 = 0b_100,

    /// Mixed decay with approximately 60% fast decay.
    /// This setting can provide better current regulation at higher speeds,
    /// at the cost of potentially increased audible noise and ripple.
    Mixed60 = 0b_101,

    /// Dynamic decay mode.
    /// This is intended to provide good current regulation over a wide range of operating conditions
    /// without requiring the decay mode to be tuned manually.
    DynamicDecay = 0b_110,

    /// Ripple control mode.
    /// This is the default setting and is generally the preferred mode when
    /// you want the DRV8462 to automatically optimize current regulation.
    #[default]
    RippleControl = 0b_111,
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

#[repr(u8)]
#[derive(Default, Copy, Clone)]
/// Blanking time.
pub enum TblankTime {
    /// 1 μs.
    Us1 = 0b_00,
    #[default]
    /// 1.5 μs.
    Us1_5 = 0b_01,
    /// 2 μs.
    Us2 = 0b_10,
    /// 2.5 μs.
    Us2_5 = 0b_11,
}

#[repr(u8)]
#[derive(Default, Copy, Clone)]
/// Step frequency tolerance.
pub enum StepFrqTol {
    /// 1%.
    Pct1 = 0b_00,
    #[default]
    /// 2%.
    Pct2 = 0b_01,
    /// 4%.
    Pct4 = 0b_10,
    /// 6%.
    Pct6 = 0b_11,
}

#[repr(u8)]
#[derive(Default, Copy, Clone)]
/// RC ripple.
pub enum RcRipple {
    #[default]
    /// 1%.
    Pct1 = 0b_00,
    /// 2%.
    Pct2 = 0b_01,
    /// 4%.
    Pct4 = 0b_10,
    /// 6%.
    Pct6 = 0b_11,
}

#[repr(u8)]
#[derive(Default, Copy, Clone)]
/// Open load detection time.
pub enum OlT {
    /// 30 ms (max).
    Ms30 = 0b_00,
    #[default]
    /// 60 ms (max).
    Ms60 = 0b_01,
    /// 120 ms (max).
    Ms120 = 0b_10,
}

#[repr(u8)]
#[derive(Default, Copy, Clone)]
/// Auto resolution.
pub enum ResAuto {
    #[default]
    /// 1/256 step.
    TwoFiftySixthStep = 0b_00,
    /// 1/128 step.
    OneTwentyEightStep = 0b_01,
    /// 1/64 step.
    SixtyFourthStep = 0b_10,
    /// 1/32 step.
    ThirtySecondStep = 0b_11,
}
