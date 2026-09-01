//! Config settings pertaining to current regulation.

/// Config settings pertaining to current regulation.
///
/// # Fields
///
/// - `output_rise_fall_time` (`Sr`) - Controls the output rise and fall time.
/// - `time_off` (`TOff`) - Sets the fixed off-time used by current regulation.
/// - `decay` (`Decay`) - Controls how motor winding current decays.
/// - `current_ripple` (`RcRipple`) - Controls the current ripple in ripple control mode.
/// - `enable_spread_spectrum` (`bool`) - Enables spread-spectrum operation to reduce EMI.
/// - `enable_torque_scaling` (`bool`) - Enables torque count scaling by a factor of 8.
/// - `current_sense_blanking_time` (`TblankTime`) - Controls the current sense blanking time.
///
/// # Examples
///
/// ```
/// use crate::...;
///
/// let s = CurrentRegulation {
///     output_rise_fall_time: value,
///     time_off: value,
///     decay: value,
///     current_ripple: value,
///     enable_spread_spectrum: value,
///     enable_torque_scaling: value,
///     current_sense_blanking_time: value,
/// };
/// ```
#[derive(Default, Copy, Clone)]
pub struct CurrentRegulation {
    /// Controls how quickly the output transitions between high and low.
    ///
    /// Corresponds to the CTRL1 `SR` field.
    pub output_rise_fall_time: Sr,

    /// Sets the fixed off-time used by motor current regulation.
    ///
    /// Corresponds to the CTRL1 `T_OFF` field.
    pub time_off: TOff,

    /// Controls how motor winding current decays during current regulation.
    ///
    /// Corresponds to the CTRL1 `DECAY` field.
    pub decay: Decay,

    /// Controls the current ripple in smart tune ripple control decay mode.
    ///
    /// Corresponds to the CTRL6 `RC_RIPPLE` field.
    pub current_ripple: RcRipple,

    /// Enables spread-spectrum operation to reduce EMI.
    ///
    /// Corresponds to the CTRL6 `DIS_SSC` field.
    pub enable_spread_spectrum: bool,

    /// Enables torque count scaling by a factor of 8.
    ///
    /// Corresponds to the CTRL6 `TRQ_SCALE` field.
    pub enable_torque_scaling: bool,

    /// Controls the current sense blanking time.
    ///
    /// Corresponds to the CTRL4 `TBLANK_TIME` field.
    pub current_sense_blanking_time: TblankTime,
}

#[repr(u8)]
#[derive(Default, Copy, Clone)]
/// Output rise and fall time.
pub enum Sr {
    /// 140 ns.
    #[default]
    Ns140 = 0b_0,

    /// 70 ns.
    Ns70 = 0b_1,
}

#[repr(u8)]
#[derive(Default, Copy, Clone)]
/// Current regulation off-time.
pub enum TOff {
    /// 9.5 μs.
    Us9_5 = 0b_00,

    /// 19 μs.
    #[default]
    Us19 = 0b_01,

    /// 27 μs.
    Us27 = 0b_10,

    /// 35 μs.
    Us35 = 0b_11,
}

#[repr(u8)]
#[derive(Default, Copy, Clone)]
/// Current regulation decay mode.
pub enum Decay {
    /// Slow decay mode.
    SlowDecay = 0b_000,

    /// Mixed decay with approximately 30% fast decay.
    Mixed30 = 0b_100,

    /// Mixed decay with approximately 60% fast decay.
    Mixed60 = 0b_101,

    /// Dynamic decay mode.
    DynamicDecay = 0b_110,

    /// Ripple control mode.
    #[default]
    RippleControl = 0b_111,
}

#[repr(u8)]
#[derive(Default, Copy, Clone)]
/// Ripple control current ripple.
pub enum RcRipple {
    /// 1%.
    #[default]
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
/// Current sense blanking time.
pub enum TblankTime {
    /// 1 μs.
    Us1 = 0b_00,

    /// 1.5 μs.
    #[default]
    Us1_5 = 0b_01,

    /// 2 μs.
    Us2 = 0b_10,

    /// 2.5 μs.
    Us2_5 = 0b_11,
}
