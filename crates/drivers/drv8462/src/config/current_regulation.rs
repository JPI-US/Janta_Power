#[derive(Default, Copy, Clone)]
pub struct CurrentRegulation {
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

    /// Controls whether to scale the torque count by a factor of 8.
    /// Corresponds to the CTRL6 `TRQ_SCALE` field.
    pub enable_torque_scaling: bool,

    /// Controls the current sense blanking time.
    ///
    /// Corresponds to the CTRL4 `TBLANK_TIME` field.
    pub current_sense_blanking_time: TblankTime,
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
