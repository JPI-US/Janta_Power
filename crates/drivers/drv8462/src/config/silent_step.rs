//! Config settings pertaining to SilentStep operation.

#[derive(Default, Clone, Copy)]
/// Config settings pertaining to SilentStep operation.
pub struct SilentStep {
    /// Controls the SilentStep current zero-crossing sampling time.
    ///
    /// Corresponds to the SS_CTRL1 `SS_SMPL_SEL` field.
    pub sample_time: SsSmplSel,

    /// Controls the SilentStep PWM frequency.
    ///
    /// Corresponds to the SS_CTRL1 `SS_PWM_FREQ` field.
    pub frequency: SsPwmFreq,

    /// Controls the proportional gain of the SilentStep PI controller.
    ///
    /// Values are clamped to the 7-bit hardware range of 0 to 127.
    ///
    /// Corresponds to the SS_CTRL2 `SS_KP` field.
    pub proportional_gain: u8,

    /// Controls the integral gain of the SilentStep PI controller.
    ///
    /// Values are clamped to the 7-bit hardware range of 0 to 127.
    ///
    /// Corresponds to the SS_CTRL3 `SS_KI` field.
    pub integral_gain: u8,

    /// Selects the divider factor for the integral gain.
    ///
    /// Actual KI = `SS_KI / SS_KI_DIV_SEL`.
    ///
    /// Corresponds to the SS_CTRL4 `SS_KI_DIV_SEL` field.
    pub ki_divider_factor: SsDivSel,

    /// Selects the divider factor for the proportional gain.
    ///
    /// Actual KP = `SS_KP / SS_KP_DIV_SEL`.
    ///
    /// Corresponds to the SS_CTRL4 `SS_KP_DIV_SEL` field.
    pub kp_divider_factor: SsDivSel,

    /// Controls the frequency at which SilentStep transitions to the
    /// configured decay mode.
    ///
    /// The value is corrected to 1 if set to 0.
    ///
    /// Corresponds to the SS_CTRL5 `SS_TRANS_FREQ` field.
    pub transition_frequency: u8,
}

#[repr(u8)]
#[derive(Default, Copy, Clone)]
/// SilentStep current zero-crossing sampling time.
pub enum SsSmplSel {
    /// 2 μs.
    #[default]
    Us2 = 0b_00,

    /// 3 μs.
    Us3 = 0b_01,

    /// 4 μs.
    Us4 = 0b_10,

    /// 5 μs.
    Us5 = 0b_11,
}

#[repr(u8)]
#[derive(Default, Copy, Clone)]
/// SilentStep PWM frequency.
pub enum SsPwmFreq {
    /// 25 kHz.
    #[default]
    Khz25 = 0b_00,

    /// 33 kHz.
    Khz33 = 0b_01,

    /// 42 kHz.
    Khz42 = 0b_10,

    /// 50 kHz.
    Khz50 = 0b_11,
}

#[repr(u8)]
#[derive(Default, Copy, Clone)]
/// SilentStep KI/KP divider factor.
pub enum SsDivSel {
    /// KI or KP / 32.
    #[default]
    Div32 = 0b_000,

    /// KI or KP / 64.
    Div64 = 0b_001,

    /// KI or KP / 128.
    Div128 = 0b_010,

    /// KI or KP / 256.
    Div256 = 0b_011,

    /// KI or KP / 512.
    Div512 = 0b_100,

    /// KI or KP / 16.
    Div16 = 0b_101,

    /// KI or KP with no division.
    NoDiv = 0b_110,
}
