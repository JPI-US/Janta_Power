#[derive(Default, Clone, Copy)]
pub struct AutoMicrostepping {
    /// Controls the resolution for auto microstepping mode.
    ///
    /// Corresponds to the CTRL9 `RES_AUTO` field.
    pub resolution: ResAuto,

    /// Controls whether auto microstepping mode is enabled.
    /// Corresponds to the CTRL9 `EN_AUTO` field.
    pub enable: bool,
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
