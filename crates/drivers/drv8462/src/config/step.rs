//! Config settings pertaining to the step signal.

#[derive(Default, Copy, Clone)]
/// Config settings pertaining to the step signal.
pub struct Step {
    /// Enables STEP input filtering according to [`Self::frequency_tolerance`].
    ///
    /// Corresponds to the CTRL4 `FRQ_CHG` field.
    pub enable_filtering: bool,

    /// Controls the frequency tolerance for the STEP input filter.
    ///
    /// Corresponds to the CTRL4 `STEP_FRQ_TOL` field.
    pub frequency_tolerance: StepFrqTol,

    /// Controls whether both rising and falling STEP edges are active.
    ///
    /// Corresponds to the CTRL9 `STEP_EDGE` field.
    pub dual_edge: bool,
}

#[repr(u8)]
#[derive(Default, Copy, Clone)]
/// STEP input frequency tolerance.
pub enum StepFrqTol {
    /// 1%.
    Pct1 = 0b_00,

    /// 2%.
    #[default]
    Pct2 = 0b_01,

    /// 4%.
    Pct4 = 0b_10,

    /// 6%.
    Pct6 = 0b_11,
}
