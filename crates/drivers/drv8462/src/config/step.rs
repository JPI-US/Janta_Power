#[derive(Default, Copy, Clone)]
pub struct Step {
    /// Enables or disables STEP input filtering as per [`Self::step_frequency_tolerance`].
    ///
    /// Corresponds to the CTRL4 `FRQ_CHG` field.
    pub enable_step_input_filtering: bool,

    /// Programs the filter setting for the STEP input. Controls how much noise to tolerate before
    /// the STEP input is considered outside the expected frequency.
    ///
    /// Corresponds to the CTRL4 `STEP_FRQ_TOL` field.
    pub step_frequency_tolerance: StepFrqTol,

    /// Controls whether the STEP edge is active on only the rising edge (false) or both
    /// the rising and falling edge (true).
    ///
    /// Corresponds to the CTRL9 `STEP_EDGE` field.
    pub dual_step_edge: bool,
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
