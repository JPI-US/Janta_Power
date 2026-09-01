#[derive(Default, Clone, Copy)]
pub struct OpenLoad {
    /// Controls whether nFAULT is released after latched OL fault is cleared using
    /// CLR_FLT bit or nSLEEP reset pulse, or whether nFAULT is released immediately
    /// after OL fault condition is removed.
    ///
    /// Corresponds to the CTRL9 `OL_MODE` field.
    pub immediate_release: bool,

    /// Controls the time between when an open load is detected and when it is registered
    /// as a fault.
    ///
    /// Corresponds to the CTRL9 `OL_T` field.
    pub detection_time: OlT,
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
