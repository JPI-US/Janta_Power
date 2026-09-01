//! Config settings pertaining to open load detection.

#[derive(Default, Clone, Copy)]
/// Config settings pertaining to open load detection.
pub struct OpenLoad {
    /// Controls when nFAULT is released after an open-load fault is cleared.
    ///
    /// Corresponds to the CTRL9 `OL_MODE` field.
    pub immediate_release: bool,

    /// Controls the time between detecting an open load and registering it as a fault.
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

    /// 60 ms (max).
    #[default]
    Ms60 = 0b_01,

    /// 120 ms (max).
    Ms120 = 0b_10,
}
