#[derive(Clone, Copy)]
pub struct Standstill {
    /// Determines whether the driver will enter a low-power state when completely idle.
    ///
    /// Corresponds to the CTRL11 `EN_STSL` field.
    pub standstill_power_saving_mode: bool,

    /// Controls the time it takes the current to reduce from the run current to the holding
    /// current after TSTSL_DLY time has elapsed.
    ///
    /// The hardware register is only 4 bits wide. Values are capped between 0 and 15.
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
    /// The hardware register is only 6 bits. Values are clamped between 1 and 63.
    ///
    /// Corresponds to the CTRL13 `TSTSL_DLY` field.
    pub standstill_delay: u8,
}

impl Default for Standstill {
    fn default() -> Self {
        Self {
            standstill_power_saving_mode: bool::default(),
            standstill_fall_time: 0b_0100,
            standstill_delay: 0b_000100,
        }
    }
}
