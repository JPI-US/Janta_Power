#[derive(Default, Copy, Clone)]
pub struct Drv8462Config {
    pub microstep_mode: MicrostepMode,
    pub internal_voltage_reference: bool,
    pub run_current: u8,
    pub auto_microstepping_resolution: ResAuto,
    pub enable_auto_microstepping: bool,
}

#[repr(u8)]
#[derive(Default, Copy, Clone)]
pub enum MicrostepMode {
    FullStep100 = 0b_0000,
    FullStep71 = 0b_0001,
    HalfStepNonCircular = 0b_0010,
    HalfStep = 0b_0011,
    QuarterStep = 0b_0100,
    EighthStep = 0b_0101,
    #[default]
    SixteenthStep = 0b_0110,
    ThirtySecondStep = 0b_0111,
    SixtyFourthStep = 0b_1000,
    OneTwentyEighthStep = 0b_1001,
    TwoFiftySixthStep = 0b_1010,
}

#[repr(u8)]
#[derive(Default, Copy, Clone)]
pub enum ResAuto {
    #[default]
    TwoFiftySixthStep = 0b_00,
    OneTwentyEighthStep = 0b_01,
    SixtyFourthStep = 0b_10,
    ThirtySecondStep = 0b_11,
}
