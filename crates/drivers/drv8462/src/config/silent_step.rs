#[derive(Clone, Copy)]
pub struct SilentStep {
    /// Controls the silent step current zero cross sampling time.
    /// Increase the sampling time if current waveform is distorted around zero crossing.
    pub silent_step_sample_time: SsSmplSel,

    /// Represents the silent step PWM frequency.
    pub silent_step_frequency: SsPwmFreq,

    /// Represents whether or not silent step decay mode should be on.
    pub silent_step_enable: bool,

    /// Represents the proportional gain of the silent step PI controller.
    ///
    /// Hardware register is only 7 bits wide. Values are clamped between 0 and 127.
    pub silent_step_proportional_gain: u8,

    /// Represents the integral gain of the silent step PI controller.
    ///
    /// Hardware register is only 7 bits wide. Values are clamped between 0 and 127.
    pub silent_step_integral_gain: u8,

    /// Divider factor for KI. Actual KI = SS_KI / SS_KI_DIV_SEL.
    pub silent_step_ki_divider_factor: SsDivSel,

    /// Divider factor for KP. Actual KP = SS_KP / SS_KP_DIV_SEL.
    pub silent_step_kp_divider_factor: SsDivSel,

    /// Programs the frequency at which the device transitions from
    /// silent step decay mode to another decay mode programmed by
    /// the DECAY bits. This frequency corresponds to the frequency of
    /// the sinusoidal current waveform.
    ///
    /// Corrects to 1 if set to 0.
    ///
    /// 00000001b: 2 Hz
    ///
    /// 00000010b: 4 Hz
    ///
    /// ............
    ///
    /// 11111111b: 510 Hz
    pub silent_step_transition_frequency: u8,
}

impl Default for SilentStep {
    fn default() -> Self {
        Self {
            silent_step_sample_time: SsSmplSel::default(),
            silent_step_frequency: SsPwmFreq::default(),
            silent_step_enable: false,
            silent_step_proportional_gain: 0b_0000000,
            silent_step_integral_gain: 0b_0000000,
            silent_step_ki_divider_factor: SsDivSel::default(),
            silent_step_kp_divider_factor: SsDivSel::default(),
            silent_step_transition_frequency: 0b_00000000,
        }
    }
}

#[repr(u8)]
#[derive(Default, Copy, Clone)]
/// Silent step current zero crossing time.
pub enum SsSmplSel {
    #[default]
    /// 2 μs.
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
/// Silent step PWM frequency.
pub enum SsPwmFreq {
    #[default]
    Khz25 = 0b_00,
    Khz33 = 0b_01,
    Khz42 = 0b_10,
    Khz50 = 0b_11,
}

#[repr(u8)]
#[derive(Default, Copy, Clone)]
/// Silent step KI/KP divider factor select.
pub enum SsDivSel {
    #[default]
    /// KI OR KP / 32.
    Div32 = 0b_000,
    /// KI OR KP / 64.
    Div64 = 0b_001,
    /// KI OR KP / 128.
    Div128 = 0b_010,
    /// KI OR KP / 256.
    Div256 = 0b_011,
    /// KI OR KP / 512.
    Div512 = 0b_100,
    /// KI OR KP / 16.
    Div16 = 0b_101,
    /// KI OR KP
    NoDiv = 0b_110,
}
