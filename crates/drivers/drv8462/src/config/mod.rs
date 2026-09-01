pub use auto_microstepping::*;
pub use basic::*;
pub use current_regulation::*;
pub use open_load::*;
pub use protection::*;
pub use silent_step::*;
pub use standstill::*;
pub use step::*;

pub mod auto_microstepping;
pub mod basic;
pub mod current_regulation;
pub mod open_load;
pub mod protection;
pub mod silent_step;
pub mod standstill;
pub mod step;

#[derive(Copy, Clone)]
/// Abstraction over the raw DRv8462 config registers. Each struct field maps to a register field on the chip.
/// For more information on the specifics of each field, see Page 74 of the [datasheet](https://www.ti.com/lit/ds/symlink/drv8452.pdf).
pub struct Drv8462Config {
    /// Basic settings such as microstepping mode, run current, holding current, etc.
    basic: Basic,
    /// Settings related to current regulation
    current_regulation: CurrentRegulation,
    /// Settings related to on-board protection and fault detection features.
    protection: Protection,
    /// Settings related to the step pulse.
    step: Step,
    /// Toggle and settings related to open load detection.
    open_load: (bool, OpenLoad),
    /// Toggle and settings related to auto microstepping.
    auto_microstepping: (bool, AutoMicrostepping),
    /// Toggle and settings related to standstill power saving.
    standstill: (bool, Standstill),
    /// Toggle and settings related to silent step.
    silent_step: (bool, SilentStep),
}

impl Default for Drv8462Config {
    fn default() -> Self {
        Self {
            basic: Basic::default(),
            current_regulation: CurrentRegulation::default(),
            protection: Protection::default(),
            step: Step::default(),
            open_load: (false, OpenLoad::default()),
            auto_microstepping: (false, AutoMicrostepping::default()),
            standstill: (false, Standstill::default()),
            silent_step: (false, SilentStep::default()),
            // enable_stall_detection: bool::default(),
            // report_stall_detection: bool::default(),
            // stall_threshold: 0b_000000000011,
        }
    }
}

impl Drv8462Config {
    pub fn new() -> Self {
        Self::default()
    }

    /// Serializes the CTRL1 fields to be written to the register.
    ///
    /// # Note:
    ///
    /// This always set the `EN_OUT` value in bit index 7 to 0, which will
    /// disable output when written. If you don't want to disable output,
    /// Re-enable the chip output after writing (recommended) or set this bit
    /// set this bit high yourself before writing.
    pub fn as_ctrl1(&self) -> u8 {
        (self.current_regulation.output_rise_fall_time as u8) << 6
            | (self.current_regulation.time_off as u8) << 3
            | self.current_regulation.decay as u8
    }

    /// Serializes the CTRL2 fields to be written to the register.
    pub fn as_ctrl2(&self) -> u8 {
        self.basic.microstep_mode as u8
    }

    /// Serializes the CTRL3 fields to be written to the register.
    pub fn as_ctrl3(&self) -> u8 {
        (self.protection.overcurrent_deglitch_time as u8) << 3
            | (self.protection.overcurrent_auto_retry as u8) << 2
            | (self.protection.overtemperature_auto_retry as u8) << 1
            | self.protection.report_temperature_warning as u8
    }

    /// Serializes the CTRL4 fields to be written to the register.
    pub fn as_ctrl4(&self) -> u8 {
        (self.current_regulation.current_sense_blanking_time as u8) << 6
            | (self.step.enable_filtering as u8) << 2
            | self.step.frequency_tolerance as u8
    }

    /// Serializes the CTRL6 fields to be written to the register.
    pub fn as_ctrl6(&self) -> u8 {
        (self.current_regulation.current_ripple as u8) << 6
            | (self.current_regulation.enable_spread_spectrum as u8) << 5
            | (self.current_regulation.enable_torque_scaling as u8) << 4
    }

    /// Serializes the CTRL9 fields to be written to the register.
    pub fn as_ctrl9(&self) -> u8 {
        (self.open_load.0 as u8) << 7
            | (self.open_load.1.immediate_release as u8) << 6
            | (self.open_load.1.detection_time as u8) << 4
            | (self.step.dual_edge as u8) << 3
            | (self.auto_microstepping.1.resolution as u8) << 1
            | self.auto_microstepping.0 as u8
    }

    /// Serializes the CTRL10 fields to be written to the register.
    pub fn as_ctrl10(&self) -> u8 {
        self.basic.holding_current
    }

    /// Serializes the CTRL11 fields to be written to the register.
    pub fn as_ctrl11(&self) -> u8 {
        self.basic.run_current
    }

    /// Serializes the CTRL12 fields to be written to the register.
    pub fn as_ctrl12(&self) -> u8 {
        let fall_time = self.standstill.1.fall_time.min(15);

        (self.standstill.0 as u8) << 7 | fall_time << 3
    }

    /// Serializes the CTRL13 fields to be written to the register.
    pub fn as_ctrl13(&self) -> u8 {
        let delay = self.standstill.1.delay.clamp(1, 63);

        delay << 2 | (self.basic.enable_internal_voltage_reference as u8) << 1
    }

    /// Serializes the SS_CTRL1 fields to be written to the register.
    pub fn as_ss_ctrl1(&self) -> u8 {
        (self.silent_step.1.sample_time as u8) << 7
            | (self.silent_step.1.frequency as u8) << 3
            | self.silent_step.0 as u8
    }

    /// Serializes the SS_CTRL2 fields to be written to the register.
    pub fn as_ss_ctrl2(&self) -> u8 {
        self.silent_step.1.proportional_gain.min(127)
    }

    /// Serializes the SS_CTRL3 fields to be written to the register.
    pub fn as_ss_ctrl3(&self) -> u8 {
        self.silent_step.1.integral_gain.min(127)
    }

    /// Serializes the SS_CTRL4 fields to be written to the register.
    pub fn as_ss_ctrl4(&self) -> u8 {
        (self.silent_step.1.ki_divider_factor as u8) << 6
            | (self.silent_step.1.kp_divider_factor as u8) << 2
    }

    /// Serializes the SS_CTRL5 fields to be written to the register.
    pub fn as_ss_ctrl5(&self) -> u8 {
        self.silent_step.1.transition_frequency.max(1)
    }

    /// Configures the basic driver settings.
    ///
    /// # Arguments
    ///
    /// - `config` (`Basic`) - Basic driver configuration.
    ///
    /// # Returns
    ///
    /// The updated driver configuration.
    pub fn configure_basic(mut self, config: Basic) -> Self {
        self.basic = config;
        self
    }

    /// Configures the current regulation settings.
    ///
    /// # Arguments
    ///
    /// - `config` (`CurrentRegulation`) - Current regulation configuration.
    ///
    /// # Returns
    ///
    /// The updated driver configuration.
    pub fn configure_current_regulation(mut self, config: CurrentRegulation) -> Self {
        self.current_regulation = config;
        self
    }

    /// Configures the driver protection settings.
    ///
    /// # Arguments
    ///
    /// - `config` (`Protection`) - Driver protection configuration.
    ///
    /// # Returns
    ///
    /// The updated driver configuration.
    pub fn configure_protection(mut self, config: Protection) -> Self {
        self.protection = config;
        self
    }

    /// Configures the step input settings.
    ///
    /// # Arguments
    ///
    /// - `config` (`Step`) - Step input configuration.
    ///
    /// # Returns
    ///
    /// The updated driver configuration.
    pub fn configure_step(mut self, config: Step) -> Self {
        self.step = config;
        self
    }

    /// Enables and configures open-load detection.
    ///
    /// # Arguments
    ///
    /// - `config` (`OpenLoad`) - Open-load detection configuration.
    ///
    /// # Returns
    ///
    /// The updated driver configuration.
    pub fn enable_open_load_detection(mut self, config: OpenLoad) -> Self {
        self.open_load = (true, config);
        self
    }

    /// Enables and configures automatic microstepping.
    ///
    /// # Arguments
    ///
    /// - `config` (`AutoMicrostepping`) - Automatic microstepping configuration.
    ///
    /// # Returns
    ///
    /// The updated driver configuration.
    pub fn enable_auto_microstepping(mut self, config: AutoMicrostepping) -> Self {
        self.auto_microstepping = (true, config);
        self
    }

    /// Enables and configures standstill power saving.
    ///
    /// # Arguments
    ///
    /// - `config` (`Standstill`) - Standstill power-saving configuration.
    ///
    /// # Returns
    ///
    /// The updated driver configuration.
    pub fn enable_standstill_power_saving(mut self, config: Standstill) -> Self {
        self.standstill = (true, config);
        self
    }

    /// Enables and configures SilentStep operation.
    ///
    /// # Arguments
    ///
    /// - `config` (`SilentStep`) - SilentStep configuration.
    ///
    /// # Returns
    ///
    /// The updated driver configuration.
    pub fn enable_silent_step(mut self, config: SilentStep) -> Self {
        self.silent_step = (true, config);
        self
    }
}
