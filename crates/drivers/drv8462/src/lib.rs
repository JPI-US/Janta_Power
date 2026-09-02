//! SPI firmware control for the Texas Instruments DRV8462 stepper motor driver.

use accel_stepper::{Device, StepContext};
use anyhow::Result;
use esp_idf_svc::hal::{
    delay::Ets,
    gpio::{InputPin, Output, OutputPin, PinDriver},
    peripheral::Peripheral,
    prelude::*,
    spi::{
        config::{Config as SpiConfig, DriverConfig, MODE_1},
        SpiAnyPins, SpiDeviceDriver, SpiDriver,
    },
};
use log::error;

pub use crate::{config::*, faults::*, registers::*};

pub mod config;
pub mod faults;
pub mod registers;

/// Represents and controls the DRV8462 device.
///
/// # Examples
///
/// ```ignore
/// let driver_hardware = Drv8462Hardware {
///     spi: peripherals.spi2,
///     sclk: peripherals.pins.gpio0,
///     mosi: peripherals.pins.gpio38,
///     miso: Some(peripherals.pins.gpio39),
///     cs: peripherals.pins.gpio40,
///     sleep: peripherals.pins.gpio21,
///     step: peripherals.pins.gpio26,
///     dir: peripherals.pins.gpio48,
/// };
///
/// let driver_config = Drv8462Config::new()
///     .configure_basic(Basic {
///         microstep_mode: MicrostepMode::FullStep100,
///         enable_internal_voltage_reference: true,
///         run_current: 35,
///         ..Default::default()
///     })
///     .enable_auto_microstepping(AutoMicrostepping {
///         resolution: ResAuto::TwoFiftySixthStep,
///         ..Default::default()
///     });
///
/// let mut driver = Drv8462::new(driver_hardware, driver_config)?;
///
/// driver.move_steps(100, true)?;
/// ```
pub struct Drv8462<'d, SLP, STP, DIR, CS>
where
    SLP: OutputPin,
    STP: OutputPin,
    DIR: OutputPin,
    CS: OutputPin,
{
    /// SPI device used to communicate with the DRV8462.
    spi: SpiDeviceDriver<'d, SpiDriver<'d>>,

    /// SPI chip-select GPIO.
    cs: PinDriver<'d, CS, Output>,

    /// Sleep control GPIO.
    sleep: PinDriver<'d, SLP, Output>,

    /// Step input GPIO.
    step: PinDriver<'d, STP, Output>,

    /// Direction input GPIO.
    dir: PinDriver<'d, DIR, Output>,

    /// Configuration used for the DRV8462 registers.
    config: Drv8462Config,

    /// Tracks the requested driver enable state.
    enabled: bool,
}

/// Hardware resources required to construct a [`Drv8462`].
pub struct Drv8462Hardware<SPI, SCLK, MOSI, MISO, CS, SLP, STP, DIR>
where
    SPI: Peripheral,
    SPI::P: SpiAnyPins,

    SCLK: Peripheral,
    SCLK::P: OutputPin,

    MOSI: Peripheral,
    MOSI::P: OutputPin,

    MISO: Peripheral,
    MISO::P: InputPin,

    CS: Peripheral,
    CS::P: OutputPin,

    SLP: OutputPin,
    STP: OutputPin,
    DIR: OutputPin,
{
    /// SPI peripheral.
    pub spi: SPI,

    /// SPI clock pin.
    pub sclk: SCLK,

    /// SPI MOSI pin.
    pub mosi: MOSI,

    /// SPI MISO pin.
    pub miso: MISO,

    /// SPI chip-select pin.
    pub cs: CS,

    /// Sleep control pin.
    pub sleep: SLP,

    /// Step input pin.
    pub step: STP,

    /// Direction input pin.
    pub dir: DIR,
}

impl<'a, SLP, STP, DIR, CS> Drv8462<'a, SLP, STP, DIR, CS>
where
    SLP: OutputPin,
    STP: OutputPin,
    DIR: OutputPin,
    CS: OutputPin,
{
    /// Creates and initializes a DRV8462 driver.
    ///
    /// The driver starts awake with its outputs disabled.
    ///
    /// # Errors
    ///
    /// Returns an error if SPI or GPIO initialization fails, or if applying
    /// the initial configuration fails.
    ///
    /// # Examples
    ///
    /// ```ignore
    /// let driver = Drv8462::new(hardware, config)?;
    /// # Ok::<(), anyhow::Error>(())
    /// ```
    pub fn new<SPI, SCLK, MOSI, MISO>(
        hardware: Drv8462Hardware<SPI, SCLK, MOSI, MISO, CS, SLP, STP, DIR>,
        config: Drv8462Config,
    ) -> Result<Self>
    where
        SPI: Peripheral + 'a,
        SPI::P: SpiAnyPins + 'a,

        SCLK: Peripheral + 'a,
        SCLK::P: OutputPin + 'a,

        MOSI: Peripheral + 'a,
        MOSI::P: OutputPin + 'a,

        MISO: Peripheral + 'a,
        MISO::P: InputPin + 'a,

        CS: Peripheral + 'a,
        CS::P: OutputPin + 'a,
    {
        let spi = SpiDriver::new(
            hardware.spi,
            hardware.sclk,
            hardware.mosi,
            Some(hardware.miso),
            &DriverConfig::new(),
        )?;

        let spi_config = SpiConfig::new()
            .baudrate(500.kHz().into())
            .data_mode(MODE_1);

        let device = SpiDeviceDriver::new(spi, None::<CS>, &spi_config)?;

        let cs = PinDriver::output(hardware.cs)?;
        let sleep = PinDriver::output(hardware.sleep)?;
        let step = PinDriver::output(hardware.step)?;
        let dir = PinDriver::output(hardware.dir)?;

        let mut driver = Self {
            spi: device,
            cs,
            sleep,
            step,
            dir,
            config,
            enabled: false,
        };

        driver.apply_config()?;

        Ok(driver)
    }

    /// Transfers one 16-bit SPI frame.
    ///
    /// # Errors
    ///
    /// Returns an error if chip-select control or the SPI transfer fails.
    fn transfer16(&mut self, value: u16) -> Result<u16> {
        let tx = value.to_be_bytes();
        let mut rx = [0u8; 2];

        self.cs.set_low()?;
        Ets::delay_us(1);

        self.spi.transfer(&mut rx, &tx)?;

        self.cs.set_high()?;
        Ets::delay_us(2);

        Ok(u16::from_be_bytes(rx))
    }

    /// Writes an 8-bit value to a DRV8462 register.
    ///
    /// Returns the low byte received during the transaction.
    ///
    /// # Errors
    ///
    /// Returns an error if the SPI transfer fails.
    pub fn write_register(&mut self, reg: Register, value: u8) -> Result<u8> {
        let frame = ((reg as u16) << 8) | value as u16;

        let response = self.transfer16(frame)?;

        Ok((response & 0xff) as u8)
    }

    /// Reads an 8-bit value from a DRV8462 register.
    ///
    /// # Errors
    ///
    /// Returns an error if the SPI transfer fails.
    pub fn read_register(&mut self, reg: Register) -> Result<u8> {
        let frame = (1 << 14) | ((reg as u16) << 8);

        let response = self.transfer16(frame)?;

        Ok((response & 0xff) as u8)
    }

    /// Applies the configured DRV8462 register values.
    ///
    /// The device is awakened and its outputs remain disabled.
    ///
    /// # Errors
    ///
    /// Returns an error if GPIO control, register access, or configuration
    /// fails.
    pub fn apply_config(&mut self) -> Result<()> {
        self.sleep.set_high()?;
        Ets::delay_us(1_000);

        self.clear_faults()?;
        self.unlock()?;

        self.write_register(Register::Ctrl1, self.config.as_ctrl1())?;
        self.write_register(Register::Ctrl2, self.config.as_ctrl2())?;
        self.write_register(Register::Ctrl3, self.config.as_ctrl3())?;
        self.write_register(Register::Ctrl4, self.config.as_ctrl4())?;
        self.write_register(Register::Ctrl6, self.config.as_ctrl6())?;
        self.write_register(Register::Ctrl9, self.config.as_ctrl9())?;
        self.write_register(Register::Ctrl10, self.config.as_ctrl10())?;
        self.write_register(Register::Ctrl11, self.config.as_ctrl11())?;
        self.write_register(Register::Ctrl12, self.config.as_ctrl12())?;
        self.write_register(Register::Ctrl13, self.config.as_ctrl13())?;
        self.write_register(Register::SsCtrl1, self.config.as_ss_ctrl1())?;
        self.write_register(Register::SsCtrl2, self.config.as_ss_ctrl2())?;
        self.write_register(Register::SsCtrl3, self.config.as_ss_ctrl3())?;
        self.write_register(Register::SsCtrl4, self.config.as_ss_ctrl4())?;
        self.write_register(Register::SsCtrl5, self.config.as_ss_ctrl5())?;

        self.lock()?;

        Ok(())
    }

    /// Enables or disables the motor driver.
    ///
    /// # Errors
    ///
    /// Returns an error if the CTRL1 register cannot be read or written.
    pub fn set_enabled(&mut self, enable: bool) -> Result<()> {
        let mut ctrl1 = self.read_register(Register::Ctrl1)?;

        ctrl1 &= !(0b1 << 7);
        ctrl1 |= (enable as u8) << 7;

        self.unlock()?;
        self.write_register(Register::Ctrl1, ctrl1)?;
        self.lock()?;

        self.enabled = enable;

        Ok(())
    }

    /// Clears the DRV8462 fault state.
    ///
    /// Active faults may be reported again after being cleared.
    fn clear_faults(&mut self) -> Result<()> {
        let mut ctrl3 = self.read_register(Register::Ctrl3)?;

        ctrl3 |= 1 << 7;

        self.write_register(Register::Ctrl3, ctrl3)?;

        Ok(())
    }

    /// Unlocks all writable registers.
    ///
    /// # Errors
    ///
    /// Returns an error if the CTRL3 register cannot be read or written.
    fn unlock(&mut self) -> Result<()> {
        let mut ctrl3 = self.read_register(Register::Ctrl3)?;

        ctrl3 &= !(0b111 << 4);
        ctrl3 |= 0b011 << 4;

        self.write_register(Register::Ctrl3, ctrl3)?;

        Ok(())
    }

    /// Locks all writable registers.
    ///
    /// # Errors
    ///
    /// Returns an error if the CTRL3 register cannot be read or written.
    fn lock(&mut self) -> Result<()> {
        let mut ctrl3 = self.read_register(Register::Ctrl3)?;

        ctrl3 &= !(0b111 << 4);
        ctrl3 |= 0b110 << 4;

        self.write_register(Register::Ctrl3, ctrl3)?;

        Ok(())
    }

    /// Generates one step pulse if there is no active fault.
    ///
    /// Returns `true` if the pulse was generated, or `false` if an active
    /// fault prevented the step.
    ///
    /// # Errors
    ///
    /// Returns an error if fault detection, register access, or GPIO control
    /// fails.
    pub fn step_once(&mut self, forward: bool) -> Result<()> {
        self.get_faults()?;

        if forward {
            self.dir.set_high()?;
        } else {
            self.dir.set_low()?;
        }

        self.step.set_high()?;
        Ets::delay_us(2);

        self.step.set_low()?;
        Ets::delay_us(2);

        Ok(())
    }

    /// Reads the DRV8462 fault register.
    ///
    /// Multiple fault conditions may be reported simultaneously.
    ///
    /// # Errors
    ///
    /// Returns an error if the fault register cannot be read or if active faults are read.
    ///
    /// # Examples
    ///
    /// ```ignore
    /// let faults = driver.get_faults()?;
    ///
    /// if faults.spi_error {
    ///     // Handle SPI communication error.
    /// }
    ///
    /// if faults.over_current {
    ///     // Handle over-current fault.
    /// }
    /// ```
    pub fn get_faults(&mut self) -> Result<()> {
        let fault_reg = self.read_register(Register::Fault)?;

        let fault_active = fault_reg & 0x_80 != 0;

        let faults = Faults {
            spi_error: fault_reg & 0x_40 != 0,
            undervoltage_lockout: fault_reg & 0x_20 != 0,
            charge_pump_undervoltage: fault_reg & 0x_10 != 0,
            over_current: fault_reg & 0x_8 != 0,
            stall: fault_reg & 0x_4 != 0,
            temperature: fault_reg & 0x_2 != 0,
            open_load: fault_reg & 0x_1 != 0,
        };

        if fault_active {
            Err(anyhow::Error::new(faults))
        } else {
            Ok(())
        }
    }
}

impl<'a, SLP, STP, DIR, CS> Device for Drv8462<'a, SLP, STP, DIR, CS>
where
    SLP: OutputPin,
    STP: OutputPin,
    DIR: OutputPin,
    CS: OutputPin,
{
    type Error = anyhow::Error;

    fn step(&mut self, ctx: &StepContext) -> Result<(), Self::Error> {
        match ctx.position {
            0 => self.step_once(false),
            1 => self.step_once(true),
            _ => unreachable!("Step context position should always be 0 or 1"),
        }
    }
}
