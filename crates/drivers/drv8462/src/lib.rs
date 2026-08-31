//! SPI firmware control for the Texas Instruments DRV8462 stepper motor driver.

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

pub use crate::{config::*, faults::*, registers::*};

pub mod config;
pub mod faults;
pub mod registers;

/// Represents and controls the DRV8462 device.
///
/// # Examples
///
/// ```ignore
/// fn main() -> anyhow::Result<()> {
///     esp_idf_svc::sys::link_patches();
///
///     let peripherals = Peripherals::take().unwrap();
///
///     let mut driver = Drv8462::new(
///         Drv8462Hardware {
///             spi: peripherals.spi2,
///             sclk: peripherals.pins.gpio41,
///             mosi: peripherals.pins.gpio40,
///             miso: Some(peripherals.pins.gpio39),
///             cs: peripherals.pins.gpio38,
///             sleep: peripherals.pins.gpio36,
///             step: peripherals.pins.gpio35,
///             dir: peripherals.pins.gpio45,
///         },
///         Drv8462Config {
///             microstep_mode: MicrostepMode::FullStep100,
///             enable_internal_voltage_reference: true,
///             run_current: 35,
///             auto_microstepping_resolution: ResAuto::TwoFiftySixthStep,
///             enable_auto_microstepping: true,
///             ..Default::default()
///         },
///     )?;
///
///     driver.move_steps(2_500, true)?;
///
///     Ok(())
/// }
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

    /// DRV8462 step input GPIO.
    step: PinDriver<'d, STP, Output>,

    /// DRV8462 direction input GPIO.
    dir: PinDriver<'d, DIR, Output>,

    /// Configuration used to initialize the DRV8462 registers.
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
    pub miso: Option<MISO>,

    /// SPI chip-select pin.
    pub cs: CS,

    /// DRV8462 sleep control pin.
    pub sleep: SLP,

    /// DRV8462 step input pin.
    pub step: STP,

    /// DRV8462 direction input pin.
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
    /// The DRV8462 is initially configured to be awake with disabled outputs.
    ///
    /// # Arguments
    ///
    /// * `hardware` - SPI peripheral and GPIO resources connected to the
    ///   DRV8462. see [`Drv8462Hardware`].
    /// * `config` - Initial DRV8462 register configuration. See [`Drv8462Config`]
    ///
    /// # Errors
    ///
    /// Returns an error if SPI or GPIO initialization fails, or if
    /// communication with the DRV8462 fails while applying the configuration.
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
            hardware.miso,
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
    /// # Arguments
    ///
    /// * `value` - 16-bit SPI frame to transmit.
    ///
    /// # Returns
    ///
    /// The 16-bit frame returned by the DRV8462.
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
    /// # Arguments
    ///
    /// * `reg` - Register to write.
    /// * `value` - Value to write to the register.
    ///
    /// # Returns
    ///
    /// The low byte returned by the DRV8462 during the SPI transaction.
    ///
    /// # Errors
    ///
    /// Returns an error if unlocking, SPI communication, or locking fails.
    pub fn write_register(&mut self, reg: Register, value: u8) -> Result<u8> {
        let frame = ((reg as u16) << 8) | value as u16;

        let response = self.transfer16(frame)?;

        Ok((response & 0xff) as u8)
    }

    /// Reads an 8-bit value from a DRV8462 register.
    ///
    /// # Arguments
    ///
    /// * `reg` - Register to read.
    ///
    /// # Returns
    ///
    /// The current 8-bit register value returned by the device.
    ///
    /// # Errors
    ///
    /// Returns an error if the SPI transaction fails.
    pub fn read_register(&mut self, reg: Register) -> Result<u8> {
        let frame = (1 << 14) | ((reg as u16) << 8);

        let response = self.transfer16(frame)?;

        Ok((response & 0xff) as u8)
    }

    /// Applies the configured DRV8462 register values.
    ///
    /// This sets the device to be not sleeping, with outputs disabled.
    ///
    /// # Errors
    ///
    /// Returns an error if a GPIO operation, register read, or register write
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
    /// # Arguments
    ///
    /// * `enable` - `true` to enable the driver, `false` to disable it.
    ///
    /// # Errors
    ///
    /// Returns an error if `CTRL1` cannot be read or written.
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
    /// Clear faults after handling a fault condition. Faults that are still active
    /// after being cleared may be reported again. Output will not be given to the
    /// motor if there is an active fault.
    fn clear_faults(&mut self) -> Result<()> {
        let mut ctrl3 = self.read_register(Register::Ctrl3)?;

        ctrl3 |= 1 << 7;

        self.write_register(Register::Ctrl3, ctrl3)?;

        Ok(())
    }

    /// Unlocks all registers and allows them to be written to.
    ///
    /// [`lock`] the registers again after writing.
    ///
    /// # Errors
    ///
    /// Returns an error if `CTRL3` cannot be read or written.
    fn unlock(&mut self) -> Result<()> {
        let mut ctrl3 = self.read_register(Register::Ctrl3)?;

        ctrl3 &= !(0b111 << 4);
        ctrl3 |= 0b011 << 4;

        self.write_register(Register::Ctrl3, ctrl3)?;

        Ok(())
    }

    /// Locks all registers and bars them from being written to.
    ///
    /// [`unlock`] the register to be able to write it.
    ///
    /// # Errors
    ///
    /// Returns an error if `CTRL3` cannot be read or written.
    fn lock(&mut self) -> Result<()> {
        let mut ctrl3 = self.read_register(Register::Ctrl3)?;

        ctrl3 &= !(0b111 << 4);
        ctrl3 |= 0b110 << 4;

        self.write_register(Register::Ctrl3, ctrl3)?;

        Ok(())
    }

    /// Generates one step pulse if there is no active fault.
    ///
    /// # Arguments
    ///
    /// * `delay_us` - Delay, in microseconds, after the step pulse.
    ///
    /// # Returns
    ///
    /// * `Ok(true)` if the step pulse was generated.
    /// * `Ok(false)` if an active fault prevented the step.
    ///
    /// # Errors
    ///
    /// Returns an error if fault detection, register access, or GPIO control
    /// fails.
    pub fn step_once(&mut self, delay_us: u32) -> Result<bool> {
        if self.get_faults()?.fault_active {
            return Ok(false);
        }

        self.set_enabled(true)?;

        self.step.set_high()?;
        Ets::delay_us(2);

        self.step.set_low()?;
        Ets::delay_us(delay_us);

        self.set_enabled(false)?;

        Ok(true)
    }

    /// Moves the motor by generating a sequence of step pulses.
    ///
    /// The motor will accelerate and decelerate linearly. Motion stops if a
    /// fault occurs.
    ///
    /// # Arguments
    ///
    /// * `steps` - Number of step pulses to generate.
    /// * `forward` - `true` to move one direction, `false` to move the other.
    ///
    /// # Returns
    ///
    /// * `Ok(true)` if the requested motion completed.
    /// * `Ok(false)` if an active fault prevented motion.
    ///
    /// # Errors
    ///
    /// Returns an error if GPIO or SPI communication fails.
    ///
    /// # Panics
    ///
    /// This method does not intentionally panic for a valid `steps` value.
    pub fn move_steps(&mut self, steps: u32, forward: bool) -> Result<bool> {
        if self.get_faults()?.fault_active {
            return Ok(false);
        }

        if forward {
            self.dir.set_high()?;
        } else {
            self.dir.set_low()?;
        }

        Ets::delay_us(20);

        let start_delay = 20_000;
        let min_delay = 5_000;
        let accel_steps = (steps / 4).min(1000);

        for i in 0..steps {
            let delay = if i < accel_steps {
                start_delay - ((start_delay - min_delay) * i / accel_steps)
            } else if i > steps - accel_steps {
                let remaining = steps - i;
                start_delay - ((start_delay - min_delay) * remaining / accel_steps)
            } else {
                min_delay
            };

            self.step_once(delay)?;
        }

        Ok(true)
    }

    /// Reads the DRV8462 fault register and returns the currently reported
    /// fault conditions.
    ///
    /// Multiple fault conditions may be reported simultaneously. The returned
    /// [`Faults`] structure contains an individual flag for each fault condition
    /// represented by the DRV8462 fault register.
    ///
    /// # Returns
    ///
    /// - `Ok(Faults)` - The fault conditions currently reported by the DRV8462.
    /// - `Err(_)` - An error if the fault register cannot be read.
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the DRV8462 fails while reading the
    /// fault register.
    ///
    /// # Examples
    ///
    /// ```
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
    pub fn get_faults(&mut self) -> Result<Faults> {
        let fault_reg = self.read_register(Register::Fault)?;

        Ok(Faults {
            fault_active: fault_reg & 0x_80 != 0,
            spi_error: fault_reg & 0x_40 != 0,
            undervoltage_lockout: fault_reg & 0x_20 != 0,
            charge_pump_undervoltage: fault_reg & 0x_10 != 0,
            over_current: fault_reg & 0x_8 != 0,
            stall: fault_reg & 0x_4 != 0,
            temperature: fault_reg & 0x_2 != 0,
            open_load: fault_reg & 0x_1 != 0,
        })
    }
}
