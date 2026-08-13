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

use crate::config::Drv8462Config;

pub mod config;

#[repr(u8)]
#[derive(Copy, Clone)]
pub enum Register {
    Fault = 0x00,
    Diag1 = 0x01,
    Diag2 = 0x02,
    Diag3 = 0x03,

    Ctrl1 = 0x04,
    Ctrl2 = 0x05,
    Ctrl3 = 0x06,
    Ctrl4 = 0x07,
    Ctrl6 = 0x09,
    Ctrl9 = 0x0C,
    Ctrl10 = 0x0D,
    Ctrl11 = 0x0E,
    Ctrl12 = 0x0F,
    Ctrl13 = 0x10,
}

pub struct Drv8462<'d, EN, SLP, STP, DIR, CS>
where
    EN: OutputPin,
    SLP: OutputPin,
    STP: OutputPin,
    DIR: OutputPin,
    CS: OutputPin,
{
    spi: SpiDeviceDriver<'d, SpiDriver<'d>>,
    cs: PinDriver<'d, CS, Output>,
    enable: PinDriver<'d, EN, Output>,
    sleep: PinDriver<'d, SLP, Output>,
    step: PinDriver<'d, STP, Output>,
    dir: PinDriver<'d, DIR, Output>,
    config: Drv8462Config,
    enabled: bool,
}

pub struct Drv8462Hardware<SPI, SCLK, MOSI, MISO, CS, EN, SLP, STP, DIR>
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

    EN: OutputPin,
    SLP: OutputPin,
    STP: OutputPin,
    DIR: OutputPin,
{
    pub spi: SPI,
    pub sclk: SCLK,
    pub mosi: MOSI,
    pub miso: Option<MISO>,
    pub cs: CS,
    pub enable: EN,
    pub sleep: SLP,
    pub step: STP,
    pub dir: DIR,
}

impl<'a, EN, SLP, STP, DIR, CS> Drv8462<'a, EN, SLP, STP, DIR, CS>
where
    EN: OutputPin,
    SLP: OutputPin,
    STP: OutputPin,
    DIR: OutputPin,
    CS: OutputPin,
{
    pub fn new<SPI, SCLK, MOSI, MISO>(
        hardware: Drv8462Hardware<SPI, SCLK, MOSI, MISO, CS, EN, SLP, STP, DIR>,
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

        let enable = PinDriver::output(hardware.enable)?;
        let sleep = PinDriver::output(hardware.sleep)?;
        let step = PinDriver::output(hardware.step)?;
        let dir = PinDriver::output(hardware.dir)?;

        let mut driver = Self {
            spi: device,
            cs,
            enable: enable,
            sleep: sleep,
            step,
            dir,
            config,
            enabled: false,
        };

        driver.apply_config()?;

        Ok(driver)
    }

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

    pub fn write_register(&mut self, reg: Register, value: u8) -> Result<u8> {
        let frame = ((reg as u16) << 8) | value as u16;

        self.unlock()?;
        let response = self.transfer16(frame)?;
        self.lock()?;

        Ok((response & 0xff) as u8)
    }

    pub fn read_register(&mut self, reg: Register) -> Result<u8> {
        let frame = (1 << 14) | ((reg as u16) << 8);

        let response = self.transfer16(frame)?;

        Ok((response & 0xff) as u8)
    }

    pub fn dump_all(&mut self) -> Result<()> {
        println!(
            "CTRL1=0x{:02X} CTRL2=0x{:02X} CTRL3=0x{:02X} CTRL4=0x{:02X}",
            self.read_register(Register::Ctrl1)?,
            self.read_register(Register::Ctrl2)?,
            self.read_register(Register::Ctrl3)?,
            self.read_register(Register::Ctrl4)?
        );

        println!(
            "CTRL9=0x{:02X} CTRL11=0x{:02X} CTRL13=0x{:02X}",
            self.read_register(Register::Ctrl9)?,
            self.read_register(Register::Ctrl11)?,
            self.read_register(Register::Ctrl13)?
        );

        println!(
            "FAULT=0x{:02X} DIAG1=0x{:02X} DIAG2=0x{:02X} DIAG3=0x{:02X}",
            self.read_register(Register::Fault)?,
            self.read_register(Register::Diag1)?,
            self.read_register(Register::Diag2)?,
            self.read_register(Register::Diag3)?
        );

        Ok(())
    }

    pub fn decode_fault(&mut self) -> Result<()> {
        let fault = self.read_register(Register::Fault)?;
        let diag1 = self.read_register(Register::Diag1)?;
        let diag2 = self.read_register(Register::Diag2)?;

        println!("FAULT=0x{:02X}", fault);
        println!("DIAG1=0x{:02X}", diag1);
        println!("DIAG2=0x{:02X}", diag2);

        if fault & 0x40 != 0 {
            println!(" -> SPI_ERROR");
        }

        if fault & 0x20 != 0 {
            println!(" -> UVLO");
        }

        if fault & 0x10 != 0 {
            println!(" -> CPUV");
        }

        if fault & 0x08 != 0 {
            println!(" -> OCP");
        }

        if fault & 0x01 != 0 {
            println!(" -> OL");
        }

        if diag2 & 0x01 != 0 {
            println!(" -> OL_A");
        }

        if diag2 & 0x02 != 0 {
            println!(" -> OL_B");
        }

        if diag2 & 0x08 != 0 {
            println!(" -> STALL");
        }

        Ok(())
    }

    pub fn apply_config(&mut self) -> Result<()> {
        self.sleep.set_high()?;
        self.dump_all()?;
        self.clear_faults()?;

        self.write_register(Register::Ctrl1, self.config.as_ctrl1())?;
        self.write_register(Register::Ctrl2, self.config.as_ctrl2())?;
        self.write_register(Register::Ctrl3, self.config.as_ctrl3())?;
        self.write_register(Register::Ctrl4, self.config.as_ctrl4())?;
        self.write_register(Register::Ctrl6, self.config.as_ctrl6())?;
        self.write_register(Register::Ctrl9, self.config.as_ctrl9())?;
        self.write_register(Register::Ctrl10, self.config.as_ctrl10())?;
        self.write_register(Register::Ctrl11, self.config.as_ctrl11())?;
        self.write_register(Register::Ctrl12, self.config.as_ctrl12())?;
        self.write_register(Register::Ctrl1, self.config.as_ctrl13())?;

        self.lock()?;

        Ok(())
    }

    pub fn set_enabled(&mut self, enable: bool) -> Result<()> {
        let mut ctrl1 = self.read_register(Register::Ctrl1)?;

        ctrl1 &= !(0b_1 << 7);
        ctrl1 |= (enable as u8) << 7;

        self.write_register(Register::Ctrl1, ctrl1)?;

        if enable {
            self.enable.set_high()?;
        } else {
            self.enable.set_low()?;
        }

        self.enabled = enable;

        Ok(())
    }

    fn clear_faults(&mut self) -> Result<()> {
        let mut ctrl3 = self.read_register(Register::Ctrl3)?;

        ctrl3 |= 0b_1 << 7;

        self.write_register(Register::Ctrl3, ctrl3)?;

        Ok(())
    }

    fn unlock(&mut self) -> Result<()> {
        let mut ctrl3 = self.read_register(Register::Ctrl3)?;

        ctrl3 &= !(0b_111 << 4);
        ctrl3 |= 0b_011 << 4;

        self.write_register(Register::Ctrl3, ctrl3)?;
        self.clear_faults()?;

        Ok(())
    }

    fn lock(&mut self) -> Result<()> {
        let mut ctrl3 = self.read_register(Register::Ctrl3)?;

        ctrl3 &= !(0b_111 << 4);
        ctrl3 |= 0b_110 << 4;

        self.write_register(Register::Ctrl3, ctrl3)?;
        self.clear_faults()?;

        Ok(())
    }

    pub fn step_once(&mut self, delay_us: u32) -> Result<()> {
        self.step.set_high()?;
        Ets::delay_us(2);

        self.step.set_low()?;
        Ets::delay_us(delay_us);

        Ok(())
    }

    pub fn move_steps(&mut self, steps: u32, forward: bool) -> Result<()> {
        self.enable.set_high()?;

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

            if i % 500 == 0 {
                let fault = self.read_register(Register::Fault)?;
                if fault != 0 {
                    println!("FAULT = {:02X}", fault);
                    self.decode_fault()?;
                }
            }
        }

        self.enable.set_low()?;

        Ok(())
    }
}
