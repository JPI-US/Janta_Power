use anyhow::{anyhow, Context, Result};
use drv8462::{
    AutoMicrostepping, Basic, Drv8462, Drv8462Config, Drv8462Hardware, MicrostepMode, ResAuto,
    SilentStep, SsDivSel, SsPwmFreq, SsSmplSel,
};
use esp_idf_svc::hal::{
    delay::Ets,
    gpio::{Gpio10, Gpio11, Gpio14, Gpio21, Gpio4, Gpio40, Gpio45, Gpio46, Gpio48, Gpio5, Gpio6},
    i2c::{I2cConfig, I2cDriver},
    modem::Modem,
    prelude::*,
};
use hdc1080::Hdc1080;
use log::{info, warn};
use motion::motion::{ActiveLevel, Motion};
use rgb_led::Led;
use shared_bus::{BusManager, I2cProxy};

use crate::hardware::buttons::Buttons;

/// Collection of initialized hardware peripherals used by the device.
pub struct PeripheralMap<'a> {
    /// Shared I2C bus used by connected peripherals.
    pub i2c_bus: &'static BusManager<std::sync::Mutex<I2cDriver<'static>>>,

    /// Status LED controller.
    pub led: Led<'a>,

    /// Motion control peripherals.
    pub motion: Motion<'a, Gpio21, Gpio45, Gpio48, Gpio40, Gpio46, Gpio14, Gpio10, Gpio11>,

    /// Cellular modem peripheral.
    pub modem: Modem,

    /// Temperature sensor.
    pub temperature_sensor:
        Option<Hdc1080<I2cProxy<'static, std::sync::Mutex<I2cDriver<'static>>>, Ets>>,

    /// East, West, and maintenance buttons
    pub buttons: Buttons<'static, Gpio5, Gpio4, Gpio6>,
}

impl PeripheralMap<'_> {
    /// Initializes all hardware peripherals required by the device.
    ///
    /// This method takes ownership of the available hardware peripherals,
    /// configures communication interfaces, initializes device controllers,
    /// and returns a fully configured peripheral map.
    ///
    /// This should only ever need to be run once.
    ///
    /// # Errors
    ///
    /// Returns an error if any peripheral fails to initialize or if a required
    /// hardware resource cannot be acquired.
    pub fn new(
        relay_active_level: ActiveLevel,
        limit_switch_active_level: ActiveLevel,
    ) -> Result<Self> {
        let peripherals = Peripherals::take().context("Failed to take peripherals")?;

        // i2c
        let i2c_config = I2cConfig::new().baudrate(10_u32.kHz().into());
        let i2c = I2cDriver::new(
            peripherals.i2c0,
            peripherals.pins.gpio8,
            peripherals.pins.gpio9,
            &i2c_config,
        )
        .context("Failed to create I2cDriver")?;
        let i2c_bus: &'static _ =
            shared_bus::new_std!(I2cDriver = i2c).ok_or(anyhow!("Failed to create shared bus"))?;

        // status led
        let led = Led::new(peripherals.pins.gpio7, peripherals.rmt.channel0)?;

        // Motor driver setup
        let motor_hardware = Drv8462Hardware {
            spi: peripherals.spi2,
            sclk: peripherals.pins.gpio0,
            mosi: peripherals.pins.gpio38,
            miso: peripherals.pins.gpio39,
            cs: peripherals.pins.gpio40,
            sleep: peripherals.pins.gpio21,
            step: peripherals.pins.gpio45,
            dir: peripherals.pins.gpio48,
        };

        let motor_config = Drv8462Config::new()
            .configure_basic(Basic {
                microstep_mode: MicrostepMode::TwoFiftySixthStep,
                enable_internal_voltage_reference: true,
                run_current: 45,
                ..Default::default()
            })
            .enable_auto_microstepping(AutoMicrostepping {
                resolution: ResAuto::TwoFiftySixthStep,
            })
            .enable_silent_step(SilentStep {
                sample_time: SsSmplSel::Us2,
                frequency: SsPwmFreq::Khz50,
                proportional_gain: 64,
                integral_gain: 64,
                ki_divider_factor: SsDivSel::Div32,
                kp_divider_factor: SsDivSel::Div32,
                transition_frequency: 1,
            });
        let motor_device = Drv8462::new(motor_hardware, motor_config)?;

        let motion = Motion::new(
            motor_device,
            peripherals.pins.gpio46, // relay
            peripherals.pins.gpio10, // encoder a
            peripherals.pins.gpio11, // encoder b
            peripherals.pins.gpio14, // limit switch
            limit_switch_active_level,
            relay_active_level,
        )?;

        let temperature_sensor = match Hdc1080::new(i2c_bus.acquire_i2c(), Ets) {
            Ok(mut sensor) => {
                let _ = sensor.init();
                if sensor.get_device_id().unwrap_or(0) == 0x1050 {
                    info!("HDC1080 detected");
                    Some(sensor)
                } else {
                    warn!("HDC1080 not detected on I2C; temp telemetry disabled");
                    None
                }
            }
            Err(e) => {
                warn!("HDC1080 init failed: {:?}; temp telemetry disabled", e);
                None
            }
        };

        // modem
        let modem = peripherals.modem;

        // buttons
        let buttons = Buttons::new(
            peripherals.pins.gpio5,
            peripherals.pins.gpio4,
            peripherals.pins.gpio6,
        )?;

        Ok(Self {
            i2c_bus,
            led,
            motion,
            modem,
            temperature_sensor,
            buttons,
        })
    }
}
