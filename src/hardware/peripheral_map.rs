use anyhow::{anyhow, Context, Result};
use esp_idf_svc::hal::{
    delay::Ets,
    gpio::{Gpio4, Gpio5, Gpio6},
    i2c::{I2cConfig, I2cDriver},
    modem::Modem,
    prelude::*,
};
use hdc1080::Hdc1080;
use log::{info, warn};
use motion::motion::{ActiveLevel, Motion};
use rgb_led::Led;
use rtc::Rtc;
use shared_bus::BusManager;

use crate::hardware::{
    buttons::Buttons,
    sensors::{self, SharedSensors},
};

/// Collection of initialized hardware peripherals used by the device.
pub struct PeripheralMap<'a> {
    /// Shared I2C bus used by connected peripherals.
    pub i2c_bus: &'static BusManager<std::sync::Mutex<I2cDriver<'static>>>,

    /// Status LED controller.
    pub led: Led<'a>,

    /// Motion control peripherals.
    pub motion: Motion<'a>,

    /// Cellular modem peripheral.
    pub modem: Modem,

    /// Devices that need an owner rather than just a bus lock — the HDC1080 and
    /// the DS3231. Handed out as a shared handle because two machines read them:
    /// the network machine for telemetry, the diagnostics machine on request.
    /// See [`crate::hardware::sensors`].
    pub sensors: SharedSensors,

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

        // motion (motor, encoder, relay, limit switch)
        let motion = Motion::new(
            peripherals.pins.gpio15,
            peripherals.pins.gpio16,
            peripherals.pins.gpio17,
            peripherals.pins.gpio14,
            peripherals.pins.gpio10,
            peripherals.pins.gpio11,
            relay_active_level,
            limit_switch_active_level,
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
            sensors: sensors::shared(temperature_sensor, Rtc::new(i2c_bus)),
            buttons,
        })
    }
}
