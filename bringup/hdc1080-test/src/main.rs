//! HDC1080 I2C bring-up firmware.
//!
//! Flash: `cargo run -p hdc1080-test`
//!
//! Wiring (ESP32-S3): GPIO8 = SDA, GPIO9 = SCL (same as production tower).
//! Expected IDs: device `0x1050`, manufacturer `0x5449`.

use esp_idf_svc::hal::{
    delay::{Ets, FreeRtos},
    i2c::{I2cConfig, I2cDriver},
    peripherals::Peripherals,
    prelude::*,
};
use hdc1080::Hdc1080;

// Required by embassy_executor when esp-idf-svc embassy features are enabled.
#[no_mangle]
pub extern "C" fn __pender() {}

fn main() -> anyhow::Result<()> {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take()?;

    let sda = peripherals.pins.gpio8;
    let scl = peripherals.pins.gpio9;

    let config = I2cConfig::new().baudrate(100.kHz().into());
    let i2c = I2cDriver::new(peripherals.i2c0, sda, scl, &config)?;

    let mut sensor =
        Hdc1080::new(i2c, Ets).map_err(|e| anyhow::anyhow!("bus init error: {e:?}"))?;
    sensor
        .init()
        .map_err(|e| anyhow::anyhow!("config error: {e:?}"))?;

    let dev_id = sensor.get_device_id().unwrap_or(0);
    let man_id = sensor.get_man_id().unwrap_or(0);
    println!("Device ID:       0x{dev_id:04X}  (expected 0x1050)");
    println!("Manufacturer ID: 0x{man_id:04X}  (expected 0x5449)");
    if dev_id != 0x1050 {
        println!(">> Not reading the chip. 0x0000 means no data on the bus");
        println!("   (check wiring, 4.7k pull-ups on SDA/SCL, address 0x40, 3V3 power).");
    }

    loop {
        let (temp_c, rh) = sensor
            .read()
            .map_err(|e| anyhow::anyhow!("read error: {e:?}"))?;
        println!("Temp: {temp_c:6.2} °C   Humidity: {rh:5.1} %RH");

        if temp_c <= -39.9 && rh <= 0.1 {
            println!("  ^ all-zero reading — likely no data on the bus, not a real measurement");
        }

        FreeRtos::delay_ms(1000);
    }
}
