use drv8462::{config::*, AutoMicrostepping, Basic, Drv8462, Drv8462Hardware};
use esp_idf_svc::hal::prelude::Peripherals;

mod app;
#[path = "../constants.rs"]
#[allow(dead_code)]
mod constants;
#[path = "../diagnostics/mod.rs"]
mod diagnostics;
mod infra;
#[path = "../switchboard.rs"]
mod switchboard;

// Required by embassy_executor when esp-idf-svc embassy features are enabled.
#[no_mangle]
pub extern "C" fn __pender() {}

fn main() -> anyhow::Result<()> {
    esp_idf_svc::sys::link_patches();

    let peripherals = Peripherals::take().unwrap();

    let driver_hardware = Drv8462Hardware {
        spi: peripherals.spi2,
        sclk: peripherals.pins.gpio0,
        mosi: peripherals.pins.gpio38,
        miso: Some(peripherals.pins.gpio39),
        cs: peripherals.pins.gpio40,
        sleep: peripherals.pins.gpio21,
        step: peripherals.pins.gpio26,
        dir: peripherals.pins.gpio48,
    };

    let driver_config = Drv8462Config::new()
        .configure_basic(Basic {
            microstep_mode: MicrostepMode::FullStep100,
            enable_internal_voltage_reference: true,
            run_current: 35,
            ..Default::default()
        })
        .enable_auto_microstepping(AutoMicrostepping {
            resolution: ResAuto::TwoFiftySixthStep,
            ..Default::default()
        });

    let mut driver = Drv8462::new(driver_hardware, driver_config)?;

    driver.move_steps(100, true)?;

    Ok(())
}
