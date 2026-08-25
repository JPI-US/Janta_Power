use drv8462::{config::*, Drv8462, Drv8462Hardware};
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

    let mut driver = Drv8462::new(
        Drv8462Hardware {
            spi: peripherals.spi2,
            sclk: peripherals.pins.gpio41,
            mosi: peripherals.pins.gpio40,
            miso: Some(peripherals.pins.gpio39),
            cs: peripherals.pins.gpio38,
            enable: peripherals.pins.gpio37,
            sleep: peripherals.pins.gpio36,
            step: peripherals.pins.gpio35,
            dir: peripherals.pins.gpio45,
        },
        Drv8462Config {
            microstep_mode: MicrostepMode::FullStep100,
            enable_internal_voltage_reference: true,
            run_current: 35,
            auto_microstepping_resolution: ResAuto::TwoFiftySixthStep,
            enable_auto_microstepping: true,
            ..Default::default()
        },
    )?;

    driver.set_enabled(true)?;
    driver.move_steps(2_500, true)?;

    Ok(())
}
