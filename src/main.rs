use core::time::Duration;

use anyhow::Result;
use esp_idf_svc::sys::*;
use fsm::{group::Group, postal::Postal, Fsm};
use rtc::Rtc;

use crate::logic::{
    fsm::{
        led::{LEDContext, LEDHold},
        motion::{MotionContext, MotionInit},
        network::{NetworkContext, WifiInitialize},
        FSMAddress,
        FSMCommand::{self, LEDOff},
    },
    startup::{startup, StartupContext},
};

mod config;
mod hardware;
mod logic;
mod services;
mod storage;

#[no_mangle]
pub extern "C" fn __pender() {}

fn main() -> Result<()> {
    let StartupContext {
        switchboard,
        sysloop,
        nvs_default_partition,
        peripherals,
        trust_nvs_state,
        version,
    } = startup()?;

    let mut postal = Postal::<FSMAddress, FSMCommand>::new(25);

    let maintenance_mailbox = postal.take(FSMAddress::Maintenance);
    let motion_mailbox = postal.take(FSMAddress::Motion);
    let network_mailbox = postal.take(FSMAddress::Network);

    Fsm::new(
        Box::new(LEDHold),
        LEDContext {
            led: peripherals.led,
        },
        maintenance_mailbox,
    )
    .spawn("LED", 8 * 1024, Duration::from_millis(500))?;

    // Motion FSM.
    Fsm::new(
        Box::new(MotionInit),
        MotionContext::new(
            peripherals.motion,
            switchboard,
            nvs_default_partition.clone(),
            peripherals.i2c_bus,
            trust_nvs_state,
            version,
        ),
        motion_mailbox,
    )
    .spawn("Motion", 8 * 1024, Duration::from_millis(10))?;

    // Network FSM.
    Fsm::new(
        Box::new(WifiInitialize),
        NetworkContext::new(
            nvs_default_partition,
            sysloop,
            peripherals.modem,
            switchboard,
            Rtc::new(peripherals.i2c_bus),
            peripherals.temperature_sensor,
        ),
        network_mailbox,
    )
    .spawn("Network", 8 * 1024, Duration::from_millis(10))?;

    loop {
        unsafe {
            println!("Free heap: {}", esp_get_free_heap_size());
        }

        std::thread::sleep(Duration::from_secs(1));
    }
}
