use core::time::Duration;

use anyhow::Result;
use esp_idf_svc::sys::*;
use fsm::{postal::Postal, Fsm};
use rtc::Rtc;

use crate::logic::{
    fsm::{
        motion::{MotionContext, MotionInit},
        network::{NetworkContext, WifiInitialize},
        FSMAddress,
        FSMCommand::{self},
        FSMState,
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

    let mut postal = Postal::<FSMAddress, FSMCommand, FSMState>::new(25);

    let (motion_mailbox, motion_bulletin) = postal.take(FSMAddress::Motion);
    let (network_mailbox, network_bulletin) = postal.take(FSMAddress::Network);

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
        motion_bulletin,
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
        network_bulletin,
    )
    .spawn("Network", 8 * 1024, Duration::from_millis(10))?;

    loop {
        unsafe {
            println!("Free heap: {}", esp_get_free_heap_size());
        }

        std::thread::sleep(Duration::from_secs(1));
    }
}
