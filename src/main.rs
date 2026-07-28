use core::time::Duration;
use std::thread::sleep;

use anyhow::Result;
use fsm::{group::Group, postal::Postal, Fsm};
use rtc::Rtc;

use crate::logic::{
    fsm::{
        buttons::{ButtonsCheckPressed, ButtonsContext},
        led::{LEDCheck, LEDContext},
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
    } = startup()?;

    let mut postal = Postal::<FSMAddress, FSMCommand, FSMState>::new(25);

    let (motion_mailbox, motion_bulletin) = postal.take(FSMAddress::Motion);
    let (network_mailbox, network_bulletin) = postal.take(FSMAddress::Network);
    let (buttons_mailbox, buttons_bulletin) = postal.take(FSMAddress::Buttons);
    let (led_mailbox, led_bulletin) = postal.take(FSMAddress::Led);

    // Motion FSM.
    Fsm::new(
        Box::new(MotionInit),
        MotionContext::new(
            peripherals.motion,
            switchboard,
            nvs_default_partition.clone(),
            peripherals.i2c_bus,
            trust_nvs_state,
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

    // Buttons + LED group
    let mut group = Group::new("Buttons + LED", 8 * 1024, Duration::from_millis(100));

    // Buttons FSM.
    Fsm::new(
        Box::new(ButtonsCheckPressed),
        ButtonsContext::new(peripherals.buttons),
        buttons_mailbox,
        buttons_bulletin,
    )
    .group(&mut group);

    // LED FSM.
    Fsm::new(
        Box::new(LEDCheck),
        LEDContext::new(peripherals.led),
        led_mailbox,
        led_bulletin,
    )
    .group(&mut group);

    // Start buttons + LED
    group.spawn()?;

    Ok(())
}
