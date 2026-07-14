use core::time::Duration;
use std::sync::mpsc;

use anyhow::Result;
use fsm::Fsm;

use crate::logic::fsm::{
    led::{LEDContext, LEDHold},
    maintenance::{MaintenanceContext, MaintenanceEnter},
    motion::{MotionContext, MotionNotMoving},
    startup::{Initialization, StartupContext},
};
mod config;
mod hardware;
mod logic;
mod services;
mod storage;

// Required by embassy_executor when esp-idf-svc embassy features are enabled.
#[no_mangle]
pub extern "C" fn __pender() {}

fn main() -> Result<()> {
    let (conductor_tx, conductor_rx) = mpsc::channel();

    // startup
    let startup_events_tx = conductor_tx.clone();
    let (_startup_commands_tx, startup_commands_rx) = mpsc::channel();
    let mut startup_result = Fsm::new_sync(
        Box::new(Initialization),
        StartupContext::new(),
        startup_events_tx,
        startup_commands_rx,
    )?;

    // get state
    let StartupContext {
        switchboard,
        sysloop,
        nvs,
        peripherals,
    } = startup_result;

    let _switchboard = switchboard.unwrap();
    let _nvs = nvs.unwrap();
    let peripherals = peripherals.unwrap();

    // led fsm
    let led_events_tx = conductor_tx.clone();
    let (led_commands_tx, led_commands_rx) = mpsc::channel();
    Fsm::new_async(
        Box::new(LEDHold),
        LEDContext {
            led: peripherals.led,
        },
        led_events_tx,
        led_commands_rx,
        "LED Control",
        16_384,
        Duration::from_millis(100),
    )?;

    // maintenance fsm
    let maintenance_events_tx = conductor_tx.clone();
    let (maintenance_commands_tx, maintenance_commands_rx) = mpsc::channel();
    Fsm::new_async(
        Box::new(MaintenanceEnter),
        MaintenanceContext::new(peripherals.buttons),
        maintenance_events_tx,
        maintenance_commands_rx,
        "Maintenance Buttons",
        16_384,
        Duration::from_millis(100),
    )?;

    // motion fsm
    let motion_events_tx = conductor_tx.clone();
    let (motion_commands_tx, motion_commands_rx) = mpsc::channel();
    Fsm::new_async(
        Box::new(MotionNotMoving),
        MotionContext::new(peripherals.motion),
        motion_events_tx,
        motion_commands_rx,
        "Motion",
        16_384,
        Duration::from_millis(100),
    )?;

    loop {
        let cmd = conductor_rx.recv().unwrap();
        log::info!("{cmd:?}");

        led_commands_tx.send(cmd).unwrap();
        maintenance_commands_tx.send(cmd).unwrap();
        motion_commands_tx.send(cmd).unwrap();
    }
}
