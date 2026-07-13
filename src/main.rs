use core::time::Duration;
use std::sync::mpsc;

use anyhow::Result;

use crate::logic::fsm::{
    led::{LEDContext, LEDHold},
    maintenance::{MaintenanceContext, MaintenanceEnter},
    startup::{Initialization, StartupContext},
    Fsm,
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
    let mut fsm = Fsm::new(
        Box::new(Initialization),
        StartupContext::new(),
        startup_events_tx,
        startup_commands_rx,
    );
    fsm.run_sync()?;

    // state
    let StartupContext {
        switchboard,
        sysloop,
        nvs,
        peripherals,
    } = fsm.into_context();

    let _switchboard = switchboard.unwrap();
    let _nvs = nvs.unwrap();
    let peripherals = peripherals.unwrap();

    // led fsm
    let led_events_tx = conductor_tx.clone();
    let (led_commands_tx, led_commands_rx) = mpsc::channel();
    Fsm::new(
        Box::new(LEDHold),
        LEDContext {
            led: peripherals.led,
        },
        led_events_tx,
        led_commands_rx,
    )
    .run("LED control", 16_384, Duration::from_millis(100))?;

    // maintenance fsm
    let maintenance_events_tx = conductor_tx.clone();
    let (maintenance_commands_tx, maintenance_commands_rx) = mpsc::channel();
    Fsm::new(
        Box::new(MaintenanceEnter),
        MaintenanceContext::new(peripherals.buttons),
        maintenance_events_tx,
        maintenance_commands_rx,
    )
    .run("Maintenance buttons", 16_384, Duration::from_millis(100))?;

    loop {
        let cmd = conductor_rx.recv()?;
        log::info!("{cmd:?}");

        led_commands_tx.send(cmd)?;
        maintenance_commands_tx.send(cmd)?;
    }
}

#[derive(Debug, Clone, Copy)]
pub enum FSMCommand {
    LEDOff,
    LEDMaintenance,
    LEDMaintenanceMovingCCW,
    LEDMaintenanceMovingCW,
}
