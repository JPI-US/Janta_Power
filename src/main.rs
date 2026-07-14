use core::time::Duration;
use std::sync::mpsc;

use anyhow::{Context, Result};
use fsm::Fsm;
use log::{error, info};

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

    let startup_result = Fsm::new_sync(
        Box::new(Initialization),
        StartupContext::new(),
        startup_events_tx,
        startup_commands_rx,
    )
    .context("Failed to start Startup FSM")?;

    let StartupContext {
        switchboard,
        sysloop: _,
        nvs,
        peripherals,
    } = startup_result;

    let _switchboard = switchboard.context("Startup did not provide switchboard")?;
    let _nvs = nvs.context("Startup did not provide NVS")?;
    let peripherals = peripherals.context("Startup did not provide peripherals")?;

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
        8 * 1_024,
        Duration::from_millis(100),
    )
    .context("Failed to start LED FSM")?;

    // maintenance fsm
    let maintenance_events_tx = conductor_tx.clone();
    let (maintenance_commands_tx, maintenance_commands_rx) = mpsc::channel();

    Fsm::new_async(
        Box::new(MaintenanceEnter),
        MaintenanceContext::new(peripherals.buttons),
        maintenance_events_tx,
        maintenance_commands_rx,
        "Maintenance Buttons",
        8 * 1_024,
        Duration::from_millis(100),
    )
    .context("Failed to start Maintenance FSM")?;

    // motion fsm
    let motion_events_tx = conductor_tx.clone();
    let (motion_commands_tx, motion_commands_rx) = mpsc::channel();

    Fsm::new_async(
        Box::new(MotionNotMoving),
        MotionContext::new(peripherals.motion),
        motion_events_tx,
        motion_commands_rx,
        "Motion",
        8 * 1_024,
        Duration::from_millis(100),
    )
    .context("Failed to start Motion FSM")?;

    loop {
        let cmd = conductor_rx.recv().context("Conductor channel closed")?;

        info!("{cmd:?}");

        if let Err(e) = led_commands_tx.send(cmd.clone()) {
            error!("Failed to send command to LED FSM: {e}");
        }

        if let Err(e) = maintenance_commands_tx.send(cmd.clone()) {
            error!("Failed to send command to Maintenance FSM: {e}");
        }

        if let Err(e) = motion_commands_tx.send(cmd) {
            error!("Failed to send command to Motion FSM: {e}");
        }
    }
}
