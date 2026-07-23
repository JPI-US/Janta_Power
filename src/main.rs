use core::time::Duration;

use anyhow::Result;
use crossbeam_channel::unbounded;
use esp_idf_svc::sys::*;
use fsm::{group::Group, Fsm};
use log::{error, info};
use rtc::Rtc;

use crate::logic::{
    fsm::{
        led::{LEDContext, LEDHold},
        maintenance::{MaintenanceContext, MaintenanceEnter},
        motion::{MotionContext, MotionInit},
        network::{NetworkContext, WifiInitialize},
    },
    startup::{startup, StartupContext},
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
    let (conductor_tx, conductor_rx) = unbounded();

    // startup
    let StartupContext {
        switchboard,
        sysloop,
        nvs_default_partition,
        peripherals,
        trust_nvs_state,
        version,
    } = startup().expect("Failed to set up initial context");

    // create led + maintenance button group
    let mut group = Group::new(
        "LED + Maintenance Button",
        8 * 1_024,
        Duration::from_millis(500),
    );

    // create led fsm and add to group
    let led_events_tx = conductor_tx.clone();
    let (led_commands_tx, led_commands_rx) = unbounded();

    Fsm::new(
        Box::new(LEDHold),
        LEDContext {
            led: peripherals.led,
        },
        led_events_tx,
        led_commands_rx,
    )
    .group(&mut group);

    // create maintenance fsm and add to group
    let maintenance_events_tx = conductor_tx.clone();
    let (maintenance_commands_tx, maintenance_commands_rx) = unbounded();

    Fsm::new(
        Box::new(MaintenanceEnter),
        MaintenanceContext::new(peripherals.buttons),
        maintenance_events_tx,
        maintenance_commands_rx,
    )
    .group(&mut group);

    // start the group
    group.spawn()?;

    // motion fsm
    let motion_events_tx = conductor_tx.clone();
    let (motion_commands_tx, motion_commands_rx) = unbounded();

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
        motion_events_tx,
        motion_commands_rx,
    )
    .spawn("Motion", 8 * 1_024, Duration::from_millis(10))?;

    // network fsm
    let network_events_tx = conductor_tx.clone();
    let (network_commands_tx, network_commands_rx) = unbounded();

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
        network_events_tx,
        network_commands_rx,
    )
    .spawn("Network", 8 * 1_024, Duration::from_millis(10))?;

    loop {
        unsafe {
            println!("Free heap: {}", esp_get_free_heap_size());
        }

        if let Ok(cmd) = conductor_rx.recv_timeout(Duration::from_secs(1)) {
            info!("{cmd:?}");

            if let Err(e) = led_commands_tx.send(cmd.clone()) {
                error!("Failed to send command to LED FSM: {e}");
            }

            if let Err(e) = maintenance_commands_tx.send(cmd.clone()) {
                error!("Failed to send command to Maintenance FSM: {e}");
            }

            if let Err(e) = motion_commands_tx.send(cmd.clone()) {
                error!("Failed to send command to Motion FSM: {e}");
            }

            if let Err(e) = network_commands_tx.send(cmd) {
                error!("Failed to send command to Wifi FSM: {e}");
            }
        }
    }
}
