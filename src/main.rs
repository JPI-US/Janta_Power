use core::time::Duration;

use anyhow::{Context, Result};
use crossbeam_channel::unbounded;
use esp_idf_svc::sys::*;
use fsm::Fsm;
use log::{error, info};
use rtc::Rtc;

use crate::logic::fsm::{
    led::{LEDContext, LEDHold},
    maintenance::{MaintenanceContext, MaintenanceEnter},
    motion::{MotionContext, MotionInit},
    network::{NetworkContext, WifiInitialize},
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
    let (conductor_tx, conductor_rx) = unbounded();

    // startup
    let startup_events_tx = conductor_tx.clone();
    let (_startup_commands_tx, startup_commands_rx) = unbounded();

    let startup_result = Fsm::new_sync(
        Box::new(Initialization),
        StartupContext::new(),
        startup_events_tx,
        startup_commands_rx,
        250,
    )
    .context("Failed to start Startup FSM")?;

    let StartupContext {
        switchboard,
        sysloop,
        nvs_default_partition,
        peripherals,
        trust_nvs_state,
        version,
    } = startup_result;

    let switchboard = switchboard.context("Startup did not provide switchboard")?;
    let sysloop = sysloop.context("Startup did not provide sysloop")?;
    let nvs_default_partition =
        nvs_default_partition.context("Startup did not provide nvs_default_partition")?;
    let peripherals = peripherals.context("Startup did not provide peripherals")?;
    let trust_nvs_state = trust_nvs_state.context("Startup did not provide trust_nvs_state")?;
    let version = version.context("Startup did not provide version")?;

    // led fsm
    let led_events_tx = conductor_tx.clone();
    let (led_commands_tx, led_commands_rx) = unbounded();

    Fsm::new_async(
        Box::new(LEDHold),
        LEDContext {
            led: peripherals.led,
        },
        led_events_tx,
        led_commands_rx,
        250,
        "LED Control",
        8 * 1_024,
        Duration::from_millis(100),
    )
    .context("Failed to start LED FSM")?;

    // maintenance fsm
    let maintenance_events_tx = conductor_tx.clone();
    let (maintenance_commands_tx, maintenance_commands_rx) = unbounded();

    Fsm::new_async(
        Box::new(MaintenanceEnter),
        MaintenanceContext::new(peripherals.buttons),
        maintenance_events_tx,
        maintenance_commands_rx,
        250,
        "Maintenance Buttons",
        8 * 1_024,
        Duration::from_millis(100),
    )
    .context("Failed to start Maintenance FSM")?;

    // motion fsm
    let motion_events_tx = conductor_tx.clone();
    let (motion_commands_tx, motion_commands_rx) = unbounded();

    Fsm::new_async(
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
        250,
        "Motion",
        8 * 1_024,
        Duration::from_millis(100),
    )
    .context("Failed to start Motion FSM")?;

    // network fsm
    let network_events_tx = conductor_tx.clone();
    let (network_commands_tx, network_commands_rx) = unbounded();

    Fsm::new_async(
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
        250,
        "Network",
        8 * 1_024,
        Duration::from_secs(2),
    )
    .context("Failed to start Network FSM")?;

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
