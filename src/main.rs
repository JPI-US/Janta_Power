use core::time::Duration;

use anyhow::Result;
use fsm::{group::Group, postal::Postal, Fsm};

use crate::logic::{
    fsm::{
        buttons::{ButtonsCheckPressed, ButtonsContext},
        diagnostics::{DiagIdle, DiagnosticsContext},
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
        // Claimed inside `startup()`, immediately after the logger and before any
        // peripheral is taken — see the comment there for why the ordering
        // matters.
        diagnostics_console,
    } = startup()?;

    let mut postal = Postal::<FSMAddress, FSMCommand, FSMState>::new(25);

    let (motion_mailbox, motion_bulletin) = postal.take(FSMAddress::Motion);
    let (network_mailbox, network_bulletin) = postal.take(FSMAddress::Network);
    let (buttons_mailbox, buttons_bulletin) = postal.take(FSMAddress::Buttons);
    let (led_mailbox, led_bulletin) = postal.take(FSMAddress::Led);
    let (diagnostics_mailbox, diagnostics_bulletin) = postal.take(FSMAddress::Diagnostics);

    // Diagnostics FSM.
    //
    // Its own thread, and deliberately not part of the Auxiliary group below: a
    // full-screen OLED write takes about a second on this board's 10 kHz bus, and
    // that would be a second in which the Buttons machine does not run. Buttons
    // carries the maintenance stop, and blocking a safety input to draw a test
    // pattern is not a trade worth making.
    //
    // Never gated behind a feature flag. This is the interface you reach for when
    // something else is wrong, so it has to be the thing that is still working.
    //
    // 20 ms so a burst of provisioning commands drains faster than it arrives; a
    // poll with nothing waiting is one non-blocking read. 12 KB because the
    // protocol formats strings and the display driver builds a 133-byte page
    // buffer on the stack — measure the high-water mark before trimming it.
    //
    // Skipped, loudly, if the console could not be claimed. The tower still
    // tracks; it just cannot be asked about itself over USB.
    match diagnostics_console {
        Some(console) => {
            Fsm::new(
                "Diagnostics",
                Box::new(DiagIdle),
                DiagnosticsContext::new(
                    console,
                    peripherals.i2c_bus,
                    peripherals.sensors.clone(),
                    nvs_default_partition.clone(),
                )?,
                diagnostics_mailbox,
                diagnostics_bulletin,
            )
            .spawn(12 * 1024, Duration::from_millis(20))?;
        }
        None => {
            log::error!("Diagnostics FSM not started: no USB Serial/JTAG console");
        }
    }

    // Motion FSM.
    Fsm::new(
        "Motion",
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
    .spawn(8 * 1024, Duration::from_millis(100))?;

    // Network FSM.
    Fsm::new(
        "Network",
        Box::new(WifiInitialize),
        NetworkContext::new(
            nvs_default_partition,
            sysloop,
            peripherals.modem,
            switchboard,
            peripherals.sensors.clone(),
        ),
        network_mailbox,
        network_bulletin,
    )
    .spawn(8 * 1024, Duration::from_millis(10))?;

    // Buttons + LED group
    let mut group = Group::new("Auxiliary", 8 * 1024, Duration::from_millis(100));

    // Buttons FSM.
    Fsm::new(
        "Buttons",
        Box::new(ButtonsCheckPressed),
        ButtonsContext::new(peripherals.buttons),
        buttons_mailbox,
        buttons_bulletin,
    )
    .group(&mut group);

    // LED FSM.
    Fsm::new(
        "LED",
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
