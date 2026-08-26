//! The board diagnostics channel: a line protocol over USB Serial/JTAG.
//!
//! This is the interface a technician's laptop talks to over the same cable that
//! flashes the board — distinct from [`crate::services`], which is the remote
//! command channel over MQTT. The two will share a command catalog; they do not
//! share a transport.
//!
//! # Why this is its own machine on its own thread
//!
//! Because the answer to "can you run a diagnostic right now?" should be yes.
//!
//! In the single-threaded firmware this replaced, roughly half the diagnostics
//! code existed to interleave serial into whatever the main loop was doing:
//! a motion callback to pump the console mid-move, sliced sleeps so boot waits
//! could answer, a second reduced dispatch path, and a set of rules for which
//! commands were allowed to run while the tower was moving. A homing search runs
//! for the better part of an hour, and for all of it the board could answer
//! almost nothing — at exactly the moment somebody is stood next to it.
//!
//! Here the move belongs to the Motion machine, on a different thread. Nothing
//! this machine does is blocked by it, so none of that machinery is needed.
//!
//! # What it owns, and what it asks for
//!
//! The rule is that a diagnostic is answered by whoever owns the hardware it
//! touches. This machine owns the transport, the protocol and the sequencing —
//! not the devices. It answers directly for anything needing nothing at all, and
//! for single self-contained bus transactions; anything else is a request to the
//! machine that holds the peripheral. [`board_diagnostics::CommandNeeds`] is the
//! classification, and it is an exhaustive match, so a command added later cannot
//! quietly go unrouted.

use board_diagnostics::ConfigStaging;
use esp_idf_hal::i2c::I2cDriver;
use esp_idf_svc::nvs::{EspDefaultNvsPartition, EspNvs, NvsDefault};
use serial_console::{SerialLineRuntime, UsbSerialJtagConsole};
use shared_bus::BusManager;

use crate::hardware::sensors::SharedSensors;

pub mod awaiting;
pub mod config;
pub mod delegate;
pub mod idle;
pub mod local;
pub mod serve;

pub use idle::DiagIdle;

/// Capabilities this firmware reports to `GET_CAPABILITIES`.
///
/// Grown as commands land, not written ahead of them. A capability list that
/// promises more than the board answers is worse than a short one: it sends a
/// host into a timeout instead of an immediate, explicable failure.
pub const CAPABILITIES: &[&str] = &[
    "SERIAL", "STATUS", "PING", "I2C_SCAN", "REBOOT", "OLED", "HDC1080", "RTC", "LED_TEST",
    "CONFIG",
];

pub fn capabilities_csv() -> String {
    CAPABILITIES.join(",")
}

/// The shared I2C bus, spelled once because the full type is unreadable inline.
pub type SharedI2cBus = &'static BusManager<std::sync::Mutex<I2cDriver<'static>>>;

pub struct DiagnosticsContext {
    /// The USB Serial/JTAG console. Nothing else in the firmware writes protocol
    /// to it — though the ESP-IDF logger shares the wire, which is why a host
    /// resolves a command only on its terminal `RESULT` line and ignores
    /// everything else it sees.
    console: UsbSerialJtagConsole,
    /// Reassembles bytes into lines across polls.
    lines: SerialLineRuntime,
    /// Locked per transaction by the bus manager, so single addressed writes are
    /// safe from this thread. Multi-transaction device sequences are not, and are
    /// routed to their owner instead — see [`board_diagnostics::CommandNeeds`].
    bus: SharedI2cBus,
    /// The HDC1080 and DS3231, shared with the network machine under one lock.
    /// See [`crate::hardware::sensors`] for why these two need an owner and the
    /// rest of the bus does not.
    sensors: SharedSensors,
    /// Next correlation ticket for a delegated command. Wraps; the only property
    /// that matters is that consecutive commands do not share a value.
    next_ticket: u32,
    /// This machine's own handle on the `storage` namespace.
    ///
    /// Its own, like every other machine's. ESP-IDF's NVS is internally locked, so
    /// concurrent handles are safe; what keeps them *correct* is that the write
    /// sets are disjoint — boot seeds only while the tower is unprovisioned,
    /// motion writes only snapshot keys, and this machine writes only
    /// `CONFIG_KEYS`.
    nvs: EspNvs<NvsDefault>,
    /// Values held by `SET_ENV` and not yet committed by `SAVE_CONFIG`.
    staging: ConfigStaging,
}

impl DiagnosticsContext {
    /// Claim the USB Serial/JTAG peripheral.
    ///
    /// Call this **first**, before any other setup. The driver install itself
    /// depends on nothing, and claiming it early is what makes the console
    /// available while the rest of boot is still happening — which is the whole
    /// point of a channel you reach for when something is wrong.
    ///
    /// Receive is 1 KB: provisioning sends a run of `SET_ENV` lines back to back,
    /// and a host that does not wait for each reply can have several in flight, so
    /// it has to hold more than one command.
    ///
    /// Transmit is 4 KB, and larger for a reason. Once the console is routed
    /// through this driver, every ESP-IDF log line from every FSM thread shares
    /// the buffer with protocol output. A `GET_CONFIG` dump alongside a burst of
    /// motion logging would crowd 1 KB, and a protocol line that cannot fit is a
    /// line the host never sees.
    ///
    /// # Returns `None` rather than an error
    ///
    /// Deliberately. If claiming the console fails, the tower should still track
    /// the sun. An earlier version propagated the failure out of `main`, which
    /// meant the *diagnostics* channel failing to start took the whole firmware
    /// with it — no motion, no telemetry, and no way to find out why, because the
    /// thing that would have told you is the thing that failed. A diagnostic must
    /// never be able to do more damage than the fault it reports.
    pub fn claim_console() -> Option<UsbSerialJtagConsole> {
        match UsbSerialJtagConsole::install_driver(1024, 4096) {
            Ok(()) => {
                log::info!("Diagnostics console: USB Serial/JTAG driver installed");
                let mut console = UsbSerialJtagConsole::new();

                // One line, once, so anyone watching the port sees the console
                // come up rather than having to send something to find out.
                // Shaped as a `PHASE` announcement precisely because no host
                // resolves a pending command on one — an unprompted line must
                // never be mistakable for a response.
                let _ = console.write_line(&board_diagnostics::phase_line("diagnostics ready"));
                Some(console)
            }
            Err(err) => {
                log::error!(
                    "Diagnostics console unavailable: usb_serial_jtag_driver_install failed ({err}). \
                     The rest of the firmware continues; serial diagnostics will not answer."
                );
                None
            }
        }
    }

    /// Build the machine's context around an already-claimed console.
    ///
    /// Split from [`claim_console`](Self::claim_console) because the two happen at
    /// different times: the console is claimed before anything else, the I2C bus
    /// only exists once the peripherals have been taken.
    pub fn new(
        console: UsbSerialJtagConsole,
        bus: SharedI2cBus,
        sensors: SharedSensors,
        partition: EspDefaultNvsPartition,
    ) -> anyhow::Result<Self> {
        let nvs = EspNvs::new(partition, "storage", true).map_err(|e| {
            anyhow::anyhow!("diagnostics could not open the storage namespace: {e}")
        })?;

        Ok(Self {
            console,
            lines: SerialLineRuntime::new(),
            bus,
            sensors,
            next_ticket: 1,
            nvs,
            staging: ConfigStaging::new(),
        })
    }

    /// Put one line on the wire.
    pub fn write_line(&mut self, line: &str) {
        if self.console.write_line(line).is_err() {
            log::error!("Diagnostics TX failed on: {line}");
        }
    }

    /// Write a command's terminal `RESULT`.
    pub fn finish(
        &mut self,
        command: &str,
        passed: bool,
        details: &[(String, String)],
        message: &str,
    ) {
        serve::finish(&mut self.console, command, passed, details, message);
    }
}

/// Writes protocol output to the console as it is produced.
///
/// Streaming rather than collecting, because some commands report progress and a
/// progress line that arrives after the command finished is not progress.
///
/// `wrote_any` covers the one thing streaming cannot see in hindsight: a handler
/// that produced no output at all. Left unnoticed, that reaches the host as a
/// lone `CMD_RECEIVED` and reads exactly like a board that has hung.
pub struct ConsoleIo<'a> {
    console: &'a mut UsbSerialJtagConsole,
    pub wrote_any: bool,
}

impl<'a> ConsoleIo<'a> {
    pub fn new(console: &'a mut UsbSerialJtagConsole) -> Self {
        Self {
            console,
            wrote_any: false,
        }
    }
}

impl board_diagnostics::DiagnosticIo for ConsoleIo<'_> {
    fn write_line(&mut self, msg: &str) -> Result<(), ()> {
        self.wrote_any = true;
        self.console.write_line(msg)
    }
}
