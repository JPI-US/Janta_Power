use anyhow::anyhow;
use board_diagnostics::{ConfigStaging, DiagnosticCommand, DiagnosticIo};
use esp_idf_svc::{hal::i2c::I2cDriver, nvs::{EspNvs, NvsPartitionId}};

use serial_console::{SerialLinePoll, SerialLineRuntime, UsbSerialJtagConsole};

use crate::diagnostics::{executor, mqtt};

/// Outcome of one serial poll.
///
/// Mirrors `board_diagnostics::DiagnosticPoll`. `RebootRequested` exists because
/// the main loop is the only place allowed to reset the board — this frontend
/// reports the request and lets the caller decide when to act on it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SerialDiagnosticsPoll {
    Idle,
    CommandProcessed,
    RebootRequested,
}

pub struct SerialDiagnosticsRuntime {
    console: UsbSerialJtagConsole,
    runtime: SerialLineRuntime,
    last_phase: Option<String>,
}

/// Writes diagnostics output straight to the console as it is produced.
///
/// The alternative — collecting into a `Vec` and flushing when the command
/// returns — means `go_home`'s `MOVING_HOME` arrives after homing has already
/// finished, which defeats the point of emitting it. `wrote_any` covers the one
/// case the streaming version cannot see in hindsight: a handler that produced no
/// output at all, where the caller must fall back to the summary message.
struct ConsoleIo<'a> {
    console: &'a mut UsbSerialJtagConsole,
    wrote_any: bool,
}

impl<'a> ConsoleIo<'a> {
    fn new(console: &'a mut UsbSerialJtagConsole) -> Self {
        Self {
            console,
            wrote_any: false,
        }
    }
}

impl DiagnosticIo for ConsoleIo<'_> {
    fn write_line(&mut self, msg: &str) -> Result<(), ()> {
        self.wrote_any = true;
        self.console.write_line(msg)
    }
}

impl SerialDiagnosticsRuntime {
    pub fn new() -> anyhow::Result<Self> {
        UsbSerialJtagConsole::install_driver(1024, 1024)
            .map_err(|err| anyhow!("usb_serial_jtag_driver_install failed: {}", err))?;
        Ok(Self {
            console: UsbSerialJtagConsole::new(),
            runtime: SerialLineRuntime::new(),
            last_phase: None,
        })
    }

    /// Announce what the tower is doing, unprompted.
    ///
    /// The host cannot otherwise distinguish a board that is busy associating Wi-Fi
    /// from one that has hung — both look like silence. Paired with the
    /// `ERROR BUSY <phase>` refusals, this lets the installer show the reason
    /// before the technician even sends anything.
    ///
    /// Repeating the current phase is a no-op, so callers inside a retry loop can
    /// call it on every pass without flooding the link.
    pub fn announce_phase(&mut self, phase: &str) {
        if self.last_phase.as_deref() == Some(phase) {
            return;
        }
        let _ = self
            .console
            .write_line(&board_diagnostics::phase_line(phase));
        self.last_phase = Some(phase.to_string());
    }

    /// Answer everything a move does not actually prevent.
    ///
    /// A move holds the motor. It does not hold the I2C bus, the status LED, or —
    /// during a homing search — NVS, so every command that needs only those is
    /// executed here for real, through the same
    /// [`executor::execute_first_wave_command`] the idle path uses. That matters
    /// because a homing sweep can run for tens of minutes, and it runs at exactly
    /// the moment a technician is stood beside the tower wanting to scan the bus
    /// or read a sensor.
    ///
    /// Only [`board_diagnostics::requires_exclusive_runtime`] commands are
    /// refused, with the same `ERROR BUSY <phase>` a boot wait sends, so nothing
    /// on the wire changes for a host — the set of commands that get that answer
    /// simply shrinks.
    ///
    /// No control-command reservation is taken, unlike [`poll`](Self::poll). The
    /// reservation stops an MQTT command and a serial command from running at
    /// once, and that cannot happen here: MQTT hands its commands to the main loop
    /// through a channel, and the main loop is the thread currently blocked inside
    /// this move. Taking it would add nothing but a way for a stale MQTT
    /// reservation to lock out the person at the tower.
    pub fn poll_minimal<T>(
        &mut self,
        phase: &str,
        firmware_version: &str,
        mut led: Option<&mut rgb_led::Led<'_>>,
        bus: &'static shared_bus::BusManagerStd<I2cDriver<'static>>,
        mut nvs: Option<&mut EspNvs<T>>,
        staging: &mut ConfigStaging,
        tolerance: board_diagnostics::MoveTolerance,
    ) -> anyhow::Result<()>
    where
        T: NvsPartitionId,
    {
        self.runtime.poll(&mut self.console, |console, line| {
            let _ = console.write_line("CMD_RECEIVED");
            let command = DiagnosticCommand::parse(line);

            if board_diagnostics::requires_exclusive_runtime(&command) {
                // Built here rather than outside the closure: this runs on every
                // motion tick, and only an actual command needs the string.
                for response in board_diagnostics::boot_phase_response(
                    &command,
                    phase,
                    firmware_version,
                    &executor::capabilities_csv(),
                ) {
                    let _ = console.write_line(&response);
                }
                return Ok::<(), anyhow::Error>(());
            }

            let mut io = ConsoleIo::new(&mut *console);
            let transcript = executor::execute_first_wave_command(
                &mut io,
                command.clone(),
                firmware_version,
                // The move has the motor. Nothing else here is withheld.
                None,
                led.as_deref_mut(),
                nvs.as_deref_mut(),
                staging,
                bus,
                tolerance.allows_long_blocking(),
            );
            let wrote_any = io.wrote_any;

            // Same fallback as the idle path: a handler that emitted nothing
            // leaves the installer with only CMD_RECEIVED, which reads as a
            // timeout.
            if !wrote_any {
                let _ = console.write_line(&transcript.message);
            }

            let _ = console.write_line(&board_diagnostics::result_line(
                executor::command_name(&command),
                transcript.status == "completed",
                &transcript.details,
                &transcript.message,
            ));

            // No reboot to propagate. `REBOOT` is refused above, precisely
            // because this closure has no way to stop the move, and it is the
            // only command that can set the flag.
            Ok::<(), anyhow::Error>(())
        })?;

        Ok(())
    }

    /// Service one serial line against whatever the runtime currently owns.
    ///
    /// `motion` is `None` during boot, before the motion stack exists. Everything
    /// that does not need it is still answered.
    pub fn poll<T>(
        &mut self,
        shared_command_state: &mqtt::SharedCommandState,
        firmware_version: &str,
        motion: Option<&mut executor::MotionContext<'_, '_>>,
        led: Option<&mut rgb_led::Led<'_>>,
        nvs: &mut EspNvs<T>,
        staging: &mut ConfigStaging,
        bus: &'static shared_bus::BusManagerStd<I2cDriver<'static>>,
    ) -> anyhow::Result<SerialDiagnosticsPoll>
    where
        T: NvsPartitionId,
    {
        let mut reboot_requested = false;
        // Reborrowed inside the closure, which runs at most once per poll.
        let mut motion = motion;
        let mut led = led;
        let poll = self.runtime.poll(&mut self.console, |console, line| {
            let _ = console.write_line("CMD_RECEIVED");
            let command = DiagnosticCommand::parse(line);

            if !executor::supports_first_wave_command(&command) {
                let name = executor::command_name(&command);
                let message = format!("{name} is not enabled in first-wave diagnostics");
                let _ = console.write_line(&format!("ERROR {message}"));
                // Terminated for a strict host. Without a RESULT the refusal is
                // invisible to one, and a command the board rejected in microseconds
                // reads as a timeout instead — the failure that says nothing.
                let _ = console.write_line(&board_diagnostics::result_line(
                    name, false, &[], &message,
                ));
                return Ok::<(), anyhow::Error>(());
            }

            let is_control = executor::is_control_command(&command);
            if is_control {
                if let Err(message) = mqtt::reserve_local_control_command(
                    shared_command_state,
                    executor::command_name(&command),
                ) {
                    let _ = console.write_line(&message);
                    // Same reason as above: a refusal a strict host cannot see is a
                    // timeout, and a timeout explains nothing.
                    let _ = console.write_line(&board_diagnostics::result_line(
                        executor::command_name(&command),
                        false,
                        &[],
                        message.trim_start_matches("ERROR ").trim(),
                    ));
                    return Ok(());
                }
            }

            let mut io = ConsoleIo::new(&mut *console);
            let transcript = executor::execute_first_wave_command(
                &mut io,
                command.clone(),
                firmware_version,
                motion.as_deref_mut(),
                led.as_deref_mut(),
                Some(&mut *nvs),
                staging,
                bus,
                // Nothing is stepping here, so nothing is lost by taking the time.
                true,
            );
            let wrote_any = io.wrote_any;

            // A handler that emitted nothing leaves the installer with only
            // CMD_RECEIVED, which reads as a timeout. Send the summary instead.
            if !wrote_any {
                let _ = console.write_line(&transcript.message);
            }

            // The terminal line, after any detail lines. Hosts that understand the
            // structured protocol resolve on this and ignore the legacy lines
            // above; older ones resolve on the legacy lines and ignore this. Both
            // dialects on the wire at once is what makes the rollout safe without
            // releasing firmware and installer in lockstep.
            let _ = console.write_line(&board_diagnostics::result_line(
                executor::command_name(&command),
                transcript.status == "completed",
                &transcript.details,
                &transcript.message,
            ));

            if is_control {
                let _ = mqtt::complete_control_command(shared_command_state, "");
            }

            // The response is on the wire; the caller reboots once it has had time
            // to drain. Release the control reservation first (above) so a reset
            // that somehow does not happen cannot wedge diagnostics.
            if transcript.reboot_requested {
                reboot_requested = true;
            }
            Ok(())
        })?;

        Ok(match poll {
            SerialLinePoll::Idle => SerialDiagnosticsPoll::Idle,
            SerialLinePoll::LineProcessed if reboot_requested => {
                SerialDiagnosticsPoll::RebootRequested
            }
            SerialLinePoll::LineProcessed => SerialDiagnosticsPoll::CommandProcessed,
        })
    }
}
