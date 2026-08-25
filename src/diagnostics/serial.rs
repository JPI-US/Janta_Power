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

    /// Answer what can be served while a move is in progress.
    ///
    /// The move holds the motor and the caller holds NVS, so almost nothing is
    /// lendable — but the status LED is. `LED_TEST` therefore works while the tower
    /// is homing, which is exactly when someone is stood next to it watching. Boot
    /// waits use the full [`poll`](Self::poll) instead, since by then the bus and
    /// NVS exist too.
    pub fn poll_minimal(
        &mut self,
        phase: &str,
        firmware_version: &str,
        mut led: Option<&mut rgb_led::Led<'_>>,
    ) -> anyhow::Result<()> {
        self.runtime.poll(&mut self.console, |console, line| {
            let _ = console.write_line("CMD_RECEIVED");
            let command = DiagnosticCommand::parse(line);

            if let (DiagnosticCommand::LedTest { color }, Some(led)) =
                (&command, led.as_deref_mut())
            {
                let mut io = ConsoleIo::new(&mut *console);
                let outcome = executor::drive_status_led(led, color, &mut io);
                let wrote_any = io.wrote_any;
                if !wrote_any {
                    let _ = console.write_line("LED_TEST produced no output");
                }
                let _ = console.write_line(&match &outcome {
                    Ok(details) => board_diagnostics::result_line("LED_TEST", true, details, ""),
                    Err(message) => board_diagnostics::result_line("LED_TEST", false, &[], message),
                });
                return Ok::<(), anyhow::Error>(());
            }

            // Built here rather than outside the closure: this runs every 50 ms
            // through boot, and only an actual command needs the string.
            for response in board_diagnostics::boot_phase_response(
                &command,
                phase,
                firmware_version,
                &executor::capabilities_csv(),
            ) {
                let _ = console.write_line(&response);
            }
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
                nvs,
                staging,
                bus,
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
