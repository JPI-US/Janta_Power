//! Reading the console and answering what arrives.
//!
//! Shared by every state of this machine, because every state has to keep
//! answering. A machine that stops listening while it waits for something else is
//! the behaviour this whole design exists to remove — in the firmware it replaced,
//! a `GO_HOME` took the console with it for the length of the search.

use std::time::Instant;

use board_diagnostics::{ConfigStaging, DiagnosticCommand};
use esp_idf_svc::nvs::{EspNvs, NvsDefault};
use fsm::postal::mailbox::Mailbox;
use serial_console::UsbSerialJtagConsole;

use crate::{
    hardware::sensors::SharedSensors,
    logic::fsm::{
        diagnostics::{delegate, local, ConsoleIo, DiagnosticsContext, SharedI2cBus},
        DiagRequest, FSMAddress, FSMCommand,
    },
};

/// What answering a batch of console input asked the machine to do next.
pub enum Served {
    /// Everything was answered here and now.
    Nothing,
    /// `REBOOT` was accepted. The caller resets once the reply has cleared.
    Reboot,
    /// A command went to another machine, and its answer is now outstanding.
    Delegated {
        ticket: u32,
        name: &'static str,
        owner: FSMAddress,
        deadline: Instant,
    },
}

/// Poll the console once and answer whatever complete lines arrived.
///
/// `pending` names the delegated command already in flight, if there is one. A
/// second is refused rather than queued: the machines on the other end own single
/// pieces of hardware, and two commands for one motor is not a thing to resolve
/// by ordering them.
pub fn poll_console(
    ctx: &mut DiagnosticsContext,
    mailbox: &Mailbox<FSMAddress, FSMCommand>,
    pending: Option<&'static str>,
) -> anyhow::Result<Served> {
    // Destructured so the console and the line reassembler are two disjoint field
    // borrows rather than one borrow of the whole context — the closure needs the
    // console while `lines` is running.
    let DiagnosticsContext {
        console,
        lines,
        bus,
        sensors,
        next_ticket,
        nvs,
        staging,
    } = ctx;
    let bus: SharedI2cBus = *bus;
    let sensors = &*sensors;

    let mut outcome = Served::Nothing;

    lines.poll(console, |console, line| {
        // A delegation started earlier in *this same batch* counts as pending.
        // Two commands can arrive in one poll, and the second must see the first.
        let pending = match &outcome {
            Served::Delegated { name, .. } => Some(*name),
            _ => pending,
        };

        let served = serve(
            console,
            bus,
            sensors,
            nvs,
            staging,
            mailbox,
            next_ticket,
            pending,
            line,
        );

        // First non-trivial outcome wins. Anything after a delegation is refused
        // and returns `Nothing`, so this cannot discard one.
        if matches!(outcome, Served::Nothing) {
            outcome = served;
        }

        Ok::<(), anyhow::Error>(())
    })?;

    Ok(outcome)
}

/// Answer one command line.
///
/// Every managed command produces exactly one terminal `RESULT`, including every
/// refusal. That is the whole contract with the host: it resolves on `RESULT` and
/// nothing else, so a command ending without one is indistinguishable from a
/// board that stopped responding — and a timeout explains nothing.
#[allow(clippy::too_many_arguments)]
fn serve(
    console: &mut UsbSerialJtagConsole,
    bus: SharedI2cBus,
    sensors: &SharedSensors,
    nvs: &mut EspNvs<NvsDefault>,
    staging: &mut ConfigStaging,
    mailbox: &Mailbox<FSMAddress, FSMCommand>,
    next_ticket: &mut u32,
    pending: Option<&'static str>,
    line: &str,
) -> Served {
    // Protocol first, log second, and the order is load-bearing.
    //
    // The ESP-IDF console shares this peripheral with the protocol, and writes
    // through the driver a byte at a time, taking and releasing the transmit
    // mutex for each one. A log line is therefore a hundred-odd lock acquisitions
    // in front of whatever is written next. Logging before the acknowledgement
    // put all of that ahead of the one line the host is waiting on.
    let ack = console.write_line("CMD_RECEIVED");

    // Now the log. Worth its cost: it is the single observation separating "the
    // board never received it" from "the board answered and the reply was lost",
    // which look identical from the far end and have different causes.
    log::info!("Diagnostics RX: {line}");

    if ack.is_err() {
        log::error!("Diagnostics TX failed on CMD_RECEIVED; the console is not being read");
    }

    let command = DiagnosticCommand::parse(line);

    if local::is_local(&command) {
        let mut io = ConsoleIo::new(console);
        let answer = local::answer(&command, bus, sensors, nvs, staging, &mut io);
        let wrote_any = io.wrote_any;

        // A handler that emitted nothing leaves the host holding only
        // `CMD_RECEIVED`, which reads exactly like a board that has hung.
        if !wrote_any {
            let _ = console.write_line(&answer.message);
        }

        finish(
            console,
            command.name(),
            answer.passed,
            &answer.details,
            &answer.message,
        );

        return if answer.reboot_requested {
            Served::Reboot
        } else {
            Served::Nothing
        };
    }

    let Some(owner) = delegate::owner(&command) else {
        // Two different situations, and they deserve different words. One is
        // worth retrying after a firmware update; the other never will be.
        let message = match board_diagnostics::command_needs(&command) {
            board_diagnostics::CommandNeeds::Unsupported => {
                format!("{} is not enabled in this firmware", command.name())
            }
            _ => format!("{} is not answerable by this firmware yet", command.name()),
        };
        let _ = console.write_line(&format!("ERROR {message}"));
        finish(console, command.name(), false, &[], &message);
        return Served::Nothing;
    };

    if let Some(busy_with) = pending {
        let message = format!(
            "{} refused: {busy_with} is still running, and one hardware command at a time is the rule",
            command.name()
        );
        let _ = console.write_line(&format!("ERROR {message}"));
        finish(console, command.name(), false, &[], &message);
        return Served::Nothing;
    }

    let ticket = *next_ticket;
    *next_ticket = next_ticket.wrapping_add(1);

    let request = FSMCommand::DiagnosticRequest(DiagRequest {
        ticket,
        command: command.clone(),
    });

    if mailbox.send(owner, request).is_err() {
        // The channel is unbounded, so this means the machine on the other end is
        // gone rather than busy — worth saying plainly, because it is a different
        // fault from one that never replies.
        let message = format!(
            "{} could not be delivered: the {} machine is not running",
            command.name(),
            delegate::owner_name(owner)
        );
        let _ = console.write_line(&format!("ERROR {message}"));
        finish(console, command.name(), false, &[], &message);
        return Served::Nothing;
    }

    Served::Delegated {
        ticket,
        name: command.name(),
        owner,
        deadline: Instant::now() + delegate::reply_budget(&command),
    }
}

/// Write the terminal `RESULT` line, and say so if it did not get out.
pub fn finish(
    console: &mut UsbSerialJtagConsole,
    command: &str,
    passed: bool,
    details: &[(String, String)],
    message: &str,
) {
    let result = board_diagnostics::result_line(command, passed, details, message);

    // Only failure is logged. Echoing every successful `RESULT` doubles traffic on
    // a link the log already shares, to repeat a line that just went out on it.
    if console.write_line(&result).is_err() {
        log::error!("Diagnostics TX failed on: {result}");
    }
}
