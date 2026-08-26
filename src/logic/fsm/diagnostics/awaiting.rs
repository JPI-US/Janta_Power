//! Waiting for another machine to answer, while still answering everything else.
//!
//! This state is the point of the whole design. In the single-threaded firmware
//! this replaced, a `GO_HOME` blocked the console for the length of the search —
//! up to an hour of a board that could not be asked anything, including whether
//! it was still alive. Here the search belongs to the motion machine and this one
//! carries on: it forwards progress, answers anything it owns, and refuses only a
//! second command for the same hardware.

use std::time::Instant;

use fsm::{
    postal::{bulletin::Bulletin, mailbox::Mailbox},
    state::{State, StateResult},
};

use crate::logic::fsm::{
    diagnostics::{
        delegate,
        idle::{DiagIdle, REBOOT_FLUSH_DELAY},
        serve::{self, Served},
        DiagnosticsContext,
    },
    DiagOutcome, FSMAddress, FSMCommand, FSMState,
};

pub struct DiagAwaitOwner {
    pub ticket: u32,
    /// The protocol name of the outstanding command, so the eventual `RESULT`
    /// names it and a second command can be refused with something specific.
    pub name: &'static str,
    pub owner: FSMAddress,
    pub deadline: Instant,
}

impl State<FSMAddress, DiagnosticsContext, FSMCommand, FSMState> for DiagAwaitOwner {
    fn process(
        &mut self,
        ctx: &mut DiagnosticsContext,
        mailbox: &mut Mailbox<FSMAddress, FSMCommand>,
        _bulletin: &Bulletin<FSMState>,
        _previous_state: Option<
            Box<dyn State<FSMAddress, DiagnosticsContext, FSMCommand, FSMState> + Send>,
        >,
    ) -> anyhow::Result<StateResult<FSMAddress, DiagnosticsContext, FSMCommand, FSMState>> {
        // Still listening. `PING`, `I2C_SCAN`, a sensor read — all still answered
        // while this waits.
        if let Served::Reboot = serve::poll_console(ctx, mailbox, Some(self.name))? {
            log::warn!("REBOOT requested over the diagnostics console; restarting");
            std::thread::sleep(REBOOT_FLUSH_DELAY);
            esp_idf_svc::hal::reset::restart();
        }

        while let Ok(message) = mailbox.receive() {
            let FSMCommand::DiagnosticReply(reply) = message else {
                log::warn!("Diagnostics has no handler for {message:?}; message discarded");
                continue;
            };

            if reply.ticket != self.ticket {
                // A late answer to a command that already timed out. Discarding it
                // is the entire reason tickets exist: without one it would resolve
                // whatever is in flight now, reporting one test's result as
                // another's.
                log::warn!(
                    "Discarding a stale reply for ticket {} while waiting on {}",
                    reply.ticket,
                    self.ticket
                );
                continue;
            }

            match reply.outcome {
                DiagOutcome::Progress(line) => {
                    ctx.write_line(&line);
                }
                DiagOutcome::Done {
                    passed,
                    details,
                    message,
                } => {
                    ctx.finish(self.name, passed, &details, &message);
                    return Ok(StateResult::Running(Box::new(DiagIdle)));
                }
            }
        }

        if Instant::now() >= self.deadline {
            // A machine that never answers must not be able to hold the console.
            // The host gets a real failure naming what went unanswered, which is a
            // far better thing to read than a timeout with no explanation.
            let message = format!(
                "no reply from the {} machine within its budget",
                delegate::owner_name(self.owner)
            );
            log::error!("{} timed out: {message}", self.name);
            ctx.write_line(&format!("ERROR {message}"));
            ctx.finish(self.name, false, &[], &message);
            return Ok(StateResult::Running(Box::new(DiagIdle)));
        }

        Ok(StateResult::Hold)
    }
}
