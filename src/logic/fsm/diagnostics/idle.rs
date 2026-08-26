//! Waiting for a command.
//!
//! The resting state. Anything this machine owns is answered here and now;
//! anything it does not is handed to the machine that does, and this becomes
//! [`super::awaiting::DiagAwaitOwner`] until the answer comes back.

use core::time::Duration;

use fsm::{
    postal::{bulletin::Bulletin, mailbox::Mailbox},
    state::{InitialState, State, StateResult},
};

use crate::logic::fsm::{
    diagnostics::{
        awaiting::DiagAwaitOwner,
        serve::{self, Served},
        DiagnosticsContext,
    },
    FSMAddress, FSMCommand, FSMState,
};

/// How long to let a response reach the wire before resetting.
///
/// `REBOOT` is answered and *then* acted on. Without the pause the reset races
/// the reply out of the transmit buffer, and the host sees the port drop with
/// nothing on it — indistinguishable from a board that crashed on the command.
pub const REBOOT_FLUSH_DELAY: Duration = Duration::from_millis(250);

pub struct DiagIdle;

impl InitialState<FSMAddress, DiagnosticsContext, FSMCommand, FSMState> for DiagIdle {}

impl State<FSMAddress, DiagnosticsContext, FSMCommand, FSMState> for DiagIdle {
    fn process(
        &mut self,
        ctx: &mut DiagnosticsContext,
        mailbox: &mut Mailbox<FSMAddress, FSMCommand>,
        _bulletin: &Bulletin<FSMState>,
        _previous_state: Option<
            Box<dyn State<FSMAddress, DiagnosticsContext, FSMCommand, FSMState> + Send>,
        >,
    ) -> anyhow::Result<StateResult<FSMAddress, DiagnosticsContext, FSMCommand, FSMState>> {
        // Nothing should be addressed to this machine while it is idle. Drained
        // anyway, so a reply that arrives after its command timed out is
        // discarded here rather than accumulating unseen.
        while let Ok(message) = mailbox.receive() {
            if let FSMCommand::DiagnosticReply(reply) = message {
                log::warn!(
                    "Discarding a reply for ticket {} — nothing is waiting on it",
                    reply.ticket
                );
            }
        }

        match serve::poll_console(ctx, mailbox, None)? {
            Served::Nothing => Ok(StateResult::Hold),

            Served::Reboot => {
                log::warn!("REBOOT requested over the diagnostics console; restarting");
                std::thread::sleep(REBOOT_FLUSH_DELAY);
                esp_idf_svc::hal::reset::restart();
            }

            Served::Delegated {
                ticket,
                name,
                owner,
                deadline,
            } => Ok(StateResult::Running(Box::new(DiagAwaitOwner {
                ticket,
                name,
                owner,
                deadline,
            }))),
        }
    }
}
