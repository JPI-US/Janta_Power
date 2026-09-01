//! State trait, boot marker, and per-step results for an [`crate::Fsm`].

use crate::postal::{bulletin::Bulletin, mailbox::Mailbox, Address};

/// One unit of FSM behavior.
///
/// Each call to [`State::process`] may transition to another state, remain in
/// the current one, or stop the machine.
pub trait State<A, Ctx, Cmd, B>: Send
where
    A: Address,
{
    /// Runs one processing step for this state.
    ///
    /// # Arguments
    ///
    /// * `ctx` - Mutable access to the FSM context.
    /// * `mailbox` - Mailbox used to send and receive messages.
    /// * `bulletin` - Shared bulletin available to all FSM instances.
    /// * `previous_state` - State displaced by the most recent
    ///   [`StateResult::Running`] transition. Only `Some` on the first
    ///   `process` call after that transition (taken from the FSM and not
    ///   restored on [`StateResult::Hold`]). Short "move then return" states
    ///   use this to resume the caller.
    ///
    /// # Returns
    ///
    /// * [`StateResult::Running`] - Transition to the returned next state.
    /// * [`StateResult::Hold`] - Remain in the current state.
    /// * [`StateResult::Stopped`] - Stop the FSM.
    ///
    /// # Errors
    ///
    /// Returns an error if processing fails; the FSM propagates it from
    /// [`crate::Runnable::step`].
    fn process(
        &mut self,
        ctx: &mut Ctx,
        mailbox: &mut Mailbox<A, Cmd>,
        bulletin: &Bulletin<B>,
        previous_state: Option<Box<dyn State<A, Ctx, Cmd, B> + Send>>,
    ) -> anyhow::Result<StateResult<A, Ctx, Cmd, B>>;

    /// Stable type name used for logging transitions.
    fn type_name(&self) -> &'static str {
        core::any::type_name::<Self>()
    }
}

/// Marker for states that are valid FSM entry points.
///
/// Implement this on boot states in addition to [`State`]. Construction via
/// [`crate::Fsm::new`] does not require the marker; it documents intent.
pub trait InitialState<A, Ctx, Cmd, B>: State<A, Ctx, Cmd, B>
where
    A: Address,
{
}

/// Result of one [`State::process`] call.
pub enum StateResult<A, Ctx, Cmd, B>
where
    A: Address,
{
    /// Install the boxed state as active and keep running.
    ///
    /// The previous active state is stored on the FSM and handed to the next
    /// state's first [`State::process`] as `previous_state`.
    Running(Box<dyn State<A, Ctx, Cmd, B> + Send>),
    /// Stay in the current state object for the next step.
    Hold,
    /// Signal that the machine has finished.
    Stopped,
}
