use crate::postal::{mailbox::Mailbox, Address};

/// An FSM state.
///
/// Each state performs one unit of processing and may transition into another
/// state, remain active, or stop the FSM.
pub trait State<A, Ctx, Cmd>
where
    A: Address,
{
    /// Processes the current state.
    ///
    /// # Arguments
    ///
    /// - `ctx` - Mutable access to the FSM context.
    /// - `mailbox` - Mailbox used to send and receive messages.
    ///
    /// # Returns
    ///
    /// - `StateResult::Running(state)` - Transition to the returned next state.
    /// - `StateResult::Hold` - Remain in the current state.
    /// - `StateResult::Stopped` - Stop the FSM.
    /// - `Err(_)` - The state encountered an error.
    fn process(
        &mut self,
        ctx: &mut Ctx,
        mailbox: &mut Mailbox<A, Cmd>,
    ) -> anyhow::Result<StateResult<A, Ctx, Cmd>>;
}

/// The initial state of an FSM.
///
/// This marker trait identifies states that are valid starting points for an
/// FSM. Initial states must also implement [`State`].
pub trait InitialState<A, Ctx, Cmd>: State<A, Ctx, Cmd>
where
    A: Address,
{
}

/// Result of processing an FSM state.
///
/// # Variants
///
/// - `Running` - The FSM transitioned to another state and should continue
///   processing using the returned state.
/// - `Hold` - The FSM did not transition states and should continue
///   processing the current state.
/// - `Stopped` - The FSM has completed execution because the current state did
///   not provide further processing.
pub enum StateResult<A, Ctx, Cmd>
where
    A: Address,
{
    Running(Box<dyn State<A, Ctx, Cmd> + Send>),
    Hold,
    Stopped,
}
