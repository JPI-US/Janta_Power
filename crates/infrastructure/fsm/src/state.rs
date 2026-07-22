use crate::Channel;

/// An FSM state.
///
/// Each state performs one unit of processing and may transition into another
/// state, remain active, or stop the FSM.
pub trait State<Ctx, Cmd> {
    /// Processes the current state.
    ///
    /// # Arguments
    ///
    /// - `ctx` - Mutable access to the FSM context.
    /// - `channel` - Communication channel used to send and receive commands.
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
        channel: &mut Channel<Cmd>,
    ) -> anyhow::Result<StateResult<Ctx, Cmd>>;
}

/// The initial state of an FSM.
///
/// This marker trait identifies states that are valid starting points for an
/// FSM. Initial states must also implement [`State`].
pub trait InitialState<Ctx, Cmd>: State<Ctx, Cmd> {}

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
pub enum StateResult<Ctx, Cmd> {
    Running(Box<dyn State<Ctx, Cmd> + Send>),
    Hold,
    Stopped,
}
