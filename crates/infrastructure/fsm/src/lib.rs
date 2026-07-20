//! Generic thread-safe finite state machine.
//!
//! # Example
//!
//! The following example creates a finite state machine, runs an
//! initialization state synchronously, then starts independent FSMs on their
//! own threads while a coordinator forwards commands between them.
//!
//! ```ignore
//! use std::sync::mpsc;
//! use core::time::Duration;
//!
//! use anyhow::Result;
//!
//! use crate::logic::fsm::{
//!     Fsm,
//!     startup::{Initialization, StartupContext},
//! };
//!
//! # fn example() -> Result<()> {
//! // Coordinator event channel.
//! let (coordinator_tx, coordinator_rx) = mpsc::channel();
//!
//! // Run the startup FSM synchronously.
//! let (_startup_cmd_tx, startup_cmd_rx) = mpsc::channel();
//!
//! let startup_ctx = Fsm::new_sync(
//!     Box::new(Initialization),
//!     StartupContext::new(),
//!     coordinator_tx.clone(),
//!     startup_cmd_rx,
//!     1,
//! )?;
//!
//! // Extract resources from the completed startup context.
//! let _peripherals = startup_ctx.peripherals.unwrap();
//!
//! // Spawn additional asynchronous FSMs using Fsm::new_async(...).
//! //
//! // The coordinator can forward commands received on `coordinator_rx` to
//! // other FSM command channels.
//!
//! let _ = coordinator_rx;
//!
//! # Ok(())
//! # }
//! ```

use core::time::Duration;
use std::thread::sleep;

use crossbeam_channel::{Receiver, Sender, TryRecvError, TrySendError};

/// Generic thread-safe finite state machine.
///
/// An FSM owns a state, persistent context, and a communication channel. It can
/// be executed synchronously using [`Fsm::new_sync`] or asynchronously on its
/// own thread using [`Fsm::new_async`].
///
/// Communication with the outside world is performed through a [`Channel`],
/// which provides non-blocking send and receive operations and optional
/// filtering of queued commands.
pub struct Fsm<Ctx, Cmd> {
    /// The current state of this FSM.
    ///
    /// The state contains the behavior for the current step and may transition
    /// the FSM to a different state by returning a new state from
    /// [`State::process`].
    pub state: Box<dyn State<Ctx, Cmd> + Send>,

    /// Persistent context owned by this FSM.
    ///
    /// The context contains resources and data that must be preserved across
    /// state transitions.
    pub ctx: Ctx,

    /// Communication channel used by states to exchange commands or events
    /// with external coordinators.
    pub channel: Channel<Cmd>,
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
pub enum StateResult<Ctx, Cmd> {
    Running(Box<dyn State<Ctx, Cmd> + Send>),
    Hold,
    Stopped,
}

/// FSM execution status.
///
/// # Variants
///
/// - `Running` - The FSM transitioned to another state.
/// - `Hold` - The FSM remains in the current state.
/// - `Stopped` - The FSM has completed execution.
pub enum FsmStatus {
    Running,
    Hold,
    Stopped,
}

impl<Ctx, Cmd> Fsm<Ctx, Cmd>
where
    Ctx: Send + 'static,
    Cmd: Send + 'static,
{
    /// Creates and runs an FSM synchronously until it stops.
    ///
    /// The initial state is executed repeatedly until a state returns
    /// `StateResult::Stopped`, indicating that the FSM should terminate.
    ///
    /// # Arguments
    ///
    /// - `initial_state` - Initial state of the FSM. Must implement
    ///   [`InitialState`].
    /// - `ctx` - Initial FSM context.
    /// - `tx` - Sender used to transmit commands or events.
    /// - `rx` - Receiver used to receive commands.
    /// - `max_pending` - Maximum number of pending received commands to retain.
    ///   Older queued commands are discarded as needed by [`Channel`].
    ///
    /// # Returns
    ///
    /// Returns the final FSM context after the FSM reaches a stopped state.
    pub fn new_sync(
        initial_state: Box<dyn InitialState<Ctx, Cmd> + Send>,
        ctx: Ctx,
        tx: Sender<Cmd>,
        rx: Receiver<Cmd>,
        max_pending: usize,
    ) -> anyhow::Result<Ctx> {
        let mut fsm = Self {
            state: initial_state,
            ctx,
            channel: Channel {
                tx,
                rx,
                max_pending,
            },
        };

        loop {
            match fsm.step()? {
                FsmStatus::Running => {}
                FsmStatus::Hold => {}
                FsmStatus::Stopped => return Ok(fsm.ctx),
            }
        }
    }

    /// Creates and starts an FSM on a dedicated thread.
    ///
    /// The FSM thread repeatedly executes [`Fsm::step`] and sleeps for the
    /// configured period between iterations. The thread exits when the FSM
    /// reaches a stopped state or when an error occurs.
    ///
    /// # Arguments
    ///
    /// - `initial_state` - Initial state of the FSM. Must implement
    ///   [`InitialState`].
    /// - `ctx` - Initial FSM context.
    /// - `tx` - Sender used to transmit commands or events.
    /// - `rx` - Receiver used to receive commands.
    /// - `max_pending` - Maximum number of pending received commands to retain.
    /// - `thread_name` - Name assigned to the FSM thread.
    /// - `thread_stack_size` - Stack size allocated for the FSM thread.
    /// - `thread_period` - Delay between successive calls to [`Fsm::step`].
    ///
    /// # Returns
    ///
    /// Returns `Ok(())` after successfully creating the FSM thread.
    pub fn new_async(
        initial_state: Box<dyn InitialState<Ctx, Cmd> + Send>,
        ctx: Ctx,
        tx: Sender<Cmd>,
        rx: Receiver<Cmd>,
        max_pending: usize,
        thread_name: impl Into<String>,
        thread_stack_size: usize,
        thread_period: Duration,
    ) -> anyhow::Result<()> {
        let mut fsm = Self {
            state: initial_state,
            ctx,
            channel: Channel {
                tx,
                rx,
                max_pending,
            },
        };

        let thread_name = thread_name.into();

        std::thread::Builder::new()
            .name(thread_name.clone())
            .stack_size(thread_stack_size)
            .spawn(move || {
                log::info!("Thread \"{thread_name}\" started");

                loop {
                    match fsm.step() {
                        Ok(FsmStatus::Running) => {}
                        Ok(FsmStatus::Hold) => {}
                        Ok(FsmStatus::Stopped) => break,
                        Err(e) => {
                            log::error!("FSM thread exited: {e:?}");
                            break;
                        }
                    }

                    sleep(thread_period);
                }
            })?;

        Ok(())
    }

    /// Advances the FSM by one execution step.
    ///
    /// The current state is given mutable access to the FSM context and its
    /// communication [`Channel`]. The state may transition to a new state,
    /// remain active, or stop the FSM.
    ///
    /// When a state transition occurs, any excess queued commands are discarded
    /// according to the channel's configured pending-command limit before the
    /// next state begins execution.
    ///
    /// # Returns
    ///
    /// - `Ok(FsmStatus::Running)` if the FSM transitioned to a new state.
    /// - `Ok(FsmStatus::Hold)` if the FSM remains in the current state.
    /// - `Ok(FsmStatus::Stopped)` if the FSM has completed execution.
    /// - `Err(_)` if the current state encountered an error.
    pub fn step(&mut self) -> anyhow::Result<FsmStatus> {
        match self.state.process(&mut self.ctx, &mut self.channel)? {
            StateResult::Running(state) => {
                self.channel.drain();
                self.state = state;
                Ok(FsmStatus::Running)
            }
            StateResult::Hold => Ok(FsmStatus::Hold),
            StateResult::Stopped => Ok(FsmStatus::Stopped),
        }
    }
}

/// The initial state of an FSM.
///
/// This marker trait identifies states that are valid starting points for an
/// FSM. Initial states must also implement [`State`].
pub trait InitialState<Ctx, Cmd>: State<Ctx, Cmd> {}

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

/// Communication channel used by an FSM.
///
/// The channel wraps a sender and receiver and provides non-blocking send and
/// receive operations.
///
/// Incoming commands can be automatically pruned so that stale queued commands
/// are discarded, allowing states to process only the most recent activity when
/// appropriate.
pub struct Channel<Cmd> {
    tx: Sender<Cmd>,
    rx: Receiver<Cmd>,
    max_pending: usize,
}

impl<Cmd> Channel<Cmd> {
    /// Creates a new communication channel.
    ///
    /// `max_pending` specifies the maximum number of queued received commands
    /// that may be retained before older commands are discarded.
    pub fn new(tx: Sender<Cmd>, rx: Receiver<Cmd>, max_pending: usize) -> Self {
        Self {
            tx,
            rx,
            max_pending,
        }
    }

    fn drain(&self) {
        while self.rx.len() > self.max_pending {
            let _ = self.rx.try_recv();
        }
    }

    fn drain_to(&self, to: usize) {
        while self.rx.len() > to {
            let _ = self.rx.try_recv();
        }
    }

    /// Receives the next pending command.
    ///
    /// Before attempting to receive, older queued commands are discarded until
    /// at most `max_pending` commands remain.
    ///
    /// Returns `TryRecvError::Empty` if no command is available.
    pub fn recv(&self) -> Result<Cmd, TryRecvError> {
        self.drain();
        self.rx.try_recv()
    }

    /// Receives only the most recently queued command.
    ///
    /// Any older queued commands are discarded before attempting to receive,
    /// ensuring that at most one pending command remains.
    ///
    /// Returns `TryRecvError::Empty` if no command is available.
    pub fn recv_latest(&self) -> Result<Cmd, TryRecvError> {
        self.drain_to(1);
        self.rx.try_recv()
    }

    /// Attempts to send a command without blocking.
    ///
    /// Returns `TrySendError` if the command cannot be queued.
    pub fn send(&self, cmd: Cmd) -> Result<(), TrySendError<Cmd>> {
        self.tx.try_send(cmd)
    }
}
