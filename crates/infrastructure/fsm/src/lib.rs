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

use crossbeam_channel::{Receiver, Sender};

use crate::{
    channel::Channel,
    state::{InitialState, State, StateResult},
};

pub mod channel;
pub mod state;

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
