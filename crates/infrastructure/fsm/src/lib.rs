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
//! # }
//! ```

use core::{option::Option::None, time::Duration};
use std::{
    sync::mpsc::{Receiver, Sender},
    thread::sleep,
};

/// Generic thread-safe finite state machine.
///
/// An FSM owns a state, a context, and communication channels. It can be
/// executed synchronously using [`Fsm::new_sync`] or asynchronously on its own
/// thread using [`Fsm::new_async`].
///
/// FSM instances are intended to be coordinated by a higher-level component
/// which sends commands to FSMs and receives events or status updates from
/// them.
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

    /// Sender used by the FSM to communicate with the coordinator.
    pub tx: Sender<Cmd>,

    /// Receiver used by the FSM to receive commands from the coordinator.
    pub rx: Receiver<Cmd>,
}

/// FSM running status.
///
/// # Variants
///
/// - `Running` - The FSM transitioned to another state and should continue
///   processing.
/// - `Stopped` - The FSM has completed execution because the current state did
///   not provide a next state.
pub enum FsmStatus {
    Running,
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
    /// `None`, indicating that no next state exists.
    ///
    /// # Arguments
    ///
    /// - `initial_state` - The initial state of the FSM. Must implement
    ///   [`InitialState`].
    /// - `ctx` - The initial FSM context.
    /// - `tx` - Sender used by states to communicate with the coordinator.
    /// - `rx` - Receiver used by states to receive coordinator commands.
    ///
    /// # Returns
    ///
    /// Returns the final FSM context after the FSM reaches a stopped state.
    pub fn new_sync(
        initial_state: Box<dyn InitialState<Ctx, Cmd> + Send>,
        ctx: Ctx,
        tx: Sender<Cmd>,
        rx: Receiver<Cmd>,
    ) -> anyhow::Result<Ctx> {
        let mut fsm = Self {
            state: initial_state,
            ctx,
            tx,
            rx,
        };

        loop {
            match fsm.step()? {
                FsmStatus::Running => {}
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
    /// - `initial_state` - The initial state of the FSM. Must implement
    ///   [`InitialState`].
    /// - `ctx` - The initial FSM context.
    /// - `tx` - Sender used by states to communicate with the coordinator.
    /// - `rx` - Receiver used by states to receive coordinator commands.
    /// - `thread_name` - Name assigned to the FSM thread.
    /// - `thread_stack_size` - Stack size allocated for the FSM thread.
    /// - `thread_period` - Delay between FSM execution steps.
    ///
    /// # Returns
    ///
    /// Returns `Ok(())` after successfully creating the FSM thread.
    pub fn new_async(
        initial_state: Box<dyn InitialState<Ctx, Cmd> + Send>,
        ctx: Ctx,
        tx: Sender<Cmd>,
        rx: Receiver<Cmd>,
        thread_name: impl Into<String>,
        thread_stack_size: usize,
        thread_period: Duration,
    ) -> anyhow::Result<()> {
        let mut fsm = Self {
            state: initial_state,
            ctx,
            tx,
            rx,
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
                        Ok(FsmStatus::Stopped) => {
                            break;
                        }
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
    /// The current state is given mutable access to the context and
    /// communication channels. It may return a new state to transition to, or
    /// `None` to stop the FSM.
    ///
    /// # Returns
    ///
    /// - `Ok(FsmStatus::Running)` if the FSM transitioned to another state.
    /// - `Ok(FsmStatus::Stopped)` if the FSM has completed execution.
    /// - `Err(_)` if the current state encountered an error.
    pub fn step(&mut self) -> anyhow::Result<FsmStatus> {
        let opt = self
            .state
            .process(&mut self.ctx, &mut self.tx, &mut self.rx)?;

        match opt {
            Some(state) => {
                self.state = state;
                Ok(FsmStatus::Running)
            }
            None => Ok(FsmStatus::Stopped),
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
/// Each state performs one unit of processing and optionally returns the next
/// state to transition into.
pub trait State<Ctx, Cmd> {
    /// Processes the current state.
    ///
    /// # Arguments
    ///
    /// - `ctx` - Mutable access to the FSM context.
    /// - `tx` - Sender used to communicate with the coordinator.
    /// - `rx` - Receiver used to receive coordinator commands.
    ///
    /// # Returns
    ///
    /// - `Ok(Some(state))` - Transition to the returned next state.
    /// - `Ok(None)` - Stop the FSM.
    /// - `Err(_)` - The state encountered an error.
    fn process(
        &mut self,
        ctx: &mut Ctx,
        tx: &mut Sender<Cmd>,
        rx: &mut Receiver<Cmd>,
    ) -> anyhow::Result<Option<Box<dyn State<Ctx, Cmd> + Send>>>;
}

/// Drains all currently available messages from a receiver and returns the
/// most recent one.
///
/// This is useful for command channels where only the latest command matters
/// and older pending commands can be discarded.
///
/// # Arguments
///
/// - `rx` - The receiver to drain.
///
/// # Returns
///
/// - `Some(Cmd)` - The latest received command, if any commands were available.
/// - `None` - No commands were available.
pub fn drain_rx<Cmd>(rx: &mut Receiver<Cmd>) -> Option<Cmd> {
    let mut latest = None;

    loop {
        match rx.try_recv() {
            Ok(cmd) => latest = Some(cmd),
            _ => break,
        }
    }

    latest
}
