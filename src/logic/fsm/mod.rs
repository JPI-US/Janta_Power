//! Generic thread-safe finite state machine.
//! # Example
//!
//! The following example creates a finite state machine, runs an
//! initialization state synchronously, then starts two independent FSMs
//! (LED control and maintenance) on their own threads while a coordinator
//! forwards commands between them.
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
//!     led::{LEDContext, LEDHold},
//!     maintenance::{MaintenanceContext, MaintenanceEnter},
//! };
//!
//! # fn example() -> Result<()> {
//! // Coordinator event channel.
//! let (coordinator_tx, coordinator_rx) = mpsc::channel();
//!
//! // Run the startup FSM to initialize hardware.
//! let startup_tx = coordinator_tx.clone();
//! let (_startup_cmd_tx, startup_cmd_rx) = mpsc::channel();
//!
//! let mut startup = Fsm::new(
//!     Box::new(Initialization),
//!     StartupContext::new(),
//!     startup_tx,
//!     startup_cmd_rx,
//! );
//!
//! startup.run_sync()?;
//!
//! // Extract initialized resources.
//! let StartupContext {
//!     peripherals,
//!     ..
//! } = startup.into_context();
//!
//! let peripherals = peripherals.unwrap();
//!
//! // Spawn the LED FSM.
//! let led_tx = coordinator_tx.clone();
//! let (led_cmd_tx, led_cmd_rx) = mpsc::channel();
//!
//! Fsm::new(
//!     Box::new(LEDHold),
//!     LEDContext {
//!         led: peripherals.led,
//!     },
//!     led_tx,
//!     led_cmd_rx,
//! )
//! .spawn(
//!     "LED control",
//!     16_384,
//!     Duration::from_millis(100),
//! )?;
//!
//! // Spawn the maintenance FSM.
//! let maintenance_tx = coordinator_tx.clone();
//! let (maintenance_cmd_tx, maintenance_cmd_rx) = mpsc::channel();
//!
//! Fsm::new(
//!     Box::new(MaintenanceEnter),
//!     MaintenanceContext::new(peripherals.buttons),
//!     maintenance_tx,
//!     maintenance_cmd_rx,
//! )
//! .spawn(
//!     "Maintenance",
//!     16_384,
//!     Duration::from_millis(100),
//! )?;
//!
//! // Coordinator loop.
//! loop {
//!     let cmd = coordinator_rx.recv()?;
//!
//!     led_cmd_tx.send(cmd)?;
//!     maintenance_cmd_tx.send(cmd)?;
//! }
//! # }
//! ```

pub mod led;
pub mod maintenance;
pub mod startup;

use core::{option::Option::None, time::Duration};
use std::{
    sync::mpsc::{Receiver, Sender},
    thread::sleep,
};

/// Generic thread-safe finite state machine.
///
/// Specifically designed to be run under a "coordinator" that controls several FSMs and facilitates communication between them.
pub struct Fsm<Ctx, Cmd> {
    /// The current state for this particular machine. See [`State`].
    pub state: Box<dyn State<Ctx, Cmd> + Send>,
    /// The current context for this particular machine.
    ///
    /// This should contain any resources the state machine owns, as well as
    /// any other context that it needs between steps. See [`Fsm::step`].
    pub ctx: Ctx,
    ///
    /// Used to send commands up to the coordinator.
    pub tx: Sender<Cmd>,
    /// Used to receive commands from the coordinator.
    pub rx: Receiver<Cmd>,
}

/// FSM running status.
///
/// # Variants
///
/// - `Running` - FSM is running.
/// - `Stopped` - FSM is stopped.
pub enum FsmStatus {
    Running,
    Stopped,
}

impl<Ctx, Cmd> Fsm<Ctx, Cmd>
where
    Ctx: Send + 'static,
    Cmd: Send + 'static,
{
    /// Creates a new FSM.
    ///
    /// # Arguments
    ///
    /// - `initial_state` (`Box<dyn InitialState<Ctx, Cmd> + Send>`) - The state that the FSM begins in. See [`InitialState`].
    /// - `ctx` (`Ctx`) - The initial context that the state receives.
    /// - `tx` (`Sender<Cmd>`) - Used to send commands up to the coordinator.
    /// - `rx` (`Receiver<Cmd>`) - Used to receieve commands from the coordinator.
    ///
    /// # Returns
    ///
    /// - `Self` - The new FSM.
    pub fn new(
        initial_state: Box<dyn InitialState<Ctx, Cmd> + Send>,
        ctx: Ctx,
        tx: Sender<Cmd>,
        rx: Receiver<Cmd>,
    ) -> Self {
        Self {
            state: initial_state,
            ctx,
            tx,
            rx,
        }
    }

    /// Advances the FSM one step.
    ///
    /// # Arguments
    ///
    /// # Returns
    ///
    /// - `anyhow::Result<FsmStatus>` -
    /// 	- Ok(`[FsmStatus]`) if the FSM is healthy.
    /// 	- Err(_) if the FSM failed.
    /// ```
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

    /// Runs this FSM without spawning a new thread.
    ///
    /// Take care to only run this when you know a state machine won't block the thread indefinitely, or when you don't care if it does.
    ///
    /// # Returns
    ///
    /// - `anyhow::Result<()>` -
    /// 	- Ok(()) - The FSM stopped successfully.
    /// 	- Err(_) - The FSM stopped with error.
    /// ```
    pub fn run_sync(&mut self) -> anyhow::Result<()> {
        loop {
            match self.step()? {
                FsmStatus::Running => {}
                FsmStatus::Stopped => return Ok(()),
            }
        }
    }

    /// Runs this FSM, spawning a new thread.
    ///
    /// # Arguments
    ///
    /// - `name` (`impl Into<String>`) - The name of the thread, used for logging.
    /// - `stack_size` (`usize`) - The stack size (in bytes) for the thread.
    /// - `period` (`Duration`) - The time to sleep between steps.
    ///
    /// # Returns
    ///
    /// - `std::io::Result<()>` - Describe the return value.
    ///
    /// # Errors
    ///
    /// Describe possible errors.
    pub fn run(
        mut self,
        name: impl Into<String>,
        stack_size: usize,
        period: Duration,
    ) -> std::io::Result<()> {
        let name = name.into();
        std::thread::Builder::new()
            .name(name.clone())
            .stack_size(stack_size)
            .spawn(move || {
                log::info!("Thread \"{name}\" started");
                loop {
                    match self.step() {
                        Ok(opt) => match opt {
                            FsmStatus::Running => {}
                            FsmStatus::Stopped => {
                                break;
                            }
                        },
                        Err(e) => {
                            log::error!("FSM thread exited: {e:?}");
                            break;
                        }
                    }
                    sleep(period)
                }
            })?;

        Ok(())
    }

    /// Turns this FSM into its inner context.
    ///
    /// # Returns
    ///
    /// - `Ctx` - The FSM context.
    pub fn into_context(self) -> Ctx {
        self.ctx
    }
}

/// The state that a specific FSM begins in,
pub trait InitialState<Ctx, Cmd>: State<Ctx, Cmd> {}

/// An FSM state.
pub trait State<Ctx, Cmd> {
    /// Processes this state, yielding the next state.
    ///
    /// # Arguments
    ///
    /// - `ctx` (`&mut Ctx`) - The current FSM context.
    /// - `tx` (`&mut Sender<Cmd>`) - Used to send commands up to the coordinator.
    /// - `rx` (`&mut Receiver<Cmd>`) - Used to receive commands from the coordinator.
    ///
    /// # Returns
    ///
    /// - `anyhow::Result<Option<Box<dyn State<Ctx, Cmd> + Send>>>` -
    /// 	- Ok(State) - The next state
    /// 	- Err(_) - If the FSM encountered an issue.
    fn process(
        &mut self,
        ctx: &mut Ctx,
        tx: &mut Sender<Cmd>,
        rx: &mut Receiver<Cmd>,
    ) -> anyhow::Result<Option<Box<dyn State<Ctx, Cmd> + Send>>>;
}
