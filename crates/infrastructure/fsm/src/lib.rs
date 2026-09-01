//! Cooperative finite-state machines.
//!
//! An [`Fsm`] owns a current [`state::State`], per-machine context, a
//! [`postal::mailbox::Mailbox`], and a shared [`postal::bulletin::Bulletin`].
//! Machines are driven by [`Runnable::step`], either alone via [`Fsm::spawn`]
//! or together on one thread via [`group::Group`].

use core::{mem, time::Duration};
use std::{io, sync::Arc, thread::JoinHandle};

use crate::{
    group::Group,
    postal::{bulletin::Bulletin, mailbox::Mailbox, Address},
    state::{State, StateResult},
};

pub mod group;
pub mod postal;
pub mod state;

/// A single finite-state machine.
///
/// Type parameters:
///
/// * `A` - routing address type ([`Address`])
/// * `Ctx` - mutable per-machine context passed to every state
/// * `Cmd` - message type carried by the [`Mailbox`]
/// * `B` - value type stored on the shared [`Bulletin`]
pub struct Fsm<A, Ctx, Cmd, B>
where
    A: Address,
{
    /// Name used in logs and as the default thread name.
    pub name: String,
    /// Currently active state.
    pub state: Box<dyn State<A, Ctx, Cmd, B> + Send>,
    /// State displaced by the last [`StateResult::Running`] transition.
    ///
    /// Consumed on the next [`Runnable::step`] and passed to
    /// [`State::process`] as `previous_state`.
    pub previous_state: Option<Box<dyn State<A, Ctx, Cmd, B> + Send>>,
    /// Mutable context shared across this machine's states.
    pub ctx: Ctx,
    /// Point-to-point inbox/outbox for this machine.
    pub mailbox: Mailbox<A, Cmd>,
    /// Board shared with every machine created from the same [`postal::Postal`].
    pub bulletin: Arc<Bulletin<B>>,
}

/// Outcome of one [`Runnable::step`].
pub enum FsmStatus {
    /// The machine installed a new state and should keep running.
    Running,
    /// The machine stayed in its current state.
    Hold,
    /// The machine reported that it has finished.
    Stopped,
}

impl<A, Ctx, Cmd, B> Fsm<A, Ctx, Cmd, B>
where
    A: Address,
    Self: Send + 'static,
    Ctx: Send + 'static,
    Cmd: Send + 'static,
    B: Send + 'static,
{
    /// Creates an FSM in `state` with the given context and postal endpoints.
    ///
    /// # Arguments
    ///
    /// * `name` - Display name used in logs and as the default thread name.
    /// * `state` - Initial active state.
    /// * `ctx` - Mutable context shared across this machine's states.
    /// * `mailbox` - Point-to-point inbox/outbox for this machine.
    /// * `bulletin` - Shared board from the same [`postal::Postal`] as `mailbox`.
    pub fn new(
        name: impl Into<String>,
        state: Box<dyn State<A, Ctx, Cmd, B> + Send>,
        ctx: Ctx,
        mailbox: Mailbox<A, Cmd>,
        bulletin: Arc<Bulletin<B>>,
    ) -> Self {
        Self {
            name: name.into(),
            state,
            previous_state: None,
            ctx,
            mailbox,
            bulletin,
        }
    }

    /// Replaces the active state, stashing the old one in [`Self::previous_state`].
    ///
    /// # Arguments
    ///
    /// * `next` - State to install as active.
    fn transition(&mut self, next: Box<dyn State<A, Ctx, Cmd, B> + Send>) {
        self.previous_state = Some(mem::replace(&mut self.state, next));
    }

    /// Adds this machine to `group` so it can share a thread with others.
    ///
    /// # Arguments
    ///
    /// * `group` - Group that will own and step this machine.
    pub fn group(self, group: &mut Group) {
        group.add(Box::new(self));
    }

    /// Runs this machine alone on a dedicated thread.
    ///
    /// Internally builds a one-machine [`Group`] named after this FSM.
    ///
    /// # Arguments
    ///
    /// * `thread_stack_size` - Stack size in bytes for the spawned thread.
    /// * `min_thread_period` - Minimum duration of each step-loop iteration.
    ///
    /// # Errors
    ///
    /// Returns an [`io::Error`] if the OS thread cannot be spawned.
    pub fn spawn(
        self,
        thread_stack_size: usize,
        min_thread_period: Duration,
    ) -> Result<JoinHandle<()>, io::Error> {
        let mut group = Group::new(self.name.clone(), thread_stack_size, min_thread_period);

        group.add(Box::new(self));

        group.spawn()
    }
}

impl<A, Ctx, Cmd, B> Runnable for Fsm<A, Ctx, Cmd, B>
where
    A: Address,
    Ctx: Send + 'static,
    Cmd: Send + 'static,
    B: Send + 'static,
{
    fn step(&mut self) -> anyhow::Result<FsmStatus> {
        let previous_state = self.previous_state.take();

        let result = self.state.process(
            &mut self.ctx,
            &mut self.mailbox,
            &self.bulletin,
            previous_state,
        )?;

        match result {
            StateResult::Running(next) => {
                self.transition(next);
                Ok(FsmStatus::Running)
            }

            StateResult::Hold => Ok(FsmStatus::Hold),

            StateResult::Stopped => Ok(FsmStatus::Stopped),
        }
    }

    fn drain(&mut self) {
        self.mailbox.drain();
    }

    fn name(&self) -> String {
        self.name.clone()
    }

    fn state(&self) -> String {
        String::from(self.state.type_name())
    }
}

/// Something that can be stepped by a [`Group`] (typically an [`Fsm`]).
pub trait Runnable: Send {
    /// Advances the machine by one state-processing step.
    ///
    /// # Errors
    ///
    /// Propagates any error returned by the active state's [`State::process`].
    fn step(&mut self) -> anyhow::Result<FsmStatus>;

    /// Drops excess mailbox messages down to the configured capacity.
    fn drain(&mut self);

    /// Returns the machine's display name.
    fn name(&self) -> String;

    /// Returns a display name for the active state.
    fn state(&self) -> String;
}
