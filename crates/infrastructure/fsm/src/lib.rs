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

pub struct Fsm<A, Ctx, Cmd, B>
where
    A: Address,
{
    pub name: String,
    pub state: Box<dyn State<A, Ctx, Cmd, B> + Send>,
    pub previous_state: Option<Box<dyn State<A, Ctx, Cmd, B> + Send>>,
    pub ctx: Ctx,
    pub mailbox: Mailbox<A, Cmd>,
    pub bulletin: Arc<Bulletin<B>>,
}

pub enum FsmStatus {
    Running,
    Hold,
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

    fn transition(&mut self, next: Box<dyn State<A, Ctx, Cmd, B> + Send>) {
        self.previous_state = Some(mem::replace(&mut self.state, next));
    }

    pub fn group(self, group: &mut Group) {
        group.add(Box::new(self));
    }

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

pub trait Runnable: Send {
    fn step(&mut self) -> anyhow::Result<FsmStatus>;
    fn drain(&mut self);
    fn name(&self) -> String;
    fn state(&self) -> String;
}
