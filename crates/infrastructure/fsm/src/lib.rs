use core::time::Duration;
use std::{io, thread::JoinHandle};

use crate::{
    group::Group,
    postal::{Address, Mailbox},
    state::{State, StateResult},
};

pub mod group;
pub mod postal;
pub mod state;

pub struct Fsm<A, Ctx, Cmd>
where
    A: Address,
{
    pub state: Box<dyn State<A, Ctx, Cmd> + Send>,
    pub ctx: Ctx,
    pub mailbox: Mailbox<A, Cmd>,
}

pub enum FsmStatus {
    Running,
    Hold,
    Stopped,
}

impl<A, Ctx, Cmd> Fsm<A, Ctx, Cmd>
where
    A: Address,
    Self: Send + 'static,
    Ctx: Send + 'static,
    Cmd: Send + 'static,
{
    pub fn new(
        state: Box<dyn State<A, Ctx, Cmd> + Send>,
        ctx: Ctx,
        mailbox: Mailbox<A, Cmd>,
    ) -> Self {
        Self {
            state,
            ctx,
            mailbox,
        }
    }

    pub fn group(self, group: &mut Group) {
        group.add(Box::new(self));
    }

    pub fn spawn(
        self,
        name: impl Into<String>,
        thread_stack_size: usize,
        min_thread_period: Duration,
    ) -> Result<JoinHandle<()>, io::Error> {
        let mut group = Group::new(name, thread_stack_size, min_thread_period);

        group.add(Box::new(self));

        group.spawn()
    }
}

impl<A, Ctx, Cmd> Runnable for Fsm<A, Ctx, Cmd>
where
    A: Address,
    Ctx: Send + 'static,
    Cmd: Send + 'static,
{
    fn step(&mut self) -> anyhow::Result<FsmStatus> {
        match self.state.process(&mut self.ctx, &mut self.mailbox)? {
            StateResult::Running(state) => {
                self.state = state;
                Ok(FsmStatus::Running)
            }

            StateResult::Hold => Ok(FsmStatus::Hold),

            StateResult::Stopped => Ok(FsmStatus::Stopped),
        }
    }
}

pub trait Runnable: Send {
    fn step(&mut self) -> anyhow::Result<FsmStatus>;
}
