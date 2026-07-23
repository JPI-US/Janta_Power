use core::time::Duration;
use std::{io, thread::JoinHandle};

use crossbeam_channel::{Receiver, Sender};

use crate::{
    channel::Channel,
    group::Group,
    state::{State, StateResult},
};

pub mod channel;
pub mod group;
pub mod state;

pub struct Fsm<Ctx, Cmd> {
    pub state: Box<dyn State<Ctx, Cmd> + Send>,
    pub ctx: Ctx,
    pub channel: Channel<Cmd>,
}

pub enum FsmStatus {
    Running,
    Hold,
    Stopped,
}

impl<Ctx, Cmd> Fsm<Ctx, Cmd>
where
    Self: Send + 'static,
    Ctx: Send + 'static,
    Cmd: Send + 'static,
{
    pub fn new(
        state: Box<dyn State<Ctx, Cmd> + Send>,
        ctx: Ctx,
        tx: Sender<Cmd>,
        rx: Receiver<Cmd>,
    ) -> Self {
        Self {
            state,
            ctx,
            channel: Channel::new(tx, rx, 10),
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

impl<Ctx, Cmd> Runnable for Fsm<Ctx, Cmd>
where
    Ctx: Send + 'static,
    Cmd: Send + 'static,
{
    fn step(&mut self) -> anyhow::Result<FsmStatus> {
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

pub trait Runnable: Send {
    fn step(&mut self) -> anyhow::Result<FsmStatus>;
}
