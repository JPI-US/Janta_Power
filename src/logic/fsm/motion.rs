use std::sync::mpsc::{Receiver, Sender};

use fsm::{drain_rx, InitialState, State};
use motion::Motion;

use crate::logic::fsm::FSMCommand::{self, MotionMoveBy};

pub struct MotionContext {
    motion: Motion<'static>,
}

impl MotionContext {
    pub fn new(motion: Motion<'static>) -> Self {
        Self { motion }
    }
}

#[derive(Default)]
pub struct MotionNotMoving;

#[derive(Default)]
pub struct MotionMoving {
    by: i64,
}

impl InitialState<MotionContext, FSMCommand> for MotionNotMoving {}

impl State<MotionContext, FSMCommand> for MotionNotMoving {
    fn process(
        &mut self,
        _ctx: &mut MotionContext,
        _tx: &mut Sender<FSMCommand>,
        rx: &mut Receiver<FSMCommand>,
    ) -> anyhow::Result<Option<Box<dyn State<MotionContext, FSMCommand> + Send>>> {
        match drain_rx(rx) {
            Some(MotionMoveBy(by)) => Ok(Some(Box::new(MotionMoving { by }))),
            _ => Ok(Some(Box::new(MotionNotMoving))),
        }
    }
}

impl State<MotionContext, FSMCommand> for MotionMoving {
    fn process(
        &mut self,
        ctx: &mut MotionContext,
        _tx: &mut Sender<FSMCommand>,
        _rx: &mut Receiver<FSMCommand>,
    ) -> anyhow::Result<Option<Box<dyn State<MotionContext, FSMCommand> + Send>>> {
        ctx.motion.move_by(self.by)?;
        Ok(Some(Box::new(MotionNotMoving)))
    }
}
