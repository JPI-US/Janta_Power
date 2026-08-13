use ::fsm::{
    postal::{bulletin::Bulletin, mailbox::Mailbox},
    state::{State, StateResult},
};
use log::error;

use crate::logic::fsm::{
    motion::{MotionContext, MotionInit, MotionMoving},
    FSMAddress,
    FSMCommand::{self},
    FSMState,
};

impl State<FSMAddress, MotionContext, FSMCommand, FSMState> for MotionMoving {
    fn process(
        &mut self,
        ctx: &mut MotionContext,
        _mailbox: &mut Mailbox<FSMAddress, FSMCommand>,
        _bulletin: &Bulletin<FSMState>,
        previous_state: Option<
            Box<dyn State<FSMAddress, MotionContext, FSMCommand, FSMState> + Send>,
        >,
    ) -> anyhow::Result<StateResult<FSMAddress, MotionContext, FSMCommand, FSMState>> {
        match ctx.motion.move_by(self.steps) {
            Ok(_) => {}
            Err(e) => {
                error!("Failed to move motor with error: {e}");
            }
        }

        if let Some(previous_state) = previous_state {
            Ok(StateResult::Running(previous_state))
        } else {
            error!("No previous state found after motion movement; re-initializing");
            Ok(StateResult::Running(Box::new(MotionInit)))
        }
    }
}
