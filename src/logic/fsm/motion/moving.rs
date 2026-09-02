use ::fsm::{
    postal::{bulletin::Bulletin, mailbox::Mailbox},
    state::{State, StateResult},
};
use esp_idf_svc::hal::gpio::{InputPin, OutputPin};
use log::error;

use crate::logic::fsm::{
    motion::{MotionContext, MotionInit, MotionMoving},
    FSMAddress,
    FSMCommand::{self},
    FSMState,
};

impl<SLP, STP, DIR, CS, RLY, LMSW, ENCA, ENCB>
    State<FSMAddress, MotionContext<SLP, STP, DIR, CS, RLY, LMSW, ENCA, ENCB>, FSMCommand, FSMState>
    for MotionMoving
where
    SLP: OutputPin,
    STP: OutputPin,
    DIR: OutputPin,
    CS: OutputPin,
    RLY: OutputPin,
    LMSW: InputPin + OutputPin,
    ENCA: InputPin,
    ENCB: InputPin,
{
    fn process(
        &mut self,
        ctx: &mut MotionContext<SLP, STP, DIR, CS, RLY, LMSW, ENCA, ENCB>,
        _mailbox: &mut Mailbox<FSMAddress, FSMCommand>,
        _bulletin: &Bulletin<FSMState>,
        previous_state: Option<
            Box<
                dyn State<
                        FSMAddress,
                        MotionContext<SLP, STP, DIR, CS, RLY, LMSW, ENCA, ENCB>,
                        FSMCommand,
                        FSMState,
                    > + Send,
            >,
        >,
    ) -> anyhow::Result<
        StateResult<
            FSMAddress,
            MotionContext<SLP, STP, DIR, CS, RLY, LMSW, ENCA, ENCB>,
            FSMCommand,
            FSMState,
        >,
    > {
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
