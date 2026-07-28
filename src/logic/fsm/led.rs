use fsm::{
    postal::{bulletin::Bulletin, mailbox::Mailbox},
    state::{InitialState, State, StateResult},
};
use log::error;
use rgb_led::Led;

use crate::logic::fsm::{FSMAddress, FSMCommand, FSMState};

pub struct LEDContext {
    led: Led<'static>,
}

impl LEDContext {
    pub fn new(led: Led<'static>) -> Self {
        Self { led }
    }
}

pub struct LEDCheck;

impl InitialState<FSMAddress, LEDContext, FSMCommand, FSMState> for LEDCheck {}

impl State<FSMAddress, LEDContext, FSMCommand, FSMState> for LEDCheck {
    fn process(
        &mut self,
        ctx: &mut LEDContext,
        _mailbox: &mut Mailbox<FSMAddress, FSMCommand>,
        bulletin: &Bulletin<FSMState>,
        _previous_state: Option<
            Box<dyn State<FSMAddress, LEDContext, FSMCommand, FSMState> + Send>,
        >,
    ) -> anyhow::Result<StateResult<FSMAddress, LEDContext, FSMCommand, FSMState>> {
        if let Some(state) = bulletin.read() {
            if state.maintenance_mode {
                ctx.led.display_maintenance()?;
            } else {
                ctx.led.display_none()?;
            }
        }

        Ok(StateResult::Hold)
    }
}
