use esp_idf_hal::gpio::InputPin;
use fsm::{
    postal::{bulletin::Bulletin, mailbox::Mailbox},
    state::{State, StateResult},
};

use crate::{
    hardware::buttons::Buttons,
    logic::fsm::{FSMAddress, FSMCommand, FSMState},
};

pub struct ButtonsContext<M, Ccw, Cw>
where
    M: InputPin,
    Ccw: InputPin,
    Cw: InputPin,
{
    buttons: Buttons<'static, M, Ccw, Cw>,
    last_maintenance_pressed: bool,
    last_ccw_pressed: bool,
    last_cw_pressed: bool,
}

impl<M, Ccw, Cw> ButtonsContext<M, Ccw, Cw>
where
    M: InputPin,
    Ccw: InputPin,
    Cw: InputPin,
{
    pub fn new(buttons: Buttons<'static, M, Ccw, Cw>) -> Self {
        Self {
            buttons,
            last_maintenance_pressed: false,
            last_ccw_pressed: false,
            last_cw_pressed: false,
        }
    }
}

pub struct ButtonsCheckPressed;

impl<M, Ccw, Cw> State<FSMAddress, ButtonsContext<M, Ccw, Cw>, FSMCommand, FSMState>
    for ButtonsCheckPressed
where
    M: InputPin,
    Ccw: InputPin,
    Cw: InputPin,
{
    fn process(
        &mut self,
        ctx: &mut ButtonsContext<M, Ccw, Cw>,
        mailbox: &mut Mailbox<FSMAddress, FSMCommand>,
        _bulletin: &mut Bulletin<FSMState>,
        _previous_state: Option<
            Box<dyn State<FSMAddress, ButtonsContext<M, Ccw, Cw>, FSMCommand, FSMState> + Send>,
        >,
    ) -> anyhow::Result<StateResult<FSMAddress, ButtonsContext<M, Ccw, Cw>, FSMCommand, FSMState>>
    {
        let maintenance_pressed = ctx.buttons.maintenance_pressed();
        let ccw_pressed = ctx.buttons.ccw_pressed();
        let cw_pressed = ctx.buttons.cw_pressed();

        if maintenance_pressed && !ctx.last_maintenance_pressed {
            mailbox.send(FSMAddress::Motion, FSMCommand::MaintenancePressed)?;
        }

        if ccw_pressed && !ctx.last_ccw_pressed {
            mailbox.send(FSMAddress::Motion, FSMCommand::CCWPressed)?;
        }

        if cw_pressed && !ctx.last_cw_pressed {
            mailbox.send(FSMAddress::Motion, FSMCommand::CWPressed)?;
        }

        ctx.last_maintenance_pressed = maintenance_pressed;
        ctx.last_ccw_pressed = ccw_pressed;
        ctx.last_cw_pressed = cw_pressed;

        Ok(StateResult::Hold)
    }
}
