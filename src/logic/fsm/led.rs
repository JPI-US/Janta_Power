use std::sync::mpsc::{Receiver, Sender};

use fsm::{drain_rx, InitialState, State, StateResult};
use rgb_led::Led;

use crate::logic::fsm::FSMCommand;

pub struct LEDContext {
    pub led: Led<'static>,
}

#[derive(Default)]
pub struct LEDHold;

#[derive(Default)]
pub struct LEDOff;

#[derive(Default)]
pub struct LEDMaintenance;

#[derive(Default)]
pub struct LEDMaintenanceMovingCCW;

#[derive(Default)]
pub struct LEDMaintenanceMovingCW;

impl InitialState<LEDContext, FSMCommand> for LEDHold {}

impl State<LEDContext, FSMCommand> for LEDHold {
    fn process(
        &mut self,
        _ctx: &mut LEDContext,
        _tx: &mut Sender<FSMCommand>,
        rx: &mut Receiver<FSMCommand>,
    ) -> anyhow::Result<StateResult<LEDContext, FSMCommand>> {
        match drain_rx(rx) {
            Some(FSMCommand::LEDOff) => Ok(StateResult::Running(Box::new(LEDOff))),
            Some(FSMCommand::LEDMaintenance) => Ok(StateResult::Running(Box::new(LEDMaintenance))),
            Some(FSMCommand::LEDMaintenanceMovingCCW) => {
                Ok(StateResult::Running(Box::new(LEDMaintenanceMovingCCW)))
            }
            Some(FSMCommand::LEDMaintenanceMovingCW) => {
                Ok(StateResult::Running(Box::new(LEDMaintenanceMovingCW)))
            }
            _ => Ok(StateResult::Hold),
        }
    }
}

impl State<LEDContext, FSMCommand> for LEDOff {
    fn process(
        &mut self,
        ctx: &mut LEDContext,
        _tx: &mut Sender<FSMCommand>,
        _rx: &mut Receiver<FSMCommand>,
    ) -> anyhow::Result<StateResult<LEDContext, FSMCommand>> {
        ctx.led.display_none()?;
        Ok(StateResult::Running(Box::new(LEDHold)))
    }
}

impl State<LEDContext, FSMCommand> for LEDMaintenance {
    fn process(
        &mut self,
        ctx: &mut LEDContext,
        _tx: &mut Sender<FSMCommand>,
        _rx: &mut Receiver<FSMCommand>,
    ) -> anyhow::Result<StateResult<LEDContext, FSMCommand>> {
        ctx.led.display_maintenance()?;
        Ok(StateResult::Running(Box::new(LEDHold)))
    }
}

impl State<LEDContext, FSMCommand> for LEDMaintenanceMovingCCW {
    fn process(
        &mut self,
        ctx: &mut LEDContext,
        _tx: &mut Sender<FSMCommand>,
        _rx: &mut Receiver<FSMCommand>,
    ) -> anyhow::Result<StateResult<LEDContext, FSMCommand>> {
        ctx.led.display_maintenance_moving_ccw()?;
        Ok(StateResult::Running(Box::new(LEDHold)))
    }
}

impl State<LEDContext, FSMCommand> for LEDMaintenanceMovingCW {
    fn process(
        &mut self,
        ctx: &mut LEDContext,
        _tx: &mut Sender<FSMCommand>,
        _rx: &mut Receiver<FSMCommand>,
    ) -> anyhow::Result<StateResult<LEDContext, FSMCommand>> {
        ctx.led.display_maintenance_moving_cw()?;
        Ok(StateResult::Running(Box::new(LEDHold)))
    }
}
