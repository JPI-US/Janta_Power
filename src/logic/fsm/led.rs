use std::sync::mpsc::{Receiver, Sender};

use fsm::{drain_rx, InitialState, State};
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
    ) -> anyhow::Result<Option<Box<dyn State<LEDContext, FSMCommand> + Send>>> {
        match drain_rx(rx) {
            Some(FSMCommand::LEDOff) => Ok(Some(Box::new(LEDOff))),
            Some(FSMCommand::LEDMaintenance) => Ok(Some(Box::new(LEDMaintenance))),
            Some(FSMCommand::LEDMaintenanceMovingCCW) => {
                Ok(Some(Box::new(LEDMaintenanceMovingCCW)))
            }
            Some(FSMCommand::LEDMaintenanceMovingCW) => Ok(Some(Box::new(LEDMaintenanceMovingCW))),
            _ => Ok(Some(Box::new(LEDHold))),
        }
    }
}

impl State<LEDContext, FSMCommand> for LEDOff {
    fn process(
        &mut self,
        ctx: &mut LEDContext,
        _tx: &mut Sender<FSMCommand>,
        _rx: &mut Receiver<FSMCommand>,
    ) -> anyhow::Result<Option<Box<dyn State<LEDContext, FSMCommand> + Send>>> {
        ctx.led.display_none()?;
        Ok(Some(Box::new(LEDHold)))
    }
}

impl State<LEDContext, FSMCommand> for LEDMaintenance {
    fn process(
        &mut self,
        ctx: &mut LEDContext,
        _tx: &mut Sender<FSMCommand>,
        _rx: &mut Receiver<FSMCommand>,
    ) -> anyhow::Result<Option<Box<dyn State<LEDContext, FSMCommand> + Send>>> {
        ctx.led.display_maintenance()?;
        Ok(Some(Box::new(LEDHold)))
    }
}

impl State<LEDContext, FSMCommand> for LEDMaintenanceMovingCCW {
    fn process(
        &mut self,
        ctx: &mut LEDContext,
        _tx: &mut Sender<FSMCommand>,
        _rx: &mut Receiver<FSMCommand>,
    ) -> anyhow::Result<Option<Box<dyn State<LEDContext, FSMCommand> + Send>>> {
        ctx.led.display_maintenance_moving_ccw()?;
        Ok(Some(Box::new(LEDHold)))
    }
}

impl State<LEDContext, FSMCommand> for LEDMaintenanceMovingCW {
    fn process(
        &mut self,
        ctx: &mut LEDContext,
        _tx: &mut Sender<FSMCommand>,
        _rx: &mut Receiver<FSMCommand>,
    ) -> anyhow::Result<Option<Box<dyn State<LEDContext, FSMCommand> + Send>>> {
        ctx.led.display_maintenance_moving_cw()?;
        Ok(Some(Box::new(LEDHold)))
    }
}
