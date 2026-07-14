use std::sync::mpsc::{Receiver, Sender};

use fsm::{InitialState, State};
use rgb_led::Led;

use crate::FSMCommand;

pub struct LEDContext<'led> {
    pub led: Led<'led>,
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

impl<'led> InitialState<LEDContext<'led>, FSMCommand> for LEDHold {}

use std::sync::mpsc::TryRecvError;

impl<'led> State<LEDContext<'led>, FSMCommand> for LEDHold {
    fn process(
        &mut self,
        ctx: &mut LEDContext<'led>,
        _tx: &mut Sender<FSMCommand>,
        rx: &mut Receiver<FSMCommand>,
    ) -> anyhow::Result<Option<Box<dyn State<LEDContext<'led>, FSMCommand> + Send>>> {
        let mut latest = None;

        loop {
            match rx.try_recv() {
                Ok(cmd) => latest = Some(cmd),
                Err(TryRecvError::Empty) => break,
                Err(TryRecvError::Disconnected) => break,
            }
        }

        match latest {
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

impl<'led> State<LEDContext<'led>, FSMCommand> for LEDOff {
    fn process(
        &mut self,
        ctx: &mut LEDContext<'led>,
        _tx: &mut Sender<FSMCommand>,
        _rx: &mut Receiver<FSMCommand>,
    ) -> anyhow::Result<
        Option<Box<dyn State<LEDContext<'led>, FSMCommand> + core::prelude::v1::Send>>,
    > {
        ctx.led.display_none()?;
        Ok(Some(Box::new(LEDHold)))
    }
}

impl<'led> State<LEDContext<'led>, FSMCommand> for LEDMaintenance {
    fn process(
        &mut self,
        ctx: &mut LEDContext<'led>,
        _tx: &mut Sender<FSMCommand>,
        _rx: &mut Receiver<FSMCommand>,
    ) -> anyhow::Result<Option<Box<dyn State<LEDContext<'led>, FSMCommand> + Send>>> {
        ctx.led.display_maintenance()?;
        Ok(Some(Box::new(LEDHold)))
    }
}

impl<'led> State<LEDContext<'led>, FSMCommand> for LEDMaintenanceMovingCCW {
    fn process(
        &mut self,
        ctx: &mut LEDContext<'led>,
        _tx: &mut Sender<FSMCommand>,
        _rx: &mut Receiver<FSMCommand>,
    ) -> anyhow::Result<Option<Box<dyn State<LEDContext<'led>, FSMCommand> + Send>>> {
        ctx.led.display_maintenance_moving_ccw()?;
        Ok(Some(Box::new(LEDHold)))
    }
}

impl<'led> State<LEDContext<'led>, FSMCommand> for LEDMaintenanceMovingCW {
    fn process(
        &mut self,
        ctx: &mut LEDContext<'led>,
        _tx: &mut Sender<FSMCommand>,
        _rx: &mut Receiver<FSMCommand>,
    ) -> anyhow::Result<Option<Box<dyn State<LEDContext<'led>, FSMCommand> + Send>>> {
        ctx.led.display_maintenance_moving_cw()?;
        Ok(Some(Box::new(LEDHold)))
    }
}
