use core::marker::Send;
use std::sync::mpsc::{Receiver, Sender};

use esp_idf_hal::gpio::Pin;
use fsm::{InitialState, State};

use crate::{hardware::buttons::Buttons, logic::fsm::FSMCommand};

pub struct MaintenanceContext<M, Ccw, Cw>
where
    M: Pin,
    Ccw: Pin,
    Cw: Pin,
{
    pub buttons: Buttons<'static, M, Ccw, Cw>,
    maintenance: bool,
    ccw: bool,
    cw: bool,
    last_maintenance: bool,
    last_ccw: bool,
    last_cw: bool,
}

impl<M, Ccw, Cw> MaintenanceContext<M, Ccw, Cw>
where
    M: Pin,
    Ccw: Pin,
    Cw: Pin,
{
    pub fn new(buttons: Buttons<'static, M, Ccw, Cw>) -> Self {
        Self {
            buttons,
            maintenance: false,
            ccw: false,
            cw: false,
            last_maintenance: false,
            last_ccw: false,
            last_cw: false,
        }
    }

    fn poll_buttons(&mut self) {
        self.last_maintenance = self.maintenance;
        self.last_ccw = self.ccw;
        self.last_cw = self.cw;

        self.maintenance = self.buttons.maintenance_pressed();
        self.ccw = self.buttons.ccw_pressed();
        self.cw = self.buttons.cw_pressed();
    }

    fn maintenance_pressed(&self) -> bool {
        self.maintenance && !self.last_maintenance
    }

    fn ccw_pressed(&self) -> bool {
        self.ccw && !self.last_ccw
    }

    fn cw_pressed(&self) -> bool {
        self.cw && !self.last_cw
    }
}

#[derive(Default)]
pub struct MaintenanceEnter;

#[derive(Default)]
pub struct MaintenanceNotMoving;

#[derive(Default)]
pub struct MaintenanceMoveCCW;

#[derive(Default)]
pub struct MaintenanceMoveCW;

#[derive(Default)]
pub struct MaintenanceExit;

impl<'mb, M, Ccw, Cw> InitialState<MaintenanceContext<M, Ccw, Cw>, FSMCommand> for MaintenanceEnter
where
    M: Pin,
    Ccw: Pin,
    Cw: Pin,
{
}

impl<'mb, M, Ccw, Cw> State<MaintenanceContext<M, Ccw, Cw>, FSMCommand> for MaintenanceEnter
where
    M: Pin,
    Ccw: Pin,
    Cw: Pin,
{
    fn process(
        &mut self,
        ctx: &mut MaintenanceContext<M, Ccw, Cw>,
        tx: &mut Sender<FSMCommand>,
        _rx: &mut Receiver<FSMCommand>,
    ) -> anyhow::Result<Option<Box<dyn State<MaintenanceContext<M, Ccw, Cw>, FSMCommand> + Send>>>
    {
        ctx.poll_buttons();

        if ctx.maintenance_pressed() {
            tx.send(FSMCommand::LEDMaintenance)?;
            return Ok(Some(Box::new(MaintenanceNotMoving)));
        }

        Ok(Some(Box::new(MaintenanceEnter)))
    }
}

impl<'mb, M, Ccw, Cw> State<MaintenanceContext<M, Ccw, Cw>, FSMCommand> for MaintenanceNotMoving
where
    M: Pin,
    Ccw: Pin,
    Cw: Pin,
{
    fn process(
        &mut self,
        ctx: &mut MaintenanceContext<M, Ccw, Cw>,
        tx: &mut Sender<FSMCommand>,
        _rx: &mut Receiver<FSMCommand>,
    ) -> anyhow::Result<Option<Box<dyn State<MaintenanceContext<M, Ccw, Cw>, FSMCommand> + Send>>>
    {
        ctx.poll_buttons();

        if ctx.maintenance_pressed() {
            tx.send(FSMCommand::LEDOff)?;
            return Ok(Some(Box::new(MaintenanceExit)));
        } else if ctx.ccw_pressed() {
            tx.send(FSMCommand::LEDMaintenanceMovingCCW)?;
            return Ok(Some(Box::new(MaintenanceMoveCCW)));
        } else if ctx.cw_pressed() {
            tx.send(FSMCommand::LEDMaintenanceMovingCW)?;
            return Ok(Some(Box::new(MaintenanceMoveCW)));
        } else {
            tx.send(FSMCommand::LEDMaintenance)?;
            return Ok(Some(Box::new(MaintenanceNotMoving)));
        }
    }
}

impl<'mb, M, Ccw, Cw> State<MaintenanceContext<M, Ccw, Cw>, FSMCommand> for MaintenanceMoveCCW
where
    M: Pin,
    Ccw: Pin,
    Cw: Pin,
{
    fn process(
        &mut self,
        ctx: &mut MaintenanceContext<M, Ccw, Cw>,
        tx: &mut Sender<FSMCommand>,
        _rx: &mut Receiver<FSMCommand>,
    ) -> anyhow::Result<Option<Box<dyn State<MaintenanceContext<M, Ccw, Cw>, FSMCommand> + Send>>>
    {
        ctx.poll_buttons();

        if ctx.maintenance_pressed() {
            tx.send(FSMCommand::LEDOff)?;
            return Ok(Some(Box::new(MaintenanceExit)));
        } else if ctx.cw_pressed() {
            tx.send(FSMCommand::LEDMaintenanceMovingCW)?;
            return Ok(Some(Box::new(MaintenanceMoveCW)));
        }

        tx.send(FSMCommand::LEDMaintenanceMovingCCW)?;
        tx.send(FSMCommand::MotionMoveBy(30_000))?;
        Ok(Some(Box::new(MaintenanceMoveCCW)))
    }
}

impl<'mb, M, Ccw, Cw> State<MaintenanceContext<M, Ccw, Cw>, FSMCommand> for MaintenanceMoveCW
where
    M: Pin,
    Ccw: Pin,
    Cw: Pin,
{
    fn process(
        &mut self,
        ctx: &mut MaintenanceContext<M, Ccw, Cw>,
        tx: &mut Sender<FSMCommand>,
        _rx: &mut Receiver<FSMCommand>,
    ) -> anyhow::Result<Option<Box<dyn State<MaintenanceContext<M, Ccw, Cw>, FSMCommand> + Send>>>
    {
        ctx.poll_buttons();

        if ctx.maintenance_pressed() {
            tx.send(FSMCommand::LEDOff)?;
            return Ok(Some(Box::new(MaintenanceExit)));
        } else if ctx.ccw_pressed() {
            tx.send(FSMCommand::LEDMaintenanceMovingCCW)?;
            return Ok(Some(Box::new(MaintenanceMoveCCW)));
        }

        tx.send(FSMCommand::LEDMaintenanceMovingCW)?;
        tx.send(FSMCommand::MotionMoveBy(-30_000))?;
        Ok(Some(Box::new(MaintenanceMoveCW)))
    }
}

impl<'mb, M, Ccw, Cw> State<MaintenanceContext<M, Ccw, Cw>, FSMCommand> for MaintenanceExit
where
    M: Pin,
    Ccw: Pin,
    Cw: Pin,
{
    fn process(
        &mut self,
        _ctx: &mut MaintenanceContext<M, Ccw, Cw>,
        _tx: &mut Sender<FSMCommand>,
        _rx: &mut Receiver<FSMCommand>,
    ) -> anyhow::Result<Option<Box<dyn State<MaintenanceContext<M, Ccw, Cw>, FSMCommand> + Send>>>
    {
        Ok(Some(Box::new(MaintenanceEnter)))
    }
}
