use fsm::{
    channel::Channel,
    state::{InitialState, State, StateResult},
};
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
        channel: &mut Channel<FSMCommand>,
    ) -> anyhow::Result<StateResult<LEDContext, FSMCommand>> {
        if let Ok(cmd) = channel.recv() {
            match cmd {
                FSMCommand::LEDOff => Ok(StateResult::Running(Box::new(LEDOff))),
                FSMCommand::LEDMaintenance => Ok(StateResult::Running(Box::new(LEDMaintenance))),
                FSMCommand::LEDMaintenanceMovingCCW => {
                    Ok(StateResult::Running(Box::new(LEDMaintenanceMovingCCW)))
                }
                FSMCommand::LEDMaintenanceMovingCW => {
                    Ok(StateResult::Running(Box::new(LEDMaintenanceMovingCW)))
                }
                _ => Ok(StateResult::Hold),
            }
        } else {
            Ok(StateResult::Hold)
        }
    }
}

impl State<LEDContext, FSMCommand> for LEDOff {
    fn process(
        &mut self,
        ctx: &mut LEDContext,
        _channel: &mut Channel<FSMCommand>,
    ) -> anyhow::Result<StateResult<LEDContext, FSMCommand>> {
        ctx.led.display_none()?;
        Ok(StateResult::Running(Box::new(LEDHold)))
    }
}

impl State<LEDContext, FSMCommand> for LEDMaintenance {
    fn process(
        &mut self,
        ctx: &mut LEDContext,
        _channel: &mut Channel<FSMCommand>,
    ) -> anyhow::Result<StateResult<LEDContext, FSMCommand>> {
        ctx.led.display_maintenance()?;
        Ok(StateResult::Running(Box::new(LEDHold)))
    }
}

impl State<LEDContext, FSMCommand> for LEDMaintenanceMovingCCW {
    fn process(
        &mut self,
        ctx: &mut LEDContext,
        _channel: &mut Channel<FSMCommand>,
    ) -> anyhow::Result<StateResult<LEDContext, FSMCommand>> {
        ctx.led.display_maintenance_moving_ccw()?;
        Ok(StateResult::Running(Box::new(LEDHold)))
    }
}

impl State<LEDContext, FSMCommand> for LEDMaintenanceMovingCW {
    fn process(
        &mut self,
        ctx: &mut LEDContext,
        _channel: &mut Channel<FSMCommand>,
    ) -> anyhow::Result<StateResult<LEDContext, FSMCommand>> {
        ctx.led.display_maintenance_moving_cw()?;
        Ok(StateResult::Running(Box::new(LEDHold)))
    }
}
