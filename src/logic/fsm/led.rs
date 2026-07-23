use fsm::{
    postal::{Address, Mailbox},
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

impl<A> InitialState<A, LEDContext, FSMCommand> for LEDHold where A: Address {}

impl<A> State<A, LEDContext, FSMCommand> for LEDHold
where
    A: Address,
{
    fn process(
        &mut self,
        _ctx: &mut LEDContext,
        mailbox: &mut Mailbox<A, FSMCommand>,
    ) -> anyhow::Result<StateResult<A, LEDContext, FSMCommand>> {
        if let Ok(cmd) = mailbox.receive() {
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

impl<A> State<A, LEDContext, FSMCommand> for LEDOff
where
    A: Address,
{
    fn process(
        &mut self,
        ctx: &mut LEDContext,
        _mailbox: &mut Mailbox<A, FSMCommand>,
    ) -> anyhow::Result<StateResult<A, LEDContext, FSMCommand>> {
        ctx.led.display_none()?;

        Ok(StateResult::Running(Box::new(LEDHold)))
    }
}

impl<A> State<A, LEDContext, FSMCommand> for LEDMaintenance
where
    A: Address,
{
    fn process(
        &mut self,
        ctx: &mut LEDContext,
        _mailbox: &mut Mailbox<A, FSMCommand>,
    ) -> anyhow::Result<StateResult<A, LEDContext, FSMCommand>> {
        ctx.led.display_maintenance()?;

        Ok(StateResult::Running(Box::new(LEDHold)))
    }
}

impl<A> State<A, LEDContext, FSMCommand> for LEDMaintenanceMovingCCW
where
    A: Address,
{
    fn process(
        &mut self,
        ctx: &mut LEDContext,
        _mailbox: &mut Mailbox<A, FSMCommand>,
    ) -> anyhow::Result<StateResult<A, LEDContext, FSMCommand>> {
        ctx.led.display_maintenance_moving_ccw()?;

        Ok(StateResult::Running(Box::new(LEDHold)))
    }
}

impl<A> State<A, LEDContext, FSMCommand> for LEDMaintenanceMovingCW
where
    A: Address,
{
    fn process(
        &mut self,
        ctx: &mut LEDContext,
        _mailbox: &mut Mailbox<A, FSMCommand>,
    ) -> anyhow::Result<StateResult<A, LEDContext, FSMCommand>> {
        ctx.led.display_maintenance_moving_cw()?;

        Ok(StateResult::Running(Box::new(LEDHold)))
    }
}
