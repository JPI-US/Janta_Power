use core::option::Option::None;

use ::fsm::{
    postal::{bulletin::Bulletin, mailbox::Mailbox},
    state::{State, StateResult},
};
use log::error;
use motion::Direction;

use crate::logic::fsm::{
    motion::{MotionContext, MotionInit, MotionMaintenance},
    FSMAddress,
    FSMCommand::{self},
    FSMState,
};

#[derive(Copy, Clone, Debug)]
pub(crate) enum MaintenanceAction {
    Moving(Direction),
    Idle,
}

/// If a maintenance button was pressed, the state to hand control to.
///
/// `return_to` is where [`MotionMaintenance`] resumes once maintenance ends, so
/// the caller passes a copy of itself and gets back where it was.
pub(crate) fn perform_maintenance_transition(
    ctx: &mut MotionContext,
    mailbox: &mut Mailbox<FSMAddress, FSMCommand>,
    return_to: Box<dyn State<FSMAddress, MotionContext, FSMCommand, FSMState>>,
) -> Option<Box<dyn State<FSMAddress, MotionContext, FSMCommand, FSMState>>> {
    Some(Box::new(MotionMaintenance {
        action: check_maintenance(ctx, mailbox)?,
        return_to: Some(return_to),
    }))
}

/// Drain the mailbox and report a pending button action, if there is one.
///
/// Draining is the point, not a side effect: this is called from every state that
/// can be interrupted, so it is also what keeps mail moving for anything else
/// addressed to this FSM. See [`super::MotionInbox`] for why it cannot simply
/// take from the mailbox directly.
pub(crate) fn check_maintenance(
    ctx: &mut MotionContext,
    mailbox: &mut Mailbox<FSMAddress, FSMCommand>,
) -> Option<MaintenanceAction> {
    ctx.inbox.fill(mailbox);
    ctx.inbox.take_button()
}

impl State<FSMAddress, MotionContext, FSMCommand, FSMState> for MotionMaintenance {
    fn process(
        &mut self,
        ctx: &mut MotionContext,
        mailbox: &mut Mailbox<FSMAddress, FSMCommand>,
        bulletin: &Bulletin<FSMState>,
        _previous_state: Option<
            Box<dyn State<FSMAddress, MotionContext, FSMCommand, FSMState> + Send>,
        >,
    ) -> anyhow::Result<StateResult<FSMAddress, MotionContext, FSMCommand, FSMState>> {
        bulletin.update(|state| {
            state.maintenance_mode = true;
        });

        if let Some(action) = check_maintenance(ctx, mailbox) {
            match action {
                MaintenanceAction::Idle => {
                    if matches!(self.action, MaintenanceAction::Idle) {
                        bulletin.update(|state| {
                            state.maintenance_mode = false;
                        });

                        return Ok(match self.return_to.take() {
                            Some(state) => StateResult::Running(state),
                            None => {
                                error!(
                                "No return state found in MotionMaintenance; falling back to MotionInit"
                            );
                                StateResult::Running(Box::new(MotionInit))
                            }
                        });
                    }

                    self.action = MaintenanceAction::Idle;
                }

                action => {
                    self.action = action;
                }
            }
        }

        match self.action {
            MaintenanceAction::Moving(direction) => {
                ctx.motion.move_by(match direction {
                    Direction::Ccw => -150_000,
                    Direction::Cw => 150_000,
                })?;

                Ok(StateResult::Hold)
            }

            MaintenanceAction::Idle => Ok(StateResult::Hold),
        }
    }
}
