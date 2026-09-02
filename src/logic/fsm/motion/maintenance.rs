use core::option::Option::None;

use ::fsm::{
    postal::{bulletin::Bulletin, mailbox::Mailbox},
    state::{State, StateResult},
};
use esp_idf_svc::hal::gpio::{InputPin, OutputPin};
use log::error;
use motion::Direction;

use crate::logic::fsm::{
    motion::{MotionContext, MotionInit, MotionMaintenance},
    FSMAddress,
    FSMCommand::{self},
    FSMState,
};

pub(crate) enum MaintenanceAction {
    Moving(Direction),
    Idle,
}

pub(crate) fn perform_maintenance_transition<SLP, STP, DIR, CS, RLY, LMSW, ENCA, ENCB>(
    mailbox: &mut Mailbox<FSMAddress, FSMCommand>,
    return_to: Box<
        dyn State<
            FSMAddress,
            MotionContext<SLP, STP, DIR, CS, RLY, LMSW, ENCA, ENCB>,
            FSMCommand,
            FSMState,
        >,
    >,
) -> Option<
    Box<
        dyn State<
            FSMAddress,
            MotionContext<SLP, STP, DIR, CS, RLY, LMSW, ENCA, ENCB>,
            FSMCommand,
            FSMState,
        >,
    >,
>
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
    Some(Box::new(MotionMaintenance {
        action: check_maintenance(mailbox)?,
        return_to: Some(return_to),
    }))
}

pub(crate) fn check_maintenance(
    mailbox: &mut Mailbox<FSMAddress, FSMCommand>,
) -> Option<MaintenanceAction> {
    match mailbox.receive_latest().ok()? {
        FSMCommand::CCWPressed => Some(MaintenanceAction::Moving(Direction::Ccw)),
        FSMCommand::CWPressed => Some(MaintenanceAction::Moving(Direction::Cw)),
        FSMCommand::MaintenancePressed => Some(MaintenanceAction::Idle),
        _ => None,
    }
}

impl<SLP, STP, DIR, CS, RLY, LMSW, ENCA, ENCB>
    State<FSMAddress, MotionContext<SLP, STP, DIR, CS, RLY, LMSW, ENCA, ENCB>, FSMCommand, FSMState>
    for MotionMaintenance<SLP, STP, DIR, CS, RLY, LMSW, ENCA, ENCB>
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
        mailbox: &mut Mailbox<FSMAddress, FSMCommand>,
        bulletin: &Bulletin<FSMState>,
        _previous_state: Option<
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
        bulletin.update(|state| {
            state.maintenance_mode = true;
        });

        if let Some(action) = check_maintenance(mailbox) {
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
