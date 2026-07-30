use core::time::Duration;
use std::time::Instant;

use ::fsm::{
    postal::{bulletin::Bulletin, mailbox::Mailbox},
    state::{State, StateResult},
};
use log::{error, info};
use motion::motion::MoveOutcome;

use crate::logic::fsm::{
    motion::{
        helpers::{
            daily_reset, detect_stepper_transition, perform_maintenance_transition,
            rehome_if_pending, run_encoder_fault, run_tracking, sync_motion_mode_from_nvs,
        },
        MotionBeginHoming, MotionContext, MotionErrorLoop, MotionTracking, MotionTrackingWait,
    },
    FSMAddress,
    FSMCommand::{self, UpdateNetworkMotionContext},
    FSMState,
};

impl State<FSMAddress, MotionContext, FSMCommand, FSMState> for MotionTracking {
    fn process(
        &mut self,
        ctx: &mut MotionContext,
        mailbox: &mut Mailbox<FSMAddress, FSMCommand>,
        _bulletin: &Bulletin<FSMState>,
        _previous_state: Option<
            Box<dyn State<FSMAddress, MotionContext, FSMCommand, FSMState> + Send>,
        >,
    ) -> anyhow::Result<StateResult<FSMAddress, MotionContext, FSMCommand, FSMState>> {
        if let Some(state) = perform_maintenance_transition(mailbox, Box::new(MotionTracking)) {
            return Ok(StateResult::Running(state));
        }

        let local_time = rtc::timezone::local_time();
        let current_datetime = format!("{}", local_time.format("%d/%m/%Y %H:%M:%S"));

        daily_reset(ctx, &local_time);

        // Re-home once when transitioning into StepperOnly
        detect_stepper_transition(ctx);
        let rehome_res = rehome_if_pending(
            ctx,
            "StepperOnly mode detected - re-homing to establish known position",
        )?;
        if rehome_res.0 {
            return Ok(StateResult::Running(Box::new(MotionBeginHoming)));
        } else if let Some((component, message, notes)) = rehome_res.1 {
            return Ok(StateResult::Running(Box::new(MotionErrorLoop {
                component,
                message,
                notes,
            })));
        }

        info!("Actual Heading: {}", ctx.motion.location());
        info!("Current datetime: {}", current_datetime.clone());

        let now = std::time::Instant::now();

        // Reload mode in case another path switched to StepperOnly.
        sync_motion_mode_from_nvs(
            ctx,
            "Motion mode changed to StepperOnly - will re-home on next iteration",
        );

        // Fault active, skip tracking this iteration
        let fault_res = run_encoder_fault(ctx)?;
        if fault_res.0 {
            return Ok(StateResult::Running(Box::new(MotionTrackingWait {
                begin: Instant::now(),
            })));
        } else if let Some((component, message, notes)) = fault_res.1 {
            return Ok(StateResult::Running(Box::new(MotionErrorLoop {
                component,
                message,
                notes,
            })));
        }

        // Re-check mode after recovery in case it changed during the probe.
        sync_motion_mode_from_nvs(
            ctx,
            "Motion mode switched to StepperOnly during encoder fault recovery - will re-home",
        );

        // Re-home before tracking if StepperOnly was activated during recovery.
        let rehome_res = rehome_if_pending(
            ctx,
            "StepperOnly mode detected - re-homing to establish known position (CCW)",
        )?;
        if rehome_res.0 {
            return Ok(StateResult::Running(Box::new(MotionTrackingWait {
                begin: Instant::now(),
            })));
        } else if let Some((component, message, notes)) = rehome_res.1 {
            return Ok(StateResult::Running(Box::new(MotionErrorLoop {
                component,
                message,
                notes,
            })));
        }

        if let Some(outcome) = run_tracking(ctx, mailbox)? {
            if let MoveOutcome::AbortedErrorLoop(component, message, notes) = outcome {
                error!("Catastrophic tracking error; manual intervention required");
                return Ok(StateResult::Running(Box::new(MotionErrorLoop {
                    component,
                    message,
                    notes,
                })));
            }
        }

        info!("Tracking loop duration (v1.1.5): {:?}", now.elapsed());

        Ok(StateResult::Running(Box::new(MotionTrackingWait {
            begin: Instant::now(),
        })))
    }
}

impl State<FSMAddress, MotionContext, FSMCommand, FSMState> for MotionTrackingWait {
    fn process(
        &mut self,
        ctx: &mut MotionContext,
        mailbox: &mut Mailbox<FSMAddress, FSMCommand>,
        _bulletin: &Bulletin<FSMState>,
        _previous_state: Option<
            Box<dyn State<FSMAddress, MotionContext, FSMCommand, FSMState> + Send>,
        >,
    ) -> anyhow::Result<StateResult<FSMAddress, MotionContext, FSMCommand, FSMState>> {
        if let Some(state) = perform_maintenance_transition(mailbox, Box::new(MotionTracking)) {
            return Ok(StateResult::Running(state));
        }

        mailbox.send(
            FSMAddress::Network,
            UpdateNetworkMotionContext(ctx.motion_mode, ctx.actual_heading),
        )?;

        if self.begin.elapsed() < Duration::from_mins(5) {
            return Ok(StateResult::Hold);
        }

        Ok(StateResult::Running(Box::new(MotionTracking)))
    }
}
