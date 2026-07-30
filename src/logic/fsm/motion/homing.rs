use core::convert::Into;

use ::fsm::{
    postal::{bulletin::Bulletin, mailbox::Mailbox},
    state::{State, StateResult},
};
use motion::motion::{calculate_steps, MotionMode};
use network::telemetry::Component;

use crate::{
    config::{constants::HOME_HEADING_DEG, switchboard::Direction},
    logic::fsm::{
        motion::{
            helpers::perform_maintenance_transition, MotionBeginHoming, MotionContext,
            MotionErrorLoop, MotionHoming, MotionMoving, MotionTracking,
        },
        FSMAddress,
        FSMCommand::{self},
        FSMState,
    },
    storage::snapshot_store::SnapshotStore,
};

/// Same tolerance as motion sunset home check (`location` vs `HOME_HEADING_DEG`).
const HOME_HEADING_VERIFY_EPS_DEG: f32 = 0.01;
// Homing policy:
// - StepperOnly: always home
// - EncoderGuarded: home when snapshot restore is unavailable/untrusted
// - EncoderGuarded: if NVS claims mechanical home (≈ home_heading_deg), require limit
//   switch active before skipping homing; otherwise re-home like snapshot miss
const HOMING_DIRECTION: Direction = Direction::Ccw;
const PERSIST_NVS: bool = true;

impl State<FSMAddress, MotionContext, FSMCommand, FSMState> for MotionBeginHoming {
    fn process(
        &mut self,
        ctx: &mut MotionContext,
        mailbox: &mut Mailbox<FSMAddress, FSMCommand>,
        _bulletin: &Bulletin<FSMState>,
        _previous_state: Option<
            Box<dyn State<FSMAddress, MotionContext, FSMCommand, FSMState> + Send>,
        >,
    ) -> anyhow::Result<StateResult<FSMAddress, MotionContext, FSMCommand, FSMState>> {
        if let Some(state) = perform_maintenance_transition(mailbox, Box::new(MotionBeginHoming)) {
            return Ok(StateResult::Running(state));
        }

        let would_skip_homing_on_snapshot = ctx.motion_mode == MotionMode::EncoderGuarded
            && ctx.restored_from_snapshot
            && ctx.trust_nvs_state;

        let restored_claims_mechanical_home =
            (ctx.actual_heading - ctx.switchboard.home_heading_deg).abs()
                < HOME_HEADING_VERIFY_EPS_DEG;

        let home_claim_needs_limit_verify = would_skip_homing_on_snapshot
            && restored_claims_mechanical_home
            && !ctx.motion.switch_pressed();

        if home_claim_needs_limit_verify {
            log::info!(
                "Restored heading matches home ({}) but limit switch not pressed; homing to verify",
                ctx.switchboard.home_heading_deg
            );
        }

        let should_home_by_mode = ctx.motion_mode == MotionMode::StepperOnly
            || !ctx.restored_from_snapshot
            || !ctx.trust_nvs_state
            || home_claim_needs_limit_verify;

        if should_home_by_mode && ctx.switchboard.boot.homing.enabled {
            if ctx.motion.lmsw_is_low() {
                log::info!("Limit switch already pressed - skipping homing");
                ctx.motion.update_position(ctx.switchboard.home_heading_deg);
                ctx.motion.force_zero_if_limit_switch_pressed();
                return Ok(StateResult::Running(Box::new(MotionTracking)));
            }

            if HOMING_DIRECTION == Direction::Cw {
                log::warn!("CW homing requested, but firmware is only configured for CCW. Performing CCW home.")
            }

            // Disable overshoot checks while homing.
            ctx.motion.set_homing(true);

            // Keep searching until switch is found or travel budget is exhausted.
            // Stall detection is temporarily disabled to avoid abort cascades.
            let stall_prev = ctx.motion.stall_detection_enabled();
            ctx.motion.set_stall_detection_enabled(false);

            log::info!("Looking for the limit switch (CCW search, max 350)");

            return Ok(StateResult::Running(Box::new(MotionHoming {
                stall_prev,
                steps_left: calculate_steps(-350.0),
            })));
        } else if should_home_by_mode {
            log::warn!("Homing skipped: HOMING_ENABLED=false");
            if !ctx.trust_nvs_state {
                return Ok(StateResult::Running(Box::new(MotionErrorLoop {
                	component: network::telemetry::Component::System,
                	message: "Homing disabled with untrusted NVS state".into(),
                	notes:  "HOMING_ENABLED=false but persisted heading could not be trusted; manual intervention required.".into(),
                })));
            }
        } else {
            log::info!("Skipping homing: restored heading+encoder snapshot from NVS");
        }

        SnapshotStore::new(&mut ctx.nvs, true).save_last_run_normal(true);

        Ok(StateResult::Running(Box::new(MotionTracking)))
    }
}

impl State<FSMAddress, MotionContext, FSMCommand, FSMState> for MotionHoming {
    fn process(
        &mut self,
        ctx: &mut MotionContext,
        mailbox: &mut Mailbox<FSMAddress, FSMCommand>,
        _bulletin: &Bulletin<FSMState>,
        _previous_state: Option<
            Box<dyn State<FSMAddress, MotionContext, FSMCommand, FSMState> + Send>,
        >,
    ) -> anyhow::Result<StateResult<FSMAddress, MotionContext, FSMCommand, FSMState>> {
        if let Some(state) = perform_maintenance_transition(
            mailbox,
            Box::new(MotionHoming {
                steps_left: self.steps_left,
                stall_prev: self.stall_prev,
            }),
        ) {
            return Ok(StateResult::Running(state));
        }

        if self.steps_left < 0 && ctx.motion.lmsw_is_high() {
            let steps = calculate_steps(-1.0);

            return Ok(StateResult::Running(Box::new(MotionMoving { steps })));
        }

        ctx.motion.set_stall_detection_enabled(self.stall_prev);
        ctx.motion.set_homing(false);

        if ctx.motion.lmsw_is_high() {
            log::info!(
                "Homing OK (dir={}): limit switch found",
                HOMING_DIRECTION.as_str()
            );
        } else {
            log::error!(
                "Homing FAILED (dir={}): limit switch could not be found",
                HOMING_DIRECTION.as_str()
            );

            return Ok(StateResult::Running(Box::new(MotionErrorLoop {
                    	component: Component::LimitSwitch,
                    	message: "Limit switch not found during boot homing".into(),
                    	notes: "Boot-time homing sweep completed without detecting the limit switch; tower orientation unknown".into()
                    })));
        }

        if self.steps_left < 0 {
            ctx.motion.update_position(HOME_HEADING_DEG);
            ctx.motion.force_zero_if_limit_switch_pressed();
        }

        // Align RAM and NVS with home heading after homing.
        ctx.actual_heading = ctx.switchboard.home_heading_deg;
        if PERSIST_NVS {
            SnapshotStore::new(&mut ctx.nvs, true).save_heading(ctx.switchboard.home_heading_deg);
            if ctx.motion_mode == MotionMode::EncoderGuarded {
                SnapshotStore::new(&mut ctx.nvs, true)
                    .save_encoder_snapshot(ctx.motion.encoder_ticks_adjusted());
            }
        }

        Ok(StateResult::Running(Box::new(MotionTracking)))
    }
}
