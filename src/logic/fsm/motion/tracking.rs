use core::option::Option::None;

use ::fsm::{
    postal::{bulletin::Bulletin, mailbox::Mailbox},
    state::{State, StateResult},
};
use chrono::{DateTime, Local};
use log::{error, info, warn};
use motion::{
    motion::{MotionMode, MoveOutcome, STEPS_PER_REV},
    Direction, MotionEvent,
};
use network::{telemetry, telemetry::topic};

use crate::{
    config::constants::{HOME_HEADING_DEG, TRACKING_DEADBAND_DEG},
    logic::{
        encoder_fault::{EncoderRecoverySwitches, EncoderTickContext},
        fsm::{
            motion::{
                check_daily_encoder_reset, maintenance::perform_maintenance_transition,
                MotionBeginHoming, MotionContext, MotionErrorLoop, MotionTracking,
            },
            FSMAddress,
            FSMCommand::{self, MqttPublishJson},
            FSMState,
        },
    },
    storage::snapshot_store::SnapshotStore,
};

const PERSIST_NVS: bool = true;

pub(crate) fn run_tracking(
    ctx: &mut MotionContext,
    mailbox: &mut Mailbox<FSMAddress, FSMCommand>,
) -> anyhow::Result<Option<MoveOutcome>> {
    if !ctx.switchboard.runtime.tracking.enabled {
        info!("Tracking disabled");
        return Ok(None);
    }
    let cfg = encoder_recovery_cfg(ctx);

    let outcome = tracking_tick(ctx, mailbox)?;

    if outcome != MoveOutcome::Completed {
        warn!("Last move aborted: {:?}", outcome);
    }
    if let Err(e) = ctx.encoder_fault.on_move_outcome(
        outcome.clone(),
        &cfg,
        &mut ctx.motion,
        &mut ctx.nvs,
        PERSIST_NVS,
    ) {
        error!("Error in encoder fault recovery: {:?}", e);
    }

    Ok(Some(outcome))
}

pub(crate) fn tracking_tick(
    ctx: &mut MotionContext,
    mailbox: &mut Mailbox<FSMAddress, FSMCommand>,
) -> anyhow::Result<MoveOutcome> {
    let tracking_done = set_tower_position(ctx, ctx.actual_heading);

    // Persist only after completed moves.
    match ctx.motion.take_last_move_outcome() {
        Some(MoveOutcome::Completed) => {
            ctx.actual_heading = ctx.motion.location();
            SnapshotStore::new(&mut ctx.nvs, ctx.switchboard.effects.persist_nvs)
                .save_heading(ctx.actual_heading);
            if ctx.motion.motion_mode() == MotionMode::EncoderGuarded {
                SnapshotStore::new(&mut ctx.nvs, ctx.switchboard.effects.persist_nvs)
                    .save_encoder_snapshot(ctx.motion.encoder_ticks_adjusted());
            }
            Ok(MoveOutcome::Completed)
        }
        Some(
            outcome @ (MoveOutcome::AbortedPowerMissing
            | MoveOutcome::AbortedStall
            | MoveOutcome::AbortedOvershoot
            | MoveOutcome::AbortedErrorLoop(_, _, _)),
        ) => Ok(outcome),
        None => {
            let tracking_done = tracking_done?;

            // No movement needed; treat as completed without writing NVS.
            if !tracking_done.0 {
                log::warn!("tracking_done=false but no MoveOutcome recorded; skipping NVS persist");
            }

            // TODO: Handle these `Try` better
            for event in tracking_done.1 {
                match event {
                    MotionEvent::Angle(payload) => {
                        let serialized = serde_json::to_string(&payload)?;
                        let topic = topic::data_angle(ctx.switchboard.device_id);
                        mailbox.send(FSMAddress::Network, MqttPublishJson(serialized, topic))?;
                    }
                    MotionEvent::HomeErrorTicks(payload) => {
                        let serialized = serde_json::to_string(&payload)?;
                        let topic = topic::data_encoder_error_ticks(ctx.switchboard.device_id);
                        mailbox.send(FSMAddress::Network, MqttPublishJson(serialized, topic))?;
                    }
                    MotionEvent::ErrorLoop(component, message, notes) => {
                        return Ok(MoveOutcome::AbortedErrorLoop(component, message, notes));
                    }
                    MotionEvent::CheckForOTA => {
                        mailbox.send(FSMAddress::Network, FSMCommand::PerformOTA)?;
                    }
                }
            }

            Ok(MoveOutcome::Completed)
        }
    }
}

/// Build the encoder-recovery config from the switchboard.
pub(crate) fn encoder_recovery_cfg(ctx: &mut MotionContext) -> EncoderRecoverySwitches {
    EncoderRecoverySwitches {
        enabled: ctx.switchboard.runtime.encoder_recovery.enabled,
        probe_interval_secs: ctx.switchboard.runtime.encoder_recovery.probe_interval_secs,
        probe_steps: ctx.switchboard.runtime.encoder_recovery.probe_steps,
        max_drift_deg: ctx.switchboard.runtime.encoder_recovery.max_drift_deg,
        rehome_dir: match ctx.switchboard.runtime.encoder_recovery.rehome_dir {
            Direction::Cw => Direction::Cw,
            Direction::Ccw => Direction::Ccw,
        },
    }
}

pub(crate) fn run_encoder_fault(
    ctx: &mut MotionContext,
) -> anyhow::Result<(bool, Option<(telemetry::Component, String, String)>)> {
    let cfg = encoder_recovery_cfg(ctx);

    let mut tick_ctx = EncoderTickContext {
        nvs: &mut ctx.nvs,
        cfg,
        persist_nvs: PERSIST_NVS,
        device_id: ctx.switchboard.device_id.into(),
        home_heading_deg: ctx.switchboard.home_heading_deg,
    };

    let fault_active =
        ctx.encoder_fault
            .tick(&mut tick_ctx, &mut ctx.motion, &mut ctx.actual_heading)?;

    Ok(fault_active)
}

pub(crate) fn sync_motion_mode_from_nvs(ctx: &mut MotionContext, stepper_switch_log: &str) {
    let mode = SnapshotStore::new(&mut ctx.nvs, PERSIST_NVS)
        .load_tracking_mode_or_init(MotionMode::EncoderGuarded);
    if mode != ctx.motion.motion_mode {
        ctx.motion.motion_mode = mode;
        ctx.motion.set_motion_mode(ctx.motion.motion_mode);
        if ctx.motion.motion_mode == MotionMode::StepperOnly {
            info!("{}", stepper_switch_log);
            ctx.motion.need_rehome = true;
        }
    }
}

pub(crate) fn set_tower_position(
    ctx: &mut MotionContext,
    location: f32,
) -> anyhow::Result<(bool, Vec<MotionEvent>)> {
    let is_daytime = match (
        ctx.clock.as_mut().unwrap().after_sunrise(),
        ctx.clock.as_mut().unwrap().after_sunset(),
    ) {
        (Ok(after_sunrise), Ok(after_sunset)) => after_sunrise && !after_sunset,
        (Err(e), _) => return Err(e.into()),
        (_, Err(e)) => return Err(e.into()),
    };

    log::info!("Daytime: {}", is_daytime);

    if is_daytime {
        // If already at home, keep encoder zeroed before daytime tracking.
        ctx.motion.force_zero_if_limit_switch_pressed();

        let sun = ctx.clock.as_mut().unwrap().noaa_sun();
        let angle_offset_raw = sun.azimuth_in_deg() - (location as f64);

        // Clamp daytime target heading to soft limits.
        let target_raw = (location as f64) + angle_offset_raw;
        let target_clamped = if ctx.motion.soft_limits_enabled {
            let min = ctx.motion.soft_limit_min_deg as f64;
            let max = ctx.motion.soft_limit_max_deg as f64;
            if target_raw < min {
                log::warn!(
                    "SOFT_LIMIT clamp: target_raw={:.2} < min={:.2} -> clamping",
                    target_raw,
                    min
                );
                min
            } else if target_raw > max {
                log::warn!(
                    "SOFT_LIMIT clamp: target_raw={:.2} > max={:.2} -> clamping",
                    target_raw,
                    max
                );
                max
            } else {
                target_raw
            }
        } else {
            target_raw
        };

        let angle_offset = target_clamped - (location as f64);
        log::info!("Actual Location: {}", location);
        log::info!(
            "Angle Offset: {} (raw_offset={} target_raw={} target_clamped={})",
            angle_offset,
            angle_offset_raw,
            target_raw,
            target_clamped
        );
        log::info!("Sun Angle: {}", sun.azimuth_in_deg());
        // Daytime tracking: no move in deadband, otherwise step by offset.
        if angle_offset.abs() <= TRACKING_DEADBAND_DEG as f64 {
            ctx.motion.relay_off();
            return Ok((true, vec![]));
        }

        ctx.motion.relay_on();

        log::info!("Tracking move (|offset| > {}°)", TRACKING_DEADBAND_DEG);
        let steps = (angle_offset / 360.0) * STEPS_PER_REV;
        log::info!("Steps Needed: {}", steps as i64);
        // TODO: Either remove the motion_moving state or force these functions to abide by it
        if let Ok(move_outcome) = ctx.motion.move_by(steps as i64) {
            if move_outcome != MoveOutcome::Completed {
                ctx.motion.relay_off();
                log::warn!("Tracking move aborted: {:?}", move_outcome);
                // Return true so main does NOT persist heading/snapshot for a move that did not happen.
                return Ok((true, vec![]));
            }
        }

        ctx.motion
            .update_position((location as f64 + angle_offset) as f32);
        ctx.motion.relay_off();

        let tower_angle = location as f64 + angle_offset;

        let now = Local::now()
            .format(network::telemetry::TIME_FORMAT)
            .to_string();

        let payload = telemetry::Angle {
            current_time: &now,
            tower_angle,
        };

        Ok((
            true,
            vec![MotionEvent::Angle(serde_json::to_string(&payload).unwrap())],
        ))
    } else {
        let mut messages: Vec<MotionEvent> = vec![];

        // Sunset Operation
        if (location - HOME_HEADING_DEG).abs() < 0.01 {
            // Verify home physically when heading says home.
            if ctx.motion.lmsw.is_high() {
                log::warn!(
                    "Heading near home but limit switch not pressed; verifying home by homing CCW"
                );

                let mut is_ok = false;
                if let Ok(ok) = ctx.motion.find_limit_switch_ccw() {
                    is_ok = ok;
                    log::info!("Home verification homing succeeded");

                    if let Some(error_ticks) = ctx.motion.report_home_error_ticks(
                        &mut ctx.nvs,
                        ctx.switchboard.device_id,
                        ctx.switchboard.effects.persist_nvs,
                    ) {
                        messages.push(error_ticks)
                    }
                }
                if !is_ok {
                    log::error!("Home verification failed: limit switch could not be found");

                    let component = telemetry::Component::LimitSwitch;
                    let msg =
                        String::from("Limit switch not found during sunset home verification");
                    let notes =
                        String::from("Heading indicated home at sunset but the limit switch did not confirm; re-verification homing sweep failed.");

                    messages.push(MotionEvent::ErrorLoop(component, msg, notes));

                    return Ok((false, messages));
                }
            }

            log::info!("At sleep position");
            // Ensure encoder ticks are truly 0 at home while we sleep.
            ctx.motion.force_zero_if_limit_switch_pressed();

            Ok((true, messages))
        } else {
            log::info!("Moving to sleep position...");
            if let Ok(limit_sw_status) = ctx.motion.find_limit_switch_ccw() {
                match limit_sw_status {
                    true => {
                        log::info!("Limit switch has returned true");
                        if let Some(error_ticks) = ctx.motion.report_home_error_ticks(
                            &mut ctx.nvs,
                            ctx.switchboard.device_id,
                            ctx.switchboard.effects.persist_nvs,
                        ) {
                            messages.push(error_ticks)
                        }
                    }
                    false => {
                        log::error!(
                            "Limit switch has returned false, limit switch could not be found"
                        );
                        let component = telemetry::Component::LimitSwitch;
                        let msg =
                            String::from("Limit switch not found during move-to-sleep homing");
                        let notes = String::from(
                            "End-of-day homing sweep failed to locate the limit switch.",
                        );

                        messages.push(MotionEvent::ErrorLoop(component, msg, notes));

                        return Ok((false, messages));
                    }
                }
            }

            log::info!("Tower has reached sleep position");

            Ok((true, messages))
        }
    }
}

pub(crate) fn daily_reset(ctx: &mut MotionContext, local_time: &DateTime<Local>) {
    let reset_occurred = check_daily_encoder_reset(&mut ctx.nvs, local_time, PERSIST_NVS);
    if reset_occurred {
        ctx.motion.motion_mode = SnapshotStore::new(&mut ctx.nvs, PERSIST_NVS)
            .load_tracking_mode_or_init(MotionMode::EncoderGuarded);
        ctx.motion.set_motion_mode(ctx.motion.motion_mode);
        ctx.encoder_fault.set_mode_switched_daily(false);
        let mode_str = match ctx.motion.motion_mode {
            MotionMode::StepperOnly => "StepperOnly",
            MotionMode::EncoderGuarded => "EncoderGuarded",
        };
        info!("Daily reset: Motion mode updated to {}", mode_str);
    }
}

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
        mailbox.send(
            FSMAddress::Network,
            FSMCommand::UpdateNetworkMotionContext(ctx.motion.motion_mode, ctx.actual_heading),
        )?;

        if let Some(state) = perform_maintenance_transition(mailbox, Box::new(MotionTracking)) {
            return Ok(StateResult::Running(state));
        }

        let local_time = rtc::timezone::local_time();
        let current_datetime = format!("{}", local_time.format("%d/%m/%Y %H:%M:%S"));

        daily_reset(ctx, &local_time);

        if ctx.motion.is_rehome_pending()? {
            info!("StepperOnly mode detected - re-homing to establish known position");
            return Ok(StateResult::Running(Box::new(MotionBeginHoming)));
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
            error!("Fault active, skipping tracking iteration");
            return Ok(StateResult::Hold);
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
        if ctx.motion.is_rehome_pending()? {
            info!("StepperOnly mode detected - re-homing to establish known position (CCW)");
            return Ok(StateResult::Running(Box::new(MotionBeginHoming)));
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

        return Ok(StateResult::Hold);
    }
}
