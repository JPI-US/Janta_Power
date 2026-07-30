use core::{convert::Into, option::Option::None, time::Duration};
use std::thread::sleep;

use ::fsm::{postal::mailbox::Mailbox, state::State};
use chrono::{DateTime, Local};
use log::{error, info, warn};
use motion::{
    motion::{MotionMode, MoveOutcome, STEPS_PER_REV},
    MotionEvent,
};
use network::telemetry::{topic, Angle, Component};

use crate::{
    config::{
        constants::{HOME_HEADING_DEG, TRACKING_DEADBAND_DEG},
        switchboard::{self, Direction},
    },
    logic::{
        encoder_fault::{self, EncoderRecoverySwitches, EncoderTickContext},
        fsm::{
            motion::{MotionContext, MotionMaintenance},
            FSMAddress,
            FSMCommand::{self, MqttPublishJson, PerformOTA},
            FSMState,
        },
    },
    storage::snapshot_store::SnapshotStore,
};

const PERSIST_NVS: bool = true;

/// Reset daily encoder mode at day rollover.
pub(crate) fn check_daily_encoder_reset<T: esp_idf_svc::nvs::NvsPartitionId>(
    nvs: &mut esp_idf_svc::nvs::EspNvs<T>,
    local_time: &DateTime<Local>,
    persist_nvs: bool,
) -> bool {
    let mut snapshot_store = SnapshotStore::new(nvs, persist_nvs);

    let encoder_daily_mode = snapshot_store.load_encoder_daily_mode();
    if !encoder_daily_mode {
        return false;
    }

    let current_date = local_time.format("%Y-%m-%d").to_string();

    let stored_date = snapshot_store.load_encoder_mode_reset_date();

    match stored_date {
        Some(stored) if stored != current_date => {
            info!("Daily reset: New day detected (stored={}, current={}), resetting encoder mode to EncoderGuarded", stored, current_date);
            snapshot_store.save_tracking_mode(MotionMode::EncoderGuarded);
            snapshot_store.save_encoder_daily_mode(false);
            snapshot_store.save_encoder_mode_reset_date(&current_date);
            true
        }
        Some(_stored) => false,
        None => {
            warn!("encoder_daily_mode is true but no reset_date found in NVS; initializing reset_date");
            snapshot_store.save_encoder_mode_reset_date(&current_date);
            false
        }
    }
}

pub(crate) enum MaintenanceAction {
    Moving(Direction),
    Idle,
}

pub(crate) fn perform_maintenance_transition(
    mailbox: &mut Mailbox<FSMAddress, FSMCommand>,
    return_to: Box<dyn State<FSMAddress, MotionContext, FSMCommand, FSMState>>,
) -> Option<Box<dyn State<FSMAddress, MotionContext, FSMCommand, FSMState>>> {
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
                        mailbox.send(FSMAddress::Network, PerformOTA)?;
                    }
                }
            }

            Ok(MoveOutcome::Completed)
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
        let target_clamped = if ctx.motion.is_soft_limits_enabled() {
            let min = ctx.motion.soft_limit_min_deg() as f64;
            let max = ctx.motion.soft_limit_max_deg() as f64;
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

        let payload = Angle {
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
            if ctx.motion.lmsw_is_high() {
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

                    let component = Component::LimitSwitch;
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
                        let component = Component::LimitSwitch;
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

/// Build the encoder-recovery config from the switchboard.
pub(crate) fn encoder_recovery_cfg(ctx: &mut MotionContext) -> EncoderRecoverySwitches {
    EncoderRecoverySwitches {
        enabled: ctx.switchboard.runtime.encoder_recovery.enabled,
        probe_interval_secs: ctx.switchboard.runtime.encoder_recovery.probe_interval_secs,
        probe_steps: ctx.switchboard.runtime.encoder_recovery.probe_steps,
        max_drift_deg: ctx.switchboard.runtime.encoder_recovery.max_drift_deg,
        rehome_dir: match ctx.switchboard.runtime.encoder_recovery.rehome_dir {
            switchboard::Direction::Cw => encoder_fault::Direction::Cw,
            switchboard::Direction::Ccw => encoder_fault::Direction::Ccw,
        },
    }
}

pub(crate) fn run_encoder_fault(
    ctx: &mut MotionContext,
) -> anyhow::Result<(bool, Option<(Component, String, String)>)> {
    let cfg = encoder_recovery_cfg(ctx);

    let mut tick_ctx = EncoderTickContext {
        nvs: &mut ctx.nvs,
        cfg,
        persist_nvs: PERSIST_NVS,
        device_id: ctx.switchboard.device_id.into(),
        home_heading_deg: ctx.switchboard.home_heading_deg,
    };

    let fault_active = ctx.encoder_fault.tick(
        &mut tick_ctx,
        &mut ctx.motion,
        ctx.motion_mode,
        &mut ctx.actual_heading,
    )?;

    Ok(fault_active)
}

pub(crate) fn sync_motion_mode_from_nvs(ctx: &mut MotionContext, stepper_switch_log: &str) {
    let mode = SnapshotStore::new(&mut ctx.nvs, PERSIST_NVS)
        .load_tracking_mode_or_init(MotionMode::EncoderGuarded);
    if mode != ctx.motion_mode {
        ctx.motion_mode = mode;
        ctx.motion.set_motion_mode(ctx.motion_mode);
        if ctx.motion_mode == MotionMode::StepperOnly {
            info!("{}", stepper_switch_log);
            ctx.need_rehome = true;
        }
    }
}

pub(crate) fn rehome_if_pending(
    ctx: &mut MotionContext,
    intro_log: &str,
) -> anyhow::Result<(bool, Option<(Component, String, String)>)> {
    if !(ctx.need_rehome && ctx.motion_mode == MotionMode::StepperOnly) {
        return Ok((false, None));
    }
    info!("{}", intro_log);
    const HOMING_DIRECTION: Direction = Direction::Ccw;
    let limit_sw_status = match HOMING_DIRECTION {
        Direction::Cw => ctx.motion.find_limit_switch_cw(),
        Direction::Ccw => ctx.motion.find_limit_switch_ccw(),
    }?;
    match limit_sw_status {
        true => {
            info!(
                "Re-homing OK (dir={}): limit switch found",
                HOMING_DIRECTION.as_str()
            );
            ctx.actual_heading = ctx.switchboard.home_heading_deg;
            ctx.motion.update_position(ctx.actual_heading);
            if PERSIST_NVS {
                SnapshotStore::new(&mut ctx.nvs, PERSIST_NVS).save_heading(ctx.actual_heading);
            }
            ctx.need_rehome = false;
        }
        false => {
            error!(
                "Re-homing FAILED (dir={}): limit switch could not be found",
                HOMING_DIRECTION.as_str()
            );

            return Ok((false, Some((Component::LimitSwitch, "Re-home failed after encoder recovery".into(), "Encoder recovery completed but subsequent re-home could not locate the limit switch.".into()))));
        }
    }
    sleep(Duration::from_secs(2));
    Ok((true, None))
}

pub(crate) fn detect_stepper_transition(ctx: &mut MotionContext) {
    if ctx.motion_mode == MotionMode::StepperOnly
        && ctx.previous_motion_mode != MotionMode::StepperOnly
    {
        info!("Motion mode switched to StepperOnly - re-homing required");
        ctx.need_rehome = true;
    }
    ctx.previous_motion_mode = ctx.motion_mode;
}

pub(crate) fn daily_reset(ctx: &mut MotionContext, local_time: &DateTime<Local>) {
    let reset_occurred = check_daily_encoder_reset(&mut ctx.nvs, local_time, PERSIST_NVS);
    if reset_occurred {
        ctx.motion_mode = SnapshotStore::new(&mut ctx.nvs, PERSIST_NVS)
            .load_tracking_mode_or_init(MotionMode::EncoderGuarded);
        ctx.motion.set_motion_mode(ctx.motion_mode);
        ctx.encoder_fault.set_mode_switched_daily(false);
        let mode_str = match ctx.motion_mode {
            MotionMode::StepperOnly => "StepperOnly",
            MotionMode::EncoderGuarded => "EncoderGuarded",
        };
        info!("Daily reset: Motion mode updated to {}", mode_str);
    }
}
