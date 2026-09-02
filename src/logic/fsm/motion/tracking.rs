use core::option::Option::None;

use ::fsm::{
    postal::{bulletin::Bulletin, mailbox::Mailbox},
    state::{State, StateResult},
};
use chrono::{DateTime, Local};
use esp_idf_svc::hal::gpio::{InputPin, OutputPin};
use log::{error, info, warn};
use motion::{
    motion::{MotionMode, MoveOutcome, STEPS_PER_REV},
    Direction, MotionEvent,
};
use network::{telemetry, telemetry::topic};

use crate::{
    config::constants::{HOME_HEADING_DEG, TRACKING_DEADBAND_DEG},
    logic::{
        encoder_fault::{EncoderFaultRecoveryTickRes, EncoderRecoverySwitches, EncoderTickContext},
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

pub(crate) struct TrackingRes {
    pub(crate) should_rehome: bool,
    pub(crate) outcome: Option<MoveOutcome>,
}

pub(crate) fn run_tracking<SLP, STP, DIR, CS, RLY, LMSW, ENCA, ENCB>(
    ctx: &mut MotionContext<SLP, STP, DIR, CS, RLY, LMSW, ENCA, ENCB>,
    mailbox: &mut Mailbox<FSMAddress, FSMCommand>,
) -> anyhow::Result<TrackingRes>
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
    if !ctx.switchboard.runtime.tracking.enabled {
        info!("Tracking disabled");
        return Ok(TrackingRes {
            should_rehome: false,
            outcome: None,
        });
    }
    let cfg = encoder_recovery_cfg(ctx);

    let res = tracking_tick(ctx, mailbox)?;

    if res.outcome != Some(MoveOutcome::Completed) {
        warn!("Last move aborted: {:?}", res.outcome);
    }
    if let Err(e) = ctx.encoder_fault.on_move_outcome(
        res.outcome.as_ref().unwrap().clone(),
        &cfg,
        &mut ctx.motion,
        &mut ctx.nvs,
        PERSIST_NVS,
    ) {
        error!("Error in encoder fault recovery: {:?}", e);
    }

    Ok(res)
}

pub(crate) fn tracking_tick<SLP, STP, DIR, CS, RLY, LMSW, ENCA, ENCB>(
    ctx: &mut MotionContext<SLP, STP, DIR, CS, RLY, LMSW, ENCA, ENCB>,
    mailbox: &mut Mailbox<FSMAddress, FSMCommand>,
) -> anyhow::Result<TrackingRes>
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
    let res = set_tower_position(ctx, ctx.actual_heading)?;

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
            Ok(TrackingRes {
                should_rehome: res.should_rehome,
                outcome: Some(MoveOutcome::Completed),
            })
        }

        Some(
            outcome @ (MoveOutcome::AbortedPowerMissing
            | MoveOutcome::AbortedStall
            | MoveOutcome::AbortedOvershoot
            | MoveOutcome::AbortedErrorLoop(_, _, _)),
        ) => Ok(TrackingRes {
            should_rehome: res.should_rehome,
            outcome: Some(outcome),
        }),
        None => {
            // No movement needed; treat as completed without writing NVS.
            if !res.tracking_done {
                log::warn!("tracking_done=false but no MoveOutcome recorded; skipping NVS persist");
            }

            // TODO: Handle these `Try` better
            if let Some(message) = res.message {
                match message {
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
                        return Ok(TrackingRes {
                            should_rehome: false,
                            outcome: Some(MoveOutcome::AbortedErrorLoop(component, message, notes)),
                        });
                    }
                    MotionEvent::CheckForOTA => {
                        mailbox.send(FSMAddress::Network, FSMCommand::PerformOTA)?;
                    }
                }
            }

            Ok(TrackingRes {
                should_rehome: res.should_rehome,
                outcome: Some(MoveOutcome::Completed),
            })
        }
    }
}

/// Build the encoder-recovery config from the switchboard.
pub(crate) fn encoder_recovery_cfg<SLP, STP, DIR, CS, RLY, LMSW, ENCA, ENCB>(
    ctx: &mut MotionContext<SLP, STP, DIR, CS, RLY, LMSW, ENCA, ENCB>,
) -> EncoderRecoverySwitches
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

pub(crate) fn run_encoder_fault<SLP, STP, DIR, CS, RLY, LMSW, ENCA, ENCB>(
    ctx: &mut MotionContext<SLP, STP, DIR, CS, RLY, LMSW, ENCA, ENCB>,
) -> anyhow::Result<EncoderFaultRecoveryTickRes>
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

pub(crate) fn sync_motion_mode_from_nvs<SLP, STP, DIR, CS, RLY, LMSW, ENCA, ENCB>(
    ctx: &mut MotionContext<SLP, STP, DIR, CS, RLY, LMSW, ENCA, ENCB>,
    stepper_switch_log: &str,
) where
    SLP: OutputPin,
    STP: OutputPin,
    DIR: OutputPin,
    CS: OutputPin,
    RLY: OutputPin,
    LMSW: InputPin + OutputPin,
    ENCA: InputPin,
    ENCB: InputPin,
{
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

pub(crate) struct SetTowerPositionRes {
    tracking_done: bool,
    should_rehome: bool,
    message: Option<MotionEvent>,
}

pub(crate) fn set_tower_position<SLP, STP, DIR, CS, RLY, LMSW, ENCA, ENCB>(
    ctx: &mut MotionContext<SLP, STP, DIR, CS, RLY, LMSW, ENCA, ENCB>,
    location: f32,
) -> anyhow::Result<SetTowerPositionRes>
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
            ctx.motion.disable()?;

            return Ok(SetTowerPositionRes {
                tracking_done: true,
                should_rehome: false,
                message: None,
            });
        }

        ctx.motion.enable()?;

        log::info!("Tracking move (|offset| > {}°)", TRACKING_DEADBAND_DEG);
        let steps = (angle_offset / 360.0) * STEPS_PER_REV;
        log::info!("Steps Needed: {}", steps as i64);
        // TODO: Either remove the motion_moving state or force these functions to abide by it
        if let Ok(move_outcome) = ctx.motion.move_by(steps as i64) {
            if move_outcome != MoveOutcome::Completed {
                ctx.motion.disable()?;
                log::warn!("Tracking move aborted: {:?}", move_outcome);
                // Return true so main does NOT persist heading/snapshot for a move that did not happen.
                return Ok(SetTowerPositionRes {
                    tracking_done: true,
                    should_rehome: false,
                    message: None,
                });
            }
        }

        ctx.motion
            .update_position((location as f64 + angle_offset) as f32);
        ctx.motion.disable()?;

        let tower_angle = location as f64 + angle_offset;

        let now = Local::now()
            .format(network::telemetry::TIME_FORMAT)
            .to_string();

        let payload = telemetry::Angle {
            current_time: &now,
            tower_angle,
        };

        Ok(SetTowerPositionRes {
            tracking_done: true,
            should_rehome: false,
            message: Some(MotionEvent::Angle(serde_json::to_string(&payload).unwrap())),
        })
    } else {
        // Sunset Operation
        if (location - HOME_HEADING_DEG).abs() < 0.01 {
            // Verify home physically when heading says home.
            if !ctx.motion.lmsw_active() {
                log::warn!(
                    "Heading near home but limit switch not pressed; verifying home by homing CCW"
                );

                return Ok(SetTowerPositionRes {
                    tracking_done: false,
                    should_rehome: true,
                    message: None,
                });
            }

            log::info!("At sleep position");
            // Ensure encoder ticks are truly 0 at home while we sleep.
            ctx.motion.force_zero_if_limit_switch_pressed();

            return Ok(SetTowerPositionRes {
                tracking_done: true,
                should_rehome: false,
                message: None,
            });
        }

        log::info!("Moving to sleep position...");

        Ok(SetTowerPositionRes {
            tracking_done: false,
            should_rehome: true,
            message: None,
        })
    }
}

pub(crate) fn daily_reset<SLP, STP, DIR, CS, RLY, LMSW, ENCA, ENCB>(
    ctx: &mut MotionContext<SLP, STP, DIR, CS, RLY, LMSW, ENCA, ENCB>,
    local_time: &DateTime<Local>,
) where
    SLP: OutputPin,
    STP: OutputPin,
    DIR: OutputPin,
    CS: OutputPin,
    RLY: OutputPin,
    LMSW: InputPin + OutputPin,
    ENCA: InputPin,
    ENCB: InputPin,
{
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

impl<SLP, STP, DIR, CS, RLY, LMSW, ENCA, ENCB>
    State<FSMAddress, MotionContext<SLP, STP, DIR, CS, RLY, LMSW, ENCA, ENCB>, FSMCommand, FSMState>
    for MotionTracking
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
        _bulletin: &Bulletin<FSMState>,
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

        if fault_res.should_rehome {
            return Ok(StateResult::Running(Box::new(MotionBeginHoming)));
        }

        if fault_res.fault_still_active {
            error!("Fault active, skipping tracking iteration");
            return Ok(StateResult::Hold);
        } else if let (Some(component), Some(message), Some(notes)) = (
            fault_res.telemetry_component,
            fault_res.telemetry_message,
            fault_res.telemetry_notes,
        ) {
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

        info!("Tracking loop duration (v1.1.5): {:?}", now.elapsed());

        let TrackingRes {
            should_rehome,
            outcome,
        } = run_tracking(ctx, mailbox)?;

        ctx.motion.need_rehome = should_rehome;
        if should_rehome {
            return Ok(StateResult::Running(Box::new(MotionBeginHoming)));
        }

        if let Some(MoveOutcome::AbortedErrorLoop(component, message, notes)) = outcome {
            error!("Catastrophic tracking error; manual intervention required");
            return Ok(StateResult::Running(Box::new(MotionErrorLoop {
                component,
                message,
                notes,
            })));
        }

        Ok(StateResult::Hold)
    }
}
