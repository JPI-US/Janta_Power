use core::{convert::Into, option::Option::None, time::Duration};
use std::{thread::sleep, time::Instant};

use anyhow::anyhow;
use chrono::{DateTime, Local};
use clock::Clock;
use esp_idf_hal::i2c::I2cDriver;
use esp_idf_svc::nvs::{EspDefaultNvsPartition, EspNvs, NvsDefault};
use fsm::{
    postal::{bulletin::Bulletin, mailbox::Mailbox},
    state::{InitialState, State, StateResult},
};
use log::{error, info, warn};
use motion::{
    motion::{calculate_steps, Motion, MotionMode, MoveOutcome, STEPS_PER_REV},
    MotionEvent,
};
use network::telemetry::{topic, Angle, Component, ErrorLog, Severity};
use shared_bus::{BusManager, I2cProxy};

use crate::{
    config::{
        constants::{HOME_HEADING_DEG, TRACKING_DEADBAND_DEG},
        switchboard::{self, Direction, Switchboard},
    },
    logic::{
        encoder_fault::{self, EncoderFaultRecovery, EncoderRecoverySwitches, EncoderTickContext},
        fsm::{
            FSMAddress,
            FSMCommand::{self, MqttPublishJson, PerformOTA, UpdateNetworkMotionContext},
            FSMState,
        },
    },
    storage::snapshot_store::SnapshotStore,
};

// TODO: Shouldn't create::config::constants be in switchboard?

// TODO: Replace const
const PERSIST_NVS: bool = true;
const POWER_ON: bool = true;
// Homing policy:
// - StepperOnly: always home
// - EncoderGuarded: home when snapshot restore is unavailable/untrusted
// - EncoderGuarded: if NVS claims mechanical home (≈ home_heading_deg), require limit
//   switch active before skipping homing; otherwise re-home like snapshot miss
const HOMING_DIRECTION: Direction = Direction::Ccw;
/// Same tolerance as motion sunset home check (`location` vs `HOME_HEADING_DEG`).
const HOME_HEADING_VERIFY_EPS_DEG: f32 = 0.01;

pub struct MotionContext {
    motion: Motion<'static>,
    switchboard: Switchboard,
    nvs: EspNvs<NvsDefault>,
    i2c_bus: &'static BusManager<std::sync::Mutex<I2cDriver<'static>>>,
    calculation: Option<Clock<I2cProxy<'static, std::sync::Mutex<I2cDriver<'static>>>>>,
    trust_nvs_state: bool,
    motion_mode: MotionMode,
    previous_motion_mode: MotionMode,
    restored_from_snapshot: bool,
    actual_heading: f32,
    encoder_fault: EncoderFaultRecovery,
    need_rehome: bool,
    clock: Option<Clock<I2cProxy<'static, std::sync::Mutex<I2cDriver<'static>>>>>,
}

impl MotionContext {
    pub fn new(
        motion: Motion<'static>,
        switchboard: Switchboard,
        nvs_partition: EspDefaultNvsPartition,
        i2c_bus: &'static BusManager<std::sync::Mutex<I2cDriver<'static>>>,
        trust_nvs_state: bool,
    ) -> Self {
        let nvs = match EspNvs::new(nvs_partition, "storage", true) {
            Ok(nvs) => {
                info!("Got namespace {:?} from default partition", "storage");
                nvs
            }
            Err(e) => Err(anyhow!("Could't get namespace {:?}", e)).expect("Failed to get NVS"),
        };

        Self {
            motion,
            switchboard,
            nvs,
            i2c_bus,
            calculation: None,
            trust_nvs_state,
            motion_mode: MotionMode::EncoderGuarded,
            previous_motion_mode: MotionMode::EncoderGuarded,
            need_rehome: false,
            restored_from_snapshot: false,
            actual_heading: 0.0,
            encoder_fault: EncoderFaultRecovery::new(),
            clock: None,
        }
    }
}

pub struct MotionInit;
pub struct MotionBeginHoming;
pub struct MotionHoming {
    stall_prev: bool,
    steps_left: i64,
}

pub struct MotionMoving {
    steps: i64,
}
pub struct MotionErrorLoop {
    component: Component,
    message: String,
    notes: String,
}
pub struct MotionTracking;
pub struct MotionTrackingWait {
    begin: Instant,
}
pub struct MotionMaintenance {
    action: MaintenanceAction,
    return_to:
        Option<Box<dyn State<FSMAddress, MotionContext, FSMCommand, FSMState> + Send + 'static>>,
}

impl InitialState<FSMAddress, MotionContext, FSMCommand, FSMState> for MotionInit {}

impl State<FSMAddress, MotionContext, FSMCommand, FSMState> for MotionInit {
    fn process(
        &mut self,
        ctx: &mut MotionContext,
        _mailbox: &mut Mailbox<FSMAddress, FSMCommand>,
        _bulletin: &mut Bulletin<FSMState>,
        _previous_state: Option<
            Box<dyn State<FSMAddress, MotionContext, FSMCommand, FSMState> + Send>,
        >,
    ) -> anyhow::Result<StateResult<FSMAddress, MotionContext, FSMCommand, FSMState>> {
        // Tower location — seeded from `TOWER_LATITUDE` / `TOWER_LONGITUDE` in
        // `.env` via `Switchboard`. When `PERSIST_NVS` is on, the switchboard
        // defaults are (re)written into NVS on every boot, so updating `.env` and
        // reflashing updates the tower coordinates on the next boot.
        let tower_latitude: f64 = ctx.switchboard.default_tower_latitude;

        if PERSIST_NVS {
            match ctx
                .nvs
                .set_str("tower_latitude", &tower_latitude.to_string())
            {
                Ok(_) => info!("Tower latitude has been updated"),
                Err(e) => error!("Tower latitude was not updated {:?}", e),
            };
        }

        let tower_longitude: f64 = ctx.switchboard.default_tower_longitude;

        if PERSIST_NVS {
            match ctx
                .nvs
                .set_str("tower_longitude", &tower_longitude.to_string())
            {
                Ok(_) => info!("Tower longitude has been updated"),
                Err(e) => error!("Tower longitude was not updated {:?}", e),
            };
        }

        let mut lat_buf = [0u8; 64];
        let mut lon_buf = [0u8; 64];

        let latitude = ctx
            .nvs
            .get_str("tower_latitude", &mut lat_buf)?
            .unwrap_or("0")
            .parse()
            .unwrap_or(0.0);

        let longitude = ctx
            .nvs
            .get_str("tower_longitude", &mut lon_buf)?
            .unwrap_or("0")
            .parse()
            .unwrap_or(0.0);

        let altitude: f64 = 0.0;

        info!(
            "Retrieved latitude: {}, and longitude: {}",
            latitude, longitude
        );

        info!(
            "Device: {}, Lat: {}, Lon: {}, Alt: {}",
            ctx.switchboard.device_id, latitude, longitude, altitude
        );

        ctx.calculation = Some(Clock::new(
            ctx.i2c_bus.acquire_i2c(),
            latitude,
            longitude,
            altitude,
        ));

        // Daily encoder mode reset before mode load
        check_daily_encoder_reset(&mut ctx.nvs, &rtc::timezone::local_time(), PERSIST_NVS);

        // Motion mode from NVS, default EncoderGuarded
        let motion_mode = SnapshotStore::new(&mut ctx.nvs, PERSIST_NVS)
            .load_tracking_mode_or_init(MotionMode::EncoderGuarded);

        ctx.motion.set_motion_mode(motion_mode);
        ctx.motion.set_motor_power_on(POWER_ON);
        info!(
            "Motion mode: {:?}",
            match motion_mode {
                MotionMode::StepperOnly => "StepperOnly",
                MotionMode::EncoderGuarded => "EncoderGuarded",
            }
        );

        // State restoration
        ctx.actual_heading = if ctx.trust_nvs_state {
            let h = SnapshotStore::new(&mut ctx.nvs, PERSIST_NVS)
                .load_heading_or_init(ctx.switchboard.home_heading_deg);
            info!("Restored heading from NVS: {}", h);
            h
        } else {
            info!("Skipping heading restore: NVS state untrusted");
            ctx.switchboard.home_heading_deg
        };

        // Keep motion state aligned with restored heading.
        ctx.motion.update_position(ctx.actual_heading);

        // Restore encoder snapshot only in EncoderGuarded mode.
        if ctx.trust_nvs_state && motion_mode == MotionMode::EncoderGuarded {
            if let Some(enc_ticks_adj) =
                SnapshotStore::new(&mut ctx.nvs, PERSIST_NVS).load_encoder_snapshot()
            {
                // Restore zero offset so adjusted ticks equal saved snapshot.
                let raw = ctx.motion.encoder_ticks_raw();
                ctx.motion.set_encoder_zero_offset(raw - enc_ticks_adj);
                info!(
                    "Restored encoder snapshot ticks from NVS: {}",
                    enc_ticks_adj
                );
                ctx.restored_from_snapshot = true;
            } else {
                info!("No valid encoder snapshot found in NVS; will home normally.");
            }
        } else {
            if !ctx.trust_nvs_state {
                info!("Skipping encoder snapshot restore: NVS state untrusted");
            } else {
                info!("Motion mode is StepperOnly: skipping encoder snapshot restore");
            }
        }

        // Encoder fault recovery
        let mut encoder_fault = EncoderFaultRecovery::new();
        let encoder_daily_mode =
            SnapshotStore::new(&mut ctx.nvs, PERSIST_NVS).load_encoder_daily_mode();
        encoder_fault.set_mode_switched_daily(encoder_daily_mode);

        // clock
        ctx.clock = Some(Clock::new(
            ctx.i2c_bus.acquire_i2c(),
            latitude,
            longitude,
            altitude,
        ));

        Ok(StateResult::Running(Box::new(MotionBeginHoming)))
    }
}

impl State<FSMAddress, MotionContext, FSMCommand, FSMState> for MotionBeginHoming {
    fn process(
        &mut self,
        ctx: &mut MotionContext,
        mailbox: &mut Mailbox<FSMAddress, FSMCommand>,
        _bulletin: &mut Bulletin<FSMState>,
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
        _bulletin: &mut Bulletin<FSMState>,
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

impl State<FSMAddress, MotionContext, FSMCommand, FSMState> for MotionMoving {
    fn process(
        &mut self,
        ctx: &mut MotionContext,
        _mailbox: &mut Mailbox<FSMAddress, FSMCommand>,
        _bulletin: &mut Bulletin<FSMState>,
        previous_state: Option<
            Box<dyn State<FSMAddress, MotionContext, FSMCommand, FSMState> + Send>,
        >,
    ) -> anyhow::Result<StateResult<FSMAddress, MotionContext, FSMCommand, FSMState>> {
        match ctx.motion.move_by(self.steps) {
            Ok(_) => {}
            Err(e) => {
                error!("Failed to move motor with error: {e}");
            }
        }

        if let Some(previous_state) = previous_state {
            Ok(StateResult::Running(previous_state))
        } else {
            error!("No previous state found after motion movement; re-initializing");
            Ok(StateResult::Running(Box::new(MotionInit)))
        }
    }
}

impl State<FSMAddress, MotionContext, FSMCommand, FSMState> for MotionErrorLoop {
    fn process(
        &mut self,
        ctx: &mut MotionContext,
        mailbox: &mut Mailbox<FSMAddress, FSMCommand>,
        _bulletin: &mut Bulletin<FSMState>,
        _previous_state: Option<
            Box<dyn State<FSMAddress, MotionContext, FSMCommand, FSMState> + Send>,
        >,
    ) -> anyhow::Result<StateResult<FSMAddress, MotionContext, FSMCommand, FSMState>> {
        let now = Local::now()
            .format(network::telemetry::TIME_FORMAT)
            .to_string();

        let payload = ErrorLog {
            current_time: now.as_str(),
            log_type: "error",
            message: self.message.as_str(),
            component: self.component,
            severity: Severity::Fault,
            value: None,
            unit: None,
            notes: self.notes.as_str(),
        };
        let serialized = serde_json::to_string(&payload)?;
        let topic = topic::logs_error(ctx.switchboard.device_id);
        let _ = mailbox.send(FSMAddress::Network, MqttPublishJson(serialized, topic));

        std::thread::sleep(Duration::from_mins(15));
        Ok(StateResult::Hold)
    }
}

impl State<FSMAddress, MotionContext, FSMCommand, FSMState> for MotionTracking {
    fn process(
        &mut self,
        ctx: &mut MotionContext,
        mailbox: &mut Mailbox<FSMAddress, FSMCommand>,
        _bulletin: &mut Bulletin<FSMState>,
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
        _bulletin: &mut Bulletin<FSMState>,
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

impl State<FSMAddress, MotionContext, FSMCommand, FSMState> for MotionMaintenance {
    fn process(
        &mut self,
        ctx: &mut MotionContext,
        mailbox: &mut Mailbox<FSMAddress, FSMCommand>,
        _bulletin: &mut Bulletin<FSMState>,
        _previous_state: Option<
            Box<dyn State<FSMAddress, MotionContext, FSMCommand, FSMState> + Send>,
        >,
    ) -> anyhow::Result<StateResult<FSMAddress, MotionContext, FSMCommand, FSMState>> {
        let Some(action) = check_maintenance(mailbox) else {
            return Ok(StateResult::Hold);
        };

        match action {
            MaintenanceAction::Idle => match self.action {
                MaintenanceAction::Moving(_) => {
                    self.action = MaintenanceAction::Idle;
                    Ok(StateResult::Hold)
                }

                MaintenanceAction::Idle => match self.return_to.take() {
                    Some(state) => Ok(StateResult::Running(state)),
                    None => {
                        error!(
                                "No return state found in MotionMaintenance; falling back to MotionInit"
                            );
                        Ok(StateResult::Running(Box::new(MotionInit)))
                    }
                },
            },

            MaintenanceAction::Moving(direction) => {
                self.action = MaintenanceAction::Moving(direction);

                ctx.motion.move_by(match direction {
                    Direction::Ccw => -150_000,
                    Direction::Cw => 150_000,
                })?;

                Ok(StateResult::Hold)
            }
        }
    }
}

enum MaintenanceAction {
    Moving(Direction),
    Idle,
}

fn perform_maintenance_transition(
    mailbox: &mut Mailbox<FSMAddress, FSMCommand>,
    return_to: Box<dyn State<FSMAddress, MotionContext, FSMCommand, FSMState>>,
) -> Option<Box<dyn State<FSMAddress, MotionContext, FSMCommand, FSMState>>> {
    Some(Box::new(MotionMaintenance {
        action: check_maintenance(mailbox)?,
        return_to: Some(return_to),
    }))
}

fn check_maintenance(mailbox: &mut Mailbox<FSMAddress, FSMCommand>) -> Option<MaintenanceAction> {
    match mailbox.receive_latest().ok()? {
        FSMCommand::CCWPressed => Some(MaintenanceAction::Moving(Direction::Ccw)),
        FSMCommand::CWPressed => Some(MaintenanceAction::Moving(Direction::Cw)),
        FSMCommand::MaintenancePressed => Some(MaintenanceAction::Idle),
        _ => None,
    }
}

fn run_tracking(
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

fn tracking_tick(
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

fn set_tower_position(
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
fn encoder_recovery_cfg(ctx: &mut MotionContext) -> EncoderRecoverySwitches {
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

fn run_encoder_fault(
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

fn sync_motion_mode_from_nvs(ctx: &mut MotionContext, stepper_switch_log: &str) {
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

fn rehome_if_pending(
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

fn detect_stepper_transition(ctx: &mut MotionContext) {
    if ctx.motion_mode == MotionMode::StepperOnly
        && ctx.previous_motion_mode != MotionMode::StepperOnly
    {
        info!("Motion mode switched to StepperOnly - re-homing required");
        ctx.need_rehome = true;
    }
    ctx.previous_motion_mode = ctx.motion_mode;
}

fn daily_reset(ctx: &mut MotionContext, local_time: &DateTime<Local>) {
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

/// Reset daily encoder mode at day rollover.
fn check_daily_encoder_reset<T: esp_idf_svc::nvs::NvsPartitionId>(
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
