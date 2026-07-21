use core::{convert::Into, option::Option::None, time::Duration};
use std::{thread::sleep, time::Instant};

use anyhow::anyhow;
use chrono::{DateTime, Local};
use clock::Clock;
use esp_idf_hal::i2c::I2cDriver;
use esp_idf_svc::nvs::{EspDefaultNvsPartition, EspNvs, NvsDefault};
use fsm::{Channel, InitialState, State, StateResult};
use log::{error, info, warn};
use motion::motion::{Motion, MotionMode, MoveOutcome};
use network::telemetry::{topic, Component, ErrorLog, Severity};
use semver::Version;
use shared_bus::{BusManager, I2cProxy};

use crate::{
    config::{
        switchboard,
        switchboard::{Direction, Switchboard},
    },
    logic::{
        encoder_fault,
        encoder_fault::{EncoderFaultRecovery, EncoderRecoverySwitches, EncoderTickContext},
        fsm::FSMCommand::{self, MotionMoveBy, MqttPublishJson, UpdateNetworkMotionContext},
        tracking_loop::{self, TrackingTickContext},
    },
    storage::snapshot_store::SnapshotStore,
};

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
    current_version: Version,
}

impl MotionContext {
    pub fn new(
        motion: Motion<'static>,
        switchboard: Switchboard,
        nvs_partition: EspDefaultNvsPartition,
        i2c_bus: &'static BusManager<std::sync::Mutex<I2cDriver<'static>>>,
        trust_nvs_state: bool,
        current_version: Version,
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
            current_version,
            restored_from_snapshot: false,
            actual_heading: 0.0,
            encoder_fault: EncoderFaultRecovery::new(),
        }
    }
}

pub struct MotionInit;
pub struct MotionNotMoving;
pub struct MotionMoving {
    by: i64,
}
pub struct MotionHoming;
pub struct MotionErrorLoop {
    component: Component,
    message: String,
    notes: String,
}
pub struct MotionTracking;
pub struct MotionTrackingWait {
    begin: Instant,
}

impl InitialState<MotionContext, FSMCommand> for MotionInit {}

impl State<MotionContext, FSMCommand> for MotionInit {
    fn process(
        &mut self,
        ctx: &mut MotionContext,
        _channel: &mut Channel<FSMCommand>,
    ) -> anyhow::Result<StateResult<MotionContext, FSMCommand>> {
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

        Ok(StateResult::Running(Box::new(MotionHoming)))
    }
}

impl State<MotionContext, FSMCommand> for MotionHoming {
    fn process(
        &mut self,
        ctx: &mut MotionContext,
        _channel: &mut Channel<FSMCommand>,
    ) -> anyhow::Result<StateResult<MotionContext, FSMCommand>> {
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
            let limit_sw_status = match HOMING_DIRECTION {
                Direction::Cw => ctx.motion.find_limit_switch_cw()?,
                Direction::Ccw => ctx.motion.find_limit_switch_ccw()?,
            };
            match limit_sw_status {
                true => log::info!(
                    "Homing OK (dir={}): limit switch found",
                    HOMING_DIRECTION.as_str()
                ),
                false => {
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
            }
            // Align RAM and NVS with home heading after homing.
            ctx.actual_heading = ctx.switchboard.home_heading_deg;
            if PERSIST_NVS {
                SnapshotStore::new(&mut ctx.nvs, true)
                    .save_heading(ctx.switchboard.home_heading_deg);
                if ctx.motion_mode == MotionMode::EncoderGuarded {
                    SnapshotStore::new(&mut ctx.nvs, true)
                        .save_encoder_snapshot(ctx.motion.encoder_ticks_adjusted());
                }
            }
            sleep(Duration::from_secs(5));
        } else if should_home_by_mode {
            log::warn!("Homing skipped: HOMING_ENABLED=false");
            if !ctx.trust_nvs_state {
                return Ok(StateResult::Running(Box::new(MotionErrorLoop{
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

impl State<MotionContext, FSMCommand> for MotionNotMoving {
    fn process(
        &mut self,
        _ctx: &mut MotionContext,
        channel: &mut Channel<FSMCommand>,
    ) -> anyhow::Result<StateResult<MotionContext, FSMCommand>> {
        if let Ok(cmd) = channel.recv_latest() {
            match cmd {
                MotionMoveBy(by) => Ok(StateResult::Running(Box::new(MotionMoving { by }))),
                _ => Ok(StateResult::Hold),
            }
        } else {
            Ok(StateResult::Hold)
        }
    }
}

impl State<MotionContext, FSMCommand> for MotionMoving {
    fn process(
        &mut self,
        ctx: &mut MotionContext,
        _channel: &mut Channel<FSMCommand>,
    ) -> anyhow::Result<StateResult<MotionContext, FSMCommand>> {
        ctx.motion.move_by(self.by)?;

        Ok(StateResult::Running(Box::new(MotionNotMoving)))
    }
}

impl State<MotionContext, FSMCommand> for MotionErrorLoop {
    fn process(
        &mut self,
        ctx: &mut MotionContext,
        channel: &mut Channel<FSMCommand>,
    ) -> anyhow::Result<StateResult<MotionContext, FSMCommand>> {
        let now = Local::now()
            .format(network::telemetry::TIME_FORMAT)
            .to_string();

        let payload = ErrorLog {
            current_time: now.as_str(),
            log_type: "error",
            message: self.message.as_str(),
            component: Component::LimitSwitch,
            severity: Severity::Fault,
            value: None,
            unit: None,
            notes: self.notes.as_str(),
        };
        let serialized = serde_json::to_string(&payload)?;
        let topic = topic::logs_error(ctx.switchboard.device_id);
        channel.send(MqttPublishJson(serialized, topic));

        std::thread::sleep(Duration::from_mins(15));
        Ok(StateResult::Hold)
    }
}

impl State<MotionContext, FSMCommand> for MotionTracking {
    fn process(
        &mut self,
        ctx: &mut MotionContext,
        channel: &mut Channel<FSMCommand>,
    ) -> anyhow::Result<StateResult<MotionContext, FSMCommand>> {
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
            return Ok(StateResult::Running(Box::new(MotionHoming)));
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

        run_tracking(ctx, current_datetime.clone(), channel)?;

        info!("Tracking loop duration (v1.1.5): {:?}", now.elapsed());

        Ok(StateResult::Running(Box::new(MotionTrackingWait {
            begin: Instant::now(),
        })))
    }
}

impl State<MotionContext, FSMCommand> for MotionTrackingWait {
    fn process(
        &mut self,
        ctx: &mut MotionContext,
        channel: &mut Channel<FSMCommand>,
    ) -> anyhow::Result<StateResult<MotionContext, FSMCommand>> {
        channel.send(UpdateNetworkMotionContext(
            ctx.motion_mode,
            ctx.actual_heading,
        ))?;

        if self.begin.elapsed() < Duration::from_mins(5) {
            return Ok(StateResult::Hold);
        }

        Ok(StateResult::Running(Box::new(MotionTracking)))
    }
}

fn run_tracking(
    ctx: &mut MotionContext,
    current_datetime: String,
    channel: &mut Channel<FSMCommand>,
) -> anyhow::Result<()> {
    if !ctx.switchboard.runtime.tracking.enabled {
        info!("Tracking disabled");
        return Ok(());
    }
    let cfg = encoder_recovery_cfg(ctx);

    let mut tracking_ctx = TrackingTickContext {
        calculation: ctx.calculation.as_mut().unwrap(),
        current_version: ctx.current_version.clone(),
        nvs: &mut ctx.nvs,
        current_datetime,
        persist_nvs: PERSIST_NVS,
        device_id: ctx.switchboard.device_id,
        allow_ota: ctx.switchboard.effects.allow_ota,
        channel,
    };

    let outcome = tracking_loop::tick(&mut ctx.motion, &mut tracking_ctx, &mut ctx.actual_heading)?;

    if outcome != MoveOutcome::Completed {
        warn!("Last move aborted: {:?}", outcome);
    }
    if let Err(e) =
        ctx.encoder_fault
            .on_move_outcome(outcome, &cfg, &mut ctx.motion, &mut ctx.nvs, PERSIST_NVS)
    {
        error!("Error in encoder fault recovery: {:?}", e);
    }

    Ok(())
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
        current_version: ctx.current_version.clone(),
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
