use core::{convert::Into, error, fmt, option::Option::None, result::Result::Err};
use std::{
    thread,
    time::{Duration, Instant},
};

use anyhow::{anyhow, Context};
use chrono::{DateTime, Local};
use clock::Clock;
use esp_idf_svc::{
    eventloop::EspSystemEventLoop,
    hal::{
        delay::Ets,
        gpio::{Gpio4, Gpio5, Gpio6, Input, PinDriver},
    },
    log::EspLogger,
    nvs::{EspDefaultNvsPartition, EspNvs},
    ota::EspOta,
};
use hdc1080::Hdc1080;
use log::{error, info, warn};
use motion::{Motion, MotionMode, MoveOutcome};
use network::mqtt::Mqtt;
use ota::OtaUpdater;
use rgb_led::Led;
use rtc::Rtc;
use semver::Version;
use wifi::wifi::{Wifi, WifiState};

use crate::{
    config::{constants, switchboard, switchboard::Switchboard},
    hardware::{peripheral_map::PeripheralMap, temperature::report_system_temperature},
    logic::{
        encoder_fault,
        encoder_fault::{
            Direction, EncoderFaultRecovery, EncoderRecoverySwitches, EncoderTickContext,
        },
        tracking_loop,
        tracking_loop::TrackingTickContext,
    },
    services::{commands, telemetry::error_loop, transport},
    storage::snapshot_store::SnapshotStore,
};

// Required by embassy_executor when esp-idf-svc embassy features are enabled.
#[no_mangle]
pub extern "C" fn __pender() {}

/// Long-lived runtime state for the tower, owned across the main tracking loop.
///
/// Built once at the end of boot (in `main`) by gathering the initialized
/// hardware/state locals, then driven by the loop. Generic over the shared-I2C
/// proxy type `I2C` so the bus-backed devices (`calculation`, `temp_sensor`)
/// can be stored without naming the verbose `shared_bus` proxy type.

/// Main-loop steps, factored out of the loop body so the loop reads as a thin
/// sequence of named operations. Each method owns one concern and mutates the
/// shared state through `&mut self` (disjoint field borrows let one method touch
/// `motion`, `nvs`, `mqtt`, ... at once). Bound to `I2C: I2c` because the
/// tracking and temperature steps drive bus-backed devices.
impl<I2C: embedded_hal::i2c::I2c> Tower<I2C> {
    /// Every loop step persists to NVS (mirrors the boot-phase `PERSIST_NVS`).

    /// Day-rollover reset of the daily encoder mode (see [`check_daily_encoder_reset`]).
    fn daily_reset(&mut self, local_time: &DateTime<Local>) {
        let reset_occurred =
            check_daily_encoder_reset(&mut self.nvs, local_time, Self::PERSIST_NVS);
        if reset_occurred {
            self.motion_mode = SnapshotStore::new(&mut self.nvs, Self::PERSIST_NVS)
                .load_tracking_mode_or_init(MotionMode::EncoderGuarded);
            self.motion.set_motion_mode(self.motion_mode);
            self.encoder_fault.set_mode_switched_daily(false);
            let mode_str = match self.motion_mode {
                MotionMode::StepperOnly => "StepperOnly",
                MotionMode::EncoderGuarded => "EncoderGuarded",
            };
            info!("Daily reset: Motion mode updated to {}", mode_str);
        }
    }

    /// Flag a re-home when the mode has just transitioned into `StepperOnly`.
    fn detect_stepper_transition(&mut self) {
        if self.motion_mode == MotionMode::StepperOnly
            && self.previous_motion_mode != MotionMode::StepperOnly
        {
            info!("Motion mode switched to StepperOnly - re-homing required");
            self.need_rehome = true;
        }
        self.previous_motion_mode = self.motion_mode;
    }

    /// Reload the motion mode from NVS in case another path switched it; if it
    /// changed to `StepperOnly`, schedule a re-home. `stepper_switch_log` is the
    /// message logged on that transition.
    fn sync_motion_mode_from_nvs(&mut self, stepper_switch_log: &str) {
        let mode = SnapshotStore::new(&mut self.nvs, Self::PERSIST_NVS)
            .load_tracking_mode_or_init(MotionMode::EncoderGuarded);
        if mode != self.motion_mode {
            self.motion_mode = mode;
            self.motion.set_motion_mode(self.motion_mode);
            if self.motion_mode == MotionMode::StepperOnly {
                info!("{}", stepper_switch_log);
                self.need_rehome = true;
            }
        }
    }

    /// Re-home to the limit switch if a `StepperOnly` re-home is pending.
    /// Returns `Ok(true)` when a re-home ran (caller should `continue` the loop).
    /// `intro_log` is logged when the sweep starts.
    fn rehome_if_pending(&mut self, intro_log: &str) -> anyhow::Result<bool> {
        if !(self.need_rehome && self.motion_mode == MotionMode::StepperOnly) {
            return Ok(false);
        }
        info!("{}", intro_log);
        const HOMING_DIRECTION: Direction = Direction::Ccw;
        let limit_sw_status = match HOMING_DIRECTION {
            Direction::Cw => self.motion.find_limit_switch_cw()?,
            Direction::Ccw => self.motion.find_limit_switch_ccw()?,
        };
        match limit_sw_status {
            true => {
                info!(
                    "Re-homing OK (dir={}): limit switch found",
                    HOMING_DIRECTION.as_str()
                );
                self.actual_heading = self.sw.home_heading_deg;
                self.motion.update_position(self.actual_heading);
                if Self::PERSIST_NVS {
                    SnapshotStore::new(&mut self.nvs, Self::PERSIST_NVS)
                        .save_heading(self.actual_heading);
                }
                self.need_rehome = false;
            }
            false => {
                error!(
                    "Re-homing FAILED (dir={}): limit switch could not be found",
                    HOMING_DIRECTION.as_str()
                );
                error_loop(
                    self.sw.device_id,
                    &mut self.mqtt,
                    network::telemetry::Component::LimitSwitch,
                    "Re-home failed after encoder recovery",
                    "Encoder recovery completed but subsequent re-home could not locate the limit switch.",
                );
            }
        }
        thread::sleep(Duration::from_secs(2));
        Ok(true)
    }

    /// Build the encoder-recovery config from the switchboard.
    fn encoder_recovery_cfg(&self) -> EncoderRecoverySwitches {
        EncoderRecoverySwitches {
            enabled: self.sw.runtime.encoder_recovery.enabled,
            probe_interval_secs: self.sw.runtime.encoder_recovery.probe_interval_secs,
            probe_steps: self.sw.runtime.encoder_recovery.probe_steps,
            max_drift_deg: self.sw.runtime.encoder_recovery.max_drift_deg,
            rehome_dir: match self.sw.runtime.encoder_recovery.rehome_dir {
                switchboard::Direction::Cw => encoder_fault::Direction::Cw,
                switchboard::Direction::Ccw => encoder_fault::Direction::Ccw,
            },
        }
    }

    /// Run one encoder-fault probe/recovery tick. Returns `Ok(true)` when a fault
    /// is active (caller should `continue` and skip tracking this iteration).
    fn run_encoder_fault(&mut self) -> anyhow::Result<bool> {
        let cfg = self.encoder_recovery_cfg();

        let mut ctx = EncoderTickContext {
            nvs: &mut self.nvs,
            mqtt: &mut self.mqtt,
            wifi: &mut self.wifi,
            cfg,
            current_version: self.current_version.clone(),
            persist_nvs: Self::PERSIST_NVS,
            device_id: self.sw.device_id.into(),
            home_heading_deg: self.sw.home_heading_deg,
        };

        let fault_active = self.encoder_fault.tick(
            &mut ctx,
            &mut self.motion,
            self.motion_mode,
            &mut self.actual_heading,
        )?;
        Ok(fault_active)
    }

    /// Run one tracking tick (gated by the switchboard) and feed the move
    /// outcome back into encoder-fault recovery.
    fn run_tracking(&mut self, current_datetime: String) -> anyhow::Result<()> {
        if !self.sw.runtime.tracking.enabled {
            info!("Tracking disabled");
            return Ok(());
        }
        let cfg = self.encoder_recovery_cfg();

        let mut ctx = TrackingTickContext {
            calculation: &mut self.calculation,
            mqtt: &mut self.mqtt,
            current_version: self.current_version.clone(),
            nvs: &mut self.nvs,
            wifi: &mut self.wifi,
            current_datetime,
            persist_nvs: Self::PERSIST_NVS,
            allow_ota: self.allow_ota,
            device_id: self.sw.device_id,
        };

        let outcome = tracking_loop::tick(&mut self.motion, &mut ctx, &mut self.actual_heading)?;

        if outcome != MoveOutcome::Completed {
            warn!("Last move aborted: {:?}", outcome);
        }
        if let Err(e) = self.encoder_fault.on_move_outcome(
            outcome,
            &cfg,
            &mut self.motion,
            &mut self.nvs,
            Self::PERSIST_NVS,
        ) {
            error!("Error in encoder fault recovery: {:?}", e);
            return Err(TrackingError::ErrorInEncoderFaultRecovery.into());
        }

        Ok(())
    }

    /// Reconnect Wi-Fi if it has dropped.
    fn maintain_wifi(&mut self) -> anyhow::Result<()> {
        if self.wifi.state() == WifiState::Disconnected {
            warn!("Wifi disconnected, attempting to reconnect...");
            self.wifi.reconnect_if_disconnected()?;
        }
        Ok(())
    }

    /// Publish the periodic heartbeat (`tower/{id}/status`).
    fn publish_heartbeat(&mut self, current_datetime: &str) {
        let payload = network::telemetry::Heartbeat {
            current_time: current_datetime,
            firmware_version: &self.current_version.to_string(),
        };
        let topic = network::telemetry::topic::status(self.sw.device_id);
        let _ = network::telemetry::publish_json(&mut self.mqtt, &topic, &payload);
    }

    /// Publish system temperature telemetry when the HDC1080 is present.
    fn report_temperature(&mut self, current_datetime: &str) {
        if let Some(ref mut sensor) = self.temp_sensor {
            report_system_temperature(sensor, &mut self.mqtt, self.sw.device_id, current_datetime);
        }
    }

    /// Answer at most one queued remote command (gated by the switchboard).
    fn process_commands(&mut self) {
        if !self.sw.runtime.commands_enabled {
            return;
        }
        let motion_mode_str = match self.motion_mode {
            MotionMode::StepperOnly => "stepper_only",
            MotionMode::EncoderGuarded => "encoder_guarded",
        };
        let firmware_version = self.current_version.to_string();
        let ctx = commands::CmdCtx {
            device_id: self.sw.device_id,
            firmware_version: &firmware_version,
            mqtt_connected: self.mqtt.is_connected(),
            wifi_connected: matches!(self.wifi.state(), WifiState::Connected(_)),
            motion_mode: motion_mode_str,
            current_heading: self.actual_heading,
        };
        if let Err(e) = transport::process_one(&mut self.mqtt, self.sw.device_id, &ctx) {
            warn!("Command processing failed: {:?}", e);
        }
    }
}

#[derive(Debug)]
enum TrackingError {
    ErrorInEncoderFaultRecovery,
}

impl fmt::Display for TrackingError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::ErrorInEncoderFaultRecovery => write!(f, "Error in encoder fault recovery"),
        }
    }
}

impl error::Error for TrackingError {}

fn oldmain() -> anyhow::Result<()> {
    // PHASE 5: MOTION INITIALIZATION ------------------------------------------

    // Hardware initialization

    let temp_sensor = match Hdc1080::new(i2c_bus.acquire_i2c(), Ets) {
        Ok(mut sensor) => {
            let _ = sensor.init();
            if sensor.get_device_id().unwrap_or(0) == 0x1050 {
                info!("HDC1080 detected");
                Some(sensor)
            } else {
                warn!("HDC1080 not detected on I2C; temp telemetry disabled");
                None
            }
        }
        Err(e) => {
            warn!("HDC1080 init failed: {:?}; temp telemetry disabled", e);
            None
        }
    };

    led.display_healthy()?;

    // Runtime guardrails from switchboard
    motion.set_stall_detection_enabled(sw.runtime.guardrails.stall_detection_enabled);
    motion.set_soft_limits(
        sw.runtime.guardrails.soft_limits_enabled,
        sw.runtime.guardrails.soft_limit_min_deg,
        sw.runtime.guardrails.soft_limit_max_deg,
    );

    const POWER_ON: bool = true;

    // Daily encoder mode reset before mode load
    check_daily_encoder_reset(&mut nvs, &rtc::timezone::local_time(), PERSIST_NVS);

    // Motion mode from NVS, default EncoderGuarded
    let motion_mode = SnapshotStore::new(&mut nvs, PERSIST_NVS)
        .load_tracking_mode_or_init(MotionMode::EncoderGuarded);
    motion.set_motion_mode(motion_mode);
    motion.set_motor_power_on(POWER_ON);
    info!(
        "Motion mode: {:?}",
        match motion_mode {
            MotionMode::StepperOnly => "StepperOnly",
            MotionMode::EncoderGuarded => "EncoderGuarded",
        }
    );

    // PHASE 6: STATE RESTORATION ----------------------------------------------
    let mut actual_heading: f32 = if trust_nvs_state {
        let h = SnapshotStore::new(&mut nvs, PERSIST_NVS).load_heading_or_init(sw.home_heading_deg);
        info!("Restored heading from NVS: {}", h);
        h
    } else {
        info!("Skipping heading restore: NVS state untrusted");
        sw.home_heading_deg
    };

    // Restore encoder snapshot only in EncoderGuarded mode.
    let mut restored_from_snapshot = false;
    if trust_nvs_state && motion_mode == MotionMode::EncoderGuarded {
        if let Some(enc_ticks_adj) =
            SnapshotStore::new(&mut nvs, PERSIST_NVS).load_encoder_snapshot()
        {
            // Restore zero offset so adjusted ticks equal saved snapshot.
            let raw = motion.encoder_ticks_raw();
            motion.set_encoder_zero_offset(raw - enc_ticks_adj);
            info!(
                "Restored encoder snapshot ticks from NVS: {}",
                enc_ticks_adj
            );
            restored_from_snapshot = true;
        } else {
            info!("No valid encoder snapshot found in NVS; will home normally.");
        }
    } else {
        if !trust_nvs_state {
            info!("Skipping encoder snapshot restore: NVS state untrusted");
        } else {
            info!("Motion mode is StepperOnly: skipping encoder snapshot restore");
        }
    }

    // Keep motion state aligned with restored heading.
    motion.update_position(actual_heading);

    // PHASE 7: NORMAL MODE BOOT ACTIONS ---------------------------------------

    // Encoder fault recovery
    let mut encoder_fault = EncoderFaultRecovery::new();
    let encoder_daily_mode = SnapshotStore::new(&mut nvs, PERSIST_NVS).load_encoder_daily_mode();
    encoder_fault.set_mode_switched_daily(encoder_daily_mode);

    // Homing policy:
    // - StepperOnly: always home
    // - EncoderGuarded: home when snapshot restore is unavailable/untrusted
    // - EncoderGuarded: if NVS claims mechanical home (≈ home_heading_deg), require limit
    //   switch active before skipping homing; otherwise re-home like snapshot miss
    const HOMING_DIRECTION: Direction = Direction::Ccw;
    /// Same tolerance as motion sunset home check (`location` vs `HOME_HEADING_DEG`).
    const HOME_HEADING_VERIFY_EPS_DEG: f32 = 0.01;

    let would_skip_homing_on_snapshot =
        motion_mode == MotionMode::EncoderGuarded && restored_from_snapshot && trust_nvs_state;
    let restored_claims_mechanical_home =
        (actual_heading - sw.home_heading_deg).abs() < HOME_HEADING_VERIFY_EPS_DEG;
    let home_claim_needs_limit_verify = would_skip_homing_on_snapshot
        && restored_claims_mechanical_home
        && !motion.switch_pressed();
    if home_claim_needs_limit_verify {
        log::info!(
            "Restored heading matches home ({}) but limit switch not pressed; homing to verify",
            sw.home_heading_deg
        );
    }

    let should_home_by_mode = motion_mode == MotionMode::StepperOnly
        || !restored_from_snapshot
        || !trust_nvs_state
        || home_claim_needs_limit_verify;
    if should_home_by_mode && sw.boot.homing.enabled {
        let limit_sw_status = match HOMING_DIRECTION {
            Direction::Cw => motion.find_limit_switch_cw()?,
            Direction::Ccw => motion.find_limit_switch_ccw()?,
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
                error_loop(
                    sw.device_id,
                    &mut mqtt,
                    network::telemetry::Component::LimitSwitch,
                    "Limit switch not found during boot homing",
                    "Boot-time homing sweep completed without detecting the limit switch; tower orientation unknown.",
                );
            }
        }
        // Align RAM and NVS with home heading after homing.
        actual_heading = sw.home_heading_deg;
        if PERSIST_NVS {
            SnapshotStore::new(&mut nvs, true).save_heading(sw.home_heading_deg);
            if motion_mode == MotionMode::EncoderGuarded {
                SnapshotStore::new(&mut nvs, true)
                    .save_encoder_snapshot(motion.encoder_ticks_adjusted());
            }
        }
        thread::sleep(Duration::from_secs(5));
    } else if should_home_by_mode {
        log::warn!("Homing skipped: HOMING_ENABLED=false");
        if !trust_nvs_state {
            error_loop(
                sw.device_id,
                &mut mqtt,
                network::telemetry::Component::System,
                "Homing disabled with untrusted NVS state",
                "HOMING_ENABLED=false but persisted heading could not be trusted; manual intervention required.",
            );
        }
    } else {
        log::info!("Skipping homing: restored heading+encoder snapshot from NVS");
    }

    // Mark this boot as Normal so next boot can trust NVS.
    SnapshotStore::new(&mut nvs, true).save_last_run_normal(true);

    // PHASE 8: MAIN TRACKING LOOP ---------------------------------------------
    // Gather the long-lived state into the Tower context; the loop drives it.
    let mut tower = Tower {
        sw,
        nvs,
        motion,
        mqtt,
        wifi,
        calculation,
        temp_sensor,
        encoder_fault,
        current_version,
        motion_mode,
        actual_heading,
        allow_ota,
        previous_motion_mode: motion_mode,
        need_rehome: false,
    };

    loop {
        let local_time = rtc::timezone::local_time();
        let current_datetime = format!("{}", local_time.format("%d/%m/%Y %H:%M:%S"));

        // Daily encoder-mode reset at day rollover.
        tower.daily_reset(&local_time);

        // Re-home once when transitioning into StepperOnly.
        tower.detect_stepper_transition();
        if tower.rehome_if_pending(
            "StepperOnly mode detected - re-homing to establish known position",
        )? {
            continue;
        }

        info!("Actual Heading: {}", tower.motion.location());
        info!("Current datetime: {}", current_datetime.clone());

        let now = std::time::Instant::now();

        // Reload mode in case another path switched to StepperOnly.
        tower.sync_motion_mode_from_nvs(
            "Motion mode changed to StepperOnly - will re-home on next iteration",
        );

        if tower.run_encoder_fault()? {
            continue; // Fault active, skip tracking this iteration
        }

        // Re-check mode after recovery in case it changed during the probe.
        tower.sync_motion_mode_from_nvs(
            "Motion mode switched to StepperOnly during encoder fault recovery - will re-home",
        );

        // Re-home before tracking if StepperOnly was activated during recovery.
        if tower.rehome_if_pending(
            "StepperOnly mode detected - re-homing to establish known position (CCW)",
        )? {
            continue;
        }

        tower.run_tracking(current_datetime.clone())?;

        info!("Tracking loop duration (v1.1.5): {:?}", now.elapsed());

        tower.maintain_wifi()?;
        tower.publish_heartbeat(&current_datetime);
        tower.report_temperature(&current_datetime);
        tower.process_commands();

        const LOOP_SLEEP_SECS: u64 = 300;
        std::thread::sleep(Duration::from_secs(LOOP_SLEEP_SECS));
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
