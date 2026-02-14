use std::time::{Duration, Instant};

use esp_idf_svc::nvs::{EspNvs, NvsPartitionId};
use motion::{Motion, MotionMode, MoveOutcome};
use network::mqtt::Mqtt;
use semver::Version;
use wifi::wifi::{Wifi, WifiState};

use crate::{
    constants::HOME_HEADING_DEG,
    infra,
    infra::SnapshotStore,
};

// Simple Direction enum (replaces switchboard::Direction)
#[derive(Clone, Copy)]
pub enum Direction {
    Cw,
    Ccw,
}

impl Direction {
    pub fn as_str(&self) -> &'static str {
        match self {
            Direction::Cw => "CW",
            Direction::Ccw => "CCW",
        }
    }
}

// Simple EncoderRecoverySwitches struct (replaces switchboard::EncoderRecoverySwitches)
pub struct EncoderRecoverySwitches {
    pub enabled: bool,
    pub probe_interval_secs: u64,
    pub probe_steps: i64,
    pub max_drift_deg: f32,
    pub rehome_dir: Direction,
}

impl EncoderRecoverySwitches {
    pub const fn default() -> Self {
        Self {
            enabled: true,
            probe_interval_secs: 30,
            probe_steps: 1000,
            max_drift_deg: 5.0,
            rehome_dir: Direction::Ccw,
        }
    }
}

pub struct EncoderFaultRecovery {
    active: bool,
    next_probe_at: Option<Instant>,
    failure_count: u8,  // Consecutive encoder failures (stalls/overshoots)
    mode_switched: bool,  // True if we've permanently switched to StepperOnly
}

const MAX_ENCODER_FAILURES: u8 = 3;  // After 3 consecutive failures, switch to StepperOnly

impl EncoderFaultRecovery {
    pub const fn new() -> Self {
        Self {
            active: false,
            next_probe_at: None,
            failure_count: 0,
            mode_switched: false,
        }
    }

    pub fn on_move_outcome<T: NvsPartitionId>(
        &mut self,
        outcome: MoveOutcome,
        cfg: &EncoderRecoverySwitches,
        motion: &mut Motion<'_>,
        nvs: &mut EspNvs<T>,
        persist_nvs: bool,
    ) -> anyhow::Result<()> {
        // Skip all encoder checks if we've already switched to StepperOnly
        if self.mode_switched {
            return Ok(());
        }
        
        // Both stall and overshoot indicate encoder issues - trigger recovery
        if outcome == MoveOutcome::AbortedStall || outcome == MoveOutcome::AbortedOvershoot {
            self.failure_count += 1;
            log::warn!("Encoder fault detected (failure_count={}/{})", self.failure_count, MAX_ENCODER_FAILURES);
            
            // Check if we've hit the 3-strike limit immediately
            if self.failure_count >= MAX_ENCODER_FAILURES {
                log::error!("CRITICAL: Encoder failed {} consecutive times, switching to StepperOnly mode permanently", self.failure_count);
                self.switch_to_stepper_only(motion, nvs, persist_nvs)?;
                return Ok(());  // Mode switched, no need to start recovery
            }
            
            self.active = true;
            self.next_probe_at = Some(Instant::now() + Duration::from_secs(cfg.probe_interval_secs));
        } else if outcome == MoveOutcome::Completed {
            // Successful move resets failure count
            if self.failure_count > 0 {
                log::info!("Encoder fault cleared: successful move resets failure count");
                self.failure_count = 0;
            }
        }
        Ok(())
    }

    /// Returns `Ok(true)` if the caller should `continue` the outer loop (fault still active).
    pub fn tick<T: NvsPartitionId>(
        &mut self,
        cfg: &EncoderRecoverySwitches,
        motion: &mut Motion<'_>,
        motion_mode: MotionMode,
        actual_heading: &mut f32,
        nvs: &mut EspNvs<T>,
        mqtt: &mut Mqtt,
        wifi: &mut Wifi<'_>,
        current_version: &Version,
        publish_mqtt: bool,
        persist_nvs: bool,
    ) -> anyhow::Result<bool> {
        // Skip all encoder checks if we've already switched to StepperOnly
        if self.mode_switched {
            return Ok(false);
        }
        
        if !self.active {
            return Ok(false);
        }

        if !cfg.enabled {
            infra::Telemetry::critical_failure_loop(
                mqtt,
                b"Critical failure: encoder fault recovery disabled in switchboard!",
                publish_mqtt,
            );
        }

        let probe_interval = Duration::from_secs(cfg.probe_interval_secs);
        let now_i = Instant::now();
        let should_probe = self.next_probe_at.map(|t| now_i >= t).unwrap_or(true);

        if !should_probe {
            let t = self.next_probe_at.unwrap();
            let remaining = t.saturating_duration_since(now_i);
            log::info!("Encoder fault: waiting {:?} until next probe...", remaining);

            self.housekeeping(wifi, mqtt, current_version, publish_mqtt)?;
            std::thread::sleep(Duration::from_secs(30));
            return Ok(true);
        }

        log::info!("Encoder fault: probing for recovery...");
        let ok = motion.probe_encoder_motion(cfg.probe_steps);
        if !ok {
            log::warn!("Encoder probe failed; will retry in {:?}", probe_interval);
            self.next_probe_at = Some(now_i + probe_interval);

            self.housekeeping(wifi, mqtt, current_version, publish_mqtt)?;
            std::thread::sleep(Duration::from_secs(30));
            return Ok(true);
        }

        log::info!("Encoder probe succeeded; recomputing heading from encoder and resuming tracking.");
        self.active = false;
        self.next_probe_at = None;
        // Reset failure count on successful recovery
        self.failure_count = 0;

        let candidate_heading = motion.heading_from_encoder_ticks(HOME_HEADING_DEG);
        let drift = angle_diff_deg(candidate_heading, *actual_heading);
        log::info!(
            "Encoder recovered: candidate_heading={} prev_heading={} drift_deg={}",
            candidate_heading,
            *actual_heading,
            drift
        );

        if drift <= cfg.max_drift_deg {
            *actual_heading = candidate_heading;
            motion.update_position(*actual_heading);
            SnapshotStore::new(nvs, persist_nvs).save_heading(*actual_heading);
            if motion_mode == MotionMode::EncoderGuarded {
                SnapshotStore::new(nvs, persist_nvs)
                    .save_encoder_snapshot(motion.encoder_ticks_adjusted());
            }
            return Ok(false);
        }

        log::warn!(
            "Encoder recovered but heading drift ({:.2}°) exceeds {:.2}°; re-homing to re-establish truth.",
            drift,
            cfg.max_drift_deg
        );
        let ok = match cfg.rehome_dir {
            Direction::Cw => motion.find_limit_switch_cw(),
            Direction::Ccw => motion.find_limit_switch_ccw(),
        };
        if !ok {
            infra::Telemetry::critical_failure_loop(
                mqtt,
                b"Critical failure: re-home after encoder recovery failed!",
                publish_mqtt,
            );
        }

        *actual_heading = HOME_HEADING_DEG;
        motion.update_position(*actual_heading);
        SnapshotStore::new(nvs, persist_nvs).save_heading(*actual_heading);
        if motion_mode == MotionMode::EncoderGuarded {
            SnapshotStore::new(nvs, persist_nvs)
                .save_encoder_snapshot(motion.encoder_ticks_adjusted());
        }
        Ok(false)
    }

    fn housekeeping(
        &self,
        wifi: &mut Wifi<'_>,
        mqtt: &mut Mqtt,
        current_version: &Version,
        publish_mqtt: bool,
    ) -> anyhow::Result<()> {
        if wifi.state() == WifiState::Disconnected {
            log::warn!("Wifi disconnected, attempting to reconnect...");
            wifi.reconnect_if_disconnected()?;
        }
        infra::Telemetry::publish_firmware_version_if(mqtt, current_version, publish_mqtt);
        Ok(())
    }

    /// Switch to StepperOnly mode permanently after 3 consecutive encoder failures.
    fn switch_to_stepper_only<T: NvsPartitionId>(
        &mut self,
        motion: &mut Motion<'_>,
        nvs: &mut EspNvs<T>,
        persist_nvs: bool,
    ) -> anyhow::Result<()> {
        log::error!("CRITICAL: Encoder failed {} consecutive times, switching to StepperOnly mode permanently", self.failure_count);
        
        // Change motion mode to StepperOnly
        motion.set_motion_mode(MotionMode::StepperOnly);
        
        // Persist mode change to NVS (1 = StepperOnly, 2 = EncoderGuarded)
        if persist_nvs {
            if let Err(e) = nvs.set_u8("tracking_mode", 1) {
                log::error!("Failed to persist mode switch to NVS: {:?}", e);
            } else {
                log::info!("Mode switch persisted to NVS: StepperOnly");
            }
        }
        
        // Mark that we've switched modes
        self.mode_switched = true;
        self.active = false;  // Stop recovery attempts
        self.failure_count = 0;  // Reset counter
        
        log::error!("Encoder permanently disabled. Device now operating in StepperOnly mode.");
        Ok(())
    }
}

fn angle_diff_deg(a: f32, b: f32) -> f32 {
    let mut d = (a - b).rem_euclid(360.0);
    if d > 180.0 {
        d = 360.0 - d;
    }
    d.abs()
}

