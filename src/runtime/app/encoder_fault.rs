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
    switchboard::{Direction, EncoderRecoverySwitches},
};

pub struct EncoderFaultRecovery {
    active: bool,
    next_probe_at: Option<Instant>,
}

impl EncoderFaultRecovery {
    pub const fn new() -> Self {
        Self {
            active: false,
            next_probe_at: None,
        }
    }

    pub fn on_move_outcome(&mut self, outcome: MoveOutcome, cfg: &EncoderRecoverySwitches) {
        // Both stall and overshoot indicate encoder issues - trigger recovery
        if outcome == MoveOutcome::AbortedStall || outcome == MoveOutcome::AbortedOvershoot {
            self.active = true;
            self.next_probe_at = Some(Instant::now() + Duration::from_secs(cfg.probe_interval_secs));
        }
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
}

fn angle_diff_deg(a: f32, b: f32) -> f32 {
    let mut d = (a - b).rem_euclid(360.0);
    if d > 180.0 {
        d = 360.0 - d;
    }
    d.abs()
}

