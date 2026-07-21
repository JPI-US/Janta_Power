use std::time::{Duration, Instant};

use anyhow::anyhow;
use esp_idf_svc::nvs::{EspNvs, NvsPartitionId};
use motion::motion::{Motion, MotionMode, MoveOutcome};
use network::{mqtt::Mqtt, telemetry::Component};
use semver::Version;
use wifi::wifi::{Wifi, WifiState};

use crate::storage::snapshot_store::SnapshotStore;

// Local direction enum used by encoder fault recovery.
#[derive(Clone, Copy, PartialEq)]
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

// Runtime switches for encoder fault recovery.
pub struct EncoderRecoverySwitches {
    pub enabled: bool,
    pub probe_interval_secs: u64,
    pub probe_steps: i64,
    pub max_drift_deg: f32,
    pub rehome_dir: Direction,
}

pub struct EncoderFaultRecovery {
    active: bool,
    next_probe_at: Option<Instant>,
    probe_failure_count: u8,
    mode_switched_daily: bool,
}

const MAX_PROBE_FAILURES: u8 = 3;

impl EncoderFaultRecovery {
    pub const fn new() -> Self {
        Self {
            active: false,
            next_probe_at: None,
            probe_failure_count: 0,
            mode_switched_daily: false,
        }
    }

    #[allow(dead_code)]
    pub fn mode_switched_daily(&self) -> bool {
        self.mode_switched_daily
    }

    pub fn set_mode_switched_daily(&mut self, value: bool) {
        self.mode_switched_daily = value;
    }

    pub fn on_move_outcome<T: NvsPartitionId>(
        &mut self,
        outcome: MoveOutcome,
        cfg: &EncoderRecoverySwitches,
        _motion: &mut Motion<'_>,
        _nvs: &mut EspNvs<T>,
        _persist_nvs: bool,
    ) -> anyhow::Result<()> {
        if self.mode_switched_daily {
            return Ok(());
        }

        // Stall/overshoot activates probe recovery; only failed probes count toward mode switch.
        if outcome == MoveOutcome::AbortedStall || outcome == MoveOutcome::AbortedOvershoot {
            log::warn!("Encoder fault detected, starting recovery probes");
            self.active = true;
            self.next_probe_at =
                Some(Instant::now() + Duration::from_secs(cfg.probe_interval_secs));
        }
        Ok(())
    }

    /// Returns `Ok(true)` if the caller should `continue` the outer loop (fault still active).
    pub fn tick<T: NvsPartitionId>(
        &mut self,
        ctx: &mut EncoderTickContext<'_, T>,
        motion: &mut Motion<'_>,
        motion_mode: MotionMode,
        actual_heading: &mut f32,
    ) -> anyhow::Result<(bool, Option<(Component, String, String)>)> {
        if self.mode_switched_daily {
            return Ok((false, None));
        }

        if !self.active {
            return Ok((false, None));
        }

        if !ctx.cfg.enabled {
            return Ok((false, Some((Component::System, "Encoder fault recovery disabled in switchboard".into(), "Switchboard disables encoder fault recovery; device cannot proceed when encoder has faulted.".into()))));
        }

        let probe_interval = Duration::from_secs(ctx.cfg.probe_interval_secs);
        let now_i = Instant::now();
        let should_probe = self.next_probe_at.map(|t| now_i >= t).unwrap_or(true);

        if !should_probe {
            let t = self
                .next_probe_at
                .ok_or_else(|| anyhow!("next_probe_at unexpectedly None"))?;
            let remaining = t.saturating_duration_since(now_i);
            log::info!("Encoder fault: waiting {:?} until next probe...", remaining);
            return Ok((true, None));
        }

        log::info!("Encoder fault: probing for recovery...");
        let ok = motion.probe_encoder_motion(ctx.cfg.probe_steps)?;
        if !ok {
            self.probe_failure_count += 1;
            log::warn!(
                "Encoder probe failed (probe_failure_count={}/{})",
                self.probe_failure_count,
                MAX_PROBE_FAILURES
            );

            if self.probe_failure_count >= MAX_PROBE_FAILURES {
                log::error!("CRITICAL: Encoder probe failed {} consecutive times, switching to StepperOnly mode for the day", self.probe_failure_count);
                self.switch_to_stepper_only_daily(
                    motion,
                    ctx.nvs,
                    ctx.persist_nvs,
                    &ctx.device_id,
                )?;
                return Ok((false, None));
            }

            self.next_probe_at = Some(now_i + probe_interval);
            return Ok((true, None));
        }

        log::info!(
            "Encoder probe succeeded; recomputing heading from encoder and resuming tracking."
        );
        self.active = false;
        self.next_probe_at = None;
        self.probe_failure_count = 0;

        let candidate_heading = motion.heading_from_encoder_ticks(ctx.home_heading_deg);
        let drift = angle_diff_deg(candidate_heading, *actual_heading);
        log::info!(
            "Encoder recovered: candidate_heading={} prev_heading={} drift_deg={}",
            candidate_heading,
            *actual_heading,
            drift
        );

        if drift <= ctx.cfg.max_drift_deg {
            *actual_heading = candidate_heading;
            motion.update_position(*actual_heading);
            SnapshotStore::new(ctx.nvs, ctx.persist_nvs).save_heading(*actual_heading);
            if motion_mode == MotionMode::EncoderGuarded {
                SnapshotStore::new(ctx.nvs, ctx.persist_nvs)
                    .save_encoder_snapshot(motion.encoder_ticks_adjusted());
            }
            return Ok((false, None));
        }

        log::warn!(
            "Encoder recovered but heading drift ({:.2}°) exceeds {:.2}°; re-homing to re-establish truth.",
            drift,
            ctx.cfg.max_drift_deg
        );
        let ok = match ctx.cfg.rehome_dir {
            Direction::Cw => motion.find_limit_switch_cw()?,
            Direction::Ccw => motion.find_limit_switch_ccw()?,
        };
        if !ok {
            return Ok((false, Some((Component::LimitSwitch, "Re-home failed after drift correction".into(), "Heading drift exceeded tolerance and the re-home sweep could not locate the limit switch.".into()))));
        }

        *actual_heading = ctx.home_heading_deg;
        motion.update_position(*actual_heading);
        SnapshotStore::new(ctx.nvs, ctx.persist_nvs).save_heading(*actual_heading);
        if motion_mode == MotionMode::EncoderGuarded {
            SnapshotStore::new(ctx.nvs, ctx.persist_nvs)
                .save_encoder_snapshot(motion.encoder_ticks_adjusted());
        }
        Ok((false, None))
    }

    fn housekeeping(
        &self,
        wifi: &mut Wifi<'_>,
        mqtt: &mut Mqtt,
        current_version: &Version,
        device_id: &str,
    ) -> anyhow::Result<()> {
        if wifi.state() == WifiState::Disconnected {
            log::warn!("Wifi disconnected, attempting to reconnect...");
            wifi.reconnect_if_disconnected()?;
        }
        let formatted_time = rtc::timezone::local_time()
            .format(network::telemetry::TIME_FORMAT)
            .to_string();
        let payload = network::telemetry::Heartbeat {
            current_time: &formatted_time,
            firmware_version: &current_version.to_string(),
        };
        let topic = network::telemetry::topic::status(device_id);
        let _ = network::telemetry::publish_json(mqtt, &topic, &payload);
        Ok(())
    }

    /// Switch to StepperOnly for the rest of the day after repeated probe failures.
    fn switch_to_stepper_only_daily<T: NvsPartitionId>(
        &mut self,
        motion: &mut Motion<'_>,
        nvs: &mut EspNvs<T>,
        persist_nvs: bool,
        _device_id: &str,
    ) -> anyhow::Result<()> {
        motion.set_motion_mode(MotionMode::StepperOnly);

        let current_date = rtc::timezone::local_time().format("%Y-%m-%d").to_string();

        if persist_nvs {
            SnapshotStore::new(nvs, persist_nvs).save_tracking_mode(MotionMode::StepperOnly);
            SnapshotStore::new(nvs, persist_nvs).save_encoder_mode_reset_date(&current_date);
            SnapshotStore::new(nvs, persist_nvs).save_encoder_daily_mode(true);
        }

        // One-shot critical: encoder probes exhausted; device falls back to
        // Stepper-only mode until midnight retry.
        let _notes = format!(
            "Probe failures reached the daily threshold ({}); device will retry encoder recovery at midnight.",
            self.probe_failure_count
        );
        let _current_time = rtc::timezone::local_time()
            .format(network::telemetry::TIME_FORMAT)
            .to_string();
        // TODO: Actually this error
        // let _ = network::telemetry::publish_error(
        //     mqtt,
        //     device_id,
        //     &current_time,
        //     network::telemetry::Component::Encoder,
        //     "Encoder probes exhausted, switched to Stepper-only",
        //     &notes,
        // );

        self.mode_switched_daily = true;
        self.active = false;
        self.probe_failure_count = 0;

        log::error!("Encoder disabled for the day. Device now operating in StepperOnly mode. Will retry at midnight.");
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

pub struct EncoderTickContext<'ctx, T: NvsPartitionId> {
    pub nvs: &'ctx mut EspNvs<T>,
    pub cfg: EncoderRecoverySwitches,
    pub current_version: Version,
    pub persist_nvs: bool,
    pub device_id: String,
    pub home_heading_deg: f32,
}
