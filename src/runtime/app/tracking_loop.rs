use esp_idf_svc::nvs::{EspNvs, NvsPartitionId};
use motion::MotionMode;
use semver::Version;

use crate::infra::SnapshotStore;

pub struct TrackingTickResult {
    pub outcome: motion::MoveOutcome,
    pub snapshot: Option<motion::TrackingSnapshot>,
}

/// Run one tracking tick and persist stable state.
///
/// `on_tick` is handed to the move itself, so the caller stays responsive across a
/// tracking pass or a sunset homing run — both of which block this thread.
pub fn tick<I2C: embedded_hal::i2c::I2c, T: NvsPartitionId>(
    motion: &mut motion::Motion,
    on_tick: motion::MoveWatcher<'_>,
    calculation: &mut clock::Clock<I2C>,
    actual_heading: &mut f32,
    mqtt: &mut network::mqtt::Mqtt,
    current_version: &Version,
    nvs: &mut EspNvs<T>,
    wifi: &mut wifi::wifi::Wifi<'_>,
    current_datetime: String,
    persist_nvs: bool,
    allow_ota: bool,
    device_id: &str,
) -> TrackingTickResult {
    let tracking_done = motion.set_tower_position(
        on_tick,
        calculation,
        *actual_heading,
        0,
        mqtt,
        current_version.clone(),
        nvs,
        wifi,
        current_datetime,
        persist_nvs,
        allow_ota,
        device_id,
    );
    let snapshot = motion.last_tracking_snapshot();

    // Persist only after completed moves.
    let outcome = match motion.take_last_move_outcome() {
        Some(motion::MoveOutcome::Completed) => {
            *actual_heading = motion.location();
            SnapshotStore::new(nvs, persist_nvs).save_heading(*actual_heading);
            if motion.motion_mode() == MotionMode::EncoderGuarded {
                SnapshotStore::new(nvs, persist_nvs)
                    .save_encoder_snapshot(motion.encoder_ticks_adjusted());
            }
            motion::MoveOutcome::Completed
        }
        Some(outcome @ (motion::MoveOutcome::AbortedPowerMissing | motion::MoveOutcome::AbortedStall | motion::MoveOutcome::AbortedOvershoot)) => {
            outcome
        }
        None => {
            // No movement needed; treat as completed without writing NVS.
            if !tracking_done {
                log::warn!("tracking_done=false but no MoveOutcome recorded; skipping NVS persist");
            }
            motion::MoveOutcome::Completed
        }
    };

    TrackingTickResult { outcome, snapshot }
}

