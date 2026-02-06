use esp_idf_svc::nvs::{EspNvs, NvsPartitionId};
use motion::MotionMode;
use semver::Version;

use crate::state::SnapshotStore;

/// One “normal mode” tracking tick: call Motion's tracking logic and persist stable results.
///
/// We keep Motion's existing `set_tower_position()` behavior, but isolate the orchestration +
/// persistence wiring behind a single function.
pub fn tick<I2C: embedded_hal::i2c::I2c, T: NvsPartitionId>(
    motion: &mut motion::Motion,
    calculation: &mut clock::Clock<I2C>,
    actual_heading: &mut f32,
    mqtt: &mut network::mqtt::Mqtt,
    current_version: &Version,
    nvs: &mut EspNvs<T>,
    wifi: &mut wifi::wifi::Wifi<'_>,
    current_datetime: String,
) -> motion::MoveOutcome {
    let tracking_done = motion.set_tower_position(
        calculation,
        *actual_heading,
        0,
        mqtt,
        current_version.clone(),
        nvs,
        wifi,
        current_datetime,
    );

    // Persist only when a move actually completed successfully.
    // This prevents “phantom saves” and prevents old headings from being overwritten on abort paths.
    match motion.take_last_move_outcome() {
        Some(motion::MoveOutcome::Completed) => {
            *actual_heading = motion.location();
            SnapshotStore::new(nvs).save_heading(*actual_heading);
            if motion.motion_mode() == MotionMode::EncoderGuarded {
                SnapshotStore::new(nvs).save_encoder_snapshot(motion.encoder_ticks_adjusted());
            }
            motion::MoveOutcome::Completed
        }
        Some(outcome @ (motion::MoveOutcome::AbortedPowerMissing | motion::MoveOutcome::AbortedStall)) => {
            outcome
        }
        None => {
            // If no movement was needed, treat as Completed for outer orchestration.
            // We do not persist here because nothing changed.
            if !tracking_done {
                log::warn!("tracking_done=false but no MoveOutcome recorded; skipping NVS persist");
            }
            motion::MoveOutcome::Completed
        }
    }
}

