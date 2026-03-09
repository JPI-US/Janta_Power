//! Custom profile behavior (this boot only).
//! When the maintenance button switches to Custom, the main loop calls [tick] here
//! instead of the normal tracking path. Add Custom-specific logic in this module.

use esp_idf_svc::nvs::{EspNvs, NvsPartitionId};
use motion::MotionMode;
use semver::Version;

use crate::infra::SnapshotStore;

/// One Custom-mode iteration. For now runs the same tracking as Normal; replace with
/// Custom-specific behavior (e.g. manual nudge, different loop timing) as needed.
pub fn tick<I2C: embedded_hal::i2c::I2c, T: NvsPartitionId>(
    motion: &mut motion::Motion,
    calculation: &mut clock::Clock<I2C>,
    actual_heading: &mut f32,
    mqtt: &mut network::mqtt::Mqtt,
    current_version: &Version,
    nvs: &mut EspNvs<T>,
    wifi: &mut wifi::wifi::Wifi<'_>,
    current_datetime: String,
    publish_mqtt: bool,
    persist_nvs: bool,
    allow_ota: bool,
) -> motion::MoveOutcome {
    // Reuse normal tracking for now; replace with Custom-only logic when ready.
    let _ = motion.set_tower_position(
        calculation,
        *actual_heading,
        0,
        mqtt,
        current_version.clone(),
        nvs,
        wifi,
        current_datetime,
        publish_mqtt,
        persist_nvs,
        allow_ota,
    );

    match motion.take_last_move_outcome() {
        Some(motion::MoveOutcome::Completed) => {
            *actual_heading = motion.location();
            SnapshotStore::new(nvs, persist_nvs).save_heading(*actual_heading);
            if motion.motion_mode() == MotionMode::EncoderGuarded {
                SnapshotStore::new(nvs, persist_nvs)
                    .save_encoder_snapshot(motion.encoder_ticks_adjusted());
            }
            motion::MoveOutcome::Completed
        }
        Some(outcome @ (motion::MoveOutcome::AbortedPowerMissing
            | motion::MoveOutcome::AbortedStall
            | motion::MoveOutcome::AbortedOvershoot)) => outcome,
        None => motion::MoveOutcome::Completed,
    }
}
