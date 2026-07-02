use core::option::Option::None;

use esp_idf_svc::nvs::{EspNvs, NvsPartitionId};
use motion::MotionMode;
use semver::Version;

use crate::infra::SnapshotStore;

pub struct TrackingTickContext<'ctx, 'wifi, I2C, T>
where
    I2C: embedded_hal::i2c::I2c,
    T: NvsPartitionId,
{
    pub calculation: &'ctx mut clock::Clock<I2C>,
    pub mqtt: &'ctx mut network::mqtt::Mqtt,
    pub current_version: Version,
    pub nvs: &'ctx mut EspNvs<T>,
    pub wifi: &'ctx mut wifi::wifi::Wifi<'wifi>,
    pub current_datetime: String,
    pub persist_nvs: bool,
    pub allow_ota: bool,
    pub device_id: &'ctx str,
}

/// Run one tracking tick and persist stable state.
pub fn tick<I2C, T>(
    motion: &mut motion::Motion,
    ctx: &mut TrackingTickContext<I2C, T>,
    actual_heading: &mut f32,
) -> motion::MoveOutcome
where
    I2C: embedded_hal::i2c::I2c,
    T: NvsPartitionId,
{
    let tracking_done = motion.set_tower_position(
        ctx.calculation,
        *actual_heading,
        0,
        ctx.mqtt,
        ctx.current_version.clone(),
        ctx.nvs,
        ctx.wifi,
        ctx.current_datetime.clone(),
        ctx.persist_nvs,
        ctx.allow_ota,
        ctx.device_id,
    );

    // Persist only after completed moves.
    match motion.take_last_move_outcome() {
        Some(motion::MoveOutcome::Completed) => {
            *actual_heading = motion.location();
            SnapshotStore::new(ctx.nvs, ctx.persist_nvs).save_heading(*actual_heading);
            if motion.motion_mode() == MotionMode::EncoderGuarded {
                SnapshotStore::new(ctx.nvs, ctx.persist_nvs)
                    .save_encoder_snapshot(motion.encoder_ticks_adjusted());
            }
            motion::MoveOutcome::Completed
        }
        Some(
            outcome @ (motion::MoveOutcome::AbortedPowerMissing
            | motion::MoveOutcome::AbortedStall
            | motion::MoveOutcome::AbortedOvershoot),
        ) => outcome,
        None => {
            // No movement needed; treat as completed without writing NVS.
            if !tracking_done {
                log::warn!("tracking_done=false but no MoveOutcome recorded; skipping NVS persist");
            }
            motion::MoveOutcome::Completed
        }
    }
}
