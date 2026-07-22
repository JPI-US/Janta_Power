use core::option::Option::None;

use ::motion::{
    motion::{Motion, MotionMode, MoveOutcome, TowerPositionCtx},
    MotionEvent,
};
use ::network::telemetry::{topic, Component, ErrorLog, Severity};
use esp_idf_svc::nvs::{EspNvs, NvsPartitionId};
use fsm::channel::Channel;
use semver::Version;

use crate::{
    logic::fsm::FSMCommand::{self, MqttPublishJson, PerformOTA},
    storage::snapshot_store::SnapshotStore,
};

pub struct TrackingTickContext<'ctx, I2C, T>
where
    I2C: embedded_hal::i2c::I2c,
    T: NvsPartitionId,
{
    pub calculation: &'ctx mut clock::Clock<I2C>,
    pub current_version: Version,
    pub nvs: &'ctx mut EspNvs<T>,
    pub current_datetime: String,
    pub persist_nvs: bool,
    pub allow_ota: bool,
    pub device_id: &'ctx str,
    pub channel: &'ctx mut Channel<FSMCommand>,
}

/// Run one tracking tick and persist stable state.
pub fn tick<I2C, T>(
    motion: &mut Motion,
    ctx: &mut TrackingTickContext<I2C, T>,
    actual_heading: &mut f32,
) -> anyhow::Result<MoveOutcome>
where
    I2C: embedded_hal::i2c::I2c,
    T: NvsPartitionId,
{
    let nctx = TowerPositionCtx {
        allow_ota: ctx.allow_ota,
        clock: ctx.calculation,
        current_version: ctx.current_version.clone(),
        device_id: ctx.device_id,
        formatted_time: ctx.current_datetime.clone(),
        nvs: ctx.nvs,
        persist_nvs: ctx.persist_nvs,
    };

    let tracking_done = motion.set_tower_position(nctx, *actual_heading, 0);

    // Persist only after completed moves.
    match motion.take_last_move_outcome() {
        Some(MoveOutcome::Completed) => {
            *actual_heading = motion.location();
            SnapshotStore::new(ctx.nvs, ctx.persist_nvs).save_heading(*actual_heading);
            if motion.motion_mode() == MotionMode::EncoderGuarded {
                SnapshotStore::new(ctx.nvs, ctx.persist_nvs)
                    .save_encoder_snapshot(motion.encoder_ticks_adjusted());
            }
            Ok(MoveOutcome::Completed)
        }
        Some(
            outcome @ (MoveOutcome::AbortedPowerMissing
            | MoveOutcome::AbortedStall
            | MoveOutcome::AbortedOvershoot),
        ) => Ok(outcome),
        None => {
            // No movement needed; treat as completed without writing NVS.
            // TODO: Figure out this part
            if !tracking_done.0? {
                log::warn!("tracking_done=false but no MoveOutcome recorded; skipping NVS persist");
            }

            for event in tracking_done.1 {
                match event {
                    MotionEvent::Angle(payload) => {
                        let serialized = serde_json::to_string(&payload)?;
                        let topic = topic::data_angle(ctx.device_id);
                        ctx.channel.send(MqttPublishJson(serialized, topic));
                    }
                    MotionEvent::HomeErrorTicks(payload) => {
                        let serialized = serde_json::to_string(&payload)?;
                        let topic = topic::data_encoder_error_ticks(ctx.device_id);
                        ctx.channel.send(MqttPublishJson(serialized, topic));
                    }
                    MotionEvent::Error(time, message, notes) => {
                        let payload = ErrorLog {
                            current_time: time.as_str(),
                            log_type: "error",
                            message: message.as_str(),
                            component: Component::LimitSwitch,
                            severity: Severity::Fault,
                            value: None,
                            unit: None,
                            notes: notes.as_str(),
                        };
                        let serialized = serde_json::to_string(&payload)?;
                        let topic = topic::logs_error(ctx.device_id);
                        ctx.channel.send(MqttPublishJson(serialized, topic));
                    }
                    MotionEvent::CheckForOTA => {
                        ctx.channel.send(PerformOTA);
                    }
                }
            }

            Ok(MoveOutcome::Completed)
        }
    }
}
