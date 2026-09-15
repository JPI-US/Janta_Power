//! MQTT transport for the remote command channel.
//!
//! Kept separate from the command catalog in [`crate::diagnostics::commands`]
//! so this file does not grow as commands are added.
//!
//! - [`subscribe`]: subscribe to `tower/{id}/cmd/diagnostics` once at boot.
//! - [`process_one`]: pull at most one queued command, dispatch it, and publish
//!   a correlated reply on `tower/{id}/cmd/diagnostics/ack`.
//!
//! Reply envelope (every reply):
//! ```json
//! { "current_time": "...", "request_id": "...", "cmd": "...",
//!   "status": "ok",    "data": { ... } }      // success
//! { "current_time": "...", "request_id": "...", "cmd": "...",
//!   "status": "error", "message": "..." }     // failure / unknown command
//! ```

use anyhow::Result;
use log::{error, info, warn};
use network::{
    mqtt::Mqtt,
    telemetry::{publish_json, topic, TIME_FORMAT},
};
use serde::Deserialize;
use serde_json::{json, Value};

use crate::diagnostics::{
    commands::{self, CmdCtx},
    motion_commands::{self, MotionCmdCtx},
};

/// Inbound command shape: `{ "cmd": "get_status", "request_id": "abc" }`.
/// `request_id` is optional and echoed back so the caller can correlate replies.
///
/// `degrees` and `ignore_encoder` are only read by the movement commands in
/// [`motion_commands`]; read-only commands ignore them. `degrees` stays a raw
/// [`Value`] so a bad value can be reported as such, rather than failing the
/// whole payload as malformed JSON.
#[derive(Deserialize)]
struct Command {
    cmd: String,
    #[serde(default)]
    request_id: Option<String>,
    #[serde(default)]
    degrees: Option<Value>,
    #[serde(default)]
    ignore_encoder: Option<bool>,
}

/// Subscribe to this tower's command topic. Call once, after MQTT connects.
pub fn subscribe(mqtt: &mut Mqtt, device_id: &str) -> Result<()> {
    let cmd_topic = topic::diagnostics_cmd(device_id);
    mqtt.subscribe(&cmd_topic)?;
    info!("Subscribed to command channel: {}", cmd_topic);
    Ok(())
}

/// Handle at most one queued command. Non-blocking.
///
/// Returns `Ok(true)` if a message was consumed (valid or not), `Ok(false)` if
/// the queue was empty. Malformed input still consumes the message and gets an
/// error reply, so a bad payload can't wedge the queue.
///
/// `motion_ctx` is what makes the movement commands available; pass `None` and
/// this is the read-only channel it has always been.
pub fn process_one(
    mqtt: &mut Mqtt,
    device_id: &str,
    ctx: &CmdCtx,
    motion_ctx: Option<&mut MotionCmdCtx<'_>>,
) -> Result<bool> {
    let Some((in_topic, payload)) = mqtt.try_receive() else {
        return Ok(false);
    };

    let cmd_topic = topic::diagnostics_cmd(device_id);
    if in_topic != cmd_topic {
        warn!(
            "Ignoring message on unexpected topic: {} (want {})",
            in_topic, cmd_topic
        );
        return Ok(false);
    }

    let body = match std::str::from_utf8(&payload) {
        Ok(s) => s,
        Err(e) => {
            error!("Command payload is not UTF-8: {:?}", e);
            reply_error(mqtt, device_id, "", "unknown", "payload must be UTF-8")?;
            return Ok(true);
        }
    };

    let command: Command = match serde_json::from_str(body) {
        Ok(c) => c,
        Err(e) => {
            error!("Command JSON is invalid: {:?}", e);
            reply_error(
                mqtt,
                device_id,
                "",
                "unknown",
                "payload must be valid JSON with a `cmd` field",
            )?;
            return Ok(true);
        }
    };

    let request_id = command.request_id.as_deref().unwrap_or("");
    info!(
        "Command received: cmd={} request_id={}",
        command.cmd, request_id
    );

    // Read-only catalog first; it is the common case and cannot touch hardware.
    if let Some(data) = commands::dispatch(&command.cmd, ctx) {
        reply_ok(mqtt, device_id, request_id, &command.cmd, data)?;
        return Ok(true);
    }

    // Then the movement catalog, if this build was handed motion access.
    let moved = motion_ctx.and_then(|motion_ctx| {
        motion_commands::dispatch(
            &command.cmd,
            command.degrees.as_ref(),
            command.ignore_encoder.unwrap_or(false),
            motion_ctx,
        )
    });

    match moved {
        Some(Ok(data)) => reply_ok(mqtt, device_id, request_id, &command.cmd, data)?,
        Some(Err(message)) => {
            warn!("Command {} refused: {}", command.cmd, message);
            reply_error(mqtt, device_id, request_id, &command.cmd, &message)?;
        }
        None => {
            warn!("Unsupported command: {}", command.cmd);
            reply_error(
                mqtt,
                device_id,
                request_id,
                &command.cmd,
                "unsupported command",
            )?;
        }
    }
    Ok(true)
}

fn reply_ok(
    mqtt: &mut Mqtt,
    device_id: &str,
    request_id: &str,
    cmd: &str,
    data: Value,
) -> Result<()> {
    let envelope = json!({
        "current_time": now(),
        "request_id": request_id,
        "cmd": cmd,
        "status": "ok",
        "data": data,
    });
    publish_json(mqtt, &topic::diagnostics_ack(device_id), &envelope)
}

fn reply_error(
    mqtt: &mut Mqtt,
    device_id: &str,
    request_id: &str,
    cmd: &str,
    message: &str,
) -> Result<()> {
    let envelope = json!({
        "current_time": now(),
        "request_id": request_id,
        "cmd": cmd,
        "status": "error",
        "message": message,
    });
    publish_json(mqtt, &topic::diagnostics_ack(device_id), &envelope)
}

/// Unprompted "I am an Install image and I am listening" on the ack topic.
///
/// Same envelope as a command reply so the AWS console subscriber that is
/// already watching `cmd/diagnostics/ack` sees it. Called once per Install
/// boot, after MQTT is up and before the main loop.
pub fn announce_install_ready(
    mqtt: &mut Mqtt,
    device_id: &str,
    heading_trusted: bool,
    lmsw_active: bool,
    max_step_deg: f32,
    tracking_inhibited: bool,
) {
    let data = json!({
        "heading_trusted": heading_trusted,
        "lmsw_active": lmsw_active,
        "max_step_deg": max_step_deg,
        "tracking_inhibited": tracking_inhibited,
    });
    if let Err(e) = reply_ok(mqtt, device_id, "", "install_ready", data) {
        warn!("Failed to announce install_ready: {:?}", e);
    } else {
        info!(
            "Announced install_ready on {}",
            topic::diagnostics_ack(device_id)
        );
    }
}

/// Tower-local timestamp string, matching the format used by all telemetry.
fn now() -> String {
    rtc::timezone::local_time().format(TIME_FORMAT).to_string()
}
