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
use network::mqtt::Mqtt;
use network::telemetry::{publish_json, topic, TIME_FORMAT};
use serde::Deserialize;
use serde_json::{json, Value};

use crate::diagnostics::commands::{self, CmdCtx};

/// Inbound command shape: `{ "cmd": "get_status", "request_id": "abc" }`.
/// `request_id` is optional and echoed back so the caller can correlate replies.
#[derive(Deserialize)]
struct Command {
    cmd: String,
    #[serde(default)]
    request_id: Option<String>,
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
pub fn process_one(mqtt: &mut Mqtt, device_id: &str, ctx: &CmdCtx) -> Result<bool> {
    let Some((in_topic, payload)) = mqtt.try_receive() else {
        return Ok(false);
    };

    let cmd_topic = topic::diagnostics_cmd(device_id);
    if in_topic != cmd_topic {
        warn!("Ignoring message on unexpected topic: {} (want {})", in_topic, cmd_topic);
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
            reply_error(mqtt, device_id, "", "unknown", "payload must be valid JSON with a `cmd` field")?;
            return Ok(true);
        }
    };

    let request_id = command.request_id.as_deref().unwrap_or("");
    info!("Command received: cmd={} request_id={}", command.cmd, request_id);

    match commands::dispatch(&command.cmd, ctx) {
        Some(data) => reply_ok(mqtt, device_id, request_id, &command.cmd, data)?,
        None => {
            warn!("Unsupported command: {}", command.cmd);
            reply_error(mqtt, device_id, request_id, &command.cmd, "unsupported command")?;
        }
    }
    Ok(true)
}

fn reply_ok(mqtt: &mut Mqtt, device_id: &str, request_id: &str, cmd: &str, data: Value) -> Result<()> {
    let envelope = json!({
        "current_time": now(),
        "request_id": request_id,
        "cmd": cmd,
        "status": "ok",
        "data": data,
    });
    publish_json(mqtt, &topic::diagnostics_ack(device_id), &envelope)
}

fn reply_error(mqtt: &mut Mqtt, device_id: &str, request_id: &str, cmd: &str, message: &str) -> Result<()> {
    let envelope = json!({
        "current_time": now(),
        "request_id": request_id,
        "cmd": cmd,
        "status": "error",
        "message": message,
    });
    publish_json(mqtt, &topic::diagnostics_ack(device_id), &envelope)
}

/// Tower-local timestamp string, matching the format used by all telemetry.
fn now() -> String {
    rtc::timezone::local_time().format(TIME_FORMAT).to_string()
}
