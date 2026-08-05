//! Catalog of remote commands the tower answers — the single place to look up
//! "what commands exist and what they do".
//!
//! To add a command:
//!   1. write a handler `fn get_xxx(ctx: &CmdCtx) -> Value`
//!   2. add one arm to [`dispatch`]
//!   3. if it needs more state, add a field to [`CmdCtx`] (and populate it in `main.rs`)
//!
//! `transport.rs` and `main.rs`'s loop body do not change as commands grow.

use serde_json::{json, Value};

/// Read-only view of tower state that commands may report on.
///
/// Built fresh each loop iteration in `main.rs` and handed to [`dispatch`].
/// Grow this as new `get_*` commands need more fields (encoder ticks,
/// temperature, last move outcome, ...).
pub struct CmdCtx<'a> {
    pub device_id: &'a str,
    pub firmware_version: &'a str,
    pub mqtt_connected: bool,
    pub wifi_connected: bool,
    pub motion_mode: &'a str,
    pub current_heading: f32,
}

/// Route a command name to its handler.
///
/// Returns `Some(data)` with the command's JSON result, or `None` for an
/// unknown command (the transport turns that into an error reply).
pub fn dispatch(cmd: &str, ctx: &CmdCtx) -> Option<Value> {
    match cmd {
        "get_status" => Some(get_status(ctx)),
        // Add more read-only commands here, e.g.:
        // "get_encoder" => Some(get_encoder(ctx)),
        _ => None,
    }
}

/// `get_status` — a snapshot of current tower health and orientation.
fn get_status(ctx: &CmdCtx) -> Value {
    json!({
        "device_id": ctx.device_id,
        "firmware_version": ctx.firmware_version,
        "mqtt_connected": ctx.mqtt_connected,
        "wifi_connected": ctx.wifi_connected,
        "motion_mode": ctx.motion_mode,
        "current_heading": ctx.current_heading,
    })
}
