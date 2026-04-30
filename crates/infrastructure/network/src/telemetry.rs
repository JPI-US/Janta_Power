//! Shared MQTT telemetry primitives: topic builders, payload structs,
//! and a generic `publish_json` helper.
//!
//! Every publisher (binary, motion, encoder_fault, ota) uses these types
//! so that topic names and payload shapes have a single source of truth.
//! On-the-wire JSON and topic strings match the AWS IoT Core spec.

use anyhow::Result;
use log::{error, info};
use serde::Serialize;

use crate::mqtt::Mqtt;

/// Standard timestamp format used across every telemetry payload.
///
/// Callers format `rtc::timezone::local_time()` with this constant and pass
/// the resulting `String` / `&str` into a payload struct's `current_time`.
pub const TIME_FORMAT: &str = "%d/%m/%Y %H:%M:%S";

// ---------- Topic builders ----------
pub mod topic {
    use super::Component;

    pub fn status(device_id: &str) -> String {
        format!("tower/{device_id}/status")
    }

    pub fn logs_boot(device_id: &str) -> String {
        format!("tower/{device_id}/logs/boot")
    }

    pub fn logs_firmware_update(device_id: &str) -> String {
        format!("tower/{device_id}/logs/firmware_update")
    }

    pub fn logs_error(device_id: &str) -> String {
        format!("tower/{device_id}/logs/error")
    }

    pub fn logs_warning(device_id: &str) -> String {
        format!("tower/{device_id}/logs/warning")
    }

    pub fn logs_info(device_id: &str) -> String {
        format!("tower/{device_id}/logs/info")
    }

    pub fn data_angle(device_id: &str) -> String {
        format!("tower/{device_id}/data/angle")
    }

    pub fn data_encoder_error_ticks(device_id: &str) -> String {
        format!("tower/{device_id}/data/encoder_error_ticks")
    }

    pub fn component_status(device_id: &str, component: Component) -> String {
        format!(
            "tower/{device_id}/component/{}/status",
            component.as_str()
        )
    }

    pub fn diagnostics_cmd(device_id: &str) -> String {
        format!("tower/{device_id}/cmd/diagnostics")
    }

    pub fn diagnostics_ack(device_id: &str) -> String {
        format!("tower/{device_id}/cmd/diagnostics/ack")
    }
}

// ---------- Shared enums ----------

/// Tower sub-component taxonomy used by `logs/*` and `component/*/status` topics.
#[derive(Copy, Clone, Debug, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Component {
    Motor,
    Encoder,
    LightSensor,
    LimitSwitch,
    System,
}

impl Component {
    pub fn as_str(&self) -> &'static str {
        match self {
            Component::Motor => "motor",
            Component::Encoder => "encoder",
            Component::LightSensor => "light_sensor",
            Component::LimitSwitch => "limit_switch",
            Component::System => "system",
        }
    }
}

/// Component status severity, used by `component/*/status` payloads.
#[derive(Copy, Clone, Debug, Serialize)]
#[serde(rename_all = "lowercase")]
pub enum Severity {
    Online,
    Warning,
    Fault,
}

// ---------- Payload structs ----------

/// `tower/{id}/status` — periodic liveness + firmware ping.
#[derive(Serialize)]
pub struct Heartbeat<'a> {
    pub current_time: &'a str,
    pub firmware_version: &'a str,
}

/// `tower/{id}/logs/boot` — published once per successful boot.
#[derive(Serialize)]
pub struct BootLog<'a> {
    pub current_time: &'a str,
    pub message: &'a str,
    pub firmware_version: &'a str,
    pub component: Component,
    pub notes: &'a str,
}

/// `tower/{id}/logs/firmware_update` — success or failure of an OTA pass.
#[derive(Serialize)]
pub struct FirmwareUpdateLog<'a> {
    pub current_time: &'a str,
    pub message: &'a str,
    pub previous_version: &'a str,
    pub current_version: &'a str,
    pub notes: &'a str,
}

/// `tower/{id}/logs/error` — structured error event.
///
/// `value` + `unit` are optional and only appear in the JSON when the error
/// carries a measurable reading (e.g. motor current, board temperature). For
/// non-numeric errors (limit switch not found, NVS corruption, ...) leave
/// them `None` and `serde` will omit the fields entirely.
#[derive(Serialize)]
pub struct ErrorLog<'a> {
    pub current_time: &'a str,
    pub message: &'a str,
    pub component: Component,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub value: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub unit: Option<&'a str>,
    pub notes: &'a str,
}

/// `tower/{id}/logs/warning` — structured warning event.
///
/// Same optional `value` + `unit` contract as `ErrorLog`; see its docstring.
#[derive(Serialize)]
pub struct WarningLog<'a> {
    pub current_time: &'a str,
    pub message: &'a str,
    pub component: Component,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub value: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub unit: Option<&'a str>,
    pub notes: &'a str,
}

/// `tower/{id}/logs/info` — structured info event.
#[derive(Serialize)]
pub struct InfoLog<'a> {
    pub current_time: &'a str,
    pub message: &'a str,
    pub component: Component,
    pub notes: &'a str,
}

/// `tower/{id}/data/angle` — periodic tracking heading.
#[derive(Serialize)]
pub struct Angle<'a> {
    pub current_time: &'a str,
    pub tower_angle: f64,
}

/// `tower/{id}/data/encoder_error_ticks` — encoder drift captured at the
/// moment the limit switch fires during sunset homing.
///
/// `encoder_error_ticks` is the raw drift (daytime CW ticks minus sunset CCW
/// ticks, relative to the previous zero reference). Sign follows encoder
/// convention: positive = switch fired before encoder thought we were home
/// (undershoot), negative = encoder passed home before switch fired (overshoot).
///
/// `category` is a structured bucket derived from the magnitude in degrees,
/// suitable for dashboard filters. `result` is the same information rendered
/// as a human-readable string for log panels / notifications.
#[derive(Serialize)]
pub struct EncoderErrorTicks<'a> {
    pub current_time: &'a str,
    pub encoder_error_ticks: i32,
    pub category: EncoderErrorCategory,
    pub result: &'a str,
}

/// Structured bucket for the encoder drift metric.
#[derive(Copy, Clone, Debug, Serialize)]
#[serde(rename_all = "lowercase")]
pub enum EncoderErrorCategory {
    Acceptable,
    Undershoot,
    Overshoot,
}

/// `tower/{id}/component/{component}/status` — per-component health snapshot.
#[derive(Serialize)]
pub struct ComponentStatus<'a> {
    pub current_time: &'a str,
    pub status: Severity,
    pub notes: &'a str,
}

/// `tower/{id}/cmd/diagnostics/ack` — acknowledgement for remote diagnostics commands.
#[derive(Serialize)]
pub struct DiagnosticsAck<'a> {
    pub current_time: &'a str,
    pub request_id: &'a str,
    pub cmd: &'a str,
    pub status: &'a str,
    pub message: &'a str,
}

// ---------- Publisher ----------

/// Serialize `payload` as JSON and publish it to `topic`.
///
/// Logs success/failure uniformly. Returns `Err` if serialization or publish
/// fails so callers can react (e.g., stash state in NVS for retry).
pub fn publish_json<T: Serialize>(mqtt: &mut Mqtt, topic: &str, payload: &T) -> Result<()> {
    let body = serde_json::to_vec(payload)?;
    match mqtt.publish(topic, &body) {
        Ok(()) => {
            info!("Published telemetry to {}", topic);
            Ok(())
        }
        Err(e) => {
            error!("Failed to publish telemetry to {}: {:?}", topic, e);
            Err(e)
        }
    }
}

/// Build an `ErrorLog` and publish it to `tower/{device_id}/logs/error`.
///
/// Convenience wrapper for the common case where `value` and `unit` are not
/// applicable. Callers supply their own `current_time` string so this crate
/// stays neutral on time-source (rtc vs chrono::Local vs test clocks).
///
/// For errors that carry a scalar reading (e.g. over-temp with degrees), build
/// the `ErrorLog` struct directly and call `publish_json`.
pub fn publish_error(
    mqtt: &mut Mqtt,
    device_id: &str,
    current_time: &str,
    component: Component,
    message: &str,
    notes: &str,
) -> Result<()> {
    let topic = topic::logs_error(device_id);
    let payload = ErrorLog {
        current_time,
        message,
        component,
        value: None,
        unit: None,
        notes,
    };
    publish_json(mqtt, &topic, &payload)
}
