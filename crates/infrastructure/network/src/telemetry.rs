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
}

// ---------- Shared enums ----------

/// Tower sub-component taxonomy used by `logs/*` and `component/*/status` topics.
#[derive(Copy, Clone, Debug, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Component {
    Motor,
    Encoder,
    LightSensor,
    HomingSensor,
    System,
}

impl Component {
    pub fn as_str(&self) -> &'static str {
        match self {
            Component::Motor => "motor",
            Component::Encoder => "encoder",
            Component::LightSensor => "light_sensor",
            Component::HomingSensor => "homing_sensor",
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
    #[serde(rename = "type")]
    pub event_type: &'a str,
    pub message: &'a str,
    pub firmware_version: &'a str,
    pub component: Component,
    pub notes: &'a str,
}

/// `tower/{id}/logs/firmware_update` — success or failure of an OTA pass.
#[derive(Serialize)]
pub struct FirmwareUpdateLog<'a> {
    pub current_time: &'a str,
    #[serde(rename = "type")]
    pub event_type: &'a str,
    pub message: &'a str,
    pub previous_version: &'a str,
    pub current_version: &'a str,
    pub notes: &'a str,
}

/// `tower/{id}/logs/error` — structured error event.
#[derive(Serialize)]
pub struct ErrorLog<'a> {
    pub current_time: &'a str,
    #[serde(rename = "type")]
    pub event_type: &'a str,
    pub message: &'a str,
    pub component: Component,
    pub notes: &'a str,
}

/// `tower/{id}/logs/warning` — structured warning event.
#[derive(Serialize)]
pub struct WarningLog<'a> {
    pub current_time: &'a str,
    #[serde(rename = "type")]
    pub event_type: &'a str,
    pub message: &'a str,
    pub component: Component,
    pub notes: &'a str,
}

/// `tower/{id}/logs/info` — structured info event.
#[derive(Serialize)]
pub struct InfoLog<'a> {
    pub current_time: &'a str,
    #[serde(rename = "type")]
    pub event_type: &'a str,
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

/// `tower/{id}/data/encoder_error_ticks` — encoder drift captured at homing.
#[derive(Serialize)]
pub struct EncoderErrorTicks<'a> {
    pub current_time: &'a str,
    pub ticks: i32,
}

/// `tower/{id}/component/{component}/status` — per-component health snapshot.
#[derive(Serialize)]
pub struct ComponentStatus<'a> {
    pub current_time: &'a str,
    pub status: Severity,
    pub notes: &'a str,
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
