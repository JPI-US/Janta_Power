use log::{error, info, warn};
use network::mqtt::Mqtt;
use network::telemetry::{publish_json, topic, DiagnosticsAck, TIME_FORMAT};
use serde::Deserialize;

#[derive(Debug, Deserialize)]
struct DiagnosticsCommand {
    cmd: String,
    request_id: Option<String>,
    message: Option<String>,
}

pub fn subscribe(mqtt: &mut Mqtt, device_id: &str) -> anyhow::Result<()> {
    let command_topic = topic::diagnostics_cmd(device_id);
    mqtt.subscribe(&command_topic)?;
    info!("Subscribed to diagnostics commands: {}", command_topic);
    Ok(())
}

pub fn process_one(mqtt: &mut Mqtt, device_id: &str) -> anyhow::Result<bool> {
    let Some((incoming_topic, payload)) = mqtt.try_receive() else {
        return Ok(false);
    };

    let command_topic = topic::diagnostics_cmd(device_id);
    if incoming_topic != command_topic {
        warn!(
            "Ignoring MQTT message on unexpected topic: {} (expected {})",
            incoming_topic, command_topic
        );
        return Ok(false);
    }

    let payload_str = match std::str::from_utf8(&payload) {
        Ok(value) => value,
        Err(e) => {
            error!("Diagnostics command payload is not UTF-8: {:?}", e);
            publish_ack(mqtt, device_id, "", "unknown", "error", "Command payload must be UTF-8")?;
            return Ok(true);
        }
    };

    let command: DiagnosticsCommand = match serde_json::from_str(payload_str) {
        Ok(value) => value,
        Err(e) => {
            error!("Failed to parse diagnostics command JSON: {:?}", e);
            publish_ack(mqtt, device_id, "", "unknown", "error", "Command payload must be valid JSON")?;
            return Ok(true);
        }
    };

    let request_id = command.request_id.as_deref().unwrap_or("");
    let message = command.message.as_deref().unwrap_or("");
    info!(
        "Diagnostics command received: cmd={} request_id={} message={}",
        command.cmd, request_id, message
    );

    match command.cmd.as_str() {
        "ping" => publish_ack(
            mqtt,
            device_id,
            request_id,
            &command.cmd,
            "received",
            "Tower received diagnostics command",
        )?,
        other => {
            warn!("Unsupported diagnostics command: {}", other);
            publish_ack(
                mqtt,
                device_id,
                request_id,
                other,
                "error",
                "Unsupported diagnostics command",
            )?;
        }
    }

    Ok(true)
}

fn publish_ack(
    mqtt: &mut Mqtt,
    device_id: &str,
    request_id: &str,
    cmd: &str,
    status: &str,
    message: &str,
) -> anyhow::Result<()> {
    let current_time = rtc::timezone::local_time()
        .format(TIME_FORMAT)
        .to_string();
    let payload = DiagnosticsAck {
        current_time: &current_time,
        request_id,
        cmd,
        status,
        message,
    };
    let ack_topic = topic::diagnostics_ack(device_id);
    publish_json(mqtt, &ack_topic, &payload)
}
