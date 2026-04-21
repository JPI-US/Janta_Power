use log::{error, info, warn};
use semver::Version;

use network::mqtt::Mqtt;

/// Build a device-prefixed MQTT topic.
pub fn topic(device_id: &str, suffix: &str) -> String {
    format!("{}/{}", device_id, suffix)
}

pub struct Telemetry;

//This is where the status heartbeat is published for the main loop
impl Telemetry {
    pub fn publish_status_heartbeat_if(
        device_id: &str,
        mqtt: &mut Mqtt,
        formatted_time: String,
        version: &Version,
        enabled: bool,
    ) {
        if !enabled {
            warn!("MQTT publish disabled: skipping status heartbeat publish");
            return;
        }
        Self::publish_status_heartbeat(device_id, mqtt, formatted_time, version);
    }

    pub fn publish_status_heartbeat(
        device_id: &str,
        mqtt: &mut Mqtt,
        formatted_time: String,
        version: &Version,
    ) {
        let payload = serde_json::json!({
            "current_time": formatted_time,
            "firmware_version": version.to_string(),
        })
        .to_string();
        let t = format!("tower/{}/status", device_id);
        match mqtt.publish(&t, payload.as_bytes()) {
            Ok(_) => info!("Published status heartbeat to {}", t),
            Err(e) => warn!("Failed to publish status heartbeat: {:?}", e),
        }
    }

    pub fn publish_boot_log_if(
        device_id: &str,
        mqtt: &mut Mqtt,
        version: &Version,
        enabled: bool,
    ) -> bool {
        if !enabled {
            warn!("MQTT publish disabled: skipping boot log publish");
            return true;
        }
        Self::publish_boot_log(device_id, mqtt, version)
    }

    pub fn publish_boot_log(device_id: &str, mqtt: &mut Mqtt, version: &Version) -> bool {
        let current_time = rtc::timezone::local_time()
            .format("%d/%m/%Y %H:%M:%S")
            .to_string();
        let payload = serde_json::json!({
            "current_time": current_time,
            "type": "boot",
            "message": "Tower rebooted successfully",
            "firmware_version": version.to_string(),
            "component": "system",
            "notes": "Scheduled reboot completed without errors",
        })
        .to_string();
        let t = format!("tower/{}/logs/boot", device_id);
        match mqtt.publish(&t, payload.as_bytes()) {
            Ok(_) => {
                info!("Published boot log to {}", t);
                true
            }
            Err(e) => {
                error!("Failed to publish boot log: {:?}", e);
                false
            }
        }
    }

    pub fn publish_firmware_update_success_if(
        device_id: &str,
        mqtt: &mut Mqtt,
        previous_version: &Version,
        current_version: &Version,
        enabled: bool,
    ) -> bool {
        if !enabled {
            warn!("MQTT publish disabled: skipping firmware_update success publish");
            return true;
        }
        let current_time = rtc::timezone::local_time()
            .format("%d/%m/%Y %H:%M:%S")
            .to_string();
        let payload = serde_json::json!({
            "current_time": current_time,
            "type": "firmware_update",
            "message": "Firmware successfully updated",
            "previous_version": previous_version.to_string(),
            "current_version": current_version.to_string(),
            "notes": "No errors during update",
        })
        .to_string();
        let t = format!("tower/{}/logs/firmware_update", device_id);
        match mqtt.publish(&t, payload.as_bytes()) {
            Ok(_) => {
                info!("Published firmware_update success to {}", t);
                true
            }
            Err(e) => {
                error!("Failed to publish firmware_update success: {:?}", e);
                false
            }
        }
    }

    pub fn publish_firmware_update_failure_if(
        device_id: &str,
        mqtt: &mut Mqtt,
        current_version: &Version,
        enabled: bool,
    ) -> bool {
        if !enabled {
            warn!("MQTT publish disabled: skipping firmware_update failure publish");
            return true;
        }
        let current_time = rtc::timezone::local_time()
            .format("%d/%m/%Y %H:%M:%S")
            .to_string();
        let payload = serde_json::json!({
            "current_time": current_time,
            "type": "firmware_update",
            "message": "Firmware update unsuccessful",
            "previous_version": current_version.to_string(),
            "current_version": current_version.to_string(),
            "notes": "Did not update due to failures",
        })
        .to_string();
        let t = format!("tower/{}/logs/firmware_update", device_id);
        match mqtt.publish(&t, payload.as_bytes()) {
            Ok(_) => {
                info!("Published firmware_update failure to {}", t);
                true
            }
            Err(e) => {
                error!("Failed to publish firmware_update failure: {:?}", e);
                false
            }
        }
    }

    /// Critical alert on `tower/status`: republish `msg` every 15 minutes until reset.
    pub fn critical_failure_loop(
        device_id: &str,
        mqtt: &mut Mqtt,
        msg: &[u8],
        publish_enabled: bool,
    ) -> ! {
        let t = topic(device_id, "tower/status");
        loop {
            if publish_enabled {
                if let Err(e) = mqtt.publish(&t, msg) {
                    warn!("Failed to publish critical status to {}: {:?}", t, e);
                }
            } else {
                info!(
                    "MQTT publish disabled: {}",
                    core::str::from_utf8(msg).unwrap_or("<non-utf8>")
                );
            }
            std::thread::sleep(std::time::Duration::from_secs(900));
        }
    }

    pub fn publish_critical_failure_loop(device_id: &str, mqtt: &mut Mqtt, msg: &[u8]) -> ! {
        Self::critical_failure_loop(device_id, mqtt, msg, true)
    }
}

