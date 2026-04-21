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

    pub fn publish_boot_check_if(device_id: &str, mqtt: &mut Mqtt, enabled: bool) -> bool {
        if !enabled {
            warn!("MQTT publish disabled: skipping boot check publish");
            return true;
        }
        Self::publish_boot_check(device_id, mqtt)
    }

    pub fn publish_boot_check(device_id: &str, mqtt: &mut Mqtt) -> bool {
        let t = topic(device_id, "boot");
        match mqtt.publish(&t, b"Boot check...") {
            Ok(_) => {
                info!("MQTT boot diagnostic publish succeeded...");
                true
            }
            Err(e) => {
                error!("MQTT boot diagnostic publish failed: {:?}", e);
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

