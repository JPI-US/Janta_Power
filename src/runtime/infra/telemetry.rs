use log::{error, info, warn};
use semver::Version;

use network::mqtt::Mqtt;

// Centralized MQTT topics.
pub const TOPIC_BOOT: &str = "device1/boot";
pub const TOPIC_FIRMWARE_VERSION: &str = "device1/firmware/version";
pub const TOPIC_TOWER_STATUS: &str = "device1/tower/status";

pub struct Telemetry;

impl Telemetry {
    pub fn publish_firmware_version_if(mqtt: &mut Mqtt, version: &Version, enabled: bool) {
        if !enabled {
            warn!("MQTT publish disabled: skipping firmware version publish");
            return;
        }
        Self::publish_firmware_version(mqtt, version);
    }

    pub fn publish_firmware_version(mqtt: &mut Mqtt, version: &Version) {
        let payload = format!("The current firmware version is: {}", version.to_string());
        match mqtt.publish(TOPIC_FIRMWARE_VERSION, payload.as_bytes()) {
            Ok(_) => info!("Published firmware version"),
            Err(e) => warn!("Failed to publish firmware version: {:?}", e),
        }
    }

    pub fn publish_boot_check_if(mqtt: &mut Mqtt, enabled: bool) -> bool {
        if !enabled {
            warn!("MQTT publish disabled: skipping boot check publish");
            return true;
        }
        Self::publish_boot_check(mqtt)
    }

    pub fn publish_boot_check(mqtt: &mut Mqtt) -> bool {
        match mqtt.publish(TOPIC_BOOT, b"Boot check...") {
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

    pub fn critical_failure_loop(mqtt: &mut Mqtt, msg: &[u8], publish_enabled: bool) -> ! {
        loop {
            if publish_enabled {
                if let Err(e) = mqtt.publish(TOPIC_TOWER_STATUS, msg) {
                    error!("Failed to publish critical error message: {:?}", e);
                }
            } else {
                error!(
                    "CRITICAL_FAILURE (MQTT disabled): {}",
                    core::str::from_utf8(msg).unwrap_or("<non-utf8 msg>")
                );
            }
            std::thread::sleep(std::time::Duration::from_secs(900)); // every 15 minutes
        }
    }

    // Backwards-compatible name for older call sites (publishes unconditionally).
    pub fn publish_critical_failure_loop(mqtt: &mut Mqtt, msg: &[u8]) -> ! {
        Self::critical_failure_loop(mqtt, msg, true)
    }
}

