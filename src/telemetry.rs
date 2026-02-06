use log::{error, info, warn};
use semver::Version;

use network::mqtt::Mqtt;

// Centralized MQTT topics.
pub const TOPIC_BOOT: &str = "device1A/boot";
pub const TOPIC_FIRMWARE_VERSION: &str = "device1A/firmware/version";
pub const TOPIC_TOWER_STATUS: &str = "device1A/tower/status";

pub struct Telemetry;

impl Telemetry {
    pub fn publish_firmware_version(mqtt: &mut Mqtt, version: &Version) {
        let payload = format!("The current firmware version is: {}", version.to_string());
        match mqtt.publish(TOPIC_FIRMWARE_VERSION, payload.as_bytes()) {
            Ok(_) => info!("Published firmware version"),
            Err(e) => warn!("Failed to publish firmware version: {:?}", e),
        }
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

    pub fn publish_critical_failure_loop(mqtt: &mut Mqtt, msg: &[u8]) -> ! {
        loop {
            if let Err(e) = mqtt.publish(TOPIC_TOWER_STATUS, msg) {
                error!("Failed to publish critical error message: {:?}", e);
            }
            std::thread::sleep(std::time::Duration::from_secs(900)); // every 15 minutes
        }
    }
}


