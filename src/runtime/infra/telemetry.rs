use log::warn;

use network::mqtt::Mqtt;

/// Build a device-prefixed MQTT topic (legacy `{id}/{suffix}` shape).
///
/// Retained for the `tower/status` critical-failure loop until that topic
/// is migrated to the new AWS `tower/{id}/logs/error` shape. New code should
/// use `network::telemetry::topic::*` instead.
pub fn topic(device_id: &str, suffix: &str) -> String {
    format!("{}/{}", device_id, suffix)
}

pub struct Telemetry;

impl Telemetry {
    /// Critical alert on the legacy `{id}/tower/status` topic: republish `msg`
    /// every 15 minutes until the device is reset.
    ///
    /// Migrates to `network::telemetry::topic::logs_error` + `ErrorLog` payload
    /// during the `tower/status` topic migration.
    pub fn critical_failure_loop(device_id: &str, mqtt: &mut Mqtt, msg: &[u8]) -> ! {
        let t = topic(device_id, "tower/status");
        loop {
            if let Err(e) = mqtt.publish(&t, msg) {
                warn!("Failed to publish critical status to {}: {:?}", t, e);
            }
            std::thread::sleep(std::time::Duration::from_secs(900));
        }
    }
}
