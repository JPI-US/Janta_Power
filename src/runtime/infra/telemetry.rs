use std::time::Duration;

use network::mqtt::Mqtt;
use network::telemetry::{publish_error, Component, TIME_FORMAT};

/// Interval between republishes of a critical error while the device is wedged.
const CRITICAL_REPUBLISH_INTERVAL: Duration = Duration::from_secs(900);

/// Publish a structured `ErrorLog` to `tower/{id}/logs/error` every 15 minutes
/// until the device is reset. Used by runtime call sites where no recovery is
/// possible (boot homing failure, re-home failure, switchboard misconfig).
///
/// Motion has its own inline version of this loop using `chrono::Local` so it
/// doesn't need to depend on the `rtc` crate.
pub fn error_loop(
    device_id: &str,
    mqtt: &mut Mqtt,
    component: Component,
    message: &str,
    notes: &str,
) -> ! {
    loop {
        let now = rtc::timezone::local_time()
            .format(TIME_FORMAT)
            .to_string();
        let _ = publish_error(mqtt, device_id, &now, component, message, notes);
        std::thread::sleep(CRITICAL_REPUBLISH_INTERVAL);
    }
}
