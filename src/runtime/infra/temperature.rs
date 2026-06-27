use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::I2c;
use hdc1080::Hdc1080;
use network::mqtt::Mqtt;
use network::telemetry::{
    publish_component_status, publish_info, publish_json, topic, Component, ErrorLog, Severity,
};

/// Read HDC1080 temperature and publish system telemetry based on threshold.
pub fn report_system_temperature<I2C, D>(
    sensor: &mut Hdc1080<I2C, D>,
    mqtt: &mut Mqtt,
    device_id: &str,
    current_time: &str,
    threshold_f: f32,
) where
    I2C: I2c,
    D: DelayNs,
{
    let Ok((temp_c, rh)) = sensor.read() else {
        log::warn!("HDC1080 read failed; skipping temp telemetry");
        return;
    };

    let temp_f = temp_c * 9.0 / 5.0 + 32.0;

    if temp_f > threshold_f {
        let message = format!("Temp is too hot; {temp_f:.1}F");
        let error_topic = topic::logs_error(device_id);
        let error_payload = ErrorLog {
            current_time,
            log_type: "error",
            message: &message,
            component: Component::System,
            severity: Severity::Fault,
            value: Some(temp_f as f64),
            unit: Some("F"),
            notes: "Severely hot",
        };
        let _ = publish_json(mqtt, &error_topic, &error_payload);

        let status_notes = format!("Board temperature {temp_f:.1}F exceeds {threshold_f:.1}F limit");
        let _ = publish_component_status(
            mqtt,
            device_id,
            current_time,
            Component::System,
            Severity::Fault,
            &status_notes,
        );
    } else {
        let message = format!("System temp {temp_f:.1}F, humidity {rh:.1}%RH");
        let _ = publish_info(
            mqtt,
            device_id,
            current_time,
            Component::System,
            &message,
            "All components functioning normally",
        );

        let status_notes = format!("Board temperature {temp_f:.1}F within normal range");
        let _ = publish_component_status(
            mqtt,
            device_id,
            current_time,
            Component::System,
            Severity::Online,
            &status_notes,
        );
    }
}
