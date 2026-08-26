use network::{
    mqtt::Mqtt,
    telemetry::{
        publish_component_status, publish_info, publish_json, topic, Component, ErrorLog, Severity,
    },
};

/// Temperature tier notes published in `logs/*` and `component/system/status`.
const NOTES_NORMAL: &str = "All components functioning normally";
const NOTES_DEGRADING: &str = "Inverters started to degrade";
const NOTES_EG4_BATTERY: &str = "Temperature too hot for eg4 battery";
const NOTES_BATTERY_SOLARK: &str = "Severely high temperature for Battery and Solark";
const NOTES_CRITICAL: &str = "EG4 inverter and all other components are in danger";

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum TempTier {
    /// Below 100 °F — info log only.
    Normal,
    /// 100.0 – 113.0 °F inclusive.
    Degrading,
    /// 113.1 – 122.0 °F.
    Eg4Battery,
    /// 122.1 – 140.0 °F (131.1–140.0 uses same tier until critical threshold).
    BatterySolark,
    /// Above 140.0 °F.
    Critical,
}

impl TempTier {
    fn classify(temp_f: f32) -> Self {
        if temp_f < 100.0 {
            Self::Normal
        } else if temp_f <= 113.0 {
            Self::Degrading
        } else if temp_f <= 122.0 {
            Self::Eg4Battery
        } else if temp_f <= 140.0 {
            Self::BatterySolark
        } else {
            Self::Critical
        }
    }

    fn notes(self) -> &'static str {
        match self {
            Self::Normal => NOTES_NORMAL,
            Self::Degrading => NOTES_DEGRADING,
            Self::Eg4Battery => NOTES_EG4_BATTERY,
            Self::BatterySolark => NOTES_BATTERY_SOLARK,
            Self::Critical => NOTES_CRITICAL,
        }
    }

    fn severity(self) -> Severity {
        match self {
            Self::Normal => Severity::Online,
            Self::Degrading => Severity::Warning,
            Self::Eg4Battery | Self::BatterySolark | Self::Critical => Severity::Fault,
        }
    }
}

/// Publish system telemetry for a temperature reading, using tiered notes.
///
/// Takes the reading rather than the sensor, so the caller can hold the sensor
/// lock for the read alone. Publishing is a network round trip, and holding a
/// device lock across one blocks every other reader for its duration.
pub fn report_system_temperature(
    temp_c: f32,
    rh: f32,
    mqtt: &mut Mqtt,
    device_id: &str,
    current_time: &str,
) {
    let temp_f = temp_c * 9.0 / 5.0 + 32.0;
    let tier = TempTier::classify(temp_f);
    let notes = tier.notes();

    if tier == TempTier::Normal {
        let message = format!("System temp {temp_f:.1}F, humidity {rh:.1}%RH");
        let _ = publish_info(
            mqtt,
            device_id,
            current_time,
            Component::System,
            &message,
            notes,
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
        return;
    }

    let message = format!("Temp is too hot; {temp_f:.1}F");
    let error_topic = topic::logs_error(device_id);
    let error_payload = ErrorLog {
        current_time,
        log_type: "error",
        message: &message,
        component: Component::System,
        severity: tier.severity(),
        value: Some(temp_f as f64),
        unit: Some("F"),
        notes,
    };
    let _ = publish_json(mqtt, &error_topic, &error_payload);

    let _ = publish_component_status(
        mqtt,
        device_id,
        current_time,
        Component::System,
        tier.severity(),
        notes,
    );
}
