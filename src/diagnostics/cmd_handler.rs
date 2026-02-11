// MQTT command handler for runtime diagnostics/test execution.
//
// Commands arrive as JSON on `device1A/admin/cmd` topic.
// Results are published to `device1A/admin/cmd/resp` topic.

use esp_idf_svc::nvs::{EspNvs, NvsPartitionId};
use log::{error, info, warn};
use motion::Motion;
use network::mqtt::Mqtt;
use semver::Version;
use wifi::wifi::Wifi;

use crate::diagnostics::admin_mode;
use crate::switchboard::{AdminSwitches, BootHomingSwitches, RecoverySwitches};

const TOPIC_CMD: &str = "device1A/admin/cmd";
const TOPIC_CMD_RESP: &str = "device1A/admin/cmd/resp";

#[derive(serde::Deserialize, Debug)]
#[serde(tag = "cmd")]
enum Command {
    #[serde(rename = "run_test")]
    RunTest { test: String },
    #[serde(rename = "get_status")]
    GetStatus,
}
 
pub struct CommandHandler;

impl CommandHandler {
    /// Subscribe to command topic and return handler instance.
    pub fn init(mqtt: &mut Mqtt) -> anyhow::Result<()> {
        mqtt.subscribe(TOPIC_CMD)?;
        info!("Command handler subscribed to: {}", TOPIC_CMD);
        Ok(())
    }

    /// Process a single received command (if any). Returns true if a command was processed.
    pub fn process_one<T: NvsPartitionId>(
        mqtt: &mut Mqtt,
        motion: &mut Motion<'_>,
        nvs: &mut EspNvs<T>,
        wifi: &mut Wifi<'_>,
        current_version: &Version,
        admin_cfg: &AdminSwitches,
        recovery_cfg: &RecoverySwitches,
        homing_cfg: &BootHomingSwitches,
        publish_mqtt: bool,
        persist_nvs: bool,
    ) -> anyhow::Result<bool> {
        let Some((topic, payload)) = mqtt.try_receive() else {
            return Ok(false);
        };

        if topic != TOPIC_CMD {
            warn!("Received message on unexpected topic: {}", topic);
            return Ok(false);
        }

        info!("Processing command: payload_len={}", payload.len());
        let payload_str = match std::str::from_utf8(&payload) {
            Ok(s) => s,
            Err(e) => {
                error!("Command payload is not valid UTF-8: {:?}", e);
                Self::publish_error(mqtt, publish_mqtt, "invalid_encoding", "Command payload must be UTF-8");
                return Ok(true);
            }
        };

        let cmd: Command = match serde_json::from_str(payload_str) {
            Ok(c) => c,
            Err(e) => {
                error!("Failed to parse command JSON: {:?}", e);
                Self::publish_error(mqtt, publish_mqtt, "parse_error", &format!("Invalid JSON: {}", e));
                return Ok(true);
            }
        };

        match cmd {
            Command::RunTest { test } => {
                info!("Executing test: {}", test);
                let result = Self::run_test(
                    &test,
                    motion,
                    nvs,
                    mqtt,
                    wifi,
                    current_version,
                    admin_cfg,
                    recovery_cfg,
                    homing_cfg,
                    publish_mqtt,
                    persist_nvs,
                )?;
                Self::publish_result(mqtt, publish_mqtt, "run_test", &test, result);
            }
            Command::GetStatus => {
                let status = Self::get_status(motion, mqtt, wifi);
                Self::publish_result(mqtt, publish_mqtt, "get_status", "", status);
            }
        }

        Ok(true)
    }

    fn run_test<T: NvsPartitionId>(
        test_name: &str,
        motion: &mut Motion<'_>,
        nvs: &mut EspNvs<T>,
        mqtt: &mut Mqtt,
        wifi: &mut Wifi<'_>,
        current_version: &Version,
        admin_cfg: &AdminSwitches,
        recovery_cfg: &RecoverySwitches,
        homing_cfg: &BootHomingSwitches,
        publish_mqtt: bool,
        persist_nvs: bool,
    ) -> anyhow::Result<String> {
        // Create a minimal admin config that only enables the requested test.
        let mut test_cfg = *admin_cfg;
        test_cfg.tests.motor_test = test_name == "motor";
        test_cfg.tests.encoder_test = test_name == "encoder";
        test_cfg.tests.persistence_test = test_name == "persistence" || test_name == "nvs";
        test_cfg.tests.wifi_mqtt_test = test_name == "wifi_mqtt" || test_name == "wifi";
        test_cfg.run_recovery_on_start = false;
        test_cfg.run_homing_on_start = false;
        test_cfg.stop_after = false;

        if !test_cfg.tests.motor_test
            && !test_cfg.tests.encoder_test
            && !test_cfg.tests.persistence_test
            && !test_cfg.tests.wifi_mqtt_test
        {
            return Ok(format!("Unknown test: {}", test_name));
        }

        // Run the test via admin_mode (reuse existing test functions).
        match admin_mode::run(
            &test_cfg,
            motion,
            recovery_cfg,
            homing_cfg,
            nvs,
            mqtt,
            wifi,
            current_version,
            publish_mqtt,
            persist_nvs,
        ) {
            Ok(_) => Ok(format!("Test '{}' completed", test_name)),
            Err(e) => Ok(format!("Test '{}' failed: {}", test_name, e)),
        }
    }

    fn get_status(motion: &mut Motion<'_>, mqtt: &mut Mqtt, wifi: &mut Wifi<'_>) -> String {
        format!(
            r#"{{"location":{},"mqtt_connected":{},"wifi_state":"{:?}"}}"#,
            motion.location(),
            mqtt.is_connected(),
            wifi.state()
        )
    }

    fn publish_result(mqtt: &mut Mqtt, enabled: bool, cmd: &str, test: &str, result: String) {
        if !enabled {
            info!("MQTT disabled: skipping result publish for {} {}", cmd, test);
            return;
        }
        let payload = format!(r#"{{"cmd":"{}","test":"{}","result":"{}"}}"#, cmd, test, result);
        if let Err(e) = mqtt.publish(TOPIC_CMD_RESP, payload.as_bytes()) {
            warn!("Failed to publish command result: {:?}", e);
        }
    }

    fn publish_error(mqtt: &mut Mqtt, enabled: bool, error_type: &str, msg: &str) {
        if !enabled {
            return;
        }
        let payload = format!(r#"{{"error":"{}","msg":"{}"}}"#, error_type, msg);
        if let Err(e) = mqtt.publish(TOPIC_CMD_RESP, payload.as_bytes()) {
            warn!("Failed to publish error: {:?}", e);
        }
    }
}
