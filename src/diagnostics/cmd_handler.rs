// MQTT command handler for runtime diagnostics/test execution.
//
// Commands arrive as JSON on `device1/admin/cmd` topic.
// Results are published to `device1/admin/cmd/resp` topic.

use esp_idf_svc::nvs::{EspNvs, NvsPartitionId};
use log::{error, info, warn};
use motion::Motion;
use network::mqtt::Mqtt;
use semver::Version;
use wifi::wifi::Wifi;

use crate::diagnostics::admin_mode;
use crate::config_manager::{ConfigManager, RuntimeMode};
use crate::switchboard::{AdminSwitches, BootHomingSwitches, RecoverySwitches};
use serde_json::Value;

fn topic_cmd(device_id: &str) -> String {
    format!("{}/admin/cmd", device_id)
}

fn topic_cmd_resp(device_id: &str) -> String {
    format!("{}/admin/cmd/resp", device_id)
}

#[derive(serde::Deserialize, Debug)]
#[serde(tag = "cmd")]
enum Command {
    #[serde(rename = "run_test")]
    RunTest { test: String },
    #[serde(rename = "get_status")]
    GetStatus,
    #[serde(rename = "set_mode")]
    SetMode { mode: String },
    #[serde(rename = "set_config")]
    SetConfig { key: String, value: Value },
    #[serde(rename = "get_config")]
    GetConfig { key: Option<String> },
    #[serde(rename = "reset_config")]
    ResetConfig,
}
 
pub struct CommandHandler;

impl CommandHandler {
    /// Subscribe to command topic and return handler instance.
    pub fn init(mqtt: &mut Mqtt, device_id: &str) -> anyhow::Result<()> {
        let t = topic_cmd(device_id);
        mqtt.subscribe(&t)?;
        info!("Command handler subscribed to: {}", t);
        Ok(())
    }

    /// Process a single received command (if any). Returns true if a command was processed.
    /// 
    /// # Parameters
    /// - `actual_heading`: Mutable reference to current heading. Updated after tests that move the motor.
    pub fn process_one<T: NvsPartitionId>(
        mqtt: &mut Mqtt,
        motion: &mut Motion<'_>,
        nvs: &mut EspNvs<T>,
        wifi: &mut Wifi<'_>,
        current_version: &Version,
        admin_cfg: &AdminSwitches,
        recovery_cfg: &RecoverySwitches,
        homing_cfg: &BootHomingSwitches,
        config_manager: &mut ConfigManager,
        actual_heading: &mut f32,
        publish_mqtt: bool,
        persist_nvs: bool,
        device_id: &str,
        formatted_time: &str,
    ) -> anyhow::Result<bool> {
        let Some((topic, payload)) = mqtt.try_receive() else {
            // No message available - this is normal, not an error
            return Ok(false);
        };

        info!("CommandHandler: Received message on topic: {}", topic);
        
        let expected = topic_cmd(device_id);
        if topic != expected {
            warn!("Received message on unexpected topic: {} (expected: {})", topic, expected);
            return Ok(false);
        }

        info!("Processing command: payload_len={}", payload.len());
        let payload_str = match std::str::from_utf8(&payload) {
            Ok(s) => s,
            Err(e) => {
                error!("Command payload is not valid UTF-8: {:?}", e);
                Self::publish_error(mqtt, device_id, publish_mqtt, "invalid_encoding", "Command payload must be UTF-8");
                return Ok(true);
            }
        };

        let cmd: Command = match serde_json::from_str(payload_str) {
            Ok(c) => c,
            Err(e) => {
                error!("Failed to parse command JSON: {:?}", e);
                Self::publish_error(mqtt, device_id, publish_mqtt, "parse_error", &format!("Invalid JSON: {}", e));
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
                    config_manager,
                    actual_heading,
                    publish_mqtt,
                    persist_nvs,
                    device_id,
                    formatted_time,
                )?;
                Self::publish_result(mqtt, device_id, publish_mqtt, "run_test", &test, result);
            }
            Command::GetStatus => {
                let status = Self::get_status(motion, mqtt, wifi);
                Self::publish_result(mqtt, device_id, publish_mqtt, "get_status", "", status);
            }
            Command::SetMode { mode } => {
                let mode_clone = mode.clone();
                let result = Self::set_mode(mode, config_manager, nvs)?;
                Self::publish_result(mqtt, device_id, publish_mqtt, "set_mode", &mode_clone, result);
            }
            Command::SetConfig { key, value } => {
                let result = Self::set_config(&key, value, config_manager, nvs)?;
                Self::publish_result(mqtt, device_id, publish_mqtt, "set_config", &key, result);
            }
            Command::GetConfig { key } => {
                let result = Self::get_config(key.as_deref(), config_manager);
                Self::publish_result(mqtt, device_id, publish_mqtt, "get_config", key.as_deref().unwrap_or("all"), result);
            }
            Command::ResetConfig => {
                let result = Self::reset_config(config_manager, nvs)?;
                Self::publish_result(mqtt, device_id, publish_mqtt, "reset_config", "", result);
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
        config_manager: &mut ConfigManager,
        actual_heading: &mut f32,
        publish_mqtt: bool,
        persist_nvs: bool,
        device_id: &str,
        formatted_time: &str,
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

        // Publish "Starting test" message
        let start_msg = format!("Starting test: {}", test_name);
        info!("{}", start_msg);
        if publish_mqtt {
            let payload = format!(r#"{{"status":"starting","test":"{}"}}"#, test_name);
            if let Err(e) = mqtt.publish(&topic_cmd_resp(device_id), payload.as_bytes()) {
                warn!("Failed to publish test start message: {:?}", e);
            }
        }

        // Run the test via admin_mode (reuse existing test functions).
        // Pass bypass_enabled_check=true to allow running tests from Normal mode
        // without requiring switchboard.admin.enabled=true
        let result = match admin_mode::run(
            device_id,
            formatted_time.to_string(),
            &test_cfg,
            motion,
            recovery_cfg,
            homing_cfg,
            nvs,
            mqtt,
            wifi,
            current_version,
            config_manager,
            publish_mqtt,
            persist_nvs,
            true, // bypass_enabled_check: allow running from Normal mode
        ) {
            Ok(_) => {
                // Test execution completed successfully
                // Note: Individual test functions log PASS/FAIL internally
                // We return a generic success message here
                format!("Test '{}' completed successfully. Check logs for PASS/FAIL details.", test_name)
            }
            Err(e) => {
                format!("Test '{}' execution failed: {}", test_name, e)
            }
        };

        // If test moved the motor (motor_test or encoder_test), we need to re-home
        // to re-establish position before returning to Normal mode tracking
        let test_moved_motor = test_cfg.tests.motor_test || test_cfg.tests.encoder_test;
        if test_moved_motor && homing_cfg.enabled {
            info!("Test moved motor - re-homing to re-establish position before returning to Normal mode");
            
            use crate::switchboard::Direction;
            let homing_ok = match homing_cfg.dir {
                Direction::Cw => motion.find_limit_switch_cw(),
                Direction::Ccw => motion.find_limit_switch_ccw(),
            };
            
            if homing_ok {
                // Update actual_heading to match homed position
                // motion.location() returns the correct heading after homing (HOME_HEADING_DEG)
                *actual_heading = motion.location();
                info!("Re-homing successful. Updated actual_heading to: {}", *actual_heading);
                
                // Save to NVS if persistence enabled
                if persist_nvs {
                    use crate::infra::SnapshotStore;
                    SnapshotStore::new(nvs, true).save_heading(*actual_heading);
                    use motion::MotionMode;
                    if motion.motion_mode() == MotionMode::EncoderGuarded {
                        SnapshotStore::new(nvs, true)
                            .save_encoder_snapshot(motion.encoder_ticks_adjusted());
                    }
                    info!("Saved re-homed position to NVS");
                }
                
                // Update result message to include re-homing info
                let result_with_homing = format!("{}. Re-homed successfully.", result);
                if publish_mqtt {
                    let payload = format!(r#"{{"status":"completed","test":"{}","result":"{}"}}"#, test_name, result_with_homing);
                    if let Err(e) = mqtt.publish(&topic_cmd_resp(device_id), payload.as_bytes()) {
                        warn!("Failed to publish test completion message: {:?}", e);
                    }
                }
                return Ok(result_with_homing);
            } else {
                error!("Re-homing failed after test! Position may be incorrect.");
                let result_with_error = format!("{}. WARNING: Re-homing failed - position may be incorrect!", result);
                if publish_mqtt {
                    let payload = format!(r#"{{"status":"completed","test":"{}","result":"{}"}}"#, test_name, result_with_error);
                    if let Err(e) = mqtt.publish(&topic_cmd_resp(device_id), payload.as_bytes()) {
                        warn!("Failed to publish test completion message: {:?}", e);
                    }
                }
                return Ok(result_with_error);
            }
        }

        // Publish completion message with status (for tests that don't move motor)
        info!("Test execution finished: {}", result);
        if publish_mqtt {
            let payload = format!(r#"{{"status":"completed","test":"{}","result":"{}"}}"#, test_name, result);
            if let Err(e) = mqtt.publish(&topic_cmd_resp(device_id), payload.as_bytes()) {
                warn!("Failed to publish test completion message: {:?}", e);
            }
        }

        Ok(result)
    }

    fn get_status(motion: &mut Motion<'_>, mqtt: &mut Mqtt, wifi: &mut Wifi<'_>) -> String {
        format!(
            r#"{{"location":{},"mqtt_connected":{},"wifi_state":"{:?}"}}"#,
            motion.location(),
            mqtt.is_connected(),
            wifi.state()
        )
    }

    fn publish_result(mqtt: &mut Mqtt, device_id: &str, enabled: bool, cmd: &str, test: &str, result: String) {
        if !enabled {
            info!("MQTT disabled: skipping result publish for {} {}", cmd, test);
            return;
        }
        let payload = format!(r#"{{"cmd":"{}","test":"{}","result":"{}"}}"#, cmd, test, result);
        if let Err(e) = mqtt.publish(&topic_cmd_resp(device_id), payload.as_bytes()) {
            warn!("Failed to publish command result: {:?}", e);
        }
    }

    fn publish_error(mqtt: &mut Mqtt, device_id: &str, enabled: bool, error_type: &str, msg: &str) {
        if !enabled {
            return;
        }
        let payload = format!(r#"{{"error":"{}","msg":"{}"}}"#, error_type, msg);
        if let Err(e) = mqtt.publish(&topic_cmd_resp(device_id), payload.as_bytes()) {
            warn!("Failed to publish error: {:?}", e);
        }
    }

    fn set_mode<T: NvsPartitionId>(
        mode_str: String,
        config_manager: &mut ConfigManager,
        nvs: &mut EspNvs<T>,
    ) -> anyhow::Result<String> {
        let mode = RuntimeMode::from_str(&mode_str)
            .ok_or_else(|| anyhow::anyhow!("Invalid mode: {}. Must be 'admin' or 'normal'", mode_str))?;
        
        config_manager.set_runtime_mode(mode, nvs)?;
        
        Ok(format!("Runtime mode set to: {}", mode.as_str()))
    }

    fn set_config<T: NvsPartitionId>(
        key: &str,
        value: Value,
        config_manager: &mut ConfigManager,
        nvs: &mut EspNvs<T>,
    ) -> anyhow::Result<String> {
        config_manager.set_runtime(key, value, nvs)?;
        Ok(format!("Config '{}' updated", key))
    }

    fn get_config(key: Option<&str>, config_manager: &ConfigManager) -> String {
        match key {
            Some(k) => {
                match config_manager.get(k) {
                    Some(val) => format!(r#"{{"key":"{}","value":{}}}"#, k, serde_json::to_string(&val).unwrap_or_default()),
                    None => format!(r#"{{"key":"{}","value":null,"note":"No override found, using .env/default"}}"#, k),
                }
            }
            None => {
                // Return all overrides
                let overrides = config_manager.get_all_overrides();
                serde_json::to_string(overrides).unwrap_or_else(|_| "{}".to_string())
            }
        }
    }

    fn reset_config<T: NvsPartitionId>(
        config_manager: &mut ConfigManager,
        nvs: &mut EspNvs<T>,
    ) -> anyhow::Result<String> {
        config_manager.reset_to_defaults(nvs)?;
        Ok("All runtime config overrides cleared".to_string())
    }
}
