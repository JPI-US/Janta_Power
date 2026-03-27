use std::time::Duration;

use esp_idf_svc::nvs::{EspNvs, NvsPartitionId};
use log::{error, info, warn};
use motion::{calculate_steps, Motion, MoveOutcome};
use network::mqtt::Mqtt;
use semver::Version;
use wifi::wifi::{Wifi, WifiState};

use crate::{
    config_manager::ConfigManager,
    diagnostics::{boot_recovery, cmd_handler},
    infra,
    switchboard::{AdminSwitches, BootHomingSwitches, Direction, RecoverySwitches},
};

const TOPIC_ADMIN_PERSIST: &str = "device1/admin/persistence_test";
const TOPIC_ADMIN_WIFI_MQTT: &str = "device1/admin/wifi_mqtt_test";
const TOPIC_ADMIN_MOTOR: &str = "device1/admin/motor_test";
const TOPIC_ADMIN_ENCODER: &str = "device1/admin/encoder_test";

const NVS_KEY_ADMIN_PERSIST_COUNTER: &str = "admin_persist_ctr";

/// Admin mode entrypoint.
///
/// Intent:
/// - Centralize "bench / diagnostics / manual" behavior behind ONE call from `main.rs`.
/// - Make it easy to turn on/off without hunting through normal runtime code paths.
///
/// # Parameters
/// - `bypass_enabled_check`: If true, skip the `cfg.enabled` check. Used when running tests
///   from Normal mode via MQTT commands (temporary admin mode).
pub fn run<T: NvsPartitionId>(
    device_id: &str,
    formatted_time: String,
    cfg: &AdminSwitches,
    motion: &mut Motion<'_>,
    recovery_cfg: &RecoverySwitches,
    homing_cfg: &BootHomingSwitches,
    nvs: &mut EspNvs<T>,
    mqtt: &mut Mqtt,
    wifi: &mut Wifi<'_>,
    current_version: &Version,
    config_manager: &mut ConfigManager,
    publish_mqtt: bool,
    persist_nvs: bool,
    bypass_enabled_check: bool,
) -> anyhow::Result<()> {
    if !bypass_enabled_check && !cfg.enabled {
        error!("ADMIN_MODE refused: SWITCHBOARD.admin.enabled=false");
        infra::Telemetry::critical_failure_loop(
            device_id,
            mqtt,
            b"Critical failure: Admin mode selected but disabled in switchboard! at Tower 1 (Office Tower)",
            publish_mqtt,
        );
    }

    if bypass_enabled_check {
        info!("================ TEMPORARY ADMIN MODE (from Normal mode) ================");
    } else {
        warn!("================ ADMIN MODE ================");
    }
    warn!(
        "Admin switches: run_recovery_on_start={} run_homing_on_start={} stop_after={}",
        cfg.run_recovery_on_start, cfg.run_homing_on_start, cfg.stop_after
    );
    warn!(
        "Admin tests: motor_test={} encoder_test={} persistence_test={} wifi_mqtt_test={}",
        cfg.tests.motor_test,
        cfg.tests.encoder_test,
        cfg.tests.persistence_test,
        cfg.tests.wifi_mqtt_test
    );
    warn!(
        "Admin effects: publish_mqtt={} persist_nvs={}",
        publish_mqtt, persist_nvs
    );

    // Keep the device "alive" on telemetry even in admin mode.
    infra::Telemetry::publish_firmware_version_if(
        device_id,
        mqtt,
        formatted_time.clone(),
        current_version,
        publish_mqtt,
    );

    // Optional startup actions (re-use existing primitives).
    if cfg.run_recovery_on_start {
        warn!("ADMIN: running recovery moves...");
        boot_recovery::run(motion, recovery_cfg);
    }

    if cfg.run_homing_on_start {
        warn!("ADMIN: running homing...");
        let ok = match homing_cfg.dir {
            Direction::Cw => motion.find_limit_switch_cw(),
            Direction::Ccw => motion.find_limit_switch_ccw(),
        };
        if !ok {
            infra::Telemetry::critical_failure_loop(
                device_id,
                mqtt,
                b"Critical failure: Admin homing failed! at Tower 1 (Office Tower)",
                publish_mqtt,
            );
        }
        warn!("ADMIN: homing OK");
    }

    // ================= Tests =================
    // Each test MUST respect effect toggles so Admin can be run safely.
    if cfg.tests.motor_test {
        run_motor_test(motion, mqtt, publish_mqtt)?;
    }
    if cfg.tests.encoder_test {
        run_encoder_test(motion, mqtt, publish_mqtt)?;
    }
    if cfg.tests.persistence_test {
        run_persistence_test(nvs, mqtt, publish_mqtt, persist_nvs)?;
    }
    if cfg.tests.wifi_mqtt_test {
        run_wifi_mqtt_test(wifi, mqtt, publish_mqtt)?;
    }

    if cfg.stop_after {
        warn!("ADMIN: stop_after=true; idling here (processing MQTT commands).");
        // In Admin mode idle loop, we don't track heading, but process_one needs it for tests
        let mut dummy_heading = motion.location();
        loop {
            // Process incoming MQTT commands (non-blocking, processes all queued commands)
            let mut commands_processed = 0;
            while commands_processed < 10 {
                match cmd_handler::CommandHandler::process_one(
                    mqtt,
                    motion,
                    nvs,
                    wifi,
                    current_version,
                    cfg,
                    recovery_cfg,
                    homing_cfg,
                    config_manager,
                    &mut dummy_heading, // Dummy heading for Admin mode (not used for tracking)
                    publish_mqtt,
                    persist_nvs,
                    device_id,
                    &formatted_time,
                ) {
                    Ok(true) => {
                        commands_processed += 1;
                        // Continue processing more commands
                    }
                    Ok(false) => {
                        // No more commands available
                        break;
                    }
                    Err(e) => {
                        warn!("Command processing error in admin mode: {:?}", e);
                        break; // Stop on error to avoid infinite loop
                    }
                }
            }
            if commands_processed > 0 {
                info!("Admin mode: Processed {} MQTT command(s)", commands_processed);
            }
            
            // Publish telemetry and sleep
            infra::Telemetry::publish_firmware_version_if(
                device_id,
                mqtt,
                formatted_time.clone(),
                current_version,
                publish_mqtt,
            );
            std::thread::sleep(Duration::from_secs(60));
        }
    }

    warn!("ADMIN: stop_after=false; returning to caller.");
    Ok(())
}

fn publish_if(mqtt: &mut Mqtt, enabled: bool, topic: &str, payload: &str) {
    if !enabled {
        info!("MQTT publish disabled: skipping {} publish (payload={})", topic, payload);
        return;
    }
    if let Err(e) = mqtt.publish(topic, payload.as_bytes()) {
        warn!("Failed to publish {}: {:?}", topic, e);
    }
}

fn run_persistence_test<T: NvsPartitionId>(
    nvs: &mut EspNvs<T>,
    mqtt: &mut Mqtt,
    publish_mqtt: bool,
    persist_nvs: bool,
) -> anyhow::Result<()> {
    // This test is about NVS *writes*, so if persistence is disabled, we can only report "skipped".
    let prev = nvs.get_u32(NVS_KEY_ADMIN_PERSIST_COUNTER)?.unwrap_or(0);

    if !persist_nvs {
        let msg = format!(
            "PERSIST_TEST SKIP: persist_nvs=false (prev_ctr={})",
            prev
        );
        warn!("{}", msg);
        publish_if(mqtt, publish_mqtt, TOPIC_ADMIN_PERSIST, &msg);
        return Ok(());
    }

    let next = prev.wrapping_add(1);
    nvs.set_u32(NVS_KEY_ADMIN_PERSIST_COUNTER, next)?;
    let readback = nvs.get_u32(NVS_KEY_ADMIN_PERSIST_COUNTER)?.unwrap_or(0);
    let ok = readback == next;

    let msg = format!(
        "PERSIST_TEST {}: prev_ctr={} wrote_next={} readback={}",
        if ok { "PASS" } else { "FAIL" },
        prev,
        next,
        readback
    );
    if ok {
        info!("{}", msg);
    } else {
        error!("{}", msg);
    }
    publish_if(mqtt, publish_mqtt, TOPIC_ADMIN_PERSIST, &msg);
    Ok(())
}

fn run_wifi_mqtt_test(
    wifi: &mut Wifi<'_>,
    mqtt: &mut Mqtt,
    publish_mqtt: bool,
) -> anyhow::Result<()> {
    let wifi_ok = matches!(wifi.state(), WifiState::Connected(_));
    let mqtt_ok = mqtt.is_connected();

    let msg = format!(
        "WIFI_MQTT_TEST: wifi_ok={} mqtt_ok={} wifi_state={:?}",
        wifi_ok,
        mqtt_ok,
        wifi.state()
    );
    if wifi_ok && mqtt_ok {
        info!("{}", msg);
    } else {
        warn!("{}", msg);
    }

    publish_if(mqtt, publish_mqtt, TOPIC_ADMIN_WIFI_MQTT, &msg);
    Ok(())
}

fn run_motor_test(motion: &mut Motion<'_>, mqtt: &mut Mqtt, publish_mqtt: bool) -> anyhow::Result<()> {
    // Conservative movement: tiny jog CW then CCW.
    const JOG_DEG: f32 = 8.0;

    let steps_cw = calculate_steps(JOG_DEG);
    let out1 = motion.move_by(steps_cw);
    let steps_ccw = calculate_steps(-JOG_DEG);
    let out2 = motion.move_by(steps_ccw);

    let ok = out1 == MoveOutcome::Completed && out2 == MoveOutcome::Completed;
    let msg = format!(
        "MOTOR_TEST {}: jog_deg={} out_cw={:?} out_ccw={:?}",
        if ok { "PASS" } else { "FAIL" },
        JOG_DEG,
        out1,
        out2
    );
    if ok { info!("{}", msg); } else { warn!("{}", msg); }
    publish_if(mqtt, publish_mqtt, TOPIC_ADMIN_MOTOR, &msg);
    Ok(())
}

fn run_encoder_test(motion: &mut Motion<'_>, mqtt: &mut Mqtt, publish_mqtt: bool) -> anyhow::Result<()> {
    // Tiny move and report encoder deltas. We don't enforce sign expectations yet (wiring can vary),
    // but we DO assert "ticks changed" for a healthy encoder.
    const JOG_DEG: f32 = 8.0;

    let start = motion.encoder_ticks_adjusted();
    let out_cw = motion.move_by(calculate_steps(JOG_DEG));
    let mid = motion.encoder_ticks_adjusted();
    let out_ccw = motion.move_by(calculate_steps(-JOG_DEG));
    let end = motion.encoder_ticks_adjusted();

    let cw_delta = mid - start;
    let ccw_delta = end - mid;

    let ticks_changed = cw_delta != 0 || ccw_delta != 0;
    let ok = ticks_changed && out_cw == MoveOutcome::Completed && out_ccw == MoveOutcome::Completed;

    let msg = format!(
        "ENCODER_TEST {}: jog_deg={} out_cw={:?} cw_delta={} out_ccw={:?} ccw_delta={} (start={} mid={} end={})",
        if ok { "PASS" } else { "FAIL" },
        JOG_DEG,
        out_cw,
        cw_delta,
        out_ccw,
        ccw_delta,
        start,
        mid,
        end
    );
    if ok { info!("{}", msg); } else { warn!("{}", msg); }
    publish_if(mqtt, publish_mqtt, TOPIC_ADMIN_ENCODER, &msg);
    Ok(())
}
