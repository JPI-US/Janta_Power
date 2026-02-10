use std::time::Duration;

use esp_idf_svc::nvs::{EspNvs, NvsPartitionId};
use log::{error, info, warn};
use motion::Motion;
use network::mqtt::Mqtt;
use semver::Version;
use wifi::wifi::Wifi;

use crate::{
    app::boot_recovery,
    infra,
    switchboard::{AdminSwitches, BootHomingSwitches, Direction, RecoverySwitches},
};

/// Admin mode entrypoint.
///
/// Intent:
/// - Centralize "bench / diagnostics / manual" behavior behind ONE call from `main.rs`.
/// - Make it easy to turn on/off without hunting through normal runtime code paths.
///
/// Phase 2: this is a skeleton runner + switches. Test implementations can evolve later.
pub fn run<T: NvsPartitionId>(
    cfg: &AdminSwitches,
    motion: &mut Motion<'_>,
    recovery_cfg: &RecoverySwitches,
    homing_cfg: &BootHomingSwitches,
    nvs: &mut EspNvs<T>,
    mqtt: &mut Mqtt,
    wifi: &mut Wifi<'_>,
    current_version: &Version,
    publish_mqtt: bool,
) -> anyhow::Result<()> {
    if !cfg.enabled {
        error!("ADMIN_MODE refused: SWITCHBOARD.admin.enabled=false");
        infra::Telemetry::critical_failure_loop(
            mqtt,
            b"Critical failure: Admin mode selected but disabled in switchboard!",
            publish_mqtt,
        );
    }

    warn!("================ ADMIN MODE ================");
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

    // Keep the device "alive" on telemetry even in admin mode.
    infra::Telemetry::publish_firmware_version_if(mqtt, current_version, publish_mqtt);

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
                mqtt,
                b"Critical failure: Admin homing failed!",
                publish_mqtt,
            );
        }
        warn!("ADMIN: homing OK");
    }

    // Phase 2 test skeletons (no side-effects gating yet).
    if cfg.tests.motor_test {
        info!("ADMIN motor_test: enabled (Phase 2 skeleton; implement patterns in Phase 5)");
    }
    if cfg.tests.encoder_test {
        info!("ADMIN encoder_test: enabled (Phase 2 skeleton; implement tick direction checks in Phase 5)");
    }
    if cfg.tests.persistence_test {
        info!("ADMIN persistence_test: enabled (Phase 2 skeleton; implement sentinel R/W in Phase 5)");
        let _ = nvs; // placeholder to make intent explicit
    }
    if cfg.tests.wifi_mqtt_test {
        info!("ADMIN wifi_mqtt_test: enabled (Phase 2 skeleton; implement connectivity checks in Phase 5)");
        let _ = wifi; // placeholder to make intent explicit
    }

    if cfg.stop_after {
        warn!("ADMIN: stop_after=true; idling here (no tracking loop).");
        loop {
            infra::Telemetry::publish_firmware_version_if(mqtt, current_version, publish_mqtt);
            std::thread::sleep(Duration::from_secs(60));
        }
    }

    warn!("ADMIN: stop_after=false; returning to caller.");
    Ok(())
}

