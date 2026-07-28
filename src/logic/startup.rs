use anyhow::anyhow;
use esp_idf_svc::{
    eventloop::EspSystemEventLoop,
    log::EspLogger,
    nvs::{EspDefaultNvsPartition, EspNvs},
};
use log::info;

use crate::{
    config::{
        constants,
        switchboard::{self, Switchboard},
    },
    hardware::peripheral_map::PeripheralMap,
    storage::snapshot_store::SnapshotStore,
};

pub struct StartupContext {
    pub switchboard: Switchboard,
    pub sysloop: EspSystemEventLoop,
    pub trust_nvs_state: bool,
    pub nvs_default_partition: EspDefaultNvsPartition,
    pub peripherals: PeripheralMap<'static>,
}

pub fn startup() -> anyhow::Result<StartupContext> {
    // apply patches
    esp_idf_svc::sys::link_patches();

    // switchboard
    let switchboard = switchboard::active(switchboard::Profile::from_env_str(
        constants::ACTIVE_PROFILE_STR,
    ));

    // Logger and event loop
    EspLogger::initialize_default();

    let sysloop = EspSystemEventLoop::take()?;

    // nvs
    let nvs_default_partition = EspDefaultNvsPartition::take()?;

    let mut nvs = match EspNvs::new(nvs_default_partition.clone(), "storage", true) {
        Ok(nvs) => {
            info!("Got namespace {:?} from default partition", "storage");
            nvs
        }
        Err(e) => Err(anyhow!("Could't get namespace {:?}", e))?,
    };

    let last_run_normal = SnapshotStore::new(&mut nvs, true).load_last_run_normal_or_init(true);
    let trust_nvs_state = last_run_normal;
    info!(
        "Last run normal={} -> trust_nvs_state={} (active_mode=Normal)",
        last_run_normal, trust_nvs_state
    );

    // peripherals
    let mut peripherals = PeripheralMap::new()?;

    peripherals.led.display_none()?;

    // motion
    peripherals.motion.init();

    let _ = peripherals.motion.run();

    // Runtime guardrails from switchboard
    peripherals
        .motion
        .set_stall_detection_enabled(switchboard.runtime.guardrails.stall_detection_enabled);

    peripherals.motion.set_soft_limits(
        switchboard.runtime.guardrails.soft_limits_enabled,
        switchboard.runtime.guardrails.soft_limit_min_deg,
        switchboard.runtime.guardrails.soft_limit_max_deg,
    );

    Ok(StartupContext {
        switchboard,
        sysloop,
        trust_nvs_state,
        nvs_default_partition,
        peripherals,
    })
}
