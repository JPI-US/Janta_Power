use anyhow::anyhow;
use esp_idf_svc::{
    eventloop::EspSystemEventLoop,
    log::EspLogger,
    nvs::{EspDefaultNvsPartition, EspNvs},
};
use log::info;
use serial_console::UsbSerialJtagConsole;

use crate::{
    config::{
        constants,
        switchboard::{self, Switchboard},
    },
    hardware::peripheral_map::PeripheralMap,
    logic::fsm::diagnostics::DiagnosticsContext,
    storage::snapshot_store::SnapshotStore,
};

pub struct StartupContext {
    pub switchboard: Switchboard,
    pub sysloop: EspSystemEventLoop,
    pub trust_nvs_state: bool,
    pub nvs_default_partition: EspDefaultNvsPartition,
    pub peripherals: PeripheralMap<'static>,
    /// `None` if the USB Serial/JTAG peripheral could not be claimed. The board
    /// runs regardless; it just cannot be asked about itself over USB.
    pub diagnostics_console: Option<UsbSerialJtagConsole>,
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

    // Claimed immediately after the logger, and before anything else is brought
    // up. Two reasons, both learned the hard way:
    //
    //   * The install depends on nothing — no peripherals, no NVS, no clock — so
    //     there is no reason to make the console wait for them. Claiming it here
    //     means it is answering while the rest of boot is still running.
    //   * It has to come *after* `EspLogger::initialize_default()`, or the log
    //     lines reporting whether the claim worked go nowhere. A diagnostic
    //     channel that fails silently is worse than one that is simply absent.
    let diagnostics_console = DiagnosticsContext::claim_console();

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
    let mut peripherals = PeripheralMap::new(
        switchboard.runtime.relay_active_level,
        switchboard.runtime.lmsw_active_level,
    )?;

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
        diagnostics_console,
    })
}
