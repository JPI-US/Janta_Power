use core::option::Option::None;
use std::sync::mpsc::{Receiver, Sender};

use anyhow::anyhow;
use esp_idf_svc::{
    eventloop::EspSystemEventLoop,
    log::EspLogger,
    nvs::{EspDefaultNvsPartition, EspNvs, NvsDefault},
};
use fsm::{InitialState, State};
use log::info;

use crate::{
    config::{
        constants,
        switchboard::{self, Switchboard},
    },
    hardware::peripheral_map::PeripheralMap,
    logic::fsm::FSMCommand,
    storage::snapshot_store::SnapshotStore,
};

pub struct StartupContext {
    pub switchboard: Option<Switchboard>,
    pub sysloop: Option<EspSystemEventLoop>,
    pub nvs_default_partition: Option<EspDefaultNvsPartition>,
    pub peripherals: Option<PeripheralMap<'static>>,
}

impl StartupContext {
    pub fn new() -> Self {
        Self {
            switchboard: None,
            sysloop: None,
            nvs_default_partition: None,
            peripherals: None,
        }
    }
}

pub struct Initialization;

impl InitialState<StartupContext, FSMCommand> for Initialization {}

impl State<StartupContext, FSMCommand> for Initialization {
    fn process(
        &mut self,
        ctx: &mut StartupContext,
        _tx: &mut Sender<FSMCommand>,
        _rx: &mut Receiver<FSMCommand>,
    ) -> anyhow::Result<Option<Box<dyn State<StartupContext, FSMCommand> + Send>>> {
        // apply patches
        esp_idf_svc::sys::link_patches();

        // switchboard
        let switchboard = switchboard::active(switchboard::Profile::from_env_str(
            constants::ACTIVE_PROFILE_STR,
        ));
        ctx.switchboard = Some(switchboard);

        // Logger and event loop
        EspLogger::initialize_default();
        let sysloop = EspSystemEventLoop::take()?;
        ctx.sysloop = Some(sysloop);

        // nvs
        let nvs_default = EspDefaultNvsPartition::take()?;

        let mut nvs = match EspNvs::new(nvs_default.clone(), "storage", true) {
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

        ctx.nvs_default_partition = Some(nvs_default);

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

        ctx.peripherals = Some(peripherals);

        Ok(None)
    }
}
