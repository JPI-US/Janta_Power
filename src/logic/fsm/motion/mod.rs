use core::option::Option::None;
use std::time::Instant;

use anyhow::anyhow;
use chrono::{DateTime, Local};
use clock::Clock;
use esp_idf_hal::i2c::I2cDriver;
use esp_idf_svc::nvs::{EspDefaultNvsPartition, EspNvs, NvsDefault};
use fsm::state::State;
use log::{info, warn};
use motion::motion::{Motion, MotionMode};
use network::telemetry::Component;
use shared_bus::{BusManager, I2cProxy};

use crate::{
    config::switchboard::Switchboard,
    logic::{
        encoder_fault::EncoderFaultRecovery,
        fsm::{
            motion::maintenance::MaintenanceAction,
            FSMAddress,
            FSMCommand::{self},
            FSMState,
        },
    },
    storage::snapshot_store::SnapshotStore,
};

pub mod error_loop;
pub mod homing;
pub mod init;
pub mod maintenance;
pub mod moving;
pub mod tracking;

pub struct MotionContext {
    motion: Motion<'static>,
    switchboard: Switchboard,
    nvs: EspNvs<NvsDefault>,
    i2c_bus: &'static BusManager<std::sync::Mutex<I2cDriver<'static>>>,
    calculation: Option<Clock<I2cProxy<'static, std::sync::Mutex<I2cDriver<'static>>>>>,
    trust_nvs_state: bool,
    restored_from_snapshot: bool,
    actual_heading: f32,
    encoder_fault: EncoderFaultRecovery,
    clock: Option<Clock<I2cProxy<'static, std::sync::Mutex<I2cDriver<'static>>>>>,
}

impl MotionContext {
    pub fn new(
        motion: Motion<'static>,
        switchboard: Switchboard,
        nvs_partition: EspDefaultNvsPartition,
        i2c_bus: &'static BusManager<std::sync::Mutex<I2cDriver<'static>>>,
        trust_nvs_state: bool,
    ) -> Self {
        let nvs = match EspNvs::new(nvs_partition, "storage", true) {
            Ok(nvs) => {
                info!("Got namespace {:?} from default partition", "storage");
                nvs
            }
            Err(e) => Err(anyhow!("Could't get namespace {:?}", e)).expect("Failed to get NVS"),
        };

        Self {
            motion,
            switchboard,
            nvs,
            i2c_bus,
            calculation: None,
            trust_nvs_state,
            restored_from_snapshot: false,
            actual_heading: 0.0,
            encoder_fault: EncoderFaultRecovery::new(),
            clock: None,
        }
    }
}

pub struct MotionInit;
pub struct MotionBeginHoming;
pub struct MotionHoming {
    stall_prev: bool,
    steps_left: i64,
}

pub struct MotionMoving {
    steps: i64,
}
pub struct MotionErrorLoop {
    component: Component,
    message: String,
    notes: String,
}
pub struct MotionTracking;
pub struct MotionTrackingWait {
    begin: Instant,
}
pub struct MotionMaintenance {
    action: MaintenanceAction,
    return_to:
        Option<Box<dyn State<FSMAddress, MotionContext, FSMCommand, FSMState> + Send + 'static>>,
}

/// Reset daily encoder mode at day rollover.
pub(crate) fn check_daily_encoder_reset<T: esp_idf_svc::nvs::NvsPartitionId>(
    nvs: &mut esp_idf_svc::nvs::EspNvs<T>,
    local_time: &DateTime<Local>,
    persist_nvs: bool,
) -> bool {
    let mut snapshot_store = SnapshotStore::new(nvs, persist_nvs);

    let encoder_daily_mode = snapshot_store.load_encoder_daily_mode();
    if !encoder_daily_mode {
        return false;
    }

    let current_date = local_time.format("%Y-%m-%d").to_string();

    let stored_date = snapshot_store.load_encoder_mode_reset_date();

    match stored_date {
        Some(stored) if stored != current_date => {
            info!("Daily reset: New day detected (stored={}, current={}), resetting encoder mode to EncoderGuarded", stored, current_date);
            snapshot_store.save_tracking_mode(MotionMode::EncoderGuarded);
            snapshot_store.save_encoder_daily_mode(false);
            snapshot_store.save_encoder_mode_reset_date(&current_date);
            true
        }
        Some(_stored) => false,
        None => {
            warn!("encoder_daily_mode is true but no reset_date found in NVS; initializing reset_date");
            snapshot_store.save_encoder_mode_reset_date(&current_date);
            false
        }
    }
}
