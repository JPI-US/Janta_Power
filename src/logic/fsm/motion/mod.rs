use core::option::Option::None;
use std::time::Instant;

use anyhow::anyhow;
use clock::Clock;
use esp_idf_hal::i2c::I2cDriver;
use esp_idf_svc::nvs::{EspDefaultNvsPartition, EspNvs, NvsDefault};
use fsm::state::State;
use log::info;
use motion::motion::{Motion, MotionMode};
use network::telemetry::Component;
use shared_bus::{BusManager, I2cProxy};

use crate::{
    config::switchboard::Switchboard,
    logic::{
        encoder_fault::EncoderFaultRecovery,
        fsm::{
            motion::helpers::MaintenanceAction,
            FSMAddress,
            FSMCommand::{self},
            FSMState,
        },
    },
};

pub mod error_loop;
mod helpers;
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
    motion_mode: MotionMode,
    previous_motion_mode: MotionMode,
    restored_from_snapshot: bool,
    actual_heading: f32,
    encoder_fault: EncoderFaultRecovery,
    need_rehome: bool,
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
            motion_mode: MotionMode::EncoderGuarded,
            previous_motion_mode: MotionMode::EncoderGuarded,
            need_rehome: false,
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
