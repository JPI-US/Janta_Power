use std::sync::mpsc::{Receiver, Sender};

use anyhow::anyhow;
use clock::Clock;
use esp_idf_hal::i2c::I2cDriver;
use esp_idf_svc::{
    eventloop::EspSystemEventLoop,
    hal::modem::Modem,
    nvs::{EspDefaultNvsPartition, EspNvs, NvsDefault},
    ota::EspOta,
};
use fsm::{drain_rx, InitialState, State, StateResult};
use log::{error, info};
use motion::Motion;
use shared_bus::{BusManager, I2cProxy};

use crate::{
    config::switchboard::Switchboard,
    logic::fsm::FSMCommand::{self, MotionMoveBy},
};

// TODO: Replace const
const PERSIST_NVS: bool = true;

pub struct MotionContext {
    motion: Motion<'static>,
    switchboard: Switchboard,
    nvs: EspNvs<NvsDefault>,
    i2c_bus: &'static BusManager<std::sync::Mutex<I2cDriver<'static>>>,
    calculation: Option<Clock<I2cProxy<'static, std::sync::Mutex<I2cDriver<'static>>>>>,
}

impl MotionContext {
    pub fn new(
        motion: Motion<'static>,
        switchboard: Switchboard,
        nvs_partition: EspDefaultNvsPartition,
        i2c_bus: &'static BusManager<std::sync::Mutex<I2cDriver<'static>>>,
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
        }
    }
}

#[derive(Default)]
pub struct MotionInit;

#[derive(Default)]
pub struct MotionNotMoving;

#[derive(Default)]
pub struct MotionMoving {
    by: i64,
}

impl InitialState<MotionContext, FSMCommand> for MotionInit {}

impl State<MotionContext, FSMCommand> for MotionInit {
    fn process(
        &mut self,
        ctx: &mut MotionContext,
        _tx: &mut Sender<FSMCommand>,
        _rx: &mut Receiver<FSMCommand>,
    ) -> anyhow::Result<StateResult<MotionContext, FSMCommand>> {
        // Tower location — seeded from `TOWER_LATITUDE` / `TOWER_LONGITUDE` in
        // `.env` via `Switchboard`. When `PERSIST_NVS` is on, the switchboard
        // defaults are (re)written into NVS on every boot, so updating `.env` and
        // reflashing updates the tower coordinates on the next boot.
        let tower_latitude: f64 = ctx.switchboard.default_tower_latitude;

        if PERSIST_NVS {
            match ctx
                .nvs
                .set_str("tower_latitude", &tower_latitude.to_string())
            {
                Ok(_) => info!("Tower latitude has been updated"),
                Err(e) => error!("Tower latitude was not updated {:?}", e),
            };
        }

        let tower_longitude: f64 = ctx.switchboard.default_tower_longitude;

        if PERSIST_NVS {
            match ctx
                .nvs
                .set_str("tower_longitude", &tower_longitude.to_string())
            {
                Ok(_) => info!("Tower longitude has been updated"),
                Err(e) => error!("Tower longitude was not updated {:?}", e),
            };
        }

        let mut lat_buf = [0u8; 64];
        let mut lon_buf = [0u8; 64];

        let latitude = ctx
            .nvs
            .get_str("tower_latitude", &mut lat_buf)?
            .unwrap_or("0")
            .parse()
            .unwrap_or(0.0);

        let longitude = ctx
            .nvs
            .get_str("tower_longitude", &mut lon_buf)?
            .unwrap_or("0")
            .parse()
            .unwrap_or(0.0);

        let altitude: f64 = 0.0;

        info!(
            "Retrieved latitude: {}, and longitude: {}",
            latitude, longitude
        );

        info!(
            "Device: {}, Lat: {}, Lon: {}, Alt: {}",
            ctx.switchboard.device_id, latitude, longitude, altitude
        );

        ctx.calculation = Some(Clock::new(
            ctx.i2c_bus.acquire_i2c(),
            latitude,
            longitude,
            altitude,
        ));

        Ok(StateResult::Running(Box::new(MotionNotMoving)))
    }
}

impl State<MotionContext, FSMCommand> for MotionNotMoving {
    fn process(
        &mut self,
        _ctx: &mut MotionContext,
        _tx: &mut Sender<FSMCommand>,
        rx: &mut Receiver<FSMCommand>,
    ) -> anyhow::Result<StateResult<MotionContext, FSMCommand>> {
        match drain_rx(rx) {
            Some(MotionMoveBy(by)) => Ok(StateResult::Running(Box::new(MotionMoving { by }))),
            _ => Ok(StateResult::Hold),
        }
    }
}

impl State<MotionContext, FSMCommand> for MotionMoving {
    fn process(
        &mut self,
        ctx: &mut MotionContext,
        _tx: &mut Sender<FSMCommand>,
        _rx: &mut Receiver<FSMCommand>,
    ) -> anyhow::Result<StateResult<MotionContext, FSMCommand>> {
        ctx.motion.move_by(self.by)?;

        Ok(StateResult::Running(Box::new(MotionNotMoving)))
    }
}
