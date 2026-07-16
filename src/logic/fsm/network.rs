use core::{option::Option::None, time::Duration};
use std::{
    sync::mpsc::{Receiver, Sender},
    thread::{self, sleep},
};

use anyhow::Context;
use esp_idf_svc::{
    eventloop::EspSystemEventLoop,
    hal::{
        delay::Ets,
        gpio::{Gpio4, Gpio5, Gpio6, Input, PinDriver},
        modem::Modem,
    },
    log::EspLogger,
    nvs::{EspDefaultNvsPartition, EspNvs, NvsDefault},
    ota::EspOta,
};
use fsm::{InitialState, State};
use log::{error, info};
use wifi::wifi::{Wifi, WifiState};

use crate::{config::switchboard::Switchboard, logic::fsm::FSMCommand};

pub struct NetworkContext {
    nvs: EspNvs<NvsDefault>,
    partition: Option<EspDefaultNvsPartition>,
    sysloop: Option<EspSystemEventLoop>,
    modem: Option<Modem>,
    switchboard: Switchboard,
    wifi: Option<Wifi<'static>>,
}

impl NetworkContext {
    pub fn new(
        nvs: EspNvs<NvsDefault>,
        partition: EspDefaultNvsPartition,
        sysloop: EspSystemEventLoop,
        modem: Modem,
        switchboard: Switchboard,
    ) -> Self {
        Self {
            nvs,
            partition: Some(partition),
            sysloop: Some(sysloop),
            modem: Some(modem),
            switchboard,
            wifi: None,
        }
    }
}

pub struct WifiInitialize;
pub struct WifiConnectImmediate;
pub struct WifiConnectIfDisconnected;
pub struct WifiWait;

impl InitialState<NetworkContext, FSMCommand> for WifiInitialize {}

impl State<NetworkContext, FSMCommand> for WifiInitialize {
    fn process(
        &mut self,
        ctx: &mut NetworkContext,
        tx: &mut Sender<FSMCommand>,
        rx: &mut Receiver<FSMCommand>,
    ) -> anyhow::Result<Option<Box<dyn State<NetworkContext, FSMCommand> + Send>>> {
        const PERSIST_NVS: bool = true;

        if PERSIST_NVS {
            match ctx
                .nvs
                .set_str("wifi_ssid", ctx.switchboard.default_wifi_ssid)
            {
                Ok(_) => info!("Wifi ssid updated"),
                Err(e) => error!("Wifi ssid not updated {:?}", e),
            };
            match ctx
                .nvs
                .set_str("wifi_pass", ctx.switchboard.default_wifi_pass)
            {
                Ok(_) => info!("Wifi password updated"),
                Err(e) => error!("Wifi password not updated {:?}", e),
            };
            match ctx
                .nvs
                .set_str("tz_posix", ctx.switchboard.default_tz_posix)
            {
                Ok(_) => info!("POSIX TZ string has been updated"),
                Err(e) => error!("tz_posix was not updated {:?}", e),
            };
        }

        let modem = ctx.modem.take().unwrap();
        let sysloop = ctx.sysloop.take().unwrap();
        let partition = ctx.partition.take().unwrap();

        let wifi = Wifi::new(modem, sysloop, partition).context("Failed to initialize WiFi")?;
        ctx.wifi = Some(wifi);

        Ok(Some(Box::new(WifiConnectImmediate)))
    }
}

impl State<NetworkContext, FSMCommand> for WifiConnectImmediate {
    fn process(
        &mut self,
        ctx: &mut NetworkContext,
        tx: &mut Sender<FSMCommand>,
        rx: &mut Receiver<FSMCommand>,
    ) -> anyhow::Result<Option<Box<dyn State<NetworkContext, FSMCommand> + Send>>> {
        let mut ssid_buf = [0u8; 64];
        let mut pass_buf = [0u8; 64];

        let ssid = match ctx.nvs.get_str("wifi_ssid", &mut ssid_buf) {
            Ok(Some(ssid)) => ssid,
            Ok(None) => ctx.switchboard.default_wifi_ssid,
            Err(e) => {
                error!("Failed to read WiFi SSID: {e:?}; falling back to default");
                ctx.switchboard.default_wifi_ssid
            }
        };

        let pass = match ctx.nvs.get_str("wifi_pass", &mut pass_buf) {
            Ok(Some(pass)) => pass,
            Ok(None) => ctx.switchboard.default_wifi_pass,
            Err(e) => {
                error!("Failed to read WiFi password: {e:?}; falling back to default");
                ctx.switchboard.default_wifi_pass
            }
        };

        info!("Connecting to wifi");

        ctx.wifi
            .as_mut()
            .context("Wifi should always be initialized by now")?
            .connect(ssid, pass)?;

        Ok(Some(Box::new(WifiConnectIfDisconnected)))
    }
}

impl State<NetworkContext, FSMCommand> for WifiConnectIfDisconnected {
    fn process(
        &mut self,
        ctx: &mut NetworkContext,
        tx: &mut Sender<FSMCommand>,
        rx: &mut Receiver<FSMCommand>,
    ) -> anyhow::Result<Option<Box<dyn State<NetworkContext, FSMCommand> + Send>>> {
        let wifi = ctx
            .wifi
            .as_mut()
            .context("Wifi should always be initialized by now")?;

        info!(
            "Wifi attempting to reconnect, if disconnected. Currently {:?}",
            wifi.state()
        );

        if wifi.state() == WifiState::Disconnected && wifi.reconnect_if_disconnected().is_err() {
            return Ok(Some(Box::new(WifiConnectIfDisconnected)));
        }

        Ok(Some(Box::new(WifiWait)))
    }
}

impl State<NetworkContext, FSMCommand> for WifiWait {
    fn process(
        &mut self,
        ctx: &mut NetworkContext,
        tx: &mut Sender<FSMCommand>,
        rx: &mut Receiver<FSMCommand>,
    ) -> anyhow::Result<Option<Box<dyn State<NetworkContext, FSMCommand> + Send>>> {
        sleep(Duration::from_secs(30));
        Ok(Some(Box::new(WifiConnectIfDisconnected)))
    }
}
