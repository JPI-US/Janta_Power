use core::{option::Option::None, time::Duration};
use std::{
    sync::mpsc::{Receiver, Sender},
    thread::sleep,
    time::Instant,
};

use anyhow::Context;
use esp_idf_svc::{
    eventloop::EspSystemEventLoop,
    hal::modem::Modem,
    nvs::{EspDefaultNvsPartition, EspNvs, NvsDefault},
};
use fsm::{InitialState, State};
use log::{error, info};
use network::mqtt::Mqtt;
use rtc::Rtc;
use wifi::wifi::{Wifi, WifiState};

use crate::{config::switchboard::Switchboard, logic::fsm::FSMCommand};

pub struct NetworkContext {
    nvs: EspNvs<NvsDefault>,
    partition: Option<EspDefaultNvsPartition>,
    sysloop: Option<EspSystemEventLoop>,
    modem: Option<Modem>,
    switchboard: Switchboard,
    wifi: Option<Wifi<'static>>,
    timer: Instant,
    init_network_services: bool,
    rtc: Rtc,
    mqtt: Option<Mqtt>,
}

impl NetworkContext {
    pub fn new(
        nvs: EspNvs<NvsDefault>,
        partition: EspDefaultNvsPartition,
        sysloop: EspSystemEventLoop,
        modem: Modem,
        switchboard: Switchboard,
        rtc: Rtc,
    ) -> Self {
        Self {
            nvs,
            partition: Some(partition),
            sysloop: Some(sysloop),
            modem: Some(modem),
            switchboard,
            wifi: None,
            timer: Instant::now(),
            init_network_services: true,
            rtc,
            mqtt: None,
        }
    }
}

pub struct WifiInitialize;
pub struct WifiConnectIfDisconnected;
pub struct WifiWait;
pub struct InitNetworkServices;

impl InitialState<NetworkContext, FSMCommand> for WifiInitialize {}

impl State<NetworkContext, FSMCommand> for WifiInitialize {
    fn process(
        &mut self,
        ctx: &mut NetworkContext,
        _tx: &mut Sender<FSMCommand>,
        _rx: &mut Receiver<FSMCommand>,
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

        let modem = ctx.modem.take().unwrap();
        let sysloop = ctx.sysloop.take().unwrap();
        let partition = ctx.partition.take().unwrap();

        let wifi = Wifi::new(modem, sysloop, partition, ssid, pass)
            .context("Failed to initialize WiFi")?;
        ctx.wifi = Some(wifi);

        Ok(Some(Box::new(WifiWait)))
    }
}

impl State<NetworkContext, FSMCommand> for WifiWait {
    fn process(
        &mut self,
        ctx: &mut NetworkContext,
        _tx: &mut Sender<FSMCommand>,
        _rx: &mut Receiver<FSMCommand>,
    ) -> anyhow::Result<Option<Box<dyn State<NetworkContext, FSMCommand> + Send>>> {
        if ctx.timer.elapsed() < Duration::from_secs(30) {
            return Ok(Some(Box::new(WifiWait)));
        }

        ctx.timer = Instant::now();
        Ok(Some(Box::new(WifiConnectIfDisconnected)))
    }
}

impl State<NetworkContext, FSMCommand> for WifiConnectIfDisconnected {
    fn process(
        &mut self,
        ctx: &mut NetworkContext,
        _tx: &mut Sender<FSMCommand>,
        _rx: &mut Receiver<FSMCommand>,
    ) -> anyhow::Result<Option<Box<dyn State<NetworkContext, FSMCommand> + Send>>> {
        let wifi = ctx
            .wifi
            .as_mut()
            .context("Wifi should always be initialized by now")?;

        if wifi.state() == WifiState::Disconnected {
            info!("WiFi disconnected; attempting to reconnect.");

            if wifi.reconnect_if_disconnected().is_err() {
                return Ok(Some(Box::new(WifiConnectIfDisconnected)));
            }
        }

        if ctx.init_network_services && matches!(wifi.state(), WifiState::Connected(_)) {
            Ok(Some(Box::new(InitNetworkServices)))
        } else {
            Ok(Some(Box::new(WifiWait)))
        }
    }
}

impl State<NetworkContext, FSMCommand> for InitNetworkServices {
    fn process(
        &mut self,
        ctx: &mut NetworkContext,
        _tx: &mut Sender<FSMCommand>,
        _rx: &mut Receiver<FSMCommand>,
    ) -> anyhow::Result<Option<Box<dyn State<NetworkContext, FSMCommand> + Send>>> {
        info!("Getting current time");

        const FORCE_NTP_SKIP_RTC: bool = false;
        let mut tz_buf = [0u8; 96];
        let tz_posix_str = ctx
            .nvs
            .get_str("tz_posix", &mut tz_buf)?
            .unwrap_or(ctx.switchboard.default_tz_posix);
        {
            ctx.rtc.init(
                &ctx.wifi
                    .as_ref()
                    .expect("Wifi should always be initialized by now"),
                tz_posix_str,
                FORCE_NTP_SKIP_RTC,
            )?;
        }
        let local_time_boot = rtc::timezone::local_time();
        let formatted_time = format!("{}", local_time_boot.format("%d/%m/%Y %H:%M:%S"));
        info!("{}", formatted_time);

        // MQTT
        info!("Initializing MQTT");
        // AWS IoT Core uses TLS client certificates baked into the firmware for
        // authentication; no username/password plumbing is required at runtime.
        // Broker URL is still hardcoded here; client ID is derived from DEVICE_ID so
        // fleet identity stays in `.env` with the MQTT topic/cert identity.

        let mqtt_client_id = format!("tower_{}", ctx.switchboard.device_id);
        match Mqtt::new_mqtt(
            "mqttS://a2exykcl6t998u-ats.iot.us-east-1.amazonaws.com:8883",
            &mqtt_client_id,
        ) {
            Ok(mqtt) => {
                info!("MQTT initialized");
                ctx.mqtt = Some(mqtt);
            }
            Err(e) => {
                error!("Failed to initialize MQTT: {:#}", e);
                ctx.mqtt = None;
            }
        }

        ctx.init_network_services = false;
        Ok(Some(Box::new(WifiConnectIfDisconnected)))
    }
}
