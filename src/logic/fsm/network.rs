use core::{option::Option::None, time::Duration};
use std::{thread::sleep, time::Instant};

use anyhow::Context;
use esp_idf_hal::{delay::Ets, i2c::I2cDriver};
use esp_idf_svc::{
    eventloop::EspSystemEventLoop,
    hal::modem::Modem,
    nvs::{EspDefaultNvsPartition, EspNvs, NvsDefault},
    ota::EspOta,
};
use fsm::{
    postal::{mailbox::Mailbox, Address},
    state::{InitialState, State, StateResult},
};
use hdc1080::Hdc1080;
use log::{error, info, warn};
use motion::motion::MotionMode;
use network::{mqtt::Mqtt, telemetry::publish_json};
use ota::OtaUpdater;
use rtc::Rtc;
use semver::Version;
use shared_bus::I2cProxy;
use wifi::wifi::{Wifi, WifiState};

use crate::{
    config::switchboard::Switchboard,
    hardware::temperature::report_system_temperature,
    logic::{fsm::FSMCommand, reset_reason::ResetReason},
    services::{commands, transport},
};

// TODO: Remove const
const PERSIST_NVS: bool = true;
const FORCE_NTP_SKIP_RTC: bool = false;
const ALLOW_BOOT_VALIDATION: bool = true;
const DEFAULT_VERSION: &str = "1.1.5";

pub struct NetworkContext {
    nvs: EspNvs<NvsDefault>,
    partition: Option<EspDefaultNvsPartition>,
    sysloop: Option<EspSystemEventLoop>,
    modem: Option<Modem>,
    switchboard: Switchboard,
    wifi: Option<Wifi<'static>>,
    init_network_services: bool,
    rtc: Rtc,
    mqtt: Option<Mqtt>,
    formatted_time: Option<String>,
    current_version: Option<Version>,
    last_heartbeat_instant: Instant,
    temperature_sensor:
        Option<Hdc1080<I2cProxy<'static, std::sync::Mutex<I2cDriver<'static>>>, Ets>>,
    motion_mode: Option<MotionMode>,
    actual_heading: Option<f32>,
}

impl NetworkContext {
    pub fn new(
        partition: EspDefaultNvsPartition,
        sysloop: EspSystemEventLoop,
        modem: Modem,
        switchboard: Switchboard,
        rtc: Rtc,
        temperature_sensor: Option<
            Hdc1080<I2cProxy<'static, std::sync::Mutex<I2cDriver<'static>>>, Ets>,
        >,
    ) -> Self {
        // TODO: Better error handling
        let nvs = match EspNvs::new(partition.clone(), "storage", true) {
            Ok(nvs) => {
                info!("Got namespace {:?} from default partition", "storage");
                nvs
            }
            Err(e) => panic!("Failed to get NVS; could't get namespace {:?}", e),
        };

        Self {
            nvs,
            partition: Some(partition),
            sysloop: Some(sysloop),
            modem: Some(modem),
            switchboard,
            wifi: None,
            init_network_services: true,
            rtc,
            mqtt: None,
            formatted_time: None,
            current_version: None,
            last_heartbeat_instant: Instant::now(),
            temperature_sensor,
            motion_mode: None,
            actual_heading: None,
        }
    }
}

pub struct WifiInitialize;
pub struct WifiConnectIfDisconnected;
pub struct WifiPublishHeartbeat;
pub struct InitNetworkServices;
pub struct BootValidation;
pub struct Ota;
pub struct MqttPublishJson(String, String);

impl<A> InitialState<A, NetworkContext, FSMCommand> for WifiInitialize where A: Address {}

impl<A> State<A, NetworkContext, FSMCommand> for WifiInitialize
where
    A: Address,
{
    fn process(
        &mut self,
        ctx: &mut NetworkContext,
        _channel: &mut Mailbox<A, FSMCommand>,
    ) -> anyhow::Result<StateResult<A, NetworkContext, FSMCommand>> {
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

        Ok(StateResult::Running(Box::new(WifiConnectIfDisconnected)))
    }
}

impl<A> State<A, NetworkContext, FSMCommand> for WifiConnectIfDisconnected
where
    A: Address,
{
    fn process(
        &mut self,
        ctx: &mut NetworkContext,
        mailbox: &mut Mailbox<A, FSMCommand>,
    ) -> anyhow::Result<StateResult<A, NetworkContext, FSMCommand>>
    where
        A: Address,
    {
        let wifi = ctx
            .wifi
            .as_mut()
            .context("Wifi should always be initialized by now")?;

        if wifi.state() == WifiState::Disconnected {
            info!("WiFi disconnected; attempting to reconnect.");

            if wifi.reconnect_if_disconnected().is_err() {
                return Ok(StateResult::Hold);
            }
        }

        if ctx.init_network_services && matches!(wifi.state(), WifiState::Connected(_)) {
            return Ok(StateResult::Running(Box::new(InitNetworkServices)));
        } else if let Ok(cmd) = mailbox.receive() {
            match cmd {
                FSMCommand::MqttPublishJson(payload, topic) => {
                    return Ok(StateResult::Running(Box::new(MqttPublishJson(
                        payload, topic,
                    ))));
                }
                FSMCommand::UpdateNetworkMotionContext(mode, actual_heading) => {
                    ctx.motion_mode = Some(mode);
                    ctx.actual_heading = Some(actual_heading);
                }
                _ => {}
            }
        }

        if ctx.last_heartbeat_instant.elapsed() > Duration::from_mins(15) {
            return Ok(StateResult::Running(Box::new(WifiPublishHeartbeat)));
        }

        Ok(StateResult::Hold)
    }
}

impl<A> State<A, NetworkContext, FSMCommand> for WifiPublishHeartbeat
where
    A: Address,
{
    fn process(
        &mut self,
        ctx: &mut NetworkContext,
        _channel: &mut Mailbox<A, FSMCommand>,
    ) -> anyhow::Result<StateResult<A, NetworkContext, FSMCommand>> {
        let local_time = rtc::timezone::local_time();
        let current_time = local_time.format("%d/%m/%Y %H:%M:%S").to_string();

        // publish heartbeat
        let firmware_version = ctx
            .current_version
            .as_ref()
            .map(|v| v.to_string())
            .unwrap_or_else(|| "unknown".to_string());

        let payload = network::telemetry::Heartbeat {
            current_time: &current_time,
            firmware_version: &firmware_version,
        };

        let topic = network::telemetry::topic::status(ctx.switchboard.device_id);

        if let Some(mqtt) = ctx.mqtt.as_mut() {
            if let Err(e) = network::telemetry::publish_json(mqtt, &topic, &payload) {
                log::error!("Failed to publish heartbeat: {:?}", e);
            }

            // publish temperature
            if let Some(ref mut sensor) = ctx.temperature_sensor {
                report_system_temperature(sensor, mqtt, ctx.switchboard.device_id, &current_time);
            }
        } else {
            log::warn!("Skipping heartbeat publish: MQTT client unavailable");
        }

        // Receive commands
        // Answer at most one queued remote command (gated by the switchboard).
        if ctx.switchboard.runtime.commands_enabled {
            let (motion_mode_str, firmware_version, mqtt, wifi, current_heading) = match (
                ctx.motion_mode,
                ctx.current_version.as_ref(),
                ctx.mqtt.as_mut(),
                ctx.wifi.as_ref(),
                ctx.actual_heading,
            ) {
                (
                    Some(MotionMode::StepperOnly),
                    Some(version),
                    Some(mqtt),
                    Some(wifi),
                    Some(heading),
                ) => ("stepper_only", version.to_string(), mqtt, wifi, heading),

                (
                    Some(MotionMode::EncoderGuarded),
                    Some(version),
                    Some(mqtt),
                    Some(wifi),
                    Some(heading),
                ) => ("encoder_guarded", version.to_string(), mqtt, wifi, heading),

                _ => {
                    warn!("Cannot process command: missing command context");
                    return Ok(StateResult::Running(Box::new(WifiConnectIfDisconnected)));
                }
            };

            let cmd_ctx = commands::CmdCtx {
                device_id: ctx.switchboard.device_id,
                firmware_version: &firmware_version,
                mqtt_connected: mqtt.is_connected(),
                wifi_connected: matches!(wifi.state(), WifiState::Connected(_)),
                motion_mode: motion_mode_str,
                current_heading,
            };

            if let Err(e) = transport::process_one(mqtt, ctx.switchboard.device_id, &cmd_ctx) {
                warn!("Command processing failed: {:?}", e);
            }
        }

        Ok(StateResult::Running(Box::new(WifiConnectIfDisconnected)))
    }
}

impl<A> State<A, NetworkContext, FSMCommand> for InitNetworkServices
where
    A: Address,
{
    fn process(
        &mut self,
        ctx: &mut NetworkContext,
        _channel: &mut Mailbox<A, FSMCommand>,
    ) -> anyhow::Result<StateResult<A, NetworkContext, FSMCommand>> {
        info!("Getting current time");

        let mut tz_buf = [0u8; 96];
        let tz_posix_str = ctx
            .nvs
            .get_str("tz_posix", &mut tz_buf)?
            .unwrap_or(ctx.switchboard.default_tz_posix);

        ctx.rtc.init(
            ctx.wifi
                .as_ref()
                .expect("Wifi should always be initialized by now"),
            tz_posix_str,
            FORCE_NTP_SKIP_RTC,
        )?;

        let local_time_boot = rtc::timezone::local_time();
        let formatted_time = format!("{}", local_time_boot.format("%d/%m/%Y %H:%M:%S"));

        info!("{}", formatted_time);
        ctx.formatted_time = Some(formatted_time);

        info!("Initializing MQTT");

        let mqtt_client_id = format!("tower_{}", ctx.switchboard.device_id);

        match Mqtt::new_mqtt(
            "mqttS://a2exykcl6t998u-ats.iot.us-east-1.amazonaws.com:8883",
            &mqtt_client_id,
        ) {
            Ok(mqtt) => {
                info!("MQTT initialized");

                ctx.mqtt = Some(mqtt);
                ctx.init_network_services = false;

                Ok(StateResult::Running(Box::new(BootValidation)))
            }
            Err(e) => {
                error!("Failed to initialize MQTT: {:#}; retrying", e);

                ctx.mqtt = None;
                ctx.init_network_services = true;

                Ok(StateResult::Hold)
            }
        }
    }
}

impl<A> State<A, NetworkContext, FSMCommand> for BootValidation
where
    A: Address,
{
    fn process(
        &mut self,
        ctx: &mut NetworkContext,
        _channel: &mut Mailbox<A, FSMCommand>,
    ) -> anyhow::Result<StateResult<A, NetworkContext, FSMCommand>> {
        let first_boot = ctx.nvs.get_u8("first_boot")?.unwrap_or(1);

        info!("Beginning boot validation");

        let mut version_buf = [0u8; 32];

        if PERSIST_NVS {
            ctx.nvs.set_str("version", DEFAULT_VERSION)?;
        }

        let current_version = ctx
            .nvs
            .get_str("version", &mut version_buf)?
            .map(|s| s.trim().parse::<Version>())
            .transpose()?
            .unwrap_or(Version::parse(DEFAULT_VERSION)?);

        ctx.current_version = Some(current_version.clone());

        let boot_diagnostic_result = if ALLOW_BOOT_VALIDATION {
            boot_diagnostic(
                ctx.switchboard.device_id,
                ctx.wifi
                    .as_mut()
                    .expect("Wifi should always be initialized by now"),
                ctx.mqtt
                    .as_mut()
                    .expect("MQTT should always be initialized by now"),
                &current_version,
            )
        } else {
            info!("Boot validation disabled");
            true
        };

        let mqtt = ctx
            .mqtt
            .as_mut()
            .expect("MQTT should always be initialized by now");

        if ALLOW_BOOT_VALIDATION && first_boot == 1 {
            info!("First boot, now performing boot diagnostics");

            let mut valid_ota = EspOta::new().context("Failed to get OTA instance")?;

            let running_slot = valid_ota.get_running_slot();

            info!("This is the running boot slot {:?}", running_slot);

            if running_slot.is_err() {
                error!("Failed to get running boot slot, skipping validation");
                return Ok(StateResult::Running(Box::new(Ota)));
            }

            if running_slot?.label == "factory" {
                info!("Running from factory partition -> skipping OTA validity marking");

                ctx.nvs.set_u8("first_boot", 0)?;
            } else if boot_diagnostic_result {
                info!("Boot validation passed, now marking firmware as valid");

                valid_ota.mark_running_slot_valid()?;
                ctx.nvs.set_u8("first_boot", 0)?;

                let mut prev_ver_buf = [0u8; 32];

                if let Some(prev_str) = ctx.nvs.get_str("prev_version", &mut prev_ver_buf)? {
                    match prev_str.trim().parse::<Version>() {
                        Ok(prev_version) => {
                            let current_time = rtc::timezone::local_time()
                                .format(network::telemetry::TIME_FORMAT)
                                .to_string();

                            let payload = network::telemetry::FirmwareUpdateLog {
                                current_time: &current_time,
                                message: "Firmware successfully updated",
                                previous_version: &prev_version.to_string(),
                                current_version: &current_version.to_string(),
                                notes: "No errors during update",
                            };

                            let topic = network::telemetry::topic::logs_firmware_update(
                                ctx.switchboard.device_id,
                            );

                            if network::telemetry::publish_json(mqtt, &topic, &payload).is_ok() {
                                let _ = ctx.nvs.remove("prev_version");
                            }
                        }
                        Err(e) => {
                            warn!(
                                "prev_version in NVS is not valid semver ({:?}), clearing: {:?}",
                                prev_str, e
                            );

                            let _ = ctx.nvs.remove("prev_version");
                        }
                    }
                }
            } else {
                error!("Boot validation failed, rolling back firmware");

                valid_ota.mark_running_slot_invalid_and_reboot();
            }
        } else {
            info!("Normal boot firmware already validated");
        }

        if ctx.switchboard.runtime.commands_enabled {
            if let Err(e) = transport::subscribe(mqtt, ctx.switchboard.device_id) {
                warn!("Failed to subscribe to command mailbox: {:?}", e);
            }
        }

        Ok(StateResult::Running(Box::new(Ota)))
    }
}

impl<A> State<A, NetworkContext, FSMCommand> for Ota
where
    A: Address,
{
    fn process(
        &mut self,
        ctx: &mut NetworkContext,
        _channel: &mut Mailbox<A, FSMCommand>,
    ) -> anyhow::Result<StateResult<A, NetworkContext, FSMCommand>> {
        if !ctx.switchboard.effects.allow_ota {
            info!("OTA disabled: skipping version compare");

            return Ok(StateResult::Running(Box::new(WifiConnectIfDisconnected)));
        }

        let mqtt = match ctx.mqtt.as_mut() {
            Some(mqtt) => mqtt,
            None => {
                warn!("OTA failed; MQTT is not initialized");

                return Ok(StateResult::Running(Box::new(WifiConnectIfDisconnected)));
            }
        };

        let current_version = match ctx.current_version.as_ref() {
            Some(version) => version,
            None => {
                error!("OTA failed; current version is not initialized");

                return Ok(StateResult::Running(Box::new(WifiConnectIfDisconnected)));
            }
        };

        let formatted_time = match ctx.formatted_time.as_ref() {
            Some(time) => time,
            None => {
                error!("OTA failed; formatted time is not initialized");

                return Ok(StateResult::Running(Box::new(WifiConnectIfDisconnected)));
            }
        };

        let payload = network::telemetry::Heartbeat {
            current_time: formatted_time,
            firmware_version: &current_version.to_string(),
        };

        let topic = network::telemetry::topic::status(ctx.switchboard.device_id);

        if let Err(e) = network::telemetry::publish_json(mqtt, &topic, &payload) {
            warn!("Failed to publish heartbeat: {:?}", e);
        }

        let mut updater = match OtaUpdater::new_ota(
            current_version.clone(),
            mqtt,
            ctx.switchboard.device_id,
            Some(ctx.switchboard.default_ota_updater),
            Some(ctx.switchboard.default_ota_password),
        ) {
            Ok(updater) => updater,
            Err(e) => {
                warn!("Failed to create OTA updater: {:?}", e);

                return Ok(StateResult::Running(Box::new(WifiConnectIfDisconnected)));
            }
        };

        info!("Checking for new OTA update in 3 seconds...");

        sleep(Duration::from_secs(3));

        if let Err(e) = updater.run_version_compare(&mut ctx.nvs) {
            error!("Version compare failed: {:?}", e);

            let current_time = rtc::timezone::local_time()
                .format(network::telemetry::TIME_FORMAT)
                .to_string();

            let version_str = current_version.to_string();

            let payload = network::telemetry::FirmwareUpdateLog {
                current_time: &current_time,
                message: "Firmware update unsuccessful",
                previous_version: &version_str,
                current_version: &version_str,
                notes: "Did not update due to failures",
            };

            let topic = network::telemetry::topic::logs_firmware_update(ctx.switchboard.device_id);

            if let Err(e) = network::telemetry::publish_json(mqtt, &topic, &payload) {
                warn!("Failed to publish OTA failure log: {:?}", e);
            }
        } else {
            info!("Version compare succeeded");
        }

        Ok(StateResult::Running(Box::new(WifiConnectIfDisconnected)))
    }
}

impl<A> State<A, NetworkContext, FSMCommand> for MqttPublishJson
where
    A: Address,
{
    fn process(
        &mut self,
        ctx: &mut NetworkContext,
        _channel: &mut Mailbox<A, FSMCommand>,
    ) -> anyhow::Result<StateResult<A, NetworkContext, FSMCommand>> {
        let mqtt = match ctx.mqtt.as_mut() {
            Some(mqtt) => mqtt,
            None => {
                warn!("JSON publish failed; MQTT is not initialized");

                return Ok(StateResult::Running(Box::new(WifiConnectIfDisconnected)));
            }
        };

        // TODO: Better error handling
        publish_json(mqtt, &self.0, &self.1)?;

        Ok(StateResult::Running(Box::new(WifiConnectIfDisconnected)))
    }
}

fn boot_diagnostic(
    device_id: &str,
    wifi: &mut Wifi,
    mqtt: &mut Mqtt,
    current_version: &Version,
) -> bool {
    info!("Starting boot validation in 5 seconds...");

    sleep(Duration::from_secs(5));

    match wifi.state() {
        WifiState::Connected(ip) => {
            info!("Wi-Fi connected with IP: {}", ip);
        }
        WifiState::Connecting => {
            warn!("Wi-Fi still connecting during validation...");
            return false;
        }
        WifiState::Disconnected => {
            error!("Wi-Fi disconnected, validation failed");
            return false;
        }
    }

    const MAX_RETRIES: u8 = 3;

    for attempt in 1..=MAX_RETRIES {
        info!("Boot diagnostic MQTT attempt {}/{}", attempt, MAX_RETRIES);

        let mut waited = 0;

        while !mqtt.is_connected() && waited < 12000 {
            sleep(Duration::from_millis(3000));
            waited += 3000;
        }

        if !mqtt.is_connected() {
            warn!("MQTT not connected yet, retrying...");
            continue;
        }

        let current_time = rtc::timezone::local_time()
            .format(network::telemetry::TIME_FORMAT)
            .to_string();

        let reset_reason = ResetReason::current();
        if reset_reason.is_unexpected() {
            warn!(
                "Reset reason: {} — {}",
                reset_reason.as_str(),
                reset_reason.boot_notes()
            );
        } else {
            info!("Reset reason: {}", reset_reason.as_str());
        }

        let payload = network::telemetry::BootLog {
            current_time: &current_time,
            message: reset_reason.boot_message(),
            firmware_version: &current_version.to_string(),
            component: network::telemetry::Component::System,
            reset_reason: reset_reason.as_str(),
            notes: reset_reason.boot_notes(),
        };

        let topic = network::telemetry::topic::logs_boot(device_id);

        if network::telemetry::publish_json(mqtt, &topic, &payload).is_ok() {
            return true;
        }

        error!("MQTT publish failed immediately");

        if attempt == MAX_RETRIES {
            error!("All MQTT boot diagnostic attempts failed...");
            return false;
        }

        sleep(Duration::from_millis(1000));
    }

    false
}
