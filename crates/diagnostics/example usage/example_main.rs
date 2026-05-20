use anyhow::{anyhow, Result};
use buttons::Buttons;
use chrono::Local;
use diagnostics::{
    run_hdc1080_diagnostic, DiagnosticBoard, DiagnosticConfigSection, DiagnosticEnvironment,
    DiagnosticIo, DiagnosticPoll, DiagnosticRuntime, Hdc1080Sensor,
    StandardDiagnosticHandler, StaticDiagnosticConfiguration,
};
use esp_idf_hal::gpio::*;
use esp_idf_hal::i2c::{I2cConfig, I2cDriver};
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::prelude::*;
use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::hal::delay::Ets;
use esp_idf_svc::nvs::{EspDefaultNvsPartition, EspNvs, NvsDefault};
use esp_idf_sys as sys;
use rand::Rng;
use serial_console::UsbSerialJtagConsole;
use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

mod config;

type SharedEnv = Arc<Mutex<HashMap<String, String>>>;
type SharedNvs = Arc<Mutex<EspNvs<NvsDefault>>>;

#[derive(Debug)]
enum FirmwareEnvironmentError {
    NvsLock,
}

struct FirmwareBoard<'a, STEP, DIR, RELAY>
where
    STEP: Pin,
    DIR: Pin,
    RELAY: Pin,
{
    step_pin: PinDriver<'a, STEP, Output>,
    dir_pin: PinDriver<'a, DIR, Output>,
    relay_motor: PinDriver<'a, RELAY, Output>,
    hdc: Option<FirmwareHdcSensor<'a>>,
}

struct FirmwareHdcSensor<'a>(hdc1080::Hdc1080<I2cDriver<'a>, Ets>);

impl<'a> Hdc1080Sensor for FirmwareHdcSensor<'a> {
    type Error = esp_idf_hal::i2c::I2cError;

    fn get_device_id(&mut self) -> Result<u16, Self::Error> {
        self.0.get_device_id()
    }

    fn get_manufacturer_id(&mut self) -> Result<u16, Self::Error> {
        self.0.get_man_id()
    }

    fn get_serial_id(&mut self) -> Result<[u16; 3], Self::Error> {
        self.0.get_serial_id()
    }

    fn read_temperature_humidity(&mut self) -> Result<(f32, f32), Self::Error> {
        self.0.read()
    }
}

impl<'a, STEP, DIR, RELAY> FirmwareBoard<'a, STEP, DIR, RELAY>
where
    STEP: Pin,
    DIR: Pin,
    RELAY: Pin,
{
    fn new(
        step_pin: PinDriver<'a, STEP, Output>,
        dir_pin: PinDriver<'a, DIR, Output>,
        relay_motor: PinDriver<'a, RELAY, Output>,
        hdc: Option<FirmwareHdcSensor<'a>>,
    ) -> Self {
        Self {
            step_pin,
            dir_pin,
            relay_motor,
            hdc,
        }
    }
}

impl<'a, STEP, DIR, RELAY> DiagnosticBoard for FirmwareBoard<'a, STEP, DIR, RELAY>
where
    STEP: Pin,
    DIR: Pin,
    RELAY: Pin,
{
    type Error = ();

    fn hdc1080_read<IO: DiagnosticIo>(&mut self, io: &mut IO) -> Result<(), Self::Error> {
        let mut rng = rand::thread_rng();
        run_hdc1080_diagnostic(io, self.hdc.as_mut(), || {
            (
                rng.gen_range(20.0..30.0),
                rng.gen_range(40.0..60.0),
            )
        });
        Ok(())
    }

    fn rtc_check<IO: DiagnosticIo>(&mut self, io: &mut IO) -> Result<(), Self::Error> {
        let now = Local::now().format("%Y-%m-%d %H:%M:%S").to_string();
        let _ = io.write_line(&format!("TIME: {}", now));
        Ok(())
    }

    fn motor_move<IO: DiagnosticIo>(
        &mut self,
        _argument: &str,
        io: &mut IO,
    ) -> Result<(), Self::Error> {
        let _ = self.dir_pin.set_low();
        for _ in 0..200 {
            let _ = self.step_pin.set_high();
            thread::sleep(Duration::from_millis(2));
            let _ = self.step_pin.set_low();
            thread::sleep(Duration::from_millis(2));
        }
        let _ = io.write_line("MOVING");
        thread::sleep(Duration::from_millis(800));
        let _ = io.write_line("OK");
        Ok(())
    }

    fn go_home<IO: DiagnosticIo>(&mut self, io: &mut IO) -> Result<(), Self::Error> {
        for _ in 0..300 {
            let _ = self.step_pin.set_high();
            thread::sleep(Duration::from_millis(2));
            let _ = self.step_pin.set_low();
            thread::sleep(Duration::from_millis(2));
        }
        let _ = io.write_line("MOVING_HOME");
        thread::sleep(Duration::from_millis(900));
        let _ = io.write_line("LIMIT");
        Ok(())
    }

    fn oled_test<IO: DiagnosticIo>(
        &mut self,
        payload: &str,
        io: &mut IO,
    ) -> Result<(), Self::Error> {
        let _ = io.write_line(&format!("OLED: {}", payload));
        let _ = io.write_line("OK");
        Ok(())
    }

    fn relay_motor<IO: DiagnosticIo>(
        &mut self,
        state: &str,
        io: &mut IO,
    ) -> Result<(), Self::Error> {
        match state.trim().to_uppercase().as_str() {
            "ON" => {
                let _ = self.relay_motor.set_high();
            }
            "OFF" => {
                let _ = self.relay_motor.set_low();
            }
            _ => {}
        }
        let _ = io.write_line("OK");
        Ok(())
    }

    fn relay_hotspot<IO: DiagnosticIo>(
        &mut self,
        state: &str,
        io: &mut IO,
    ) -> Result<(), Self::Error> {
        match state.trim().to_uppercase().as_str() {
            "ON" => {
                let _ = self.relay_motor.set_high();
            }
            "OFF" => {
                let _ = self.relay_motor.set_low();
            }
            _ => {}
        }
        let _ = io.write_line("OK");
        Ok(())
    }
}

struct FirmwareEnvironment {
    env: SharedEnv,
    nvs: SharedNvs,
}

impl FirmwareEnvironment {
    fn new(env: SharedEnv, nvs: SharedNvs) -> Self {
        Self { env, nvs }
    }
}

impl DiagnosticEnvironment for FirmwareEnvironment {
    type Error = FirmwareEnvironmentError;

    fn set_env(&mut self, key: &str, value: &str) -> Result<(), Self::Error> {
        let mut env = self.env.lock().unwrap();
        env.insert(key.to_string(), value.to_string());
        Ok(())
    }

    fn get_env(&self, key: &str) -> Result<Option<String>, Self::Error> {
        let env = self.env.lock().unwrap();
        Ok(env.get(key).cloned())
    }

    fn save_config(&mut self) -> Result<(), Self::Error> {
        let env = self.env.lock().unwrap();
        let mut nvs = self.nvs.lock().map_err(|_| FirmwareEnvironmentError::NvsLock)?;
        for (key, value) in env.iter() {
            let _ = nvs.set_str(key.as_str(), value.as_str());
        }
        Ok(())
    }

    fn save_config_error_message(&self) -> &str {
        "ERROR NVS_LOCK"
    }
}

#[export_name = "__pender"]
pub extern "C" fn __pender(_context: *mut ()) {}

fn main() -> Result<()> {
    esp_idf_svc::sys::link_patches();
    UsbSerialJtagConsole::install_driver(1024, 1024)
        .map_err(|e| anyhow!("usb_serial_jtag_driver_install failed: {}", e))?;

    let _sysloop = EspSystemEventLoop::take()
        .map_err(|e| anyhow!("EspSystemEventLoop::take failed: {:?}", e))?;

    let peripherals = Peripherals::take()
        .map_err(|e| anyhow!("Peripherals::take failed: {:?}", e))?;

    let mut rgb_led = PinDriver::output(peripherals.pins.gpio7)
        .map_err(|e| anyhow!("gpio7 output init failed: {:?}", e))?;
    let step_pin = PinDriver::output(peripherals.pins.gpio15)
        .map_err(|e| anyhow!("gpio15 output init failed: {:?}", e))?;
    let dir_pin = PinDriver::output(peripherals.pins.gpio16)
        .map_err(|e| anyhow!("gpio16 output init failed: {:?}", e))?;
    let relay_motor = PinDriver::output(peripherals.pins.gpio17)
        .map_err(|e| anyhow!("gpio17 output init failed: {:?}", e))?;

    let i2c_config = I2cConfig::new().baudrate(100u32.kHz().into());
    let i2c = I2cDriver::new(
        peripherals.i2c0,
        peripherals.pins.gpio8,
        peripherals.pins.gpio9,
        &i2c_config,
    )
    .map_err(|e| anyhow!("I2C init failed: {:?}", e))?;

    let env = Arc::new(Mutex::new(HashMap::<String, String>::new()));
    let nvs_partition = EspDefaultNvsPartition::take()
        .map_err(|e| anyhow!("EspDefaultNvsPartition::take failed: {:?}", e))?;
    let nvs = EspNvs::new(nvs_partition, "storage", true)
        .map_err(|e| anyhow!("EspNvs init failed: {:?}", e))?;
    let nvs = Arc::new(Mutex::new(nvs));
    let cfg = Arc::new(load_config_fallback());

    {
        let env = Arc::clone(&env);
        let nvs = Arc::clone(&nvs);
        let cfg = Arc::clone(&cfg);

        thread::spawn(move || {
            let board = FirmwareBoard::new(
                step_pin,
                dir_pin,
                relay_motor,
                hdc1080::Hdc1080::new(i2c, Ets).ok().map(FirmwareHdcSensor),
            );
            let environment = FirmwareEnvironment::new(env, nvs);
            let configuration = build_diagnostic_configuration(&cfg);
            let mut handler = StandardDiagnosticHandler::new(board, environment, configuration);
            let mut runtime = DiagnosticRuntime::new();
            let mut console = UsbSerialJtagConsole::new();

            loop {
                match runtime.poll(&mut console, &mut handler) {
                    Ok(DiagnosticPoll::Idle) => thread::sleep(Duration::from_millis(50)),
                    Ok(DiagnosticPoll::CommandProcessed) => {}
                    Ok(DiagnosticPoll::RebootRequested) => {
                        thread::sleep(Duration::from_millis(100));
                        unsafe {
                            sys::esp_restart();
                        }
                    }
                    Err(_) => thread::sleep(Duration::from_millis(50)),
                }
            }
        });
    }

    {
        let mut btns = Buttons::new(
            peripherals.pins.gpio5,
            peripherals.pins.gpio4,
            peripherals.pins.gpio6,
        );

        thread::spawn(move || {
            let mut last = (false, false, false);
            let mut last_raw = (true, true, true);
            let mut raw_log_counter: u32 = 0;

            loop {
                btns.tick();
                let east = btns.is_east_pressed();
                let maintenance = btns.is_maintenance_pressed();
                let west = btns.is_west_pressed();

                if (east, maintenance, west) != last {
                    last = (east, maintenance, west);
                }

                let mut handled_click = false;
                if east {
                    handled_click |= send_console_event("EAST");
                }
                if maintenance {
                    handled_click |= send_console_event("MAINTENANCE");
                }
                if west {
                    handled_click |= send_console_event("WEST");
                }

                if handled_click {
                    btns.reset_all();
                }

                raw_log_counter = raw_log_counter.wrapping_add(1);
                if raw_log_counter >= 10 {
                    raw_log_counter = 0;
                    let raw = btns.raw_pin_levels();
                    if raw != last_raw {
                        if raw.0 {
                            let _ = send_console_event("EAST");
                        }
                        if raw.1 {
                            let _ = send_console_event("MAINTENANCE");
                        }
                        if raw.2 {
                            let _ = send_console_event("WEST");
                        }
                        last_raw = raw;
                    }
                }

                thread::sleep(Duration::from_millis(20));
            }
        });
    }

    loop {
        rgb_led
            .set_high()
            .map_err(|e| anyhow!("rgb_led set_high failed: {:?}", e))?;
        thread::sleep(Duration::from_millis(300));
        rgb_led
            .set_low()
            .map_err(|e| anyhow!("rgb_led set_low failed: {:?}", e))?;
        thread::sleep(Duration::from_millis(1200));
    }
}

fn load_config_fallback() -> config::Config {
    match config::Config::load() {
        Ok(config) => config,
        Err(_) => match config::Config::load() {
            Ok(config) => config,
            Err(_) => config::Config {
                device: config::DeviceConfig {
                    tower_id: String::from("TOWER_UNKNOWN"),
                },
                wifi: config::WifiConfig {
                    ssid: String::new(),
                    password: String::new(),
                },
                mqtt: config::MqttConfig {
                    broker: String::new(),
                    port: 1883,
                    username: String::new(),
                    password: String::new(),
                    topic: String::new(),
                },
                customer: config::CustomerConfig {
                    full_name: String::new(),
                    address: String::new(),
                    phone: String::new(),
                },
                location: config::LocationConfig {
                    latitude: 0.0,
                    longitude: 0.0,
                    altitude: 0.0,
                    timezone_offset_hours: 0,
                },
            },
        },
    }
}

fn build_diagnostic_configuration(cfg: &config::Config) -> StaticDiagnosticConfiguration {
    StaticDiagnosticConfiguration::new("1.0.0")
        .with_capability("MQTT")
        .with_capability("WIFI")
        .with_capability("SENSOR")
        .with_section(
            DiagnosticConfigSection::new("device")
                .with_entry("tower_id", cfg.get_tower_id()),
        )
        .with_section(
            DiagnosticConfigSection::new("wifi")
                .with_entry("ssid", cfg.get_wifi_ssid())
                .with_entry("password", cfg.get_wifi_password()),
        )
        .with_section(
            DiagnosticConfigSection::new("location")
                .with_entry("latitude", cfg.get_latitude().to_string())
                .with_entry("longitude", cfg.get_longitude().to_string())
                .with_entry("altitude", cfg.get_altitude().to_string())
                .with_entry(
                    "timezone_offset_hours",
                    cfg.get_timezone_offset().to_string(),
                ),
        )
        .with_section(
            DiagnosticConfigSection::new("mqtt")
                .with_entry("broker", cfg.get_mqtt_broker())
                .with_entry("port", cfg.get_mqtt_port().to_string())
                .with_entry("username", cfg.get_mqtt_username())
                .with_entry("password", cfg.get_mqtt_password())
                .with_entry("topic", cfg.get_mqtt_topic()),
        )
        .with_section(
            DiagnosticConfigSection::new("customer")
                .with_entry("full_name", cfg.get_customer_full_name())
                .with_entry("address", cfg.get_customer_address())
                .with_entry("phone", cfg.get_customer_phone()),
        )
}

fn send_console_event(msg: &str) -> bool {
    let mut console = UsbSerialJtagConsole::new();
    console.write_line(msg).is_ok()
}
