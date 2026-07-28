//! Transport-agnostic diagnostics protocol: command parsing, the board and
//! environment traits a device implements, and a runtime that drives them.
//!
//! This crate has no hardware or platform dependencies, so the protocol can be
//! exercised on the host with `cargo test -p diagnostics`. Keep it that way —
//! the USB Serial/JTAG transport belongs in `serial_console`.

use core::fmt;
use std::string::String;
use std::vec::Vec;

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum DiagnosticCommand {
    Hdc1080Read,
    RtcCheck,
    MotorMove { argument: String },
    GoHome,
    OledTest { payload: String },
    RelayMotor { state: String },
    RelayHotspot { state: String },
    ConfigMode,
    SetEnv { key: String, value: String },
    InvalidSetEnv { original: String },
    SaveConfig,
    Reboot,
    WriteEnvFile,
    GetEnv { key: String },
    FirmwareVersion,
    GetCapabilities,
    GetConfig,
    Unknown { command: String, original: String },
}

impl DiagnosticCommand {
    pub fn parse(line: &str) -> Self {
        let trimmed = line.trim();
        let mut parts = trimmed.splitn(2, ' ');
        let command = parts.next().unwrap_or("").to_uppercase();
        let rest = parts.next().unwrap_or("").trim();

        match command.as_str() {
            "HDC1080_READ" => Self::Hdc1080Read,
            "RTC_CHECK" => Self::RtcCheck,
            "MOTOR_MOVE" => Self::MotorMove {
                argument: rest.to_string(),
            },
            "GO_HOME" => Self::GoHome,
            "OLED_TEST" => Self::OledTest {
                payload: rest.to_string(),
            },
            "RELAY_MOTOR" => Self::RelayMotor {
                state: rest.to_string(),
            },
            "RELAY_HOTSPOT" => Self::RelayHotspot {
                state: rest.to_string(),
            },
            "CONFIG_MODE" => Self::ConfigMode,
            "SET_ENV" => match parse_set_env(trimmed, rest) {
                Some((key, value)) => Self::SetEnv { key, value },
                None => Self::InvalidSetEnv {
                    original: trimmed.to_string(),
                },
            },
            "SAVE_CONFIG" => Self::SaveConfig,
            "REBOOT" => Self::Reboot,
            "WRITE_ENV_FILE" => Self::WriteEnvFile,
            "GET_ENV" => Self::GetEnv {
                key: rest.to_string(),
            },
            "FIRMWARE_VERSION" => Self::FirmwareVersion,
            "GET_CAPABILITIES" => Self::GetCapabilities,
            "GET_CONFIG" => Self::GetConfig,
            _ => Self::Unknown {
                command,
                original: trimmed.to_string(),
            },
        }
    }
}

fn parse_set_env(line: &str, rest: &str) -> Option<(String, String)> {
    if let Some((key, value)) = rest.split_once('=') {
        return Some((key.trim().to_string(), value.trim().to_string()));
    }

    if let Some((key, value)) = line.split_once('=') {
        return Some((
            key.trim().trim_start_matches("SET_ENV ").to_string(),
            value.trim().to_string(),
        ));
    }

    None
}

pub trait DiagnosticIo {
    fn write_line(&mut self, msg: &str) -> Result<(), ()>;
}

pub trait DiagnosticTransport: DiagnosticIo {
    fn read_nonblocking(&mut self, buf: &mut [u8]) -> usize;
}

pub trait DiagnosticHandler {
    type Error;

    fn handle_command<IO: DiagnosticIo>(
        &mut self,
        command: DiagnosticCommand,
        io: &mut IO,
    ) -> Result<DiagnosticControl, Self::Error>;
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DiagnosticControl {
    Continue,
    RebootRequested,
}

pub trait DiagnosticBoard {
    type Error;

    fn hdc1080_read<IO: DiagnosticIo>(&mut self, io: &mut IO) -> Result<(), Self::Error>;

    fn rtc_check<IO: DiagnosticIo>(&mut self, io: &mut IO) -> Result<(), Self::Error>;

    fn motor_move<IO: DiagnosticIo>(
        &mut self,
        argument: &str,
        io: &mut IO,
    ) -> Result<(), Self::Error>;

    fn go_home<IO: DiagnosticIo>(&mut self, io: &mut IO) -> Result<(), Self::Error>;

    fn oled_test<IO: DiagnosticIo>(
        &mut self,
        payload: &str,
        io: &mut IO,
    ) -> Result<(), Self::Error>;

    fn relay_motor<IO: DiagnosticIo>(
        &mut self,
        state: &str,
        io: &mut IO,
    ) -> Result<(), Self::Error>;

    fn relay_hotspot<IO: DiagnosticIo>(
        &mut self,
        state: &str,
        io: &mut IO,
    ) -> Result<(), Self::Error>;

    fn config_mode<IO: DiagnosticIo>(&mut self, _io: &mut IO) -> Result<(), Self::Error> {
        Ok(())
    }
}

pub trait DiagnosticEnvironment {
    type Error;

    fn set_env(&mut self, key: &str, value: &str) -> Result<(), Self::Error>;

    fn get_env(&self, key: &str) -> Result<Option<String>, Self::Error>;

    fn save_config(&mut self) -> Result<(), Self::Error>;

    fn save_config_error_message(&self) -> &str {
        "ERROR SAVE_CONFIG"
    }

    fn write_env_file(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    fn write_env_file_error_message(&self) -> &str {
        "ERROR WRITE_ENV_FILE"
    }
}

pub trait DiagnosticConfiguration {
    type Error;

    fn firmware_version(&self) -> &str;

    fn capabilities_csv(&self) -> String;

    fn write_config<IO: DiagnosticIo>(&self, io: &mut IO) -> Result<(), Self::Error>;

    fn write_config_error_message(&self) -> &str {
        "ERROR GET_CONFIG"
    }
}

pub trait Hdc1080Sensor {
    type Error: fmt::Debug;

    fn get_device_id(&mut self) -> Result<u16, Self::Error>;

    fn get_manufacturer_id(&mut self) -> Result<u16, Self::Error>;

    fn get_serial_id(&mut self) -> Result<[u16; 3], Self::Error>;

    fn read_temperature_humidity(&mut self) -> Result<(f32, f32), Self::Error>;
}

pub fn run_hdc1080_diagnostic<IO, Sensor, F>(
    io: &mut IO,
    sensor: Option<&mut Sensor>,
    mut simulate: F,
) where
    IO: DiagnosticIo,
    Sensor: Hdc1080Sensor,
    F: FnMut() -> (f32, f32),
{
    let (temp, hum) = simulate();
    let _ = io.write_line(&format!("temp:{:.2} hum:{:.2}", temp, hum));

    let Some(sensor) = sensor else {
        let (temp, hum) = simulate();
        let _ = io.write_line(&format!(
            "NO SENSOR ERROR. Simulating temp:{:.2} hum:{:.2}",
            temp, hum
        ));
        return;
    };

    match sensor.get_device_id() {
        Ok(0x1050) => {
            let _ = io.write_line("HDC1080 Correct device_id: 0x1050");
        }
        Ok(device_id) => {
            let (temp, hum) = simulate();
            let _ = io.write_line(&format!(
                "Unexpected device id 0x{:04X} temp:{:.2} hum:{:.2}",
                device_id, temp, hum
            ));
        }
        Err(err) => {
            let (temp, hum) = simulate();
            let _ = io.write_line(&format!(
                "HDC1080 get_device_id failed: {:?} temp:{:.2} hum:{:.2}",
                err, temp, hum
            ));
            return;
        }
    }

    match sensor.get_manufacturer_id() {
        Ok(0x5449) => {
            let _ = io.write_line("HDC1080 Correct manufacturer_id: 0x5449");
        }
        Ok(_) => {
            let (temp, hum) = simulate();
            let _ = io.write_line(&format!("temp:{:.2} hum:{:.2}", temp, hum));
        }
        Err(_) => {
            let (temp, hum) = simulate();
            let _ = io.write_line(&format!("temp:{:.2} hum:{:.2}", temp, hum));
            return;
        }
    }

    match sensor.get_serial_id() {
        Ok(serial_id) => {
            let _ = io.write_line(&format!(
                "HDC1080 serial_id: {:04X}-{:04X}-{:04X}",
                serial_id[0], serial_id[1], serial_id[2]
            ));
        }
        Err(_) => {
            let (temp, hum) = simulate();
            let _ = io.write_line(&format!("temp:{:.2} hum:{:.2}", temp, hum));
            return;
        }
    }

    match sensor.read_temperature_humidity() {
        Ok((temp, hum)) => {
            let _ = io.write_line(&format!("HDC1080 Correct read: temp:{:.2} hum:{:.2}", temp, hum));
        }
        Err(_) => {
            let (temp, hum) = simulate();
            let _ = io.write_line(&format!(
                "HDC1080 read failed. Simulating temp:{:.2} hum:{:.2}",
                temp, hum
            ));
        }
    }
}

#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct DiagnosticConfigSection {
    name: String,
    entries: Vec<(String, String)>,
}

impl DiagnosticConfigSection {
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            entries: Vec::new(),
        }
    }

    pub fn with_entry(mut self, key: impl Into<String>, value: impl Into<String>) -> Self {
        self.entries.push((key.into(), value.into()));
        self
    }
}

#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct StaticDiagnosticConfiguration {
    firmware_version: String,
    capabilities: Vec<String>,
    sections: Vec<DiagnosticConfigSection>,
}

impl StaticDiagnosticConfiguration {
    pub fn new(firmware_version: impl Into<String>) -> Self {
        Self {
            firmware_version: firmware_version.into(),
            capabilities: Vec::new(),
            sections: Vec::new(),
        }
    }

    pub fn with_capability(mut self, capability: impl Into<String>) -> Self {
        self.capabilities.push(capability.into());
        self
    }

    pub fn with_section(mut self, section: DiagnosticConfigSection) -> Self {
        self.sections.push(section);
        self
    }
}

impl DiagnosticConfiguration for StaticDiagnosticConfiguration {
    type Error = ();

    fn firmware_version(&self) -> &str {
        &self.firmware_version
    }

    fn capabilities_csv(&self) -> String {
        self.capabilities.join(",")
    }

    fn write_config<IO: DiagnosticIo>(&self, io: &mut IO) -> Result<(), Self::Error> {
        for section in &self.sections {
            let _ = io.write_line(&format!("[{}]", section.name));
            for (key, value) in &section.entries {
                let _ = io.write_line(&format!("{}={}", key, value));
            }
        }
        Ok(())
    }
}

#[derive(Debug)]
pub enum StandardDiagnosticError<BoardError, EnvError, ConfigError> {
    Board(BoardError),
    Environment(EnvError),
    Config(ConfigError),
}

pub struct StandardDiagnosticHandler<Board, Environment, Configuration> {
    board: Board,
    environment: Environment,
    configuration: Configuration,
}

impl<Board, Environment, Configuration> StandardDiagnosticHandler<Board, Environment, Configuration> {
    pub fn new(board: Board, environment: Environment, configuration: Configuration) -> Self {
        Self {
            board,
            environment,
            configuration,
        }
    }

    pub fn board(&self) -> &Board {
        &self.board
    }

    pub fn board_mut(&mut self) -> &mut Board {
        &mut self.board
    }

    pub fn environment(&self) -> &Environment {
        &self.environment
    }

    pub fn environment_mut(&mut self) -> &mut Environment {
        &mut self.environment
    }

    pub fn configuration(&self) -> &Configuration {
        &self.configuration
    }
}

impl<Board, Environment, Configuration> DiagnosticHandler
    for StandardDiagnosticHandler<Board, Environment, Configuration>
where
    Board: DiagnosticBoard,
    Environment: DiagnosticEnvironment,
    Configuration: DiagnosticConfiguration,
{
    type Error = StandardDiagnosticError<
        Board::Error,
        Environment::Error,
        Configuration::Error,
    >;

    fn handle_command<IO: DiagnosticIo>(
        &mut self,
        command: DiagnosticCommand,
        io: &mut IO,
    ) -> Result<DiagnosticControl, Self::Error> {
        match command {
            DiagnosticCommand::Hdc1080Read => self
                .board
                .hdc1080_read(io)
                .map_err(StandardDiagnosticError::Board)?,
            DiagnosticCommand::RtcCheck => self
                .board
                .rtc_check(io)
                .map_err(StandardDiagnosticError::Board)?,
            DiagnosticCommand::MotorMove { argument } => self
                .board
                .motor_move(&argument, io)
                .map_err(StandardDiagnosticError::Board)?,
            DiagnosticCommand::GoHome => self
                .board
                .go_home(io)
                .map_err(StandardDiagnosticError::Board)?,
            DiagnosticCommand::OledTest { payload } => self
                .board
                .oled_test(&payload, io)
                .map_err(StandardDiagnosticError::Board)?,
            DiagnosticCommand::RelayMotor { state } => self
                .board
                .relay_motor(&state, io)
                .map_err(StandardDiagnosticError::Board)?,
            DiagnosticCommand::RelayHotspot { state } => self
                .board
                .relay_hotspot(&state, io)
                .map_err(StandardDiagnosticError::Board)?,
            DiagnosticCommand::ConfigMode => self
                .board
                .config_mode(io)
                .map_err(StandardDiagnosticError::Board)?,
            DiagnosticCommand::SetEnv { key, value } => {
                self.environment
                    .set_env(&key, &value)
                    .map_err(StandardDiagnosticError::Environment)?;
                let _ = io.write_line("OK");
            }
            DiagnosticCommand::InvalidSetEnv { .. } => {
                let _ = io.write_line("ERROR Bad SET_ENV format");
            }
            DiagnosticCommand::SaveConfig => {
                if let Err(err) = self.environment.save_config() {
                    let _ = io.write_line(self.environment.save_config_error_message());
                    return Err(StandardDiagnosticError::Environment(err));
                }
                let _ = io.write_line("OK");
            }
            DiagnosticCommand::Reboot => {
                let _ = io.write_line("OK Rebooting");
                return Ok(DiagnosticControl::RebootRequested);
            }
            DiagnosticCommand::WriteEnvFile => {
                if let Err(err) = self.environment.write_env_file() {
                    let _ = io.write_line(self.environment.write_env_file_error_message());
                    return Err(StandardDiagnosticError::Environment(err));
                }
                let _ = io.write_line("OK");
            }
            DiagnosticCommand::GetEnv { key } => {
                let value = self
                    .environment
                    .get_env(&key)
                    .map_err(StandardDiagnosticError::Environment)?;
                match value {
                    Some(value) => {
                        let _ = io.write_line(&format!("{}={}", key, value));
                    }
                    None => {
                        let _ = io.write_line(&format!("{}=", key));
                    }
                }
            }
            DiagnosticCommand::FirmwareVersion => {
                let _ = io.write_line(&format!(
                    "VERSION {}",
                    self.configuration.firmware_version()
                ));
            }
            DiagnosticCommand::GetCapabilities => {
                let _ = io.write_line(&self.configuration.capabilities_csv());
            }
            DiagnosticCommand::GetConfig => {
                if let Err(err) = self.configuration.write_config(io) {
                    let _ = io.write_line(self.configuration.write_config_error_message());
                    return Err(StandardDiagnosticError::Config(err));
                }
            }
            DiagnosticCommand::Unknown { command, .. } => {
                let _ = io.write_line(&format!("ERROR Unknown command: {}", command));
            }
        }

        Ok(DiagnosticControl::Continue)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DiagnosticPoll {
    Idle,
    CommandProcessed,
    RebootRequested,
}

pub struct DiagnosticRuntime {
    buffer: Vec<u8>,
    ack_line: &'static str,
}

impl Default for DiagnosticRuntime {
    fn default() -> Self {
        Self::new()
    }
}

impl DiagnosticRuntime {
    pub fn new() -> Self {
        Self {
            buffer: Vec::new(),
            ack_line: "CMD_RECEIVED",
        }
    }

    pub fn poll<T, H>(
        &mut self,
        transport: &mut T,
        handler: &mut H,
    ) -> Result<DiagnosticPoll, H::Error>
    where
        T: DiagnosticTransport,
        H: DiagnosticHandler,
    {
        let mut read_buf = [0u8; 128];
        let bytes_read = transport.read_nonblocking(&mut read_buf);
        if bytes_read == 0 {
            return Ok(DiagnosticPoll::Idle);
        }

        self.buffer.extend_from_slice(&read_buf[..bytes_read]);
        let mut processed_any = false;

        while let Some(pos) = self
            .buffer
            .iter()
            .position(|byte| *byte == b'\n' || *byte == b'\r')
        {
            let line = self.buffer.drain(..=pos).collect::<Vec<u8>>();
            let message = String::from_utf8_lossy(&line).trim().to_string();
            if message.is_empty() {
                continue;
            }

            let _ = transport.write_line(self.ack_line);
            let control = handler.handle_command(DiagnosticCommand::parse(&message), transport)?;
            processed_any = true;

            if control == DiagnosticControl::RebootRequested {
                return Ok(DiagnosticPoll::RebootRequested);
            }
        }

        if processed_any {
            Ok(DiagnosticPoll::CommandProcessed)
        } else {
            Ok(DiagnosticPoll::Idle)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::HashMap;
    use std::collections::VecDeque;

    #[derive(Default)]
    struct MockIo {
        writes: Vec<String>,
    }

    impl DiagnosticIo for MockIo {
        fn write_line(&mut self, msg: &str) -> Result<(), ()> {
            self.writes.push(msg.to_string());
            Ok(())
        }
    }

    #[derive(Default)]
    struct MockTransport {
        reads: VecDeque<u8>,
        writes: Vec<String>,
    }

    impl MockTransport {
        fn from_input(input: &str) -> Self {
            Self {
                reads: input.as_bytes().iter().copied().collect(),
                writes: Vec::new(),
            }
        }
    }

    impl DiagnosticIo for MockTransport {
        fn write_line(&mut self, msg: &str) -> Result<(), ()> {
            self.writes.push(msg.to_string());
            Ok(())
        }
    }

    impl DiagnosticTransport for MockTransport {
        fn read_nonblocking(&mut self, buf: &mut [u8]) -> usize {
            let mut count = 0;
            while count < buf.len() {
                match self.reads.pop_front() {
                    Some(byte) => {
                        buf[count] = byte;
                        count += 1;
                    }
                    None => break,
                }
            }
            count
        }
    }

    #[derive(Default)]
    struct MockBoard;

    impl DiagnosticBoard for MockBoard {
        type Error = &'static str;

        fn hdc1080_read<IO: DiagnosticIo>(&mut self, io: &mut IO) -> Result<(), Self::Error> {
            let _ = io.write_line("HDC1080_OK");
            Ok(())
        }

        fn rtc_check<IO: DiagnosticIo>(&mut self, io: &mut IO) -> Result<(), Self::Error> {
            let _ = io.write_line("TIME: 2026-01-01 00:00:00");
            Ok(())
        }

        fn motor_move<IO: DiagnosticIo>(
            &mut self,
            argument: &str,
            io: &mut IO,
        ) -> Result<(), Self::Error> {
            let _ = io.write_line(&format!("MOTOR {}", argument));
            let _ = io.write_line("OK");
            Ok(())
        }

        fn go_home<IO: DiagnosticIo>(&mut self, io: &mut IO) -> Result<(), Self::Error> {
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
            let _ = io.write_line(&format!("RELAY_MOTOR {}", state));
            let _ = io.write_line("OK");
            Ok(())
        }

        fn relay_hotspot<IO: DiagnosticIo>(
            &mut self,
            state: &str,
            io: &mut IO,
        ) -> Result<(), Self::Error> {
            let _ = io.write_line(&format!("RELAY_HOTSPOT {}", state));
            let _ = io.write_line("OK");
            Ok(())
        }
    }

    #[derive(Default)]
    struct MockHdcSensor {
        fail_device_id: bool,
        fail_manufacturer_id: bool,
        fail_serial_id: bool,
        fail_read: bool,
    }

    impl Hdc1080Sensor for MockHdcSensor {
        type Error = &'static str;

        fn get_device_id(&mut self) -> Result<u16, Self::Error> {
            if self.fail_device_id {
                Err("device_id_failed")
            } else {
                Ok(0x1050)
            }
        }

        fn get_manufacturer_id(&mut self) -> Result<u16, Self::Error> {
            if self.fail_manufacturer_id {
                Err("manufacturer_failed")
            } else {
                Ok(0x5449)
            }
        }

        fn get_serial_id(&mut self) -> Result<[u16; 3], Self::Error> {
            if self.fail_serial_id {
                Err("serial_failed")
            } else {
                Ok([0x1111, 0x2222, 0x3333])
            }
        }

        fn read_temperature_humidity(&mut self) -> Result<(f32, f32), Self::Error> {
            if self.fail_read {
                Err("read_failed")
            } else {
                Ok((24.5, 55.5))
            }
        }
    }

    struct MockEnvironment {
        env: HashMap<String, String>,
        fail_save: bool,
        fail_write_env_file: bool,
    }

    impl Default for MockEnvironment {
        fn default() -> Self {
            Self {
                env: HashMap::new(),
                fail_save: false,
                fail_write_env_file: false,
            }
        }
    }

    impl DiagnosticEnvironment for MockEnvironment {
        type Error = &'static str;

        fn set_env(&mut self, key: &str, value: &str) -> Result<(), Self::Error> {
            self.env.insert(key.to_string(), value.to_string());
            Ok(())
        }

        fn get_env(&self, key: &str) -> Result<Option<String>, Self::Error> {
            Ok(self.env.get(key).cloned())
        }

        fn save_config(&mut self) -> Result<(), Self::Error> {
            if self.fail_save {
                Err("save_failed")
            } else {
                Ok(())
            }
        }

        fn save_config_error_message(&self) -> &str {
            "ERROR NVS_LOCK"
        }

        fn write_env_file(&mut self) -> Result<(), Self::Error> {
            if self.fail_write_env_file {
                Err("write_env_failed")
            } else {
                Ok(())
            }
        }
    }

    struct MockConfiguration {
        version: &'static str,
        capabilities: &'static str,
        config_lines: Vec<&'static str>,
        fail_write_config: bool,
    }

    impl Default for MockConfiguration {
        fn default() -> Self {
            Self {
                version: "1.2.3",
                capabilities: "MQTT,WIFI,SENSOR",
                config_lines: vec![
                    "[device]",
                    "tower_id=TOWER_TEST_001",
                    "[wifi]",
                    "ssid=TestNetwork",
                ],
                fail_write_config: false,
            }
        }
    }

    impl DiagnosticConfiguration for MockConfiguration {
        type Error = &'static str;

        fn firmware_version(&self) -> &str {
            self.version
        }

        fn capabilities_csv(&self) -> String {
            self.capabilities.to_string()
        }

        fn write_config<IO: DiagnosticIo>(&self, io: &mut IO) -> Result<(), Self::Error> {
            if self.fail_write_config {
                return Err("write_config_failed");
            }

            for line in &self.config_lines {
                let _ = io.write_line(line);
            }

            Ok(())
        }
    }

    #[test]
    fn parses_set_env_command_variants() {
        assert_eq!(
            DiagnosticCommand::parse("SET_ENV tower_id=TOWER_42"),
            DiagnosticCommand::SetEnv {
                key: "tower_id".to_string(),
                value: "TOWER_42".to_string(),
            }
        );

        assert_eq!(
            DiagnosticCommand::parse("set_env wifi.password = secret "),
            DiagnosticCommand::SetEnv {
                key: "wifi.password".to_string(),
                value: "secret".to_string(),
            }
        );

        assert_eq!(
            DiagnosticCommand::parse("SET_ENV malformed"),
            DiagnosticCommand::InvalidSetEnv {
                original: "SET_ENV malformed".to_string(),
            }
        );
    }

    #[test]
    fn parses_unknown_command() {
        assert_eq!(
            DiagnosticCommand::parse("PING 123"),
            DiagnosticCommand::Unknown {
                command: "PING".to_string(),
                original: "PING 123".to_string(),
            }
        );
    }

    #[test]
    fn standard_handler_emits_expected_protocol_for_env_version_caps_and_unknown() {
        let mut runtime = DiagnosticRuntime::new();
        let mut transport = MockTransport::from_input(
            "SET_ENV tower_id=TOWER_42\r\nGET_ENV tower_id\r\nFIRMWARE_VERSION\r\nGET_CAPABILITIES\r\nUNKNOWN_CMD\r\n",
        );
        let mut handler = StandardDiagnosticHandler::new(
            MockBoard,
            MockEnvironment::default(),
            MockConfiguration::default(),
        );

        let poll = runtime.poll(&mut transport, &mut handler).unwrap();

        assert_eq!(poll, DiagnosticPoll::CommandProcessed);
        assert_eq!(
            transport.writes,
            vec![
                "CMD_RECEIVED",
                "OK",
                "CMD_RECEIVED",
                "tower_id=TOWER_42",
                "CMD_RECEIVED",
                "VERSION 1.2.3",
                "CMD_RECEIVED",
                "MQTT,WIFI,SENSOR",
                "CMD_RECEIVED",
                "ERROR Unknown command: UNKNOWN_CMD",
            ]
        );
    }

    #[test]
    fn standard_handler_streams_config_lines() {
        let mut runtime = DiagnosticRuntime::new();
        let mut transport = MockTransport::from_input("GET_CONFIG\r\n");
        let mut handler = StandardDiagnosticHandler::new(
            MockBoard,
            MockEnvironment::default(),
            MockConfiguration::default(),
        );

        let poll = runtime.poll(&mut transport, &mut handler).unwrap();

        assert_eq!(poll, DiagnosticPoll::CommandProcessed);
        assert_eq!(
            transport.writes,
            vec![
                "CMD_RECEIVED",
                "[device]",
                "tower_id=TOWER_TEST_001",
                "[wifi]",
                "ssid=TestNetwork",
            ]
        );
    }

    #[test]
    fn standard_handler_reports_save_config_failures_with_protocol_message() {
        let mut runtime = DiagnosticRuntime::new();
        let mut transport = MockTransport::from_input("SAVE_CONFIG\r\n");
        let mut handler = StandardDiagnosticHandler::new(
            MockBoard,
            MockEnvironment {
                fail_save: true,
                ..MockEnvironment::default()
            },
            MockConfiguration::default(),
        );

        let result = runtime.poll(&mut transport, &mut handler);

        assert!(matches!(
            result,
            Err(StandardDiagnosticError::Environment("save_failed"))
        ));
        assert_eq!(transport.writes, vec!["CMD_RECEIVED", "ERROR NVS_LOCK"]);
    }

    #[test]
    fn runtime_returns_reboot_requested_and_preserves_reboot_message() {
        let mut runtime = DiagnosticRuntime::new();
        let mut transport = MockTransport::from_input("REBOOT\r\n");
        let mut handler = StandardDiagnosticHandler::new(
            MockBoard,
            MockEnvironment::default(),
            MockConfiguration::default(),
        );

        let poll = runtime.poll(&mut transport, &mut handler).unwrap();

        assert_eq!(poll, DiagnosticPoll::RebootRequested);
        assert_eq!(transport.writes, vec!["CMD_RECEIVED", "OK Rebooting"]);
    }

    #[test]
    fn board_commands_can_be_tested_through_standard_handler() {
        let mut io = MockIo::default();
        let mut handler = StandardDiagnosticHandler::new(
            MockBoard,
            MockEnvironment::default(),
            MockConfiguration::default(),
        );

        let result = handler.handle_command(
            DiagnosticCommand::MotorMove {
                argument: "CW".to_string(),
            },
            &mut io,
        );

        assert_eq!(result.unwrap(), DiagnosticControl::Continue);
        assert_eq!(io.writes, vec!["MOTOR CW", "OK"]);
    }

    #[test]
    fn hdc1080_helper_emits_full_success_sequence() {
        let mut io = MockIo::default();
        let mut sensor = MockHdcSensor::default();
        let mut samples = VecDeque::from([(21.0_f32, 41.0_f32)]);

        run_hdc1080_diagnostic(&mut io, Some(&mut sensor), || {
            samples.pop_front().unwrap_or((25.0, 50.0))
        });

        assert_eq!(
            io.writes,
            vec![
                "temp:21.00 hum:41.00",
                "HDC1080 Correct device_id: 0x1050",
                "HDC1080 Correct manufacturer_id: 0x5449",
                "HDC1080 serial_id: 1111-2222-3333",
                "HDC1080 Correct read: temp:24.50 hum:55.50",
            ]
        );
    }

    #[test]
    fn static_configuration_builder_writes_expected_sections() {
        let config = StaticDiagnosticConfiguration::new("9.9.9")
            .with_capability("MQTT")
            .with_capability("SENSOR")
            .with_section(
                DiagnosticConfigSection::new("device")
                    .with_entry("tower_id", "TOWER_007"),
            )
            .with_section(
                DiagnosticConfigSection::new("wifi")
                    .with_entry("ssid", "TestNet")
                    .with_entry("password", "secret"),
            );
        let mut io = MockIo::default();

        assert_eq!(config.firmware_version(), "9.9.9");
        assert_eq!(config.capabilities_csv(), "MQTT,SENSOR");
        config.write_config(&mut io).unwrap();

        assert_eq!(
            io.writes,
            vec![
                "[device]",
                "tower_id=TOWER_007",
                "[wifi]",
                "ssid=TestNet",
                "password=secret",
            ]
        );
    }
}
