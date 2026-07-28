use board_diagnostics::{
    DiagnosticBoard, DiagnosticCommand, DiagnosticControl, DiagnosticEnvironment,
    DiagnosticHandler, DiagnosticIo, StandardDiagnosticError, StandardDiagnosticHandler,
    StaticDiagnosticConfiguration,
};
use chrono::Local;
use esp_idf_svc::{
    hal::{delay::Ets, i2c::I2cDriver},
    nvs::{EspNvs, NvsPartitionId},
};
use hdc1080::Hdc1080;
use motion::{Motion, MotionMode};

use crate::{app::encoder_fault::Direction, infra};

pub const FIRST_WAVE_CAPABILITIES: &[&str] = &[
    "MQTT",
    "SERIAL",
    "STATUS",
    "RTC",
    "HDC1080",
    "GO_HOME",
];

#[derive(Clone, Debug, Default)]
pub struct DiagnosticsTranscript {
    pub status: &'static str,
    pub message: String,
    pub lines: Vec<String>,
}

#[derive(Debug)]
enum RuntimeBoardError {
    Hdc1080(String),
    GoHome(String),
    Unsupported(&'static str),
}

#[derive(Debug)]
enum RuntimeEnvironmentError {
    Unsupported(&'static str),
}

#[derive(Default)]
struct TranscriptIo {
    lines: Vec<String>,
}

struct RuntimeDiagnosticBoard<'ctx, 'motion, T>
where
    T: NvsPartitionId,
{
    motion: &'ctx mut Motion<'motion>,
    nvs: &'ctx mut EspNvs<T>,
    bus: &'static shared_bus::BusManagerStd<I2cDriver<'static>>,
    home_heading_deg: f32,
    motion_mode: MotionMode,
    actual_heading: &'ctx mut f32,
    persist_nvs: bool,
    need_rehome_stepper_only: &'ctx mut bool,
}

struct UnsupportedEnvironment;

impl TranscriptIo {
    fn into_lines(self) -> Vec<String> {
        self.lines
    }
}

impl DiagnosticIo for TranscriptIo {
    fn write_line(&mut self, msg: &str) -> Result<(), ()> {
        self.lines.push(msg.to_string());
        Ok(())
    }
}

/// Emit the failure on the wire before returning it.
///
/// `serial.rs` only falls back to `transcript.message` when the transcript has no
/// lines, so a board method that fails after writing output would otherwise go
/// silent and leave the installer waiting for its timeout.
fn hdc1080_failure<IO: DiagnosticIo>(io: &mut IO, message: String) -> RuntimeBoardError {
    let _ = io.write_line(&format!("ERROR {message}"));
    RuntimeBoardError::Hdc1080(message)
}

impl<T> DiagnosticBoard for RuntimeDiagnosticBoard<'_, '_, T>
where
    T: NvsPartitionId,
{
    type Error = RuntimeBoardError;

    fn hdc1080_read<IO: DiagnosticIo>(&mut self, io: &mut IO) -> Result<(), Self::Error> {
        let mut sensor = Hdc1080::new(self.bus.acquire_i2c(), Ets)
            .map_err(|err| hdc1080_failure(io, format!("HDC1080 init failed: {:?}", err)))?;

        let device_id = sensor
            .get_device_id()
            .map_err(|err| hdc1080_failure(io, format!("HDC1080 get_device_id failed: {:?}", err)))?;
        let _ = io.write_line(&format!("HDC1080 device_id: 0x{device_id:04X}"));

        let manufacturer_id = sensor
            .get_man_id()
            .map_err(|err| hdc1080_failure(io, format!("HDC1080 get_manufacturer_id failed: {:?}", err)))?;
        let _ = io.write_line(&format!(
            "HDC1080 manufacturer_id: 0x{manufacturer_id:04X}"
        ));

        let serial_id = sensor
            .get_serial_id()
            .map_err(|err| hdc1080_failure(io, format!("HDC1080 get_serial_id failed: {:?}", err)))?;
        let _ = io.write_line(&format!(
            "HDC1080 serial_id: {:04X}-{:04X}-{:04X}",
            serial_id[0], serial_id[1], serial_id[2]
        ));

        let (temp_c, humidity) = sensor
            .read()
            .map_err(|err| hdc1080_failure(io, format!("HDC1080 read failed: {:?}", err)))?;
        // Terminal line for HDC1080_READ. The installer matches this exact shape;
        // see `docs/serial-protocol.md` in Janta_Installer before changing it.
        let _ = io.write_line(&format!(
            "HDC1080 Correct read: temp:{temp_c:.2} hum:{humidity:.2}"
        ));
        Ok(())
    }

    fn rtc_check<IO: DiagnosticIo>(&mut self, io: &mut IO) -> Result<(), Self::Error> {
        let now = Local::now().format("%Y-%m-%d %H:%M:%S").to_string();
        let _ = io.write_line(&format!("TIME: {now}"));
        Ok(())
    }

    fn motor_move<IO: DiagnosticIo>(
        &mut self,
        _argument: &str,
        io: &mut IO,
    ) -> Result<(), Self::Error> {
        let _ = io.write_line("ERROR MOTOR_MOVE is not enabled in first-wave diagnostics");
        Err(RuntimeBoardError::Unsupported("MOTOR_MOVE"))
    }

    fn go_home<IO: DiagnosticIo>(&mut self, io: &mut IO) -> Result<(), Self::Error> {
        let _ = io.write_line("MOVING_HOME");
        let limit_sw_status = match Direction::Ccw {
            Direction::Cw => self.motion.find_limit_switch_cw(),
            Direction::Ccw => self.motion.find_limit_switch_ccw(),
        };

        if !limit_sw_status {
            let message =
                "ERROR Diagnostics go_home failed: limit switch was not found while moving ccw";
            let _ = io.write_line(message);
            return Err(RuntimeBoardError::GoHome(message.to_string()));
        }

        *self.actual_heading = self.home_heading_deg;
        self.motion.update_position(*self.actual_heading);

        if self.persist_nvs {
            infra::SnapshotStore::new(self.nvs, true).save_heading(*self.actual_heading);
            if self.motion_mode == MotionMode::EncoderGuarded {
                infra::SnapshotStore::new(self.nvs, true)
                    .save_encoder_snapshot(self.motion.encoder_ticks_adjusted());
            }
        }

        *self.need_rehome_stepper_only = false;
        let _ = io.write_line("LIMIT");
        Ok(())
    }

    fn oled_test<IO: DiagnosticIo>(
        &mut self,
        _payload: &str,
        io: &mut IO,
    ) -> Result<(), Self::Error> {
        let _ = io.write_line("ERROR OLED_TEST is not enabled in first-wave diagnostics");
        Err(RuntimeBoardError::Unsupported("OLED_TEST"))
    }

    fn relay_motor<IO: DiagnosticIo>(
        &mut self,
        _state: &str,
        io: &mut IO,
    ) -> Result<(), Self::Error> {
        let _ = io.write_line("ERROR RELAY_MOTOR is not enabled in first-wave diagnostics");
        Err(RuntimeBoardError::Unsupported("RELAY_MOTOR"))
    }

    fn relay_hotspot<IO: DiagnosticIo>(
        &mut self,
        _state: &str,
        io: &mut IO,
    ) -> Result<(), Self::Error> {
        let _ = io.write_line("ERROR RELAY_HOTSPOT is not enabled in first-wave diagnostics");
        Err(RuntimeBoardError::Unsupported("RELAY_HOTSPOT"))
    }
}

impl DiagnosticEnvironment for UnsupportedEnvironment {
    type Error = RuntimeEnvironmentError;

    fn set_env(&mut self, _key: &str, _value: &str) -> Result<(), Self::Error> {
        Err(RuntimeEnvironmentError::Unsupported("SET_ENV"))
    }

    fn get_env(&self, _key: &str) -> Result<Option<String>, Self::Error> {
        Err(RuntimeEnvironmentError::Unsupported("GET_ENV"))
    }

    fn save_config(&mut self) -> Result<(), Self::Error> {
        Err(RuntimeEnvironmentError::Unsupported("SAVE_CONFIG"))
    }

    fn save_config_error_message(&self) -> &str {
        "ERROR SAVE_CONFIG is not enabled in first-wave diagnostics"
    }

    fn write_env_file(&mut self) -> Result<(), Self::Error> {
        Err(RuntimeEnvironmentError::Unsupported("WRITE_ENV_FILE"))
    }

    fn write_env_file_error_message(&self) -> &str {
        "ERROR WRITE_ENV_FILE is not enabled in first-wave diagnostics"
    }
}

pub fn capabilities_csv() -> String {
    FIRST_WAVE_CAPABILITIES.join(",")
}

pub fn firmware_version_line(firmware_version: &str) -> String {
    format!("VERSION {firmware_version}")
}

pub fn direct_first_wave_lines(cmd: &str, firmware_version: &str) -> Option<Vec<String>> {
    match cmd {
        "firmware_version" => Some(vec![firmware_version_line(firmware_version)]),
        "get_capabilities" => Some(vec![capabilities_csv()]),
        _ => None,
    }
}

pub fn command_name(command: &DiagnosticCommand) -> &'static str {
    match command {
        DiagnosticCommand::Hdc1080Read => "HDC1080_READ",
        DiagnosticCommand::RtcCheck => "RTC_CHECK",
        DiagnosticCommand::GoHome => "GO_HOME",
        DiagnosticCommand::FirmwareVersion => "FIRMWARE_VERSION",
        DiagnosticCommand::GetCapabilities => "GET_CAPABILITIES",
        DiagnosticCommand::MotorMove { .. } => "MOTOR_MOVE",
        DiagnosticCommand::OledTest { .. } => "OLED_TEST",
        DiagnosticCommand::RelayMotor { .. } => "RELAY_MOTOR",
        DiagnosticCommand::RelayHotspot { .. } => "RELAY_HOTSPOT",
        DiagnosticCommand::ConfigMode => "CONFIG_MODE",
        DiagnosticCommand::SetEnv { .. } => "SET_ENV",
        DiagnosticCommand::InvalidSetEnv { .. } => "SET_ENV",
        DiagnosticCommand::SaveConfig => "SAVE_CONFIG",
        DiagnosticCommand::Reboot => "REBOOT",
        DiagnosticCommand::WriteEnvFile => "WRITE_ENV_FILE",
        DiagnosticCommand::GetEnv { .. } => "GET_ENV",
        DiagnosticCommand::GetConfig => "GET_CONFIG",
        DiagnosticCommand::Unknown { .. } => "UNKNOWN",
    }
}

pub fn supports_first_wave_command(command: &DiagnosticCommand) -> bool {
    matches!(
        command,
        DiagnosticCommand::FirmwareVersion
            | DiagnosticCommand::GetCapabilities
            | DiagnosticCommand::RtcCheck
            | DiagnosticCommand::Hdc1080Read
            | DiagnosticCommand::GoHome
    )
}

pub fn is_control_command(command: &DiagnosticCommand) -> bool {
    matches!(
        command,
        DiagnosticCommand::RtcCheck | DiagnosticCommand::Hdc1080Read | DiagnosticCommand::GoHome
    )
}

pub fn mqtt_control_command(cmd: &str) -> Option<DiagnosticCommand> {
    match cmd {
        "rtc_check" => Some(DiagnosticCommand::RtcCheck),
        "hdc1080_read" => Some(DiagnosticCommand::Hdc1080Read),
        "go_home" | "request_rehome" => Some(DiagnosticCommand::GoHome),
        _ => None,
    }
}

pub fn execute_first_wave_command<'ctx, 'motion, T>(
    command: DiagnosticCommand,
    firmware_version: &str,
    motion: &'ctx mut Motion<'motion>,
    nvs: &'ctx mut EspNvs<T>,
    bus: &'static shared_bus::BusManagerStd<I2cDriver<'static>>,
    home_heading_deg: f32,
    motion_mode: MotionMode,
    actual_heading: &'ctx mut f32,
    persist_nvs: bool,
    need_rehome_stepper_only: &'ctx mut bool,
) -> DiagnosticsTranscript
where
    T: NvsPartitionId,
{
    if !supports_first_wave_command(&command) {
        return DiagnosticsTranscript {
            status: "failed",
            message: format!(
                "{} is not enabled in first-wave diagnostics",
                command_name(&command)
            ),
            lines: vec![format!(
                "ERROR {} is not enabled in first-wave diagnostics",
                command_name(&command)
            )],
        };
    }

    let configuration = StaticDiagnosticConfiguration::new(firmware_version)
        .with_capability("MQTT")
        .with_capability("SERIAL")
        .with_capability("STATUS")
        .with_capability("RTC")
        .with_capability("HDC1080")
        .with_capability("GO_HOME");
    let board = RuntimeDiagnosticBoard {
        motion,
        nvs,
        bus,
        home_heading_deg,
        motion_mode,
        actual_heading,
        persist_nvs,
        need_rehome_stepper_only,
    };
    let environment = UnsupportedEnvironment;
    let mut handler = StandardDiagnosticHandler::new(board, environment, configuration);
    let mut io = TranscriptIo::default();

    match handler.handle_command(command.clone(), &mut io) {
        Ok(DiagnosticControl::Continue) => DiagnosticsTranscript {
            status: "completed",
            message: success_message(&command),
            lines: io.into_lines(),
        },
        Ok(DiagnosticControl::RebootRequested) => DiagnosticsTranscript {
            status: "completed",
            message: String::from("Diagnostics command requested a reboot"),
            lines: io.into_lines(),
        },
        Err(err) => DiagnosticsTranscript {
            status: "failed",
            message: failure_message(&command, &err),
            lines: io.into_lines(),
        },
    }
}

fn success_message(command: &DiagnosticCommand) -> String {
    match command {
        DiagnosticCommand::FirmwareVersion => String::from("Firmware version reported"),
        DiagnosticCommand::GetCapabilities => String::from("Diagnostics capabilities reported"),
        DiagnosticCommand::RtcCheck => String::from("RTC check completed"),
        DiagnosticCommand::Hdc1080Read => String::from("HDC1080 diagnostic completed"),
        DiagnosticCommand::GoHome => String::from("Diagnostics go_home completed successfully"),
        _ => format!("{} completed", command_name(command)),
    }
}

fn failure_message(
    command: &DiagnosticCommand,
    error: &StandardDiagnosticError<RuntimeBoardError, RuntimeEnvironmentError, ()>,
) -> String {
    match error {
        StandardDiagnosticError::Board(RuntimeBoardError::Hdc1080(message))
        | StandardDiagnosticError::Board(RuntimeBoardError::GoHome(message)) => {
            message.to_string()
        }
        StandardDiagnosticError::Board(RuntimeBoardError::Unsupported(name)) => {
            format!("{name} is not enabled in first-wave diagnostics")
        }
        StandardDiagnosticError::Environment(RuntimeEnvironmentError::Unsupported(name)) => {
            format!("{name} is not enabled in first-wave diagnostics")
        }
        StandardDiagnosticError::Config(()) => {
            format!("{} could not report configuration", command_name(command))
        }
    }
}
