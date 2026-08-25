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
    Ping,
    I2cScan,
    LedTest { color: String },
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
            "PING" => Self::Ping,
            "I2C_SCAN" => Self::I2cScan,
            "LED_TEST" => Self::LedTest {
                color: rest.to_uppercase(),
            },
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

impl DiagnosticCommand {
    /// The protocol name for this command, as it appears in a `RESULT` line.
    pub fn name(&self) -> &'static str {
        match self {
            Self::Ping => "PING",
            Self::I2cScan => "I2C_SCAN",
            Self::LedTest { .. } => "LED_TEST",
            Self::Hdc1080Read => "HDC1080_READ",
            Self::RtcCheck => "RTC_CHECK",
            Self::MotorMove { .. } => "MOTOR_MOVE",
            Self::GoHome => "GO_HOME",
            Self::OledTest { .. } => "OLED_TEST",
            Self::RelayMotor { .. } => "RELAY_MOTOR",
            Self::RelayHotspot { .. } => "RELAY_HOTSPOT",
            Self::ConfigMode => "CONFIG_MODE",
            Self::SetEnv { .. } | Self::InvalidSetEnv { .. } => "SET_ENV",
            Self::SaveConfig => "SAVE_CONFIG",
            Self::Reboot => "REBOOT",
            Self::WriteEnvFile => "WRITE_ENV_FILE",
            Self::GetEnv { .. } => "GET_ENV",
            Self::FirmwareVersion => "FIRMWARE_VERSION",
            Self::GetCapabilities => "GET_CAPABILITIES",
            Self::GetConfig => "GET_CONFIG",
            Self::Unknown { .. } => "UNKNOWN",
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

/// The `FIRMWARE_VERSION` terminal line.
///
/// One formatter, so the standard handler, the boot-phase responder, and the
/// firmware's MQTT path cannot drift apart on a string the installer matches.
pub fn firmware_version_line(firmware_version: &str) -> String {
    format!("VERSION {}", firmware_version)
}

/// Lines answering a command that arrived before the runtime owns any hardware.
///
/// During boot there is no motion, NVS, or I2C bus to lend a diagnostics
/// handler, but the installer is already connected and asking — it queries
/// `FIRMWARE_VERSION` one second after opening the port. These three commands
/// need none of that hardware and are answered for real. Everything else is
/// refused immediately, naming the phase, so the installer can show why instead
/// of waiting out its timeout in silence.
///
/// `phase` is a short human-readable description of what the tower is doing;
/// it goes on the wire verbatim.
pub fn boot_phase_response(
    command: &DiagnosticCommand,
    phase: &str,
    firmware_version: &str,
    capabilities_csv: &str,
) -> Vec<String> {
    let name = command.name();

    // Every branch ends in a RESULT. A host running the structured protocol
    // resolves on that and nothing else, so a refusal emitting only
    // `ERROR BUSY ...` would leave it waiting out its timeout — exactly the stall
    // this path exists to prevent.
    match command {
        DiagnosticCommand::Ping => {
            vec![String::from("PONG"), result_line(name, true, &[], "pong")]
        }
        DiagnosticCommand::FirmwareVersion => vec![
            protocol_version_line(),
            firmware_version_line(firmware_version),
            result_line(
                name,
                true,
                &[(String::from("version"), firmware_version.to_string())],
                "",
            ),
        ],
        DiagnosticCommand::GetCapabilities => vec![
            capabilities_csv.to_string(),
            result_line(
                name,
                true,
                &[(String::from("capabilities"), capabilities_csv.to_string())],
                "",
            ),
        ],
        _ => vec![
            format!("ERROR BUSY {}", phase),
            result_line(name, false, &[(String::from("busy"), phase.to_string())], ""),
        ],
    }
}

/// Commands that cannot be served from inside a move, whatever else the runtime
/// can lend.
///
/// The mid-move serial path answers everything needing only the I2C bus, the
/// status LED, or NVS — none of which the stepping loop holds. Refusing them
/// meant a technician stood beside a homing tower could not read a sensor, scan
/// the bus or push a configuration, and a homing search can run for tens of
/// minutes.
///
/// What stays refused is refused for a reason, not for tidiness:
///
/// - `GO_HOME`, `MOTOR_MOVE` and `RELAY_MOTOR` want the motor or its power, which
///   the move in progress is using.
/// - `REBOOT` belongs to the main loop alone. Serving it here would mean either
///   resetting from inside the stepping loop with the motor relay still closed,
///   or holding the request until the move ends — which for a homing sweep can be
///   twenty minutes after it was asked for.
///
/// Written as an exhaustive match with no wildcard arm: a command added later
/// stops this compiling until someone classifies it. Getting it wrong is
/// otherwise silent, because a command that could have run is refused as
/// `ERROR BUSY <phase>`, which on the wire is indistinguishable from a real one.
pub fn requires_exclusive_runtime(command: &DiagnosticCommand) -> bool {
    match command {
        DiagnosticCommand::GoHome
        | DiagnosticCommand::MotorMove { .. }
        | DiagnosticCommand::RelayMotor { .. }
        | DiagnosticCommand::Reboot => true,

        // Bus, LED and NVS are all lendable while the tower moves.
        DiagnosticCommand::Ping
        | DiagnosticCommand::I2cScan
        | DiagnosticCommand::LedTest { .. }
        | DiagnosticCommand::Hdc1080Read
        | DiagnosticCommand::RtcCheck
        | DiagnosticCommand::OledTest { .. }
        | DiagnosticCommand::RelayHotspot { .. }
        | DiagnosticCommand::ConfigMode
        | DiagnosticCommand::SetEnv { .. }
        | DiagnosticCommand::InvalidSetEnv { .. }
        | DiagnosticCommand::SaveConfig
        | DiagnosticCommand::WriteEnvFile
        | DiagnosticCommand::GetEnv { .. }
        | DiagnosticCommand::FirmwareVersion
        | DiagnosticCommand::GetCapabilities
        | DiagnosticCommand::GetConfig => false,

        // Answered so the host gets `UNKNOWN is not enabled`, which says what is
        // wrong, rather than `BUSY homing`, which sends someone waiting for a
        // move to finish before retrying a command that will never work.
        DiagnosticCommand::Unknown { .. } => false,
    }
}

/// How much the move in progress can afford to be interrupted.
///
/// Not a stand-in for "is the tower moving" — both variants mean it is. What
/// differs is what a pause forfeits, and it is not what one might assume. The
/// tower turns slowly enough that pausing it is mechanically harmless either way:
/// 25,600 microsteps per motor revolution behind an 85:1 slew, accelerating at
/// 200 steps/s², puts a one-degree homing move at eleven seconds and a peak of
/// about 1,100 steps/s — under 3 rpm at the motor. Nothing restarted from rest at
/// that speed loses steps.
///
/// What a pause really costs is *supervision*. The stall and overshoot detectors
/// live inside the stepping loop and run only between `motor.poll()` calls, so a
/// blocked loop is a blind loop.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MoveTolerance {
    /// A homing search, which switches stall detection off and skips the overshoot
    /// check for its whole duration. There is no supervision to interrupt, and the
    /// heading afterwards comes from the limit switch rather than the step count —
    /// so a pause costs time and nothing else.
    Homing,
    /// A tracking or recovery move, where stall detection and overshoot protection
    /// are both live. Every millisecond the loop spends blocked is a millisecond a
    /// stalled or overshooting tower goes unnoticed.
    PositionCritical,
}

impl MoveTolerance {
    /// May a command hold the stepping loop for seconds rather than milliseconds?
    ///
    /// Only the full `OLED_TEST` walk asks for that: on this board's 10 kHz I2C
    /// bus a single full-screen write is close to a second, and the walk is five
    /// of them. Everything else is two orders of magnitude cheaper — a whole
    /// `HDC1080_READ` is about 60 ms, against a stall detector that samples every
    /// 250 ms — so the rest run under either tolerance.
    pub fn allows_long_blocking(self) -> bool {
        matches!(self, Self::Homing)
    }
}

/// The structured protocol version this firmware speaks.
///
/// Emitted ahead of the `VERSION` line so a host can record it while the command
/// is still pending — the installer resolves `FIRMWARE_VERSION` on `VERSION`, so
/// anything sent after that arrives with no request to attach it to.
pub const PROTOCOL_VERSION: u32 = 1;

pub fn protocol_version_line() -> String {
    format!("PROTOCOL_VERSION {PROTOCOL_VERSION}")
}

/// Collapse anything that would break the line protocol.
///
/// Details and failure messages are built from device output and error text, and a
/// newline in either would split one response into two lines — the second of which
/// the host would try to interpret as a result of its own.
fn single_line(text: &str) -> String {
    text.split_whitespace().collect::<Vec<_>>().join(" ")
}

/// The terminal `RESULT` line for a command.
///
/// One formatter for every transport and every command, because the alternative is
/// what caused the HDC1080 bug: two repositories agreeing about a string until they
/// quietly stopped.
///
/// `details` renders as `key=value` pairs the host can display as data. When there
/// are none, `fallback` carries the human-readable summary instead — a failure
/// reason is far more useful than an empty result.
pub fn result_line(
    command: &str,
    passed: bool,
    details: &[(String, String)],
    fallback: &str,
) -> String {
    let status = if passed { "PASS" } else { "FAIL" };
    if details.is_empty() {
        let summary = single_line(fallback);
        if summary.is_empty() {
            return format!("RESULT {command} {status}");
        }
        return format!("RESULT {command} {status} {summary}");
    }

    let rendered = details
        .iter()
        .map(|(key, value)| format!("{}={}", single_line(key), single_line(value)))
        .collect::<Vec<_>>()
        .join(" ");
    format!("RESULT {command} {status} {rendered}")
}

/// Colours `LED_TEST` can drive, and the exact values it writes.
///
/// Red, green and blue are here to exercise each channel of the WS2812
/// independently — a single dead channel is a real failure mode that a white-only
/// test would miss. White proves all three together, and off proves the LED can be
/// driven dark rather than merely being stuck on.
///
/// The host renders its own preview from these values, so the two must agree:
/// `led_test_colors_match_the_installer` pins them here, and
/// `diagnosticProtocol.test.ts` pins the matching side.
pub const LED_TEST_COLORS: &[(&str, u8, u8, u8)] = &[
    ("RED", 255, 0, 0),
    ("GREEN", 0, 255, 0),
    ("BLUE", 0, 0, 255),
    ("WHITE", 150, 150, 150),
    ("OFF", 0, 0, 0),
];

/// The colour an argument selects: a name, or a literal `R,G,B` triplet.
///
/// The triplet form exists for probing hardware. A named set is enough to walk an
/// operator through a test, but not to answer questions like "does 0,0,1 light up
/// when 0,0,0 does not" — and those are what distinguish a firmware bug from an LED
/// that does not go fully dark.
pub fn led_test_color(name: &str) -> Option<(u8, u8, u8)> {
    if let Some(triplet) = parse_rgb_triplet(name) {
        return Some(triplet);
    }
    LED_TEST_COLORS
        .iter()
        .find(|(candidate, ..)| candidate.eq_ignore_ascii_case(name))
        .map(|(_, r, g, b)| (*r, *g, *b))
}

fn parse_rgb_triplet(text: &str) -> Option<(u8, u8, u8)> {
    let mut parts = text.split(',');
    let red = parts.next()?.trim().parse().ok()?;
    let green = parts.next()?.trim().parse().ok()?;
    let blue = parts.next()?.trim().parse().ok()?;
    // Reject a fourth field rather than ignoring it: silently dropping part of what
    // someone typed is how a probe gives a confidently wrong answer.
    if parts.next().is_some() {
        return None;
    }
    Some((red, green, blue))
}

/// Names `LED_TEST` accepts, for the error a bad argument produces.
pub fn led_test_color_names() -> String {
    LED_TEST_COLORS
        .iter()
        .map(|(name, ..)| *name)
        .collect::<Vec<_>>()
        .join(",")
}

/// An unprompted announcement of what the tower is doing.
///
/// Boot and long moves are exactly when a technician is most likely to be staring
/// at a board that cannot answer yet. Without this the host sees silence and has
/// no way to tell "still associating Wi-Fi" from "wedged".
///
/// Unsolicited, so it can land while a command is pending: the shape constraints
/// in `phase_lines_cannot_be_mistaken_for_a_result` are what keep it from
/// resolving somebody else's command.
pub fn phase_line(phase: &str) -> String {
    format!("PHASE {phase}")
}

/// A progress line emitted while a homing search is running.
///
/// Each line restarts the installer's silence budget, which is what lets a search
/// lasting minutes finish instead of being timed out. The shape matters as much as
/// the content: it must not equal `LIMIT` and must not begin with `ERROR`, `FAIL`
/// or `BAD`, or the host would resolve the command early — as a pass or a failure
/// — while the tower is still moving.
/// `homing_progress_lines_cannot_be_mistaken_for_a_result` pins that here, and
/// `diagnosticProtocol.test.ts` pins the matching side in the installer.
pub fn homing_progress_line(steps_remaining: i64, encoder_ticks: i32) -> String {
    format!("HOMING steps_remaining={steps_remaining} encoder_ticks={encoder_ticks}")
}

// ---- Provisioned configuration -------------------------------------------

/// NVS flag set by `SAVE_CONFIG`, meaning the tower carries a provisioned site
/// configuration that boot must not overwrite from build-time defaults.
pub const NVS_KEY_PROVISIONED: &str = "provisioned";

/// Longest value `SET_ENV` accepts, in bytes. Bounds the staging buffer so a host
/// cannot exhaust RAM by sending one enormous value.
pub const CONFIG_VALUE_MAX_BYTES: usize = 128;

/// Reported in place of a secret that is configured.
pub const SECRET_SET: &str = "<set>";
/// Reported in place of a secret that is not configured.
pub const SECRET_UNSET: &str = "<unset>";

/// How a value is validated before the board will accept it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConfigValueKind {
    Text,
    Latitude,
    Longitude,
    Altitude,
    TimezoneOffsetHours,
    Port,
}

/// One provisioned setting: what the installer calls it, where it lives in NVS,
/// and what counts as a valid value.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ConfigKey {
    pub protocol: &'static str,
    pub nvs: &'static str,
    pub kind: ConfigValueKind,
    /// Never echoed back; reads report only whether it is set.
    pub secret: bool,
}

impl ConfigKey {
    /// The `[section]` this key belongs to in `GET_CONFIG` output.
    pub fn section(&self) -> &'static str {
        match self.protocol.split_once('.') {
            Some((section, _)) => section,
            None => self.protocol,
        }
    }

    /// The key name within its section.
    pub fn name(&self) -> &'static str {
        match self.protocol.split_once('.') {
            Some((_, name)) => name,
            None => self.protocol,
        }
    }
}

/// Every setting the installer may provision, and nothing else.
///
/// NVS key names cap at 15 bytes, so they cannot simply be the protocol keys —
/// `location.timezone_offset_hours` is 30. Existing runtime keys are reused
/// deliberately, so a provisioned value feeds the code that already reads it.
///
/// Anything absent from this table is rejected. Accepting an unknown key and
/// answering `OK` would report a provisioning that never happened, which is worse
/// than failing.
pub const CONFIG_KEYS: &[ConfigKey] = &[
    ConfigKey { protocol: "device.tower_id", nvs: "tower_id", kind: ConfigValueKind::Text, secret: false },
    ConfigKey { protocol: "wifi.ssid", nvs: "wifi_ssid", kind: ConfigValueKind::Text, secret: false },
    ConfigKey { protocol: "wifi.password", nvs: "wifi_pass", kind: ConfigValueKind::Text, secret: true },
    ConfigKey { protocol: "location.latitude", nvs: "tower_latitude", kind: ConfigValueKind::Latitude, secret: false },
    ConfigKey { protocol: "location.longitude", nvs: "tower_longitude", kind: ConfigValueKind::Longitude, secret: false },
    ConfigKey { protocol: "location.altitude", nvs: "tower_altitude", kind: ConfigValueKind::Altitude, secret: false },
    ConfigKey { protocol: "location.timezone_offset_hours", nvs: "tz_offset_h", kind: ConfigValueKind::TimezoneOffsetHours, secret: false },
    ConfigKey { protocol: "mqtt.broker", nvs: "mqtt_broker", kind: ConfigValueKind::Text, secret: false },
    ConfigKey { protocol: "mqtt.port", nvs: "mqtt_port", kind: ConfigValueKind::Port, secret: false },
    ConfigKey { protocol: "mqtt.username", nvs: "mqtt_user", kind: ConfigValueKind::Text, secret: false },
    ConfigKey { protocol: "mqtt.password", nvs: "mqtt_pass", kind: ConfigValueKind::Text, secret: true },
    ConfigKey { protocol: "mqtt.topic", nvs: "mqtt_topic", kind: ConfigValueKind::Text, secret: false },
    ConfigKey { protocol: "customer.full_name", nvs: "cust_name", kind: ConfigValueKind::Text, secret: false },
    ConfigKey { protocol: "customer.address", nvs: "cust_addr", kind: ConfigValueKind::Text, secret: false },
    ConfigKey { protocol: "customer.phone", nvs: "cust_phone", kind: ConfigValueKind::Text, secret: false },
];

/// Look up a protocol key. `None` means the host sent something we do not provision.
pub fn config_key(protocol: &str) -> Option<&'static ConfigKey> {
    CONFIG_KEYS.iter().find(|entry| entry.protocol == protocol)
}

/// What a read reports for a secret.
///
/// The value never leaves the board — echoing it would mean anyone with a USB
/// cable could lift the site's Wi-Fi password. The installer treats an unmodified
/// `<set>` as "leave this alone" and omits the `SET_ENV` for it.
pub fn secret_placeholder(is_set: bool) -> &'static str {
    if is_set { SECRET_SET } else { SECRET_UNSET }
}

/// Whether a value is one of the read-back placeholders rather than a real secret.
pub fn is_secret_placeholder(value: &str) -> bool {
    value == SECRET_SET || value == SECRET_UNSET
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConfigError {
    UnknownKey,
    TooLong,
    NotANumber,
    OutOfRange,
    SecretPlaceholder,
    WriteFailed,
    Unsupported,
}

impl ConfigError {
    pub fn message(self) -> &'static str {
        match self {
            Self::UnknownKey => "unknown configuration key",
            Self::TooLong => "value is too long",
            Self::NotANumber => "value is not a number",
            Self::OutOfRange => "value is out of range",
            Self::SecretPlaceholder => "refusing to store a read-back placeholder as a secret",
            Self::WriteFailed => "configuration could not be written",
            Self::Unsupported => "not supported on this device",
        }
    }
}

/// Validate a value against its key.
///
/// The installer validates too. This exists because the board must not trust the
/// host: a bad latitude persisted here sends the tower to the wrong place, and a
/// placeholder stored as a password silently breaks the site's Wi-Fi.
pub fn validate_config_value(key: &ConfigKey, value: &str) -> Result<(), ConfigError> {
    if value.len() > CONFIG_VALUE_MAX_BYTES {
        return Err(ConfigError::TooLong);
    }
    if key.secret && is_secret_placeholder(value) {
        return Err(ConfigError::SecretPlaceholder);
    }

    match key.kind {
        ConfigValueKind::Text => Ok(()),
        ConfigValueKind::Latitude => check_range_f64(value, -90.0, 90.0),
        ConfigValueKind::Longitude => check_range_f64(value, -180.0, 180.0),
        // Generous bounds rather than none: enough for any real site, tight enough
        // to catch a value that arrived in the wrong units or the wrong field.
        ConfigValueKind::Altitude => check_range_f64(value, -500.0, 10_000.0),
        ConfigValueKind::TimezoneOffsetHours => {
            let parsed = value.trim().parse::<i32>().map_err(|_| ConfigError::NotANumber)?;
            if (-12..=14).contains(&parsed) { Ok(()) } else { Err(ConfigError::OutOfRange) }
        }
        ConfigValueKind::Port => {
            let parsed = value.trim().parse::<u32>().map_err(|_| ConfigError::NotANumber)?;
            if (1..=65_535).contains(&parsed) { Ok(()) } else { Err(ConfigError::OutOfRange) }
        }
    }
}

fn check_range_f64(value: &str, min: f64, max: f64) -> Result<(), ConfigError> {
    let parsed = value.trim().parse::<f64>().map_err(|_| ConfigError::NotANumber)?;
    if !parsed.is_finite() {
        return Err(ConfigError::NotANumber);
    }
    if parsed < min || parsed > max {
        return Err(ConfigError::OutOfRange);
    }
    Ok(())
}

/// Values accepted by `SET_ENV` but not yet written to NVS.
///
/// The wire sequence is many `SET_ENV`, one `SAVE_CONFIG`, then `GET_ENV` to
/// verify. Staging means an interrupted sequence — a technician unplugging
/// halfway through — leaves the tower on its previous configuration rather than
/// half-provisioned with a new SSID and an old password.
#[derive(Debug, Default)]
pub struct ConfigStaging {
    entries: Vec<(&'static ConfigKey, String)>,
}

impl ConfigStaging {
    pub fn new() -> Self {
        Self::default()
    }

    /// Stage a validated value, replacing any earlier one for the same key.
    pub fn stage(&mut self, key: &'static ConfigKey, value: String) {
        match self
            .entries
            .iter_mut()
            .find(|(staged, _)| staged.protocol == key.protocol)
        {
            Some(entry) => entry.1 = value,
            None => self.entries.push((key, value)),
        }
    }

    /// The staged value for a protocol key, if one is waiting.
    pub fn get(&self, protocol: &str) -> Option<&str> {
        self.entries
            .iter()
            .find(|(staged, _)| staged.protocol == protocol)
            .map(|(_, value)| value.as_str())
    }

    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    pub fn len(&self) -> usize {
        self.entries.len()
    }

    /// Everything staged, in the order it was first set.
    pub fn entries(&self) -> &[(&'static ConfigKey, String)] {
        &self.entries
    }

    pub fn clear(&mut self) {
        self.entries.clear();
    }
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

    /// Drive the status LED to a named colour, or `RESTORE` to put it back.
    ///
    /// Purely an output: a WS2812 has no readback, so this reports that the colour
    /// was written, never that light came out. Whether the LED works is a judgement
    /// only the person looking at the board can make.
    fn led_test<IO: DiagnosticIo>(&mut self, _color: &str, _io: &mut IO) -> Result<(), Self::Error> {
        Ok(())
    }

    /// Report which devices acknowledge on the shared I2C bus.
    ///
    /// The question a failed sensor read cannot answer on its own: a `NoAcknowledge`
    /// tells you one address stayed silent, not whether the bus itself is alive.
    fn i2c_scan<IO: DiagnosticIo>(&mut self, _io: &mut IO) -> Result<(), Self::Error> {
        Ok(())
    }

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

    /// Validate and accept a value.
    ///
    /// `io` is here for the same reason [`DiagnosticBoard`]'s methods have it: an
    /// implementor must be able to say *why* it refused. `SET_ENV` is sent once per
    /// setting, and a rejection that writes nothing reads to the host as a hung
    /// board rather than a bad value.
    fn set_env<IO: DiagnosticIo>(
        &mut self,
        key: &str,
        value: &str,
        io: &mut IO,
    ) -> Result<(), Self::Error>;

    fn get_env<IO: DiagnosticIo>(
        &mut self,
        key: &str,
        io: &mut IO,
    ) -> Result<Option<String>, Self::Error>;

    fn save_config<IO: DiagnosticIo>(&mut self, io: &mut IO) -> Result<(), Self::Error>;

    fn write_env_file<IO: DiagnosticIo>(&mut self, _io: &mut IO) -> Result<(), Self::Error> {
        Ok(())
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

    /// Number of sections `GET_CONFIG` will write.
    ///
    /// Reported in its terminal `RESULT` so the host knows the response is complete
    /// rather than having to guess from a timer.
    pub fn section_count(&self) -> usize {
        self.sections.len()
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
            DiagnosticCommand::Ping => {
                let _ = io.write_line("PONG");
            }
            DiagnosticCommand::Hdc1080Read => self
                .board
                .hdc1080_read(io)
                .map_err(StandardDiagnosticError::Board)?,
            DiagnosticCommand::RtcCheck => self
                .board
                .rtc_check(io)
                .map_err(StandardDiagnosticError::Board)?,
            DiagnosticCommand::I2cScan => self
                .board
                .i2c_scan(io)
                .map_err(StandardDiagnosticError::Board)?,
            DiagnosticCommand::LedTest { color } => self
                .board
                .led_test(&color, io)
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
                    .set_env(&key, &value, io)
                    .map_err(StandardDiagnosticError::Environment)?;
                let _ = io.write_line("OK");
            }
            DiagnosticCommand::InvalidSetEnv { .. } => {
                let _ = io.write_line("ERROR Bad SET_ENV format");
            }
            DiagnosticCommand::SaveConfig => {
                self.environment
                    .save_config(io)
                    .map_err(StandardDiagnosticError::Environment)?;
                let _ = io.write_line("OK");
            }
            DiagnosticCommand::Reboot => {
                let _ = io.write_line("OK Rebooting");
                return Ok(DiagnosticControl::RebootRequested);
            }
            DiagnosticCommand::WriteEnvFile => {
                self.environment
                    .write_env_file(io)
                    .map_err(StandardDiagnosticError::Environment)?;
                let _ = io.write_line("OK");
            }
            DiagnosticCommand::GetEnv { key } => {
                let value = self
                    .environment
                    .get_env(&key, io)
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
                // Ordering matters: the host resolves this command on the VERSION
                // line, so the handshake has to precede it or it lands after the
                // request has already completed and is discarded.
                let _ = io.write_line(&protocol_version_line());
                let _ = io.write_line(&firmware_version_line(
                    self.configuration.firmware_version(),
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

        fn set_env<IO: DiagnosticIo>(
            &mut self,
            key: &str,
            value: &str,
            _io: &mut IO,
        ) -> Result<(), Self::Error> {
            self.env.insert(key.to_string(), value.to_string());
            Ok(())
        }

        fn get_env<IO: DiagnosticIo>(
            &mut self,
            key: &str,
            _io: &mut IO,
        ) -> Result<Option<String>, Self::Error> {
            Ok(self.env.get(key).cloned())
        }

        fn save_config<IO: DiagnosticIo>(&mut self, io: &mut IO) -> Result<(), Self::Error> {
            if self.fail_save {
                let _ = io.write_line("ERROR NVS_LOCK");
                Err("save_failed")
            } else {
                Ok(())
            }
        }

        fn write_env_file<IO: DiagnosticIo>(&mut self, _io: &mut IO) -> Result<(), Self::Error> {
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

    // The boot-phase path is the only diagnostics the installer can reach while
    // the tower is associating to Wi-Fi or validating its boot, so its exact
    // wire lines are pinned here rather than left to the firmware crate.
    #[test]
    fn only_the_motor_and_the_reset_are_withheld_during_a_move() {
        // The point of the mid-move path. Every one of these needs the I2C bus,
        // the status LED or NVS, and a move holds none of them — so refusing them
        // stranded a technician for the length of a homing search.
        for line in [
            "PING",
            "I2C_SCAN",
            "HDC1080_READ",
            "RTC_CHECK",
            "LED_TEST RED",
            "OLED_TEST BORDER",
            "CONFIG_MODE",
            "SET_ENV wifi.ssid=site",
            "SAVE_CONFIG",
            "GET_ENV device.tower_id",
            "GET_CONFIG",
            "FIRMWARE_VERSION",
            "GET_CAPABILITIES",
        ] {
            assert!(
                !requires_exclusive_runtime(&DiagnosticCommand::parse(line)),
                "{line} needs nothing a move holds and must be answerable mid-move"
            );
        }

        for line in ["GO_HOME", "MOTOR_MOVE 10", "RELAY_MOTOR ON", "REBOOT"] {
            assert!(
                requires_exclusive_runtime(&DiagnosticCommand::parse(line)),
                "{line} wants the motor or the main loop and must stay refused mid-move"
            );
        }
    }

    #[test]
    fn an_unrecognised_command_is_not_reported_as_busy() {
        // `BUSY homing` invites a retry after the move; `not enabled` says the
        // retry will not help. A typo deserves the second answer.
        assert!(!requires_exclusive_runtime(&DiagnosticCommand::parse("NONSENSE")));
    }

    #[test]
    fn only_homing_may_block_the_stepping_loop_for_seconds() {
        assert!(MoveTolerance::Homing.allows_long_blocking());
        assert!(!MoveTolerance::PositionCritical.allows_long_blocking());
    }

    #[test]
    fn boot_phase_answers_hardware_free_commands_and_refuses_the_rest() {
        let caps = "SERIAL,STATUS,PING";
        let answer = |line: &str| {
            boot_phase_response(
                &DiagnosticCommand::parse(line),
                "waiting for Wi-Fi",
                "1.1.3",
                caps,
            )
        };

        // Every response ends in a RESULT so a strict host can resolve on it;
        // the legacy line stays ahead of it for hosts that predate the dialect.
        assert_eq!(answer("PING"), vec!["PONG", "RESULT PING PASS pong"]);
        assert_eq!(
            answer("FIRMWARE_VERSION"),
            vec![
                "PROTOCOL_VERSION 1",
                "VERSION 1.1.3",
                "RESULT FIRMWARE_VERSION PASS version=1.1.3",
            ]
        );
        assert_eq!(
            answer("GET_CAPABILITIES"),
            vec![caps, "RESULT GET_CAPABILITIES PASS capabilities=SERIAL,STATUS,PING"]
        );

        // Anything needing hardware is refused with the phase, not left silent —
        // and the refusal terminates for a strict host too.
        for line in ["HDC1080_READ", "GO_HOME", "REBOOT", "SET_ENV wifi.ssid=x", "NONSENSE"] {
            let answered = answer(line);
            assert_eq!(
                answered[0], "ERROR BUSY waiting for Wi-Fi",
                "unexpected boot-phase answer for {line}"
            );
            assert!(
                answered[1].starts_with("RESULT ") && answered[1].contains(" FAIL busy=waiting for Wi-Fi"),
                "refusal must terminate with a RESULT: {}",
                answered[1]
            );
        }

        // The RESULT names the command that was refused, not a generic label.
        assert!(answer("GO_HOME")[1].starts_with("RESULT GO_HOME FAIL"));
        assert!(answer("SET_ENV wifi.ssid=x")[1].starts_with("RESULT SET_ENV FAIL"));
    }

    /// Assert a line the firmware emits unprompted cannot resolve a pending command.
    ///
    /// These arrive whenever the firmware has something to say, including while the
    /// host is waiting on an unrelated command. Matching a terminal pattern would
    /// resolve that command against output that has nothing to do with it — a
    /// homing progress line reading as `LIMIT` would report the tower home while it
    /// is still searching, which is the worst failure this protocol can produce.
    ///
    /// Mirrors the accept rules in the installer's `diagnosticProtocol.ts`.
    fn assert_cannot_resolve_a_command(line: &str) {
        for terminal in ["OK", "LIMIT", "PONG"] {
            assert_ne!(line, terminal, "unsolicited line must not equal {terminal}");
        }
        for prefix in ["ERROR", "FAIL", "BAD", "VERSION ", "RESULT ", "TIME:"] {
            assert!(
                !line.starts_with(prefix),
                "unsolicited line must not start with {prefix}: {line}"
            );
        }
        // GET_CAPABILITIES accepts any bare comma-separated identifier list; a
        // space disqualifies a line from that pattern.
        assert!(
            line.contains(' '),
            "unsolicited line needs a space so it cannot read as a capability list: {line}"
        );
    }

    #[test]
    fn homing_progress_lines_cannot_be_mistaken_for_a_result() {
        let line = homing_progress_line(1_244_444, -8_421);
        assert_eq!(line, "HOMING steps_remaining=1244444 encoder_ticks=-8421");
        assert_cannot_resolve_a_command(&line);
    }

    #[test]
    fn result_lines_render_details_and_survive_hostile_text() {
        assert_eq!(
            result_line(
                "HDC1080_READ",
                true,
                &[
                    (String::from("temp"), String::from("24.50")),
                    (String::from("hum"), String::from("55.50")),
                ],
                "ignored when details exist",
            ),
            "RESULT HDC1080_READ PASS temp=24.50 hum=55.50"
        );

        // With nothing structured to say, the summary carries the meaning — which
        // matters most on failures.
        assert_eq!(
            result_line("GO_HOME", false, &[], "limit switch was not found"),
            "RESULT GO_HOME FAIL limit switch was not found"
        );
        assert_eq!(result_line("REBOOT", true, &[], ""), "RESULT REBOOT PASS");

        // A newline in device output or an error would split one response into two
        // lines, and the host would read the second as a result of its own.
        let hostile = result_line(
            "SET_ENV",
            false,
            &[],
            "bad value\r\nRESULT SET_ENV PASS injected",
        );
        assert_eq!(hostile.lines().count(), 1, "result line must stay on one line");
        assert_eq!(
            hostile,
            "RESULT SET_ENV FAIL bad value RESULT SET_ENV PASS injected"
        );
    }

    #[test]
    fn protocol_version_line_matches_the_documented_dialect() {
        assert_eq!(protocol_version_line(), "PROTOCOL_VERSION 1");
        assert_eq!(PROTOCOL_VERSION, 1);
    }

    #[test]
    fn led_test_colors_match_the_installer() {
        // The host renders its spinner from its own copy of these values; a drift
        // here would show the technician one colour and drive another.
        assert_eq!(led_test_color("RED"), Some((255, 0, 0)));
        assert_eq!(led_test_color("GREEN"), Some((0, 255, 0)));
        assert_eq!(led_test_color("BLUE"), Some((0, 0, 255)));
        assert_eq!(led_test_color("WHITE"), Some((150, 150, 150)));
        assert_eq!(led_test_color("OFF"), Some((0, 0, 0)));

        assert_eq!(led_test_color("red"), Some((255, 0, 0)), "names are case-insensitive");
        assert_eq!(led_test_color("PURPLE"), None);

        // Literal triplets, for probing the LED directly.
        assert_eq!(led_test_color("0,0,1"), Some((0, 0, 1)));
        assert_eq!(led_test_color("255,128,0"), Some((255, 128, 0)));
        assert_eq!(led_test_color(" 1 , 2 , 3 "), Some((1, 2, 3)));
        assert_eq!(led_test_color("1,2"), None, "too few fields");
        assert_eq!(led_test_color("1,2,3,4"), None, "a dropped field would mislead");
        assert_eq!(led_test_color("1,2,300"), None, "out of range for a u8");
        assert_eq!(led_test_color("1,2,x"), None);
        assert_eq!(led_test_color(""), None);
        assert_eq!(led_test_color_names(), "RED,GREEN,BLUE,WHITE,OFF");

        assert_eq!(
            DiagnosticCommand::parse("LED_TEST red"),
            DiagnosticCommand::LedTest { color: String::from("RED") }
        );
    }

    #[test]
    fn phase_lines_cannot_be_mistaken_for_a_result() {
        assert_eq!(phase_line("waiting for MQTT"), "PHASE waiting for MQTT");

        // Every phase the runtime announces, so a new one cannot be added without
        // this test having an opinion about it.
        for phase in [
            "waiting to connect Wi-Fi",
            "boot validation",
            "waiting for MQTT",
            "retrying MQTT boot validation",
            "checking for OTA update",
            "homing",
            "settling after homing",
            "re-homing",
            "encoder recovery",
            "tracking move",
            "ready",
        ] {
            assert_cannot_resolve_a_command(&phase_line(phase));
        }
    }

    #[test]
    fn ping_is_answered_through_the_standard_handler() {
        let mut transport = MockTransport::from_input("PING\r\n");
        let mut runtime = DiagnosticRuntime::new();
        let mut handler = StandardDiagnosticHandler::new(
            MockBoard,
            MockEnvironment::default(),
            StaticDiagnosticConfiguration::new("1.1.3"),
        );

        let poll = runtime.poll(&mut transport, &mut handler).unwrap();

        assert_eq!(poll, DiagnosticPoll::CommandProcessed);
        assert_eq!(transport.writes, vec!["CMD_RECEIVED", "PONG"]);
    }

    /// The 15 keys `buildEnvironmentEntries` sends in the installer's
    /// `customerConfig.ts`. If that list changes, this test is what catches it —
    /// otherwise the board answers `ERROR unknown configuration key` for a key the
    /// installer believes it provisioned.
    const INSTALLER_KEYS: &[&str] = &[
        "device.tower_id",
        "wifi.ssid",
        "wifi.password",
        "location.latitude",
        "location.longitude",
        "location.altitude",
        "location.timezone_offset_hours",
        "mqtt.broker",
        "mqtt.port",
        "mqtt.username",
        "mqtt.password",
        "mqtt.topic",
        "customer.full_name",
        "customer.address",
        "customer.phone",
    ];

    #[test]
    fn config_table_covers_exactly_what_the_installer_sends() {
        for key in INSTALLER_KEYS {
            assert!(config_key(key).is_some(), "installer sends {key} but the board rejects it");
        }
        assert_eq!(
            CONFIG_KEYS.len(),
            INSTALLER_KEYS.len(),
            "the board provisions a key the installer never sends, or vice versa"
        );
        assert!(config_key("device.nonsense").is_none());
    }

    #[test]
    fn config_table_is_storable_and_unambiguous() {
        for entry in CONFIG_KEYS {
            // The constraint that forced the table to exist in the first place.
            assert!(
                entry.nvs.len() <= 15,
                "NVS key {} is {} bytes; the limit is 15",
                entry.nvs,
                entry.nvs.len()
            );
            assert!(entry.protocol.contains('.'), "{} has no section", entry.protocol);

            let duplicates = CONFIG_KEYS
                .iter()
                .filter(|other| other.nvs == entry.nvs || other.protocol == entry.protocol)
                .count();
            assert_eq!(duplicates, 1, "{} is duplicated in the table", entry.protocol);
        }

        let tower = config_key("device.tower_id").unwrap();
        assert_eq!(tower.section(), "device");
        assert_eq!(tower.name(), "tower_id");
        let tz = config_key("location.timezone_offset_hours").unwrap();
        assert_eq!(tz.name(), "timezone_offset_hours");
    }

    #[test]
    fn config_table_groups_each_section_contiguously() {
        // GET_CONFIG walks this table in order and opens a new `[section]` each
        // time the section changes, so a key out of place would emit a duplicate
        // header and the installer's parser would keep only the later values.
        let mut seen: Vec<&str> = Vec::new();
        let mut current = "";
        for entry in CONFIG_KEYS {
            if entry.section() != current {
                assert!(
                    !seen.contains(&entry.section()),
                    "section {} is split across the table",
                    entry.section()
                );
                seen.push(entry.section());
                current = entry.section();
            }
        }
        assert_eq!(seen, vec!["device", "wifi", "location", "mqtt", "customer"]);
    }

    #[test]
    fn config_values_are_validated_against_their_kind() {
        let check = |protocol: &str, value: &str| {
            validate_config_value(config_key(protocol).unwrap(), value)
        };

        assert_eq!(check("location.latitude", "40.5"), Ok(()));
        assert_eq!(check("location.latitude", "-90"), Ok(()));
        assert_eq!(check("location.latitude", "90.1"), Err(ConfigError::OutOfRange));
        assert_eq!(check("location.latitude", "north"), Err(ConfigError::NotANumber));
        assert_eq!(check("location.longitude", "-180"), Ok(()));
        assert_eq!(check("location.longitude", "180.5"), Err(ConfigError::OutOfRange));
        assert_eq!(check("location.timezone_offset_hours", "-6"), Ok(()));
        assert_eq!(check("location.timezone_offset_hours", "15"), Err(ConfigError::OutOfRange));
        assert_eq!(check("location.timezone_offset_hours", "1.5"), Err(ConfigError::NotANumber));
        assert_eq!(check("mqtt.port", "8883"), Ok(()));
        assert_eq!(check("mqtt.port", "0"), Err(ConfigError::OutOfRange));
        assert_eq!(check("mqtt.port", "70000"), Err(ConfigError::OutOfRange));

        // Text is unconstrained apart from length.
        assert_eq!(check("customer.address", "12 Any Street, Apt 4"), Ok(()));
        let too_long = "x".repeat(CONFIG_VALUE_MAX_BYTES + 1);
        assert_eq!(check("customer.address", &too_long), Err(ConfigError::TooLong));

        // A placeholder read back from the board must never be stored as a secret;
        // the installer omits untouched secrets rather than echoing these.
        assert_eq!(check("wifi.password", SECRET_SET), Err(ConfigError::SecretPlaceholder));
        assert_eq!(check("mqtt.password", SECRET_UNSET), Err(ConfigError::SecretPlaceholder));
        // The same literal is an ordinary value for a field that is not a secret.
        assert_eq!(check("customer.full_name", SECRET_SET), Ok(()));
    }

    #[test]
    fn staging_replaces_by_key_and_empties_on_take() {
        let mut staging = ConfigStaging::new();
        let ssid = config_key("wifi.ssid").unwrap();
        let tower = config_key("device.tower_id").unwrap();

        assert!(staging.is_empty());
        staging.stage(ssid, String::from("first"));
        staging.stage(tower, String::from("tower-7"));
        staging.stage(ssid, String::from("second"));

        assert_eq!(staging.len(), 2, "restaging a key must replace, not append");
        assert_eq!(staging.get("wifi.ssid"), Some("second"));
        assert_eq!(staging.get("mqtt.topic"), None);

        assert_eq!(staging.entries().len(), 2);
        staging.clear();
        assert!(staging.is_empty(), "a committed SAVE_CONFIG must not leave values behind");
    }

    #[test]
    fn secret_placeholders_report_presence_without_the_value() {
        assert_eq!(secret_placeholder(true), "<set>");
        assert_eq!(secret_placeholder(false), "<unset>");
        assert!(is_secret_placeholder(SECRET_SET));
        assert!(is_secret_placeholder(SECRET_UNSET));
        assert!(!is_secret_placeholder("hunter2"));
    }

    #[test]
    fn parses_unknown_command() {
        assert_eq!(
            DiagnosticCommand::parse("FLY_AWAY 123"),
            DiagnosticCommand::Unknown {
                command: "FLY_AWAY".to_string(),
                original: "FLY_AWAY 123".to_string(),
            }
        );

        // Argument-free commands ignore trailing text rather than falling through
        // to Unknown, the same way GO_HOME and RTC_CHECK do.
        assert_eq!(DiagnosticCommand::parse("PING 123"), DiagnosticCommand::Ping);
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
                // The handshake precedes the line the host resolves on.
                "PROTOCOL_VERSION 1",
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
