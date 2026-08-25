use board_diagnostics::{
    ConfigError, ConfigKey, ConfigStaging, DiagnosticBoard, DiagnosticCommand, DiagnosticConfigSection,
    DiagnosticControl, DiagnosticEnvironment, DiagnosticHandler, DiagnosticIo,
    StandardDiagnosticError, StandardDiagnosticHandler, StaticDiagnosticConfiguration,
};
use chrono::Local;
use std::time::{Duration, Instant};
use esp_idf_svc::{
    hal::{delay::Ets, i2c::I2cDriver},
    nvs::{EspNvs, NvsPartitionId},
};
use hdc1080::Hdc1080;
use motion::{Motion, MotionMode, MoveTick};

use crate::{app::encoder_fault::Direction, infra};

/// Devices expected on the shared I2C bus, so a scan can name what it found and
/// say plainly what is missing.
const EXPECTED_I2C_DEVICES: &[(u8, &str)] = &[
    (0x3C, "SSD1306 OLED"),
    (0x40, "HDC1080"),
    (0x68, "DS3231"),
];

/// Identity registers the HDC1080 reports. A device answering anything else is not
/// the sensor we think we are reading, and its measurements mean nothing.
const HDC1080_DEVICE_ID: u16 = 0x1050;
const HDC1080_MANUFACTURER_ID: u16 = 0x5449;

/// How long each pattern is held during a full `OLED_TEST` walkthrough.
///
/// Long enough to register, short enough that the whole cycle stays inside the
/// host's default silence budget — serial is not serviced while this runs.
const OLED_PATTERN_HOLD: Duration = Duration::from_millis(700);

/// How often `GO_HOME` reports progress while searching for the limit switch.
///
/// The search runs silent for up to tens of minutes otherwise, which no host-side
/// timeout can distinguish from a wedged board. Five seconds is frequent enough
/// for the installer to keep a short silence budget and still not flood the link.
const HOMING_PROGRESS_INTERVAL: Duration = Duration::from_secs(5);

pub const FIRST_WAVE_CAPABILITIES: &[&str] = &[
    "MQTT",
    "SERIAL",
    "STATUS",
    "RTC",
    "HDC1080",
    "GO_HOME",
    "REBOOT",
    "PING",
    "CONFIG",
    "I2C_SCAN",
    "LED_TEST",
    "OLED",
];

#[derive(Clone, Debug, Default)]
pub struct DiagnosticsTranscript {
    pub status: &'static str,
    pub message: String,
    /// Output captured for a caller that reports after the fact.
    ///
    /// `execute_first_wave_command` leaves this empty — it writes to the caller's
    /// `DiagnosticIo` as the command produces output, so a long-running command's
    /// progress reaches the host while it is still running. The MQTT path, which
    /// publishes one message at the end, passes a [`TranscriptIo`] and copies its
    /// lines in here.
    pub lines: Vec<String>,
    /// Set when the command asked the board to restart. Neither the executor nor
    /// a transport may reset — only the main loop can, once the response has
    /// cleared the wire — so the request rides out with the transcript.
    pub reboot_requested: bool,
    /// Structured key/value results, rendered into the terminal `RESULT` line.
    ///
    /// The point is to give the host data rather than prose to scrape: a reading
    /// of `temp=24.50 hum=55.50` can be displayed and compared, where
    /// `HDC1080 Correct read: temp:24.50 hum:55.50` can only be matched.
    pub details: Vec<(String, String)>,
}

#[derive(Debug)]
enum RuntimeBoardError {
    Hdc1080(String),
    OledTest(String),
    LedTest(String),
    I2cScan(String),
    RtcCheck(String),
    GoHome(String),
    Unsupported(&'static str),
}

#[derive(Debug)]
enum RuntimeEnvironmentError {
    Unsupported(&'static str),
}

/// A [`DiagnosticIo`] that collects instead of transmitting.
///
/// For callers that report once at the end — the MQTT path publishes a single
/// transcript — rather than streaming, which is what the serial console does.
#[derive(Default)]
pub struct TranscriptIo {
    lines: Vec<String>,
}

/// Everything a motion command needs, bundled so it can be absent as a unit.
///
/// Absent for most of boot, and absent again *during* a move: the move itself holds
/// the motor. The status LED is deliberately not in here — it stays lendable
/// throughout, which is what lets `LED_TEST` run while the tower is homing.
pub struct MotionContext<'ctx, 'motion> {
    pub motion: &'ctx mut Motion<'motion>,
    pub home_heading_deg: f32,
    pub motion_mode: MotionMode,
    pub actual_heading: &'ctx mut f32,
    pub persist_nvs: bool,
    pub need_rehome_stepper_only: &'ctx mut bool,
}

struct RuntimeDiagnosticBoard<'ctx, 'ctm, 'motion, 'led, T>
where
    T: NvsPartitionId,
{
    /// `None` before the motion stack exists, and while a move holds it.
    motion: Option<&'ctx mut MotionContext<'ctm, 'motion>>,
    /// `None` only before the LED exists. Unlike motion, a move does not take it.
    led: Option<&'ctx mut rgb_led::Led<'led>>,
    /// `None` when the caller holds it — a tracking pass and encoder recovery
    /// both take `&mut EspNvs` for their own bookkeeping while they run. Only
    /// `go_home` reads it on this path, and `go_home` needs the motor anyway.
    nvs: Option<&'ctx mut EspNvs<T>>,
    bus: &'static shared_bus::BusManagerStd<I2cDriver<'static>>,
    /// May a command hold this thread for seconds rather than milliseconds?
    ///
    /// False only during a position-critical move; see
    /// [`board_diagnostics::MoveTolerance`]. Everywhere else — boot, idle, a
    /// homing search — there is nothing to lose by taking the time.
    allow_long_blocking: bool,
    /// Filled by whichever board method ran; read back through
    /// `StandardDiagnosticHandler::board` once the command completes.
    details: Vec<(String, String)>,
}

impl<T> RuntimeDiagnosticBoard<'_, '_, '_, '_, T>
where
    T: NvsPartitionId,
{
    fn detail(&mut self, key: &str, value: impl Into<String>) {
        self.details.push((key.to_string(), value.into()));
    }
}

struct UnsupportedEnvironment;

impl TranscriptIo {
    pub fn into_lines(self) -> Vec<String> {
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

impl<T> DiagnosticBoard for RuntimeDiagnosticBoard<'_, '_, '_, '_, T>
where
    T: NvsPartitionId,
{
    type Error = RuntimeBoardError;

    fn hdc1080_read<IO: DiagnosticIo>(&mut self, io: &mut IO) -> Result<(), Self::Error> {
        let mut sensor = Hdc1080::new(self.bus.acquire_i2c(), Ets)
            .map_err(|err| hdc1080_failure(io, format!("HDC1080 init failed: {:?}", err)))?;

        // Writes the configuration the read path assumes. Without it the device
        // stays in its power-on mode, where a four-byte sequential read is not
        // valid — so this is not merely tidiness.
        sensor
            .init()
            .map_err(|err| hdc1080_failure(io, format!("HDC1080 init failed: {:?}", err)))?;

        let device_id = sensor
            .get_device_id()
            .map_err(|err| hdc1080_failure(io, format!("HDC1080 get_device_id failed: {:?}", err)))?;
        let _ = io.write_line(&format!("HDC1080 device_id: 0x{device_id:04X}"));
        if device_id != HDC1080_DEVICE_ID {
            return Err(hdc1080_failure(
                io,
                format!(
                    "HDC1080 wrong device_id: read 0x{device_id:04X}, expected 0x{HDC1080_DEVICE_ID:04X}"
                ),
            ));
        }

        let manufacturer_id = sensor
            .get_man_id()
            .map_err(|err| hdc1080_failure(io, format!("HDC1080 get_manufacturer_id failed: {:?}", err)))?;
        let _ = io.write_line(&format!(
            "HDC1080 manufacturer_id: 0x{manufacturer_id:04X}"
        ));
        if manufacturer_id != HDC1080_MANUFACTURER_ID {
            return Err(hdc1080_failure(
                io,
                format!(
                    "HDC1080 wrong manufacturer_id: read 0x{manufacturer_id:04X}, expected 0x{HDC1080_MANUFACTURER_ID:04X}"
                ),
            ));
        }

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
        // Legacy terminal line for HDC1080_READ, kept while hosts that predate the
        // structured protocol are still in the field. The installer matches this
        // exact shape; see `docs/serial-protocol.md` before changing it.
        let _ = io.write_line(&format!(
            "HDC1080 Correct read: temp:{temp_c:.2} hum:{humidity:.2}"
        ));
        self.detail("temp", format!("{temp_c:.2}"));
        self.detail("hum", format!("{humidity:.2}"));
        Ok(())
    }

    fn rtc_check<IO: DiagnosticIo>(&mut self, io: &mut IO) -> Result<(), Self::Error> {
        // Reads the DS3231 itself. This used to report `Local::now()`, which tests
        // chrono rather than the hardware: it passes happily on a board whose RTC
        // has lost its time — the exact fault this check exists to catch, and the
        // one that stops the tower booting.
        let mut rtc = rtc::Rtc::new(self.bus);
        let Some(rtc_time) = rtc.read() else {
            let message = "RTC_CHECK failed: DS3231 did not return a time";
            let _ = io.write_line(&format!("ERROR {message}"));
            return Err(RuntimeBoardError::RtcCheck(message.to_string()));
        };

        let stamp = rtc_time.format("%Y-%m-%d %H:%M:%S").to_string();

        // Checked before the legacy `TIME:` line is written, not after. A host that
        // predates the structured protocol resolves on that line, so emitting it
        // first would report a pass for a clock we are about to call invalid.
        if !rtc::Rtc::is_sane(&rtc_time) {
            let message = format!(
                "RTC_CHECK failed: DS3231 reads {stamp}, outside the plausible range — battery or never set"
            );
            let _ = io.write_line(&format!("ERROR {message}"));
            self.details.push((String::from("rtc_utc"), stamp));
            return Err(RuntimeBoardError::RtcCheck(message));
        }

        let _ = io.write_line(&format!("TIME: {stamp}"));
        self.detail("rtc_utc", stamp);
        self.detail(
            "system_local",
            Local::now().format("%Y-%m-%d %H:%M:%S").to_string(),
        );
        Ok(())
    }

    fn led_test<IO: DiagnosticIo>(&mut self, color: &str, io: &mut IO) -> Result<(), Self::Error> {
        let Some(led) = self.led.as_deref_mut() else {
            let message = "LED_TEST unavailable: the status LED is not initialised yet";
            let _ = io.write_line(&format!("ERROR {message}"));
            return Err(RuntimeBoardError::LedTest(message.to_string()));
        };

        match drive_status_led(led, color, io) {
            Ok(details) => {
                self.details.extend(details);
                Ok(())
            }
            Err(message) => Err(RuntimeBoardError::LedTest(message)),
        }
    }

    fn i2c_scan<IO: DiagnosticIo>(&mut self, io: &mut IO) -> Result<(), Self::Error> {
        use embedded_hal::i2c::I2c;

        let mut bus = self.bus.acquire_i2c();
        let mut found: Vec<&str> = Vec::new();
        let mut missing: Vec<&str> = Vec::new();

        // Only the addresses this board actually carries, not a sweep of the whole
        // range. Every probe of an absent device costs a full bus timeout, so 112
        // of them on a dead bus runs for minutes and blocks the main loop the whole
        // time — well past any sane host budget. Two probes answer the question this
        // command exists for: is the bus alive, or is one device missing?
        for (address, name) in EXPECTED_I2C_DEVICES {
            // A zero-length write addresses the device and checks for an ACK without
            // transferring anything — the standard probe, and the safe one. Reading a
            // byte instead can disturb devices that treat a read as a pointer advance.
            if bus.write(*address, &[]).is_ok() {
                let _ = io.write_line(&format!("I2C device 0x{address:02X}: {name}"));
                found.push(name);
            } else {
                let _ = io.write_line(&format!("I2C no response 0x{address:02X}: {name}"));
                missing.push(name);
            }
        }

        if found.is_empty() {
            let message = "I2C_SCAN: no device answered, the bus itself is not responding";
            let _ = io.write_line(&format!("ERROR {message}"));
            return Err(RuntimeBoardError::I2cScan(message.to_string()));
        }

        self.detail("found", found.join(","));
        if !missing.is_empty() {
            self.detail("missing", missing.join(","));
        }

        // Devices answered, so the bus is healthy — report that as a pass even when
        // one of ours is absent. Which sensor is missing is the detail; whether the
        // bus works is the question this command exists to settle.
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
        // Named precisely. Blaming whatever the tower happens to be waiting for
        // sends a technician looking at Wi-Fi when the answer is that the motion
        // stack has not been built yet.
        let Some(context) = self.motion.as_deref_mut() else {
            let message = "GO_HOME unavailable: motion is not initialised yet";
            let _ = io.write_line(&format!("ERROR {message}"));
            return Err(RuntimeBoardError::GoHome(message.to_string()));
        };

        let _ = io.write_line("MOVING_HOME");

        // Report while the search runs. `io` streams straight to the transport, so
        // these reach the host as they happen; the motion crate ticks about ten
        // times a second, which is far more often than the link needs.
        let mut last_report = Instant::now();
        let mut on_tick = |tick: MoveTick| {
            if last_report.elapsed() >= HOMING_PROGRESS_INTERVAL {
                last_report = Instant::now();
                let _ = io.write_line(&board_diagnostics::homing_progress_line(
                    tick.steps_remaining,
                    tick.encoder_ticks,
                ));
            }
        };

        let limit_sw_status = match Direction::Ccw {
            Direction::Cw => context.motion.find_limit_switch_cw_watched(&mut on_tick),
            Direction::Ccw => context.motion.find_limit_switch_ccw_watched(&mut on_tick),
        };
        drop(on_tick);

        if !limit_sw_status {
            let message =
                "ERROR Diagnostics go_home failed: limit switch was not found while moving ccw";
            let _ = io.write_line(message);
            return Err(RuntimeBoardError::GoHome(message.to_string()));
        }

        *context.actual_heading = context.home_heading_deg;
        context.motion.update_position(*context.actual_heading);

        if context.persist_nvs {
            match self.nvs.as_deref_mut() {
                Some(nvs) => {
                    infra::SnapshotStore::new(nvs, true).save_heading(*context.actual_heading);
                    if context.motion_mode == MotionMode::EncoderGuarded {
                        infra::SnapshotStore::new(nvs, true)
                            .save_encoder_snapshot(context.motion.encoder_ticks_adjusted());
                    }
                }
                // Unreachable today: the only caller without NVS is the mid-move
                // path, which has no motion to lend either and so never gets
                // here. Said out loud rather than assumed, because a homing run
                // whose heading silently failed to persist is a tower that comes
                // back up believing the wrong direction.
                None => {
                    let _ = io.write_line(
                        "WARN GO_HOME found home but could not persist the heading: NVS is not available on this path",
                    );
                }
            }
        }

        *context.need_rehome_stepper_only = false;
        let _ = io.write_line("LIMIT");
        let heading = *context.actual_heading;
        // Pushed to the field directly: `detail` takes all of `self`, which the
        // motion borrow above still holds.
        self.details.push((String::from("heading"), format!("{heading:.2}")));
        Ok(())
    }

    fn oled_test<IO: DiagnosticIo>(
        &mut self,
        payload: &str,
        io: &mut IO,
    ) -> Result<(), Self::Error> {
        match drive_oled_pattern(self.bus, payload, io, self.allow_long_blocking) {
            Ok(details) => {
                self.details.extend(details);
                Ok(())
            }
            Err(message) => Err(RuntimeBoardError::OledTest(message)),
        }
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

/// Rejects everything.
///
/// Kept for the board path: configuration commands are routed elsewhere, so a
/// board-path handler should never see one, and saying so plainly is better than
/// pretending to succeed.
impl DiagnosticEnvironment for UnsupportedEnvironment {
    type Error = RuntimeEnvironmentError;

    fn set_env<IO: DiagnosticIo>(
        &mut self,
        _key: &str,
        _value: &str,
        io: &mut IO,
    ) -> Result<(), Self::Error> {
        let _ = io.write_line("ERROR SET_ENV is not available on this path");
        Err(RuntimeEnvironmentError::Unsupported("SET_ENV"))
    }

    fn get_env<IO: DiagnosticIo>(
        &mut self,
        _key: &str,
        io: &mut IO,
    ) -> Result<Option<String>, Self::Error> {
        let _ = io.write_line("ERROR GET_ENV is not available on this path");
        Err(RuntimeEnvironmentError::Unsupported("GET_ENV"))
    }

    fn save_config<IO: DiagnosticIo>(&mut self, io: &mut IO) -> Result<(), Self::Error> {
        let _ = io.write_line("ERROR SAVE_CONFIG is not available on this path");
        Err(RuntimeEnvironmentError::Unsupported("SAVE_CONFIG"))
    }

    fn write_env_file<IO: DiagnosticIo>(&mut self, io: &mut IO) -> Result<(), Self::Error> {
        let _ = io.write_line("ERROR WRITE_ENV_FILE is not supported on this device");
        Err(RuntimeEnvironmentError::Unsupported("WRITE_ENV_FILE"))
    }
}

/// Reads and writes provisioned settings in NVS, staging until `SAVE_CONFIG`.
struct NvsEnvironment<'ctx, T>
where
    T: NvsPartitionId,
{
    nvs: &'ctx mut EspNvs<T>,
    staging: &'ctx mut ConfigStaging,
}

/// The value a read should report: staged if a `SET_ENV` is waiting, otherwise
/// what is committed. Secrets report only whether they are set — echoing one would
/// let anyone with a USB cable lift the site's Wi-Fi password.
fn read_config_value<T>(
    nvs: &mut EspNvs<T>,
    staging: &ConfigStaging,
    entry: &ConfigKey,
) -> String
where
    T: NvsPartitionId,
{
    if entry.secret {
        let is_set = staging.get(entry.protocol).is_some()
            || read_stored_value(nvs, entry).is_some_and(|value| !value.is_empty());
        return board_diagnostics::secret_placeholder(is_set).to_string();
    }
    if let Some(staged) = staging.get(entry.protocol) {
        return staged.to_string();
    }
    read_stored_value(nvs, entry).unwrap_or_default()
}

fn read_stored_value<T>(nvs: &mut EspNvs<T>, entry: &ConfigKey) -> Option<String>
where
    T: NvsPartitionId,
{
    // Every provisioned value is stored as a string, including latitude and
    // longitude — that is how the existing runtime readers already parse them.
    let mut buffer = [0u8; board_diagnostics::CONFIG_VALUE_MAX_BYTES + 1];
    nvs.get_str(entry.nvs, &mut buffer)
        .ok()
        .flatten()
        .map(|value| value.to_string())
}

impl<T> DiagnosticEnvironment for NvsEnvironment<'_, T>
where
    T: NvsPartitionId,
{
    type Error = ConfigError;

    fn set_env<IO: DiagnosticIo>(
        &mut self,
        key: &str,
        value: &str,
        io: &mut IO,
    ) -> Result<(), Self::Error> {
        let reject = |io: &mut IO, error: ConfigError| {
            let _ = io.write_line(&format!("ERROR SET_ENV {key}: {}", error.message()));
            error
        };

        let entry = board_diagnostics::config_key(key)
            .ok_or_else(|| reject(io, ConfigError::UnknownKey))?;
        board_diagnostics::validate_config_value(entry, value)
            .map_err(|error| reject(io, error))?;

        // Staged, not written. SAVE_CONFIG commits the set, so a sequence
        // interrupted halfway leaves the tower on its previous configuration
        // rather than with a new SSID and an old password.
        self.staging.stage(entry, value.to_string());
        Ok(())
    }

    fn get_env<IO: DiagnosticIo>(
        &mut self,
        key: &str,
        io: &mut IO,
    ) -> Result<Option<String>, Self::Error> {
        let Some(entry) = board_diagnostics::config_key(key) else {
            let _ = io.write_line(&format!(
                "ERROR GET_ENV {key}: {}",
                ConfigError::UnknownKey.message()
            ));
            return Err(ConfigError::UnknownKey);
        };
        Ok(Some(read_config_value(self.nvs, self.staging, entry)))
    }

    fn save_config<IO: DiagnosticIo>(&mut self, io: &mut IO) -> Result<(), Self::Error> {
        for (entry, value) in self.staging.entries() {
            if self.nvs.set_str(entry.nvs, value).is_err() {
                // The staging buffer is left intact so the installer can retry the
                // commit without resending every value.
                let _ = io.write_line(&format!(
                    "ERROR SAVE_CONFIG {}: {}",
                    entry.protocol,
                    ConfigError::WriteFailed.message()
                ));
                return Err(ConfigError::WriteFailed);
            }
        }

        // Boot seeds defaults over these keys until this flag is set, so without it
        // the next power cycle would silently discard everything just written.
        if self
            .nvs
            .set_u8(board_diagnostics::NVS_KEY_PROVISIONED, 1)
            .is_err()
        {
            let _ = io.write_line("ERROR SAVE_CONFIG could not mark the tower provisioned");
            return Err(ConfigError::WriteFailed);
        }

        self.staging.clear();
        Ok(())
    }

    fn write_env_file<IO: DiagnosticIo>(&mut self, io: &mut IO) -> Result<(), Self::Error> {
        let _ = io.write_line("ERROR WRITE_ENV_FILE is not supported on this device");
        Err(ConfigError::Unsupported)
    }
}

/// A board that refuses everything.
///
/// Configuration commands never touch the board, and the runtime board holds the
/// single `&mut EspNvs` that the environment also needs. Handling configuration
/// with this in the board slot keeps that handle from having to be in two places
/// at once.
struct UnsupportedBoard;

impl DiagnosticBoard for UnsupportedBoard {
    type Error = RuntimeBoardError;

    fn hdc1080_read<IO: DiagnosticIo>(&mut self, _io: &mut IO) -> Result<(), Self::Error> {
        Err(RuntimeBoardError::Unsupported("HDC1080_READ"))
    }

    fn rtc_check<IO: DiagnosticIo>(&mut self, _io: &mut IO) -> Result<(), Self::Error> {
        Err(RuntimeBoardError::Unsupported("RTC_CHECK"))
    }

    fn i2c_scan<IO: DiagnosticIo>(&mut self, _io: &mut IO) -> Result<(), Self::Error> {
        Err(RuntimeBoardError::Unsupported("I2C_SCAN"))
    }

    fn led_test<IO: DiagnosticIo>(&mut self, _color: &str, _io: &mut IO) -> Result<(), Self::Error> {
        Err(RuntimeBoardError::Unsupported("LED_TEST"))
    }

    fn motor_move<IO: DiagnosticIo>(
        &mut self,
        _argument: &str,
        _io: &mut IO,
    ) -> Result<(), Self::Error> {
        Err(RuntimeBoardError::Unsupported("MOTOR_MOVE"))
    }

    fn go_home<IO: DiagnosticIo>(&mut self, _io: &mut IO) -> Result<(), Self::Error> {
        Err(RuntimeBoardError::Unsupported("GO_HOME"))
    }

    fn oled_test<IO: DiagnosticIo>(
        &mut self,
        _payload: &str,
        _io: &mut IO,
    ) -> Result<(), Self::Error> {
        Err(RuntimeBoardError::Unsupported("OLED_TEST"))
    }

    fn relay_motor<IO: DiagnosticIo>(
        &mut self,
        _state: &str,
        _io: &mut IO,
    ) -> Result<(), Self::Error> {
        Err(RuntimeBoardError::Unsupported("RELAY_MOTOR"))
    }

    fn relay_hotspot<IO: DiagnosticIo>(
        &mut self,
        _state: &str,
        _io: &mut IO,
    ) -> Result<(), Self::Error> {
        Err(RuntimeBoardError::Unsupported("RELAY_HOTSPOT"))
    }
}

/// Draw a display pattern, shared by the full diagnostics path and the mid-move one.
///
/// The I2C bus is a `&'static` manager that locks per transaction, and motion does
/// not touch it — so this is reachable while the tower is moving, in the same way
/// the status LED is.
///
/// It is far from free, though. The bus runs at 10 kHz, where a full-screen write is
/// roughly a second, and during a move that second is one the stepping loop — and
/// so the stall and overshoot detectors that live inside it — is not running.
/// `allow_full_walk` is false during a position-critical move for that reason: a
/// caller must name one pattern rather than asking for all five. A homing search
/// sets it, because homing switches both detectors off anyway, so there is no
/// supervision left to interrupt.
///
/// Returns the details for a `RESULT` on success, or the failure message.
pub fn drive_oled_pattern<IO: DiagnosticIo>(
    bus: &'static shared_bus::BusManagerStd<I2cDriver<'static>>,
    payload: &str,
    io: &mut IO,
    allow_full_walk: bool,
) -> Result<Vec<(String, String)>, String> {
    let mut panel = ssd1306_oled::Ssd1306::new(bus.acquire_i2c());

    // Checked before anything is drawn. Unlike the status LED this part answers, so
    // a missing panel is a real failure the board can report by itself rather than
    // something only a person looking at it could notice.
    if !panel.present() {
        let message = format!(
            "OLED_TEST: no response at 0x{:02X}",
            ssd1306_oled::DEFAULT_ADDRESS
        );
        let _ = io.write_line(&format!("ERROR {message}"));
        return Err(message);
    }

    if let Err(err) = panel.init() {
        let message = format!("OLED_TEST: initialisation failed: {err:?}");
        let _ = io.write_line(&format!("ERROR {message}"));
        return Err(message);
    }

    let requested = payload.trim();
    let patterns: Vec<ssd1306_oled::Pattern> = if requested.is_empty() {
        if !allow_full_walk {
            let message = String::from(
                "OLED_TEST: name one pattern during a tracking move; walking all five would leave stall detection unserviced for about five seconds",
            );
            let _ = io.write_line(&format!("ERROR {message}"));
            return Err(message);
        }
        ssd1306_oled::Pattern::ALL
            .iter()
            .map(|(_, pattern)| *pattern)
            .collect()
    } else {
        match ssd1306_oled::Pattern::from_name(requested) {
            Some(pattern) => vec![pattern],
            None => {
                let names: Vec<&str> = ssd1306_oled::Pattern::ALL
                    .iter()
                    .map(|(name, _)| *name)
                    .collect();
                let message = format!(
                    "OLED_TEST: unknown pattern '{}', expected one of {}",
                    requested,
                    names.join(",")
                );
                let _ = io.write_line(&format!("ERROR {message}"));
                return Err(message);
            }
        }
    };

    let walking = patterns.len() > 1;
    for (index, pattern) in patterns.iter().enumerate() {
        // `Off` goes through `clear`, which also blanks the columns behind the glass.
        // Writing only the visible 128 leaves a strip holding whatever was drawn
        // there last, which is how a border survives being switched off.
        let outcome = if *pattern == ssd1306_oled::Pattern::Off {
            panel.clear()
        } else {
            panel.write_pattern(*pattern)
        };
        if let Err(err) = outcome {
            let message = format!("OLED_TEST: writing {} failed: {err:?}", pattern.name());
            let _ = io.write_line(&format!("ERROR {message}"));
            return Err(message);
        }
        let _ = io.write_line(&format!("OLED pattern {}", pattern.name()));
        if walking && index + 1 < patterns.len() {
            std::thread::sleep(OLED_PATTERN_HOLD);
        }
    }

    // Legacy terminal line, for a host that predates the structured protocol.
    let _ = io.write_line("OK");
    Ok(vec![
        (String::from("patterns"), patterns.len().to_string()),
        (
            String::from("shown"),
            patterns
                .iter()
                .map(|pattern| pattern.name())
                .collect::<Vec<_>>()
                .join(","),
        ),
    ])
}

/// Drive the status LED, shared by the full diagnostics path and the mid-move one.
///
/// The LED is the one piece of hardware that stays lendable while the tower moves —
/// the move holds the motor, the caller holds NVS, but nothing holds this. Both
/// callers route through here so they write the same colours and report the same
/// details.
///
/// Returns the details for a `RESULT` on success, or the failure message.
pub fn drive_status_led<IO: DiagnosticIo>(
    led: &mut rgb_led::Led<'_>,
    color: &str,
    io: &mut IO,
) -> Result<Vec<(String, String)>, String> {
    // `RESTORE` puts the LED back to what the runtime shows, so a test does not
    // leave the board dark or the wrong colour once the technician walks away.
    if color.eq_ignore_ascii_case("RESTORE") {
        led.display_healthy();
        let _ = io.write_line("LED restored to the runtime status colour");
        return Ok(vec![(String::from("color"), String::from("RESTORE"))]);
    }

    let Some((red, green, blue)) = board_diagnostics::led_test_color(color) else {
        let message = format!(
            "LED_TEST: unknown colour '{}', expected one of {}, RESTORE, or an R,G,B triplet",
            color,
            board_diagnostics::led_test_color_names()
        );
        let _ = io.write_line(&format!("ERROR {message}"));
        return Err(message);
    };

    if let Err(err) = led.set_color(rgb_led::RGB8::new(red, green, blue)) {
        let message = format!("LED_TEST could not drive the LED: {err:?}");
        let _ = io.write_line(&format!("ERROR {message}"));
        return Err(message);
    }

    // Deliberately says the colour was written, not that the LED works. A WS2812
    // has no readback: a dead LED, a broken joint and a wrong pin all look
    // identical from here, so only the person watching can judge it.
    let _ = io.write_line(&format!("LED driven {color} rgb {red},{green},{blue}"));
    Ok(vec![
        (String::from("color"), color.to_uppercase()),
        (String::from("rgb"), format!("{red},{green},{blue}")),
    ])
}

pub fn capabilities_csv() -> String {
    FIRST_WAVE_CAPABILITIES.join(",")
}

pub fn firmware_version_line(firmware_version: &str) -> String {
    // Delegated so the boot-phase responder, the standard handler, and this MQTT
    // path all emit the one line the installer matches.
    board_diagnostics::firmware_version_line(firmware_version)
}

pub fn direct_first_wave_lines(cmd: &str, firmware_version: &str) -> Option<Vec<String>> {
    match cmd {
        "firmware_version" => Some(vec![firmware_version_line(firmware_version)]),
        "get_capabilities" => Some(vec![capabilities_csv()]),
        _ => None,
    }
}

pub fn command_name(command: &DiagnosticCommand) -> &'static str {
    // Delegated: the protocol crate owns the names that appear in RESULT lines.
    command.name()
}

pub fn supports_first_wave_command(command: &DiagnosticCommand) -> bool {
    matches!(
        command,
        DiagnosticCommand::Ping
            | DiagnosticCommand::I2cScan
            | DiagnosticCommand::LedTest { .. }
            | DiagnosticCommand::OledTest { .. }
            | DiagnosticCommand::FirmwareVersion
            | DiagnosticCommand::GetCapabilities
            | DiagnosticCommand::RtcCheck
            | DiagnosticCommand::Hdc1080Read
            | DiagnosticCommand::GoHome
            | DiagnosticCommand::Reboot
    ) || is_config_command(command)
}

/// Commands that read or write provisioned configuration.
///
/// Routed away from the board path because they need the `&mut EspNvs` the runtime
/// board also holds, and because they touch no hardware at all.
pub fn is_config_command(command: &DiagnosticCommand) -> bool {
    matches!(
        command,
        // `CONFIG_MODE` is here because what it reports lives in NVS, not on the
        // board: this firmware has no separate mode to enter, so the command is a
        // question about the provisioning path rather than an instruction to it.
        DiagnosticCommand::ConfigMode
            | DiagnosticCommand::SetEnv { .. }
            | DiagnosticCommand::InvalidSetEnv { .. }
            | DiagnosticCommand::GetEnv { .. }
            | DiagnosticCommand::SaveConfig
            | DiagnosticCommand::GetConfig
            | DiagnosticCommand::WriteEnvFile
    )
}

pub fn is_control_command(command: &DiagnosticCommand) -> bool {
    matches!(
        command,
        DiagnosticCommand::RtcCheck
            | DiagnosticCommand::Hdc1080Read
            | DiagnosticCommand::I2cScan
            | DiagnosticCommand::LedTest { .. }
            | DiagnosticCommand::OledTest { .. }
            | DiagnosticCommand::GoHome
            | DiagnosticCommand::Reboot
            // SET_ENV only stages into RAM; SAVE_CONFIG is the one that writes.
            | DiagnosticCommand::SaveConfig
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

/// Run a first-wave command, writing its output to `io` as it happens.
///
/// `io` is the caller's, not this function's: the serial transport passes a
/// writer that goes straight to the console, so `go_home`'s `MOVING_HOME` reaches
/// the technician when homing starts rather than after it finishes. Callers that
/// need the output afterwards pass a [`TranscriptIo`] and read it back.
/// Build the `GET_CONFIG` view of every provisioned setting, grouped by section.
///
/// Walks `CONFIG_KEYS` in order and opens a new `[section]` whenever the section
/// changes; `config_table_groups_each_section_contiguously` in the diagnostics
/// crate is what keeps that assumption true.
fn nvs_configuration<T>(
    firmware_version: &str,
    nvs: &mut EspNvs<T>,
    staging: &ConfigStaging,
) -> StaticDiagnosticConfiguration
where
    T: NvsPartitionId,
{
    let mut configuration = base_configuration(firmware_version);
    let mut section_name = "";
    let mut section = DiagnosticConfigSection::new("");

    for entry in board_diagnostics::CONFIG_KEYS {
        if entry.section() != section_name {
            if !section_name.is_empty() {
                configuration = configuration.with_section(section);
            }
            section_name = entry.section();
            section = DiagnosticConfigSection::new(section_name);
        }
        section = section.with_entry(entry.name(), read_config_value(nvs, staging, entry));
    }
    if !section_name.is_empty() {
        configuration = configuration.with_section(section);
    }

    configuration
}

/// Run a configuration command against NVS and the staging buffer.
fn execute_config_command<IO, T>(
    io: &mut IO,
    command: DiagnosticCommand,
    firmware_version: &str,
    nvs: &mut EspNvs<T>,
    staging: &mut ConfigStaging,
) -> DiagnosticsTranscript
where
    IO: DiagnosticIo,
    T: NvsPartitionId,
{
    if matches!(command, DiagnosticCommand::ConfigMode) {
        return execute_config_mode(io, nvs, staging);
    }

    // Built first, and it holds owned data — so the read borrow is finished before
    // the environment takes the same handle mutably.
    let configuration = nvs_configuration(firmware_version, nvs, staging);
    // Captured before the command runs: SAVE_CONFIG empties the buffer, and
    // GET_CONFIG's terminator is what tells the host the response is complete.
    let sections = configuration.section_count();
    let staged = staging.len();

    let environment = NvsEnvironment { nvs, staging };
    let mut handler = StandardDiagnosticHandler::new(UnsupportedBoard, environment, configuration);

    match handler.handle_command(command.clone(), io) {
        Ok(_) => DiagnosticsTranscript {
            status: "completed",
            message: success_message(&command),
            lines: Vec::new(),
            reboot_requested: false,
            details: match command {
                DiagnosticCommand::GetConfig => {
                    vec![(String::from("sections"), sections.to_string())]
                }
                DiagnosticCommand::SaveConfig => {
                    vec![(String::from("committed"), staged.to_string())]
                }
                _ => Vec::new(),
            },
        },
        // The environment writes its own `ERROR ...` line, naming the key and the
        // reason, before returning — see `NvsEnvironment::set_env`.
        Err(error) => DiagnosticsTranscript {
            status: "failed",
            message: config_failure_message(&command, &error),
            lines: Vec::new(),
            reboot_requested: false,
            details: Vec::new(),
        },
    }
}

/// Answer `CONFIG_MODE`: is the provisioning path usable, and what does it hold?
///
/// There is no mode to enter. `SET_ENV` and `SAVE_CONFIG` are accepted whenever
/// NVS is lendable, so a command that merely replied `OK` would be asserting that
/// rather than showing it — another diagnostic that cannot fail, which this file
/// has already produced twice. Instead it reads NVS, which fails when the
/// namespace cannot be opened: the fault that would otherwise stay hidden until a
/// `SAVE_CONFIG` fifteen commands later reported success and lost the lot.
///
/// The counts are the useful part. `stored` against `staged` tells a technician
/// whether the values they are looking at in the app are committed or still
/// waiting on a save.
fn execute_config_mode<IO, T>(
    io: &mut IO,
    nvs: &mut EspNvs<T>,
    staging: &ConfigStaging,
) -> DiagnosticsTranscript
where
    IO: DiagnosticIo,
    T: NvsPartitionId,
{
    let provisioned = match nvs.get_u8(board_diagnostics::NVS_KEY_PROVISIONED) {
        Ok(flag) => flag.unwrap_or(0) == 1,
        Err(err) => {
            let message = format!("CONFIG_MODE failed: NVS is not readable: {err}");
            let _ = io.write_line(&format!("ERROR {message}"));
            return DiagnosticsTranscript {
                status: "failed",
                message,
                ..Default::default()
            };
        }
    };

    // A plain loop rather than a filtered iterator: `read_stored_value` needs the
    // same `&mut` handle the iterator would still be holding.
    let mut stored = 0usize;
    for entry in board_diagnostics::CONFIG_KEYS {
        if read_stored_value(nvs, entry).is_some_and(|value| !value.is_empty()) {
            stored += 1;
        }
    }

    // Legacy terminal line, for a host that predates the structured protocol.
    // `docs/serial-protocol.md` records the three spellings the installer accepts.
    let _ = io.write_line("CONFIG_MODE OK");
    DiagnosticsTranscript {
        status: "completed",
        message: String::from("Configuration storage is readable"),
        details: vec![
            (
                String::from("provisioned"),
                String::from(if provisioned { "yes" } else { "no" }),
            ),
            (String::from("stored"), stored.to_string()),
            (String::from("staged"), staging.len().to_string()),
            (
                String::from("keys"),
                board_diagnostics::CONFIG_KEYS.len().to_string(),
            ),
        ],
        ..Default::default()
    }
}

fn config_failure_message(
    command: &DiagnosticCommand,
    error: &StandardDiagnosticError<RuntimeBoardError, ConfigError, ()>,
) -> String {
    match error {
        StandardDiagnosticError::Environment(err) => {
            format!("{} failed: {}", command_name(command), err.message())
        }
        StandardDiagnosticError::Board(_) => {
            format!("{} is not a configuration command", command_name(command))
        }
        StandardDiagnosticError::Config(()) => {
            format!("{} could not report configuration", command_name(command))
        }
    }
}

/// Capabilities derived from `FIRST_WAVE_CAPABILITIES` rather than repeated, so
/// what `GET_CAPABILITIES` reports and what a handler claims cannot drift apart.
fn base_configuration(firmware_version: &str) -> StaticDiagnosticConfiguration {
    FIRST_WAVE_CAPABILITIES.iter().fold(
        StaticDiagnosticConfiguration::new(firmware_version),
        |configuration, capability| configuration.with_capability(*capability),
    )
}

/// Run a diagnostics command against whatever the runtime currently owns.
///
/// `motion` is `None` before the motion stack exists, which is most of boot. The
/// sensors and the configuration set do not need it and are answered normally;
/// only `GO_HOME` refuses, and it says why.
pub fn execute_first_wave_command<IO, T>(
    io: &mut IO,
    command: DiagnosticCommand,
    firmware_version: &str,
    motion: Option<&mut MotionContext<'_, '_>>,
    led: Option<&mut rgb_led::Led<'_>>,
    nvs: Option<&mut EspNvs<T>>,
    staging: &mut ConfigStaging,
    bus: &'static shared_bus::BusManagerStd<I2cDriver<'static>>,
    allow_long_blocking: bool,
) -> DiagnosticsTranscript
where
    IO: DiagnosticIo,
    T: NvsPartitionId,
{
    let mut nvs = nvs;

    if is_config_command(&command) {
        let Some(nvs) = nvs.as_deref_mut() else {
            // Named precisely, and never as `BUSY`. The tracking pass holds NVS
            // for its own bookkeeping; a technician who reads "busy" waits for the
            // move to end, where "not lendable during a tracking move" tells them
            // the same command works during the homing search they will see next.
            let message = format!(
                "{} unavailable: configuration storage is not lendable during a tracking move",
                command_name(&command)
            );
            let _ = io.write_line(&format!("ERROR {message}"));
            return DiagnosticsTranscript {
                status: "failed",
                message,
                ..Default::default()
            };
        };
        return execute_config_command(io, command, firmware_version, nvs, staging);
    }

    if !supports_first_wave_command(&command) {
        let message = format!(
            "{} is not enabled in first-wave diagnostics",
            command_name(&command)
        );
        let _ = io.write_line(&format!("ERROR {message}"));
        return DiagnosticsTranscript {
            status: "failed",
            message,
            lines: Vec::new(),
            reboot_requested: false,
            details: Vec::new(),
        };
    }

    let configuration = base_configuration(firmware_version);
    let board = RuntimeDiagnosticBoard {
        motion,
        led,
        nvs,
        bus,
        allow_long_blocking,
        details: Vec::new(),
    };
    let environment = UnsupportedEnvironment;
    let mut handler = StandardDiagnosticHandler::new(board, environment, configuration);

    let outcome = handler.handle_command(command.clone(), io);
    // Read back after the command: board methods record what they measured, and the
    // handler owns the board until now.
    let details = handler.board().details.clone();

    match outcome {
        Ok(DiagnosticControl::Continue) => DiagnosticsTranscript {
            status: "completed",
            message: success_message(&command),
            lines: Vec::new(),
            reboot_requested: false,
            details,
        },
        Ok(DiagnosticControl::RebootRequested) => DiagnosticsTranscript {
            status: "completed",
            message: String::from("Diagnostics command requested a reboot"),
            lines: Vec::new(),
            reboot_requested: true,
            details,
        },
        Err(err) => DiagnosticsTranscript {
            status: "failed",
            message: failure_message(&command, &err),
            lines: Vec::new(),
            reboot_requested: false,
            details,
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
        | StandardDiagnosticError::Board(RuntimeBoardError::OledTest(message))
        | StandardDiagnosticError::Board(RuntimeBoardError::LedTest(message))
        | StandardDiagnosticError::Board(RuntimeBoardError::I2cScan(message))
        | StandardDiagnosticError::Board(RuntimeBoardError::RtcCheck(message))
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
