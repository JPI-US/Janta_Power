//! Commands this machine answers itself, without asking another.
//!
//! Two kinds qualify, and both are decided by [`board_diagnostics::command_needs`]
//! rather than by a list kept here:
//!
//! * [`CommandNeeds::Nothing`] — answerable from the protocol layer.
//! * [`CommandNeeds::SharedBus`] — one self-contained I2C transaction against a
//!   device nothing else in the firmware drives. The bus manager locks per
//!   transaction, so these are safe from this thread. A *sequence* of
//!   transactions is not, which is why the sensors are classified separately.
//! * [`CommandNeeds::Reset`] — `REBOOT`, which needs no peripheral at all.

use core::time::Duration;

use board_diagnostics::{CommandNeeds, ConfigStaging, DiagnosticCommand, DiagnosticIo};
use embedded_hal::i2c::I2c;
use esp_idf_svc::nvs::{EspNvs, NvsDefault};

use crate::{
    config::FIRMWARE_VERSION,
    hardware::sensors::{self, SharedSensors},
    logic::fsm::diagnostics::{config, SharedI2cBus},
};

/// Devices expected on the shared bus, so a scan can name what it found and say
/// plainly what is missing.
const EXPECTED_I2C_DEVICES: &[(u8, &str)] =
    &[(0x3C, "SSD1306 OLED"), (0x40, "HDC1080"), (0x68, "DS3231")];

/// How long each pattern is held during a full `OLED_TEST` walkthrough.
///
/// Long enough for a person to register it. There is no upper bound to respect
/// here — the walk runs on the diagnostics thread and blocks nothing else — which
/// is the difference from the single-threaded firmware, where the same walk had
/// to be refused mid-move because it would have stalled the stepping loop.
const OLED_PATTERN_HOLD: Duration = Duration::from_millis(700);

/// Identity registers the HDC1080 reports.
///
/// A device answering anything else is not the sensor we think we are reading,
/// and its measurements mean nothing. Checked because this is precisely the fault
/// that produced a confident `-40.00 C / 0.00 %RH` on a board with no HDC1080
/// fitted at all.
const HDC1080_DEVICE_ID: u16 = 0x1050;
const HDC1080_MANUFACTURER_ID: u16 = 0x5449;

/// How long a diagnostic may wait for the sensor lock.
///
/// Generous compared with the heartbeat's budget, because somebody asked for this
/// reading and is waiting for it — but still bounded, so a clock sync at boot
/// cannot hold the console.
const SENSOR_LOCK_BUDGET: Duration = Duration::from_secs(2);

/// What answering a command produced.
///
/// `passed` and `details` become the terminal `RESULT` line; `message` is the
/// human-readable summary a host shows when there are no structured details.
#[derive(Debug)]
pub struct Answer {
    pub passed: bool,
    pub details: Vec<(String, String)>,
    pub message: String,
    /// The command asked the board to restart. Acted on by the caller once the
    /// response has had time to reach the wire.
    pub reboot_requested: bool,
}

impl Answer {
    pub(super) fn pass(details: Vec<(String, String)>, message: &str) -> Self {
        Self {
            passed: true,
            details,
            message: message.to_string(),
            reboot_requested: false,
        }
    }

    pub(super) fn fail(message: String) -> Self {
        Self {
            passed: false,
            details: Vec::new(),
            message,
            reboot_requested: false,
        }
    }
}

/// Can this machine answer the command without asking another?
///
/// [`CommandNeeds::Sensor`] is in here because this machine holds a share of the
/// sensor lock, not because sensors are unowned — the lock *is* the ownership.
/// [`CommandNeeds::Storage`] is in here because it holds its own NVS handle, the
/// same way the network and motion machines hold theirs.
pub fn is_local(command: &DiagnosticCommand) -> bool {
    matches!(
        board_diagnostics::command_needs(command),
        CommandNeeds::Nothing
            | CommandNeeds::SharedBus
            | CommandNeeds::Sensor
            | CommandNeeds::Storage
            | CommandNeeds::Reset
    )
}

/// Answer a command, writing any intermediate output to `io` as it happens.
#[allow(clippy::too_many_arguments)]
pub fn answer<IO: DiagnosticIo>(
    command: &DiagnosticCommand,
    bus: SharedI2cBus,
    sensors: &SharedSensors,
    nvs: &mut EspNvs<NvsDefault>,
    staging: &mut ConfigStaging,
    io: &mut IO,
) -> Answer {
    match command {
        DiagnosticCommand::Ping => {
            // Legacy terminal line, for a host predating the structured protocol.
            let _ = io.write_line("PONG");
            Answer::pass(Vec::new(), "pong")
        }

        DiagnosticCommand::FirmwareVersion => {
            // Order matters: a host resolves this command on the `VERSION` line,
            // so the handshake has to precede it or it arrives with no pending
            // request to attach it to and is discarded.
            let _ = io.write_line(&board_diagnostics::protocol_version_line());
            let _ = io.write_line(&board_diagnostics::firmware_version_line(FIRMWARE_VERSION));
            Answer::pass(
                vec![(String::from("version"), FIRMWARE_VERSION.to_string())],
                "",
            )
        }

        DiagnosticCommand::GetCapabilities => {
            let csv = super::capabilities_csv();
            let _ = io.write_line(&csv);
            Answer::pass(vec![(String::from("capabilities"), csv)], "")
        }

        DiagnosticCommand::I2cScan => i2c_scan(bus, io),

        DiagnosticCommand::OledTest { payload } => oled_test(bus, payload, io),

        DiagnosticCommand::Hdc1080Read => hdc1080_read(sensors, io),

        DiagnosticCommand::RtcCheck => rtc_check(sensors, io),

        DiagnosticCommand::SetEnv { key, value } => config::set_env(key, value, staging, io),

        DiagnosticCommand::InvalidSetEnv { .. } => {
            // Parsed but unusable. Terminated like anything else, so a host sees a
            // failure rather than waiting out a timeout on a typo.
            let message = String::from("SET_ENV: expected <key>=<value>");
            let _ = io.write_line("ERROR Bad SET_ENV format");
            Answer::fail(message)
        }

        DiagnosticCommand::GetEnv { key } => config::get_env(key, nvs, staging, io),

        DiagnosticCommand::SaveConfig => config::save_config(nvs, staging, io),

        DiagnosticCommand::GetConfig => config::get_config(nvs, staging, io),

        DiagnosticCommand::ConfigMode => config::config_mode(nvs, staging, io),

        DiagnosticCommand::WriteEnvFile => config::write_env_file(io),

        DiagnosticCommand::Reboot => {
            let _ = io.write_line("OK Rebooting");
            Answer {
                passed: true,
                details: Vec::new(),
                message: String::from("Rebooting"),
                reboot_requested: true,
            }
        }

        // `is_local` is the gate, and it is derived from the same classification
        // this arm falls through from — so reaching here means a command was
        // classified local and then not implemented, which is a bug rather than a
        // configuration.
        other => {
            let message = format!(
                "{} is classified as locally answerable but has no handler",
                other.name()
            );
            let _ = io.write_line(&format!("ERROR {message}"));
            Answer::fail(message)
        }
    }
}

/// Read temperature and humidity, having first proved it is the right chip.
///
/// The identity checks are the point. Without them this command reports whatever
/// the driver hands back, and a driver reading an absent device hands back zeros
/// — which decode to a perfectly plausible `-40.00 C / 0.00 %RH`. A diagnostic
/// that cannot fail is worse than no diagnostic, because someone believes it.
fn hdc1080_read<IO: DiagnosticIo>(sensors: &SharedSensors, io: &mut IO) -> Answer {
    let Some(mut set) = sensors::lock(sensors, SENSOR_LOCK_BUDGET) else {
        let message = String::from(
            "HDC1080_READ: the sensor is in use elsewhere and did not free up in time",
        );
        let _ = io.write_line(&format!("ERROR {message}"));
        return Answer::fail(message);
    };

    let Some(sensor) = set.hdc1080.as_mut() else {
        // Distinct from a read failure, and worth saying so: no HDC1080 answered
        // at boot, which on this board is a real hardware state rather than a
        // fault. Earlier revisions were not fitted with one.
        let message = String::from("HDC1080_READ: no HDC1080 was detected at boot");
        let _ = io.write_line(&format!("ERROR {message}"));
        return Answer::fail(message);
    };

    // Writes the configuration the read path assumes. Without it the device stays
    // in its power-on mode, where the four-byte sequential read below is not
    // valid — so this is not merely tidiness.
    if let Err(err) = sensor.init() {
        let message = format!("HDC1080 init failed: {err:?}");
        let _ = io.write_line(&format!("ERROR {message}"));
        return Answer::fail(message);
    }

    let device_id = match sensor.get_device_id() {
        Ok(id) => id,
        Err(err) => {
            let message = format!("HDC1080 get_device_id failed: {err:?}");
            let _ = io.write_line(&format!("ERROR {message}"));
            return Answer::fail(message);
        }
    };
    let _ = io.write_line(&format!("HDC1080 device_id: 0x{device_id:04X}"));
    if device_id != HDC1080_DEVICE_ID {
        let message = format!(
            "HDC1080 wrong device_id: read 0x{device_id:04X}, expected 0x{HDC1080_DEVICE_ID:04X}"
        );
        let _ = io.write_line(&format!("ERROR {message}"));
        return Answer::fail(message);
    }

    let manufacturer_id = match sensor.get_man_id() {
        Ok(id) => id,
        Err(err) => {
            let message = format!("HDC1080 get_manufacturer_id failed: {err:?}");
            let _ = io.write_line(&format!("ERROR {message}"));
            return Answer::fail(message);
        }
    };
    let _ = io.write_line(&format!("HDC1080 manufacturer_id: 0x{manufacturer_id:04X}"));
    if manufacturer_id != HDC1080_MANUFACTURER_ID {
        let message = format!(
            "HDC1080 wrong manufacturer_id: read 0x{manufacturer_id:04X}, expected 0x{HDC1080_MANUFACTURER_ID:04X}"
        );
        let _ = io.write_line(&format!("ERROR {message}"));
        return Answer::fail(message);
    }

    if let Ok(serial_id) = sensor.get_serial_id() {
        let _ = io.write_line(&format!(
            "HDC1080 serial_id: {:04X}-{:04X}-{:04X}",
            serial_id[0], serial_id[1], serial_id[2]
        ));
    }

    let (temp_c, humidity) = match sensor.read() {
        Ok(reading) => reading,
        Err(err) => {
            let message = format!("HDC1080 read failed: {err:?}");
            let _ = io.write_line(&format!("ERROR {message}"));
            return Answer::fail(message);
        }
    };

    // Legacy terminal line. The installer matches this exact shape; see
    // `docs/serial-protocol.md` in the installer before changing it.
    let _ = io.write_line(&format!(
        "HDC1080 Correct read: temp:{temp_c:.2} hum:{humidity:.2}"
    ));

    Answer::pass(
        vec![
            (String::from("temp"), format!("{temp_c:.2}")),
            (String::from("hum"), format!("{humidity:.2}")),
        ],
        "",
    )
}

/// Read the DS3231 and say whether the time it holds is usable.
///
/// Reads the chip, not the system clock. Reporting `Local::now()` here — which an
/// earlier version of this command did — tests the `chrono` crate: it passes
/// happily on a board whose RTC has lost its time, which is the exact fault this
/// exists to catch and the one that stops the tower booting.
fn rtc_check<IO: DiagnosticIo>(sensors: &SharedSensors, io: &mut IO) -> Answer {
    let Some(mut set) = sensors::lock(sensors, SENSOR_LOCK_BUDGET) else {
        let message =
            String::from("RTC_CHECK: the clock is in use elsewhere and did not free up in time");
        let _ = io.write_line(&format!("ERROR {message}"));
        return Answer::fail(message);
    };

    let Some(rtc_time) = set.rtc.read() else {
        let message = String::from("RTC_CHECK failed: DS3231 did not return a time");
        let _ = io.write_line(&format!("ERROR {message}"));
        return Answer::fail(message);
    };

    let stamp = rtc_time.format("%Y-%m-%d %H:%M:%S").to_string();

    // Checked *before* the legacy `TIME:` line is written. A host predating the
    // structured protocol resolves on that line, so emitting it first would report
    // a pass for a clock we are about to call invalid.
    if !rtc::Rtc::is_sane(&rtc_time) {
        let message = format!(
            "RTC_CHECK failed: DS3231 reads {stamp}, outside the plausible range — battery or never set"
        );
        let _ = io.write_line(&format!("ERROR {message}"));
        return Answer {
            passed: false,
            details: vec![(String::from("rtc_utc"), stamp)],
            message,
            reboot_requested: false,
        };
    }

    let _ = io.write_line(&format!("TIME: {stamp}"));

    Answer::pass(
        vec![
            (String::from("rtc_utc"), stamp),
            (
                String::from("system_local"),
                rtc::timezone::local_time()
                    .format("%Y-%m-%d %H:%M:%S")
                    .to_string(),
            ),
        ],
        "",
    )
}

/// Draw one named pattern, or walk all of them.
///
/// The panel is the one piece of hardware only this machine touches, so it needs
/// no owner but this one. A full walk takes several seconds — five full-screen
/// writes at 10 kHz, plus the holds — and that is simply fine here: the thread it
/// blocks is its own.
fn oled_test<IO: DiagnosticIo>(bus: SharedI2cBus, payload: &str, io: &mut IO) -> Answer {
    let mut panel = ssd1306_oled::Ssd1306::new(bus.acquire_i2c());

    // Checked before anything is drawn. Unlike the status LED, this part
    // acknowledges its address — so a missing panel is a real failure the board
    // can report by itself, rather than something only a person looking at it
    // could notice.
    if !panel.present() {
        let message = format!(
            "OLED_TEST: no response at 0x{:02X}",
            ssd1306_oled::DEFAULT_ADDRESS
        );
        let _ = io.write_line(&format!("ERROR {message}"));
        return Answer::fail(message);
    }

    if let Err(err) = panel.init() {
        let message = format!("OLED_TEST: initialisation failed: {err:?}");
        let _ = io.write_line(&format!("ERROR {message}"));
        return Answer::fail(message);
    }

    let requested = payload.trim();
    let patterns: Vec<ssd1306_oled::Pattern> = if requested.is_empty() {
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
                return Answer::fail(message);
            }
        }
    };

    let walking = patterns.len() > 1;
    for (index, pattern) in patterns.iter().enumerate() {
        // `Off` goes through `clear`, which also blanks the columns behind the
        // glass. Writing only the visible 128 leaves a strip holding whatever was
        // drawn there last, which is how a border survives being switched off.
        let outcome = if *pattern == ssd1306_oled::Pattern::Off {
            panel.clear()
        } else {
            panel.write_pattern(*pattern)
        };

        if let Err(err) = outcome {
            let message = format!("OLED_TEST: writing {} failed: {err:?}", pattern.name());
            let _ = io.write_line(&format!("ERROR {message}"));
            return Answer::fail(message);
        }

        let _ = io.write_line(&format!("OLED pattern {}", pattern.name()));
        if walking && index + 1 < patterns.len() {
            std::thread::sleep(OLED_PATTERN_HOLD);
        }
    }

    // Legacy terminal line, for a host that predates the structured protocol.
    let _ = io.write_line("OK");

    // Deliberately says what was *written*, not that anything appeared. The panel
    // acknowledging its address proves it is there; only the person looking at it
    // can say the right pixels lit.
    Answer::pass(
        vec![
            (String::from("patterns"), patterns.len().to_string()),
            (
                String::from("shown"),
                patterns
                    .iter()
                    .map(|pattern| pattern.name())
                    .collect::<Vec<_>>()
                    .join(","),
            ),
        ],
        "",
    )
}

/// Probe the expected addresses and report which answered.
///
/// Only the addresses this board actually carries, not a sweep of the range. Each
/// probe of an absent device costs a bus timeout, so 112 of them on a dead bus
/// runs for minutes — well past any host's patience, and it answers no question
/// the three below do not.
fn i2c_scan<IO: DiagnosticIo>(bus: SharedI2cBus, io: &mut IO) -> Answer {
    let mut proxy = bus.acquire_i2c();
    let mut found: Vec<&str> = Vec::new();
    let mut missing: Vec<&str> = Vec::new();

    for (address, name) in EXPECTED_I2C_DEVICES {
        // A zero-length write addresses the device and waits for an ACK without
        // transferring anything: the standard probe, and the safe one. Reading a
        // byte instead can disturb devices that treat a read as a pointer advance.
        if proxy.write(*address, &[]).is_ok() {
            let _ = io.write_line(&format!("I2C device 0x{address:02X}: {name}"));
            found.push(name);
        } else {
            let _ = io.write_line(&format!("I2C no response 0x{address:02X}: {name}"));
            missing.push(name);
        }
    }

    if found.is_empty() {
        let message =
            String::from("I2C_SCAN: no device answered, the bus itself is not responding");
        let _ = io.write_line(&format!("ERROR {message}"));
        return Answer::fail(message);
    }

    let mut details = vec![(String::from("found"), found.join(","))];
    if !missing.is_empty() {
        details.push((String::from("missing"), missing.join(",")));
    }

    // Devices answered, so the bus works — a pass, even with one of ours absent.
    // Which device is missing is a detail; whether the bus is alive is the
    // question this command exists to settle, and the one a failed sensor read
    // cannot answer on its own.
    Answer::pass(details, "")
}
