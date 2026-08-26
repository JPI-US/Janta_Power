//! Provisioned configuration: reading it, staging it, committing it.
//!
//! Answered by this machine rather than delegated, because the thing being
//! provisioned is NVS and this machine can hold its own handle to it. Every
//! machine that needs NVS opens its own — the network and motion machines already
//! do — and after the boot-seeding gate the write sets are disjoint: boot seeds
//! only while the tower is unprovisioned, motion writes only its snapshot keys,
//! and these commands write only [`board_diagnostics::CONFIG_KEYS`].
//!
//! # Staging, then committing
//!
//! `SET_ENV` validates and holds a value in RAM; `SAVE_CONFIG` writes the set and
//! marks the tower provisioned. The split is what makes a half-finished
//! provisioning run harmless: values arrive one at a time, and a run interrupted
//! partway leaves the tower on its previous configuration rather than with a new
//! SSID and an old password.

use board_diagnostics::{ConfigError, ConfigKey, ConfigStaging, DiagnosticIo};
use esp_idf_svc::nvs::{EspNvs, NvsPartitionId};

use crate::logic::fsm::diagnostics::local::Answer;

/// The value a read should report: staged if a `SET_ENV` is waiting, otherwise
/// what is committed.
///
/// Secrets report only whether they are set. Echoing one would let anyone with a
/// USB cable lift the site's Wi-Fi password off the tower.
fn read_value<T: NvsPartitionId>(
    nvs: &mut EspNvs<T>,
    staging: &ConfigStaging,
    entry: &ConfigKey,
) -> String {
    if entry.secret {
        let is_set = staging.get(entry.protocol).is_some()
            || read_stored(nvs, entry).is_some_and(|value| !value.is_empty());
        return board_diagnostics::secret_placeholder(is_set).to_string();
    }

    if let Some(staged) = staging.get(entry.protocol) {
        return staged.to_string();
    }

    read_stored(nvs, entry).unwrap_or_default()
}

fn read_stored<T: NvsPartitionId>(nvs: &mut EspNvs<T>, entry: &ConfigKey) -> Option<String> {
    // Every provisioned value is stored as a string, latitude and longitude
    // included — that is how the runtime readers already parse them.
    let mut buffer = [0u8; board_diagnostics::CONFIG_VALUE_MAX_BYTES + 1];
    nvs.get_str(entry.nvs, &mut buffer)
        .ok()
        .flatten()
        .map(|value| value.to_string())
}

/// `SET_ENV <key>=<value>` — validate and hold, without writing.
/// Takes no NVS handle on purpose: staging touches storage not at all, which is
/// the property that makes an interrupted provisioning run a no-op.
pub fn set_env<IO: DiagnosticIo>(
    key: &str,
    value: &str,
    staging: &mut ConfigStaging,
    io: &mut IO,
) -> Answer {
    let Some(entry) = board_diagnostics::config_key(key) else {
        return reject(io, key, ConfigError::UnknownKey);
    };

    if let Err(error) = board_diagnostics::validate_config_value(entry, value) {
        return reject(io, key, error);
    }

    staging.stage(entry, value.to_string());

    // Legacy terminal line. The installer resolves `SET_ENV` on exactly `OK`.
    let _ = io.write_line("OK");
    Answer::pass(
        vec![(String::from("staged"), staging.len().to_string())],
        "",
    )
}

fn reject<IO: DiagnosticIo>(io: &mut IO, key: &str, error: ConfigError) -> Answer {
    // Names the key and the reason. A rejection that writes nothing reads to the
    // host as a hung board rather than a bad value.
    let message = format!("SET_ENV {key}: {}", error.message());
    let _ = io.write_line(&format!("ERROR {message}"));
    Answer::fail(message)
}

/// `GET_ENV <key>` — report one value.
pub fn get_env<IO: DiagnosticIo, T: NvsPartitionId>(
    key: &str,
    nvs: &mut EspNvs<T>,
    staging: &ConfigStaging,
    io: &mut IO,
) -> Answer {
    let Some(entry) = board_diagnostics::config_key(key) else {
        let message = format!("GET_ENV {key}: {}", ConfigError::UnknownKey.message());
        let _ = io.write_line(&format!("ERROR {message}"));
        return Answer::fail(message);
    };

    let value = read_value(nvs, staging, entry);
    let _ = io.write_line(&format!("{key}={value}"));
    Answer::pass(vec![(String::from(entry.protocol), value)], "")
}

/// `SAVE_CONFIG` — commit the staged set and mark the tower provisioned.
pub fn save_config<IO: DiagnosticIo, T: NvsPartitionId>(
    nvs: &mut EspNvs<T>,
    staging: &mut ConfigStaging,
    io: &mut IO,
) -> Answer {
    let staged = staging.len();

    for (entry, value) in staging.entries() {
        if let Err(e) = nvs.set_str(entry.nvs, value) {
            // The staging buffer is left intact so the installer can retry the
            // commit without resending every value.
            let message = format!("SAVE_CONFIG {}: could not write ({e:?})", entry.protocol);
            let _ = io.write_line(&format!("ERROR {message}"));
            return Answer::fail(message);
        }
    }

    // Set last, and only after every value landed. The flag is what tells boot to
    // stop seeding the build-time defaults over the top, so setting it before the
    // values were written would strand a tower on neither configuration.
    if let Err(e) = nvs.set_u8(board_diagnostics::NVS_KEY_PROVISIONED, 1) {
        let message = format!("SAVE_CONFIG: could not mark the tower provisioned ({e:?})");
        let _ = io.write_line(&format!("ERROR {message}"));
        return Answer::fail(message);
    }

    staging.clear();

    let _ = io.write_line("OK");
    Answer::pass(vec![(String::from("committed"), staged.to_string())], "")
}

/// `GET_CONFIG` — stream every provisioned setting, grouped by section.
///
/// Walks the key table in order and opens a new `[section]` whenever the section
/// changes. `config_table_groups_each_section_contiguously` in the protocol crate
/// is what keeps that assumption true: a key out of place would emit a duplicate
/// header, and the installer's parser keeps only the later values.
pub fn get_config<IO: DiagnosticIo, T: NvsPartitionId>(
    nvs: &mut EspNvs<T>,
    staging: &ConfigStaging,
    io: &mut IO,
) -> Answer {
    let mut sections = 0usize;
    let mut current = "";

    for entry in board_diagnostics::CONFIG_KEYS {
        if entry.section() != current {
            current = entry.section();
            sections += 1;
            let _ = io.write_line(&format!("[{current}]"));
        }

        let value = read_value(nvs, staging, entry);
        let _ = io.write_line(&format!("{}={}", entry.name(), value));
    }

    Answer::pass(vec![(String::from("sections"), sections.to_string())], "")
}

/// `CONFIG_MODE` — is the provisioning path usable, and what does it hold?
///
/// There is no mode to enter. `SET_ENV` and `SAVE_CONFIG` work whenever this
/// machine is running, so a command that merely replied `OK` would be asserting
/// that rather than showing it — another diagnostic that cannot fail, which this
/// firmware has already produced twice.
///
/// It reads NVS instead, so it fails when the namespace cannot be opened: the
/// fault that would otherwise stay hidden until a `SAVE_CONFIG` several commands
/// later reported success and lost the lot. `stored` against `staged` also tells a
/// technician whether what they are looking at is committed or still waiting.
pub fn config_mode<IO: DiagnosticIo, T: NvsPartitionId>(
    nvs: &mut EspNvs<T>,
    staging: &ConfigStaging,
    io: &mut IO,
) -> Answer {
    let provisioned = match nvs.get_u8(board_diagnostics::NVS_KEY_PROVISIONED) {
        Ok(flag) => flag.unwrap_or(0) == 1,
        Err(e) => {
            let message = format!("CONFIG_MODE failed: NVS is not readable ({e:?})");
            let _ = io.write_line(&format!("ERROR {message}"));
            return Answer::fail(message);
        }
    };

    // A plain loop rather than a filtered iterator: `read_stored` needs the same
    // `&mut` handle an iterator would still be holding.
    let mut stored = 0usize;
    for entry in board_diagnostics::CONFIG_KEYS {
        if read_stored(nvs, entry).is_some_and(|value| !value.is_empty()) {
            stored += 1;
        }
    }

    // Legacy terminal line. `docs/serial-protocol.md` in the installer records the
    // three spellings a host accepts.
    let _ = io.write_line("CONFIG_MODE OK");

    Answer::pass(
        vec![
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
        "",
    )
}

/// `WRITE_ENV_FILE` — refused, and says why.
///
/// There is no filesystem on this board. Answering `OK` would report a write that
/// never happened, which is worse than refusing.
pub fn write_env_file<IO: DiagnosticIo>(io: &mut IO) -> Answer {
    let message = String::from("WRITE_ENV_FILE: this board has no filesystem to write to");
    let _ = io.write_line(&format!("ERROR {message}"));
    Answer::fail(message)
}
