#[allow(dead_code)]
pub mod constants;
pub mod switchboard;

/// The firmware version this image reports, everywhere it is asked.
///
/// One definition on purpose. There were three: a private `DEFAULT_VERSION` in
/// `logic/fsm/network.rs`, an unused `Switchboard::default_version` that had gone
/// stale at `1.0.4`, and — once the diagnostics channel was added — whatever it
/// answered `FIRMWARE_VERSION` with. A tower that tells the dashboard one version
/// and a technician's laptop another is worse than one that reports neither,
/// because both look authoritative.
///
/// Bump this and nothing else. `BootValidation` writes it to NVS, telemetry
/// publishes it, and the diagnostics `VERSION` line reports it.
pub const FIRMWARE_VERSION: &str = "1.1.5";
