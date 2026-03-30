#![allow(dead_code)]

// =============================================================================
// Switchboard: single source of deployment/default values for the app.
// Values default to crate::constants (generated from .env by build.rs).
// To override for an OTA: replace any field below with a hardcoded literal
// in this file and ship that build; the app reads only from switchboard.
// =============================================================================

#[derive(Copy, Clone, Debug)]
pub enum Profile {
    Normal,
    Admin,
    Custom,
}

// =========================
// Types for future phases
// =========================

#[derive(Copy, Clone, Debug)]
pub enum Direction {
    Cw,
    Ccw,
}

impl Direction {
    pub const fn as_str(&self) -> &'static str {
        match self {
            Direction::Cw => "CW",
            Direction::Ccw => "CCW",
        }
    }

    /// Returns degrees with sign applied (CW positive, CCW negative).
    pub const fn apply_to_deg(&self, deg: f32) -> f32 {
        match self {
            Direction::Cw => deg,
            Direction::Ccw => -deg,
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub struct RecoveryMoveSpec {
    pub dir: Direction,
    pub deg: f32,
}

#[derive(Copy, Clone, Debug)]
pub struct BootHomingSwitches {
    /// If false, we will skip the homing search entirely (even if snapshot restore failed).
    pub enabled: bool,
    /// Which direction to search for the limit switch.
    pub dir: Direction,
}

#[derive(Copy, Clone, Debug)]
pub struct RecoverySwitches {
    pub enabled: bool,
    pub moves: &'static [RecoveryMoveSpec],
    pub verify_with_encoder: bool,
    pub verify_tol_deg: f32,
    pub stop_on_verify_fail: bool,
    pub stop_after: bool,
    /// If true, temporarily disable stall detection while running recovery moves.
    pub disable_stall_detection: bool,
}

#[derive(Copy, Clone, Debug)]
pub struct BootSwitches {
    pub recovery: RecoverySwitches,
    pub homing: BootHomingSwitches,
}

#[derive(Copy, Clone, Debug)]
pub struct TrackingSwitches {
    /// If false, we do not call into the tracking logic at all.
    pub enabled: bool,
    /// Delay between tracking iterations (seconds).
    pub loop_sleep_secs: u64,
}

#[derive(Copy, Clone)]
pub enum MotionModePolicy {
    /// Use NVS `tracking_mode` if present; otherwise initialize NVS with this default.
    FromNvsDefault(motion::MotionMode),
    /// Ignore NVS and force this mode at runtime (diagnostics / bring-up).
    Force(motion::MotionMode),
}

impl core::fmt::Debug for MotionModePolicy {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            MotionModePolicy::FromNvsDefault(_) => f.write_str("FromNvsDefault(..)"),
            MotionModePolicy::Force(_) => f.write_str("Force(..)"),
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub struct EncoderRecoverySwitches {
    /// If false, we never enter the encoder-fault probe loop.
    pub enabled: bool,
    pub probe_interval_secs: u64,
    pub probe_steps: i64,
    pub max_drift_deg: f32,
    pub rehome_dir: Direction,
}

#[derive(Copy, Clone, Debug)]
pub struct GuardrailsSwitches {
    pub stall_detection_enabled: bool,
    pub soft_limits_enabled: bool,
    pub soft_limit_min_deg: f32,
    pub soft_limit_max_deg: f32,
}

#[derive(Copy, Clone, Debug)]
pub struct EffectsSwitches {
    pub publish_mqtt: bool,
    pub persist_nvs: bool,
    pub allow_ota: bool,
    pub allow_boot_validation: bool,
}

#[derive(Copy, Clone, Debug)]
pub struct AdminTestsSwitches {
    pub motor_test: bool,
    pub encoder_test: bool,
    pub persistence_test: bool,
    pub wifi_mqtt_test: bool,
}

#[derive(Copy, Clone, Debug)]
pub struct AdminSwitches {
    pub enabled: bool,
    pub run_recovery_on_start: bool,
    pub run_homing_on_start: bool,
    pub tests: AdminTestsSwitches,
    pub stop_after: bool,
}

#[derive(Copy, Clone, Debug)]
pub struct RuntimeSwitches {
    pub tracking: TrackingSwitches,
    pub motion_mode: MotionModePolicy,
    pub encoder_recovery: EncoderRecoverySwitches,
    pub guardrails: GuardrailsSwitches,
}

// Default "unstuck" sequence (kept identical across profiles unless overridden).
const DEFAULT_RECOVERY_MOVES: &[RecoveryMoveSpec] = &[
    RecoveryMoveSpec {
        dir: Direction::Cw,
        deg: 360.0,
    },
    RecoveryMoveSpec {
        dir: Direction::Ccw,
        deg: 0.0,
    },
];

// =========================
// Switchboard configuration
// =========================

/// Switchboard: policy/data only.
#[derive(Copy, Clone, Debug)]
pub struct Switchboard {
    /// Device id used as MQTT topic prefix (e.g. "device5" -> "device5/firmware/status").
    pub device_id: &'static str,

    // Timing
    pub wifi_connect_delay_secs: u64,
    pub tracking_loop_sleep_secs: u64,
    pub ota_check_delay_secs: u64,

    // MQTT
    pub mqtt_broker_url: &'static str,
    pub mqtt_client_id: &'static str,

    // Firmware + persistence keys
    pub default_version: &'static str,
    pub heading_tag: &'static str,

    pub enc_snapshot_version: u32,
    pub nvs_key_enc_snapshot_version: &'static str,
    pub nvs_key_enc_ticks_adj: &'static str,
    pub enc_home_tol_ticks: i32,
    /// Heading in degrees at limit switch (home); used by encoder recovery.
    pub home_heading_deg: f32,

    // Defaults currently written into NVS on boot
    pub default_mqtt_user: &'static str,
    pub default_mqtt_pass: &'static str,
    pub default_wifi_ssid: &'static str,
    pub default_wifi_pass: &'static str,
    pub default_tz_offset_hours: i32,

    // Tower defaults
    pub default_tower_latitude: f64,
    pub default_tower_longitude: f64,
    pub default_tower_id: u32,
    pub default_ota_updater: &'static str,
    pub default_ota_password: &'static str,

    // Nested (Phase 0: present for parity; unused until later phases)
    pub boot: BootSwitches,
    pub runtime: RuntimeSwitches,
    pub effects: EffectsSwitches,
    pub admin: AdminSwitches,
}

pub const fn normal() -> Switchboard {
    Switchboard {
        device_id: crate::constants::DEVICE_ID,

        wifi_connect_delay_secs: 20,
        tracking_loop_sleep_secs: 300,
        ota_check_delay_secs: 3,

        mqtt_broker_url: "mqttS://mqtt.jantaus.com:9443",
        mqtt_client_id: "device1_pub",

        default_version: "1.0.4",
        heading_tag: "heading",

        enc_snapshot_version: 1,
        nvs_key_enc_snapshot_version: "enc_snapshot_v",
        nvs_key_enc_ticks_adj: "enc_ticks_adj",
        enc_home_tol_ticks: 50,
        home_heading_deg: crate::constants::HOME_HEADING_DEG,

        default_mqtt_user: "device1",
        default_mqtt_pass: "1device",
        default_wifi_ssid: crate::constants::WIFI_SSID,
        default_wifi_pass: crate::constants::WIFI_PASSWORD,
        default_tz_offset_hours: crate::constants::TIMEZONE_OFFSET_HOURS_I32,

        default_ota_updater: "device1A",
        default_ota_password: "device1A",

        default_tower_latitude: crate::constants::TOWER_LATITUDE,
        default_tower_longitude: crate::constants::TOWER_LONGITUDE,
        default_tower_id: crate::constants::TOWER_ID,

        boot: BootSwitches {
            recovery: RecoverySwitches {
                enabled: false,
                moves: DEFAULT_RECOVERY_MOVES,
                verify_with_encoder: true,
                verify_tol_deg: 100.0,
                stop_on_verify_fail: false,
                stop_after: true,
                disable_stall_detection: true,
            },
            homing: BootHomingSwitches {
                enabled: true,
                dir: Direction::Cw,
            },
        },
        runtime: RuntimeSwitches {
            tracking: TrackingSwitches {
                enabled: true,
                loop_sleep_secs: 300,
            },
            motion_mode: MotionModePolicy::FromNvsDefault(motion::MotionMode::EncoderGuarded),
            encoder_recovery: EncoderRecoverySwitches {
                enabled: true,
                probe_interval_secs: 180,
                probe_steps: crate::constants::ENCODER_PROBE_STEPS,
                max_drift_deg: 15.0,
                rehome_dir: Direction::Cw,
            },
            guardrails: GuardrailsSwitches {
                stall_detection_enabled: true,
                soft_limits_enabled: true,
                soft_limit_min_deg: crate::constants::SOFT_LIMIT_MIN_DEG,
                soft_limit_max_deg: crate::constants::SOFT_LIMIT_MAX_DEG,
            },
        },
        effects: EffectsSwitches {
            publish_mqtt: true,
            persist_nvs: true,
            allow_ota: true,
            allow_boot_validation: true,
        },
        admin: AdminSwitches {
            enabled: false,
            run_recovery_on_start: false,
            run_homing_on_start: false,
            tests: AdminTestsSwitches {
                motor_test: false,
                encoder_test: false,
                persistence_test: false,
                wifi_mqtt_test: false,
            },
            stop_after: true,
        },
    }
}

pub const fn admin() -> Switchboard {
    // Phase 0: placeholder that keeps behavior identical if selected.
    normal()
}

pub const fn custom() -> Switchboard {
    // Phase 0: placeholder that keeps behavior identical if selected.
    normal()
}

pub const fn active(profile: Profile) -> Switchboard {
    match profile {
        Profile::Normal => normal(),
        Profile::Admin => admin(),
        Profile::Custom => custom(),
    }
}

