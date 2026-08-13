#![allow(dead_code)]

use motion::{
    motion::{ActiveLevel, MotionMode},
    Direction,
};

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

impl Profile {
    /// Map the `ACTIVE_PROFILE` string from `.env`/constants to a profile.
    /// Unknown values fall back to `Normal` (production-safe default).
    pub fn from_env_str(s: &str) -> Self {
        match s {
            "Admin" => Profile::Admin,
            "Custom" => Profile::Custom,
            _ => Profile::Normal,
        }
    }
}

// =========================
// Types for future phases
// =========================

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
    FromNvsDefault(MotionMode),
    /// Ignore NVS and force this mode at runtime (diagnostics / bring-up).
    Force(MotionMode),
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
    /// Remote MQTT command channel (subscribe at boot + handle one command per loop).
    pub commands_enabled: bool,
    pub relay_active_level: ActiveLevel,
    pub lmsw_active_level: ActiveLevel,
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
    /// MQTT topic prefix; sourced from `DEVICE_ID` in `.env` (e.g. `"2a"`).
    /// Used as the tower id in AWS topics like `tower/{device_id}/status`.
    pub device_id: &'static str,

    // Timing
    pub wifi_connect_delay_secs: u64,
    pub tracking_loop_sleep_secs: u64,
    pub ota_check_delay_secs: u64,

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
    pub default_wifi_ssid: &'static str,
    pub default_wifi_pass: &'static str,
    /// POSIX `TZ` string for libc (`setenv` + `tzset`); build default from `TZ_POSIX` in `.env`.
    pub default_tz_posix: &'static str,

    // Tower defaults
    pub default_tower_latitude: f64,
    pub default_tower_longitude: f64,
    pub default_ota_updater: &'static str,
    pub default_ota_password: &'static str,

    // Nested (Phase 0: present for parity; unused until later phases)
    pub boot: BootSwitches,
    pub runtime: RuntimeSwitches,
    pub effects: EffectsSwitches,
    pub admin: AdminSwitches,
}

pub const fn normal() -> Switchboard {
    let relay_active_level = if crate::config::constants::RELAY_ACTIVE_HIGH {
        ActiveLevel::ActiveHigh
    } else {
        ActiveLevel::ActiveLow
    };

    let lmsw_active_level = if crate::config::constants::LIMIT_SWITCH_ACTIVE_HIGH {
        ActiveLevel::ActiveHigh
    } else {
        ActiveLevel::ActiveLow
    };

    Switchboard {
        device_id: crate::config::constants::DEVICE_ID,

        wifi_connect_delay_secs: 20,
        tracking_loop_sleep_secs: 300,
        ota_check_delay_secs: 3,

        default_version: "1.0.4",
        heading_tag: "heading",

        enc_snapshot_version: 1,
        nvs_key_enc_snapshot_version: "enc_snapshot_v",
        nvs_key_enc_ticks_adj: "enc_ticks_adj",
        enc_home_tol_ticks: 50,
        home_heading_deg: crate::config::constants::HOME_HEADING_DEG,

        default_wifi_ssid: crate::config::constants::WIFI_SSID,
        default_wifi_pass: crate::config::constants::WIFI_PASSWORD,
        default_tz_posix: crate::config::constants::TZ_POSIX,

        default_ota_updater: "device1A",
        default_ota_password: "device1A",

        default_tower_latitude: crate::config::constants::TOWER_LATITUDE,
        default_tower_longitude: crate::config::constants::TOWER_LONGITUDE,

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
            motion_mode: MotionModePolicy::FromNvsDefault(MotionMode::EncoderGuarded),
            encoder_recovery: EncoderRecoverySwitches {
                enabled: true,
                probe_interval_secs: 180,
                probe_steps: crate::config::constants::ENCODER_PROBE_STEPS,
                max_drift_deg: 15.0,
                rehome_dir: Direction::Cw,
            },
            guardrails: GuardrailsSwitches {
                stall_detection_enabled: true,
                soft_limits_enabled: true,
                soft_limit_min_deg: crate::config::constants::SOFT_LIMIT_MIN_DEG,
                soft_limit_max_deg: crate::config::constants::SOFT_LIMIT_MAX_DEG,
            },
            commands_enabled: true,
            relay_active_level,
            lmsw_active_level,
        },
        effects: EffectsSwitches {
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

/// Diagnostics sandbox. Same hardware/network defaults as [`normal`], but
/// tracking, boot homing, and OTA are turned **off** so the tower stays put and
/// nothing competes with the feature under test. The command channel stays **on**.
pub fn admin() -> Switchboard {
    let mut sw = normal();
    sw.admin.enabled = true;
    sw.runtime.tracking.enabled = false;
    sw.boot.homing.enabled = false;
    sw.effects.allow_ota = false;
    sw.runtime.commands_enabled = true;
    sw
}

/// Hook for site-specific images; identical to [`normal`] until customized here.
pub const fn custom() -> Switchboard {
    normal()
}

pub fn active(profile: Profile) -> Switchboard {
    match profile {
        Profile::Normal => normal(),
        Profile::Admin => admin(),
        Profile::Custom => custom(),
    }
}
