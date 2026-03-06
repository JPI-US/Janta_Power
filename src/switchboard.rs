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
        // `motion::MotionMode` doesn't implement Debug, so only print the variant name.
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
    /// How long to wait between recovery probes.
    pub probe_interval_secs: u64,
    /// How many steps to move during a probe.
    pub probe_steps: i64,
    /// Maximum heading drift allowed when encoder comes back before we re-home.
    pub max_drift_deg: f32,
    /// Which direction to use when re-homing after drift is too large.
    pub rehome_dir: Direction,
}

#[derive(Copy, Clone, Debug)]
pub struct RuntimeSwitches {
    pub tracking: TrackingSwitches,
    pub motion_mode: MotionModePolicy,
    pub encoder_recovery: EncoderRecoverySwitches,
    pub guardrails: GuardrailsSwitches,
}

#[derive(Copy, Clone, Debug)]
pub struct Switchboard {
    pub boot: BootSwitches,
    pub runtime: RuntimeSwitches,
    pub effects: EffectsSwitches,
    pub admin: AdminSwitches,
}

#[derive(Copy, Clone, Debug)]
pub struct GuardrailsSwitches {
    /// Stall detector in move execution (EncoderGuarded only).
    pub stall_detection_enabled: bool,
    /// Clamp tracking target heading into [soft_limit_min_deg, soft_limit_max_deg].
    pub soft_limits_enabled: bool,
    pub soft_limit_min_deg: f32,
    pub soft_limit_max_deg: f32,
}

#[derive(Copy, Clone, Debug)]
pub struct EffectsSwitches {
    /// If false, suppress MQTT publishes across the app (best-effort).
    pub publish_mqtt: bool,
    /// If false, suppress NVS writes across the app (best-effort).
    pub persist_nvs: bool,
    /// If false, suppress OTA checks (best-effort).
    pub allow_ota: bool,
    /// If false, skip first-boot validation/rollback logic.
    pub allow_boot_validation: bool,
}

#[derive(Copy, Clone, Debug)]
pub struct AdminTestsSwitches {
    /// Basic motor movement sanity checks (open-loop).
    pub motor_test: bool,
    /// Encoder sanity checks (tick direction / tick change).
    pub encoder_test: bool,
    /// NVS read/write verification test.
    pub persistence_test: bool,
    /// WiFi + MQTT connectivity and publish test.
    pub wifi_mqtt_test: bool,
}

#[derive(Copy, Clone, Debug)]
pub struct AdminSwitches {
    /// If false, Admin mode will refuse to start (even if `ACTIVE_MODE=Admin`).
    pub enabled: bool,
    /// If true, run the recovery move sequence when Admin mode starts.
    pub run_recovery_on_start: bool,
    /// If true, run homing when Admin mode starts.
    pub run_homing_on_start: bool,
    /// Which test cases are enabled in Admin mode.
    pub tests: AdminTestsSwitches,
    /// If true, stop/idle after running startup actions/tests (do not enter normal tracking loop).
    pub stop_after: bool,
}

// =========================
// Profiles
// =========================

#[derive(Copy, Clone, Debug)]
pub enum Profile {
    Normal,
    Diagnostic,
    Custom,
}

// Default "unstuck" sequence (kept identical across profiles unless overridden).
const DEFAULT_RECOVERY_MOVES: &[RecoveryMoveSpec] = &[
    RecoveryMoveSpec {
        dir: Direction::Cw,
        deg: 180.0,
    },
    RecoveryMoveSpec {
        dir: Direction::Ccw,
        deg: 0.0,
    },
];

pub const fn normal() -> Switchboard {
    // Must match the previous `main.rs` constants (no functional change).
    Switchboard {
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
                dir: Direction::Ccw,
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
                probe_steps: 30_000,
                max_drift_deg: 15.0,
                rehome_dir: Direction::Ccw,
            },
            guardrails: GuardrailsSwitches {
                stall_detection_enabled: true,
                soft_limits_enabled: false,
                // Set the numbers now for convenience; flip `soft_limits_enabled` when ready.
                soft_limit_min_deg: 0.0,
                soft_limit_max_deg: 285.0,
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

pub const fn diagnostic() -> Switchboard {
    // A safer bench profile: disable normal tracking; keep recovery/homing enabled.
    Switchboard {
        boot: BootSwitches {
            recovery: RecoverySwitches {
                enabled: true,
                moves: DEFAULT_RECOVERY_MOVES,
                verify_with_encoder: true,
                verify_tol_deg: 2.0,
                stop_on_verify_fail: true,
                stop_after: true,
                disable_stall_detection: true,
            },
            homing: BootHomingSwitches {
                enabled: true,
                dir: Direction::Ccw,
            },
        },
        runtime: RuntimeSwitches {
            tracking: TrackingSwitches {
                enabled: false,
                loop_sleep_secs: 300,
            },
            motion_mode: MotionModePolicy::Force(motion::MotionMode::EncoderGuarded),
            encoder_recovery: EncoderRecoverySwitches {
                enabled: true,
                probe_interval_secs: 60,
                probe_steps: 30_000,
                max_drift_deg: 15.0,
                rehome_dir: Direction::Ccw,
            },
            guardrails: GuardrailsSwitches {
                stall_detection_enabled: false,
                soft_limits_enabled: false,
                soft_limit_min_deg: 0.0,
                soft_limit_max_deg: 285.0,
            },
        },
        effects: EffectsSwitches {
            // Diagnostics usually want to avoid touching flash, but still allow MQTT for tests.
            publish_mqtt: true,
            persist_nvs: true,
            allow_ota: false,
            allow_boot_validation: false,
        },
        admin: AdminSwitches {
            enabled: true,
            run_recovery_on_start: true,
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

pub const fn custom() -> Switchboard {
    // User-tuned profile placeholder: start from normal and tweak here.
    normal()
}

pub const fn active(profile: Profile) -> Switchboard {
    match profile {
        Profile::Normal => normal(),
        Profile::Diagnostic => diagnostic(),
        Profile::Custom => custom(),
    }
}
