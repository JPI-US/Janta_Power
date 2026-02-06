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
}

#[derive(Copy, Clone, Debug)]
pub struct Switchboard {
    pub boot: BootSwitches,
    pub runtime: RuntimeSwitches,
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
        deg: 190.0,
    },
    RecoveryMoveSpec {
        dir: Direction::Ccw,
        deg: 180.0,
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
                verify_tol_deg: 100.0,
                stop_on_verify_fail: true,
                stop_after: true,
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

