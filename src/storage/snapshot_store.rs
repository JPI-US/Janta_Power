use esp_idf_svc::nvs::{EspNvs, NvsPartitionId};
use log::{info, warn};
use motion::MotionMode;

// NVS keys used by runtime state.
pub const NVS_KEY_TRACKING_MODE: &str = "tracking_mode";
pub const NVS_KEY_HEADING: &str = "heading";

pub const NVS_KEY_ENC_SNAPSHOT_VERSION: &str = "enc_snapshot_v";
pub const NVS_KEY_ENC_TICKS_ADJ: &str = "enc_ticks_adj";

// Tracks which run mode last wrote stateful values.
pub const NVS_KEY_LAST_RUN_NORMAL: &str = "last_run_normal";

// ESP-IDF NVS key names are limited to 15 bytes.
pub const NVS_KEY_ENCODER_MODE_RESET_DATE: &str = "enc_rst_date";
pub const NVS_KEY_ENCODER_DAILY_MODE: &str = "enc_daily_mode";

pub const ENC_SNAPSHOT_VERSION: u32 = 1;

pub struct SnapshotStore<'a, T: NvsPartitionId> {
    nvs: &'a mut EspNvs<T>,
    persist_enabled: bool,
}

impl<'a, T: NvsPartitionId> SnapshotStore<'a, T> {
    pub fn new(nvs: &'a mut EspNvs<T>, persist_enabled: bool) -> Self {
        Self {
            nvs,
            persist_enabled,
        }
    }

    pub fn load_tracking_mode_or_init(&mut self, default: MotionMode) -> MotionMode {
        match self.nvs.get_u8(NVS_KEY_TRACKING_MODE).ok().flatten() {
            Some(1) => MotionMode::StepperOnly,
            Some(2) => MotionMode::EncoderGuarded,
            Some(other) => {
                warn!("Invalid tracking_mode={} in NVS; defaulting", other);
                default
            }
            None => {
                if self.persist_enabled {
                    let _ = self.nvs.set_u8(
                        NVS_KEY_TRACKING_MODE,
                        match default {
                            MotionMode::StepperOnly => 1,
                            MotionMode::EncoderGuarded => 2,
                        },
                    );
                } else {
                    warn!(
                        "NVS persist disabled: skipping init write for {}",
                        NVS_KEY_TRACKING_MODE
                    );
                }
                default
            }
        }
    }

    pub fn load_last_run_normal_or_init(&mut self, default: bool) -> bool {
        match self.nvs.get_u8(NVS_KEY_LAST_RUN_NORMAL).ok().flatten() {
            Some(1) => true,
            Some(0) => false,
            Some(other) => {
                warn!(
                    "Invalid {}={} in NVS; defaulting",
                    NVS_KEY_LAST_RUN_NORMAL, other
                );
                default
            }
            None => {
                if self.persist_enabled {
                    let _ = self
                        .nvs
                        .set_u8(NVS_KEY_LAST_RUN_NORMAL, if default { 1 } else { 0 });
                } else {
                    warn!(
                        "NVS persist disabled: skipping init write for {}",
                        NVS_KEY_LAST_RUN_NORMAL
                    );
                }
                default
            }
        }
    }

    pub fn save_last_run_normal(&mut self, normal: bool) {
        if !self.persist_enabled {
            warn!(
                "NVS persist disabled: skipping save_last_run_normal({})",
                normal
            );
            return;
        }
        if let Err(e) = self
            .nvs
            .set_u8(NVS_KEY_LAST_RUN_NORMAL, if normal { 1 } else { 0 })
        {
            warn!(
                "Failed to store {} in NVS: {:?}",
                NVS_KEY_LAST_RUN_NORMAL, e
            );
        } else {
            info!("Stored {} in NVS: {}", NVS_KEY_LAST_RUN_NORMAL, normal);
        }
    }

    pub fn load_heading_or_init(&mut self, default_heading: f32) -> f32 {
        match self.nvs.get_u32(NVS_KEY_HEADING).ok().flatten() {
            Some(v) => f32::from_bits(v),
            None => {
                if self.persist_enabled {
                    let _ = self.nvs.set_u32(NVS_KEY_HEADING, default_heading.to_bits());
                } else {
                    warn!(
                        "NVS persist disabled: skipping init write for {}",
                        NVS_KEY_HEADING
                    );
                }
                default_heading
            }
        }
    }

    pub fn save_heading(&mut self, heading: f32) {
        if !self.persist_enabled {
            warn!("NVS persist disabled: skipping save_heading({})", heading);
            return;
        }
        match self.nvs.set_u32(NVS_KEY_HEADING, heading.to_bits()) {
            Ok(_) => info!("Stored stable heading in NVS: {}", heading),
            Err(e) => warn!("Failed to store heading in NVS: {:?}", e),
        }
    }

    pub fn load_encoder_snapshot(&mut self) -> Option<i32> {
        let v = self
            .nvs
            .get_u32(NVS_KEY_ENC_SNAPSHOT_VERSION)
            .ok()
            .flatten()?;
        if v != ENC_SNAPSHOT_VERSION {
            warn!(
                "Encoder snapshot version mismatch: stored={}, expected={}",
                v, ENC_SNAPSHOT_VERSION
            );
            return None;
        }
        self.nvs.get_i32(NVS_KEY_ENC_TICKS_ADJ).ok().flatten()
    }

    pub fn save_encoder_snapshot(&mut self, enc_ticks_adj: i32) {
        if !self.persist_enabled {
            warn!(
                "NVS persist disabled: skipping save_encoder_snapshot(enc_ticks_adj={})",
                enc_ticks_adj
            );
            return;
        }
        if let Err(e) = self
            .nvs
            .set_u32(NVS_KEY_ENC_SNAPSHOT_VERSION, ENC_SNAPSHOT_VERSION)
        {
            warn!(
                "Failed to store encoder snapshot version in NVS ({}): {:?}",
                NVS_KEY_ENC_SNAPSHOT_VERSION, e
            );
        }
        match self.nvs.set_i32(NVS_KEY_ENC_TICKS_ADJ, enc_ticks_adj) {
            Ok(_) => info!(
                "Stored encoder snapshot in NVS: {}={} (v={})",
                NVS_KEY_ENC_TICKS_ADJ, enc_ticks_adj, ENC_SNAPSHOT_VERSION
            ),
            Err(e) => warn!(
                "Failed to store encoder ticks in NVS ({}): {:?}",
                NVS_KEY_ENC_TICKS_ADJ, e
            ),
        }
    }

    /// Load encoder mode reset date.
    pub fn load_encoder_mode_reset_date(&mut self) -> Option<String> {
        let mut buf = [0u8; 16];
        self.nvs
            .get_str(NVS_KEY_ENCODER_MODE_RESET_DATE, &mut buf)
            .ok()
            .flatten()
            .map(|s| s.to_string())
    }

    /// Save encoder mode reset date (`YYYY-MM-DD`).
    pub fn save_encoder_mode_reset_date(&mut self, date: &str) {
        if !self.persist_enabled {
            warn!(
                "NVS persist disabled: skipping save_encoder_mode_reset_date({})",
                date
            );
            return;
        }
        match self.nvs.set_str(NVS_KEY_ENCODER_MODE_RESET_DATE, date) {
            Ok(_) => info!(
                "Stored {} in NVS: {}",
                NVS_KEY_ENCODER_MODE_RESET_DATE, date
            ),
            Err(e) => warn!(
                "Failed to store {} in NVS: {:?}",
                NVS_KEY_ENCODER_MODE_RESET_DATE, e
            ),
        }
    }

    /// Load encoder daily mode flag.
    pub fn load_encoder_daily_mode(&mut self) -> bool {
        match self.nvs.get_u8(NVS_KEY_ENCODER_DAILY_MODE).ok().flatten() {
            Some(1) => true,
            Some(0) => false,
            Some(other) => {
                warn!(
                    "Invalid {}={} in NVS; defaulting to false",
                    NVS_KEY_ENCODER_DAILY_MODE, other
                );
                false
            }
            None => false,
        }
    }

    /// Save encoder daily mode flag.
    pub fn save_encoder_daily_mode(&mut self, daily_mode: bool) {
        if !self.persist_enabled {
            warn!(
                "NVS persist disabled: skipping save_encoder_daily_mode({})",
                daily_mode
            );
            return;
        }
        match self
            .nvs
            .set_u8(NVS_KEY_ENCODER_DAILY_MODE, if daily_mode { 1 } else { 0 })
        {
            Ok(_) => info!(
                "Stored {} in NVS: {}",
                NVS_KEY_ENCODER_DAILY_MODE, daily_mode
            ),
            Err(e) => warn!(
                "Failed to store {} in NVS: {:?}",
                NVS_KEY_ENCODER_DAILY_MODE, e
            ),
        }
    }

    /// Save tracking mode for recovery flow.
    pub fn save_tracking_mode(&mut self, mode: MotionMode) {
        if !self.persist_enabled {
            let mode_str = match mode {
                MotionMode::StepperOnly => "StepperOnly",
                MotionMode::EncoderGuarded => "EncoderGuarded",
            };
            warn!(
                "NVS persist disabled: skipping save_tracking_mode({})",
                mode_str
            );
            return;
        }
        let mode_value = match mode {
            MotionMode::StepperOnly => 1,
            MotionMode::EncoderGuarded => 2,
        };
        let mode_str = match mode {
            MotionMode::StepperOnly => "StepperOnly",
            MotionMode::EncoderGuarded => "EncoderGuarded",
        };
        match self.nvs.set_u8(NVS_KEY_TRACKING_MODE, mode_value) {
            Ok(_) => info!("Stored {} in NVS: {}", NVS_KEY_TRACKING_MODE, mode_str),
            Err(e) => warn!("Failed to store {} in NVS: {:?}", NVS_KEY_TRACKING_MODE, e),
        }
    }
}
