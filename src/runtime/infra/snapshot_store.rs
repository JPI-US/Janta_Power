use esp_idf_svc::nvs::{EspNvs, NvsPartitionId};
use log::{info, warn};

use motion::MotionMode;

// NVS keys used by the app.
pub const NVS_KEY_TRACKING_MODE: &str = "tracking_mode"; // 1=L1 legacy, 2=L2 current
pub const NVS_KEY_HEADING: &str = "heading";

pub const NVS_KEY_ENC_SNAPSHOT_VERSION: &str = "enc_snapshot_v";
pub const NVS_KEY_ENC_TICKS_ADJ: &str = "enc_ticks_adj";

// Tracks which run mode last wrote potentially-stateful values.
pub const NVS_KEY_LAST_RUN_NORMAL: &str = "last_run_normal"; // 1=Normal, 0=Admin/Other

// Encoder snapshot version (u32) stored in NVS.
pub const ENC_SNAPSHOT_VERSION: u32 = 1;

pub struct SnapshotStore<'a, T: NvsPartitionId> {
    nvs: &'a mut EspNvs<T>,
    persist_enabled: bool,
}

impl<'a, T: NvsPartitionId> SnapshotStore<'a, T> {
    pub fn new(nvs: &'a mut EspNvs<T>, persist_enabled: bool) -> Self {
        Self { nvs, persist_enabled }
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
                    warn!("NVS persist disabled: skipping init write for {}", NVS_KEY_TRACKING_MODE);
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
                warn!("Invalid {}={} in NVS; defaulting", NVS_KEY_LAST_RUN_NORMAL, other);
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
                    warn!("NVS persist disabled: skipping init write for {}", NVS_KEY_HEADING);
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
        let v = self.nvs.get_u32(NVS_KEY_ENC_SNAPSHOT_VERSION).ok().flatten()?;
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
}

