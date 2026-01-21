use esp_idf_svc::nvs::{EspNvs, NvsPartitionId};
use log::{info, warn};

use motion::MotionMode;

// NVS keys (Stage 1 extraction)
pub const NVS_KEY_TRACKING_MODE: &str = "tracking_mode"; // 1=L1 legacy, 2=L2 current
pub const NVS_KEY_HEADING: &str = "heading";

pub const NVS_KEY_ENC_SNAPSHOT_VERSION: &str = "enc_snapshot_v";
pub const NVS_KEY_ENC_TICKS_ADJ: &str = "enc_ticks_adj";

// Encoder snapshot version (u32) stored in NVS.
pub const ENC_SNAPSHOT_VERSION: u32 = 1;

pub struct SnapshotStore<'a, T: NvsPartitionId> {
    nvs: &'a mut EspNvs<T>,
}

impl<'a, T: NvsPartitionId> SnapshotStore<'a, T> {
    pub fn new(nvs: &'a mut EspNvs<T>) -> Self {
        Self { nvs }
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
                let _ = self.nvs.set_u8(
                    NVS_KEY_TRACKING_MODE,
                    match default {
                        MotionMode::StepperOnly => 1,
                        MotionMode::EncoderGuarded => 2,
                    },
                );
                default
            }
        }
    }

    pub fn load_heading_or_init(&mut self, default_heading: f32) -> f32 {
        match self.nvs.get_u32(NVS_KEY_HEADING).ok().flatten() {
            Some(v) => f32::from_bits(v),
            None => {
                let _ = self.nvs.set_u32(NVS_KEY_HEADING, default_heading.to_bits());
                default_heading
            }
        }
    }

    pub fn save_heading(&mut self, heading: f32) {
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


