// Include the auto-generated constants from build.rs
include!(concat!(env!("OUT_DIR"), "/constants.rs"));

// Re-export RunMode enum
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum RunMode {
    Normal,
    Admin,
}

// Helper functions removed - no longer needed without switchboard
