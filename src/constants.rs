// Include the auto-generated constants from build.rs
include!(concat!(env!("OUT_DIR"), "/constants.rs"));

// Re-export RunMode enum
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum RunMode {
    Normal,
    Admin,
}

// Helper functions to convert string constants to enums
// Note: These cannot be const fn because string matching isn't const-evaluable in stable Rust
// They are evaluated at runtime, but the values come from compile-time .env
pub fn get_active_profile() -> crate::switchboard::Profile {
    match ACTIVE_PROFILE_STR {
        "Normal" => crate::switchboard::Profile::Normal,
        "Diagnostic" => crate::switchboard::Profile::Diagnostic,
        "Custom" => crate::switchboard::Profile::Custom,
        _ => crate::switchboard::Profile::Normal,
    }
}

pub fn get_active_mode() -> RunMode {
    match ACTIVE_MODE_STR {
        "Normal" => RunMode::Normal,
        "Admin" => RunMode::Admin,
        _ => RunMode::Normal,
    }
}
