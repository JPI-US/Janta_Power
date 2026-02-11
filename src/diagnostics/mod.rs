// Diagnostics module: all test bench / admin / manual control code.
//
// This module is intentionally separate from `src/app/` (normal operation)
// so that future feature developers don't need to navigate through diagnostic code.

pub mod admin_mode;
pub mod boot_recovery;
pub mod cmd_handler;