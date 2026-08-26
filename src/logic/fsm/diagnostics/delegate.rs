//! Where a command goes when this machine cannot answer it itself.
//!
//! The mapping is derived from [`board_diagnostics::CommandNeeds`] rather than
//! written out per command, so adding a command means classifying what it needs
//! and nothing else. That classification is an exhaustive match in the protocol
//! crate, which is what stops a new command going quietly unrouted.

use core::time::Duration;

use board_diagnostics::{CommandNeeds, DiagnosticCommand};

use crate::logic::fsm::FSMAddress;

/// Which machine owns the hardware this command touches.
///
/// `None` means nothing can answer it: either no machine owns that hardware yet,
/// or the command is unimplemented. The caller distinguishes those two cases when
/// it explains itself, because they call for different reactions — one is worth
/// retrying after an update, the other never will be.
pub fn owner(command: &DiagnosticCommand) -> Option<FSMAddress> {
    match board_diagnostics::command_needs(command) {
        CommandNeeds::StatusLed => Some(FSMAddress::Led),
        CommandNeeds::Motor => Some(FSMAddress::Motion),

        // Answered here, so they never reach delegation.
        CommandNeeds::Nothing
        | CommandNeeds::SharedBus
        | CommandNeeds::Sensor
        | CommandNeeds::Reset => None,

        // No machine holds provisioned configuration yet.
        CommandNeeds::Storage => None,

        CommandNeeds::Unsupported => None,
    }
}

/// How long to wait for that machine before giving up on it.
///
/// Bounded for every command, without exception. A machine that never answers
/// must not be able to hold the console: the point of this design is that the
/// board always says *something*, and "no reply from the motion machine" is a far
/// better thing for a technician to read than nothing at all.
///
/// The budgets differ by orders of magnitude because the work does. A colour
/// change is one mailbox round trip between machines stepping at 100 ms and
/// 20 ms; a homing search really can run for the better part of an hour.
pub fn reply_budget(command: &DiagnosticCommand) -> Duration {
    match board_diagnostics::command_needs(command) {
        // Generous against a ~150 ms round trip. Anything approaching this means
        // the auxiliary machine is stuck, not busy.
        CommandNeeds::StatusLed => Duration::from_secs(3),

        // A full-travel homing sweep at this board's step rate is roughly an
        // hour. Progress lines arrive throughout, so a host measuring silence
        // never has to wait anything like this long to learn something is wrong.
        CommandNeeds::Motor => Duration::from_secs(3900),

        _ => Duration::from_secs(5),
    }
}

/// A human-readable name for the machine, for the message a technician reads.
pub fn owner_name(address: FSMAddress) -> &'static str {
    match address {
        FSMAddress::Motion => "motion",
        FSMAddress::Network => "network",
        FSMAddress::Buttons => "buttons",
        FSMAddress::Led => "LED",
        FSMAddress::Diagnostics => "diagnostics",
    }
}
