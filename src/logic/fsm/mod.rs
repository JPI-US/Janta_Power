use ::motion::motion::MotionMode;
use board_diagnostics::DiagnosticCommand;
use fsm::postal::Address;

pub mod buttons;
pub mod diagnostics;
pub mod led;
pub mod motion;
pub mod network;

#[derive(Debug, Clone)]
pub enum FSMCommand {
    MqttPublishJson(String, String),
    UpdateNetworkMotionContext(MotionMode, f32),
    PerformOTA,
    MaintenancePressed,
    CCWPressed,
    CWPressed,
    /// A diagnostics command handed to whichever machine owns the hardware.
    DiagnosticRequest(DiagRequest),
    /// That machine's answer, on its way back.
    DiagnosticReply(DiagReply),
}

/// A diagnostics command the diagnostics machine cannot answer itself.
///
/// The rule is that a diagnostic is answered by whoever owns the hardware it
/// touches; this is how it gets there. See [`board_diagnostics::CommandNeeds`]
/// for the classification that decides where "there" is.
#[derive(Debug, Clone)]
pub struct DiagRequest {
    /// Correlates the reply with the request.
    ///
    /// Not decoration. A command that times out is answered to the host and
    /// forgotten, but the machine handling it may still reply afterwards — and
    /// without a ticket that late reply would resolve whatever command happens to
    /// be in flight *next*, reporting one test's result as another's.
    pub ticket: u32,
    pub command: DiagnosticCommand,
}

/// An answer travelling back to the diagnostics machine.
#[derive(Debug, Clone)]
pub struct DiagReply {
    pub ticket: u32,
    pub outcome: DiagOutcome,
}

#[derive(Debug, Clone)]
pub enum DiagOutcome {
    /// A line to put on the wire while the command is still running.
    ///
    /// Long commands need this. A host measures silence, not elapsed time, so a
    /// command that reports nothing for minutes is indistinguishable from a board
    /// that has stopped — however healthy the eventual answer.
    Progress(String),
    /// The command finished. Renders as the terminal `RESULT` line.
    Done {
        passed: bool,
        details: Vec<(String, String)>,
        message: String,
    },
}

#[derive(Copy, Clone, Debug)]
pub enum FSMAddress {
    Motion,
    Network,
    Buttons,
    Led,
    Diagnostics,
}

impl Address for FSMAddress {
    fn index(self) -> usize {
        match self {
            FSMAddress::Motion => 0,
            FSMAddress::Network => 1,
            FSMAddress::Buttons => 2,
            FSMAddress::Led => 3,
            FSMAddress::Diagnostics => 4,
        }
    }

    fn count() -> usize {
        5
    }
}

#[derive(Copy, Clone, Debug, Default)]
pub struct FSMState {
    maintenance_mode: bool,
}
