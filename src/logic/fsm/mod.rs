use ::motion::motion::MotionMode;
use fsm::postal::Address;

pub mod buttons;
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
}

#[derive(Copy, Clone, Debug)]
pub enum FSMAddress {
    Motion,
    Network,
    Buttons,
}

impl Address for FSMAddress {
    fn index(self) -> usize {
        match self {
            FSMAddress::Motion => 0,
            FSMAddress::Network => 1,
            FSMAddress::Buttons => 2,
        }
    }

    fn count() -> usize {
        3
    }
}

#[derive(Copy, Clone, Debug, Default)]
pub struct FSMState {}
