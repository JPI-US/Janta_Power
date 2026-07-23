use ::motion::motion::MotionMode;
use fsm::postal::Address;

pub mod led;
pub mod maintenance;
pub mod motion;
pub mod network;
#[derive(Debug, Clone)]

pub enum FSMCommand {
    LEDOff,
    LEDMaintenance,
    LEDMaintenanceMovingCCW,
    LEDMaintenanceMovingCW,
    MotionMoveBy(i64),
    MqttPublishJson(String, String),
    UpdateNetworkMotionContext(MotionMode, f32),
    PerformOTA,
}

#[derive(Copy, Clone, Debug)]
pub enum FSMAddress {
    Motion,
    Network,
    Led,
    Maintenance,
}

impl Address for FSMAddress {
    fn index(self) -> usize {
        match self {
            FSMAddress::Motion => 0,
            FSMAddress::Network => 1,
            FSMAddress::Led => 2,
            FSMAddress::Maintenance => 3,
        }
    }

    fn count() -> usize {
        4
    }
}
