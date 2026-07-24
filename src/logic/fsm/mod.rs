use ::motion::motion::MotionMode;
use fsm::postal::Address;

pub mod led;
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
    Maintenance,
}

impl Address for FSMAddress {
    fn index(self) -> usize {
        match self {
            FSMAddress::Motion => 0,
            FSMAddress::Network => 1,
            FSMAddress::Maintenance => 2,
        }
    }

    fn count() -> usize {
        3
    }
}
