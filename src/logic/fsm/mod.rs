use ::motion::motion::MotionMode;

pub mod led;
pub mod maintenance;
pub mod motion;
pub mod network;
pub mod startup;

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
