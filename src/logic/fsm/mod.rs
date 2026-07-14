pub mod led;
pub mod maintenance;
pub mod motion;
pub mod startup;

#[derive(Debug, Clone, Copy)]
pub enum FSMCommand {
    LEDOff,
    LEDMaintenance,
    LEDMaintenanceMovingCCW,
    LEDMaintenanceMovingCW,
    MotionMoveBy(i64),
}
