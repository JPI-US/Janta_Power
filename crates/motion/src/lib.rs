use core::ops::Not;

use network::telemetry::Component;

pub mod motion {
    use std::time::Instant;

    use accel_stepper::{Driver, OperatingSystemClock, StepAndDirection};
    use anyhow::Result;
    use chrono::Local;
    use clock::Clock;
    use encoder::ENC_TICKS_PER_DEG;
    use esp_idf_svc::{
        hal::gpio::{Gpio10, Gpio11, Gpio14, Gpio15, Gpio16, Gpio17, Input, Output, PinDriver},
        nvs::*,
    };
    use log::{info, warn};
    use network::telemetry::Component;
    use quadrature_encoder::{IncrementalEncoder, QuadStep, Rotary};
    use semver::Version;

    use crate::{
        Direction,
        MotionEvent::{self},
    };

    // Focused motion modules.
    mod encoder;
    mod homing;
    mod move_exec;

    // Build-time constants generated from .env.
    include!(concat!(env!("OUT_DIR"), "/constants.rs"));

    // ESP-IDF NVS key names are limited to 15 characters.
    const NVS_KEY_HOME_ERROR_TICKS: &str = "home_err_ticks";

    // Keep step math in one place.
    // TODO: This should be in switchboard i think
    pub const STEPS_PER_REV: f64 = MICROSTEPS * GEAR_REDUCTION * SLEW_BEARING;

    #[derive(PartialEq, Copy, Clone, Debug)]
    pub enum MotionMode {
        // Stepper-only movement (open-loop).
        StepperOnly,
        // Stepper movement with encoder-based guardrails (stall detection, snapshots, etc.).
        EncoderGuarded,
    }

    #[derive(Debug, PartialEq, Clone)]
    pub enum MoveOutcome {
        Completed,
        AbortedPowerMissing,
        AbortedStall,
        AbortedOvershoot,
        AbortedErrorLoop(Component, String, String),
    }

    #[derive(Copy, Clone, Debug, PartialEq)]
    pub enum ActiveLevel {
        ActiveLow,
        ActiveHigh,
    }

    pub fn calculate_steps(offset_deg: f32) -> i64 {
        ((offset_deg as f64 / 360.0) * STEPS_PER_REV) as i64
    }

    pub struct Motion<'a> {
        pub location: f32,
        pub motion_mode: MotionMode,
        pub previous_motion_mode: MotionMode,
        pub speed: f32,
        pub acceleration: u16,
        pub motor: Driver,
        pub motor_device:
            StepAndDirection<PinDriver<'a, Gpio15, Output>, PinDriver<'a, Gpio16, Output>>,
        pub motor_clock: OperatingSystemClock,
        pub relay: PinDriver<'a, Gpio17, Output>,
        pub lmsw: PinDriver<'a, Gpio14, Input>,
        pub encoder: IncrementalEncoder<
            Rotary,
            PinDriver<'a, Gpio10, Input>,
            PinDriver<'a, Gpio11, Input>,
            QuadStep,
        >,
        // Encoder zero is a software offset: adjusted = raw - offset.
        pub encoder_zero_offset: i32,
        // Limit-switch debounce state (active-low switch).
        pub lmsw_last_state_pressed: bool,
        pub lmsw_last_change: Instant,
        pub lmsw_zeroed_this_press: bool,
        // captured when the limit switch is hit (before we re-zero).
        pub last_home_error_ticks: Option<i32>,

        // Stall detector state.
        pub motor_power_on: bool,
        pub stall_detection_enabled: bool,
        pub stall_last_check: Instant,
        pub stall_step_pos_at_last_enc_change: i64,
        pub stall_last_enc_ticks_seen: i32,
        pub stall_reported: bool,
        pub stall_consecutive: u8,

        // Ratio-based stall detection state (EncoderGuarded mode only).
        pub stall_check_start_encoder_ticks: i32,
        pub stall_check_start_step_pos: i64,
        pub stall_check_last_interval_step: i64,

        // Encoder overshoot protection state (EncoderGuarded mode only).
        pub overshoot_enc_start: Option<i32>,
        pub overshoot_expected_ticks: Option<i64>,

        // Last attempted move outcome, consumed by `take_last_move_outcome`.
        pub last_move_outcome: Option<MoveOutcome>,

        // Tracking soft limits.
        pub soft_limits_enabled: bool,
        pub soft_limit_min_deg: f32,
        pub soft_limit_max_deg: f32,

        // During homing, overshoot checks are disabled.
        pub is_homing: bool,
        pub need_rehome: bool,

        pub relay_active_level: ActiveLevel,
        pub limit_switch_active_level: ActiveLevel,
    }

    // Direction and step wiring notes:
    // - CW controls direction pin
    // - CCW steps by convention in this wiring
    impl Motion<'_> {
        pub fn new<'a>(
            step_pin: Gpio15,
            direction_pin: Gpio16,
            relay_pin: Gpio17,
            limit_switch_pin: Gpio14,
            encoder_a_pin: Gpio10,
            encoder_b_pin: Gpio11,
            relay_active_level: ActiveLevel,
            limit_switch_active_level: ActiveLevel,
        ) -> Result<Motion<'a>> {
            let step = PinDriver::output(step_pin)?;
            let direction = PinDriver::output(direction_pin)?;

            // Relay is active-low: boot with relay OFF.
            let mut relay = PinDriver::output(relay_pin)?;
            relay.set_high().unwrap_or_default();

            let encoder_a = PinDriver::input(encoder_a_pin).unwrap();
            let encoder_b = PinDriver::input(encoder_b_pin).unwrap();

            let mut lmsw = PinDriver::input(limit_switch_pin).unwrap();
            match limit_switch_active_level {
                ActiveLevel::ActiveHigh => lmsw
                    .set_pull(esp_idf_svc::hal::gpio::Pull::Down)
                    .unwrap_or_default(),
                ActiveLevel::ActiveLow => lmsw
                    .set_pull(esp_idf_svc::hal::gpio::Pull::Up)
                    .unwrap_or_default(),
            }

            let encoder = IncrementalEncoder::<Rotary, _, _, QuadStep>::new(encoder_a, encoder_b);

            let now = Instant::now();
            Ok(Motion {
                location: 0.0,
                motion_mode: MotionMode::EncoderGuarded,
                previous_motion_mode: MotionMode::EncoderGuarded,
                speed: DEFAULT_MAX_SPEED_STEPS_PER_S,
                acceleration: DEFAULT_ACCEL_STEPS_PER_S2,
                motor: Driver::new(),
                motor_device: StepAndDirection::new(step, direction),
                motor_clock: OperatingSystemClock::new(),
                relay,
                lmsw,
                encoder,
                encoder_zero_offset: 0,
                lmsw_last_state_pressed: false,
                lmsw_last_change: now,
                lmsw_zeroed_this_press: false,
                last_home_error_ticks: None,

                motor_power_on: true,
                stall_detection_enabled: true,
                stall_last_check: now,
                stall_step_pos_at_last_enc_change: 0,
                stall_last_enc_ticks_seen: 0,
                stall_reported: false,
                stall_consecutive: 0,

                stall_check_start_encoder_ticks: 0,
                stall_check_start_step_pos: 0,
                stall_check_last_interval_step: 0,

                overshoot_enc_start: None,
                overshoot_expected_ticks: None,

                last_move_outcome: None,

                soft_limits_enabled: false,
                soft_limit_min_deg: 0.0,
                soft_limit_max_deg: 290.0,

                is_homing: false,
                need_rehome: false,

                relay_active_level,
                limit_switch_active_level,
            })
        }

        pub fn update_position(&mut self, location: f32) {
            self.location = location;
        }

        pub fn set_stall_detection_enabled(&mut self, enabled: bool) {
            self.stall_detection_enabled = enabled;
        }

        pub fn stall_detection_enabled(&self) -> bool {
            self.stall_detection_enabled
        }

        pub fn set_soft_limits(&mut self, enabled: bool, min_deg: f32, max_deg: f32) {
            self.soft_limits_enabled = enabled;
            self.soft_limit_min_deg = min_deg;
            self.soft_limit_max_deg = max_deg;
        }

        pub fn set_motion_mode(&mut self, mode: MotionMode) {
            self.motion_mode = mode;
        }

        pub fn motion_mode(&self) -> MotionMode {
            self.motion_mode
        }

        pub fn set_motor_power_on(&mut self, power_on: bool) {
            self.motor_power_on = power_on;
        }

        pub fn location(&mut self) -> f32 {
            self.location
        }

        pub fn flip_relay(&mut self) {
            self.relay.toggle().unwrap_or_default();
        }

        #[inline]
        pub fn relay_on(&mut self) {
            // Active-low: LOW = ON
            if self.relay_active_level == ActiveLevel::ActiveLow {
                self.relay.set_low().unwrap_or_default();
            }
            // Active-high: HIGH = ON
            else {
                self.relay.set_high().unwrap_or_default();
            }
        }

        #[inline]
        pub fn relay_off(&mut self) {
            // Active-low: HIGH = OFF
            if self.relay_active_level == ActiveLevel::ActiveLow {
                self.relay.set_high().unwrap_or_default();
            }
            // Active-high: LOW = OFF
            else {
                self.relay.set_low().unwrap_or_default();
            }
        }

        pub fn lmsw_active(&self) -> bool {
            match self.limit_switch_active_level {
                ActiveLevel::ActiveHigh => self.lmsw.is_high(),
                ActiveLevel::ActiveLow => self.lmsw.is_low(),
            }
        }

        /// Consume the sunset-homing drift captured in `last_home_error_ticks`,
        /// publish it to `tower/{id}/data/encoder_error_ticks`, and (optionally)
        /// persist the raw tick count to NVS for post-reboot inspection.
        ///
        /// The encoder tick delta is categorized against
        /// `HOME_ERROR_ACCEPTABLE_DEG`:
        /// - `> +threshold`  → `undershoot`
        /// - `< -threshold`  → `overshoot`
        /// - otherwise       → `acceptable`
        ///
        /// Called from the two sunset-time homing sites inside
        /// `set_tower_position`. If no drift was captured this run, logs a
        /// warning and returns without publishing.
        pub fn report_home_error_ticks<T: NvsPartitionId>(
            &mut self,
            nvs: &mut EspNvs<T>,
            _device_id: &str,
            persist_nvs: bool,
        ) -> Option<MotionEvent> {
            let Some(home_error_ticks) = self.take_last_home_error_ticks() else {
                log::warn!("No home_error_ticks captured on this homing run");
                return None;
            };

            let error_deg = home_error_ticks as f32 / ENC_TICKS_PER_DEG;
            let (category, result) = if error_deg > HOME_ERROR_ACCEPTABLE_DEG {
                (
                    network::telemetry::EncoderErrorCategory::Undershoot,
                    format!(
                        "I undershot by: {:.2}° ({} ticks)",
                        error_deg, home_error_ticks
                    ),
                )
            } else if error_deg < -HOME_ERROR_ACCEPTABLE_DEG {
                (
                    network::telemetry::EncoderErrorCategory::Overshoot,
                    format!(
                        "I overshot by: {:.2}° ({} ticks)",
                        error_deg.abs(),
                        home_error_ticks
                    ),
                )
            } else {
                (
                    network::telemetry::EncoderErrorCategory::Acceptable,
                    format!(
                        "I performed within the acceptable range ({:.2}°, {} ticks)",
                        error_deg, home_error_ticks
                    ),
                )
            };

            if persist_nvs {
                match nvs.set_i32(NVS_KEY_HOME_ERROR_TICKS, home_error_ticks) {
                    Ok(()) => log::info!("Stored home_error_ticks in NVS: {}", home_error_ticks),
                    Err(e) => log::warn!("Failed to store home_error_ticks in NVS: {:?}", e),
                }
            } else {
                log::warn!("NVS persist disabled: skipping home_error_ticks store");
            }

            let now = Local::now()
                .format(network::telemetry::TIME_FORMAT)
                .to_string();
            let payload = network::telemetry::EncoderErrorTicks {
                current_time: now,
                encoder_error_ticks: home_error_ticks,
                category,
                result,
            };

            let event = MotionEvent::HomeErrorTicks(serde_json::to_string(&payload).unwrap());

            Some(event)
        }

        pub fn detect_stepper_transition(&mut self) {
            if self.motion_mode == MotionMode::StepperOnly
                && self.previous_motion_mode != MotionMode::StepperOnly
            {
                info!("Motion mode switched to StepperOnly - re-homing required");
                self.need_rehome = true;
            }
            self.previous_motion_mode = self.motion_mode;
        }

        pub fn is_rehome_pending(&mut self) -> anyhow::Result<bool> {
            if !(self.need_rehome && self.motion_mode == MotionMode::StepperOnly) {
                return Ok(false);
            }

            const HOMING_DIRECTION: Direction = Direction::Ccw;
            match HOMING_DIRECTION {
                Direction::Cw => {
                    warn!("CW homing requested, but firmware is only configured for CCW. Performing CCW home.");
                }
                Direction::Ccw => {}
            };

            Ok(true)
        }
    }

    pub struct TowerPositionCtx<'ctx, I2C, T>
    where
        I2C: embedded_hal::i2c::I2c,
        T: NvsPartitionId,
    {
        pub clock: &'ctx mut Clock<I2C>,
        pub nvs: &'ctx mut EspNvs<T>,
        pub current_version: Version,
        pub formatted_time: String,
        pub persist_nvs: bool,
        pub allow_ota: bool,
        pub device_id: &'ctx str,
    }
}

pub enum MotionEvent {
    Angle(String),
    HomeErrorTicks(String),
    ErrorLoop(Component, String, String),
    CheckForOTA,
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub enum Direction {
    Cw,
    Ccw,
}

impl Not for Direction {
    type Output = Self;

    fn not(self) -> Self::Output {
        match self {
            Direction::Cw => Direction::Ccw,
            Direction::Ccw => Direction::Cw,
        }
    }
}

impl Direction {
    pub const fn as_str(&self) -> &'static str {
        match self {
            Direction::Cw => "CW",
            Direction::Ccw => "CCW",
        }
    }

    /// Returns degrees with sign applied (CW positive, CCW negative).
    pub const fn apply_to_deg(&self, deg: f32) -> f32 {
        match self {
            Direction::Cw => deg,
            Direction::Ccw => -deg,
        }
    }
}
