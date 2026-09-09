use clock::Clock;
use fsm::{
    postal::{bulletin::Bulletin, mailbox::Mailbox},
    state::{InitialState, State, StateResult},
};
use log::info;
use motion::motion::MotionMode;

use crate::{
    logic::fsm::{
        motion::{check_daily_encoder_reset, MotionBeginHoming, MotionContext, MotionInit},
        FSMAddress,
        FSMCommand::{self},
        FSMState,
    },
    storage::snapshot_store::SnapshotStore,
};

const POWER_ON: bool = true;
const PERSIST_NVS: bool = true;

impl InitialState<FSMAddress, MotionContext, FSMCommand, FSMState> for MotionInit {}

impl State<FSMAddress, MotionContext, FSMCommand, FSMState> for MotionInit {
    fn process(
        &mut self,
        ctx: &mut MotionContext,
        _mailbox: &mut Mailbox<FSMAddress, FSMCommand>,
        _bulletin: &Bulletin<FSMState>,
        _previous_state: Option<
            Box<dyn State<FSMAddress, MotionContext, FSMCommand, FSMState> + Send>,
        >,
    ) -> anyhow::Result<StateResult<FSMAddress, MotionContext, FSMCommand, FSMState>> {
        // Tower location — read directly from `TOWER_LATITUDE` / `TOWER_LONGITUDE`from switchboard
        let latitude: f64 = ctx.switchboard.default_tower_latitude;
        let longitude: f64 = ctx.switchboard.default_tower_longitude;
        let altitude: f64 = ctx.switchboard.default_tower_altitude;

        info!(
            "Retrieved latitude: {}, and longitude: {}",
            latitude, longitude
        );

        info!(
            "Device: {}, Lat: {}, Lon: {}, Alt: {}",
            ctx.switchboard.device_id, latitude, longitude, altitude
        );

        ctx.calculation = Some(Clock::new(
            ctx.i2c_bus.acquire_i2c(),
            latitude,
            longitude,
            altitude,
        ));

        // Daily encoder mode reset before mode load
        check_daily_encoder_reset(&mut ctx.nvs, &rtc::timezone::local_time(), PERSIST_NVS);

        // Motion mode from NVS, default EncoderGuarded
        let motion_mode = SnapshotStore::new(&mut ctx.nvs, PERSIST_NVS)
            .load_tracking_mode_or_init(MotionMode::EncoderGuarded);

        ctx.motion.set_motion_mode(motion_mode);
        ctx.motion.set_motor_power_on(POWER_ON);
        info!(
            "Motion mode: {:?}",
            match motion_mode {
                MotionMode::StepperOnly => "StepperOnly",
                MotionMode::EncoderGuarded => "EncoderGuarded",
            }
        );

        // State restoration
        ctx.actual_heading = if ctx.trust_nvs_state {
            let h = SnapshotStore::new(&mut ctx.nvs, PERSIST_NVS)
                .load_heading_or_init(ctx.switchboard.home_heading_deg);
            info!("Restored heading from NVS: {}", h);
            h
        } else {
            info!("Skipping heading restore: NVS state untrusted");
            ctx.switchboard.home_heading_deg
        };

        // Keep motion state aligned with restored heading.
        ctx.motion.update_position(ctx.actual_heading);

        // Restore encoder snapshot only in EncoderGuarded mode.
        if ctx.trust_nvs_state && motion_mode == MotionMode::EncoderGuarded {
            if let Some(enc_ticks_adj) =
                SnapshotStore::new(&mut ctx.nvs, PERSIST_NVS).load_encoder_snapshot()
            {
                // Restore zero offset so adjusted ticks equal saved snapshot.
                let raw = ctx.motion.encoder_ticks_raw();
                ctx.motion.set_encoder_zero_offset(raw - enc_ticks_adj);
                info!(
                    "Restored encoder snapshot ticks from NVS: {}",
                    enc_ticks_adj
                );
                ctx.restored_from_snapshot = true;
            } else {
                info!("No valid encoder snapshot found in NVS; will home normally.");
            }
        } else {
            if !ctx.trust_nvs_state {
                info!("Skipping encoder snapshot restore: NVS state untrusted");
            } else {
                info!("Motion mode is StepperOnly: skipping encoder snapshot restore");
            }
        }

        // Encoder fault recovery

        let encoder_daily_mode =
            SnapshotStore::new(&mut ctx.nvs, PERSIST_NVS).load_encoder_daily_mode();
        ctx.encoder_fault
            .set_mode_switched_daily(encoder_daily_mode);

        // clock
        ctx.clock = Some(Clock::new(
            ctx.i2c_bus.acquire_i2c(),
            latitude,
            longitude,
            altitude,
        ));

        Ok(StateResult::Running(Box::new(MotionBeginHoming)))
    }
}
