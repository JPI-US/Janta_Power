use core::{option::Option::None, time::Duration};

use ::fsm::{
    postal::{bulletin::Bulletin, mailbox::Mailbox},
    state::{State, StateResult},
};
use chrono::Local;
use esp_idf_svc::hal::gpio::{InputPin, OutputPin};
use network::telemetry::{topic, ErrorLog, Severity};

use crate::logic::fsm::{
    motion::{MotionContext, MotionErrorLoop},
    FSMAddress,
    FSMCommand::{self, MqttPublishJson},
    FSMState,
};

impl<SLP, STP, DIR, CS, RLY, LMSW, ENCA, ENCB>
    State<FSMAddress, MotionContext<SLP, STP, DIR, CS, RLY, LMSW, ENCA, ENCB>, FSMCommand, FSMState>
    for MotionErrorLoop
where
    SLP: OutputPin,
    STP: OutputPin,
    DIR: OutputPin,
    CS: OutputPin,
    RLY: OutputPin,
    LMSW: InputPin + OutputPin,
    ENCA: InputPin,
    ENCB: InputPin,
{
    fn process(
        &mut self,
        ctx: &mut MotionContext<SLP, STP, DIR, CS, RLY, LMSW, ENCA, ENCB>,
        mailbox: &mut Mailbox<FSMAddress, FSMCommand>,
        _bulletin: &Bulletin<FSMState>,
        _previous_state: Option<
            Box<
                dyn State<
                        FSMAddress,
                        MotionContext<SLP, STP, DIR, CS, RLY, LMSW, ENCA, ENCB>,
                        FSMCommand,
                        FSMState,
                    > + Send,
            >,
        >,
    ) -> anyhow::Result<
        StateResult<
            FSMAddress,
            MotionContext<SLP, STP, DIR, CS, RLY, LMSW, ENCA, ENCB>,
            FSMCommand,
            FSMState,
        >,
    > {
        let now = Local::now()
            .format(network::telemetry::TIME_FORMAT)
            .to_string();

        let payload = ErrorLog {
            current_time: now.as_str(),
            log_type: "error",
            message: self.message.as_str(),
            component: self.component,
            severity: Severity::Fault,
            value: None,
            unit: None,
            notes: self.notes.as_str(),
        };
        let serialized = serde_json::to_string(&payload)?;
        let topic = topic::logs_error(ctx.switchboard.device_id);
        let _ = mailbox.send(FSMAddress::Network, MqttPublishJson(serialized, topic));

        std::thread::sleep(Duration::from_mins(15));
        Ok(StateResult::Hold)
    }
}
