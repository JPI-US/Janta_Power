//! The status LED.
//!
//! Two states, and the second one exists for a specific reason. Normally the LED
//! follows the bulletin — maintenance colour, or dark. A diagnostics `LED_TEST`
//! has to override that, and an override only means anything if something stops
//! the normal state painting over it: this machine steps every 100 ms, so a test
//! colour with no override survives for a tenth of a second.

use fsm::{
    postal::{bulletin::Bulletin, mailbox::Mailbox},
    state::{InitialState, State, StateResult},
};
use rgb_led::{Led, RGB8};

use crate::logic::fsm::{DiagOutcome, DiagReply, DiagRequest, FSMAddress, FSMCommand, FSMState};

/// What the runtime wants the LED to show when no test is running.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum RuntimeLook {
    Maintenance,
    Idle,
}

pub struct LEDContext {
    led: Led<'static>,
    /// The runtime look currently on the LED, so it is driven on change rather
    /// than on every step.
    ///
    /// Not an optimisation. Re-driving unconditionally is what made a test colour
    /// impossible to hold, and "only write when the picture changes" is the
    /// property that makes the override below correct by construction rather than
    /// by everyone remembering it.
    shown: Option<RuntimeLook>,
}

impl LEDContext {
    pub fn new(led: Led<'static>) -> Self {
        Self { led, shown: None }
    }

    /// Paint the runtime look, if it is not already showing.
    fn show_runtime(&mut self, look: RuntimeLook) {
        if self.shown == Some(look) {
            return;
        }

        let outcome = match look {
            RuntimeLook::Maintenance => self.led.display_maintenance(),
            RuntimeLook::Idle => self.led.display_none(),
        };

        match outcome {
            Ok(()) => self.shown = Some(look),
            // Left unrecorded on failure, so the next step tries again rather than
            // believing a colour reached the LED when it did not.
            Err(e) => log::warn!("Could not drive the status LED: {e:?}"),
        }
    }
}

fn runtime_look(bulletin: &Bulletin<FSMState>) -> RuntimeLook {
    match bulletin.read() {
        Some(state) if state.maintenance_mode => RuntimeLook::Maintenance,
        _ => RuntimeLook::Idle,
    }
}

/// Answer a `LED_TEST`, or explain why not. Returns the colour actually driven.
fn drive_test(
    ctx: &mut LEDContext,
    mailbox: &Mailbox<FSMAddress, FSMCommand>,
    ticket: u32,
    colour: &str,
) -> Option<(u8, u8, u8)> {
    let Some((red, green, blue)) = board_diagnostics::led_test_color(colour) else {
        reply_done(
            mailbox,
            ticket,
            false,
            Vec::new(),
            format!(
                "LED_TEST: unknown colour '{}', expected one of {}, RESTORE, or an R,G,B triplet",
                colour,
                board_diagnostics::led_test_color_names()
            ),
        );
        return None;
    };

    if let Err(e) = ctx.led.set_color(RGB8::new(red, green, blue)) {
        reply_done(
            mailbox,
            ticket,
            false,
            Vec::new(),
            format!("LED_TEST could not drive the LED: {e:?}"),
        );
        return None;
    }

    // Anything the runtime had on the LED is gone now, so forget it — otherwise
    // returning to the runtime look would decide it was already showing.
    ctx.shown = None;

    reply_progress(
        mailbox,
        ticket,
        format!("LED driven {colour} rgb {red},{green},{blue}"),
    );

    // Deliberately reports that the colour was *written*, never that the LED
    // works. A WS2812 has no readback: a dead LED, a broken joint and a wrong pin
    // are indistinguishable from here, so only the person looking at the board can
    // judge it.
    reply_done(
        mailbox,
        ticket,
        true,
        vec![
            (String::from("color"), colour.to_uppercase()),
            (String::from("rgb"), format!("{red},{green},{blue}")),
        ],
        String::new(),
    );

    Some((red, green, blue))
}

fn reply_progress(mailbox: &Mailbox<FSMAddress, FSMCommand>, ticket: u32, line: String) {
    let _ = mailbox.send(
        FSMAddress::Diagnostics,
        FSMCommand::DiagnosticReply(DiagReply {
            ticket,
            outcome: DiagOutcome::Progress(line),
        }),
    );
}

fn reply_done(
    mailbox: &Mailbox<FSMAddress, FSMCommand>,
    ticket: u32,
    passed: bool,
    details: Vec<(String, String)>,
    message: String,
) {
    let _ = mailbox.send(
        FSMAddress::Diagnostics,
        FSMCommand::DiagnosticReply(DiagReply {
            ticket,
            outcome: DiagOutcome::Done {
                passed,
                details,
                message,
            },
        }),
    );
}

/// Pull the next `LED_TEST` out of the mailbox, discarding what is not ours.
fn next_led_test(mailbox: &Mailbox<FSMAddress, FSMCommand>) -> Option<(u32, String)> {
    while let Ok(message) = mailbox.receive() {
        let FSMCommand::DiagnosticRequest(DiagRequest { ticket, command }) = message else {
            log::warn!("LED machine has no handler for a message; discarding it");
            continue;
        };

        match command {
            board_diagnostics::DiagnosticCommand::LedTest { color } => {
                return Some((ticket, color));
            }
            other => {
                // Routed here in error. Answered rather than dropped, so the
                // diagnostics machine is not left waiting out its budget for a
                // reply that was never going to come.
                log::warn!("LED machine was sent {other:?}, which it does not own");
                reply_done(
                    mailbox,
                    ticket,
                    false,
                    Vec::new(),
                    String::from("routed to the LED machine, which does not own that hardware"),
                );
            }
        }
    }

    None
}

pub struct LEDCheck;

/// Holding a colour a technician asked for, ignoring the runtime look.
pub struct LEDDiagnosticHold;

impl InitialState<FSMAddress, LEDContext, FSMCommand, FSMState> for LEDCheck {}

impl State<FSMAddress, LEDContext, FSMCommand, FSMState> for LEDCheck {
    fn process(
        &mut self,
        ctx: &mut LEDContext,
        mailbox: &mut Mailbox<FSMAddress, FSMCommand>,
        bulletin: &Bulletin<FSMState>,
        _previous_state: Option<
            Box<dyn State<FSMAddress, LEDContext, FSMCommand, FSMState> + Send>,
        >,
    ) -> anyhow::Result<StateResult<FSMAddress, LEDContext, FSMCommand, FSMState>> {
        if let Some((ticket, colour)) = next_led_test(mailbox) {
            if colour.eq_ignore_ascii_case("RESTORE") {
                // Already following the runtime; nothing to restore.
                reply_progress(
                    mailbox,
                    ticket,
                    String::from("LED restored to the runtime status colour"),
                );
                reply_done(
                    mailbox,
                    ticket,
                    true,
                    vec![(String::from("color"), String::from("RESTORE"))],
                    String::new(),
                );
            } else if drive_test(ctx, mailbox, ticket, &colour).is_some() {
                return Ok(StateResult::Running(Box::new(LEDDiagnosticHold)));
            }
        }

        ctx.show_runtime(runtime_look(bulletin));
        Ok(StateResult::Hold)
    }
}

impl State<FSMAddress, LEDContext, FSMCommand, FSMState> for LEDDiagnosticHold {
    fn process(
        &mut self,
        ctx: &mut LEDContext,
        mailbox: &mut Mailbox<FSMAddress, FSMCommand>,
        bulletin: &Bulletin<FSMState>,
        _previous_state: Option<
            Box<dyn State<FSMAddress, LEDContext, FSMCommand, FSMState> + Send>,
        >,
    ) -> anyhow::Result<StateResult<FSMAddress, LEDContext, FSMCommand, FSMState>> {
        // Maintenance wins, always. The LED is how a person standing at the tower
        // knows it is in maintenance, and a test colour must never be able to hide
        // that — whether or not anyone remembers to send `RESTORE`.
        if runtime_look(bulletin) == RuntimeLook::Maintenance {
            log::info!("LED test ended: the tower entered maintenance");
            return Ok(StateResult::Running(Box::new(LEDCheck)));
        }

        if let Some((ticket, colour)) = next_led_test(mailbox) {
            if colour.eq_ignore_ascii_case("RESTORE") {
                reply_progress(
                    mailbox,
                    ticket,
                    String::from("LED restored to the runtime status colour"),
                );
                reply_done(
                    mailbox,
                    ticket,
                    true,
                    vec![(String::from("color"), String::from("RESTORE"))],
                    String::new(),
                );
                return Ok(StateResult::Running(Box::new(LEDCheck)));
            }

            // A different colour: stay here and hold the new one.
            drive_test(ctx, mailbox, ticket, &colour);
        }

        // Deliberately does not touch the LED. Holding the colour *is* the state.
        Ok(StateResult::Hold)
    }
}
