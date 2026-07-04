use core::time::Duration;

use embedded_hal::digital::OutputPin;

use crate::{Device, StepContext};

/// A [`Device`] which has step and direction pins.
///
/// Requires the `hal` feature.
#[derive(Debug, Default, Clone, PartialEq)]
pub struct StepAndDirection<Step, Direction> {
    step: Step,
    direction: Direction,
}

impl<Step, Direction> StepAndDirection<Step, Direction> {
    pub fn new(step: Step, direction: Direction) -> Self {
        StepAndDirection { step, direction }
    }

    pub fn into_inner(self) -> (Step, Direction) {
        (self.step, self.direction)
    }
}

fn set_output<P: OutputPin>(pin: &mut P, mask: u8) -> Result<(), P::Error> {
    if mask != 0 {
        pin.set_high()
    } else {
        pin.set_low()
    }
}

impl<Step, Direction, E> StepAndDirection<Step, Direction>
where
    Step: OutputPin<Error = E>,
    Direction: OutputPin<Error = E>,
{
    fn set_output(&mut self, mask: u8) -> Result<(), E> {
        set_output(&mut self.step, mask & 0b01)?;
        set_output(&mut self.direction, mask & 0b10)?;

        Ok(())
    }
}

impl<Step, Direction, E> Device for StepAndDirection<Step, Direction>
where
    Step: OutputPin<Error = E>,
    Direction: OutputPin<Error = E>,
{
    type Error = E;

    #[inline]
    fn step(&mut self, ctx: &StepContext) -> Result<(), Self::Error> {
        // copied straight from AccelStepper::step2()
        // println!("{}", ctx.position);
        let _ = self.set_output(if ctx.position > 0 { 0b10 } else { 0b00 });
        let _ = self.set_output(if ctx.position > 0 { 0b11 } else { 0b01 });
        // Many stepper drivers require a minimum STEP high pulse width (often a few microseconds).
        // Use a slightly more forgiving default to improve compatibility.
        std::thread::sleep(Duration::from_micros(5));
        self.set_output(if ctx.position > 0 { 0b10 } else { 0b00 })
    }
}
