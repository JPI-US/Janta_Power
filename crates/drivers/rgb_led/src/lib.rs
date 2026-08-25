use anyhow::Result;
use core::time::Duration;
use esp_idf_svc::hal::{
    gpio::OutputPin,
    peripheral::Peripheral,
    rmt::{config::TransmitConfig, FixedLengthSignal, PinState, Pulse, RmtChannel, TxRmtDriver},
};

pub use rgb::RGB8;

pub struct Led<'a> {
    tx_rtm_driver: TxRmtDriver<'a>,
}

impl<'d> Led<'d> {
    // Rust ESP Board gpio2,  ESP32-C3-DevKitC-02 gpio8
    pub fn new(
        led: impl Peripheral<P = impl OutputPin> + 'd,
        channel: impl Peripheral<P = impl RmtChannel> + 'd,
    ) -> Result<Self> {
        let config = TransmitConfig::new().clock_divider(2);
        let tx = TxRmtDriver::new(channel, led, &config)?;
        Ok(Self { tx_rtm_driver: tx })
    }

    /// Number of times each frame is sent.
    ///
    /// Worth being precise about what this buys, because the obvious reasoning is
    /// wrong: the LED displays whatever the *last* frame it latched said, and the
    /// last frame is no likelier to be correct than any other. Repetition therefore
    /// does **not** reduce the odds of a wrong colour from a corrupted-but-latched
    /// frame.
    ///
    /// It helps only in the other failure mode — a frame mangled badly enough that
    /// the part does not latch it at all, where another attempt is another chance.
    /// Kept for that, and because three frames cost about a millisecond.
    const FRAME_REPEATS: usize = 3;

    pub fn set_color(&mut self, rgb: RGB8) -> Result<()> {
        let color: u32 = ((rgb.g as u32) << 16) | ((rgb.r as u32) << 8) | rgb.b as u32;
        let ticks_hz = self.tx_rtm_driver.counter_clock()?;

        // Chosen to maximise the gap either side of the ~500 ns the part uses to
        // tell a 1 from a 0, rather than to sit on the datasheet's nominal figures.
        // T0H is at the short end of its 250-550 ns window and T1H near the long end
        // of its 650-950 ns one, so a pulse has to be distorted much further before
        // it is misread.
        //
        // The direction matters: a 0 misread as a 1 is what shows up as a faint
        // glow when the LED is asked to go dark, and shortening T0H is what buys
        // margin against it. (Originally T0H 350 / T1H 700, which left a 1 bit only
        // just above the threshold; then 400/800, which helped the 1s and cost the
        // 0s.)
        let t0h = Pulse::new_with_duration(ticks_hz, PinState::High, &ns(300))?;
        let t0l = Pulse::new_with_duration(ticks_hz, PinState::Low, &ns(850))?;
        let t1h = Pulse::new_with_duration(ticks_hz, PinState::High, &ns(850))?;
        let t1l = Pulse::new_with_duration(ticks_hz, PinState::Low, &ns(450))?;

        let mut signal = FixedLengthSignal::<24>::new();
        for i in (0..24).rev() {
            let p = 2_u32.pow(i);
            let bit = p & color != 0;
            let (high_pulse, low_pulse) = if bit { (t1h, t1l) } else { (t0h, t0l) };
            signal.set(23 - i as usize, &(high_pulse, low_pulse))?;
        }

        for repeat in 0..Self::FRAME_REPEATS {
            self.tx_rtm_driver.start_blocking(&signal)?;
            // The part latches on a low of >50 us. The RMT line idles low, so this
            // only has to separate one frame from the next.
            if repeat + 1 < Self::FRAME_REPEATS {
                std::thread::sleep(core::time::Duration::from_micros(300));
            }
        }

        Ok(())
    }

    pub fn display_warning(&mut self) {
        self.set_color(RGB8::new(255, 222, 33));
    }

    pub fn display_healthy(&mut self) {
        self.set_color(RGB8::new(0, 255, 0));
    }

    pub fn display_error(&mut self) {
        self.set_color(RGB8::new(255, 0, 0));
    }

    pub fn display_maintenance(&mut self) {
        self.set_color(RGB8::new(0, 0, 255));
    }

    pub fn display_connecting(&mut self) {
        self.set_color(RGB8::new(255, 10, 200));
        std::thread::sleep(std::time::Duration::from_millis(350));
        self.set_color(RGB8::new(150, 150, 150));
        std::thread::sleep(std::time::Duration::from_millis(350));
    }
}

fn ns(nanos: u64) -> Duration {
    Duration::from_nanos(nanos)
}
