use core::time::Duration;

use anyhow::Result;
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

    pub fn set_color(&mut self, rgb: RGB8) -> Result<()> {
        let color: u32 = ((rgb.g as u32) << 16) | ((rgb.r as u32) << 8) | rgb.b as u32;
        let ticks_hz = self.tx_rtm_driver.counter_clock()?;
        // A WS2812 tells a 1 from a 0 by how long the line is held high, splitting
        // them at roughly 500 ns. These are chosen to sit as far from that boundary
        // as the part's own timing windows allow, because the margin is what this
        // board is short of: the LED is a 5 V part driven from a 3.3 V GPIO, so its
        // logic-high threshold is only just met and every edge arrives slower than
        // the datasheet assumes.
        //
        // The previous values were 350/800/700/600, which left about 150 ns either
        // side. That is what produced the scrambled colours seen on the bench —
        // red arriving as yellow, blue as purple, and "off" glowing faintly — since
        // a single misread bit shifts a whole channel.
        //
        // 300 and 850 put ~200 ns and ~350 ns between the two cases. Both stay
        // inside the WS2812B window (T0H 0.4 us +/- 150 ns, T1H 0.8 us +/- 150 ns)
        // and the 1.25 us +/- 600 ns bit period holds for both.
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
        self.tx_rtm_driver.start_blocking(&signal)?;

        Ok(())
    }

    pub fn display_warning(&mut self) -> Result<()> {
        self.set_color(RGB8::new(255, 222, 33))
    }

    pub fn display_healthy(&mut self) -> Result<()> {
        self.set_color(RGB8::new(0, 255, 0))
    }

    pub fn display_error(&mut self) -> Result<()> {
        self.set_color(RGB8::new(255, 0, 0))
    }

    /// Display pure blue.
    /// Indicates that we are in maintenance mode and nothing is moving.
    pub fn display_maintenance(&mut self) -> Result<()> {
        self.set_color(RGB8::new(0, 0, 255))
    }

    /// Display magenta.
    /// Indicates that we are in maintenance mode and motors are moving CW.
    pub fn display_maintenance_moving_cw(&mut self) -> Result<()> {
        self.set_color(RGB8::new(150, 0, 255))
    }

    /// Display teal.
    /// Indicates that we are in maintenance mode and motors are moving CCW.
    pub fn display_maintenance_moving_ccw(&mut self) -> Result<()> {
        self.set_color(RGB8::new(0, 150, 255))
    }

    pub fn display_connecting(&mut self) -> Result<()> {
        self.set_color(RGB8::new(255, 10, 200))?;
        std::thread::sleep(std::time::Duration::from_millis(350));
        self.set_color(RGB8::new(150, 150, 150))?;
        std::thread::sleep(std::time::Duration::from_millis(350));
        Ok(())
    }

    /// Turn off the LED
    pub fn display_none(&mut self) -> Result<()> {
        self.set_color(RGB8::new(0, 0, 0))
    }
}

fn ns(nanos: u64) -> Duration {
    Duration::from_nanos(nanos)
}
