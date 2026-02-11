pub mod buttons {
    use button_driver::{Button, ButtonConfig};
    use esp_idf_svc::hal::gpio::{Gpio4, Gpio5, Gpio6, Input, PinDriver};
    use std::time::{Duration, Instant};

    pub struct Buttons<'a> {
        m_button: Button<PinDriver<'a, Gpio5, Input>, Instant, Duration>,
        e_button: Button<PinDriver<'a, Gpio4, Input>, Instant, Duration>,
        w_button: Button<PinDriver<'a, Gpio6, Input>, Instant, Duration>,
    }

    impl Buttons<'_> {
        pub fn new<'a>(mb: Gpio5, eb: Gpio4, wb: Gpio6) -> Buttons<'a> {
            Buttons {
                m_button: Button::<_, Instant>::new(
                    PinDriver::input(mb).unwrap(),
                    ButtonConfig::default(),
                ),
                e_button: Button::<_, Instant>::new(
                    PinDriver::input(eb).unwrap(),
                    ButtonConfig::default(),
                ),
                w_button: Button::<_, Instant>::new(
                    PinDriver::input(wb).unwrap(),
                    ButtonConfig::default(),
                ),
            }
        }

        pub fn is_maintenance_pressed(&mut self) -> bool {
            self.m_button.is_clicked()
        }

        pub fn maintenance_double(&mut self) -> bool {
            self.m_button.is_double_clicked()
        }

        pub fn is_east_pressed(&mut self) -> bool {
            self.e_button.is_clicked()
        }

        pub fn is_west_pressed(&mut self) -> bool {
            self.w_button.is_clicked()
        }

        pub fn tick(&mut self) {
            self.m_button.tick();
            self.e_button.tick();
            self.w_button.tick();
        }
    }
}

pub use buttons::Buttons;
