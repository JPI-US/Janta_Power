use esp_idf_hal::gpio::{Input, Pin, PinDriver};

pub struct Buttons<'a, M, Ccw, Cw>
where
    M: Pin,
    Ccw: Pin,
    Cw: Pin,
{
    pub maintenance: PinDriver<'a, M, Input>,
    pub ccw: PinDriver<'a, Ccw, Input>,
    pub cw: PinDriver<'a, Cw, Input>,
}

impl<'a, M, Ccw, Cw> Buttons<'a, M, Ccw, Cw>
where
    M: Pin,
    Ccw: Pin,
    Cw: Pin,
{
    pub fn maintenance_pressed(&self) -> bool {
        self.maintenance.is_high()
    }

    pub fn ccw_pressed(&self) -> bool {
        self.ccw.is_high()
    }

    pub fn cw_pressed(&self) -> bool {
        self.cw.is_high()
    }
}
