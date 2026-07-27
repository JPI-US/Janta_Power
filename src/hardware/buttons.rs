use esp_idf_hal::{
    gpio::{Input, InputPin, PinDriver},
    sys::EspError,
};

pub struct Buttons<'a, M, Ccw, Cw>
where
    M: InputPin,
    Ccw: InputPin,
    Cw: InputPin,
{
    pub maintenance: PinDriver<'a, M, Input>,
    pub ccw: PinDriver<'a, Ccw, Input>,
    pub cw: PinDriver<'a, Cw, Input>,
}

impl<'a, M, Ccw, Cw> Buttons<'a, M, Ccw, Cw>
where
    M: InputPin,
    Ccw: InputPin,
    Cw: InputPin,
{
    pub fn new(maintenance: M, ccw: Ccw, cw: Cw) -> Result<Self, EspError> {
        Ok(Self {
            maintenance: PinDriver::input(maintenance)?,
            ccw: PinDriver::input(ccw)?,
            cw: PinDriver::input(cw)?,
        })
    }

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
