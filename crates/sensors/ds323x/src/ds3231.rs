//! Functions exclusive of DS3231

use crate::{BitFlags, Ds323x, Error, CONTROL_POR_VALUE, DEVICE_ADDRESS};

impl<I2C> Ds323x<I2C>
where
    I2C: embedded_hal::i2c::I2c,
{
    /// Create a new instance of the DS3231 device.
    pub fn new_ds3231(i2c: I2C) -> Self {
        const STATUS_POR_VALUE: u8 = BitFlags::OSC_STOP | BitFlags::EN32KHZ;
        Ds323x {
            i2c: i2c,
            control: CONTROL_POR_VALUE,
            status: STATUS_POR_VALUE,
        }
    }

    pub fn write_register(&mut self, register: u8, data: u8) -> Result<(), Error> {
        let payload: [u8; 2] = [register, data];
        self.i2c
            .write(DEVICE_ADDRESS, &payload)
            .map_err(|_err| Error::Comm);
        Ok(())
    }

    pub fn write_data(&mut self, payload: &mut [u8]) -> Result<(), Error> {
        self.i2c
            .write(DEVICE_ADDRESS, payload)
            .map_err(|_err| Error::Comm);
        Ok(())
    }

    pub fn read_register(&mut self, register: u8) -> Result<u8, Error> {
        let mut data = [0];
        self.i2c
            .write_read(DEVICE_ADDRESS, &[register], &mut data)
            .map_err(|_err| Error::Comm);
        Ok(data[0])
    }

    pub fn read_data(&mut self, payload: &mut [u8]) -> Result<(), Error> {
        let len = payload.len();
        self.i2c
            .write_read(DEVICE_ADDRESS, &[payload[0]], &mut payload[1..len])
            .map_err(|_err| Error::Comm);
        Ok(())
    }
}
