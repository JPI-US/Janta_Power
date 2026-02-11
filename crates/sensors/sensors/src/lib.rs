pub mod sensors {
    use esp_idf_svc::hal::adc::{
        AdcContConfig, AdcContDriver, AdcMeasurement, Attenuated, EmptyAdcChannels, ADC1,
    };
    use esp_idf_svc::hal::delay::Ets;
    use esp_idf_svc::hal::gpio::{Gpio2, Gpio3};
    use hdc1080::Hdc1080;

    pub struct Sensors<'a, I2C> {
        humidity_sensor: Hdc1080<I2C, Ets>,
        light_sensor: AdcContDriver<'a>,
    }

    impl<I2C> Sensors<'_, I2C>
    where
        I2C: embedded_hal::i2c::I2c,
    {
        pub fn new<'a>(bus: I2C, adc: ADC1, ldr_e: Gpio2, ldr_w: Gpio3) -> Sensors<'a, I2C> {
            let att_e = Attenuated::db11(ldr_e);
            let att_w = Attenuated::db11(ldr_w);

            let temp_channel = EmptyAdcChannels::chain(att_e);
            let adc_channels = temp_channel.chain(att_w);

            let adc_config = AdcContConfig::default();
            let mut driver = AdcContDriver::new(adc, &adc_config, adc_channels).unwrap();
            driver.start().unwrap();

            Sensors {
                humidity_sensor: Hdc1080::new(bus, Ets).unwrap(),
                light_sensor: driver,
            }
        }

        pub fn temperature(&mut self) -> f32 {
            (self.humidity_sensor.temperature().unwrap_or_default() * 9.0 / 5.0) + 32.0
        }

        pub fn humidity(&mut self) -> f32 {
            self.humidity_sensor.humidity().unwrap_or_default()
        }

        pub fn east_ldr(&mut self) -> i32 {
            let mut samples: [AdcMeasurement; 128] = [Default::default(); 128];
            if let Ok(_) = self.light_sensor.read(&mut samples, 128) {
                return samples[0].data() as i32;
            }
            -1
        }

        pub fn west_ldr(&mut self) -> i32 {
            let mut samples: [AdcMeasurement; 128] = [Default::default(); 128];
            if let Ok(_) = self.light_sensor.read(&mut samples, 128) {
                return samples[1].data() as i32;
            }
            -1
        }

        pub fn balance_gap(&mut self) -> i32 {
            let mut samples: [AdcMeasurement; 128] = [Default::default(); 128];
            if let Ok(_) = self.light_sensor.read(&mut samples, 128) {
                return (samples[0].data() as i32 - samples[1].data() as i32) as i32;
            }
            -10000
        }
    }
}

pub use sensors::Sensors;
