pub mod clock {
    use chrono::prelude::*;
    use chrono::MappedLocalTime;
    use chrono::Utc;
    use ds323x::{DateTimeAccess, Ds323x, NaiveDate, Rtcc};

    pub struct Clock<I2C> {
        rtc: Ds323x<I2C>,
        latitude: f64,
        longitude: f64,
        altitude: f64,
    }

    impl<I2C> Clock<I2C>
    where
        I2C: embedded_hal::i2c::I2c,
    {
        // Constructor for Clock
        pub fn new(i2c: I2C, latitude: f64, longitude: f64, altitude: f64) -> Clock<I2C> {
            Clock {
                rtc: Ds323x::new_ds3231(i2c),
                latitude,
                longitude,
                altitude,
            }
        }

        /// Calculate sunrise and sunset times in UTC
        pub fn sunrise_times(&mut self) -> Option<DateTime<FixedOffset>> {
            //Calculate date in utc

            let times = sun_times::sun_times(
                self.rtc.date().unwrap(),
                self.latitude,
                self.longitude,
                self.altitude,
            );

            match times {
                Some((sunrise, _sunset)) => Some(DateTime::from_naive_utc_and_offset(
                    sunrise.naive_utc(),
                    FixedOffset::west_opt(5 * 3600).unwrap(),
                )),
                None => None,
            }
        }

        pub fn sunset_times(&mut self) -> Option<DateTime<FixedOffset>> {
            //Calculate date in utc
            let prop_date = self.rtc.date().unwrap();

            let year = prop_date.year();
            let month = prop_date.month();
            let day = prop_date.day();

            let date = NaiveDate::from_ymd_opt(year as i32, month as u32, day as u32)
                .expect("Invalid date provided");

            let times = sun_times::sun_times(date, self.latitude, self.longitude, self.altitude);
            match times {
                Some((_sunrise, sunset)) => Some(DateTime::from_naive_utc_and_offset(
                    sunset.naive_utc(),
                    FixedOffset::west_opt(5 * 3600).unwrap(),
                )),
                None => None, // Handle the case where `None` is returned
            }
        }

        /// Method to get the hours
        pub fn get_hour(&mut self) -> u8 {
            let hour = self.rtc.hours().unwrap();
            match hour {
                ds323x::Hours::AM(h) => h,
                ds323x::Hours::PM(h) => h + 11,
                ds323x::Hours::H24(h) => h,
            }
        }

        /// Method to get the minutes
        pub fn get_minutes(&mut self) -> u8 {
            self.rtc.minutes().unwrap()
        }

        /// Method to get the seconds
        pub fn get_seconds(&mut self) -> u8 {
            self.rtc.seconds().unwrap()
        }

        /// Method to get the day
        pub fn get_day(&mut self) -> u32 {
            self.rtc.date().unwrap().ordinal()
        }

        /// Method to get the day
        pub fn get_month(&mut self) -> u8 {
            self.rtc.month().unwrap()
        }

        /// Method to get the day
        pub fn get_year(&mut self) -> u16 {
            self.rtc.year().unwrap()
        }

        /// Method to get the longitude
        pub fn get_longitude(&mut self) -> f64 {
            self.longitude
        }

        /// Method to get the latitude
        pub fn get_latitude(&mut self) -> f64 {
            self.latitude
        }

        /// Method to get the altitude
        pub fn get_altitude(&mut self) -> f64 {
            self.altitude
        }

        /// Method for setting a datetime string
        pub fn set_date_time(&mut self, dateTime: &NaiveDateTime) {
            self.rtc.set_datetime(dateTime);
        }

        /// Method for returning a datetime string
        pub fn get_date_time(&mut self) -> NaiveDateTime {
            self.rtc.datetime().unwrap()
        }

        /// Method for returning a boolean for if it is after sunrsie today
        pub fn after_sunrise(&mut self) -> bool {
            if let Some(sunrise) = self.sunrise_times() {
                let current_time: MappedLocalTime<DateTime<FixedOffset>> = self
                    .get_date_time()
                    .and_local_timezone(FixedOffset::west_opt(5 * 3600).unwrap());
                // println!("{:?}", current_time);
                current_time.single().unwrap() >= sunrise
            } else {
                false // Return false if sunrise is None
            }
        }

        /// Method for returning a boolean for if it is after sunset today
        pub fn after_sunset(&mut self) -> bool {
            if let Some(sunset) = self.sunset_times() {
                let current_time: MappedLocalTime<DateTime<FixedOffset>> = self
                    .get_date_time()
                    .and_local_timezone(FixedOffset::west_opt(5 * 3600).unwrap());
                // println!("{:?}", current_time);
                current_time.single().unwrap() >= sunset
            } else {
                false // Return false if sunset is None
            }
        }

        ///Returns a unix timestamp based on the current date time provided
        pub fn datetime_to_unix_timestamp(&mut self) -> i64 {
            let current_time: MappedLocalTime<DateTime<FixedOffset>> = self
                .get_date_time()
                .and_local_timezone(FixedOffset::west_opt(5 * 3600).unwrap());
            let unix_timestamp = current_time.single().unwrap().timestamp();
            unix_timestamp
        }
    }
}

pub use clock::Clock;
