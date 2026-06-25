pub mod clock {
    use chrono::prelude::*;
    use ds323x::{DateTimeAccess, Ds323x, Rtcc};

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

        /// Calculate sunrise time and represent it in local timezone.
        ///
        /// Astronomy lookup needs the LOCAL civil date at the tower, not the
        /// UTC date the DS3231 holds. For any longitude west of Greenwich the
        /// two dates diverge after UTC-midnight (19:00 CDT / 18:00 CST at
        /// Sadler); passing the UTC date causes `sun_times` to return
        /// tomorrow's events, which flips `after_sunrise` back to `false` and
        /// trips the sunset-home branch roughly an hour before real sunset.
        /// `Local::now()` reads the system clock (seeded from the DS3231 at
        /// boot) and applies the `TZ=CST6CDT,...` rules set via `tzset`, so
        /// it's DST-aware without any extra plumbing and never touches Wi-Fi.
        pub fn sunrise_times(&mut self) -> Option<DateTime<Local>> {
            let date = Local::now().date_naive();
            let times = sun_times::sun_times(date, self.latitude, self.longitude, self.altitude);
            match times {
                Some((sunrise, _sunset)) => Some(sunrise.with_timezone(&Local)),
                None => None,
            }
        }

        /// Calculate sunset time and represent it in local timezone.
        /// See `sunrise_times` for the UTC-vs-local date rationale.
        pub fn sunset_times(&mut self) -> Option<DateTime<Local>> {
            let date = Local::now().date_naive();
            let times = sun_times::sun_times(date, self.latitude, self.longitude, self.altitude);
            match times {
                Some((_sunrise, sunset)) => Some(sunset.with_timezone(&Local)),
                None => None,
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
        pub fn set_date_time(&mut self, dateTime: &NaiveDateTime) -> Result<(), ds323x::Error> {
            self.rtc.set_datetime(dateTime)
        }

        /// Method for returning a datetime string
        pub fn get_date_time(&mut self) -> NaiveDateTime {
            self.rtc.datetime().unwrap()
        }

        fn rtc_now_utc(&mut self) -> DateTime<Utc> {
            DateTime::<Utc>::from_naive_utc_and_offset(self.get_date_time(), Utc)
        }

        /// Method for returning a boolean for if it is after sunrsie today
        pub fn after_sunrise(&mut self) -> bool {
            if let Some(sunrise) = self.sunrise_times() {
                self.rtc_now_utc() >= sunrise.with_timezone(&Utc)
            } else {
                false // Return false if sunrise is None
            }
        }

        /// Method for returning a boolean for if it is after sunset today
        pub fn after_sunset(&mut self) -> bool {
            if let Some(sunset) = self.sunset_times() {
                self.rtc_now_utc() >= sunset.with_timezone(&Utc)
            } else {
                false // Return false if sunset is None
            }
        }

        ///Returns a unix timestamp based on the current date time provided
        pub fn datetime_to_unix_timestamp(&mut self) -> i64 {
            self.rtc_now_utc().timestamp()
        }
    }
}

pub use clock::Clock;
