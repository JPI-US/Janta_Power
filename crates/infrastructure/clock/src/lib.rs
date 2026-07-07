pub mod clock {
    use core::option::Option::None;

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
        pub fn get_hour(&mut self) -> Result<u8, ds323x::Error> {
            let hour = self.rtc.hours()?;
            match hour {
                ds323x::Hours::AM(h) | ds323x::Hours::H24(h) => Ok(h),
                ds323x::Hours::PM(h) => Ok(h + 11),
            }
        }

        /// Method to get the minutes
        pub fn get_minutes(&mut self) -> Result<u8, ds323x::Error> {
            self.rtc.minutes()
        }

        /// Method to get the seconds
        pub fn get_seconds(&mut self) -> Result<u8, ds323x::Error> {
            self.rtc.seconds()
        }

        /// Method to get the day
        pub fn get_day(&mut self) -> Result<u32, ds323x::Error> {
            Ok(self.rtc.date()?.ordinal())
        }

        /// Method to get the day
        pub fn get_month(&mut self) -> Result<u8, ds323x::Error> {
            self.rtc.month()
        }

        /// Method to get the day
        pub fn get_year(&mut self) -> Result<u16, ds323x::Error> {
            self.rtc.year()
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
        pub fn set_date_time(&mut self, date_time: &NaiveDateTime) -> Result<(), ds323x::Error> {
            self.rtc.set_datetime(date_time)
        }

        /// Method for returning a datetime string
        pub fn get_date_time(&mut self) -> Result<NaiveDateTime, ds323x::Error> {
            self.rtc.datetime()
        }

        fn rtc_now_utc(&mut self) -> Result<DateTime<Utc>, ds323x::Error> {
            Ok(DateTime::<Utc>::from_naive_utc_and_offset(
                self.get_date_time()?,
                Utc,
            ))
        }

        /// Method for returning a boolean for if it is after sunrsie today
        pub fn after_sunrise(&mut self) -> Result<bool, ds323x::Error> {
            if let Some(sunrise) = self.sunrise_times() {
                Ok(self.rtc_now_utc()? >= sunrise.with_timezone(&Utc))
            } else {
                Ok(false) // Return false if sunrise is None
            }
        }

        /// Method for returning a boolean for if it is after sunset today
        pub fn after_sunset(&mut self) -> Result<bool, ds323x::Error> {
            if let Some(sunset) = self.sunset_times() {
                Ok(self.rtc_now_utc()? >= sunset.with_timezone(&Utc))
            } else {
                Ok(false) // Return false if sunset is None
            }
        }

        ///Returns a unix timestamp based on the current date time provided
        pub fn datetime_to_unix_timestamp(&mut self) -> Result<i64, ds323x::Error> {
            Ok(self.rtc_now_utc()?.timestamp())
        }
    }
}

pub use clock::Clock;
