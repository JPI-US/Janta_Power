//! DS3231-backed real time + SNTP fallback + `settimeofday`.
//! Used by the tower firmware as the time backbone (see rollout stages).

pub mod timezone;

use std::{thread, time::Duration};

use ds323x::{DateTimeAccess, Datelike, Ds323x, NaiveDate, NaiveDateTime, Timelike};
use esp_idf_svc::{
    hal::i2c::I2cDriver,
    sntp::{EspSntp, SyncStatus},
};
use log::*;
use wifi::wifi::{Wifi, WifiState};

const RTC_MIN_YEAR: i32 = 2024;
const RTC_MAX_YEAR: i32 = 2100;

type RtcInner = Ds323x<shared_bus::I2cProxy<'static, std::sync::Mutex<I2cDriver<'static>>>>;

pub struct Rtc {
    device: RtcInner,
}

impl Rtc {
    pub fn new(bus: &'static shared_bus::BusManager<std::sync::Mutex<I2cDriver<'static>>>) -> Self {
        let device = Ds323x::new_ds3231(bus.acquire_i2c());
        Self { device }
    }

    /// Core init flow:
    /// 1. Set timezone first so all subsequent time reads are DST-aware
    /// 2. Unless `force_ntp`: read RTC — if sane, set system time and return immediately
    /// 3. If RTC is skipped, invalid, or insane: check wifi and attempt NTP sync as fallback
    /// 4. If NTP succeeds, write corrected UTC time back to RTC
    /// 5. If both fail, panic — time is unrecoverable
    ///
    /// `force_ntp`: skip trusting RTC on this boot (always run SNTP and write RTC), e.g. bring-up
    /// when you know the battery-backed clock is garbage.
    pub fn init(&mut self, wifi: &Wifi, tz: &str, force_ntp: bool) {
        timezone::set_timezone(tz);

        if !force_ntp {
            if let Some(rtc_time) = self.read() {
                if Self::is_sane(&rtc_time) {
                    Self::set_system_time(rtc_time);
                    info!(
                        "System time restored from RTC — local time: {}",
                        timezone::local_time()
                    );
                    return;
                }
                warn!("RTC time outside sane range — falling back to NTP.");
            } else {
                warn!("RTC read failed — falling back to NTP.");
            }
        } else {
            info!("Skipping RTC read — forced NTP (policy); will sync and write RTC from SNTP.");
        }

        match wifi.state() {
            WifiState::Connected(_) => {
                info!("WiFi connected — attempting NTP sync...");
                self.sync_ntp();
            }
            WifiState::Disconnected => {
                panic!("Unrecoverable: RTC invalid and WiFi disconnected — cannot determine time.");
            }
            WifiState::Connecting => {
                warn!("WiFi still connecting — waiting before NTP attempt...");
                thread::sleep(Duration::from_secs(5));
                match wifi.state() {
                    WifiState::Connected(_) => self.sync_ntp(),
                    _ => panic!(
                        "Unrecoverable: RTC invalid and WiFi unavailable — cannot determine time."
                    ),
                }
            }
        }
    }

    fn sync_ntp(&mut self) {
        thread::sleep(Duration::from_secs(10));

        let ntp = match EspSntp::new_default() {
            Ok(n) => {
                info!("SNTP initialized.");
                n
            }
            Err(e) => {
                panic!("Failed to start SNTP: {:?}", e);
            }
        };

        info!("Synchronizing with NTP Server...");

        const NTP_TIMEOUT_SECS: u64 = 60;
        let ntp_start = std::time::Instant::now();

        loop {
            let status = ntp.get_sync_status();

            if status == SyncStatus::Completed {
                let utc_now = timezone::utc_now();
                self.write(utc_now);
                Self::set_system_time(utc_now);
                info!(
                    "Time Sync Completed — UTC: {} | Local: {}",
                    utc_now,
                    timezone::local_time()
                );
                break;
            }

            if ntp_start.elapsed() >= Duration::from_secs(NTP_TIMEOUT_SECS) {
                error!(
                    "NTP sync failed after {}s — status: {:?}",
                    NTP_TIMEOUT_SECS, status
                );
                panic!(
                    "Unrecoverable: RTC invalid and NTP sync timed out — cannot determine time."
                );
            }

            info!(
                "Waiting for NTP sync... ({:.0}s elapsed, status: {:?})",
                ntp_start.elapsed().as_secs_f32(),
                status
            );
            thread::sleep(Duration::from_secs(1));
        }
    }

    /// Read the current datetime from the RTC chip (always stored as UTC)
    pub fn read(&mut self) -> Option<NaiveDateTime> {
        match self.device.datetime() {
            Ok(dt) => {
                info!(
                    "RTC time (UTC): {}-{:02}-{:02} {:02}:{:02}:{:02}",
                    dt.year(),
                    dt.month(),
                    dt.day(),
                    dt.hour(),
                    dt.minute(),
                    dt.second()
                );
                Some(dt)
            }
            Err(e) => {
                error!("RTC read error: {:?}", e);
                None
            }
        }
    }

    /// Write a corrected datetime back to the RTC chip (always store UTC)
    pub fn write(&mut self, datetime: NaiveDateTime) {
        match self.device.set_datetime(&datetime) {
            Ok(_) => info!("RTC updated with corrected time (UTC)."),
            Err(e) => error!("Failed to write corrected time to RTC: {:?}", e),
        }
    }

    pub fn is_sane(dt: &NaiveDateTime) -> bool {
        dt.year() >= RTC_MIN_YEAR && dt.year() <= RTC_MAX_YEAR
    }

    /// Set the ESP system clock from a NaiveDateTime (assumed UTC)
    pub fn set_system_time(dt: NaiveDateTime) {
        let epoch = dt.and_utc().timestamp();
        unsafe {
            let tv = esp_idf_svc::sys::timeval {
                tv_sec: epoch,
                tv_usec: 0,
            };
            esp_idf_svc::sys::settimeofday(&tv, std::ptr::null());
        }
        info!("System time set to UTC: {}", dt);
    }

    pub fn temperature(&mut self) -> Option<f32> {
        self.device.temperature().ok()
    }

    pub fn reset(&mut self, year: i32, month: u32, day: u32, hour: u32, min: u32, sec: u32) {
        let datetime = NaiveDate::from_ymd_opt(year, month, day)
            .expect("Invalid date")
            .and_hms_opt(hour, min, sec)
            .expect("Invalid time");

        match self.device.set_datetime(&datetime) {
            Ok(_) => info!("RTC hard reset to UTC: {}", datetime),
            Err(e) => error!("RTC reset failed: {:?}", e),
        }
    }
}
