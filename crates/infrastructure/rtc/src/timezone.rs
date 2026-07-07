use std::ffi::{CString, NulError};

use log::info;

// US Timezones
pub const TZ_EASTERN: &str = "EST5EDT,M3.2.0,M11.1.0";
pub const TZ_CENTRAL: &str = "CST6CDT,M3.2.0,M11.1.0";
pub const TZ_MOUNTAIN: &str = "MST7MDT,M3.2.0,M11.1.0";
pub const TZ_PACIFIC: &str = "PST8PDT,M3.2.0,M11.1.0";
pub const TZ_ARIZONA: &str = "MST7"; // No DST

// EU Timezones
pub const TZ_UK: &str = "GMT0BST,M3.5.0/1,M10.5.0";
pub const TZ_CENTRAL_EU: &str = "CET-1CEST,M3.5.0,M10.5.0/3";
pub const TZ_EASTERN_EU: &str = "EET-2EEST,M3.5.0/3,M10.5.0/4";

pub fn set_timezone(tz: &str) -> Result<(), NulError> {
    let tz_cstr = CString::new(tz)?;
    unsafe {
        libc::setenv(c"TZ".as_ptr().cast(), tz_cstr.as_ptr(), 1);
        esp_idf_svc::sys::tzset();
    }
    info!("Timezone set to: {tz}");

    Ok(())
}

pub fn local_time() -> chrono::DateTime<chrono::Local> {
    let now = std::time::SystemTime::now();
    let dt_utc: chrono::DateTime<chrono::Utc> = now.into();
    dt_utc.with_timezone(&chrono::Local)
}

pub fn utc_now() -> chrono::NaiveDateTime {
    unsafe {
        let mut t: libc::time_t = 0;
        libc::time(&mut t);
        chrono::DateTime::from_timestamp(t as i64, 0)
            .expect("Invalid timestamp from libc::time")
            .naive_utc()
    }
}
