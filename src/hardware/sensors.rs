//! The I2C devices that more than one machine needs to read.
//!
//! # Why these are behind a lock and the rest of the bus is not
//!
//! `shared_bus` serialises individual *transactions*. That is everything a
//! one-shot addressed write needs, which is why the diagnostics machine can probe
//! the bus or draw to the display from its own thread without coordinating with
//! anyone.
//!
//! It does nothing for a device *sequence*. `Hdc1080::read` triggers a
//! conversion, waits 20 ms, then reads the result; the DS3231 carries a register
//! pointer between transactions. A second reader landing in either gap takes the
//! measurement meant for the first, and the bus lock cannot see it happen — the
//! individual transactions were all perfectly well formed. Only an owner can
//! prevent that, so these two get one.
//!
//! Holding both under a single lock rather than one each is deliberate. They
//! share a bus, so contention is bounded by the bus regardless, and a single lock
//! cannot be acquired in the wrong order against itself.

use core::time::Duration;
use std::sync::{Arc, Mutex, MutexGuard};

use esp_idf_svc::hal::{delay::Ets, i2c::I2cDriver};
use hdc1080::Hdc1080;
use rtc::Rtc;
use shared_bus::I2cProxy;

/// The HDC1080 as this board wires it: on the shared bus, with a busy-wait delay.
pub type BoardHdc1080 = Hdc1080<I2cProxy<'static, Mutex<I2cDriver<'static>>>, Ets>;

/// Devices with multi-transaction access patterns, and their lock.
pub struct SensorSet {
    /// `None` when no HDC1080 answered at boot. Absent is a normal state for this
    /// board — earlier revisions were not fitted with one — so it is represented
    /// rather than assumed.
    pub hdc1080: Option<BoardHdc1080>,
    pub rtc: Rtc,
}

/// A handle every machine that needs a sensor holds a clone of.
pub type SharedSensors = Arc<Mutex<SensorSet>>;

pub fn shared(hdc1080: Option<BoardHdc1080>, rtc: Rtc) -> SharedSensors {
    Arc::new(Mutex::new(SensorSet { hdc1080, rtc }))
}

/// Take the sensor lock, or give up and say so.
///
/// Bounded rather than blocking, because the promise the diagnostics machine
/// makes is that it always answers. The lock is normally held for one sensor
/// read — tens of milliseconds — but `Rtc::init` holds it across an SNTP sync at
/// boot, and waiting out a network round trip is exactly the stall that would
/// make the console useless at the moment somebody reaches for it.
///
/// Returning `None` is a real answer: "something else is using this right now".
/// That is information. Blocking is not.
pub fn lock(sensors: &SharedSensors, budget: Duration) -> Option<MutexGuard<'_, SensorSet>> {
    let deadline = std::time::Instant::now() + budget;

    loop {
        // Poisoning is recovered from rather than propagated. A panic while
        // holding this lock leaves a sensor mid-sequence, which the next read
        // resynchronises anyway — refusing to read from then on would turn one
        // bad reading into a permanently dead diagnostic.
        match sensors.try_lock() {
            Ok(guard) => return Some(guard),
            Err(std::sync::TryLockError::Poisoned(poisoned)) => return Some(poisoned.into_inner()),
            Err(std::sync::TryLockError::WouldBlock) => {
                if std::time::Instant::now() >= deadline {
                    return None;
                }
                std::thread::sleep(Duration::from_millis(5));
            }
        }
    }
}
