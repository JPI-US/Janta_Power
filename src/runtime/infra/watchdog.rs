use std::{ffi::CString, ptr};

use anyhow::{anyhow, Result};
use esp_idf_hal::sys;

/// Global Task Watchdog (TWDT) initializer.
///
/// The ESP-IDF Task Watchdog monitors registered tasks and/or users and
/// triggers a panic or reset if they fail to "feed" within a configured timeout.
///
/// To feed this timeout, create and feed [`Watchdog`]s for tasks.
/// a restart will be triggered if a [`Watchdog`] is not fed in the timeout time.
///
/// This struct provides a thin wrapper around global TWDT initialization.
pub struct TaskWatchdog;

impl TaskWatchdog {
    /// Attempts to initialize the ESP-IDF Task Watchdog subsystem.
    ///
    /// # Arguments
    ///
    /// * `timeout_ms` - Maximum allowed time (in milliseconds) between watchdog
    ///   feeds before the system triggers a panic/reset.
    ///
    /// # Behavior
    ///
    /// - Configures the watchdog to trigger a panic on timeout.
    /// - Safe to call multiple times.
    /// - Does nothing if called before, but will produce an error in the log.
    pub fn init(timeout_ms: u32) -> Result<()> {
        unsafe {
            let cfg = sys::esp_task_wdt_config_t {
                timeout_ms,
                idle_core_mask: 0,
                trigger_panic: true,
            };

            let err = sys::esp_task_wdt_init(&cfg);
            if err != sys::ESP_OK as i32 && err != sys::ESP_ERR_INVALID_STATE as i32 {
                esp_err_to_anyhow(err, "esp_task_wdt_init")?;
            }

            Ok(())
        }
    }
}

/// A registered ESP-IDF Task Watchdog user.
///
/// Each instance represents a logical component that must periodically call
/// [`Watchdog::feed`] to indicate it is still alive.
///
/// If `feed()` is not called within the configured timeout, the ESP-IDF Task
/// Watchdog will trigger a panic or system reset depending on `sdkconfig.defaults` configuration.
pub struct Watchdog {
    user: sys::esp_task_wdt_user_handle_t,
}

impl Watchdog {
    /// Registers a new Task Watchdog user.
    /// Make to initialize and set the global timeout with `TaskWatchdog::init(u32)` before creating a `Watchdog`.
    ///
    /// # Arguments
    ///
    /// * `name` - A C-style string used to identify this watchdog user in logs
    ///   and debugging output.
    ///
    /// # Behavior
    ///
    /// - Registers a new watchdog user with the ESP-IDF TWDT subsystem.
    /// - Immediately performs an initial "feed" to establish baseline activity.
    ///
    /// # Errors
    ///
    /// Returns an error if registration or initial reset fails.
    pub fn new(name: &str) -> Result<Self> {
        let mut user = ptr::null_mut();
        let c_name = cstr_from_str(name)?;

        unsafe {
            esp_err_to_anyhow(
                sys::esp_task_wdt_add_user(c_name.as_ptr(), &mut user),
                "esp_task_wdt_add_user",
            )?;

            esp_err_to_anyhow(
                sys::esp_task_wdt_reset_user(user),
                "esp_task_wdt_reset_user",
            )?;
        }

        Ok(Self { user })
    }

    /// Resets the watchdog timer for this user.
    ///
    /// This method should be called periodically to indicate that the monitored
    /// component is still functioning correctly.
    ///
    /// # Errors
    ///
    /// Returns an error if the underlying ESP-IDF watchdog reset operation fails.
    pub fn feed(&self) -> Result<()> {
        unsafe {
            esp_err_to_anyhow(
                sys::esp_task_wdt_reset_user(self.user),
                "esp_task_wdt_reset_user",
            )
        }
    }

    /// Explicitly unregisters this watchdog user from the Task Watchdog.
    ///
    /// After calling this:
    /// - This watchdog will no longer be monitored by the TWDT
    /// - Further calls to `feed()` are invalid
    ///
    /// Note that Watchdogs are also disabled when they fall out of scope.
    /// Disabling manually before moving on from a task is not required.
    /// See [`drop`].
    pub fn kill(mut self) -> Result<()> {
        unsafe {
            esp_err_to_anyhow(
                sys::esp_task_wdt_delete_user(self.user),
                "esp_task_wdt_delete_user",
            )?;
        }

        // Prevent Drop from double-deleting
        self.user = ptr::null_mut();

        Ok(())
    }
}

impl Drop for Watchdog {
    /// Unregisters the watchdog user when this instance is dropped.
    ///
    /// This prevents stale watchdog registrations from remaining active in the
    /// ESP-IDF Task Watchdog system.
    fn drop(&mut self) {
        unsafe {
            let _ = sys::esp_task_wdt_delete_user(self.user);
        }
    }
}

/// Converts an ESP-IDF `esp_err_t` into a Rust `Result`.
///
/// # Arguments
///
/// * `err` - ESP-IDF error code returned by a C API call.
/// * `ctx` - A short string describing the operation that failed.
///
/// # Returns
///
/// - `Ok(())` if `err == ESP_OK`.
/// - `Err(anyhow::Error)` otherwise, containing the context and error code.
fn esp_err_to_anyhow(err: i32, ctx: &'static str) -> Result<()> {
    if err == sys::ESP_OK as i32 {
        Ok(())
    } else {
        Err(anyhow!("{} failed with esp_err_t = {}", ctx, err))
    }
}

pub fn cstr_from_str(s: &str) -> Result<CString> {
    if s.contains('\0') {
        return Err(anyhow!("string contains interior null byte"));
    }
    Ok(CString::new(s)?)
}
