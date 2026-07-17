use std::{ffi::CString, ptr};

use anyhow::{anyhow, Result};
use esp_idf_hal::sys;
use log::error;

pub struct TaskWatchdog;

impl TaskWatchdog {
    /// Registers the current FreeRTOS task with the ESP-IDF Task Watchdog.
    ///
    /// The registered task must periodically call [`TaskWatchdog::feed`] to
    /// indicate that it is still making progress.
    ///
    /// If the task does not feed the watchdog within the configured timeout,
    /// the ESP-IDF Task Watchdog will trigger a panic or system reset depending
    /// on the `sdkconfig.defaults` configuration.
    ///
    /// # Behavior
    ///
    /// - Registers the currently executing FreeRTOS task with the TWDT.
    /// - The registration applies only to the calling task.
    /// - The task remains monitored until [`TaskWatchdog::unregister`] is called
    ///   or the task exits.
    ///
    /// # Errors
    ///
    /// Returns an error if the underlying ESP-IDF watchdog registration fails.
    pub fn register() -> Result<Self> {
        unsafe { esp_err_to_anyhow(sys::esp_task_wdt_add(ptr::null_mut()), "esp_task_wdt_add")? }
        Ok(Self {})
    }

    /// Resets the Task Watchdog timer for the current FreeRTOS task.
    ///
    /// This should be called periodically to indicate that the monitored task
    /// is still functioning correctly.
    ///
    /// # Errors
    ///
    /// Returns an error if the underlying ESP-IDF watchdog reset operation fails.
    pub fn feed(&self) -> Result<()> {
        unsafe { esp_err_to_anyhow(sys::esp_task_wdt_reset(), "esp_task_wdt_reset") }
    }

    /// Explicitly unregisters the current FreeRTOS task from the Task Watchdog.
    ///
    /// After calling this:
    /// - The current task will no longer be monitored by the TWDT.
    /// - Further calls to [`TaskWatchdog::feed`] are invalid unless the task is
    ///   registered again.
    ///
    /// Note that tasks should normally unregister before exiting if they were
    /// registered manually.
    ///
    /// # Errors
    ///
    /// Returns an error if the underlying ESP-IDF watchdog removal operation fails.
    pub fn unregister(&self) -> Result<()> {
        unsafe {
            esp_err_to_anyhow(
                sys::esp_task_wdt_delete(ptr::null_mut()),
                "esp_task_wdt_delete",
            )
        }
    }
}

/// A registered ESP-IDF Task Watchdog user.
///
/// Each instance represents a logical component that must periodically call
/// [`Watchdog::feed`] to indicate it is still alive.
///
/// We intentionally keep ownership of [`_name`] even though it is unused so that the pointer stays
/// for when the name get printed to the logs (like during resets and errors)
pub struct UserWatchdog {
    user: sys::esp_task_wdt_user_handle_t,
    _name: CString,
    registered: bool,
}

impl UserWatchdog {
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

        Ok(Self {
            user,
            _name: c_name,
            registered: true,
        })
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
    pub fn unregister(&mut self) -> Result<()> {
        if !self.registered {
            return Ok(());
        }

        unsafe {
            esp_err_to_anyhow(
                sys::esp_task_wdt_delete_user(self.user),
                "esp_task_wdt_delete_user",
            )?;
        }

        self.registered = false;

        Ok(())
    }
}

impl Drop for UserWatchdog {
    /// Unregisters the watchdog user when this instance is dropped.
    ///
    /// This prevents stale watchdog registrations from remaining active in the
    /// ESP-IDF Task Watchdog system.
    fn drop(&mut self) {
        match self.unregister() {
            Ok(_) => {}
            Err(err) => error!("{err}"),
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
    if err == sys::ESP_OK {
        Ok(())
    } else {
        Err(anyhow!("{ctx} failed with esp_err_t = {err}"))
    }
}

/// Converts a Rust String to a C String that does not allow interior null bytes.
///
/// This shouldn't fail unless you do anything weird.
///
/// # Arguments
/// * `s` - the String to convert
///
/// # Returns
///
/// - `Ok(CString)` if `s` can be safely converted to a C string
/// - `Err(anyhow::Error)` if `s` cannot be safely converted to a C string
fn cstr_from_str(s: &str) -> Result<CString> {
    if s.contains('\0') {
        return Err(anyhow!("string contains interior null byte"));
    }
    Ok(CString::new(s)?)
}
