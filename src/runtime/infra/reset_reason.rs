//! Why the last reboot happened, as reported by the ESP32 reset cause register.
//!
//! The value survives the reboot in hardware, so reading it during boot tells
//! us whether the tower came back from a power cut, a brownout, a firmware
//! panic, or a deliberate OTA restart. It is published on
//! `tower/{id}/logs/boot` so a tower that restarts in the field can say why.

use esp_idf_svc::sys;

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ResetReason {
    /// Cause could not be determined.
    Unknown,
    /// Cold boot: supply was removed and restored.
    PowerOn,
    /// Reset pin was asserted.
    External,
    /// Deliberate `esp_restart()` — our OTA path reboots this way.
    Software,
    /// Firmware panicked: unwrap on `None`, failed allocation, bad memory access.
    Panic,
    /// Interrupt watchdog: interrupts were disabled for too long.
    InterruptWdt,
    /// Task watchdog: a registered task stopped feeding it (a hang).
    TaskWdt,
    /// Another watchdog (RTC / bootloader) fired.
    OtherWdt,
    /// Woke from deep sleep.
    DeepSleep,
    /// Supply voltage sagged below the brownout threshold.
    Brownout,
    /// SDIO reset.
    Sdio,
}

impl ResetReason {
    /// Read the reset cause for the boot we are currently in.
    pub fn current() -> Self {
        // SAFETY: `esp_reset_reason` reads a register and has no preconditions.
        let raw = unsafe { sys::esp_reset_reason() };

        #[allow(non_upper_case_globals)]
        match raw {
            sys::esp_reset_reason_t_ESP_RST_POWERON => Self::PowerOn,
            sys::esp_reset_reason_t_ESP_RST_EXT => Self::External,
            sys::esp_reset_reason_t_ESP_RST_SW => Self::Software,
            sys::esp_reset_reason_t_ESP_RST_PANIC => Self::Panic,
            sys::esp_reset_reason_t_ESP_RST_INT_WDT => Self::InterruptWdt,
            sys::esp_reset_reason_t_ESP_RST_TASK_WDT => Self::TaskWdt,
            sys::esp_reset_reason_t_ESP_RST_WDT => Self::OtherWdt,
            sys::esp_reset_reason_t_ESP_RST_DEEPSLEEP => Self::DeepSleep,
            sys::esp_reset_reason_t_ESP_RST_BROWNOUT => Self::Brownout,
            sys::esp_reset_reason_t_ESP_RST_SDIO => Self::Sdio,
            _ => Self::Unknown,
        }
    }

    /// Stable identifier for dashboards and log filters.
    pub const fn as_str(&self) -> &'static str {
        match self {
            Self::Unknown => "unknown",
            Self::PowerOn => "power_on",
            Self::External => "external",
            Self::Software => "software",
            Self::Panic => "panic",
            Self::InterruptWdt => "interrupt_watchdog",
            Self::TaskWdt => "task_watchdog",
            Self::OtherWdt => "watchdog",
            Self::DeepSleep => "deep_sleep",
            Self::Brownout => "brownout",
            Self::Sdio => "sdio",
        }
    }

    /// True when the tower did not restart on purpose. `Software` is excluded
    /// because that is how our own OTA path reboots.
    pub const fn is_unexpected(&self) -> bool {
        matches!(
            self,
            Self::Panic
                | Self::InterruptWdt
                | Self::TaskWdt
                | Self::OtherWdt
                | Self::Brownout
                | Self::Unknown
        )
    }

    /// Headline for the boot log.
    pub const fn boot_message(&self) -> &'static str {
        match self {
            Self::Software => "Tower rebooted after a firmware-initiated restart",
            Self::PowerOn => "Tower rebooted after losing power",
            Self::Brownout => "Tower rebooted after a supply voltage brownout",
            Self::Panic => "Tower rebooted after a firmware panic",
            Self::TaskWdt | Self::InterruptWdt | Self::OtherWdt => {
                "Tower rebooted after a watchdog timeout"
            }
            Self::External => "Tower rebooted from an external reset",
            Self::DeepSleep => "Tower woke from deep sleep",
            Self::Sdio => "Tower rebooted from an SDIO reset",
            Self::Unknown => "Tower rebooted for an unknown reason",
        }
    }

    /// Operator-facing detail: what this reset cause implies about the tower.
    pub const fn boot_notes(&self) -> &'static str {
        match self {
            Self::Software => "Deliberate restart, e.g. completing an OTA update.",
            Self::PowerOn => "Supply was removed and restored; site power is the likely cause.",
            Self::Brownout => {
                "Supply dipped below the brownout threshold; check the tower's power rail under motor load."
            }
            Self::Panic => "Firmware hit an unrecoverable error and rebooted.",
            Self::TaskWdt | Self::InterruptWdt | Self::OtherWdt => {
                "Firmware stopped responding and the watchdog reset it."
            }
            Self::External => "Reset pin asserted, e.g. a manual reset or a debug probe.",
            Self::DeepSleep => "Scheduled wake from deep sleep.",
            Self::Sdio => "SDIO peripheral triggered the reset.",
            Self::Unknown => "Reset cause could not be determined.",
        }
    }
}
