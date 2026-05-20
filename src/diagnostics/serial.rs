use anyhow::anyhow;
use board_diagnostics::DiagnosticCommand;
use esp_idf_svc::{hal::i2c::I2cDriver, nvs::{EspNvs, NvsPartitionId}};
use motion::{Motion, MotionMode};
use serial_console::{SerialLinePoll, SerialLineRuntime, UsbSerialJtagConsole};

use crate::diagnostics::{executor, mqtt};

pub struct SerialDiagnosticsRuntime {
    console: UsbSerialJtagConsole,
    runtime: SerialLineRuntime,
}

impl SerialDiagnosticsRuntime {
    pub fn new() -> anyhow::Result<Self> {
        UsbSerialJtagConsole::install_driver(1024, 1024)
            .map_err(|err| anyhow!("usb_serial_jtag_driver_install failed: {}", err))?;
        Ok(Self {
            console: UsbSerialJtagConsole::new(),
            runtime: SerialLineRuntime::new(),
        })
    }

    pub fn poll<T>(
        &mut self,
        shared_command_state: &mqtt::SharedCommandState,
        firmware_version: &str,
        motion: &mut Motion<'_>,
        nvs: &mut EspNvs<T>,
        bus: &'static shared_bus::BusManagerStd<I2cDriver<'static>>,
        home_heading_deg: f32,
        motion_mode: MotionMode,
        actual_heading: &mut f32,
        persist_nvs: bool,
        need_rehome_stepper_only: &mut bool,
    ) -> anyhow::Result<bool>
    where
        T: NvsPartitionId,
    {
        let poll = self.runtime.poll(&mut self.console, |console, line| {
            let _ = console.write_line("CMD_RECEIVED");
            let command = DiagnosticCommand::parse(line);

            if !executor::supports_first_wave_command(&command) {
                let _ = console.write_line(&format!(
                    "ERROR {} is not enabled in first-wave diagnostics",
                    executor::command_name(&command)
                ));
                return Ok::<(), anyhow::Error>(());
            }

            let is_control = executor::is_control_command(&command);
            if is_control {
                if let Err(message) = mqtt::reserve_local_control_command(
                    shared_command_state,
                    executor::command_name(&command),
                ) {
                    let _ = console.write_line(&message);
                    return Ok(());
                }
            }

            let transcript = executor::execute_first_wave_command(
                command.clone(),
                firmware_version,
                motion,
                nvs,
                bus,
                home_heading_deg,
                motion_mode,
                actual_heading,
                persist_nvs,
                need_rehome_stepper_only,
            );

            for line in &transcript.lines {
                let _ = console.write_line(line);
            }
            if transcript.lines.is_empty() {
                let _ = console.write_line(&transcript.message);
            }

            if is_control {
                let _ = mqtt::complete_control_command(shared_command_state, "");
            }
            Ok(())
        })?;

        if matches!(poll, SerialLinePoll::LineProcessed) {
            Ok(true)
        } else {
            Ok(false)
        }
    }
}
