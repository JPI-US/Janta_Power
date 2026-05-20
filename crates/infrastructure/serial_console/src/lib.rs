use core::fmt;
use std::string::String;
use std::vec::Vec;

#[cfg(target_os = "espidf")]
use esp_idf_sys as sys;

#[cfg(target_os = "espidf")]
extern "C" {
    fn usb_serial_jtag_read_bytes(buf: *mut u8, length: u32, ticks_to_wait: u32) -> i32;
    fn usb_serial_jtag_write_bytes(buf: *const u8, length: u32, ticks_to_wait: u32) -> i32;
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct UsbSerialJtagError {
    code: i32,
}

impl UsbSerialJtagError {
    pub fn code(self) -> i32 {
        self.code
    }
}

impl fmt::Display for UsbSerialJtagError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "USB Serial/JTAG error {}", self.code)
    }
}

impl std::error::Error for UsbSerialJtagError {}

pub trait SerialIo {
    fn write_line(&mut self, msg: &str) -> Result<(), ()>;
}

pub trait SerialTransport: SerialIo {
    fn read_nonblocking(&mut self, buf: &mut [u8]) -> usize;
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SerialLinePoll {
    Idle,
    LineProcessed,
}

pub struct SerialLineRuntime {
    buffer: Vec<u8>,
}

impl Default for SerialLineRuntime {
    fn default() -> Self {
        Self::new()
    }
}

impl SerialLineRuntime {
    pub fn new() -> Self {
        Self { buffer: Vec::new() }
    }

    pub fn poll<T, F, E>(
        &mut self,
        transport: &mut T,
        mut on_line: F,
    ) -> Result<SerialLinePoll, E>
    where
        T: SerialTransport,
        F: FnMut(&mut T, &str) -> Result<(), E>,
    {
        let mut read_buf = [0u8; 128];
        let bytes_read = transport.read_nonblocking(&mut read_buf);
        if bytes_read == 0 {
            return Ok(SerialLinePoll::Idle);
        }

        self.buffer.extend_from_slice(&read_buf[..bytes_read]);
        let mut processed_any = false;

        while let Some(pos) = self
            .buffer
            .iter()
            .position(|byte| *byte == b'\n' || *byte == b'\r')
        {
            let line = self.buffer.drain(..=pos).collect::<Vec<u8>>();
            let message = String::from_utf8_lossy(&line).trim().to_string();
            if message.is_empty() {
                continue;
            }

            on_line(transport, &message)?;
            processed_any = true;
        }

        if processed_any {
            Ok(SerialLinePoll::LineProcessed)
        } else {
            Ok(SerialLinePoll::Idle)
        }
    }
}

#[derive(Debug, Clone, Copy, Default)]
pub struct UsbSerialJtagConsole;

impl UsbSerialJtagConsole {
    pub fn new() -> Self {
        Self
    }

    #[cfg(target_os = "espidf")]
    pub fn install_driver(
        rx_buffer_size: u32,
        tx_buffer_size: u32,
    ) -> Result<(), UsbSerialJtagError> {
        unsafe {
            let mut cfg = sys::usb_serial_jtag_driver_config_t {
                rx_buffer_size,
                tx_buffer_size,
            };

            let err = sys::usb_serial_jtag_driver_install(&mut cfg as *mut _);
            if err == sys::ESP_OK as i32 {
                Ok(())
            } else {
                Err(UsbSerialJtagError { code: err })
            }
        }
    }

    #[cfg(not(target_os = "espidf"))]
    pub fn install_driver(
        _rx_buffer_size: u32,
        _tx_buffer_size: u32,
    ) -> Result<(), UsbSerialJtagError> {
        Err(UsbSerialJtagError { code: -1 })
    }

    pub fn write_line(&mut self, msg: &str) -> Result<(), ()> {
        SerialIo::write_line(self, msg)
    }
}

impl SerialIo for UsbSerialJtagConsole {
    #[cfg(target_os = "espidf")]
    fn write_line(&mut self, msg: &str) -> Result<(), ()> {
        let mut out = msg.as_bytes().to_vec();
        out.extend_from_slice(b"\r\n");

        unsafe {
            let written = usb_serial_jtag_write_bytes(out.as_ptr(), out.len() as u32, 1);
            if written == out.len() as i32 {
                Ok(())
            } else {
                Err(())
            }
        }
    }

    #[cfg(not(target_os = "espidf"))]
    fn write_line(&mut self, _msg: &str) -> Result<(), ()> {
        Err(())
    }
}

impl SerialTransport for UsbSerialJtagConsole {
    #[cfg(target_os = "espidf")]
    fn read_nonblocking(&mut self, buf: &mut [u8]) -> usize {
        unsafe {
            let read = usb_serial_jtag_read_bytes(buf.as_mut_ptr(), buf.len() as u32, 0);
            if read > 0 {
                read as usize
            } else {
                0
            }
        }
    }

    #[cfg(not(target_os = "espidf"))]
    fn read_nonblocking(&mut self, _buf: &mut [u8]) -> usize {
        0
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::VecDeque;

    #[derive(Default)]
    struct MockTransport {
        reads: VecDeque<u8>,
        writes: Vec<String>,
    }

    impl MockTransport {
        fn from_input(input: &str) -> Self {
            Self {
                reads: input.as_bytes().iter().copied().collect(),
                writes: Vec::new(),
            }
        }
    }

    impl SerialIo for MockTransport {
        fn write_line(&mut self, msg: &str) -> Result<(), ()> {
            self.writes.push(msg.to_string());
            Ok(())
        }
    }

    impl SerialTransport for MockTransport {
        fn read_nonblocking(&mut self, buf: &mut [u8]) -> usize {
            let mut count = 0;
            while count < buf.len() {
                match self.reads.pop_front() {
                    Some(byte) => {
                        buf[count] = byte;
                        count += 1;
                    }
                    None => break,
                }
            }
            count
        }
    }

    #[test]
    fn line_runtime_processes_complete_lines() {
        let mut runtime = SerialLineRuntime::new();
        let mut transport = MockTransport::from_input("PING\r\nPONG\n");
        let mut seen = Vec::new();

        let poll = runtime
            .poll(&mut transport, |_transport, line| {
                seen.push(line.to_string());
                Ok::<(), ()>(())
            })
            .unwrap();

        assert_eq!(poll, SerialLinePoll::LineProcessed);
        assert_eq!(seen, vec!["PING", "PONG"]);
    }
}
