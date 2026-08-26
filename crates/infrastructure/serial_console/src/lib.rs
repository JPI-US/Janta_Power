use core::fmt;
#[cfg(target_os = "espidf")]
use std::time::Instant;
use std::{string::String, vec::Vec};

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

    pub fn poll<T, F, E>(&mut self, transport: &mut T, mut on_line: F) -> Result<SerialLinePoll, E>
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

    /// Install the driver and hand ESP-IDF's console over to it.
    ///
    /// The second half is not optional, and leaving it out corrupts the wire.
    ///
    /// On this chip ESP-IDF's console is mirrored onto USB Serial/JTAG
    /// (`CONFIG_ESP_CONSOLE_SECONDARY_USB_SERIAL_JTAG`), and by default it writes
    /// there with `usb_serial_jtag_ll_write_txfifo` — straight into the 64-byte
    /// hardware FIFO, one byte at a time, with no lock. The driver installed
    /// above writes the *same* FIFO from its own interrupt handler, draining a
    /// ring buffer. Two unsynchronised writers on one FIFO do exactly what you
    /// would expect: bytes disappear from the middle of lines. `CMD_RECEIVED`
    /// arrives as `CMD_RECIVED`, `RESULT I2C_SCAN` as `RESULT I2_SCAN`, and a
    /// host that resolves commands by matching whole lines simply never sees an
    /// answer.
    ///
    /// It is worse than corruption, too. The console's writer spins waiting for
    /// FIFO space and only gives up after `TX_FLUSH_TIMEOUT_US`, which is 50 ms
    /// **per byte**. Contended, a single hundred-character log line can hold its
    /// calling thread for seconds — so a thread that logs and writes protocol on
    /// the same peripheral stalls itself, and commands go unanswered for as long
    /// as it takes.
    ///
    /// `esp_vfs_usb_serial_jtag_use_driver` repoints the console at
    /// `usb_serial_jtag_write_bytes`, the same mutex-protected ring buffer this
    /// crate writes to. One writer, one lock, bounded waits. It also stops the
    /// console reading the receive FIFO directly, so nothing can race us for
    /// inbound bytes either.
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
            if err != sys::ESP_OK as i32 {
                return Err(UsbSerialJtagError { code: err });
            }

            sys::esp_vfs_usb_serial_jtag_use_driver();
        }

        Ok(())
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

/// Longest one line may spend trying to reach the transmit buffer.
///
/// The buffer only drains while a USB host is actually reading it, so an
/// unattended board fills it and stays full. Waiting indefinitely on that would
/// wedge whichever thread is writing, so a line that cannot be delivered inside
/// this budget is abandoned and reported instead.
#[cfg(target_os = "espidf")]
const WRITE_LINE_BUDGET_MS: u128 = 100;

impl SerialIo for UsbSerialJtagConsole {
    #[cfg(target_os = "espidf")]
    fn write_line(&mut self, msg: &str) -> Result<(), ()> {
        let mut out = msg.as_bytes().to_vec();
        out.extend_from_slice(b"\r\n");

        // Looped, because a short write is not a failure — it means the ring
        // buffer filled part-way through the line. The previous version treated
        // it as one and discarded the remainder, which put *half a protocol line*
        // on the wire. That is worse than sending nothing: the host reads the
        // fragment, fails to match it, and waits out its timeout none the wiser.
        let started = Instant::now();
        let mut sent = 0usize;

        while sent < out.len() {
            // One tick per byte inside the call, so this paces itself rather than
            // spinning hot when the buffer is full.
            let written = unsafe {
                usb_serial_jtag_write_bytes(out[sent..].as_ptr(), (out.len() - sent) as u32, 1)
            };

            if written > 0 {
                sent += written as usize;
                continue;
            }

            if started.elapsed().as_millis() >= WRITE_LINE_BUDGET_MS {
                return Err(());
            }
        }

        Ok(())
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
    use std::collections::VecDeque;

    use super::*;

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
