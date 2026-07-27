# serial_console

`serial_console` is a reusable, line-oriented serial transport crate. It is responsible for reading nonblocking bytes, buffering incomplete input, splitting complete command lines, and writing line-based responses.

It does not know anything about diagnostics or board hardware. A firmware can use it for diagnostics, a configuration shell, manufacturing commands, or any other text protocol.

## Responsibilities

- Define the transport-independent `SerialIo` and `SerialTransport` traits.
- Convert incoming bytes into trimmed UTF-8 command lines with `SerialLineRuntime`.
- Provide `UsbSerialJtagConsole` for the ESP32 USB Serial/JTAG peripheral.
- Keep line-processing logic testable on a computer without ESP32 hardware.

The crate recognizes both LF and CRLF input. Empty lines are ignored, incomplete lines remain buffered for the next poll, invalid UTF-8 is converted lossily, and `write_line` appends CRLF when using `UsbSerialJtagConsole`.

## Adding the Crate

From the repository root firmware:

```toml
[dependencies]
serial_console = { path = "crates/infrastructure/serial_console" }
```

If another crate inside the workspace depends on it, adjust the relative path to match that crate's location. If this crate is moved into its own repository, replace the path with the appropriate Git or published-crate dependency.

## Using USB Serial/JTAG on ESP32

Install the ESP-IDF driver once during startup, create a console and runtime, then poll from a task or loop:

```rust
use serial_console::{SerialLinePoll, SerialLineRuntime, UsbSerialJtagConsole};
use std::thread;
use std::time::Duration;

UsbSerialJtagConsole::install_driver(1024, 1024)?;

let mut console = UsbSerialJtagConsole::new();
let mut runtime = SerialLineRuntime::new();

loop {
    match runtime.poll(&mut console, |io, line| {
        match line {
            "PING" => io.write_line("PONG")?,
            _ => io.write_line("ERROR Unknown command")?,
        }
        Ok::<(), ()>(())
    })? {
        SerialLinePoll::Idle => thread::sleep(Duration::from_millis(20)),
        SerialLinePoll::LineProcessed => {}
    }
}
```

The exact error conversion around `install_driver` and `poll` depends on the firmware's error type. The current firmware provides a complete integration example in:

- [`src/diagnostics/serial.rs`](../../../src/diagnostics/serial.rs)
- [`src/runtime/main.rs`](../../../src/runtime/main.rs)

Do not repeatedly install the USB Serial/JTAG driver. Install it once before constructing the polling loop. Also yield or sleep when polling returns `Idle`; otherwise a firmware task can busy-spin.

## Using Another Serial Peripheral

`UsbSerialJtagConsole` is specifically for ESP32 USB Serial/JTAG. To use a UART, Bluetooth stream, test double, or another transport, implement the two traits:

```rust
use serial_console::{SerialIo, SerialTransport};

struct MyTransport {
    // UART driver, socket, queue, or another byte source
}

impl SerialIo for MyTransport {
    fn write_line(&mut self, message: &str) -> Result<(), ()> {
        // Write message bytes followed by a line ending.
        todo!()
    }
}

impl SerialTransport for MyTransport {
    fn read_nonblocking(&mut self, buffer: &mut [u8]) -> usize {
        // Copy only immediately available bytes and return their count.
        // Return zero when no bytes are available.
        todo!()
    }
}
```

`read_nonblocking` must never wait indefinitely. `SerialLineRuntime` expects to be called repeatedly and retains partial input internally until it receives CR or LF.

## Unit Tests

The test module uses an in-memory `MockTransport`. Incoming bytes are held in a `VecDeque<u8>`, while output lines are captured in a `Vec<String>`.

The current test suite feeds:

```text
PING\r\nPONG\n
```

It verifies that:

- CRLF and LF are both accepted.
- The callback receives exactly `PING` and `PONG`.
- Line endings are removed.
- The poll result is `SerialLinePoll::LineProcessed`.

## Running Tests

The repository-level `.cargo/config.toml` defaults to the ESP32-S3 target. Override it with the computer's host target when running unit tests.

From the repository root on Windows:

```powershell
cargo test -p serial_console --target x86_64-pc-windows-msvc
```

From inside the crate directory:

```powershell
cargo test --target x86_64-pc-windows-msvc
```

After dependencies have already been downloaded, `--offline` can be added to prevent Cargo from accessing crates.io:

```powershell
cargo test --target x86_64-pc-windows-msvc --offline
```

On another operating system, replace `x86_64-pc-windows-msvc` with the host triple printed by:

```text
rustc -vV
```

Look for the `host:` line.

## Adding Tests

Transport tests should remain hardware-independent. Feed byte chunks through a mock `SerialTransport`, call `SerialLineRuntime::poll`, and assert both the callback lines and `SerialLinePoll` result.

Useful next cases include:

- A command split across multiple polls.
- Several commands received in one read.
- Empty CR/LF sequences.
- Invalid UTF-8 input.
- Input that does not yet contain a complete line.
- A maximum line-length policy.

## Current Limitations

- The internal line buffer currently has no maximum length.
- Transport write errors use `Result<(), ()>` and therefore carry no details.
- The host implementation of `UsbSerialJtagConsole` is intentionally nonfunctional; host tests must use a mock or another `SerialTransport`.
- The crate frames lines but does not define command syntax, acknowledgements, timeouts, or response formats.
