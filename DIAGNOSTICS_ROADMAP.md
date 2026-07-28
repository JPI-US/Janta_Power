# Diagnostics Roadmap

Plan for closing the gap between the serial contract in
`Janta_Installer/docs/serial-protocol.md` and what this firmware actually
implements. Companion to `REMOTE_DIAGNOSTICS_PARKING_LOT.md`, which covers the
MQTT side.

Three workstreams, in dependency order:

1. Command coverage — implement the commands the installer already sends.
2. Serial availability — make the board answer when the installer asks.
3. Structured protocol — replace legacy line matching with `RESULT` lines.

Workstream 2 is worth starting early. Several of its items are cheap and they
make workstream 1 testable.

## Completed

**`HDC1080_READ` false failure.** It emitted `HDC1080 read: temp_c:<n>
humidity:<n>`, which matches no pattern the installer accepts, so a healthy
sensor reported a timeout. The terminal line is now `HDC1080 Correct read:
temp:<n> hum:<n>`. Failures inside the read sequence now write an `ERROR` line to
the console before returning, so a fault occurring after the identity lines
resolves as a failure instead of a timeout.

**Target build was broken.** `crates/diagnostics` imported `esp_idf_sys` without
declaring it, so the crate — and therefore the `tower` binary — could not compile
for `xtensa-esp32s3-espidf`. The import existed only to support a duplicate copy
of the USB Serial/JTAG console that the runtime had already stopped using in
favour of `serial_console`.

Fixed by deleting the duplicate: the `extern "C"` block, `UsbSerialJtagError`,
`UsbSerialJtagConsole` and its `DiagnosticIo`/`DiagnosticTransport` impls. The
crate is now dependency-free and its nine protocol tests run on the host in
seconds:

```powershell
cargo +esp test -p diagnostics --target x86_64-pc-windows-msvc
```

Keep it that way. Host-testable protocol tests are the cheapest guard against the
kind of cross-repo string drift that caused the HDC1080 bug, and they are what
workstream 3 builds on. `DiagnosticRuntime`, `DiagnosticTransport`, and
`DiagnosticPoll` were kept — they are pure, they back those tests, and they are
not duplicates of `serial_console::SerialLineRuntime` (that one only splits
lines; this one also acknowledges and dispatches).

---

## 1. Command coverage

### The gate

`executor::supports_first_wave_command` allows five commands:
`FIRMWARE_VERSION`, `GET_CAPABILITIES`, `RTC_CHECK`, `HDC1080_READ`, `GO_HOME`.
`serial.rs` checks it before dispatch, so everything else is rejected with
`ERROR <CMD> is not enabled in first-wave diagnostics`.

Rejections are at least fast and legible. The installer fails the command
immediately with that text rather than waiting out its timeout.

Three things change as commands land:

- The allowlist becomes per-command and shrinks to nothing.
- `is_control_command` must list every new state-mutating command, or serial and
  MQTT can drive the motor at the same time. `reserve_local_control_command` is
  the existing interlock.
- `FIRST_WAVE_CAPABILITIES` must stay truthful — the installer displays it. It
  is currently duplicated as a `.with_capability()` chain in
  `execute_first_wave_command`; derive the chain from the const so the two
  cannot drift.

### Group A — configuration (`SET_ENV`, `GET_ENV`, `SAVE_CONFIG`, `GET_CONFIG`)

Highest value. This is the installer's actual purpose, and today the entire
Customer Data panel fails. `UnsupportedEnvironment` gets replaced by an
`NvsEnvironment` implementing `DiagnosticEnvironment` over `EspNvs`.

**Boot-time seeding defeats this and must be fixed first.** `main.rs`
unconditionally rewrites `wifi_ssid`, `wifi_pass`, `tz_posix`, `version`,
`tower_latitude`, and `tower_longitude` from the switchboard on every boot
(lines 92–110, 164, 335–347). Anything `SAVE_CONFIG` writes is discarded on the
next power cycle. Change these to seed-if-absent, or gate them behind a
`provisioned` flag in NVS. Without this, the rest of Group A is theatre.

**Key mapping is not optional.** NVS key names cap at 15 characters.
`location.timezone_offset_hours` is 30. The installer sends 15 dotted keys
(`buildEnvironmentEntries` in `customerConfig.ts`). Define one static table —
protocol key, NVS key, value kind — and reject anything not in it. Silently
accepting an unknown key and answering `OK` is worse than failing.

| Protocol key | NVS key | Kind |
| --- | --- | --- |
| `device.tower_id` | `tower_id` | string |
| `wifi.ssid` | `wifi_ssid` | string (existing) |
| `wifi.password` | `wifi_pass` | string (existing) |
| `location.latitude` | `tower_latitude` | f64, −90..90 (existing) |
| `location.longitude` | `tower_longitude` | f64, −180..180 (existing) |
| `location.altitude` | `tower_altitude` | f64 |
| `location.timezone_offset_hours` | `tz_offset_h` | i32, −12..14 |
| `mqtt.broker` | `mqtt_broker` | string |
| `mqtt.port` | `mqtt_port` | u16 |
| `mqtt.username` | `mqtt_user` | string |
| `mqtt.password` | `mqtt_pass` | string |
| `mqtt.topic` | `mqtt_topic` | string |
| `customer.full_name` | `cust_name` | string |
| `customer.address` | `cust_addr` | string |
| `customer.phone` | `cust_phone` | string |

Existing keys are reused deliberately so provisioned values feed the code paths
that already read them.

**Stage, then commit.** The wire sequence is many `SET_ENV`, one `SAVE_CONFIG`,
then `GET_ENV` to verify. `SET_ENV` should validate and stage into RAM;
`SAVE_CONFIG` writes the staged set and clears it. An interrupted sequence then
leaves the tower on its previous configuration rather than half-provisioned.
`GET_ENV` returns the staged value if one exists, otherwise the committed one.

Validate types and ranges in firmware. The installer validates too, but the
board must not trust the host. Cap value length (128 bytes is generous) to bound
staging RAM.

`GET_CONFIG` builds `StaticDiagnosticConfiguration` sections from NVS, which
already emits the `[section]` / `key=value` shape the installer parses.

Two decisions needed before implementing (see Open Questions).

### Group B — actuators (`MOTOR_MOVE`, `RELAY_MOTOR`, `RELAY_HOTSPOT`)

`MOTOR_MOVE` moves a large metal tower on installer command, so it needs more
care than the rest of the list combined.

- **Argument format is undefined.** `docs/serial-protocol.md` does not specify
  one. Propose `MOTOR_MOVE <CW|CCW> <degrees>`, degrees bounded to a small
  travel (10° is enough to prove the drivetrain).
- **No usable motion primitive exists.** `Motion::set_tower_position` is the
  tracking entry point: it takes clock, MQTT, Wi-Fi, OTA, and NVS, and decides
  what to do from sunrise/sunset state. It is the wrong tool. Extract a small
  explicit primitive — `Motion::jog(delta_deg) -> MoveOutcome` — that honours
  soft limits and stall detection and does nothing else.
- **Safety gates:** enforce `sw.runtime.guardrails` soft limits, keep stall
  detection on, require the motor relay energised, and refuse if the tower is
  mid-track.
- **Persist the result.** Update `actual_heading` and save the heading (and, in
  `EncoderGuarded`, the encoder snapshot) exactly as `go_home` does, or the next
  boot restores a stale position.

`RELAY_MOTOR ON|OFF` maps to the GPIO17 relay. `Motion::relay_on` / `relay_off`
are `pub(crate)`; expose a checked `set_relay(bool)` rather than making the raw
pair public, and re-energise on command completion so a test cannot leave the
drive dead.

`RELAY_HOTSPOT` is **blocked**: there is no hotspot relay pin in the firmware.
GPIO17 is the only relay wired. Needs a pin assignment before it can be planned.

### Group C — `OLED_TEST`

**Blocked.** There is no display driver in the tree. The I²C bus carries the
HDC1080 and the DS3231 only. Work needed: identify the controller on SDA/SCL
(SSD1306, or an HD44780 behind a PCF8574 expander), add a driver under
`crates/drivers/`, acquire a handle from the existing `shared_bus`, then have
`oled_test` draw a pattern and report. Until the part is identified this cannot
be scoped.

### Group D — `REBOOT` and `CONFIG_MODE`

`REBOOT` is nearly free and should go first of everything in this section. The
crate handler already writes `OK Rebooting` and returns
`DiagnosticControl::RebootRequested`; `execute_first_wave_command` flattens that
into an ordinary completed transcript and drops it. Thread the signal out of
`SerialDiagnosticsRuntime::poll`, flush the console, delay briefly so the line
clears the USB FIFO, then call `esp_idf_svc::hal::reset::restart()` (the OTA
crate already does this).

Rebooting drops the USB CDC device and the host COM port disappears. The
installer has `reconnectWithRetries` but does not call it after `REBOOT` — a
cross-repo item.

`CONFIG_MODE` has no defined meaning. `DiagnosticBoard::config_mode` defaults to
a no-op that writes nothing, which reads as a timeout. Proposal: a RAM-only mode
with a TTL that suspends tracking and holds position so an installer can work
safely, auto-expiring the way the parking lot's `admin_unlock` sketch does. The
tracking loop checks the flag; the handler answers `OK`.

### Suggested order

`REBOOT` → Group A → `CONFIG_MODE` → `MOTOR_MOVE` / `RELAY_MOTOR` →
`RELAY_HOTSPOT` (blocked) → `OLED_TEST` (blocked).

---

## 2. Serial availability

### What is wrong

`serial_diagnostics.poll` is called from exactly one place:
`sleep_with_diagnostics_control_until`, the idle window at the end of each
tracking pass. Consequences:

- **Nothing answers during boot.** `SerialDiagnosticsRuntime::new()` is not even
  constructed until after Wi-Fi (main.rs:199). Ahead of it: a 20 s sleep, Wi-Fi
  association, SNTP/RTC, MQTT connect, boot validation with its own 5 s sleep,
  and homing, which can run for minutes. The installer auto-queries
  `FIRMWARE_VERSION` one second after connecting, which lands squarely in this
  dead zone.
- **Nothing answers during the tracking pass.** A sun-tracking move blocks the
  loop before the idle window is reached.
- **`GO_HOME` blocks for as long as homing takes** and the installer allows 8 s.

A background serial thread is the obvious fix and the wrong one: single main-loop
ownership of motion, NVS, and the I²C bus is the design rule that keeps this
firmware sane. The MQTT path already solved the same problem with a cached
snapshot plus a bounded control queue drained on the main loop.

### 2a. Construct the console first

Move `SerialDiagnosticsRuntime::new()` up to just after logger init. It only
installs the USB Serial/JTAG driver and depends on nothing else. Cheap, no risk.

### 2b. Pump serial during boot waits

Replace the bare `thread::sleep` calls in the boot path with a slice loop that
polls serial. Motion, NVS, and the bus do not exist yet at most of those points,
so add a restricted entry point — `poll_readonly(phase, firmware_version)` — that
answers `FIRMWARE_VERSION`, `GET_CAPABILITIES`, and a new `PING`, and rejects
everything else with `ERROR BUSY <phase>`.

A fast honest rejection is much better than silence: the installer shows the
reason instead of an eight-second stall.

### 2c. Stream output instead of buffering it

`TranscriptIo` collects lines into a `Vec` and `serial.rs` writes them only after
the command finishes. So `go_home`'s `MOVING_HOME` line — which exists precisely
to signal progress — does not reach the host until homing has already completed.
Progress reporting is currently impossible.

Change `execute_first_wave_command` to accept `&mut impl DiagnosticIo` rather
than constructing its own. The serial path passes a console writer that goes
straight out; the MQTT path keeps passing `TranscriptIo`. If serial also needs
the transcript for logging, pass a `TeeIo` that writes through and records.

This is a prerequisite for any long-running command being observable, and it is a
contained refactor.

### 2d. Fix the long-command timeout

Two changes, both worth making:

- **Installer:** treat the 8 s budget as an inactivity timeout rather than a
  total one — reset it on every line received for the active command. Combined
  with 2c, a homing run that reports progress never times out spuriously. This
  is a small change in `serialCommandDispatcher.ts`.
- **Installer:** per-command caps for the genuinely slow ones (`GO_HOME`,
  `MOTOR_MOVE`), passed through `runTest`.

### 2e. Service serial during long moves

The surgical version: give the motion stepping loop a callback hook the main loop
can use to pump serial mid-move. The larger version: route serial control
commands through the existing `diagnostics_control_tx` queue the way MQTT does,
answering `QUEUED` immediately and the real `RESULT` later. The queue version
changes the wire protocol into an asynchronous one — defer it until 2a–2d are in
and there is evidence the callback is not enough.

### 2f. Announce boot phase

Emit `PHASE <name>` as boot progresses. The installer's terminal then shows why
the board is not answering yet. Nearly free once 2a is done.

---

## 3. Structured protocol

`docs/serial-protocol.md` presents `PROTOCOL_VERSION 1` and
`RESULT <CMD> PASS|FAIL <details>` as the preferred format and legacy lines as a
compatibility shim. In reality the firmware emits only legacy lines and the
installer's `parseProtocolVersion` is called nowhere outside its own test.
`SUPPORTED_PROTOCOL_VERSION` is exported and unused.

The HDC1080 bug is the argument for finishing this: a diagnostic reported failure
for months because two repos disagreed about a string, and nothing on either side
could detect the disagreement.

### 3a. Emit `RESULT` from the firmware

The seam is `serial.rs`, which already knows the command name and the transcript
status after dispatch. Append `RESULT <NAME> PASS <message>` or
`RESULT <NAME> FAIL <message>` after the existing detail lines.

Build the line in the `diagnostics` crate, not in `serial.rs`, so the MQTT
transcript payload and the serial line share one formatter and cannot drift.
Extend `DiagnosticsTranscript` with typed detail pairs while doing it.

Roll out in two stages:

- **Stage 1, parallel.** Keep legacy lines and append `RESULT`. Old and new
  installers both work. The installer resolves on the first matching line, so
  legacy still wins and the new line is decorative — that is fine, it is being
  proven in the field.
- **Stage 2, structured only.** Drop the legacy terminal lines. Informational
  lines plus a final `RESULT`. Requires the handshake below so the installer
  knows which dialect it is talking to.

### 3b. Version handshake

Have `FIRMWARE_VERSION` also emit `PROTOCOL_VERSION 1`, or add a dedicated
command. On connect the installer records the value, and:

- version present and ≥ 1 → strict mode, legacy patterns ignored;
- absent or timed out → legacy mode, as today;
- newer than `SUPPORTED_PROTOCOL_VERSION` → connect, but warn that the app is
  older than the board.

This is what makes Stage 2 safe to deploy without lockstep releases.

### 3c. Structured details

`RESULT HDC1080_READ PASS temp=24.50 hum=55.50` — key/value pairs the installer
can display as data instead of scraping prose. Document the detail keys per
command in `docs/serial-protocol.md`.

### 3d. Terminate multi-line responses

`GET_CONFIG` has no terminator, which is why the installer sends it outside the
dispatcher and scrapes five seconds of terminal output into a buffer
(`finalizeConfig`). A closing `RESULT GET_CONFIG PASS sections=<n>` lets
`GET_CONFIG` move onto the dispatcher and deletes the timer entirely. Good
cleanup, and it removes a real source of flaky config reads.

### 3e. Log and protocol traffic share one link

ESP-IDF logging and the diagnostics protocol both go out the USB Serial/JTAG
peripheral. Unmatched lines are ignored, so this mostly works, and `EspLogger`
formats records as `E (12345) tag: msg` — which does not trip the installer's
`^(ERROR|FAIL|BAD)` failure rule. But any logged message whose own text begins a
line with `ERROR` would fail whatever command happens to be in flight.

Options, cheapest first: prefix protocol lines with a sentinel the installer can
filter on; lower the log level while a command is in flight; or move logging to a
spare UART and reserve USB Serial/JTAG for the protocol.

### 3f. Test the contract on both sides

The crate's existing `MockTransport` tests assert exact write sequences — extend
them to cover `RESULT` lines. The installer side has the matching test in
`diagnosticProtocol.test.ts`. The two suites asserting the same literal strings
is what would have caught the HDC1080 drift, and it is the cheapest guard
available.

---

## Open questions

1. **Does `GET_CONFIG` return credentials?** Echoing `wifi.password` lets an
   installer verify provisioning, but it also means anyone with a USB cable can
   read the site's Wi-Fi password off the board. Omitting the key entirely is
   safer, but the installer auto-fills its form from `GET_CONFIG` — a
   `<set>`/`<unset>` placeholder would be written back as a literal password on
   the next submit. Needs a decision on both sides together.
2. **`MOTOR_MOVE` argument format and travel limit.** Proposal above is
   `MOTOR_MOVE <CW|CCW> <degrees>` bounded to ~10°.
3. **`RELAY_HOTSPOT` pin assignment.** Blocked until the hardware is defined.
4. **Which controller is on the LCD?** Blocks `OLED_TEST` entirely.
5. **What should `CONFIG_MODE` do?** Proposal above is a TTL-bounded hold that
   suspends tracking.
