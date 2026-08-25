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

**`REBOOT` was unreachable twice over.** It was missing from
`supports_first_wave_command`, so `serial.rs` rejected it before dispatch; and
`execute_first_wave_command` flattened `DiagnosticControl::RebootRequested` into
an ordinary completed transcript and dropped the signal. The installer's Reboot
button could never succeed.

`DiagnosticsTranscript` now carries `reboot_requested`, `SerialDiagnosticsRuntime::poll`
returns `SerialDiagnosticsPoll` (`Idle` / `CommandProcessed` / `RebootRequested`)
instead of a bare bool, and the main loop performs the reset — 250 ms after the
reply, so `OK Rebooting` clears the USB FIFO before the CDC device drops with it.
`REBOOT` is also in `is_control_command`, so it is refused with `ERROR BUSY` rather
than resetting the board out from under a queued MQTT `go_home`.

The installer half landed with it: `rebootDevice()` now releases the port and calls
`reconnectWithRetries()`, and `diagnosticProtocol.test.ts` pins `OK Rebooting`
against the crate test that asserts the same two lines.

**`FIRST_WAVE_CAPABILITIES` was duplicated.** The `.with_capability()` chain in
`execute_first_wave_command` is now folded from the const, so adding a capability
is a one-line change that cannot half-apply.

**The serial console was constructed last.** `SerialDiagnosticsRuntime::new()` now
runs immediately after logger init instead of at main.rs:199, so the USB
Serial/JTAG peripheral exists from the start of boot rather than after Wi-Fi,
SNTP, MQTT, boot validation, and homing. (2a)

**The boot path was dead air.** The installer queries `FIRMWARE_VERSION` one
second after opening the port, which landed in a silence lasting minutes — so the
first thing the app did always timed out.

`SerialDiagnosticsRuntime::poll_readonly(phase, firmware_version)` is the
restricted entry point: it borrows no motion, NVS, or bus, which is what makes it
callable from points in boot where none of them exist. It answers `PING` (new),
`FIRMWARE_VERSION`, and `GET_CAPABILITIES` for real, and refuses everything else
with `ERROR BUSY <phase>`.

`sleep_answering_boot_diagnostics` replaces the bare `thread::sleep` calls in the
boot path with 50 ms slices that pump it — the 20 s pre-Wi-Fi wait, the OTA check
delay, the post-homing settle, and all three waits inside `boot_diagnostic`. Total
durations are unchanged. (2b)

The response strings live in `boot_phase_response` in the `diagnostics` crate, not
in `serial.rs`, so they are covered by the host tests. `firmware_version_line` was
pulled into the crate at the same time — `VERSION <n>` had three emitters and was
about to have a fourth. (2f, `PHASE <name>` announcements, is still open; the
phase strings this introduced are what it would broadcast.)

**Output was buffered until the command finished.** `execute_first_wave_command`
built its own `TranscriptIo`, so `go_home`'s `MOVING_HOME` — a progress line —
only reached the host once homing had already completed.

It now takes the caller's `&mut impl DiagnosticIo`. Serial passes `ConsoleIo`,
which writes straight to the console and tracks whether anything was written (the
empty-output fallback needs that); MQTT passes the now-public `TranscriptIo` and
copies the lines into the transcript it publishes. `DiagnosticsTranscript.lines`
is therefore filled by the caller, not by the executor. (2c)

**The installer's timeout was a total, not an inactivity budget.** Eight seconds
from send, whatever the board was doing.

`serialCommandDispatcher` now runs two timers: an inactivity budget restarted by
any traffic, and an absolute ceiling. The ceiling is not optional — ESP-IDF
logging shares the link, so an inactivity-only budget would let a chattering but
unanswering board hold the single command slot forever. `COMMAND_TIMEOUTS` in
`diagnosticProtocol.ts` carries per-command overrides, applied through `runTest`.
(2d)

**Long moves blocked the link with nothing to show for it.** A homing search runs
for tens of minutes and said nothing the whole time; a tracking pass blocked
before the idle window was ever reached.

`motion` now offers `MoveTick` and `MoveWatcher`, and a `_watched` variant of each
entry point that can block: `run`, `move_by`, `find_limit_switch_ccw` / `_cw`,
`set_tower_position`. The existing names remain as no-op-callback wrappers, so no
call site changed behaviour. The tick fires from the 100 ms block in `run` that
already logs position to the same USB peripheral, which is why a comparable
consumer is affordable there.

Two consumers:

- `go_home` in `executor.rs` reports `HOMING steps_remaining=... encoder_ticks=...`
  every five seconds, through the streaming `DiagnosticIo` that 2c introduced.
- `main.rs` passes `pump_serial_during_move`, which calls `poll_readonly` — the
  hardware-free path from 2b — so read-only commands are answered throughout boot
  homing, re-homing, encoder recovery, and tracking passes. Because it can only
  answer commands that need no hardware, running it from inside the stepping loop
  does not break single-owner ownership of motion, NVS, and the bus.

With progress on the wire, `COMMAND_TIMEOUTS.GO_HOME` drops from a 180 s
everything-budget to a 30 s silence budget with a one-hour ceiling: a board that
dies mid-search now fails in half a minute instead of three, and a legitimate
full-travel search is no longer cut off. (2e, and the caveat left open in 2d)

**The board never said what it was doing.** Busy and wedged look identical from
the host: both are silence.

`SerialDiagnosticsRuntime::announce_phase` emits `PHASE <name>` whenever the phase
changes — repeating the current one is a no-op, so the retry loops can call it
freely. Every place that already knew its phase now announces it:
`sleep_answering_boot_diagnostics`, `pump_serial_during_move`, and the idle window,
which announces `ready` because it is the one point where the whole command set is
available.

Phase lines are the first output the firmware sends unprompted, so unlike every
other line they can land while an unrelated command is pending.
`phase_lines_cannot_be_mistaken_for_a_result` checks every phase name the runtime
uses against the host's accept rules, and the installer pins the other side across
every command. The installer parses them with `parsePhaseLine` and shows the
current one in the Board Information panel. (2f)

**Configuration was the whole point and none of it worked.** `UnsupportedEnvironment`
rejected every `SET_ENV`, so the Customer Data panel failed on its first write —
and boot rewrote `wifi_ssid`, `wifi_pass`, `tz_posix`, `tower_latitude` and
`tower_longitude` from the switchboard every time, so anything that did get
written was discarded on the next power cycle.

Both halves are fixed. `SAVE_CONFIG` sets a `provisioned` flag in NVS; boot seeds
defaults only while that flag is clear, so an unprovisioned tower still tracks
`.env` and an OTA, and a provisioned one is left alone. `version` stays
unconditional — it is firmware identity, not site configuration.

`CONFIG_KEYS` in the diagnostics crate is the single table of protocol key → NVS
key → value kind, with tests that every NVS name fits in 15 bytes, that sections
are contiguous, and that the table matches what `buildEnvironmentEntries` sends.
`SET_ENV` validates and stages into RAM; `SAVE_CONFIG` commits the set and clears
it, leaving the buffer intact on a write failure so a commit can be retried.
`GET_ENV` reports the staged value if one is waiting, otherwise the committed one.
`GET_CONFIG` builds its sections from the same table.

Configuration is routed away from the board path: it needs the `&mut EspNvs` the
runtime board also holds, so it runs against an `UnsupportedBoard` stub rather than
trying to alias the handle.

`DiagnosticEnvironment`'s methods now take `&mut impl DiagnosticIo`, the way
`DiagnosticBoard`'s already did. Without it a rejected `SET_ENV` wrote nothing at
all, which reads to the host as a hung board rather than a bad value — and `SET_ENV`
is the command the installer sends fifteen times in a row.

Secrets are never read back. `GET_CONFIG` and `GET_ENV` report `<set>` or
`<unset>`; the board refuses to store either as a password, and the installer omits
the `SET_ENV` for any secret still holding a placeholder. So a technician can see
that a password is configured without a USB cable being able to read it. (Group A)

**The structured protocol existed only on paper.** `docs/serial-protocol.md`
presented `RESULT` as preferred and legacy lines as a shim; the firmware emitted
only legacy lines and the installer's `parseProtocolVersion` was called nowhere
outside its own test.

`board_diagnostics::result_line` is now the single formatter, and `serial.rs`
appends a `RESULT` after every command — including boot-phase refusals, which
would otherwise leave a strict host waiting out its timeout. Both dialects go out
at once, so firmware and installer still ship independently. (3a)

`FIRMWARE_VERSION` emits `PROTOCOL_VERSION 1` **before** its version line. The
order is load-bearing: the installer resolves the command on `VERSION`, so an
announcement after it has no pending request to attach to. The installer records
it and switches to strict matching, warns when a board is newer than it
understands, and stays on legacy patterns for a board that announces nothing. (3b)

Board methods record typed details — `temp=24.50 hum=55.50`, `heading=180.00`,
`sections=5`, `committed=15` — collected off the board through
`StandardDiagnosticHandler::board` rather than by widening the trait. (3c)

`GET_CONFIG` terminates with `RESULT GET_CONFIG PASS sections=<n>`, so it is now
an ordinary managed command. The installer's five-second terminal scrape and all
its buffering state are gone; a board that sends no terminator has its partial
output read out of the timeout error instead of discarded. (3d)

3e fell out of strict mode rather than needing a sentinel or a second UART: with
only `RESULT` resolving a command, a log line beginning `ERROR` can no longer fail
whatever was in flight. Mutation-tested on both sides. (3e, 3f)

---

## 1. Command coverage

### The gate

`executor::supports_first_wave_command` allows six commands:
`FIRMWARE_VERSION`, `GET_CAPABILITIES`, `RTC_CHECK`, `HDC1080_READ`, `GO_HOME`,
`REBOOT`. `serial.rs` checks it before dispatch, so everything else is rejected
with `ERROR <CMD> is not enabled in first-wave diagnostics`.

Rejections are at least fast and legible. The installer fails the command
immediately with that text rather than waiting out its timeout.

Three things change as commands land:

- The allowlist becomes per-command and shrinks to nothing.
- `is_control_command` must list every new state-mutating command, or serial and
  MQTT can drive the motor at the same time. `reserve_local_control_command` is
  the existing interlock.
- `FIRST_WAVE_CAPABILITIES` must stay truthful — the installer displays it. The
  `.with_capability()` chain in `execute_first_wave_command` is now folded from
  the const, so adding a command means adding one entry in one place.

### Group A — configuration (`SET_ENV`, `GET_ENV`, `SAVE_CONFIG`, `GET_CONFIG`) — done

See Completed. The two open questions were decided: secrets are reported as
`<set>`/`<unset>` and never echoed, and boot seeding is gated behind a
`provisioned` flag rather than seed-if-absent — so an unprovisioned tower still
picks up changed defaults from a reflash or an OTA.

The rest of this section is kept as the record of what was built and why.

**Provisioned is not the same as consumed.** All fifteen keys are validated,
stored, survive a power cycle and read back — but only four currently change what
the tower does, and they are exactly the ones the table below marks *(existing)*:
`wifi_ssid`, `wifi_pass`, `tower_latitude`, `tower_longitude`. The rest are
recorded and inert:

- `tz_offset_h` — the runtime reads `tz_posix`, not an offset. **Do not derive one
  from the other naively.** A bare offset cannot express DST, and v1.1.3 fixed
  sunset homing firing an hour early precisely by making the clock DST-aware.
  Either provision a POSIX TZ string instead, or carry a zone name.
- `mqtt_broker`, `mqtt_port`, `mqtt_user`, `mqtt_pass`, `mqtt_topic` — the firmware
  talks to AWS IoT with certificates embedded at compile time and a hardcoded
  broker URL. These cannot take effect without reworking that.
- `tower_altitude` — `main.rs` hardcodes `altitude = 0.0`.
- `tower_id` — identity comes from `DEVICE_ID` in `.env` via the switchboard.
- `cust_name`, `cust_addr`, `cust_phone` — installation record only, with nothing
  to consume.

Nothing here is wrong: the keys had to exist before anything could read them, and
storing the installation record is worth doing on its own. But a technician who
sets a timezone or an MQTT broker today will not see the tower's behaviour change,
so wiring these up is the natural follow-on.

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

`REBOOT` is **done** — see Completed above. Both halves shipped together: the
firmware answers and resets, and the installer releases the port and reconnects.

`CONFIG_MODE` has no defined meaning. `DiagnosticBoard::config_mode` defaults to
a no-op that writes nothing, which reads as a timeout. Proposal: a RAM-only mode
with a TTL that suspends tracking and holds position so an installer can work
safely, auto-expiring the way the parking lot's `admin_unlock` sketch does. The
tracking loop checks the flag; the handler answers `OK`.

### Suggested order

~~`REBOOT`~~ (done) → Group A → `CONFIG_MODE` → `MOTOR_MOVE` / `RELAY_MOTOR` →
`RELAY_HOTSPOT` (blocked) → `OLED_TEST` (blocked).

---

## 2. Serial availability

### What is wrong

`serial_diagnostics.poll` is called from exactly one place:
`sleep_with_diagnostics_control_until`, the idle window at the end of each
tracking pass. Consequences:

- ~~**Nothing answers during boot.**~~ Fixed by 2a + 2b — see Completed. The
  blocking waits now answer read-only commands and refuse the rest by name.
  Homing was the remaining hole; 2e closed it.
- ~~**Nothing answers during the tracking pass.**~~ Fixed by 2e — the tracking
  move carries a watcher that services read-only commands.
- ~~**`GO_HOME` blocks for as long as homing takes**~~ and the installer allowed
  8 s. Fixed across 2c, 2d and 2e: output streams, progress is reported every
  five seconds, and the installer measures silence rather than elapsed time.

A background serial thread is the obvious fix and the wrong one: single main-loop
ownership of motion, NVS, and the I²C bus is the design rule that keeps this
firmware sane. The MQTT path already solved the same problem with a cached
snapshot plus a bounded control queue drained on the main loop.

### 2a. Construct the console first — done

`SerialDiagnosticsRuntime::new()` now runs just after logger init. The driver is
installed from the start of boot; 2b is what makes the board actually answer
during it.

### 2g. The boot gate is over-restrictive — done

Found on hardware: during the Wi-Fi wait the board refuses `HDC1080_READ`,
`RTC_CHECK` and every configuration command with `ERROR BUSY waiting to connect
Wi-Fi`, but all of them could run. The construction order in `main.rs` says so:

| | line | exists at the Wi-Fi wait (line 160)? |
| --- | --- | --- |
| NVS | 78 | yes |
| I²C bus | 103 | yes |
| `Motion` | 425 | no |

`poll_readonly` was written as a fixed three-command allowlist rather than
deriving availability from what has actually been constructed. Only `GO_HOME`
genuinely cannot run that early.

There is a second inaccuracy: `OLED_TEST`, `RELAY_MOTOR`, `RELAY_HOTSPOT` and
`CONFIG_MODE` are not implemented at all, so blaming Wi-Fi misdescribes why they
were refused — they would be refused after boot too, for a different reason.

Fixed as described. `MotionContext` now bundles the motion handle with the five
values only `go_home` uses, and the board holds it as an `Option` — which also cut
`execute_first_wave_command` from twelve parameters to seven. Boot waits call the
ordinary `poll` with `motion: None`, so there is no separate response path to keep
in step: sensors and configuration commands are answered exactly as they are after
boot, and `GO_HOME` refuses with `motion is not initialised yet` rather than
blaming Wi-Fi. Commands that are simply unimplemented give their own reason again.

`poll_readonly` became `poll_minimal` and is now used for one situation only:
mid-move, where the motion stack holds the motor and the caller holds NVS, so
genuinely nothing can be lent out.

One consequence worth knowing: `RTC_CHECK` during the Wi-Fi wait reports the system
clock before SNTP or the DS3231 have set it, so an early answer of
`TIME: 1970-01-01 ...` means "the clock is not set yet", not "the RTC is broken".

### 2b. Pump serial during boot waits — done

See Completed. One caveat inherited from 3e: boot is also when ESP-IDF logging is
heaviest, and log traffic now shares the link with a command far more often than
before. `EspLogger`'s `E (12345) tag: msg` format does not trip the installer's
`^(ERROR|FAIL|BAD)` rule, but a logged message whose own text starts a line with
`ERROR` would fail whatever command is in flight. That was always true; 2b widens
the window it can happen in.

### 2c. Stream output instead of buffering it — done

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

### 2d. Fix the long-command timeout — done

Both installer changes landed; see Completed.

The number that matters is `COMMAND_TIMEOUTS.GO_HOME`, and it is worth recording
why it is what it is. `find_limit_switch_ccw` searches one degree at a time and
each degree is a complete `move_by` — relay on, accelerate, decelerate, relay off.
How long that takes depends on the build's gearing, which comes from `.env` at
compile time. With `.env.example`'s values (`MICROSTEPS` 25,600 × `GEAR_REDUCTION`
50 × `SLEW_BEARING` 85 = 108.8M steps/rev, so ~302k steps per degree at 43k steps/s
and 20k steps/s²) that is roughly nine seconds per degree against a 350° budget: a
worst-case search runs for tens of minutes, which no total timeout can cover.

Note that `build.rs`'s fallbacks are **not** `.env.example`'s values —
`GEAR_REDUCTION` defaults to 1.0, not 50.0, along with much lower speed, accel and
stall thresholds. A build with no `.env` is roughly fifty times faster and behaves
quite differently on the same hardware. The budgets below are sized for the geared
case, which costs nothing on an ungeared build.

2e made it reportable rather than merely long. With `HOMING` progress every five
seconds, the budget is 30 s of silence — six missed reports — against a one-hour
ceiling. A board that dies mid-search fails in half a minute; a legitimate
full-travel search is never cut off.

**If the firmware ever stops reporting progress, that 30 s must go back up to
cover the whole silent run.** The two numbers are one contract.

### 2e. Service serial during long moves — done (surgical version)

See Completed. The queue version — routing serial control commands through
`diagnostics_control_tx`, answering `QUEUED` immediately and the real `RESULT`
later — remains deliberately undone. It turns the wire protocol asynchronous, and
the callback appears to be enough.

One thing the callback deliberately does **not** do: answer commands that touch
hardware. Mid-move, `poll_readonly` refuses those with `ERROR BUSY`. That is the
point rather than a limitation — the alternative is re-entering the motion stack
while it holds the motor.

Worth knowing: during a diagnostics `GO_HOME` the pump is not what helps, because
the installer already has its one command in flight and sends nothing else. What
saves that case is the progress reporting. The pump matters for moves the
*firmware* started — boot homing, sunset homing, tracking — when a technician
plugs in and asks something.

### 2f. Announce boot phase — done

See Completed. Went slightly beyond boot: long moves and the idle window announce
themselves too, since the same question — "why is it not answering?" — applies
whenever the board is busy, not only while it is starting up.

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

### 3a. Emit `RESULT` from the firmware — done

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

### 3b. Version handshake — done

Have `FIRMWARE_VERSION` also emit `PROTOCOL_VERSION 1`, or add a dedicated
command. On connect the installer records the value, and:

- version present and ≥ 1 → strict mode, legacy patterns ignored;
- absent or timed out → legacy mode, as today;
- newer than `SUPPORTED_PROTOCOL_VERSION` → connect, but warn that the app is
  older than the board.

This is what makes Stage 2 safe to deploy without lockstep releases.

### 3c. Structured details — done

`RESULT HDC1080_READ PASS temp=24.50 hum=55.50` — key/value pairs the installer
can display as data instead of scraping prose. Document the detail keys per
command in `docs/serial-protocol.md`.

### 3d. Terminate multi-line responses — done

`GET_CONFIG` has no terminator, which is why the installer sends it outside the
dispatcher and scrapes five seconds of terminal output into a buffer
(`finalizeConfig`). A closing `RESULT GET_CONFIG PASS sections=<n>` lets
`GET_CONFIG` move onto the dispatcher and deletes the timer entirely. Good
cleanup, and it removes a real source of flaky config reads.

### 3e. Log and protocol traffic share one link — resolved by strict mode

ESP-IDF logging and the diagnostics protocol both go out the USB Serial/JTAG
peripheral. Unmatched lines are ignored, so this mostly works, and `EspLogger`
formats records as `E (12345) tag: msg` — which does not trip the installer's
`^(ERROR|FAIL|BAD)` failure rule. But any logged message whose own text begins a
line with `ERROR` would fail whatever command happens to be in flight.

Options, cheapest first: prefix protocol lines with a sentinel the installer can
filter on; lower the log level while a command is in flight; or move logging to a
spare UART and reserve USB Serial/JTAG for the protocol.

### 3f. Test the contract on both sides — done

The crate's existing `MockTransport` tests assert exact write sequences — extend
them to cover `RESULT` lines. The installer side has the matching test in
`diagnosticProtocol.test.ts`. The two suites asserting the same literal strings
is what would have caught the HDC1080 drift, and it is the cheapest guard
available.

---

## Decided: a tower with no clock still panics

`Rtc::init` panics when the DS3231 is unreadable *and* Wi-Fi is down. It is
tempting to make that non-fatal, because a board in that state reboots roughly
every minute and can only be diagnosed during the boot wait — which is precisely
when you most want it alive.

**Deliberately left as is.** A tracker that cannot tell the time would drive the
tower to the wrong place, and the reboot loop is a loud, honest symptom of broken
hardware rather than something to paper over. Fix the hardware.

The practical consequence: on such a board, `sw.wifi_connect_delay_secs` (20 s by
default, now actually read rather than hardcoded) is the whole diagnostics window
per boot cycle. Widen it there if a longer one is needed for bring-up.

If this is ever revisited, the shape to aim for is separating "do not move" from
"do not live" — stay up, serve diagnostics, and refuse to home or track until the
time is known. Do not simply delete the panic.

## Open questions

1. ~~**Does `GET_CONFIG` return credentials?**~~ **Decided: placeholders.**
   `GET_CONFIG` and `GET_ENV` report `<set>` or `<unset>`, never the value. The
   danger the question raised — the placeholder being written back as a literal
   password — is closed from both ends: `buildEnvironmentEntries` omits any secret
   still holding a placeholder, and `validate_config_value` refuses to store one
   even if a buggy host sent it.
2. **`MOTOR_MOVE` argument format and travel limit.** Proposal above is
   `MOTOR_MOVE <CW|CCW> <degrees>` bounded to ~10°.
3. **`RELAY_HOTSPOT` pin assignment.** Blocked until the hardware is defined.
4. **Which controller is on the LCD?** Blocks `OLED_TEST` entirely.
5. **What should `CONFIG_MODE` do?** Proposal above is a TTL-bounded hold that
   suspends tracking.
