# Porting Diagnostics onto the FSM Firmware

A plan for bringing the serial + MQTT diagnostics work from `diagnostics-integration`
onto `FSM_Diagnostics_Integration`.

Everything below is written from a read of both working trees as they stand today, not
from the branch history — the `diagnostics-integration` worktree has substantial
uncommitted work, including a whole untracked crate.

## Status

| Phase | State |
|---|---|
| 0 — prerequisites | **done**, nothing flashed yet |
| 1 — land the portable crates | **done** |
| 2 — Diagnostics FSM skeleton | **done**, nothing flashed yet |
| 3 — local hardware commands | **done** (`OLED_TEST`, `HDC1080_READ`, `RTC_CHECK`) |
| 4 — routing and delegation | **`LED_TEST` done**; `GO_HOME` outstanding |
| 5 — configuration | **done** (`SET_ENV`/`GET_ENV`/`SAVE_CONFIG`/`GET_CONFIG`/`CONFIG_MODE`) |
| 6 — MQTT transport | not started |
| 7 — `PHASE` from the bulletin, cleanup | not started |

Everything through Phase 2 compiles and links for `xtensa-esp32s3-espidf`, with 34
host tests in this repo (28 protocol, 5 display, 1 console) and 40 in the installer.
**None of it has run on hardware.** Compile-success is not behaviour here: the serial
console, the I2C scan and the reboot path all need a board to be believed.

---

## 1. The one thing that changes everything

Roughly **half the diagnostics code on the old branch exists to work around the fact
that the firmware had one thread.**

The old firmware ran a single main loop that owned motion, NVS and the I²C bus. Serial
could only be serviced by *interleaving* it into whatever that loop was doing, which
produced:

| Machinery | What it is | Lines |
|---|---|---|
| `motion::MoveTick` / `MoveWatcher` / every `_watched` entry point | a callback so the stepping loop could pump serial | ~120 |
| `pump_serial_during_move` | the callback, wired at 5 call sites | ~40 |
| `sleep_answering_boot_diagnostics` + `BootServices` | 50 ms sleep slices so boot waits could answer | ~70 |
| `SerialDiagnosticsRuntime::poll_minimal` | a second, reduced dispatch path for mid-move | ~90 |
| `requires_exclusive_runtime` + `MoveTolerance` | rules for what may run mid-move, and for how long | ~90 |
| `boot_phase_response` + `announce_phase` | `ERROR BUSY <phase>` so a blocked board says why | ~60 |
| `reserve_local_control_command` | keeps MQTT and serial from running a command at once | ~80 |

**On the FSM firmware, none of that is needed.** Diagnostics gets its own thread. It is
never blocked by homing, tracking, Wi-Fi association, MQTT or OTA, so there is nothing
to interleave into and nothing to refuse for being busy.

That is not a small simplification. It is the difference between a design that fights
the runtime and one that fits it. The port should therefore be understood as
**"re-express the diagnostics protocol as an FSM"**, not as "copy files across".

What survives untouched is the part that was already transport-agnostic:

| Asset | Lines | Status |
|---|---|---|
| `crates/diagnostics` (`board_diagnostics`) | 2,044 | **Copy as-is.** Dependency-free, 25 host tests, builds on macOS. |
| `crates/infrastructure/serial_console` | 244 | **Copy as-is.** USB Serial/JTAG transport. |
| `crates/drivers/ssd1306` (`ssd1306_oled`) | 293 | **Copy as-is.** `embedded-hal` only, 5 host tests. Currently *untracked* on the other branch — commit it before porting or it is easy to lose. |
| `src/diagnostics/executor.rs` | 1,237 | **Split.** The `DiagnosticBoard` impl (the actual sensor/LED/OLED/motor work) survives; the ownership plumbing around it is replaced by FSM routing. |
| `src/diagnostics/serial.rs` | 286 | **Replaced** by a Diagnostics FSM. |
| `src/diagnostics/mqtt.rs` | 874 | **Mostly deleted.** The FSM's mailbox + the existing `services/transport.rs` already do this job. |

---

## 2. Target architecture

Today the FSM firmware runs **three threads**:

```
Motion      own thread, 100 ms period, 8 KB      MotionInit → BeginHoming → Homing ⇄ Moving → Tracking
Network     own thread,  10 ms period, 8 KB      WifiInitialize → ConnectIfDisconnected ⇄ Heartbeat / InitServices / BootValidation / Ota
Auxiliary   shared thread, 100 ms, 8 KB          Buttons + LED
```

Add a **fourth**:

```
Diagnostics own thread,  20 ms period, 12 KB     DiagIdle ⇄ DiagAwaitOwner ⇄ DiagOledWalk
```

### Why its own thread and not the Auxiliary group

This is the one threading decision that actually matters, and it is settled by a single
fact: **an `OLED_TEST` pattern write takes about a second.** The I²C bus is at 10 kHz
(`peripheral_map.rs:60`), and a full 128×64 frame is 1,104 bytes.

If Diagnostics shared the Auxiliary thread, that second would be a second in which the
**Buttons FSM does not run** — and Buttons carries the maintenance-stop input. Blocking
a safety input to draw a test pattern is not a trade worth making.

Sharing with Network is worse still: an OTA download would take the console offline for
its whole duration.

### Why one Diagnostics thread and not two (serial + MQTT)

Two transports, one catalog, one sequencer. Two threads would re-create exactly the
problem `reserve_local_control_command` was written to solve — a serial `GO_HOME` and an
MQTT `GO_HOME` arriving at once. With one FSM holding one in-flight ticket, that is
structurally impossible rather than defended against.

MQTT diagnostics therefore route *through* the Diagnostics FSM: `Network` forwards the
raw inbound payload, Diagnostics answers, and the reply goes back out as the existing
`FSMCommand::MqttPublishJson`. `services/transport.rs` keeps its envelope shape
(`{current_time, request_id, cmd, status, data}`) so the dashboard is unaffected — it
becomes pure plumbing, which is what its own doc comment already claims it is.

---

## 3. The ownership rule

The old design asked *"is the runtime busy?"*. The FSM design asks a better question:

> **A diagnostic is answered by whichever FSM owns the hardware it touches.
> Diagnostics owns the transport, the protocol, and the sequencing — not the devices.**

That turns `requires_exclusive_runtime` from a refusal list into a routing table, which
is the same anti-drift discipline in a more useful shape:

```rust
pub enum CommandOwner { Local, Motion, Led, Sensors, Unsupported }

/// Exhaustive match, no wildcard arm: a new `DiagnosticCommand` variant will not
/// compile until somebody says who owns it.
pub fn owner(command: &DiagnosticCommand) -> CommandOwner
```

| Command | Owner | Why |
|---|---|---|
| `PING`, `FIRMWARE_VERSION`, `GET_CAPABILITIES`, `PROTOCOL_VERSION` | Local | need nothing |
| `I2C_SCAN` | Local | zero-length probes are *single* bus transactions — safe from any thread |
| `OLED_TEST` | Local | nothing else on the board touches the panel |
| `SET_ENV` / `GET_ENV` / `SAVE_CONFIG` / `GET_CONFIG` / `CONFIG_MODE` | Local | Diagnostics opens its own NVS handle; see §4.4 |
| `HDC1080_READ`, `RTC_CHECK` | Sensors | multi-transaction sequences; see §4.1 |
| `LED_TEST` | Led | the LED FSM owns `Led` |
| `GO_HOME`, `MOTOR_MOVE` | Motion | the Motion FSM owns `Motion` |
| `RELAY_MOTOR`, `RELAY_HOTSPOT` | Unsupported | unimplemented, as today |
| `REBOOT` | Local | now genuinely local — no move to interrupt |

### Delegation is a state, not a refusal

The single best thing the FSM buys us. In the old firmware a `GO_HOME` blocked the whole
console for up to an hour. Here:

```
DiagIdle
  └─ GO_HOME arrives
     ├─ send FSMCommand::DiagRequest { ticket, GoHome } → FSMAddress::Motion
     └─ → DiagAwaitOwner { ticket, deadline }
             • keeps reading serial and answering anything Local
             • forwards DiagReply::Progress lines straight to the wire
             • refuses a *second* delegated command (one in flight at a time)
             • on DiagReply::Done  → emit RESULT, → DiagIdle
             • on deadline expiry  → emit RESULT … FAIL "no reply from Motion",
                                     → DiagIdle   (a wedged FSM must not wedge the console)
```

`HOMING steps_remaining=… encoder_ticks=…` progress now falls out for free: the FSM
version of homing already steps one degree at a time (`MotionHoming → MotionMoving →
MotionHoming`, ~11 s per degree), so `MotionHoming` can post progress on each pass. **No
`MoveWatcher` machinery at all.** That entire workstream is deleted rather than ported.

Motion's side reuses the pattern it already has: `MotionMaintenance` carries a
`return_to: Option<Box<dyn State>>` and resumes the caller when it finishes. A
`MotionDiagnosticHoming { ticket, return_to }` is the same shape.

### `PHASE` becomes real

The old `PHASE <name>` line was a hand-maintained string that had to be kept in step with
what the firmware was actually doing. The FSM framework already tracks this —
`Runnable::state()` returns the live state name. Post each FSM's state to the `Bulletin`
and `PHASE` becomes a derived value that cannot go stale.

`ERROR BUSY <phase>` survives, but narrows to its honest meaning: the *owning* FSM cannot
take the command right now (e.g. `GO_HOME` while already homing). The installer needs no
change — it already understands the line.

---

## 4. Hazards the FSM introduces

These are new. They do not exist on the single-threaded branch, and three of them are
exactly the failure mode this project keeps hitting: **a diagnostic that reports success
without having measured anything.**

### 4.1 Multi-transaction I²C sequences are no longer safe

`shared_bus::BusManager` locks **per transaction**. That makes single transactions safe
from any thread — which covers `I2C_SCAN`'s probes and the OLED.

It does **not** cover device sequences that span transactions:

```rust
// hdc1080::read()
self.i2c.write(ADDR, &[Register::TEMPERATURE])?;   // trigger
self.delay.delay_ms(20);                            // ← another thread can interleave here
self.i2c.read(ADDR, &mut buf)?;                     // read
```

Today `NetworkContext` owns the `Hdc1080` and the `Rtc`. If Diagnostics constructs its
own instances, a heartbeat temperature read and an `HDC1080_READ` can interleave and
return garbage — or, worse, plausible-looking garbage.

**Do not route these to Network.** Network only reaches its sensor code from
`WifiPublishHeartbeat`, which is only reachable once Wi-Fi associates. On a bench board
with no Wi-Fi, `HDC1080_READ` would simply never be answered. That is the primary
use case for the installer.

**Fix:** `PeripheralMap` exposes the multi-transaction devices behind one guard:

```rust
pub struct SensorSet {
    pub hdc1080: Option<Hdc1080<I2cProxy<'static, Mutex<I2cDriver<'static>>>, Ets>>,
    pub rtc: Rtc,
}
pub sensors: Arc<Mutex<SensorSet>>,
```

Both Network and Diagnostics hold the `Arc`. You cannot touch the sensor without holding
the lock, so the race is impossible by construction rather than by discipline — the same
idea as `BusManager`, applied one level up. About 30 lines, no FSM churn.

`Rtc::init()` (SNTP + system clock) stays a Network boot concern; it just takes the lock.

> Optional later cleanup: extract a `Sensors` FSM that owns the set outright and posts
> readings to the `Bulletin`. Cleaner ownership, but it means rewriting Network's
> heartbeat, so it is not on the critical path.

### 4.2 The LED FSM will overwrite the test colour within 100 ms

`LEDCheck::process` unconditionally re-drives the LED from the bulletin on every pass:

```rust
if state.maintenance_mode { ctx.led.display_maintenance()?; } else { ctx.led.display_none()?; }
```

An `LED_TEST RED` would survive for at most 100 ms. The LED FSM needs a real override
state — `LedOverride { color, ticket }` that holds until `LED_TEST RESTORE` or a
maintenance event pre-empts it. This is a nicer expression of the test than the old
inline version, but it is **not optional**; without it the test silently does nothing.

### 4.3 Both existing FSMs silently eat unrecognised mailbox messages

Two separate instances of the same bug, and both will swallow diagnostics traffic:

- `motion/maintenance.rs::check_maintenance` uses `mailbox.receive_latest()`, which
  **discards every queued message but the newest**, then drops anything that is not a
  button press via `_ => None`.
- `network.rs::WifiConnectIfDisconnected` reads one message and drops unmatched variants
  via `_ => {}`. It also only reads the mailbox in an `else if` branch, so nothing is
  read at all while `init_network_services` is true.

Before any `DiagRequest` is sent to either, both need a proper inbox drain: loop on
`receive()`, route by variant, and collapse "latest wins" *within* the button case rather
than across the whole queue. Small change, but it must come first or the delegated
commands will vanish without a trace.

### 4.4 Three FSMs will hold NVS handles on the same namespace

`startup()`, `MotionContext::new` and `NetworkContext::new` each already call
`EspNvs::new(partition, "storage", true)`. Diagnostics adds a fourth. ESP-IDF's NVS is
internally locked, so there is no corruption risk — but read-modify-write races are ours.

After the §5.1 fix the write sets are disjoint (boot seeding is gated off, Motion writes
only snapshot keys, Diagnostics writes only `CONFIG_KEYS`). Document that as an invariant
in `ARCHITECTURE.md`'s single-source-of-truth table.

### 4.5 Logs and protocol will share one wire

`sdkconfig.defaults` on the diagnostics branch sets `CONFIG_ESP_CONSOLE_UART=n` /
`CONFIG_USB_SERIAL_JTAG=y`, which sends `log::*` out the same peripheral the protocol
uses. That is already handled host-side by strict mode (resolve only on `RESULT`), so it
carries over — but note the FSM adds a new source: `Group::spawn` logs
`"{name} Transitioned to {state}"` on **every** transition.

Checked, and it is survivable today: Network holds between heartbeats (15 min) and Motion
transitions twice per degree (~11 s). Worth re-checking after adding a fourth machine,
and worth considering demoting that line to `debug!`.

### 4.6 The task watchdog is enabled here and disabled there

FSM `sdkconfig.defaults` has `CONFIG_ESP_TASK_WDT_EN=y` with `PANIC=y` and a 1800 s
timeout; the diagnostics branch has it off. Nothing registers with it today
(`src/logic/watchdog.rs` is `#![allow(dead_code)]` and unused), and idle-task checks are
disabled, so it is inert. Keep it enabled, and note for whoever wires `watchdog.rs` later
that `MotionMoving` blocks ~11 s per call and `MotionErrorLoop` sleeps 15 minutes.

---

## 5. Blockers found in the FSM branch

These are defects on `FSM_Diagnostics_Integration` today. Each one independently makes
part of the diagnostics feature a no-op, so they are prerequisites, not nice-to-haves.

### 5.1 Boot seeding overwrites provisioned configuration on every reboot

`network.rs:124-140` and `motion/init.rs:43-55` unconditionally write build-time constants
into NVS on every boot:

| Key | Written by | Also a provisioned `CONFIG_KEY` |
|---|---|---|
| `wifi_ssid` | `WifiInitialize` | yes |
| `wifi_pass` | `WifiInitialize` | yes |
| `tz_posix` | `WifiInitialize` | (adjacent to `tz_offset_h`) |
| `tower_latitude` | `MotionInit` | yes |
| `tower_longitude` | `MotionInit` | yes |

So a tower provisioned through the installer reverts to the flashed defaults on its next
reboot. **The entire customer-configuration feature is inert until this is gated** on
`board_diagnostics::NVS_KEY_PROVISIONED`: seed only when the flag is absent.

(This is the same defect flagged as "Boot-time seeding defeats this and must be fixed
first" in the old `DIAGNOSTICS_ROADMAP.md`. It was fixed there; the fix did not travel.)

### 5.2 `hdc1080` still swallows every I²C error

`crates/sensors/hdc1080/src/lib.rs` has **22** `.unwrap_or_default()` calls on bus
results. A failed read leaves a zeroed buffer, and zero raw counts decode to exactly
`-40.00 °C / 0.00 %RH` — which is how this reported a confident PASS on a board that had
no HDC1080 fitted at all. The diagnostics branch replaced all 22 with `?`; that fix must
come across, or `HDC1080_READ` cannot fail.

### 5.3 `rgb_led` still has the marginal WS2812 timings

FSM: `T0H 350 / T0L 800 / T1H 700 / T1L 600`. A WS2812 splits 0 from 1 at roughly 500 ns
of high time, leaving ~150 ns of margin either side at 3.3 V into a 5 V part. That is what
produced the scrambled colours — red reading as yellow, "off" glowing faint purple. The
diagnostics branch uses `T0H 300 / T0L 850 / T1H 850 / T1L 450`.

Port the timings **only**. The FSM version's API is better and should be kept: its
`display_*` methods return `Result<()>`, and it adds `display_maintenance_moving_cw/ccw`
and `display_none`, none of which exist on the other branch. (`FRAME_REPEATS = 3` can be
left behind — it does not help the corrupted-but-latched case.)

### 5.4 `sdkconfig.defaults` has no USB Serial/JTAG console config

Needs the four lines from the diagnostics branch (`CONFIG_ESP_CONSOLE_USB_CDC=n`,
`CONFIG_USB_SERIAL_JTAG=y`, `CONFIG_ESP_CONSOLE_UART=n`, TinyUSB CDC off).

### 5.5 `.cargo/config.toml` is missing `--before usb-reset`

Without it, a board already running firmware that claims USB Serial/JTAG refuses to
flash: espflash's default DTR/RTS reset only reaches the ROM while the peripheral is
unclaimed, and this firmware claims it in the first milliseconds. This bites the moment
the serial console is added, and presents as "Failed to connect to the device" — easy to
misdiagnose as a cable or a busy port.

### 5.6 Minor, worth folding in while nearby

- `MotionInit` builds **two** identical `Clock` instances (`ctx.calculation` and
  `ctx.clock`, lines 100 and 163) — two I²C proxies for one job.
- `FSMCommand::UpdateNetworkMotionContext(mode, heading)` is a mailbox message doing a
  bulletin's job. Once `FSMState` carries the live snapshot for `PHASE` and `GET_STATUS`,
  this variant can be deleted.
- `ARCHITECTURE.md` still documents `src/runtime/`, `src/diagnostics/` and a single main
  loop. It describes the pre-FSM layout throughout and will mislead anyone using it to
  find their way around this port.

---

## 6. Phased plan

Each phase ends at a clean `cargo check`, and every phase that touches the protocol crate
ends at green host tests (`cargo +stable test -p diagnostics --target aarch64-apple-darwin`).

**Phase 0 — prerequisites** — done *(no diagnostics code yet)*

0. **Make the branch build at all.** It does not, on this machine: `network` fails with
   three `include_str!` errors because this worktree has no device certificates and no
   `.env`. Note the path moved in the FSM refactor — certs now live at repo-root
   `certs/`, not `crates/infrastructure/network/`. Copy `AmazonRootCA1.pem`,
   `tower_1-certificate.pem.crt` and `tower_1-private.pem.key` into `certs/` (they are in
   `crates/infrastructure/network/` on the other worktree) and add a `.env` with at least
   `DEVICE_ID=1`. `build.rs` falls back per key, so a partial `.env` is safe.
1. Commit `crates/drivers/ssd1306` on the source branch before it is lost.
2. Fix §5.2 (hdc1080 `?`), §5.3 (LED timings), §5.4 (sdkconfig), §5.5 (usb-reset).
3. Fix §4.3 (mailbox drains in Motion and Network) — everything later depends on it.
4. Fix §5.1 (gate boot seeding on `provisioned`).
*Verify: flash, confirm colours are stable and `HDC1080` reports a real reading or a real
error. This phase is independently valuable even if the port stalls.*

**Phase 1 — land the portable crates** — done
Copy `crates/diagnostics`, `crates/infrastructure/serial_console`, `crates/drivers/ssd1306`
into the workspace; add to `[workspace.members]` and `[workspace.dependencies]`. No
firmware wiring.
*Verify: 25 + 5 host tests pass; `cargo check` on target unchanged.*

**Phase 2 — the Diagnostics FSM skeleton** — done
`src/logic/fsm/diagnostics/` with `DiagnosticsContext` (console, own NVS handle,
`ConfigStaging`, `&'static` bus, `Arc<Mutex<SensorSet>>`), and `DiagIdle` answering only
the Local set: `PING`, `FIRMWARE_VERSION`, `GET_CAPABILITIES`, `I2C_SCAN`, `REBOOT`.
Add `FSMAddress::Diagnostics`, bump `Address::count()`, spawn at 20 ms / 12 KB.
*Verify: installer connects, Firmware Version and I2C Scan pass **while the tower is
homing** — the whole point, demonstrated with five commands.*

**Phase 3 — Local hardware commands**
`OLED_TEST` (full five-pattern walk now unconditionally allowed — nothing to stall) and
the sensor commands behind the `SensorSet` mutex. Port the `DiagnosticBoard` impl out of
`executor.rs`, dropping the `MotionContext` / `nvs: Option<_>` / `allow_long_blocking`
plumbing.
*Verify: installer's OLED and HDC1080 walkthroughs, during homing.*

**Phase 4 — routing and delegation**
`CommandOwner`, `FSMCommand::{DiagRequest, DiagReply}`, `DiagAwaitOwner`. `LED_TEST` →
LED FSM (with §4.2's override state). `GO_HOME` → Motion FSM, with per-degree progress.
*Verify: `LED_TEST` mid-homing; `GO_HOME` from the installer with the console still
answering `PING` throughout; a deliberately unanswered ticket times out cleanly.*

**Phase 5 — configuration**
`SET_ENV` / `GET_ENV` / `SAVE_CONFIG` / `GET_CONFIG` / `CONFIG_MODE` against the
Diagnostics NVS handle.
*Verify: full provisioning run from the installer, then **power-cycle and re-read** — this
is what proves §5.1 is actually fixed.*

**Phase 6 — MQTT transport**
Network forwards inbound command payloads to Diagnostics; replies return as
`MqttPublishJson`. `services/commands.rs::dispatch` becomes a JSON adapter over the same
`DiagnosticCommand` catalog, keeping `get_status` and the existing envelope.
*Verify: `get_status` unchanged from the dashboard's point of view; a second command
proves the catalog is shared.*

**Phase 7 — `PHASE` from the bulletin, and cleanup**
Extend `FSMState` with the live snapshot; derive `PHASE`; delete
`UpdateNetworkMotionContext`; rewrite `ARCHITECTURE.md`; port `DIAGNOSTICS.md` and the
protocol docs.

---

## 7. What the installer needs

**Nothing.** That is worth stating explicitly, and worth verifying rather than assuming.

The wire format does not change: same commands, same `RESULT <CMD> PASS|FAIL <k=v>`,
same `PROTOCOL_VERSION 1`, same legacy lines, same `ERROR BUSY <phase>`. The set of
commands that answer `BUSY` shrinks to almost nothing, which the installer already
handles — it just stops seeing them.

Two things to re-check on hardware rather than reason about:

- `COMMAND_TIMEOUTS.GO_HOME` is a 30 s silence budget with a 1 h ceiling. Progress now
  arrives about once per 11 s (one degree per `MotionMoving`), comfortably inside 30 s —
  but confirm the cadence on a real sweep.
- `COMMAND_TIMEOUTS.I2C_SCAN` is 20 s / 30 s, sized for a scan that had to share a
  thread with everything else. It can probably come down a long way.

---

## 8. Decisions taken, and why

| Decision | Alternative rejected | Reason |
|---|---|---|
| Diagnostics on its own thread | join the Auxiliary group | a 1 s OLED write would block the maintenance button |
| One Diagnostics FSM, two transports | separate serial and MQTT machines | one in-flight ticket makes concurrent commands impossible instead of merely defended-against |
| Route by hardware owner | keep a "can this run now" predicate | the old predicate answered "is the runtime busy", which is no longer a meaningful question |
| Sensors behind `Arc<Mutex<SensorSet>>` | route sensor commands to Network | Network only reaches its sensor code once Wi-Fi associates; a bench board would never answer |
| Delete `MoveWatcher` / `MoveTolerance` | port them | both exist solely to interleave work into a stepping loop that Diagnostics no longer shares |
| Keep the FSM `rgb_led` API, port only the timings | take the diagnostics-branch file | the FSM version is strictly better — `Result` returns, three extra display modes |
| Extract a `Sensors` FSM later, if at all | do it now | it forces a Network heartbeat rewrite for no correctness gain over the mutex |

---

## 9. Open questions

1. **`RELAY_HOTSPOT` pin assignment** — still undefined in either firmware. Blocks that
   command; unchanged from before.
2. **`MOTOR_MOVE` argument format and travel limit** — proposed previously as
   `MOTOR_MOVE <CW|CCW> <degrees>` bounded to ~10°. Now easier to do safely, since it is
   a delegated request to the Motion FSM rather than a re-entrant call into it.
3. **I²C bus clock.** Still 10 kHz. Every device on it (SSD1306, HDC1080, DS3231) supports
   400 kHz, which would take a full OLED frame from ~994 ms to ~25 ms. Not raised
   deliberately, since this board has already shown marginal signalling on the LED line
   and a bus error is a worse failure than a slow test. With Diagnostics on its own thread
   the pressure to raise it is much lower — the second no longer costs anything but the
   second.
4. **Service hold.** The old roadmap wanted a TTL-bounded mode that suspends tracking so
   an installer can work safely. It maps cleanly onto a `MotionServiceHold { deadline }`
   state and is much more natural here than it was before. Worth doing, as its own verb
   (`SERVICE_HOLD`), not folded into `CONFIG_MODE`.
5. **Does `MotionMoving`'s 11 s block need breaking up?** Maintenance button presses
   currently wait up to 11 s to be seen. Pre-existing and out of scope for this port, but
   it is the same class of problem, and `move_by` could return after N steps and be
   re-entered the way homing already is.
