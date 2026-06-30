# Tower Firmware — Architecture

This document explains **what is where and why** in the tower firmware, so a new
teammate can find their way around and extend the code without breaking it.

The firmware runs on an **ESP32-S3** (Rust, `esp-idf-svc`). One tower = one
device, talking to AWS IoT Core over MQTT. Each device has its own identity
(`tower_{DEVICE_ID}`) and per-device TLS certs.

---

## 1. The 30-second mental model

The code is **layered**, and each layer only knows about the one below it:

```
        ┌─────────────────────────────────────────────┐
        │  main.rs  — the ORCHESTRATOR                 │   "wire it up + run the loop"
        ├─────────────────────────────────────────────┤
        │  src/runtime/app  — APPLICATION LOGIC        │   tracking, homing, recovery
        │  src/diagnostics  — REMOTE COMMAND CHANNEL   │   talk to a live tower
        ├─────────────────────────────────────────────┤
        │  src/runtime/infra — RUNTIME SERVICES        │   telemetry, NVS, temperature
        ├─────────────────────────────────────────────┤
        │  crates/* , motion/ , rtc/  — LIBRARIES      │   chips, wifi, mqtt, time, motors
        └─────────────────────────────────────────────┘
```

Three things shape the runtime, and they're the most important to understand:

1. **Switchboard** decides *what is turned on* (feature flags + profiles).
2. **Tower** context holds *the live state* (one struct instead of loose variables).
3. **Command channel** lets us *talk to a tower over MQTT* without reflashing.

---

## 2. Repository layout

The repo is a Cargo **workspace**: a set of small library crates plus the
`tower` binary that ties them together.

```
.
├── Cargo.toml          # workspace + the `tower` binary package
├── build.rs            # generates src/constants.rs from .env at build time
├── .env                # secrets + per-device config (GITIGNORED — never commit)
├── crates/             # reusable libraries (no app knowledge)
├── motion/             # motor + encoder + limit-switch + homing
├── rtc/                # real time (DS3231 + SNTP)
└── src/                # the tower application itself
```

### 2a. Library crates (`crates/`, `motion/`, `rtc/`)

These are **self-contained** and know nothing about "the tower app." That
boundary is the point: a driver can't reach into application logic, so it stays
reusable and testable on its own.

| Crate | What it is | Why it's separate |
|---|---|---|
| `crates/sensors/ds323x` | DS3231 RTC chip driver | Pure chip driver, vendored so we pin the version |
| `crates/sensors/hdc1080` | HDC1080 temp/humidity chip driver | Pure chip driver |
| `crates/sensors/bno080` | BNO080 IMU (orientation) chip driver | Pure chip driver |
| `crates/sensors/sensors` | Sensor aggregation over the chip drivers | One place to read "the sensors" |
| `crates/drivers/accel-stepper` | Stepper motor driver | Reusable motor primitive |
| `crates/drivers/quadrature-decoder` | Encoder decoding | Reusable encoder primitive |
| `crates/drivers/rgb_led` | Status LED | Reusable driver |
| `crates/ui/buttons` | Physical button inputs | Reusable input layer |
| `crates/infrastructure/clock` | Sun position / sunrise-sunset math (`Clock`) | Isolated astronomy, testable alone |
| `crates/infrastructure/wifi` | Wi-Fi connect / reconnect | Infra service |
| `crates/infrastructure/network` | **MQTT client + all telemetry** | Single source of truth for anything on the wire |
| `crates/infrastructure/ota` | Over-the-air firmware updates | Infra service |
| `rtc/` | Real time: DS3231 + SNTP fallback, sets the system clock | "What time is it" shouldn't live in `main` |
| `motion/` | Motor + encoder + limit-switch control, homing primitives | The mechanical heart; big enough to own |

> **The one rule to remember:** anything that goes over MQTT — a topic name or a
> JSON payload shape — is defined in
> [`crates/infrastructure/network/src/telemetry.rs`](crates/infrastructure/network/src/telemetry.rs),
> nowhere else. That's why the firmware and the dashboard never disagree.

### 2b. The application (`src/`)

This is the only place that knows "we are a solar tracker."

```
src/
├── runtime/
│   ├── main.rs              # the orchestrator: boot phases, then the tracking loop
│   ├── infra/              # cross-cutting RUNTIME services (need app context)
│   │   ├── telemetry.rs     #   error_loop: the "wedge + report on fatal" helper
│   │   ├── snapshot_store.rs#   all NVS (persisted state) read/write in one place
│   │   └── temperature.rs   #   reads HDC1080, publishes temperature telemetry
│   └── app/                # the APPLICATION logic / loop steps
│       ├── tracking_loop.rs #   one tracking "tick" (compute sun → move → publish)
│       └── encoder_fault.rs #   encoder drift detection + recovery
├── diagnostics/            # the remote command channel (talk to a live tower)
│   ├── transport.rs         #   MQTT plumbing: subscribe, receive, reply
│   └── commands.rs          #   the command CATALOG (get_status, future get_*)
├── constants.rs            # GENERATED from .env by build.rs — do not hand-edit
└── switchboard.rs          # feature flags + Normal/Admin/Custom profiles
```

**Why the `infra` vs `app` split?**
- **`infra/`** = runtime *services* that need our app context (NVS keys,
  telemetry topics) and therefore can't be plain standalone crates.
- **`app/`** = the actual *business logic* of tracking and recovery.

---

## 3. Boot sequence (`src/runtime/main.rs`)

`main.rs` should read top-to-bottom like a story. It runs a series of boot
phases, then enters the main loop:

1. **Init** — take peripherals, set up the shared I2C bus, NVS, logging.
2. **Network** — bring up Wi-Fi, then connect MQTT as `tower_{DEVICE_ID}`.
3. **Boot validation** — sanity checks before doing anything mechanical.
4. **OTA** — check for / apply a firmware update (gated by the switchboard).
5. **Motion init** — initialize the motor / encoder / motion mode.
6. **State restoration** — restore heading and encoder snapshot from NVS.
7. **Homing** — find the limit switch if state couldn't be restored.

After phase 7, all the long-lived state is gathered into **one `Tower` struct**
(see §4.3), and the **main loop** drives `tower.*`:

```
loop {
    daily reset / re-home as needed
    tracking tick
    heartbeat + telemetry
    temperature report
    process one remote command
    sleep
}
```

---

## 4. The three runtime concepts

These are the parts most worth understanding before you change anything.

### 4.1 Switchboard — *"what's turned on"*

File: [`src/switchboard.rs`](src/switchboard.rs)

A single struct of feature flags (tracking on/off, OTA on/off, boot homing,
command channel, guardrails, soft limits…) plus three **profiles**:

- **`Normal`** — production. Its values match what's running on the fleet today.
- **`Admin`** — a **diagnostics sandbox**: tracking, boot homing, and OTA are
  turned **off** so the tower stays put and nothing competes with the feature
  under test; the command channel stays **on**.
- **`Custom`** — a hook for site-specific images; identical to `Normal` until
  customized.

The active profile is chosen at boot from `ACTIVE_PROFILE` in `.env`:

```rust
let sw = switchboard::active(
    switchboard::Profile::from_env_str(crate::constants::ACTIVE_PROFILE_STR)
);
```

**Why it exists:** to change *behavior* without changing *code*. Want to poke at
a tower safely? Flash it with `ACTIVE_PROFILE=Admin` — no code edits, and you
can't accidentally let it track or OTA mid-test. The golden rule: `Normal`'s
values mirror the fleet, so flipping the profile is the *only* behavior change.

### 4.2 Command channel — *"talk to a tower"*

Files: [`src/diagnostics/transport.rs`](src/diagnostics/transport.rs),
[`src/diagnostics/commands.rs`](src/diagnostics/commands.rs)

Request/response over MQTT. The cloud publishes a command; the tower replies on
an ack topic.

- **Topics:**
  - in:  `tower/{id}/cmd/diagnostics`
  - out: `tower/{id}/cmd/diagnostics/ack`
- **Request:** `{ "cmd": "get_status", "request_id": "abc" }` (`request_id`
  optional, echoed back so the caller can correlate replies).
- **Reply envelope** (always one of):
  ```json
  { "current_time": "...", "request_id": "...", "cmd": "...", "status": "ok",    "data": { ... } }
  { "current_time": "...", "request_id": "...", "cmd": "...", "status": "error", "message": "..." }
  ```

**The split is the whole point:**
- `transport.rs` = the plumbing (subscribe, parse, route, reply). **It does not
  change when you add a command.** Malformed input still gets an error reply, so
  a bad payload can't wedge the queue.
- `commands.rs` = the catalog. The single place to answer "what can a tower do?"

**To add a command** (3 steps, all in `commands.rs`):
1. write a handler `fn get_xxx(ctx: &CmdCtx) -> Value`
2. add one arm to `dispatch`
3. if it needs more state, add a field to `CmdCtx` (and populate it in `main.rs`)

`CmdCtx` is a read-only snapshot of tower state, rebuilt each loop iteration and
handed to the dispatcher. Today it carries device id, firmware version, MQTT/Wi-Fi
connectivity, motion mode, and current heading.

### 4.3 Tower context — *"the live state"*

File: [`src/runtime/main.rs`](src/runtime/main.rs)

A `Tower<I2C>` struct holds all the long-lived runtime state — motion, mqtt,
wifi, nvs, clock, sensors, version, heading, motion mode, switchboard, etc. — in
one place. It's built once at the end of boot, and the loop drives `tower.*`.

It's generic over the I2C proxy type (`Tower<I2C>`) to avoid spelling out the
verbose `shared_bus::I2cProxy<'static, …>` everywhere.

**Why it exists:** before this, the loop juggled ~12 loose variables, and any
logic you tried to extract needed a dozen arguments. With one context, loop steps
can become clean methods. This is the foundation that lets `main.rs` shrink (see
the roadmap in §8).

---

## 5. Conventions / single-source-of-truth

Teach these four and most "where does X go?" questions answer themselves:

| Concern | The one place it lives |
|---|---|
| Anything on the MQTT wire (topics, payloads) | `crates/infrastructure/network/src/telemetry.rs` |
| What features are on/off | `src/switchboard.rs` (or flip `ACTIVE_PROFILE`) |
| Remote commands | `src/diagnostics/commands.rs` |
| Persisted (across-reboot) state | `src/runtime/infra/snapshot_store.rs` |
| Tuning constants / coords / creds | `.env` → generates `src/constants.rs` |

---

## 6. Build & configuration

- **`.env`** holds secrets (Wi-Fi password, MQTT creds) and per-device config
  (`DEVICE_ID`, lat/lon, TZ, `ACTIVE_PROFILE`). It is **gitignored — never commit
  it.** See `.env.example` for the shape.
- **`build.rs`** reads `.env` at build time and generates `src/constants.rs`.
  Do not hand-edit `constants.rs`; change `.env` and rebuild.
- Each device flashes with its own `DEVICE_ID` and its own TLS client certs
  (`tower_{DEVICE_ID}-*.pem.*`, found in `certs/`), which gives the device its fleet identity.

---

## 7. Cheat-sheet: how to do common things

- **Add a remote command** → `src/diagnostics/commands.rs` only.
- **Change a topic or JSON payload** → `network/src/telemetry.rs` only.
- **Turn a feature off for testing** → `src/switchboard.rs`, or set
  `ACTIVE_PROFILE=Admin` in `.env`. Not `main.rs`.
- **Change a tuning constant (speeds, limits, coords)** → `.env`, then reflash.
- **What persists across reboots?** → everything in `snapshot_store.rs`.
- **Where does the tower actually move?** → `motion/` (primitives), called from
  `src/runtime/app/tracking_loop.rs` (decisions).

---

## 8. Roadmap (in progress)

The `Tower` refactor is being done in steps so each one is verifiable by a clean
`cargo check` (we have no hardware to test on, so compile-success = behavior
preserved for pure-rename refactors):

- **Step 1 — done.** Consolidate loop state into the `Tower<I2C>` context struct;
  loop drives `tower.*`.
- **Step 3 — next (lower risk).** Convert the loop's inline blocks (daily reset,
  re-home, tracking, heartbeat, temperature, commands) into `Tower` methods, so
  the loop reads as `tower.track(); tower.report(); tower.process_commands();`.
- **Step 2 — after Step 3 (higher risk).** A `Tower::bringup()` constructor that
  folds boot phases 1–7 into one call. Lifetime-heavy, so it's last.

**End state:** `main.rs` is a one-screen story, and each step is independently
understandable. Plus an ever-growing catalog of `get_*` diagnostics commands.

> Nothing promotes to `deployment` (the fleet) without a hardware flash-test
> first. `testing` is staging.
