# Future Additions

Features and fixes developed on the `deployment` branch that are not yet promoted to `master`.
These were removed from `deployment` to bring it back in sync with `master`, and are preserved
here as a reference for future integration.

---

## 1. Limit-Switch Failure Recovery (LMSW Bypass + Reboot)

**Commits:** `116e95b`, `398a0e9`, `d7facf9`
**Files touched:**
- `src/runtime/infra/lmsw_recovery.rs`
- `src/runtime/infra/snapshot_store.rs`
- `src/runtime/infra/mod.rs`
- `src/runtime/app/tracking_loop.rs`
- `src/runtime/app/encoder_fault.rs`
- `src/runtime/main.rs`
- `motion/src/lib.rs`
- `motion/src/motion/homing.rs`

### What it does

When a limit-switch search fails during boot homing or sunset homing, instead of wedging
in an error loop indefinitely, the device enters **LMSW bypass mode**:

1. Saves `lmsw_bypass = 1` in NVS.
2. Calls `motion.encoder_return_to_zero(home_heading_deg)` to unwind the tower to its
   software-zero position using encoder feedback alone.
3. Saves heading + encoder snapshot to NVS (so the next boot trusts state).
4. Sets `lmsw_post_rb = 1` in NVS (signals to the next boot that it came from a bypass
   reboot, so it skips the limit-switch search).
5. Waits 20 seconds, then calls `restart()`.

On the next boot, if `lmsw_bypass` is active, tracking continues in
`EncoderGuarded` mode without requiring a limit-switch homing sweep.
When a homing sweep later **succeeds** (switch repaired or reconnected),
`on_lmsw_homing_success()` clears both `lmsw_bypass` and `lmsw_post_rb`.

To fully reset bypass state without a successful homing, flash-erase NVS.

### NVS keys added

| Key | Type | Meaning |
|-----|------|---------|
| `lmsw_bypass` | `u8` (0/1) | Bypass active: skip switch searches |
| `lmsw_post_rb` | `u8` (0/1) | Set before reboot-after-unwind; cleared on next bypass boot |

### Why it was pulled back

Still in test — `motion::encoder_return_to_zero` and the boot-path bypass routing in
`main.rs` need field validation before merging to `master`.

---

## 2. WiFi Best-Effort Reconnect

**Committed as:** `5e7ce39` on `deployment` (re-applied below as part of the wifi fix)
**File:** `crates/infrastructure/wifi/src/lib.rs`

### What it does

`reconnect_if_disconnected()` was changed from a fallible function (propagating errors
via `?`) to a best-effort one (logging errors and always returning `Ok(())`).

Two root bugs fixed:
- `start()` was called on an already-running driver after a connection drop, which
  returns `ESP_ERR_INVALID_STATE` and short-circuits before `connect()` runs — so WiFi
  never actually reconnected. `start()` is now omitted from the reconnect path.
- Any error from `connect()` or `wifi_wait_while()` previously propagated up through
  `?` into `app_main()`, causing the entire program to exit (all RAII destructors fire,
  `Returned from app_main()` in the logs). Errors are now logged as warnings and
  swallowed so the tower keeps moving regardless of WiFi state.
- Timeout raised from 10 s → 30 s to accommodate AWS IoT Core's public DNS lookup +
  full mTLS handshake latency.