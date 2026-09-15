//! Catalog of remote commands that **move** the tower — the counterpart to
//! [`crate::diagnostics::commands`], which is read-only by construction.
//!
//! Kept in its own module for one reason: these handlers need `&mut Motion` and
//! `&mut EspNvs`, and putting those in `CmdCtx` would let every future `get_*`
//! command drive the motor by accident.
//!
//! To add a command:
//!   1. write a handler `fn xxx(ctx: &mut MotionCmdCtx) -> Result<Value, String>`
//!   2. add one arm to [`dispatch`]
//!
//! `transport.rs` does not change as commands grow.
//!
//! These exist for commissioning. Before the limit switch is mounted the tower
//! has no home reference, so the installer reads the heading, sends the delta
//! needed to reach `home_heading_deg`, mounts the switch there, and commits it
//! with `set_home_here`.

use esp_idf_svc::nvs::{EspNvs, NvsDefault};
use log::{info, warn};
use motion::{calculate_steps, Motion, MoveOutcome};
use serde_json::{json, Value};

use crate::{infra::SnapshotStore, switchboard::Switchboard};

/// Matches the boot-phase and loop-step persistence flags in `runtime::main`.
const PERSIST_NVS: bool = true;

/// Mutable view of tower state that movement commands may drive.
///
/// Built fresh each time a command is processed, from disjoint `Tower` fields.
/// Unlike [`crate::diagnostics::commands::CmdCtx`] this is deliberately *not*
/// `Copy` and *not* read-only — holding one means you can turn the tower.
pub struct MotionCmdCtx<'a> {
    pub motion: &'a mut Motion<'static>,
    pub nvs: &'a mut EspNvs<NvsDefault>,
    /// The tower's dead-reckoned heading, updated in place after a completed move.
    pub actual_heading: &'a mut f32,
    /// Flipped to `true` by [`set_home_here`], which is what establishes the
    /// home reference in the first place.
    pub heading_trusted: &'a mut bool,
    pub sw: Switchboard,
    /// Set by [`exit_install`]. `process_commands` publishes the ack first,
    /// then reboots — the handler cannot restart inline because the reply
    /// has not gone out yet.
    pub reboot_requested: &'a mut bool,
}

impl MotionCmdCtx<'_> {
    /// Whether the tower is standing still enough to accept a commanded move.
    ///
    /// A tracking tower would walk the pose back toward the sun on its next
    /// iteration, so "succeeding" there would be a lie. Install and Admin images
    /// both have tracking off, which is exactly when jogging makes sense.
    fn movement_allowed(&self) -> bool {
        self.sw.install_mode || !self.sw.runtime.tracking.enabled
    }

    /// Bound a requested move, returning the degrees actually permitted.
    ///
    /// Note this is the **only** bound that applies. The soft limits are read in
    /// exactly one place in the firmware — the daytime tracking branch in the
    /// motion crate — so a raw `move_by` is otherwise completely unclamped.
    fn bound(&self, requested: f32) -> Result<f32, String> {
        if !*self.heading_trusted {
            // Soft limits are positions, not distances, and there is no home
            // reference yet: `actual_heading` is sitting at its default of
            // `home_heading_deg`, which *is* `soft_limit_min_deg`. Clamping
            // against it would refuse every CCW move — including the one the
            // installer actually needs. Bound the distance instead.
            let cap = self.sw.install_max_step_deg;
            if requested.abs() > cap {
                return Err(format!(
                    "`degrees` must be within ±{cap} while the heading is untrusted (got {requested})"
                ));
            }
            return Ok(requested);
        }

        if !self.sw.runtime.guardrails.soft_limits_enabled {
            return Ok(requested);
        }

        let min = self.sw.runtime.guardrails.soft_limit_min_deg;
        let max = self.sw.runtime.guardrails.soft_limit_max_deg;
        let target = (*self.actual_heading + requested).clamp(min, max);
        let applied = target - *self.actual_heading;

        if applied.abs() < f32::EPSILON {
            return Err(format!(
                "refused: heading {:.2} is already at the soft limit ({min}..{max})",
                *self.actual_heading
            ));
        }
        Ok(applied)
    }
}

/// Route a command name to its handler.
///
/// Returns `None` for a name this catalog does not own, so the transport can
/// fall through to its "unsupported command" reply. `Some(Err(msg))` is a
/// refusal the operator should see — `msg` goes out as the error reply.
pub fn dispatch(
    cmd: &str,
    degrees: Option<&Value>,
    ignore_encoder: bool,
    ctx: &mut MotionCmdCtx<'_>,
) -> Option<Result<Value, String>> {
    match cmd {
        "move_by" => Some(move_by(degrees, ignore_encoder, ctx)),
        "set_home_here" => Some(set_home_here(ctx)),
        "exit_install" => Some(exit_install(ctx)),
        _ => None,
    }
}

/// `move_by` — turn the tower by a signed number of degrees.
///
/// Sign carries direction, matching `calculate_steps` and `Motion::move_by`
/// everywhere else in the firmware: `+` is CW, `-` is CCW. So `-110` turns 110°
/// counter-clockwise; there is no separate direction field to disagree with it.
fn move_by(
    degrees: Option<&Value>,
    ignore_encoder: bool,
    ctx: &mut MotionCmdCtx<'_>,
) -> Result<Value, String> {
    if !ctx.movement_allowed() {
        return Err(
            "refused: tracking is enabled; move_by is only accepted on an Install or Admin image"
                .into(),
        );
    }

    let requested = parse_degrees(degrees)?;
    let applied = ctx.bound(requested)?;
    let from_heading = *ctx.actual_heading;

    info!(
        "move_by: requested={requested}° applied={applied}° from_heading={from_heading}° \
         trusted={} bypass={ignore_encoder}",
        *ctx.heading_trusted
    );

    // Stall and overshoot detection compare deltas *within* the move, so a
    // missing home reference does not upset them — they only fire when the
    // encoder is genuinely dead or unwired. Bypassing is therefore an explicit
    // request, for a tower whose encoder is not connected yet, and it is echoed
    // back in the reply so an unprotected move is visible afterwards.
    let stall_prev = ctx.motion.stall_detection_enabled();
    if ignore_encoder {
        warn!("move_by: encoder guardrails bypassed by request");
        ctx.motion.set_stall_detection_enabled(false);
    }

    let outcome = ctx.motion.move_by(calculate_steps(applied));

    if ignore_encoder {
        ctx.motion.set_stall_detection_enabled(stall_prev);
    }

    if outcome == MoveOutcome::Completed {
        let to_heading = from_heading + applied;
        *ctx.actual_heading = to_heading;
        ctx.motion.update_position(to_heading);

        let mut store = SnapshotStore::new(ctx.nvs, PERSIST_NVS);
        store.save_heading(to_heading);
        // Never persist an encoder snapshot from here, and drop any that a
        // previous run left behind. `should_home_by_mode` in `runtime::main`
        // skips the homing sweep outright when it finds a restorable snapshot —
        // so a snapshot written or kept across an operator move would let the
        // next boot trust an encoder zero measured from a home the tower has
        // since moved away from, and silently never home again. Clearing it
        // guarantees the next Normal boot re-establishes home for real.
        store.clear_encoder_snapshot();
    } else {
        warn!(
            "move_by aborted: {:?}; heading not updated or persisted",
            outcome
        );
    }

    Ok(json!({
        "requested_degrees": requested,
        "applied_degrees": applied,
        "from_heading": from_heading,
        "to_heading": *ctx.actual_heading,
        "outcome": outcome_str(&outcome),
        "heading_trusted": *ctx.heading_trusted,
        "lmsw_active": ctx.motion.lmsw_active(),
        "guardrails_bypassed": ignore_encoder,
    }))
}

/// `set_home_here` — commit the tower's current pose as mechanical home.
///
/// The last step of an install: the operator has jogged the tower to the right
/// place and mounted the limit switch there, so the encoder is zeroed against
/// the switch and the heading becomes `home_heading_deg`.
fn set_home_here(ctx: &mut MotionCmdCtx<'_>) -> Result<Value, String> {
    if !ctx.motion.lmsw_active() {
        return Err(
            "refused: limit switch is not pressed — mount it at the intended home position first"
                .into(),
        );
    }

    // Establishes the encoder zero against the physical switch. The read/stash/
    // zero ordering inside is load-bearing (it also captures the home-error
    // metric), so call the helper rather than re-zeroing by hand.
    ctx.motion.force_zero_if_limit_switch_pressed();

    let home = ctx.sw.home_heading_deg;
    *ctx.actual_heading = home;
    ctx.motion.update_position(home);
    *ctx.heading_trusted = true;

    let mut store = SnapshotStore::new(ctx.nvs, PERSIST_NVS);
    store.save_heading(home);
    // Deliberately no encoder snapshot here either, for the same reason as in
    // `move_by`: the first Normal boot after an install should prove the switch
    // through the real homing sweep, not inherit a zero from a commissioning
    // image. The whole Install path leaves NVS with no snapshot.
    store.clear_encoder_snapshot();

    info!("set_home_here: committed {home}° as home; encoder zeroed against the limit switch");

    Ok(json!({
        "heading": home,
        "heading_trusted": true,
        "lmsw_active": true,
    }))
}

/// `exit_install` — one-way latch back to Normal, then reboot.
///
/// Writes `install_done` so the next boot of this same Install image behaves
/// as Normal, with no reflash. There is no reverse path: nothing remote can
/// put a producing tower into Install. Re-entering needs an erase-flash.
///
/// Does not require the limit switch. If it is not pressed, the following
/// Normal boot will sweep ~350° looking for it — that sweep is no longer
/// fatal (a miss inhibits tracking instead of wedging).
fn exit_install(ctx: &mut MotionCmdCtx<'_>) -> Result<Value, String> {
    if !ctx.sw.install_mode {
        return Err("refused: not an Install image".into());
    }

    SnapshotStore::new(ctx.nvs, PERSIST_NVS).save_install_done(true);
    *ctx.reboot_requested = true;

    let lmsw_active = ctx.motion.lmsw_active();
    let note = if lmsw_active {
        "limit switch is pressed; Normal boot should find home immediately"
    } else {
        "limit switch is not pressed; Normal boot will sweep ~350° (~40 min) looking for it"
    };
    info!("exit_install: install_done set; rebooting as Normal. {note}");

    Ok(json!({
        "lmsw_active": lmsw_active,
        "note": note,
    }))
}

/// Pull `degrees` out of the payload with a message naming what was wrong.
///
/// Taken as a raw [`Value`] rather than typed by `serde` so a bad value reports
/// itself, instead of failing the whole payload as "not valid JSON".
fn parse_degrees(degrees: Option<&Value>) -> Result<f32, String> {
    let Some(raw) = degrees else {
        return Err("move_by requires a `degrees` field".into());
    };
    let Some(value) = raw.as_f64() else {
        return Err(format!("`degrees` must be a number (got {raw})"));
    };
    let value = value as f32;
    if !value.is_finite() {
        return Err("`degrees` must be a finite number".into());
    }
    if value == 0.0 {
        return Err("`degrees` must be non-zero".into());
    }
    Ok(value)
}

/// Stable snake_case name for a move outcome, matching the telemetry payloads.
fn outcome_str(outcome: &MoveOutcome) -> &'static str {
    match outcome {
        MoveOutcome::Completed => "completed",
        MoveOutcome::AbortedPowerMissing => "aborted_power_missing",
        MoveOutcome::AbortedStall => "aborted_stall",
        MoveOutcome::AbortedOvershoot => "aborted_overshoot",
    }
}
