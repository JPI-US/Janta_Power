use std::time::Duration;

use log::{error, warn};
use motion::{calculate_steps, Motion};

use crate::switchboard::RecoverySwitches;

/// Run the boot-time recovery move sequence (unstick/back-off).
///
/// This is intentionally "dumb": it just executes the configured moves and (optionally)
/// verifies encoder delta after each move.
pub fn run(motion: &mut Motion<'_>, cfg: &RecoverySwitches) {
    // Convention for *current wiring*: positive step movement = physical CW.
    let tol_ticks = motion.encoder_ticks_for_deg(cfg.verify_tol_deg.abs()).abs();
    let mut all_ok = true;

    // Bench convenience: optionally disable stall detection for recovery moves.
    let prev_stall = motion.stall_detection_enabled();
    if cfg.disable_stall_detection {
        warn!("RECOVERY: disabling stall detection for recovery moves");
        motion.set_stall_detection_enabled(false);
    }

    for (idx, mv) in cfg.moves.iter().enumerate() {
        let move_n = idx + 1;
        let signed_deg = mv.dir.apply_to_deg(mv.deg);
        let steps = calculate_steps(signed_deg);

        warn!(
            "RECOVERY_MOVE_{}: dir={} deg={:.1} signed_deg={:+.1} steps={} tol_ticks={}",
            move_n,
            mv.dir.as_str(),
            mv.deg,
            signed_deg,
            steps,
            tol_ticks
        );

        let enc_start = motion.encoder_ticks_raw();
        let outcome = motion.move_by(steps);
        let enc_end = motion.encoder_ticks_raw();
        let enc_delta = enc_end - enc_start;

        if cfg.verify_with_encoder {
            let expected_enc_delta = motion.encoder_ticks_for_deg(signed_deg);
            let err_ticks = enc_delta - expected_enc_delta;
            let reached = err_ticks.abs() <= tol_ticks;

            if reached {
                warn!(
                    "RECOVERY_MOVE_{} verify OK: outcome={:?} enc_start={} enc_end={} enc_delta={} expected_delta={} err_ticks={} tol_ticks={}",
                    move_n,
                    outcome,
                    enc_start,
                    enc_end,
                    enc_delta,
                    expected_enc_delta,
                    err_ticks,
                    tol_ticks
                );
            } else {
                all_ok = false;
                error!(
                    "RECOVERY_MOVE_{} verify FAILED: outcome={:?} enc_start={} enc_end={} enc_delta={} expected_delta={} err_ticks={} tol_ticks={}",
                    move_n,
                    outcome,
                    enc_start,
                    enc_end,
                    enc_delta,
                    expected_enc_delta,
                    err_ticks,
                    tol_ticks
                );
                if cfg.stop_on_verify_fail {
                    // Safety: if we didn't reach the target (by encoder), don't continue to the next move.
                    break;
                } else {
                    warn!("RECOVERY_STOP_ON_VERIFY_FAIL=false: continuing to next recovery move despite verify failure");
                }
            }
        } else {
            warn!(
                "RECOVERY_MOVE_{} complete (no verify): outcome={:?} enc_start={} enc_end={} enc_delta={}",
                move_n,
                outcome,
                enc_start,
                enc_end,
                enc_delta
            );
        }
    }

    warn!(
        "RECOVERY_MOVES done: all_ok={} then {}",
        all_ok,
        if cfg.stop_after { "STOPPING" } else { "continuing" }
    );

    // Restore stall detection to previous state.
    if cfg.disable_stall_detection {
        motion.set_stall_detection_enabled(prev_stall);
    }

    if cfg.stop_after {
        warn!("STOP_AFTER_RECOVERY_MOVE=true: idling after recovery move (no homing/tracking)");
        loop {
            std::thread::sleep(Duration::from_secs(60));
        }
    }
}

