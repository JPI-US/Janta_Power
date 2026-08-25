use board_diagnostics::DiagnosticCommand;
use log::{error, info, warn};
use network::mqtt::Mqtt;
use network::telemetry::{publish_json, topic, DiagnosticsAck, DiagnosticsStatus, TIME_FORMAT};
use serde::{Deserialize, Serialize};
use std::collections::VecDeque;
use std::sync::{mpsc, Arc, Mutex, MutexGuard};
use std::thread;
use std::time::{Duration, Instant};

use crate::diagnostics::executor;

#[derive(Debug, Deserialize)]
struct DiagnosticsCommand {
    cmd: String,
    request_id: Option<String>,
    message: Option<String>,
}

pub struct StatusSnapshot<'a> {
    pub device_id: &'a str,
    pub firmware_version: &'a str,
    pub mqtt_connected: bool,
    pub wifi_connected: bool,
    pub motion_mode: &'a str,
    pub current_heading: f32,
    pub activity: &'a str,
    pub activity_reason: &'a str,
    pub sun_angle: Option<f64>,
    pub target_heading: Option<f64>,
    pub angle_offset: Option<f64>,
    pub last_move_outcome: Option<&'a str>,
}

#[derive(Clone, Debug)]
pub struct OwnedStatusSnapshot {
    pub device_id: String,
    pub firmware_version: String,
    pub mqtt_connected: bool,
    pub wifi_connected: bool,
    pub motion_mode: String,
    pub current_heading: f32,
    pub activity: String,
    pub activity_reason: String,
    pub sun_angle: Option<f64>,
    pub target_heading: Option<f64>,
    pub angle_offset: Option<f64>,
    pub last_move_outcome: Option<String>,
}

impl OwnedStatusSnapshot {
    fn as_borrowed(&self) -> StatusSnapshot<'_> {
        StatusSnapshot {
            device_id: &self.device_id,
            firmware_version: &self.firmware_version,
            mqtt_connected: self.mqtt_connected,
            wifi_connected: self.wifi_connected,
            motion_mode: &self.motion_mode,
            current_heading: self.current_heading,
            activity: &self.activity,
            activity_reason: &self.activity_reason,
            sun_angle: self.sun_angle,
            target_heading: self.target_heading,
            angle_offset: self.angle_offset,
            last_move_outcome: self.last_move_outcome.as_deref(),
        }
    }
}

pub type SharedStatusSnapshot = Arc<Mutex<OwnedStatusSnapshot>>;
pub type ControlCommandSender = mpsc::SyncSender<ControlCommand>;
pub type ControlCommandReceiver = mpsc::Receiver<ControlCommand>;
pub type SharedCommandState = Arc<Mutex<CommandState>>;

const MAX_RECENT_REQUEST_IDS: usize = 64;
const RATE_LIMIT_MAX_COMMANDS: usize = 12;
const RATE_LIMIT_WINDOW: Duration = Duration::from_secs(5);
const CONTROL_COMMAND_TIMEOUT: Duration = Duration::from_secs(30);

#[derive(Debug)]
pub enum ControlCommand {
    ExecuteFirstWave {
        request_id: String,
        cmd: String,
        command: DiagnosticCommand,
    },
}

#[derive(Debug)]
/// Public because `SharedCommandState` already exposes it through a `pub type`
/// alias. The fields stay private, so this widens nothing that callers could not
/// already name — it just stops newer rustc rejecting the alias as
/// private-in-public.
pub struct CommandState {
    recent_request_ids: VecDeque<String>,
    inflight_control: Option<InflightControl>,
    rate_limit_window_started_at: Option<Instant>,
    rate_limit_count: usize,
}

#[derive(Clone, Debug)]
struct InflightControl {
    request_id: String,
    cmd: String,
    queued_at: Instant,
}

#[derive(Debug, PartialEq, Eq)]
enum PreflightDecision {
    AllowReadOnly,
    ControlReserved,
    Duplicate,
    Busy,
    RateLimited,
}

#[derive(Debug, PartialEq, Eq)]
pub enum ControlCompletionDisposition {
    PublishFinalResult,
    SuppressLateResult,
}

pub fn new_shared_snapshot(initial: OwnedStatusSnapshot) -> SharedStatusSnapshot {
    Arc::new(Mutex::new(initial))
}

pub fn new_shared_command_state() -> SharedCommandState {
    Arc::new(Mutex::new(CommandState {
        recent_request_ids: VecDeque::with_capacity(MAX_RECENT_REQUEST_IDS),
        inflight_control: None,
        rate_limit_window_started_at: None,
        rate_limit_count: 0,
    }))
}

pub fn new_control_channel(bound: usize) -> (ControlCommandSender, ControlCommandReceiver) {
    // Use a bounded queue so backend bursts cannot grow memory without limit.
    mpsc::sync_channel(bound)
}

pub fn update_snapshot(shared_snapshot: &SharedStatusSnapshot, next: OwnedStatusSnapshot) {
    // Keep the critical section tiny: diagnostics only needs the latest complete
    // snapshot, not an incremental stream of field updates.
    let mut snapshot = lock_snapshot(shared_snapshot);
    *snapshot = next;
}

/// Stack for the diagnostics listener thread. See `MQTT_EVENT_THREAD_STACK`.
const DIAGNOSTICS_THREAD_STACK: usize = 8192;

pub fn spawn_listener(
    broker_url: &str,
    client_id: &str,
    device_id: &str,
    shared_snapshot: SharedStatusSnapshot,
    command_state: SharedCommandState,
    control_tx: ControlCommandSender,
) -> anyhow::Result<()> {
    // Phase 1 uses a dedicated MQTT connection for diagnostics so inbound
    // commands are no longer gated by the main tracking loop sleep interval.
    let mqtt = Mqtt::new_mqtt(broker_url, client_id)?;
    let device_id = device_id.to_string();
    let client_id = client_id.to_string();

    // Explicit stack size, for the same reason as the MQTT event thread: ESP-IDF's
    // 3 KB pthread default does not cover JSON serialisation and transcript
    // formatting in Rust.
    thread::Builder::new()
        .stack_size(DIAGNOSTICS_THREAD_STACK)
        .spawn(move || {
            diagnostics_loop(
                mqtt,
                device_id,
                client_id,
                shared_snapshot,
                command_state,
                control_tx,
            )
        })?;
    Ok(())
}

pub fn complete_control_command(
    shared_command_state: &SharedCommandState,
    request_id: &str,
) -> ControlCompletionDisposition {
    let mut state = lock_command_state(shared_command_state);
    match state.inflight_control.as_ref() {
        Some(inflight) if inflight.request_id == request_id => {
            state.inflight_control = None;
            state.record_recent_request_id(request_id);
            ControlCompletionDisposition::PublishFinalResult
        }
        _ if !request_id.is_empty() && state.recent_request_ids.iter().any(|id| id == request_id) => {
            ControlCompletionDisposition::SuppressLateResult
        }
        _ => {
            state.record_recent_request_id(request_id);
            ControlCompletionDisposition::PublishFinalResult
        }
    }
}

pub fn subscribe(mqtt: &mut Mqtt, device_id: &str) -> anyhow::Result<()> {
    let command_topic = topic::diagnostics_cmd(device_id);
    mqtt.subscribe(&command_topic)?;
    info!("Subscribed to diagnostics commands: {}", command_topic);
    Ok(())
}

pub fn process_one(
    mqtt: &mut Mqtt,
    device_id: &str,
    status_snapshot: &StatusSnapshot<'_>,
    shared_command_state: &SharedCommandState,
    control_tx: &ControlCommandSender,
) -> anyhow::Result<bool> {
    let Some((incoming_topic, payload)) = mqtt.try_receive() else {
        return Ok(false);
    };

    let command_topic = topic::diagnostics_cmd(device_id);
    if incoming_topic != command_topic {
        warn!(
            "Ignoring MQTT message on unexpected topic: {} (expected {})",
            incoming_topic, command_topic
        );
        return Ok(false);
    }

    let payload_str = match std::str::from_utf8(&payload) {
        Ok(value) => value,
        Err(e) => {
            error!("Diagnostics command payload is not UTF-8: {:?}", e);
            publish_ack(mqtt, device_id, "", "unknown", "error", "Command payload must be UTF-8")?;
            return Ok(true);
        }
    };

    let command: DiagnosticsCommand = match serde_json::from_str(payload_str) {
        Ok(value) => value,
        Err(e) => {
            error!("Failed to parse diagnostics command JSON: {:?}", e);
            publish_ack(mqtt, device_id, "", "unknown", "error", "Command payload must be valid JSON")?;
            return Ok(true);
        }
    };

    let request_id = command.request_id.as_deref().unwrap_or("");
    let message = command.message.as_deref().unwrap_or("");
    let control_command = executor::mqtt_control_command(&command.cmd);
    let direct_lines = executor::direct_first_wave_lines(&command.cmd, status_snapshot.firmware_version);
    let needs_control = control_command.is_some();
    info!(
        "Diagnostics command received: cmd={} request_id={} message={}",
        command.cmd, request_id, message
    );

    match preflight_command(
        shared_command_state,
        Instant::now(),
        request_id,
        &command.cmd,
        needs_control,
    ) {
        PreflightDecision::Duplicate => {
            publish_ack(
                mqtt,
                device_id,
                request_id,
                &command.cmd,
                "duplicate",
                "Duplicate request_id ignored for this boot",
            )?;
            return Ok(true);
        }
        PreflightDecision::Busy => {
            publish_ack(
                mqtt,
                device_id,
                request_id,
                &command.cmd,
                "busy",
                "A diagnostics control command is already in progress",
            )?;
            return Ok(true);
        }
        PreflightDecision::RateLimited => {
            publish_ack(
                mqtt,
                device_id,
                request_id,
                &command.cmd,
                "rate_limited",
                "Diagnostics command rate limit exceeded; try again shortly",
            )?;
            return Ok(true);
        }
        PreflightDecision::AllowReadOnly | PreflightDecision::ControlReserved => {}
    }

    match command.cmd.as_str() {
        "ping" => publish_ack(
            mqtt,
            device_id,
            request_id,
            &command.cmd,
            "received",
            "Tower received diagnostics command",
        )
        .map(|_| mark_read_only_request_complete(shared_command_state, request_id))?,
        "get_status" => publish_status(mqtt, device_id, request_id, &command.cmd, status_snapshot)
            .map(|_| mark_read_only_request_complete(shared_command_state, request_id))?,
        _ if direct_lines.is_some() => publish_transcript(
            mqtt,
            device_id,
            request_id,
            &command.cmd,
            &executor::DiagnosticsTranscript {
                status: "completed",
                message: format!("{} reported successfully", &command.cmd),
                lines: direct_lines.unwrap_or_default(),
                reboot_requested: false,
                details: Vec::new(),
            },
        )
        .map(|_| mark_read_only_request_complete(shared_command_state, request_id))?,
        _ if control_command.is_some() => queue_control_command(
            mqtt,
            device_id,
            request_id,
            &command.cmd,
            shared_command_state,
            control_tx,
            control_command.unwrap(),
        )?,
        other => {
            warn!("Unsupported diagnostics command: {}", other);
            publish_ack(
                mqtt,
                device_id,
                request_id,
                other,
                "error",
                "Unsupported diagnostics command",
            )
            .map(|_| mark_read_only_request_complete(shared_command_state, request_id))?;
        }
    }

    Ok(true)
}

pub fn publish_ack_message(
    mqtt: &mut Mqtt,
    device_id: &str,
    request_id: &str,
    cmd: &str,
    status: &str,
    message: &str,
) -> anyhow::Result<()> {
    publish_ack(mqtt, device_id, request_id, cmd, status, message)
}

fn publish_ack(
    mqtt: &mut Mqtt,
    device_id: &str,
    request_id: &str,
    cmd: &str,
    status: &str,
    message: &str,
) -> anyhow::Result<()> {
    let current_time = rtc::timezone::local_time()
        .format(TIME_FORMAT)
        .to_string();
    let payload = DiagnosticsAck {
        current_time: &current_time,
        request_id,
        cmd,
        status,
        message,
    };
    let ack_topic = topic::diagnostics_ack(device_id);
    publish_json(mqtt, &ack_topic, &payload)
}

#[derive(Serialize)]
struct DiagnosticsTranscriptPayload<'a> {
    current_time: &'a str,
    request_id: &'a str,
    cmd: &'a str,
    status: &'a str,
    message: &'a str,
    lines: &'a [String],
}

fn diagnostics_loop(
    mut mqtt: Mqtt,
    device_id: String,
    client_id: String,
    shared_snapshot: SharedStatusSnapshot,
    shared_command_state: SharedCommandState,
    control_tx: ControlCommandSender,
) {
    let mut subscribed = false;

    loop {
        if !mqtt.is_connected() {
            if subscribed {
                warn!(
                    "Diagnostics MQTT listener disconnected; waiting to re-subscribe (client_id={})",
                    client_id
                );
            }
            subscribed = false;
            thread::sleep(Duration::from_millis(250));
            continue;
        }

        if !subscribed {
            match subscribe(&mut mqtt, &device_id) {
                Ok(()) => {
                    subscribed = true;
                    info!(
                        "Diagnostics MQTT listener ready (client_id={}, topic={})",
                        client_id,
                        topic::diagnostics_cmd(&device_id)
                    );
                }
                Err(e) => {
                    warn!(
                        "Diagnostics MQTT subscribe failed (client_id={}): {:?}",
                        client_id, e
                    );
                    thread::sleep(Duration::from_secs(1));
                    continue;
                }
            }
        }

        if let Err(e) = publish_timed_out_control_if_needed(
            &mut mqtt,
            &device_id,
            &shared_command_state,
        ) {
            warn!(
                "Diagnostics control timeout handling failed (client_id={}): {:?}",
                client_id, e
            );
        }

        // Clone the latest status under the lock, then release it before any
        // MQTT parsing/publish work. That keeps the shared snapshot from ever
        // being held across network activity.
        let snapshot = {
            let snapshot = lock_snapshot(&shared_snapshot);
            snapshot.clone()
        };
        let snapshot = snapshot.as_borrowed();

        // Drain a small batch per wake so diagnostics stays responsive without
        // starving the rest of the system if the backend sends a burst.
        let mut processed = 0;
        while processed < 8 {
            match process_one(
                &mut mqtt,
                &device_id,
                &snapshot,
                &shared_command_state,
                &control_tx,
            ) {
                Ok(true) => {
                    processed += 1;
                }
                Ok(false) => {
                    break;
                }
                Err(e) => {
                    warn!(
                        "Diagnostics command processing failed (client_id={}): {:?}",
                        client_id, e
                    );
                    break;
                }
            }
        }

        if processed == 0 {
            thread::sleep(Duration::from_millis(100));
        }
    }
}

fn queue_control_command(
    mqtt: &mut Mqtt,
    device_id: &str,
    request_id: &str,
    cmd: &str,
    shared_command_state: &SharedCommandState,
    control_tx: &ControlCommandSender,
    command: DiagnosticCommand,
) -> anyhow::Result<()> {
    match control_tx.try_send(ControlCommand::ExecuteFirstWave {
        request_id: request_id.to_string(),
        cmd: cmd.to_string(),
        command,
    }) {
        // A queued acknowledgement means the command has been handed off safely
        // to the main control loop, which will publish the final completion/failure result.
        Ok(()) => publish_ack(
            mqtt,
            device_id,
            request_id,
            cmd,
            "queued",
            "Diagnostics command accepted by the main control loop",
        ),
        Err(mpsc::TrySendError::Full(_)) => {
            release_reserved_control_command(shared_command_state, request_id, cmd);
            publish_ack(
                mqtt,
                device_id,
                request_id,
                cmd,
                "busy",
                "Diagnostics control queue is full; try again shortly",
            )
        }
        Err(mpsc::TrySendError::Disconnected(_)) => {
            release_reserved_control_command(shared_command_state, request_id, cmd);
            publish_ack(
                mqtt,
                device_id,
                request_id,
                cmd,
                "error",
                "Main control loop is unavailable",
            )
        }
    }
}

fn preflight_command(
    shared_command_state: &SharedCommandState,
    now: Instant,
    request_id: &str,
    cmd: &str,
    needs_control: bool,
) -> PreflightDecision {
    let mut state = lock_command_state(shared_command_state);
    state.advance_rate_window(now);

    if state.rate_limit_count >= RATE_LIMIT_MAX_COMMANDS {
        return PreflightDecision::RateLimited;
    }
    state.rate_limit_count += 1;

    if !request_id.is_empty() {
        if state.recent_request_ids.iter().any(|id| id == request_id) {
            return PreflightDecision::Duplicate;
        }
    }

    if let Some(inflight) = state.inflight_control.as_ref() {
        if !request_id.is_empty() && inflight.request_id == request_id {
            return PreflightDecision::Duplicate;
        }
        if needs_control {
            return PreflightDecision::Busy;
        }
    }

    if needs_control {
        state.inflight_control = Some(InflightControl {
            request_id: request_id.to_string(),
            cmd: cmd.to_string(),
            queued_at: now,
        });
        PreflightDecision::ControlReserved
    } else {
        PreflightDecision::AllowReadOnly
    }
}

fn mark_read_only_request_complete(shared_command_state: &SharedCommandState, request_id: &str) {
    let mut state = lock_command_state(shared_command_state);
    state.record_recent_request_id(request_id);
}

fn publish_timed_out_control_if_needed(
    mqtt: &mut Mqtt,
    device_id: &str,
    shared_command_state: &SharedCommandState,
) -> anyhow::Result<()> {
    let timed_out = {
        let mut state = lock_command_state(shared_command_state);
        state.take_timed_out_control(Instant::now())
    };

    if let Some(timed_out) = timed_out {
        warn!(
            "Diagnostics control command timed out: cmd={} request_id={}",
            timed_out.cmd,
            timed_out.request_id
        );
        publish_ack(
            mqtt,
            device_id,
            &timed_out.request_id,
            &timed_out.cmd,
            "timeout",
            "Diagnostics control command timed out waiting for main loop completion",
        )?;
    }

    Ok(())
}

fn lock_snapshot<'a>(shared_snapshot: &'a SharedStatusSnapshot) -> MutexGuard<'a, OwnedStatusSnapshot> {
    match shared_snapshot.lock() {
        Ok(guard) => guard,
        Err(poisoned) => {
            // If another thread panicked while holding the snapshot, keep the
            // device alive and continue serving diagnostics with the last state.
            warn!("Diagnostics status snapshot lock was poisoned; recovering");
            poisoned.into_inner()
        }
    }
}

fn lock_command_state<'a>(shared_command_state: &'a SharedCommandState) -> MutexGuard<'a, CommandState> {
    match shared_command_state.lock() {
        Ok(guard) => guard,
        Err(poisoned) => {
            warn!("Diagnostics command state lock was poisoned; recovering");
            poisoned.into_inner()
        }
    }
}

fn publish_status(
    mqtt: &mut Mqtt,
    device_id: &str,
    request_id: &str,
    cmd: &str,
    snapshot: &StatusSnapshot<'_>,
) -> anyhow::Result<()> {
    let current_time = rtc::timezone::local_time()
        .format(TIME_FORMAT)
        .to_string();
    let payload = DiagnosticsStatus {
        current_time: &current_time,
        request_id,
        cmd,
        status: "ok",
        message: "Tower status snapshot",
        activity: snapshot.activity,
        activity_reason: snapshot.activity_reason,
        device_id: snapshot.device_id,
        firmware_version: snapshot.firmware_version,
        mqtt_connected: snapshot.mqtt_connected,
        wifi_connected: snapshot.wifi_connected,
        motion_mode: snapshot.motion_mode,
        current_heading: snapshot.current_heading,
        sun_angle: snapshot.sun_angle,
        target_heading: snapshot.target_heading,
        angle_offset: snapshot.angle_offset,
        last_move_outcome: snapshot.last_move_outcome,
    };
    let ack_topic = topic::diagnostics_ack(device_id);
    publish_json(mqtt, &ack_topic, &payload)
}

pub fn publish_transcript(
    mqtt: &mut Mqtt,
    device_id: &str,
    request_id: &str,
    cmd: &str,
    transcript: &executor::DiagnosticsTranscript,
) -> anyhow::Result<()> {
    let current_time = rtc::timezone::local_time()
        .format(TIME_FORMAT)
        .to_string();
    let payload = DiagnosticsTranscriptPayload {
        current_time: &current_time,
        request_id,
        cmd,
        status: transcript.status,
        message: &transcript.message,
        lines: &transcript.lines,
    };
    let ack_topic = topic::diagnostics_ack(device_id);
    publish_json(mqtt, &ack_topic, &payload)
}

impl CommandState {
    fn advance_rate_window(&mut self, now: Instant) {
        match self.rate_limit_window_started_at {
            Some(started_at) if now.saturating_duration_since(started_at) < RATE_LIMIT_WINDOW => {}
            _ => {
                self.rate_limit_window_started_at = Some(now);
                self.rate_limit_count = 0;
            }
        }
    }

    fn record_recent_request_id(&mut self, request_id: &str) {
        if request_id.is_empty() {
            return;
        }
        if self.recent_request_ids.iter().any(|id| id == request_id) {
            return;
        }
        if self.recent_request_ids.len() >= MAX_RECENT_REQUEST_IDS {
            self.recent_request_ids.pop_front();
        }
        self.recent_request_ids.push_back(request_id.to_string());
    }

    fn take_timed_out_control(&mut self, now: Instant) -> Option<InflightControl> {
        let inflight = self.inflight_control.as_ref()?;
        if now.saturating_duration_since(inflight.queued_at) < CONTROL_COMMAND_TIMEOUT {
            return None;
        }

        let timed_out = self.inflight_control.take()?;
        self.record_recent_request_id(&timed_out.request_id);
        Some(timed_out)
    }
}

pub fn reserve_local_control_command(
    shared_command_state: &SharedCommandState,
    cmd: &str,
) -> Result<(), String> {
    let mut state = lock_command_state(shared_command_state);
    if state.inflight_control.is_some() {
        return Err(String::from(
            "ERROR BUSY another diagnostics control command is already in progress",
        ));
    }

    state.inflight_control = Some(InflightControl {
        request_id: String::new(),
        cmd: cmd.to_string(),
        queued_at: Instant::now(),
    });
    Ok(())
}

fn release_reserved_control_command(
    shared_command_state: &SharedCommandState,
    request_id: &str,
    cmd: &str,
) {
    let mut state = lock_command_state(shared_command_state);
    match state.inflight_control.as_ref() {
        Some(inflight) if inflight.request_id == request_id && inflight.cmd == cmd => {
            state.inflight_control = None;
        }
        _ => {}
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn test_command_state() -> SharedCommandState {
        new_shared_command_state()
    }

    #[test]
    fn malformed_json_is_rejected() {
        let parsed = serde_json::from_str::<DiagnosticsCommand>(r#"{"cmd":"ping""#);
        assert!(parsed.is_err());
    }

    #[test]
    fn duplicate_request_id_is_rejected_after_completion() {
        let state = test_command_state();
        mark_read_only_request_complete(&state, "dup-1");

        let decision = preflight_command(&state, Instant::now(), "dup-1", "ping", false);
        assert_eq!(decision, PreflightDecision::Duplicate);
    }

    #[test]
    fn second_control_command_is_busy_while_one_is_inflight() {
        let state = test_command_state();
        let now = Instant::now();
        {
            let mut guard = lock_command_state(&state);
            guard.inflight_control = Some(InflightControl {
                request_id: String::from("req-1"),
                cmd: String::from("request_rehome"),
                queued_at: now,
            });
        }

        let decision =
            preflight_command(&state, now + Duration::from_millis(10), "req-2", "rtc_check", true);
        assert_eq!(decision, PreflightDecision::Busy);
    }

    #[test]
    fn timed_out_control_command_is_released() {
        let mut state = CommandState {
            recent_request_ids: VecDeque::new(),
            inflight_control: Some(InflightControl {
                request_id: String::from("req-timeout"),
                cmd: String::from("request_rehome"),
                queued_at: Instant::now(),
            }),
            rate_limit_window_started_at: None,
            rate_limit_count: 0,
        };

        let timed_out = state.take_timed_out_control(Instant::now() + CONTROL_COMMAND_TIMEOUT + Duration::from_secs(1));

        assert!(timed_out.is_some());
        assert!(state.inflight_control.is_none());
        assert!(state.recent_request_ids.iter().any(|id| id == "req-timeout"));
    }

    #[test]
    fn rate_limit_rejects_burst() {
        let state = test_command_state();
        let now = Instant::now();

        for idx in 0..RATE_LIMIT_MAX_COMMANDS {
            let request_id = format!("req-{idx}");
            let decision = preflight_command(&state, now, &request_id, "ping", false);
            assert_eq!(decision, PreflightDecision::AllowReadOnly);
            mark_read_only_request_complete(&state, &request_id);
        }

        let decision = preflight_command(&state, now, "req-over", "ping", false);
        assert_eq!(decision, PreflightDecision::RateLimited);
    }

    #[test]
    fn zero_capacity_queue_reports_full() {
        let (tx, _rx) = mpsc::sync_channel::<ControlCommand>(0);
        let result = tx.try_send(ControlCommand::ExecuteFirstWave {
            request_id: String::from("req-full"),
            cmd: String::from("request_rehome"),
            command: DiagnosticCommand::GoHome,
        });

        assert!(matches!(result, Err(mpsc::TrySendError::Full(_))));
    }

    #[test]
    fn late_completion_is_suppressed_after_timeout() {
        let state = test_command_state();
        {
            let mut guard = lock_command_state(&state);
            guard.inflight_control = Some(InflightControl {
                request_id: String::from("req-late"),
                cmd: String::from("request_rehome"),
                queued_at: Instant::now() - CONTROL_COMMAND_TIMEOUT - Duration::from_secs(1),
            });
        }
        {
            let mut guard = lock_command_state(&state);
            let timed_out = guard.take_timed_out_control(Instant::now());
            assert!(timed_out.is_some());
        }

        let disposition = complete_control_command(&state, "req-late");
        assert_eq!(disposition, ControlCompletionDisposition::SuppressLateResult);
    }
}
