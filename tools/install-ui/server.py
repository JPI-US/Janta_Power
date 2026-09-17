"""
Installer UI for an Install-profile tower.

Two ways to authenticate to AWS IoT, chosen with INSTALL_UI_AUTH:

  iam  (default) SigV4 over WebSocket, using the same AWS credentials the IoT
       console uses. Authorization comes from your **IAM** policy, so this
       needs no change to the device certificate's IoT policy.
  cert Mutual TLS with the tower's device cert, on a different MQTT client id
       (`install-ui-{DEVICE_ID}`) so the tower keeps its own connection. This
       requires the cert's IoT policy to allow iot:Connect for that client id
       — without it AWS drops the TCP connection before CONNACK and you see a
       silent reconnect loop.

  pip install -r tools/install-ui/requirements.txt
  python tools/install-ui/server.py
  open http://127.0.0.1:8765

Enter the physical heading you see, pick the target (default 50° home),
press Move. The UI sends `move_by` with (target - current) on
`tower/{id}/cmd/diagnostics`.
"""

from __future__ import annotations

import datetime
import hashlib
import hmac
import json
import os
import ssl
import threading
import time
import urllib.parse
import uuid
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import urlparse

import paho.mqtt.client as mqtt

REPO_ROOT = Path(__file__).resolve().parents[2]
CERTS_DIR = REPO_ROOT / "certs"
STATIC_DIR = Path(__file__).resolve().parent
ENV_PATH = REPO_ROOT / ".env"

AWS_IOT_HOST = "a2exykcl6t998u-ats.iot.us-east-1.amazonaws.com"
AWS_REGION = "us-east-1"
# 8883 is mTLS; WebSocket/SigV4 rides 443.
AWS_IOT_PORT_TLS = 8883
AWS_IOT_PORT_WSS = 443
AUTH_MODE = os.environ.get("INSTALL_UI_AUTH", "iam").strip().lower()
LISTEN_HOST = "127.0.0.1"
LISTEN_PORT = 8765

# Ceiling on how long to hold the in-flight lock before assuming the ack is
# never coming. The worst legitimate case is a full-cap move: 200° at ~7 s/deg
# is ~23 min, so 45 min is comfortably clear of it while still unsticking a UI
# whose tower rebooted (`exit_install`) or dropped off mid-move.
PENDING_TIMEOUT_S = 45 * 60


def load_env(path: Path) -> dict[str, str]:
    values: dict[str, str] = {}
    if not path.exists():
        return values
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        values[key.strip()] = value.strip().strip('"').strip("'")
    return values


ENV = load_env(ENV_PATH)
DEVICE_ID = ENV.get("DEVICE_ID", "9001")
HOME_HEADING = float(ENV.get("HOME_HEADING_DEG", "50.0"))
CMD_TOPIC = f"tower/{DEVICE_ID}/cmd/diagnostics"
ACK_TOPIC = f"tower/{DEVICE_ID}/cmd/diagnostics/ack"
CLIENT_ID = f"install-ui-{DEVICE_ID}"

_lock = threading.Lock()
_events: deque[dict[str, Any]] = deque(maxlen=80)
_connected = False
_mqtt: mqtt.Client | None = None

# The command we are waiting on an ack for, or None. A move blocks the tower's
# main loop for ~7 s per degree, so a 110° jog is ~13 minutes of silence — the
# UI must refuse to queue a second one behind it.
_pending: dict[str, Any] | None = None
# Firmware's `install_max_step_deg`, learned from any ack that carries it
# (`install_ready` announces it at boot). Default matches switchboard.rs.
_max_step_deg = 200.0


def push_event(kind: str, **payload: Any) -> None:
    item = {"kind": kind, "ts": time.strftime("%H:%M:%S"), **payload}
    with _lock:
        prev = _events[0] if _events else None
        # Collapse consecutive identical messages. A refused connection retries
        # every 30 s, and 80 copies of the same line bury the one event that
        # actually explains what happened.
        if (
            prev is not None
            and "message" in item
            and prev.get("kind") == kind
            and prev.get("message") == item["message"]
        ):
            prev["ts"] = item["ts"]
            prev["repeat"] = prev.get("repeat", 1) + 1
            return
        _events.appendleft(item)


def clear_pending(reason: str) -> None:
    global _pending
    with _lock:
        if _pending is None:
            return
        cmd = _pending["cmd"]
        _pending = None
    push_event("info", message=f"Stopped waiting on '{cmd}' ({reason})")


def snapshot() -> dict[str, Any]:
    with _lock:
        stale = (
            _pending is not None
            and time.time() - _pending["sent_at"] > PENDING_TIMEOUT_S
        )
    if stale:
        clear_pending("no ack within 45 min")

    with _lock:
        events = list(_events)
        connected = _connected
        pending = dict(_pending) if _pending else None
        max_step = _max_step_deg
    if pending is not None:
        pending["elapsed"] = round(time.time() - pending["sent_at"], 1)
    return {
        "connected": connected,
        "device_id": DEVICE_ID,
        "home_heading": HOME_HEADING,
        "cmd_topic": CMD_TOPIC,
        "ack_topic": ACK_TOPIC,
        "client_id": CLIENT_ID,
        "auth_mode": AUTH_MODE,
        "max_step_deg": max_step,
        "pending": pending,
        "events": events,
    }


def publish_command(body: dict[str, Any]) -> dict[str, Any]:
    global _pending
    client = _mqtt
    if client is None:
        raise RuntimeError("MQTT client not started")
    with _lock:
        if not _connected:
            raise RuntimeError("Not connected to AWS IoT")
        # One command in flight at a time. The tower answers a move only once
        # the motor has stopped, and anything sent meanwhile just queues on the
        # device — so a double-click would turn into two consecutive jogs.
        if _pending is not None:
            raise RuntimeError(
                f"Still waiting on '{_pending['cmd']}' sent "
                f"{round(time.time() - _pending['sent_at'])}s ago"
            )
    if "request_id" not in body:
        body["request_id"] = uuid.uuid4().hex[:8]
    payload = json.dumps(body, separators=(",", ":"))
    info = client.publish(CMD_TOPIC, payload, qos=0, retain=False)
    if info.rc != mqtt.MQTT_ERR_SUCCESS:
        raise RuntimeError(f"Publish failed (rc={info.rc})")
    with _lock:
        _pending = {
            "request_id": body["request_id"],
            "cmd": body["cmd"],
            "sent_at": time.time(),
        }
    push_event("sent", topic=CMD_TOPIC, body=body)
    return body


def _reason(code: Any) -> str:
    if hasattr(code, "is_failure"):
        return f"{int(code)} {code}"
    return str(code)


def on_connect(client: mqtt.Client, _userdata: Any, _flags: Any, rc: Any, *_args: Any) -> None:
    global _connected
    failed = bool(getattr(rc, "is_failure", False)) or (isinstance(rc, int) and rc != 0)
    if not failed:
        with _lock:
            _connected = True
        client.subscribe(ACK_TOPIC, qos=0)
        push_event("info", message=f"Connected to AWS IoT as {CLIENT_ID}")
        push_event("info", message=f"Subscribed {ACK_TOPIC}")
        return
    with _lock:
        _connected = False
    push_event("error", message=f"AWS IoT CONNACK failed ({_reason(rc)})")


def on_disconnect(client: mqtt.Client, _userdata: Any, rc: Any, *_args: Any) -> None:
    global _connected
    with _lock:
        _connected = False
    push_event("error", message=f"Disconnected from AWS IoT ({_reason(rc)})")
    if AUTH_MODE == "iam":
        # paho's auto-reconnect replays the WebSocket path verbatim, and the
        # SigV4 signature baked into it has expired by then. Re-sign before the
        # retry, or every reconnect fails for a reason that looks like the
        # original failure.
        try:
            client.ws_set_options(path=presign_ws_path())
        except Exception as e:  # noqa: BLE001 - surfaced to the operator
            push_event("error", message=f"Could not re-sign WebSocket URL: {e}")


def on_message(_client: mqtt.Client, _userdata: Any, msg: mqtt.MQTTMessage) -> None:
    global _pending, _max_step_deg
    try:
        body = json.loads(msg.payload.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError):
        body = {"raw": msg.payload.decode("utf-8", errors="replace")}

    data = body.get("data") if isinstance(body.get("data"), dict) else {}
    with _lock:
        if isinstance(data.get("max_step_deg"), (int, float)):
            _max_step_deg = float(data["max_step_deg"])
        # Release the in-flight lock only for the command we are waiting on.
        # The tower also publishes unprompted (`install_ready`), which must not
        # be mistaken for an answer to a move.
        if _pending is not None and body.get("request_id") == _pending["request_id"]:
            _pending = None

    push_event("ack", topic=msg.topic, body=body)


def _hmac(key: bytes, msg: str) -> bytes:
    return hmac.new(key, msg.encode("utf-8"), hashlib.sha256).digest()


def presign_ws_path() -> str:
    """Build the SigV4-signed `/mqtt?...` path for an AWS IoT WebSocket.

    Authorization here is IAM, not the device certificate's IoT policy — which
    is the whole point of this mode. Uses whatever boto3 resolves: env vars,
    ~/.aws/credentials, SSO, or an instance role.

    The signature is short-lived, so this is regenerated for every connect
    attempt (see `on_disconnect`). Reusing a stale path is the classic cause of
    a WebSocket reconnect loop.
    """
    try:
        import boto3
    except ImportError as e:  # pragma: no cover - dependency guidance
        raise RuntimeError(
            "INSTALL_UI_AUTH=iam needs boto3 — "
            "pip install -r tools/install-ui/requirements.txt"
        ) from e

    session = boto3.Session()
    creds = session.get_credentials()
    if creds is None:
        raise RuntimeError(
            "No AWS credentials found. Run `aws configure`, or set "
            "AWS_ACCESS_KEY_ID / AWS_SECRET_ACCESS_KEY, or use INSTALL_UI_AUTH=cert."
        )
    creds = creds.get_frozen_credentials()
    region = session.region_name or AWS_REGION

    algorithm = "AWS4-HMAC-SHA256"
    service = "iotdevicegateway"
    canonical_uri = "/mqtt"

    now = datetime.datetime.now(datetime.timezone.utc)
    amz_date = now.strftime("%Y%m%dT%H%M%SZ")
    date_stamp = now.strftime("%Y%m%d")
    scope = f"{date_stamp}/{region}/{service}/aws4_request"

    # Query params must be in canonical (sorted) order for the signature.
    query = "&".join(
        [
            f"X-Amz-Algorithm={algorithm}",
            "X-Amz-Credential="
            + urllib.parse.quote(f"{creds.access_key}/{scope}", safe=""),
            f"X-Amz-Date={amz_date}",
            "X-Amz-SignedHeaders=host",
        ]
    )

    canonical_request = "\n".join(
        [
            "GET",
            canonical_uri,
            query,
            f"host:{AWS_IOT_HOST}\n",
            "host",
            hashlib.sha256(b"").hexdigest(),
        ]
    )
    string_to_sign = "\n".join(
        [
            algorithm,
            amz_date,
            scope,
            hashlib.sha256(canonical_request.encode("utf-8")).hexdigest(),
        ]
    )

    k_date = _hmac(f"AWS4{creds.secret_key}".encode("utf-8"), date_stamp)
    k_region = _hmac(k_date, region)
    k_service = _hmac(k_region, service)
    signing_key = _hmac(k_service, "aws4_request")
    signature = hmac.new(
        signing_key, string_to_sign.encode("utf-8"), hashlib.sha256
    ).hexdigest()

    query += f"&X-Amz-Signature={signature}"
    if creds.token:
        # IoT-specific: the session token is appended *after* signing rather
        # than folded into the canonical query string. If temporary credentials
        # fail to connect while long-lived keys work, this line is the suspect.
        query += "&X-Amz-Security-Token=" + urllib.parse.quote(creds.token, safe="")

    return f"{canonical_uri}?{query}"


def _new_client(transport: str) -> mqtt.Client:
    kwargs: dict[str, Any] = {
        "client_id": CLIENT_ID,
        "protocol": mqtt.MQTTv311,
        "clean_session": True,
        "transport": transport,
    }
    callback_api = getattr(mqtt, "CallbackAPIVersion", None)
    if callback_api is not None:
        # VERSION1 keeps on_connect(client, userdata, flags, rc).
        kwargs["callback_api_version"] = callback_api.VERSION1
    client = mqtt.Client(**kwargs)
    client.on_connect = on_connect
    client.on_disconnect = on_disconnect
    client.on_message = on_message
    client.reconnect_delay_set(min_delay=1, max_delay=30)
    return client


def start_mqtt() -> mqtt.Client:
    ca = CERTS_DIR / "AmazonRootCA1.pem"

    if AUTH_MODE == "iam":
        client = _new_client("websockets")
        client.ws_set_options(path=presign_ws_path())
        # Server verification only — there is no client certificate in this mode.
        client.tls_set(
            ca_certs=str(ca) if ca.exists() else None,
            tls_version=ssl.PROTOCOL_TLS_CLIENT,
        )
        client.connect_async(AWS_IOT_HOST, AWS_IOT_PORT_WSS, keepalive=60)
        client.loop_start()
        return client

    if AUTH_MODE != "cert":
        raise RuntimeError(f"INSTALL_UI_AUTH must be 'iam' or 'cert', got {AUTH_MODE!r}")

    cert = CERTS_DIR / f"tower_{DEVICE_ID}-certificate.pem.crt"
    key = CERTS_DIR / f"tower_{DEVICE_ID}-private.pem.key"
    missing = [p for p in (cert, key, ca) if not p.exists()]
    if missing:
        raise FileNotFoundError(
            "Missing TLS files: " + ", ".join(str(p) for p in missing)
        )

    client = _new_client("tcp")
    client.tls_set(
        ca_certs=str(ca),
        certfile=str(cert),
        keyfile=str(key),
        tls_version=ssl.PROTOCOL_TLS_CLIENT,
    )
    client.connect_async(AWS_IOT_HOST, AWS_IOT_PORT_TLS, keepalive=60)
    client.loop_start()
    return client


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt: str, *args: Any) -> None:
        return

    def _send(self, code: int, body: bytes, content_type: str) -> None:
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _send_json(self, code: int, payload: Any) -> None:
        data = json.dumps(payload).encode("utf-8")
        self._send(code, data, "application/json; charset=utf-8")

    def do_GET(self) -> None:
        path = urlparse(self.path).path
        if path == "/api/state":
            self._send_json(200, snapshot())
            return
        if path in ("/", "/index.html"):
            html = (STATIC_DIR / "index.html").read_bytes()
            self._send(200, html, "text/html; charset=utf-8")
            return
        self._send_json(404, {"error": "not found"})

    def do_POST(self) -> None:
        path = urlparse(self.path).path
        length = int(self.headers.get("Content-Length", "0"))
        raw = self.rfile.read(length) if length else b"{}"
        try:
            body = json.loads(raw.decode("utf-8") or "{}")
        except json.JSONDecodeError:
            self._send_json(400, {"error": "invalid JSON"})
            return

        try:
            if path == "/api/move":
                current = float(body["current"])
                target = float(body.get("target", HOME_HEADING))
                if not (current == current and target == target):  # NaN
                    raise ValueError("angles must be finite")
                degrees = target - current
                if degrees == 0:
                    raise ValueError("already at the target heading")
                cmd: dict[str, Any] = {
                    "cmd": "move_by",
                    "degrees": round(degrees, 3),
                }
                if body.get("ignore_encoder"):
                    cmd["ignore_encoder"] = True
                sent = publish_command(cmd)
                self._send_json(
                    200,
                    {
                        "ok": True,
                        "sent": sent,
                        "current": current,
                        "target": target,
                        "degrees": sent["degrees"],
                    },
                )
                return

            if path == "/api/command":
                cmd_name = body.get("cmd")
                if cmd_name not in ("get_status", "set_home_here", "exit_install"):
                    raise ValueError("unknown command")
                sent = publish_command({"cmd": cmd_name})
                # `exit_install` reboots the tower ~3 s after it acks. Hold the
                # lock and the operator would sit staring at a dead wait.
                if cmd_name == "exit_install":
                    clear_pending("tower is rebooting as Normal")
                self._send_json(200, {"ok": True, "sent": sent})
                return

            if path == "/api/clear":
                clear_pending("cleared by operator")
                self._send_json(200, {"ok": True})
                return
        except (KeyError, TypeError, ValueError) as e:
            self._send_json(400, {"error": str(e)})
            return
        except RuntimeError as e:
            self._send_json(503, {"error": str(e)})
            return

        self._send_json(404, {"error": "not found"})


def main() -> None:
    global _mqtt
    port = AWS_IOT_PORT_WSS if AUTH_MODE == "iam" else AWS_IOT_PORT_TLS
    how = "IAM SigV4 over WebSocket" if AUTH_MODE == "iam" else "device certificate"
    print(f"Repo        {REPO_ROOT}")
    print(f"Device      tower_{DEVICE_ID}   (from .env — restart me if you change it)")
    print(f"Topics      {CMD_TOPIC}")
    print(f"Home        {HOME_HEADING}°")
    print(f"Auth        {how}  [INSTALL_UI_AUTH={AUTH_MODE}]")
    print(f"MQTT        {AWS_IOT_HOST}:{port} as {CLIENT_ID}")
    print(f"UI          http://{LISTEN_HOST}:{LISTEN_PORT}")
    _mqtt = start_mqtt()
    server = ThreadingHTTPServer((LISTEN_HOST, LISTEN_PORT), Handler)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping")
    finally:
        server.server_close()
        if _mqtt is not None:
            _mqtt.loop_stop()
            _mqtt.disconnect()


if __name__ == "__main__":
    main()
