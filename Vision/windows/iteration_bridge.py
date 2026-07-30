"""Local machine-readable bridge for live vision iteration.

The monitor remains the owner of the camera session.  This module exposes only
localhost HTTP commands and an atomically updated JSON status file so another
tool (including Codex) can inspect, mark, tune, snapshot, or stop an experiment
without clicking through the GUI.
"""

import json
import os
from pathlib import Path
import queue
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import threading
import time
import uuid
from urllib.parse import urlsplit


MAX_REQUEST_BYTES = 64 * 1024
STATUS_FILE_INTERVAL_S = 0.50


def _write_json_atomic(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(value, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    os.replace(temporary, path)


class _BridgeServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True


class _BridgeHandler(BaseHTTPRequestHandler):
    server_version = "PipeBallIteration/1"

    @property
    def bridge(self):
        return self.server.bridge

    def _send_json(self, status_code, value):
        payload = json.dumps(
            value, ensure_ascii=False, separators=(",", ":")
        ).encode("utf-8")
        self.send_response(int(status_code))
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(payload)

    def _send_bytes(self, status_code, payload, content_type):
        payload = bytes(payload)
        self.send_response(int(status_code))
        self.send_header("Content-Type", str(content_type))
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
        self.send_header("Pragma", "no-cache")
        self.end_headers()
        self.wfile.write(payload)

    def _read_json(self):
        try:
            length = int(self.headers.get("Content-Length", "0"))
        except ValueError:
            raise ValueError("invalid Content-Length")
        if length < 0 or length > MAX_REQUEST_BYTES:
            raise ValueError("request body is too large")
        raw = self.rfile.read(length)
        if not raw:
            return {}
        value = json.loads(raw.decode("utf-8"))
        if not isinstance(value, dict):
            raise ValueError("JSON body must be an object")
        return value

    def do_GET(self):
        path = urlsplit(self.path).path
        if path in ("/", "/status"):
            self._send_json(200, self.bridge.status())
            return
        if path == "/telemetry":
            self._stream_telemetry()
            return
        if path == "/frame.jpg":
            frame = self.bridge.frame()
            if frame is None:
                self._send_json(
                    503, {"ok": False, "error": "frame not available"}
                )
            else:
                self._send_bytes(200, frame, "image/jpeg")
            return
        self._send_json(404, {"ok": False, "error": "unknown endpoint"})

    def _stream_telemetry(self):
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("X-Accel-Buffering", "no")
        self.end_headers()
        sequence = 0
        try:
            while not self.bridge.closed():
                next_sequence, sample = self.bridge.wait_for_telemetry(
                    sequence, timeout=10.0
                )
                if sample is None:
                    self.wfile.write(b": keepalive\n\n")
                else:
                    payload = json.dumps(
                        sample,
                        ensure_ascii=False,
                        separators=(",", ":"),
                    ).encode("utf-8")
                    self.wfile.write(
                        b"id: "
                        + str(next_sequence).encode("ascii")
                        + b"\ndata: "
                        + payload
                        + b"\n\n"
                    )
                    sequence = next_sequence
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, OSError):
            return

    def do_POST(self):
        path = urlsplit(self.path).path
        command_types = {
            "/config": "set_config",
            "/mark": "mark",
            "/snapshot": "snapshot",
            "/stop": "stop",
            "/sync": "set_sync_offset",
        }
        command_type = command_types.get(path)
        if command_type is None:
            self._send_json(404, {"ok": False, "error": "unknown endpoint"})
            return
        try:
            body = self._read_json()
            command = self.bridge.enqueue(command_type, body)
        except (ValueError, json.JSONDecodeError) as exc:
            self._send_json(400, {"ok": False, "error": str(exc)})
            return
        self._send_json(
            202,
            {
                "ok": True,
                "accepted": True,
                "command_id": command["id"],
                "type": command["type"],
            },
        )

    def log_message(self, _format, *_args):
        return


class IterationBridge:
    def __init__(
        self,
        session_dir,
        runtime_dir,
        host="127.0.0.1",
        port=8765,
    ):
        self.session_dir = Path(session_dir)
        self.runtime_dir = Path(runtime_dir)
        self.host = str(host)
        self.requested_port = int(port)
        self.port = None
        self._status = {
            "schema": 1,
            "state": "starting",
            "session_directory": str(self.session_dir),
        }
        self._lock = threading.Lock()
        self._commands = queue.Queue()
        self._stop_requested = threading.Event()
        self._server = None
        self._thread = None
        self._last_status_file_write = 0.0
        self._frame_jpeg = None
        self._frame_sequence = 0
        self._telemetry_condition = threading.Condition()
        self._telemetry_sample = None
        self._telemetry_sequence = 0
        self._closed = threading.Event()
        self.live_status_path = self.runtime_dir / "live_status.json"
        self.latest_session_path = self.runtime_dir / "latest_session.json"

    def start(self):
        self.runtime_dir.mkdir(parents=True, exist_ok=True)
        self._server = _BridgeServer(
            (self.host, self.requested_port),
            _BridgeHandler,
        )
        self._server.bridge = self
        self.port = int(self._server.server_address[1])
        self._thread = threading.Thread(
            target=self._server.serve_forever,
            name="vision-iteration-http",
            daemon=True,
        )
        self._thread.start()
        self.publish(
            {
                "api_url": "http://{}:{}".format(self.host, self.port),
                "status_file": str(self.live_status_path),
            },
            force=True,
        )
        _write_json_atomic(
            self.latest_session_path,
            {
                "schema": 1,
                "session_directory": str(self.session_dir),
                "status_file": str(self.live_status_path),
                "api_url": "http://{}:{}".format(self.host, self.port),
            },
        )
        return self.port

    def status(self):
        with self._lock:
            return dict(self._status)

    def frame(self):
        with self._lock:
            if self._frame_jpeg is None:
                return None
            return bytes(self._frame_jpeg)

    def publish_frame(self, jpeg_bytes):
        payload = bytes(jpeg_bytes)
        if not payload:
            raise ValueError("JPEG frame is empty")
        with self._lock:
            self._frame_jpeg = payload
            self._frame_sequence += 1
            self._status["preview_frame_sequence"] = self._frame_sequence
            self._status["preview_frame_epoch_ns"] = time.time_ns()
        return self._frame_sequence

    def publish_telemetry(self, sample):
        value = dict(sample or {})
        if not value:
            return self._telemetry_sequence
        with self._telemetry_condition:
            self._telemetry_sequence += 1
            self._telemetry_sample = value
            sequence = self._telemetry_sequence
            self._telemetry_condition.notify_all()
        return sequence

    def wait_for_telemetry(self, after_sequence, timeout=10.0):
        deadline = time.monotonic() + max(0.0, float(timeout))
        with self._telemetry_condition:
            while (
                self._telemetry_sequence <= int(after_sequence)
                and not self._closed.is_set()
            ):
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return self._telemetry_sequence, None
                self._telemetry_condition.wait(remaining)
            if self._telemetry_sequence <= int(after_sequence):
                return self._telemetry_sequence, None
            return self._telemetry_sequence, dict(self._telemetry_sample)

    def closed(self):
        return self._closed.is_set()

    def publish(self, values, force=False):
        now_monotonic = time.monotonic()
        with self._lock:
            self._status.update(dict(values))
            self._status["updated_epoch_ns"] = time.time_ns()
            snapshot = dict(self._status)
            write_status_file = (
                bool(force)
                or now_monotonic - self._last_status_file_write
                >= STATUS_FILE_INTERVAL_S
            )
            if write_status_file:
                self._last_status_file_write = now_monotonic
        if write_status_file:
            _write_json_atomic(self.live_status_path, snapshot)
        return snapshot

    def enqueue(self, command_type, body=None):
        command = {
            "id": uuid.uuid4().hex,
            "type": str(command_type),
            "body": dict(body or {}),
            "received_epoch_ns": time.time_ns(),
        }
        if command["type"] == "stop":
            self._stop_requested.set()
        self._commands.put(command)
        return command

    def stop_requested(self):
        return self._stop_requested.is_set()

    def poll_commands(self, limit=20):
        commands = []
        for _ in range(max(0, int(limit))):
            try:
                commands.append(self._commands.get_nowait())
            except queue.Empty:
                break
        return commands

    def stop(self, final_status=None):
        if final_status:
            self.publish(final_status, force=True)
        self._closed.set()
        with self._telemetry_condition:
            self._telemetry_condition.notify_all()
        if self._server is not None:
            self._server.shutdown()
            self._server.server_close()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        self._server = None
        self._thread = None
