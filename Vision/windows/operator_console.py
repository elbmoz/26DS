"""Local operator console coordinating device lifecycle and live monitoring.

The HTTP server binds to loopback only.  Human controls and Codex use the
same JSON endpoints, so neither side depends on desktop UI automation.
"""

from __future__ import annotations

import argparse
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import subprocess
import sys
import threading
import time
from urllib.error import HTTPError, URLError
from urllib.parse import urlsplit
from urllib.request import Request, urlopen
import webbrowser


WINDOWS_DIR = Path(__file__).resolve().parent
ASSET_DIR = WINDOWS_DIR / "console_ui"
MONITOR_URL = "http://127.0.0.1:8765"
MAX_BODY_BYTES = 64 * 1024


def _http_json(url, path="/status", body=None, timeout=1.0):
    payload = None
    method = "GET"
    headers = {}
    if body is not None:
        payload = json.dumps(body, ensure_ascii=False).encode("utf-8")
        method = "POST"
        headers["Content-Type"] = "application/json; charset=utf-8"
    request = Request(
        url.rstrip("/") + path,
        data=payload,
        headers=headers,
        method=method,
    )
    with urlopen(request, timeout=float(timeout)) as response:
        return json.loads(response.read().decode("utf-8"))


class ConsoleController:
    def __init__(self, device_ip):
        self.device_ip = str(device_ip)
        self.monitor_process = None
        self.monitor_recording = False
        self._lock = threading.RLock()
        self._logs = deque(maxlen=300)
        self._operation = {
            "state": "idle",
            "name": None,
            "message": "",
        }
        self._last_frame = None
        self._last_analysis = None
        self._load_latest_recorded_analysis()
        self._device = None
        self._device_updated = 0.0
        self._closed = threading.Event()
        self._refresh_thread = threading.Thread(
            target=self._refresh_loop,
            name="vision-console-device-status",
            daemon=True,
        )
        self._refresh_thread.start()

    def log(self, message):
        message = str(message).rstrip()
        if not message:
            return
        timestamp = time.strftime("%H:%M:%S")
        with self._lock:
            for line in message.splitlines():
                self._logs.append("{}  {}".format(timestamp, line))

    def _agent_command(
        self, *arguments, timeout=45.0, log_output=True
    ):
        command = [
            sys.executable,
            str(WINDOWS_DIR / "vision_agent.py"),
            "--device-ip",
            self.device_ip,
            *[str(value) for value in arguments],
        ]
        completed = subprocess.run(
            command,
            cwd=str(WINDOWS_DIR.parents[1]),
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            timeout=float(timeout),
        )
        if completed.stdout and log_output:
            self.log(completed.stdout)
        if completed.stderr and log_output:
            self.log(completed.stderr)
        try:
            result = json.loads(completed.stdout or completed.stderr)
        except json.JSONDecodeError:
            result = {
                "ok": completed.returncode == 0,
                "stdout": completed.stdout,
                "stderr": completed.stderr,
            }
        if completed.returncode != 0:
            raise RuntimeError(
                result.get("error")
                or "vision_agent exited {}".format(completed.returncode)
            )
        return result

    def _monitor_status(self):
        try:
            return _http_json(MONITOR_URL, timeout=0.35)
        except (OSError, HTTPError, URLError, ValueError):
            return None

    def _monitor_output_reader(self, process):
        try:
            for line in process.stdout:
                self.log(line)
        finally:
            return_code = process.poll()
            self.log(
                "monitor process exited{}".format(
                    "" if return_code is None else " ({})".format(return_code)
                )
            )

    def _launch_monitor(self, record):
        current = self._monitor_status()
        if current is not None:
            current_recording = bool(current.get("recording"))
            if current_recording == bool(record):
                self.monitor_recording = current_recording
                return current
            self._stop_monitor()

        command = [
            sys.executable,
            str(WINDOWS_DIR / "stream_receiver.py"),
            "--device-ip",
            self.device_ip,
            "--headless",
        ]
        if not record:
            command.append("--no-record")
        process = subprocess.Popen(
            command,
            cwd=str(WINDOWS_DIR.parents[1]),
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            encoding="utf-8",
            errors="replace",
            bufsize=1,
        )
        with self._lock:
            self.monitor_process = process
            self.monitor_recording = bool(record)
        threading.Thread(
            target=self._monitor_output_reader,
            args=(process,),
            name="vision-console-monitor-output",
            daemon=True,
        ).start()

        deadline = time.monotonic() + 20.0
        status = None
        while time.monotonic() < deadline:
            if process.poll() is not None:
                raise RuntimeError(
                    "monitor exited during startup ({})".format(
                        process.returncode
                    )
                )
            status = self._monitor_status()
            if status is not None:
                return status
            time.sleep(0.20)
        raise RuntimeError("monitor API did not start within 20 seconds")

    def _stop_monitor(self):
        status = self._monitor_status()
        if status is not None:
            try:
                _http_json(MONITOR_URL, "/stop", {}, timeout=1.5)
            except (OSError, HTTPError, URLError, ValueError):
                pass
        with self._lock:
            process = self.monitor_process
        if process is not None:
            try:
                process.wait(timeout=12.0)
            except subprocess.TimeoutExpired:
                process.terminate()
                try:
                    process.wait(timeout=3.0)
                except subprocess.TimeoutExpired:
                    process.kill()
            with self._lock:
                if self.monitor_process is process:
                    self.monitor_process = None
        self._capture_last_analysis()
        self.monitor_recording = False

    def _capture_last_analysis(self):
        status_path = WINDOWS_DIR.parent / "runtime" / "live_status.json"
        try:
            status = json.loads(status_path.read_text(encoding="utf-8"))
            if not status.get("video"):
                return
            report_path = status.get("analysis_report")
            if not report_path:
                return
            analysis_path = Path(report_path).parent / "analysis.json"
            snapshot = self._analysis_snapshot(
                analysis_path, Path(report_path)
            )
        except (OSError, ValueError, json.JSONDecodeError):
            return
        with self._lock:
            self._last_analysis = snapshot

    def _analysis_snapshot(self, analysis_path, report_path=None):
        analysis_path = Path(analysis_path)
        analysis = json.loads(analysis_path.read_text(encoding="utf-8"))
        metrics = analysis.get("metrics", {})
        return {
            "session_directory": str(analysis_path.parent),
            "analysis_report": (
                None if report_path is None else str(report_path)
            ),
            "analysis_json": str(analysis_path),
            "measured_ratio": metrics.get("measured_ratio"),
            "valid_ratio": metrics.get("valid_ratio"),
            "detector_fps_mean": (
                metrics.get("reported_detector_fps") or {}
            ).get("mean"),
            "video_latency_p50_ms": (
                metrics.get("video_pipeline_latency_ms") or {}
            ).get("p50"),
            "evaluation": metrics.get("evaluation"),
        }

    def _load_latest_recorded_analysis(self):
        sessions_root = (
            WINDOWS_DIR.parent / "captures" / "stream_sessions"
        )
        if not sessions_root.is_dir():
            return
        for session in sorted(
            sessions_root.iterdir(),
            key=lambda path: path.name,
            reverse=True,
        ):
            if not session.is_dir():
                continue
            analysis_path = session / "analysis.json"
            if not analysis_path.is_file():
                continue
            if not (
                (session / "video.mp4").is_file()
                or (session / "video.mkv").is_file()
            ):
                continue
            try:
                self._last_analysis = self._analysis_snapshot(
                    analysis_path, session / "analysis.txt"
                )
            except (OSError, ValueError, json.JSONDecodeError):
                continue
            return

    def _run_operation(self, name, function):
        with self._lock:
            if self._operation["state"] == "running":
                raise RuntimeError(
                    "operation already running: {}".format(
                        self._operation["name"]
                    )
                )
            self._operation = {
                "state": "running",
                "name": str(name),
                "message": "",
                "started_epoch_ns": time.time_ns(),
            }

        def worker():
            try:
                result = function()
                with self._lock:
                    self._operation = {
                        "state": "completed",
                        "name": str(name),
                        "message": "完成",
                        "result": result,
                        "finished_epoch_ns": time.time_ns(),
                    }
            except Exception as exc:
                self.log("{} failed: {}".format(name, exc))
                with self._lock:
                    self._operation = {
                        "state": "failed",
                        "name": str(name),
                        "message": str(exc),
                        "finished_epoch_ns": time.time_ns(),
                    }

        threading.Thread(
            target=worker,
            name="vision-console-{}".format(name),
            daemon=True,
        ).start()
        return {"ok": True, "accepted": True, "operation": name}

    def action(self, action, body):
        action = str(action)
        body = dict(body or {})
        if action == "preview":
            return self._run_operation(
                action, lambda: self._launch_monitor(False)
            )
        if action == "record":
            return self._run_operation(
                action, lambda: self._launch_monitor(True)
            )
        if action == "stop_record":
            def stop_and_preview():
                self._stop_monitor()
                return self._launch_monitor(False)

            return self._run_operation(action, stop_and_preview)
        if action == "stop_monitor":
            return self._run_operation(action, self._stop_monitor)
        if action == "deploy_restart":
            def deploy_restart():
                self._stop_monitor()
                result = self._agent_command(
                    "restart", "--deploy", timeout=70
                )
                self._device_updated = 0.0
                self._launch_monitor(False)
                return result

            return self._run_operation(action, deploy_restart)
        if action == "device_start":
            def start_device():
                result = self._agent_command("start")
                self._device_updated = 0.0
                self._launch_monitor(False)
                return result

            return self._run_operation(action, start_device)
        if action == "device_stop":
            def stop_device():
                self._stop_monitor()
                result = self._agent_command("stop")
                self._device_updated = 0.0
                return result

            return self._run_operation(action, stop_device)
        if action == "rollback":
            release_id = str(body.get("release_id", "")).strip()
            if not release_id:
                raise ValueError("release_id is required")

            def rollback():
                self._stop_monitor()
                self._agent_command("stop")
                result = self._agent_command(
                    "rollback", release_id, "--start"
                )
                self._device_updated = 0.0
                self._launch_monitor(False)
                return result

            return self._run_operation(action, rollback)
        if action == "snapshot":
            return _http_json(MONITOR_URL, "/snapshot", {}, timeout=2.0)
        if action == "mark":
            label = str(body.get("label", "")).strip()
            if not label:
                raise ValueError("marker label is required")
            return _http_json(
                MONITOR_URL, "/mark", {"label": label[:120]}, timeout=2.0
            )
        if action == "config":
            params = body.get("params")
            if not isinstance(params, dict) or not params:
                raise ValueError("params must be a non-empty object")
            return _http_json(
                MONITOR_URL, "/config", {"params": params}, timeout=2.0
            )
        if action == "sync":
            offset_ms = float(body.get("offset_ms", 0.0))
            return _http_json(
                MONITOR_URL,
                "/sync",
                {"offset_ms": offset_ms},
                timeout=2.0,
            )
        raise ValueError("unknown action: {}".format(action))

    def _refresh_loop(self):
        while not self._closed.wait(0.5):
            if time.monotonic() - self._device_updated < 3.0:
                continue
            try:
                result = self._agent_command(
                    "status", timeout=12.0, log_output=False
                )
            except Exception as exc:
                result = {
                    "ok": False,
                    "host": self.device_ip,
                    "error": str(exc),
                    "running": False,
                }
            with self._lock:
                self._device = result
                self._device_updated = time.monotonic()

    def state(self):
        monitor = self._monitor_status()
        with self._lock:
            device = dict(self._device or {})
            operation = dict(self._operation)
            logs = list(self._logs)[-90:]
            last_analysis = (
                None
                if self._last_analysis is None
                else dict(self._last_analysis)
            )
        device_log = str(device.pop("log_tail", "") or "")
        if device_log:
            logs.extend(
                ["— MaixCAM —"]
                + device_log.splitlines()[-30:]
            )
            logs = logs[-90:]
        return {
            "ok": True,
            "schema": 1,
            "device_ip": self.device_ip,
            "device": device,
            "monitor": monitor,
            "operation": operation,
            "last_analysis": last_analysis,
            "logs": logs,
            "agent_api": "http://127.0.0.1:8770/api",
            "iteration_api": MONITOR_URL,
            "updated_epoch_ns": time.time_ns(),
        }

    def frame(self):
        try:
            request = Request(
                MONITOR_URL + "/frame.jpg",
                headers={"Cache-Control": "no-cache"},
            )
            with urlopen(request, timeout=1.0) as response:
                frame = response.read()
            with self._lock:
                self._last_frame = frame
            return frame
        except (OSError, HTTPError, URLError):
            with self._lock:
                return self._last_frame

    def start_default_preview(self):
        if self._monitor_status() is not None:
            return
        try:
            self._run_operation(
                "preview", lambda: self._launch_monitor(False)
            )
        except RuntimeError:
            pass

    def close(self):
        self._closed.set()
        try:
            self._stop_monitor()
        except Exception:
            pass


class ConsoleServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True


class ConsoleHandler(BaseHTTPRequestHandler):
    server_version = "PipeBallConsole/1"

    @property
    def controller(self):
        return self.server.controller

    def _send(self, status, payload, content_type):
        payload = bytes(payload)
        self.send_response(int(status))
        self.send_header("Content-Type", str(content_type))
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(payload)

    def _json(self, status, value):
        self._send(
            status,
            json.dumps(
                value, ensure_ascii=False, separators=(",", ":")
            ).encode("utf-8"),
            "application/json; charset=utf-8",
        )

    def _read_json(self):
        length = int(self.headers.get("Content-Length", "0"))
        if length < 0 or length > MAX_BODY_BYTES:
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
        if path in ("/", "/index.html"):
            return self._asset("index.html", "text/html; charset=utf-8")
        if path == "/style.css":
            return self._asset("style.css", "text/css; charset=utf-8")
        if path == "/app.js":
            return self._asset(
                "app.js", "application/javascript; charset=utf-8"
            )
        if path == "/favicon.ico":
            self._send(204, b"", "image/x-icon")
            return
        if path in ("/api", "/api/state"):
            self._json(200, self.controller.state())
            return
        if path == "/api/frame.jpg":
            frame = self.controller.frame()
            if frame is None:
                self._json(503, {"ok": False, "error": "frame unavailable"})
            else:
                self._send(200, frame, "image/jpeg")
            return
        self._json(404, {"ok": False, "error": "not found"})

    def _asset(self, name, content_type):
        path = ASSET_DIR / name
        if not path.is_file():
            self._json(404, {"ok": False, "error": "asset missing"})
            return
        self._send(200, path.read_bytes(), content_type)

    def do_POST(self):
        path = urlsplit(self.path).path
        if path != "/api/action":
            self._json(404, {"ok": False, "error": "not found"})
            return
        try:
            body = self._read_json()
            result = self.controller.action(body.get("action"), body)
        except (ValueError, RuntimeError, OSError, HTTPError, URLError) as exc:
            self._json(
                409,
                {
                    "ok": False,
                    "error": str(exc),
                    "error_type": type(exc).__name__,
                },
            )
            return
        self._json(202, result)

    def log_message(self, _format, *_args):
        return


def build_parser():
    parser = argparse.ArgumentParser(
        description="滚球视觉本地一体化上位机"
    )
    parser.add_argument(
        "--device-ip",
        default=os.environ.get("MAIXCAM_IP", "10.16.6.1"),
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8770)
    parser.add_argument("--no-browser", action="store_true")
    parser.add_argument("--no-auto-preview", action="store_true")
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    if args.host not in ("127.0.0.1", "localhost", "::1"):
        raise ValueError("operator console must bind to loopback")
    controller = ConsoleController(args.device_ip)
    server = ConsoleServer((args.host, args.port), ConsoleHandler)
    server.controller = controller
    url = "http://{}:{}/".format(args.host, server.server_address[1])
    print("operator console:", url)
    print("agent API:", url + "api/state")
    if not args.no_auto_preview:
        controller.start_default_preview()
    if not args.no_browser:
        threading.Timer(0.6, lambda: webbrowser.open(url)).start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        server.server_close()
        controller.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
