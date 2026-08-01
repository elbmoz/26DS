"""Core Windows-side transport, logging, and FFmpeg recording utilities."""

from collections import deque
import csv
import json
import os
from pathlib import Path
import re
import shutil
import socket
import subprocess
import sys
import threading
import time
from urllib.parse import urlsplit, urlunsplit


MAIXCAM_DIR = Path(__file__).resolve().parents[1] / "maixcam"
if str(MAIXCAM_DIR) not in sys.path:
    sys.path.insert(0, str(MAIXCAM_DIR))

from stream_protocol import (  # noqa: E402
    ProtocolError,
    decode_packet,
    encode_packet,
    make_set_config_request,
    make_pid_request,
    make_subscribe_request,
)


TRACKING_FIELDS = (
    "host_epoch_ns",
    "host_monotonic_ns",
    "source_ip",
    "session",
    "seq",
    "device_ms",
    "frame_id",
    "loop_dt_ms",
    "fps",
    "detect_ms",
    "measured",
    "valid",
    "coasting",
    "track_x",
    "track_y",
    "measurement_x",
    "measurement_y",
    "measurement_radius",
    "radius",
    "position",
    "position_px",
    "travel_position_px",
    "travel_length_px",
    "target_axis_px",
    "error_px",
    "lateral_px",
    "velocity_px_s",
    "quality",
    "position_rejects",
    "lateral_rejects",
    "fixture_rejects",
    "quality_rejects",
    "jump_rejects",
    "hits",
    "misses",
    "raw_blob_count",
    "circle_count",
    "candidate_count",
    "candidates",
    "algorithm",
    "ai_boxes",
    "local_search",
    "fell_back",
    "axis_x0",
    "axis_y0",
    "axis_x1",
    "axis_y1",
    "roi_x",
    "roi_y",
    "roi_w",
    "roi_h",
    "roi_quad",
    "pipe_measured",
    "pipe_valid",
    "pipe_age_frames",
    "pipe_blob_count",
    "pipe_length",
    "pipe_width",
    "pipe_score",
)

STM32_FEEDBACK_FIELDS = (
    "host_epoch_ns",
    "host_monotonic_ns",
    "source_ip",
    "session",
    "transport_seq",
    "device_ms",
    "seq",
    "seq_gap",
    "mcu_ms",
    "vision_frame",
    "vision_age_ms",
    "position_x10",
    "velocity_x10",
    "error_x10",
    "p_x100",
    "i_x100",
    "d_x100",
    "position_px",
    "velocity_px_s",
    "control_error_px",
    "p_term",
    "i_term",
    "d_term",
    "motor_command",
    "motor_status",
    "motor_status_name",
    "feedback_version",
    "target_rod_angle_deg",
    "actual_rod_angle_deg",
    "rod_rate_deg_s",
    "angle_error_deg",
    "desired_motor_speed",
    "position_age_ms",
    "position_valid",
    "protection_state",
    "tuning_mode",
    "tuning_sequence",
    "tuning_phase",
    "phase_elapsed_ms",
    "raw_line",
)

VIDEO_FRAME_FIELDS = (
    "preview_frame_id",
    "host_epoch_ns",
    "host_monotonic_ns",
    "video_source_epoch_ns",
    "video_pipeline_latency_ms",
    "telemetry_match_delta_ms",
    "sync_offset_ms",
    "decoded_frame_seq",
    "dropped_frames",
    "telemetry_session",
    "telemetry_seq",
    "telemetry_device_ms",
    "tracking_frame_id",
)


def unique_session_directory(root):
    root = Path(root)
    root.mkdir(parents=True, exist_ok=True)
    stem = time.strftime("stream_%Y%m%d_%H%M%S")
    candidate = root / stem
    suffix = 2
    while candidate.exists():
        candidate = root / "{}_{:02d}".format(stem, suffix)
        suffix += 1
    candidate.mkdir()
    return candidate


def normalize_rtsp_url(url, device_ip):
    if not url:
        return "rtsp://{}:8554/live".format(device_ip)
    parsed = urlsplit(url)
    hostname = parsed.hostname
    if hostname not in (None, "", "0.0.0.0", "127.0.0.1", "localhost"):
        return url

    port = parsed.port or 8554
    netloc = "{}:{}".format(device_ip, port)
    return urlunsplit(
        (
            parsed.scheme or "rtsp",
            netloc,
            parsed.path or "/live",
            parsed.query,
            parsed.fragment,
        )
    )


def parse_parameter_assignments(assignments):
    params = {}
    for assignment in assignments or ():
        if "=" not in assignment:
            raise ValueError(
                "parameter must use name=value: {}".format(assignment)
            )
        name, raw_value = assignment.split("=", 1)
        name = name.strip()
        raw_value = raw_value.strip()
        if not name:
            raise ValueError("parameter name cannot be empty")
        try:
            value = float(raw_value)
        except ValueError:
            raise ValueError(
                "parameter value must be numeric: {}".format(assignment)
            )
        if value.is_integer():
            value = int(value)
        params[name] = value
    return params


class SessionLogger:
    def __init__(self, session_dir):
        self.session_dir = Path(session_dir)
        self.started_epoch_ns = time.time_ns()
        self.started_monotonic_ns = time.monotonic_ns()
        self.packet_count = 0
        self.tracking_count = 0
        self.feedback_count = 0
        self.video_frame_count = 0
        self._lock = threading.Lock()

        self._raw = (self.session_dir / "telemetry.jsonl").open(
            "w", encoding="utf-8", newline="\n"
        )
        self._events = (self.session_dir / "events.jsonl").open(
            "w", encoding="utf-8", newline="\n"
        )
        self._tracking_file = (self.session_dir / "tracking.csv").open(
            "w", encoding="utf-8", newline=""
        )
        self._tracking = csv.DictWriter(
            self._tracking_file,
            fieldnames=TRACKING_FIELDS,
            extrasaction="ignore",
        )
        self._tracking.writeheader()
        self._feedback_file = (
            self.session_dir / "stm32_feedback.csv"
        ).open("w", encoding="utf-8", newline="")
        self._feedback = csv.DictWriter(
            self._feedback_file,
            fieldnames=STM32_FEEDBACK_FIELDS,
            extrasaction="ignore",
        )
        self._feedback.writeheader()
        self._video_file = (self.session_dir / "video_frames.csv").open(
            "w", encoding="utf-8", newline=""
        )
        self._video = csv.DictWriter(
            self._video_file,
            fieldnames=VIDEO_FRAME_FIELDS,
            extrasaction="ignore",
        )
        self._video.writeheader()

    def log_packet(
        self,
        packet,
        source_ip,
        host_epoch_ns,
        host_monotonic_ns,
    ):
        enriched = dict(packet)
        enriched["host_epoch_ns"] = int(host_epoch_ns)
        enriched["host_monotonic_ns"] = int(host_monotonic_ns)
        enriched["source_ip"] = str(source_ip)
        with self._lock:
            self._raw.write(
                json.dumps(enriched, ensure_ascii=False, separators=(",", ":"))
                + "\n"
            )
            self.packet_count += 1
            if packet.get("type") == "tracking":
                self._tracking.writerow(enriched)
                self.tracking_count += 1
            elif packet.get("type") == "stm32_feedback":
                self._feedback.writerow(enriched)
                self.feedback_count += 1
            if self.packet_count % 30 == 0:
                self._raw.flush()
                self._tracking_file.flush()
                self._feedback_file.flush()

    def log_video_frame(
        self,
        frame_id,
        latest_tracking,
        frame_info=None,
        match_info=None,
    ):
        frame_info = frame_info or {}
        match_info = match_info or {}
        now_epoch_ns = int(
            frame_info.get("read_epoch_ns") or time.time_ns()
        )
        now_monotonic_ns = int(
            frame_info.get("read_monotonic_ns") or time.monotonic_ns()
        )
        latest_tracking = latest_tracking or {}
        row = {
            "preview_frame_id": int(frame_id),
            "host_epoch_ns": now_epoch_ns,
            "host_monotonic_ns": now_monotonic_ns,
            "video_source_epoch_ns": frame_info.get(
                "source_epoch_ns", ""
            ),
            "video_pipeline_latency_ms": frame_info.get(
                "pipeline_latency_ms", ""
            ),
            "telemetry_match_delta_ms": match_info.get(
                "match_delta_ms", ""
            ),
            "sync_offset_ms": match_info.get("sync_offset_ms", ""),
            "decoded_frame_seq": frame_info.get("decode_seq", ""),
            "dropped_frames": frame_info.get("dropped_frames", 0),
            "telemetry_session": latest_tracking.get("session", ""),
            "telemetry_seq": latest_tracking.get("seq", ""),
            "telemetry_device_ms": latest_tracking.get("device_ms", ""),
            "tracking_frame_id": latest_tracking.get("frame_id", ""),
        }
        with self._lock:
            self._video.writerow(row)
            self.video_frame_count += 1
            if self.video_frame_count % 30 == 0:
                self._video_file.flush()
        return now_epoch_ns, now_monotonic_ns

    def log_event(self, event_type, details):
        entry = {
            "host_epoch_ns": time.time_ns(),
            "host_monotonic_ns": time.monotonic_ns(),
            "type": str(event_type),
            "details": details,
        }
        with self._lock:
            self._events.write(
                json.dumps(entry, ensure_ascii=False, separators=(",", ":"))
                + "\n"
            )
            self._events.flush()

    def write_manifest(self, manifest):
        final = dict(manifest)
        final.update(
            {
                "started_epoch_ns": self.started_epoch_ns,
                "started_monotonic_ns": self.started_monotonic_ns,
                "ended_epoch_ns": time.time_ns(),
                "ended_monotonic_ns": time.monotonic_ns(),
                "packet_count": self.packet_count,
                "tracking_count": self.tracking_count,
                "feedback_count": self.feedback_count,
                "video_frame_count": self.video_frame_count,
            }
        )
        path = self.session_dir / "session.json"
        temporary = path.with_suffix(".json.tmp")
        temporary.write_text(
            json.dumps(final, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        os.replace(temporary, path)

    def close(self):
        with self._lock:
            for handle in (
                self._raw,
                self._events,
                self._tracking_file,
                self._feedback_file,
                self._video_file,
            ):
                try:
                    handle.flush()
                    handle.close()
                except Exception:
                    pass


class TelemetryReceiver:
    def __init__(self, port, logger, bind_host="0.0.0.0"):
        self.port = int(port)
        self.logger = logger
        self.bind_host = bind_host
        self.invalid_packets = 0
        self.latest_tracking = None
        self.latest_feedback = None
        self.latest_status = None
        self.latest_ack = None
        self.device_ip = None
        self.control_port = None
        self._tracking_history = deque(maxlen=900)
        self._tracking_session = None
        self._tracking_listeners = []
        self._feedback_listeners = []
        self._acks = {}
        self._condition = threading.Condition()
        self._stop = threading.Event()
        self._thread = None

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.socket.bind((self.bind_host, self.port))
        self.socket.settimeout(0.25)

    def start(self):
        self._thread = threading.Thread(
            target=self._run,
            name="vision-telemetry",
            daemon=True,
        )
        self._thread.start()

    def add_tracking_listener(self, listener):
        if not callable(listener):
            raise TypeError("tracking listener must be callable")
        with self._condition:
            self._tracking_listeners.append(listener)
        return listener

    def add_feedback_listener(self, listener):
        if not callable(listener):
            raise TypeError("feedback listener must be callable")
        with self._condition:
            self._feedback_listeners.append(listener)
        return listener

    def _run(self):
        while not self._stop.is_set():
            try:
                data, address = self.socket.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                break

            host_epoch_ns = time.time_ns()
            host_monotonic_ns = time.monotonic_ns()
            try:
                packet = decode_packet(data)
            except ProtocolError as exc:
                self.invalid_packets += 1
                self.logger.log_event(
                    "invalid_packet",
                    {"source": address[0], "error": str(exc)},
                )
                continue

            packet["_host_epoch_ns"] = host_epoch_ns
            packet["_host_monotonic_ns"] = host_monotonic_ns
            packet["_source_ip"] = address[0]
            self.logger.log_packet(
                packet,
                address[0],
                host_epoch_ns,
                host_monotonic_ns,
            )

            tracking_listeners = ()
            feedback_listeners = ()
            with self._condition:
                packet_type = packet.get("type")
                if packet_type == "tracking":
                    packet_session = packet.get("session")
                    if (
                        self._tracking_session is not None
                        and packet_session != self._tracking_session
                    ):
                        self._tracking_history.clear()
                    self._tracking_session = packet_session
                    self.latest_tracking = packet
                    self._tracking_history.append(packet)
                    tracking_listeners = tuple(
                        self._tracking_listeners
                    )
                elif packet_type == "status":
                    self.latest_status = packet
                    self.device_ip = address[0]
                    self.control_port = int(
                        packet.get("control_port", 42102)
                    )
                elif packet_type == "stm32_feedback":
                    self.latest_feedback = packet
                    feedback_listeners = tuple(
                        self._feedback_listeners
                    )
                elif packet_type in (
                    "config_ack",
                    "subscribe_ack",
                    "pid_ack",
                ):
                    self.latest_ack = packet
                    request_id = packet.get("request_id")
                    if request_id:
                        self._acks[request_id] = packet
                self._condition.notify_all()
            for listener in tracking_listeners:
                try:
                    listener(dict(packet))
                except Exception as exc:
                    self.logger.log_event(
                        "tracking_listener_error",
                        {"error": str(exc)},
                    )
            for listener in feedback_listeners:
                try:
                    listener(dict(packet))
                except Exception as exc:
                    self.logger.log_event(
                        "feedback_listener_error",
                        {"error": str(exc)},
                    )

    def wait_for_status(self, timeout):
        deadline = time.monotonic() + float(timeout)
        with self._condition:
            while self.latest_status is None:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self._condition.wait(remaining)
            return dict(self.latest_status)

    def snapshot(self):
        with self._condition:
            tracking = (
                None
                if self.latest_tracking is None
                else dict(self.latest_tracking)
            )
            status = (
                None if self.latest_status is None else dict(self.latest_status)
            )
            ack = None if self.latest_ack is None else dict(self.latest_ack)
        return tracking, status, ack

    def status_snapshot(self):
        with self._condition:
            return (
                None
                if self.latest_status is None
                else dict(self.latest_status)
            )

    def feedback_snapshot(self):
        with self._condition:
            return (
                None
                if self.latest_feedback is None
                else dict(self.latest_feedback)
            )

    def tracking_for_epoch(
        self,
        video_source_epoch_ns,
        sync_offset_ms=0.0,
        max_delta_ms=250.0,
    ):
        """Return telemetry nearest to a video frame's source timestamp.

        RTSP frames and UDP tracking packets travel through independent
        pipelines.  FFmpeg exposes the wall-clock arrival timestamp attached
        to each decoded frame; matching against the UDP receive history avoids
        drawing the newest tracking point on an older buffered video frame.
        """
        if video_source_epoch_ns is None:
            with self._condition:
                tracking = (
                    None
                    if self.latest_tracking is None
                    else dict(self.latest_tracking)
                )
            return tracking, {
                "matched": tracking is not None,
                "match_delta_ms": None,
                "sync_offset_ms": float(sync_offset_ms),
                "mode": "latest_fallback",
            }

        target_ns = int(
            int(video_source_epoch_ns)
            + float(sync_offset_ms) * 1_000_000.0
        )
        with self._condition:
            matched = None
            best_delta_ns = None
            for item in reversed(self._tracking_history):
                item_epoch_ns = int(item["_host_epoch_ns"])
                delta_ns = item_epoch_ns - target_ns
                absolute_delta_ns = abs(delta_ns)
                if (
                    best_delta_ns is None
                    or absolute_delta_ns < best_delta_ns
                ):
                    matched = item
                    best_delta_ns = absolute_delta_ns
                if item_epoch_ns <= target_ns:
                    break
            if matched is not None:
                matched = dict(matched)
        if matched is None:
            return None, {
                "matched": False,
                "match_delta_ms": None,
                "sync_offset_ms": float(sync_offset_ms),
                "mode": "timestamp",
            }

        delta_ms = (
            int(matched["_host_epoch_ns"]) - target_ns
        ) / 1_000_000.0
        if abs(delta_ms) > float(max_delta_ms):
            return None, {
                "matched": False,
                "match_delta_ms": delta_ms,
                "sync_offset_ms": float(sync_offset_ms),
                "mode": "timestamp",
            }
        return dict(matched), {
            "matched": True,
            "match_delta_ms": delta_ms,
            "sync_offset_ms": float(sync_offset_ms),
            "mode": "timestamp",
        }

    def send_parameters(self, token, params):
        if not self.device_ip or not self.control_port:
            raise RuntimeError("device status not discovered yet")
        request_id = "pc-{}".format(time.time_ns())
        packet = make_set_config_request(request_id, token, params)
        self.socket.sendto(
            encode_packet(packet),
            (self.device_ip, self.control_port),
        )
        self.logger.log_event(
            "config_request",
            {
                "request_id": request_id,
                "device_ip": self.device_ip,
                "params": params,
            },
        )
        return request_id

    def send_pid_request(self, token, action, params=None):
        if not self.device_ip or not self.control_port:
            raise RuntimeError("device status not discovered yet")
        request_id = "pid-{}".format(time.time_ns())
        packet = make_pid_request(request_id, token, action, params)
        self.socket.sendto(
            encode_packet(packet),
            (self.device_ip, self.control_port),
        )
        self.logger.log_event(
            "pid_request",
            {
                "request_id": request_id,
                "device_ip": self.device_ip,
                "action": action,
                "params": dict(params or {}),
            },
        )
        return request_id

    def subscribe(self, device_ip, control_port, token):
        request_id = "sub-{}".format(time.time_ns())
        packet = make_subscribe_request(request_id, token, self.port)
        self.socket.sendto(
            encode_packet(packet),
            (str(device_ip), int(control_port)),
        )
        self.logger.log_event(
            "subscribe_request",
            {
                "request_id": request_id,
                "device_ip": str(device_ip),
                "control_port": int(control_port),
                "telemetry_port": self.port,
            },
        )
        return request_id

    def wait_for_ack(self, request_id, timeout=2.0):
        deadline = time.monotonic() + float(timeout)
        with self._condition:
            while request_id not in self._acks:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self._condition.wait(remaining)
            return dict(self._acks.pop(request_id))

    def pop_ack(self, request_id):
        with self._condition:
            packet = self._acks.pop(request_id, None)
            return None if packet is None else dict(packet)

    def stop(self):
        self._stop.set()
        try:
            self.socket.close()
        except OSError:
            pass
        if self._thread is not None:
            self._thread.join(timeout=1.0)


class FfmpegPipeline:
    """Low-latency RTSP decode feeding recording and a latest-frame preview.

    A background reader continuously drains raw frames and keeps only the
    newest one.  This is important for a control monitor: a slow GUI callback
    or a parameter ACK must never turn into seconds of permanent video
    backlog.
    """

    _PTS_TIME_PATTERN = re.compile(
        r"\bpts_time:\s*"
        r"([+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?)"
    )

    def __init__(
        self,
        url,
        width,
        height,
        fps,
        session_dir,
        record=True,
        transport="tcp",
        remux_mp4=True,
    ):
        self.url = str(url)
        self.width = int(width)
        self.height = int(height)
        self.fps = max(1, int(fps))
        self.session_dir = Path(session_dir)
        self.record = bool(record)
        self.transport = str(transport)
        self.remux_mp4 = bool(remux_mp4)
        self.frame_bytes = self.width * self.height * 3
        self.ffmpeg = shutil.which("ffmpeg")
        if not self.ffmpeg:
            raise RuntimeError("ffmpeg was not found on PATH")
        self.process = None
        self.stderr_handle = None
        self._stderr_thread = None
        self._frame_thread = None
        self._timing_condition = threading.Condition()
        self._frame_condition = threading.Condition()
        self._timing_queue = deque()
        self._latest_frame = None
        self._latest_frame_info = None
        self._decode_seq = -1
        self._last_delivered_seq = -1
        self._reader_error = None
        self.decoded_frames = 0
        self.dropped_preview_frames = 0
        self.timing_frames = 0
        self.started_epoch_ns = None
        self.started_monotonic_ns = None
        self.mkv_path = self.session_dir / "video.mkv"
        self.mp4_path = self.session_dir / "video.mp4"

    def command(self):
        preview_filter = "showinfo@vision_sync,setpts=N/({}*TB)".format(
            self.fps
        )
        command = [
            self.ffmpeg,
            "-hide_banner",
            "-loglevel",
            "info",
            "-y",
            "-rtsp_transport",
            self.transport,
            "-use_wallclock_as_timestamps",
            "1",
            "-fflags",
            "+nobuffer+discardcorrupt",
            "-flags",
            "low_delay",
            "-analyzeduration",
            "0",
            "-probesize",
            "32768",
            "-max_delay",
            "0",
            "-reorder_queue_size",
            "0",
            "-copyts",
            "-i",
            self.url,
        ]
        if self.record:
            command.extend(
                [
                    "-map",
                    "0:v:0",
                    "-an",
                    "-c:v",
                    "copy",
                    "-avoid_negative_ts",
                    "make_zero",
                    str(self.mkv_path),
                ]
            )
        command.extend(
            [
                "-map",
                "0:v:0",
                "-an",
                "-vf",
                preview_filter,
                "-c:v",
                "rawvideo",
                "-pix_fmt",
                "bgr24",
                "-fps_mode",
                "passthrough",
                "-f",
                "rawvideo",
                "pipe:1",
            ]
        )
        return command

    def start(self):
        self.stderr_handle = (self.session_dir / "ffmpeg.log").open(
            "wb"
        )
        self.started_epoch_ns = time.time_ns()
        self.started_monotonic_ns = time.monotonic_ns()
        self.process = subprocess.Popen(
            self.command(),
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            bufsize=0,
        )
        self._stderr_thread = threading.Thread(
            target=self._read_stderr,
            name="ffmpeg-stderr",
            daemon=True,
        )
        self._frame_thread = threading.Thread(
            target=self._read_frames,
            name="ffmpeg-latest-frame",
            daemon=True,
        )
        self._stderr_thread.start()
        self._frame_thread.start()

    def _read_exact_frame(self):
        if self.process is None or self.process.stdout is None:
            return None
        buffer = bytearray(self.frame_bytes)
        view = memoryview(buffer)
        offset = 0
        while offset < self.frame_bytes:
            chunk = self.process.stdout.read(self.frame_bytes - offset)
            if not chunk:
                return None
            view[offset : offset + len(chunk)] = chunk
            offset += len(chunk)
        import numpy

        return numpy.frombuffer(buffer, dtype=numpy.uint8).reshape(
            (self.height, self.width, 3)
        )

    def _read_stderr(self):
        if self.process is None or self.process.stderr is None:
            return
        try:
            while True:
                line = self.process.stderr.readline()
                if not line:
                    break
                text = line.decode("utf-8", errors="replace")
                should_log = "vision_sync" not in text
                if "vision_sync" not in text:
                    if self.stderr_handle is not None:
                        self.stderr_handle.write(line)
                    continue
                match = self._PTS_TIME_PATTERN.search(text)
                if match is None:
                    continue
                source_epoch_ns = int(float(match.group(1)) * 1_000_000_000)
                timing = {
                    "source_epoch_ns": source_epoch_ns,
                    "timing_observed_epoch_ns": time.time_ns(),
                }
                with self._timing_condition:
                    self._timing_queue.append(timing)
                    self.timing_frames += 1
                    timing_frame = self.timing_frames
                    self._timing_condition.notify_all()
                should_log = (
                    timing_frame == 1
                    or timing_frame % self.fps == 0
                )
                if should_log and self.stderr_handle is not None:
                    self.stderr_handle.write(line)
        except Exception as exc:
            self._reader_error = exc
        finally:
            with self._timing_condition:
                self._timing_condition.notify_all()

    def _take_timing(self, timeout=0.20):
        deadline = time.monotonic() + float(timeout)
        with self._timing_condition:
            while not self._timing_queue:
                if self.process is not None and self.process.poll() is not None:
                    return None
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self._timing_condition.wait(remaining)
            return self._timing_queue.popleft()

    def _read_frames(self):
        try:
            while True:
                frame = self._read_exact_frame()
                if frame is None:
                    break
                timing = self._take_timing() or {}
                read_epoch_ns = time.time_ns()
                read_monotonic_ns = time.monotonic_ns()
                source_epoch_ns = timing.get("source_epoch_ns")
                pipeline_latency_ms = None
                if source_epoch_ns is not None:
                    pipeline_latency_ms = (
                        read_epoch_ns - int(source_epoch_ns)
                    ) / 1_000_000.0

                with self._frame_condition:
                    self._decode_seq += 1
                    self.decoded_frames += 1
                    self._latest_frame = frame
                    self._latest_frame_info = {
                        "decode_seq": self._decode_seq,
                        "source_epoch_ns": source_epoch_ns,
                        "read_epoch_ns": read_epoch_ns,
                        "read_monotonic_ns": read_monotonic_ns,
                        "pipeline_latency_ms": pipeline_latency_ms,
                    }
                    self._frame_condition.notify_all()
        except Exception as exc:
            self._reader_error = exc
        finally:
            with self._frame_condition:
                self._frame_condition.notify_all()

    def read_latest_frame(self, after_sequence=None, timeout=2.0):
        if after_sequence is None:
            after_sequence = self._last_delivered_seq
        deadline = time.monotonic() + float(timeout)
        with self._frame_condition:
            while self._decode_seq <= int(after_sequence):
                if self.process is not None and self.process.poll() is not None:
                    return None, None
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None, None
                self._frame_condition.wait(remaining)

            frame = self._latest_frame
            info = dict(self._latest_frame_info or {})
            delivered_seq = int(info.get("decode_seq", self._decode_seq))
            dropped = max(0, delivered_seq - int(after_sequence) - 1)
            self.dropped_preview_frames += dropped
            self._last_delivered_seq = delivered_seq
            info["dropped_frames"] = dropped
            return frame, info

    def read_frame(self):
        """Compatibility wrapper returning the next available latest frame."""
        frame, _info = self.read_latest_frame()
        return frame

    def _finish_process(self):
        if self.process is None:
            return
        if self.process.poll() is None:
            try:
                if self.process.stdin is not None:
                    self.process.stdin.write(b"q\n")
                    self.process.stdin.flush()
                self.process.wait(timeout=10)
            except Exception:
                self.process.terminate()
                try:
                    self.process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    self.process.kill()
                    self.process.wait(timeout=2)
        with self._frame_condition:
            self._frame_condition.notify_all()
        with self._timing_condition:
            self._timing_condition.notify_all()
        if self._frame_thread is not None:
            self._frame_thread.join(timeout=2)
        if self._stderr_thread is not None:
            self._stderr_thread.join(timeout=2)
        if self.process.stdout is not None:
            self.process.stdout.close()
        if self.process.stderr is not None:
            self.process.stderr.close()
        if self.process.stdin is not None:
            self.process.stdin.close()

    def _remux(self):
        if (
            not self.record
            or not self.remux_mp4
            or not self.mkv_path.exists()
            or self.mkv_path.stat().st_size == 0
        ):
            return None
        result = subprocess.run(
            [
                self.ffmpeg,
                "-hide_banner",
                "-loglevel",
                "warning",
                "-y",
                "-i",
                str(self.mkv_path),
                "-map",
                "0:v:0",
                "-an",
                "-c:v",
                "copy",
                str(self.mp4_path),
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            check=False,
        )
        if result.returncode == 0 and self.mp4_path.exists():
            self.mkv_path.unlink()
            return self.mp4_path
        return None

    def stop(self):
        self._finish_process()
        if self.stderr_handle is not None:
            self.stderr_handle.flush()
            self.stderr_handle.close()
        return self._remux()

    @property
    def returncode(self):
        return None if self.process is None else self.process.poll()

    @property
    def reader_error(self):
        return self._reader_error
