"""UART output adapter for the STM32 Question 2 controller.

This module is deliberately separate from ball detection and tracking.  It
only translates the already-computed error/velocity state into the agreed
serial protocol, handles the STM32 start/stop commands, and parses controller
feedback without blocking the vision loop.
"""

STM32_FEEDBACK_RAW_FIELDS = (
    "seq",
    "mcu_ms",
    "vision_frame",
    "vision_age_ms",
    "position_x10",
    "velocity_x10",
    "error_x10",
    "p_x100",
    "i_x100",
    "d_x100",
    "motor_command",
    "motor_status",
)

STM32_FEEDBACK_V2_RAW_FIELDS = (
    "seq",
    "mcu_ms",
    "vision_frame",
    "vision_age_ms",
    "position_x10",
    "velocity_x10",
    "error_x10",
    "p_x100",
    "i_x100",
    "d_x100",
    "target_angle_x100",
    "actual_angle_x100",
    "rod_rate_x100",
    "angle_error_x100",
    "desired_speed_x100",
    "motor_command",
    "position_age_ms",
    "position_valid",
    "protection_state",
    "motor_status",
    "tuning_mode",
)

PID_PARAMETER_ORDER = (
    "outer_kp",
    "outer_kd",
    "angle_limit",
    "inner_kp",
    "inner_kd",
    "speed_limit",
    "slew",
    "deadband",
    "min_speed",
)

PID_PARAMETER_MASKS = {
    name: 1 << index for index, name in enumerate(PID_PARAMETER_ORDER)
}

STM32_FEEDBACK_CSV_FIELDS = (
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
    "raw_line",
)

MOTOR_STATUS_NAMES = (
    "HAL_OK",
    "HAL_ERROR",
    "HAL_BUSY",
    "HAL_TIMEOUT",
)

Q9_RAW_FIELDS = (
    "seq",
    "mcu_ms",
    "motor_position",
    "angle_x_x10",
    "angle_y_x10",
    "angle_z_x10",
    "imu_valid",
    "position_valid",
    "position_status",
    "position_updates",
    "move_direction",
    "move_status",
)

_MAX_RX_LINE_CHARS = 512
_UINT32_MASK = 0xFFFFFFFF
_UINT32_HALF_RANGE = 0x80000000


def _decode_ascii(data):
    if data is None:
        return ""
    if isinstance(data, str):
        return data
    try:
        return bytes(data).decode("ascii", "ignore")
    except Exception:
        try:
            return data.decode("ascii", "ignore")
        except Exception:
            return ""


def parse_stm32_feedback_line(line):
    """Parse one legacy ``F`` or complete ``F2`` controller frame."""
    text = _decode_ascii(line).strip()
    fields = text.split(",")
    if fields[0] == "F":
        raw_fields = STM32_FEEDBACK_RAW_FIELDS
        version = 1
    elif fields[0] == "F2":
        raw_fields = STM32_FEEDBACK_V2_RAW_FIELDS
        version = 2
    else:
        raise ValueError("STM32 feedback must start with F or F2")
    if len(fields) != len(raw_fields) + 1:
        raise ValueError("invalid STM32 feedback field count")

    feedback = {}
    for name, value in zip(raw_fields, fields[1:]):
        feedback[name] = int(value)

    motor_status = feedback["motor_status"]
    if motor_status < 0 or motor_status >= len(MOTOR_STATUS_NAMES):
        raise ValueError("invalid STM32 motor_status")

    feedback.update(
        {
            "position_px": feedback["position_x10"] / 10.0,
            "velocity_px_s": feedback["velocity_x10"] / 10.0,
            "control_error_px": feedback["error_x10"] / 10.0,
            "p_term": feedback["p_x100"] / 100.0,
            "i_term": feedback["i_x100"] / 100.0,
            "d_term": feedback["d_x100"] / 100.0,
            "motor_status_name": MOTOR_STATUS_NAMES[motor_status],
            "feedback_version": version,
            "raw_line": text,
        }
    )
    if version == 2:
        feedback.update(
            {
                "target_rod_angle_deg": (
                    feedback["target_angle_x100"] / 100.0
                ),
                "actual_rod_angle_deg": (
                    feedback["actual_angle_x100"] / 100.0
                ),
                "rod_rate_deg_s": feedback["rod_rate_x100"] / 100.0,
                "angle_error_deg": feedback["angle_error_x100"] / 100.0,
                "desired_motor_speed": (
                    feedback["desired_speed_x100"] / 100.0
                ),
            }
        )
    return feedback


def parse_pid_ack_line(line):
    """Parse the STM32 ACK emitted only after a RAM update is applied."""
    text = _decode_ascii(line).strip()
    fields = text.split(",")
    if len(fields) != 15 or fields[0] != "PA":
        raise ValueError("PID ACK must contain 15 fields and start with PA")
    values = [_parse_decimal_integer(value) for value in fields[1:]]
    config = {
        "outer_kp": values[2] / 1000000.0,
        "outer_kd": values[3] / 1000000.0,
        "angle_limit": values[4] / 1000.0,
        "inner_kp": values[5] / 1000.0,
        "inner_kd": values[6] / 1000.0,
        "speed_limit": values[7] / 100.0,
        "slew": values[8] / 100.0,
        "deadband": values[9] / 100.0,
        "min_speed": values[10] / 100.0,
    }
    return {
        "seq": values[0] & _UINT32_MASK,
        "status": values[1],
        "ok": values[1] == 0,
        "config": config,
        "mode": values[11],
        "test_target": values[12] / 100.0,
        "remaining_ms": values[13],
        "raw_line": text,
    }


def _parse_decimal_integer(value):
    if not value:
        raise ValueError("empty decimal integer")
    digits = value
    if value[0] in ("+", "-"):
        digits = value[1:]
    if not digits or not digits.isdigit():
        raise ValueError("invalid decimal integer")
    return int(value, 10)


def parse_q9_line(line):
    """Strictly parse one STM32 Question 9 telemetry frame."""
    if isinstance(line, str):
        text = line.strip()
        try:
            text.encode("ascii")
        except UnicodeEncodeError:
            raise ValueError("Q9 frame must be ASCII")
    else:
        try:
            text = bytes(line).decode("ascii").strip()
        except Exception:
            raise ValueError("Q9 frame must be ASCII")

    fields = text.split(",")
    if len(fields) != len(Q9_RAW_FIELDS) + 1:
        raise ValueError("Q9 field count must be 13")
    if fields[0] != "Q9":
        raise ValueError("Q9 frame must start with Q9")

    frame = {}
    for name, value in zip(Q9_RAW_FIELDS, fields[1:]):
        frame[name] = _parse_decimal_integer(value)

    sequence = frame["seq"]
    if sequence < 0 or sequence > _UINT32_MASK:
        raise ValueError("Q9 seq must be an unsigned 32-bit integer")
    if frame["imu_valid"] not in (0, 1):
        raise ValueError("Q9 imu_valid must be 0 or 1")
    if frame["position_valid"] not in (0, 1):
        raise ValueError("Q9 position_valid must be 0 or 1")
    if frame["position_status"] not in (0, 1, 2, 3):
        raise ValueError("Q9 position_status must be in range 0..3")
    if frame["move_direction"] not in (-1, 0, 1):
        raise ValueError("Q9 move_direction must be -1, 0 or 1")
    if frame["move_status"] not in (0, 1, 2, 3):
        raise ValueError("Q9 move_status must be in range 0..3")

    frame.update(
        {
            "angle_x_deg": frame["angle_x_x10"] / 10.0,
            "angle_y_deg": frame["angle_y_x10"] / 10.0,
            "angle_z_deg": frame["angle_z_x10"] / 10.0,
            "raw_line": text,
        }
    )
    return frame


def q9_overlay_lines(q9):
    """Return the four compact preview lines for a parsed Q9 frame."""
    if not q9:
        return ()
    return (
        "Q9 P:{}".format(q9["motor_position"]),
        "X:{:.1f} Y:{:.1f} Z:{:.1f}".format(
            q9["angle_x_deg"],
            q9["angle_y_deg"],
            q9["angle_z_deg"],
        ),
        "IMU:{} POS:{} RX:{} N:{}".format(
            "V" if q9["imu_valid"] else "X",
            "V" if q9["position_valid"] else "X",
            q9["position_status"],
            q9["position_updates"],
        ),
        "DIR:{} MOVE:{}".format(
            q9["move_direction"],
            q9["move_status"],
        ),
    )


def feedback_csv_header():
    return ",".join(STM32_FEEDBACK_CSV_FIELDS) + "\n"


def _csv_value(value):
    if value is None:
        return ""
    text = str(value)
    if any(character in text for character in (",", '"', "\r", "\n")):
        return '"' + text.replace('"', '""') + '"'
    return text


def feedback_csv_row(feedback, device_ms):
    """Format a feedback record for MaixCAM's local recording mode."""
    row = dict(feedback)
    row["device_ms"] = int(device_ms)
    return (
        ",".join(
            _csv_value(row.get(name, ""))
            for name in STM32_FEEDBACK_CSV_FIELDS
        )
        + "\n"
    )


def _clamp_int16(value):
    value = int(round(float(value)))
    if value > 32767:
        return 32767
    if value < -32768:
        return -32768
    return value


def format_stm32_line(state, output_scale=1.0):
    """Return ``B,error_px,velocity_px_s`` or ``none`` with a newline."""
    if not state.get("valid", False):
        return "none\n"

    scale = float(output_scale)
    error_px = _clamp_int16(state["error_px"] * scale)
    velocity_px_s = _clamp_int16(state["velocity_px_s"] * scale)
    return "B,{},{}\n".format(error_px, velocity_px_s)


class Stm32Link:
    """Non-blocking command/feedback receiver and tracking-frame sender."""

    def __init__(self, serial_port, output_scale=1.0, feedback_queue_size=128):
        self.serial_port = serial_port
        self.output_scale = float(output_scale)
        self.streaming = False
        self.command_tail = ""
        self.rx_buffer = ""
        self.rx_discarding_line = False
        self.rx_line_invalid = False
        # Kept as a compatibility alias for older diagnostics.
        self.feedback_tail = ""
        self.feedback_queue = []
        self.pid_ack_queue = []
        self.feedback_queue_size = max(1, int(feedback_queue_size))
        self.last_feedback_seq = None
        self.latest_q9 = None
        self.last_q9_seq = None
        self.start_count = 0
        self.stop_count = 0
        self.rx_error_count = 0
        self.tx_error_count = 0
        self.tx_frame_count = 0
        self.feedback_count = 0
        self.feedback_error_count = 0
        self.feedback_gap_count = 0
        self.feedback_queue_drop_count = 0
        self.q9_frame_count = 0
        self.q9_parse_error_count = 0
        self.q9_sequence_gap_count = 0
        self.pid_request_sequence = 0
        self.pid_ack_count = 0
        self.pid_ack_error_count = 0
        self.pid_config = None

    def _consume_commands(self, text):
        for character in text.lower():
            if character in " \t\r\n":
                continue
            self.command_tail = (self.command_tail + character)[-2:]
            if self.command_tail == "c2":
                self.streaming = True
                self.start_count += 1
                self.command_tail = ""
            elif self.command_tail == "ok":
                self.streaming = False
                self.stop_count += 1
                self.command_tail = ""

    def _queue_feedback(self, feedback):
        sequence = int(feedback["seq"]) & 0xFFFFFFFF
        gap = 0
        if self.last_feedback_seq is not None:
            delta = (sequence - self.last_feedback_seq) & 0xFFFFFFFF
            if 1 < delta < 0x80000000:
                gap = delta - 1
                self.feedback_gap_count += gap
        self.last_feedback_seq = sequence
        feedback["seq_gap"] = gap

        if len(self.feedback_queue) >= self.feedback_queue_size:
            del self.feedback_queue[0]
            self.feedback_queue_drop_count += 1
        self.feedback_queue.append(feedback)
        self.feedback_count += 1

    def _queue_q9(self, frame):
        sequence = int(frame["seq"]) & _UINT32_MASK
        gap = 0
        if self.last_q9_seq is None:
            self.last_q9_seq = sequence
        else:
            delta = (sequence - self.last_q9_seq) & _UINT32_MASK
            if 0 < delta < _UINT32_HALF_RANGE:
                if delta > 1:
                    gap = delta - 1
                    self.q9_sequence_gap_count += gap
                self.last_q9_seq = sequence
            # A duplicate or an older/out-of-order frame is still valid data,
            # but must not move the sequence reference backwards.
        frame["seq_gap"] = gap
        self.latest_q9 = frame
        self.q9_frame_count += 1

    def _count_bad_line(self, line):
        q9_marker = line.rfind("Q9")
        feedback_marker = max(line.rfind("F,"), line.rfind("F2,"))
        pid_ack_marker = line.rfind("PA,")
        if q9_marker >= feedback_marker and q9_marker >= 0:
            self.q9_parse_error_count += 1
        elif pid_ack_marker >= 0:
            self.pid_ack_error_count += 1
        elif feedback_marker >= 0:
            self.feedback_error_count += 1

    def _consume_line(self, line):
        text = line.strip()
        if not text:
            return
        if self.rx_line_invalid:
            self._count_bad_line(text)
            return

        q9_marker = text.rfind("Q9")
        feedback_marker = max(text.rfind("F,"), text.rfind("F2,"))
        pid_ack_marker = text.rfind("PA,")
        if pid_ack_marker >= 0:
            try:
                ack = parse_pid_ack_line(text[pid_ack_marker:])
            except (TypeError, ValueError):
                self.pid_ack_error_count += 1
                return
            self.pid_config = dict(ack["config"])
            if ack["ok"]:
                self.streaming = True
            self.pid_ack_queue.append(ack)
            self.pid_ack_count += 1
            return
        if q9_marker >= feedback_marker and q9_marker >= 0:
            try:
                frame = parse_q9_line(text[q9_marker:])
            except (TypeError, ValueError):
                self.q9_parse_error_count += 1
                return
            self._queue_q9(frame)
            return
        if feedback_marker >= 0:
            try:
                feedback = parse_stm32_feedback_line(
                    text[feedback_marker:]
                )
            except (TypeError, ValueError):
                self.feedback_error_count += 1
                return
            self._queue_feedback(feedback)

    def _finish_rx_line(self):
        if self.rx_discarding_line:
            self.rx_discarding_line = False
            self.rx_line_invalid = False
            self.rx_buffer = ""
            self.feedback_tail = ""
            return
        line = self.rx_buffer
        self.rx_buffer = ""
        self.feedback_tail = ""
        self._consume_line(line)
        self.rx_line_invalid = False

    def _discard_overlong_line(self):
        self.rx_error_count += 1
        self._count_bad_line(self.rx_buffer)
        self.rx_buffer = ""
        self.feedback_tail = ""
        self.rx_line_invalid = False
        self.rx_discarding_line = True

    def _consume_rx_character(self, character):
        self._consume_commands(character)
        if character in "\r\n":
            self._finish_rx_line()
            return
        if self.rx_discarding_line:
            return
        if len(self.rx_buffer) >= _MAX_RX_LINE_CHARS:
            self._discard_overlong_line()
            return
        self.rx_buffer += character
        self.feedback_tail = self.rx_buffer

    def _consume_rx_data(self, data):
        if data is None:
            return
        if isinstance(data, str):
            characters = data
        else:
            try:
                characters = bytes(data)
            except Exception:
                self.rx_error_count += 1
                return

        for value in characters:
            if isinstance(value, int):
                if value > 0x7F:
                    self.rx_error_count += 1
                    self.rx_line_invalid = True
                    self.command_tail = ""
                    continue
                character = chr(value)
            else:
                if ord(value) > 0x7F:
                    self.rx_error_count += 1
                    self.rx_line_invalid = True
                    self.command_tail = ""
                    continue
                character = value
            self._consume_rx_character(character)

    def poll_commands(self):
        """Consume currently buffered commands and feedback without blocking."""
        if self.serial_port is None:
            return self.streaming

        try:
            data = self.serial_port.read()
        except Exception:
            self.rx_error_count += 1
            return self.streaming

        self._consume_rx_data(data)

        return self.streaming

    def get_latest_q9(self):
        """Return an isolated copy of the latest valid Q9 frame."""
        if self.latest_q9 is None:
            return None
        return dict(self.latest_q9)

    def drain_feedback(self, max_items=None):
        """Return queued feedback in receive order and remove it from the queue."""
        if max_items is None:
            count = len(self.feedback_queue)
        else:
            count = min(len(self.feedback_queue), max(0, int(max_items)))
        items = self.feedback_queue[:count]
        del self.feedback_queue[:count]
        return items

    def drain_pid_acks(self):
        items = self.pid_ack_queue[:]
        del self.pid_ack_queue[:]
        return items

    def send_pid_request(self, action, params=None):
        """Forward one Windows PID operation to STM32 and return MCU seq."""
        if self.serial_port is None:
            raise RuntimeError("STM32 UART is unavailable")
        params = dict(params or {})
        self.pid_request_sequence = (
            self.pid_request_sequence + 1
        ) & _UINT32_MASK
        sequence = self.pid_request_sequence
        if action == "get":
            line = "PG,{}\n".format(sequence)
        elif action == "reset":
            line = "PR,{}\n".format(sequence)
        elif action == "stop":
            line = "PX,{}\n".format(sequence)
        elif action == "test":
            mode = str(params.get("mode", "")).lower()
            mode_code = {"inner": "I", "outer": "O"}.get(mode)
            if mode_code is None:
                raise ValueError("PID test mode must be inner or outer")
            target = float(params["target"])
            duration_ms = int(params["duration_ms"])
            line = "PT,{},{},{:.9g},{}\n".format(
                sequence, mode_code, target, duration_ms
            )
        elif action == "set":
            unknown = set(params) - set(PID_PARAMETER_ORDER)
            if unknown or not params:
                raise ValueError(
                    "unknown or empty PID parameters: {}".format(
                        sorted(unknown)
                    )
                )
            mask = 0
            values = []
            for name in PID_PARAMETER_ORDER:
                if name in params:
                    mask |= PID_PARAMETER_MASKS[name]
                    values.append("{:.9g}".format(float(params[name])))
            line = "PS,{},{},{}\n".format(
                sequence, mask, ",".join(values)
            )
        else:
            raise ValueError("unknown PID action {}".format(action))

        written = self.serial_port.write_str(line)
        if written is not None and written < len(line):
            self.tx_error_count += 1
            raise RuntimeError("short STM32 PID UART write")
        return sequence

    def send_state(self, state):
        """Send one frame when Question 2 streaming has been enabled."""
        if self.serial_port is None or not self.streaming:
            return 0

        line = format_stm32_line(state, self.output_scale)
        try:
            written = self.serial_port.write_str(line)
        except Exception:
            self.tx_error_count += 1
            return -1

        if written is None:
            written = len(line)
        if written < 0:
            self.tx_error_count += 1
            return written

        self.tx_frame_count += 1
        return written
