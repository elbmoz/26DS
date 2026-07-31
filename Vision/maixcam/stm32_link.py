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
    "raw_line",
)

MOTOR_STATUS_NAMES = (
    "HAL_OK",
    "HAL_ERROR",
    "HAL_BUSY",
    "HAL_TIMEOUT",
)

_MAX_FEEDBACK_LINE_CHARS = 512


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
    """Parse one ``F,...`` line and preserve raw and scaled values."""
    text = _decode_ascii(line).strip()
    fields = text.split(",")
    if len(fields) != len(STM32_FEEDBACK_RAW_FIELDS) + 1:
        raise ValueError("STM32 feedback field count must be 13")
    if fields[0] != "F":
        raise ValueError("STM32 feedback must start with F")

    feedback = {}
    for name, value in zip(STM32_FEEDBACK_RAW_FIELDS, fields[1:]):
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
            "raw_line": text,
        }
    )
    return feedback


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
        self.feedback_tail = ""
        self.feedback_queue = []
        self.feedback_queue_size = max(1, int(feedback_queue_size))
        self.last_feedback_seq = None
        self.start_count = 0
        self.stop_count = 0
        self.rx_error_count = 0
        self.tx_error_count = 0
        self.tx_frame_count = 0
        self.feedback_count = 0
        self.feedback_error_count = 0
        self.feedback_gap_count = 0
        self.feedback_queue_drop_count = 0

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

    def _consume_feedback(self, text):
        for character in text:
            self.feedback_tail += character
            if character not in "\r\n":
                if len(self.feedback_tail) > _MAX_FEEDBACK_LINE_CHARS:
                    marker = self.feedback_tail.rfind("F,")
                    if marker >= 0:
                        self.feedback_tail = self.feedback_tail[marker:]
                    else:
                        self.feedback_tail = self.feedback_tail[
                            -_MAX_FEEDBACK_LINE_CHARS:
                        ]
                continue

            line = self.feedback_tail.strip()
            self.feedback_tail = ""
            # Prefer the newest frame marker so one damaged/missing newline
            # cannot make the following complete feedback frame undecodable.
            marker = line.rfind("F,")
            if marker < 0:
                continue
            try:
                feedback = parse_stm32_feedback_line(line[marker:])
            except (TypeError, ValueError):
                self.feedback_error_count += 1
                continue
            self._queue_feedback(feedback)

    def poll_commands(self):
        """Consume currently buffered commands and feedback without blocking."""
        if self.serial_port is None:
            return self.streaming

        try:
            data = self.serial_port.read()
        except Exception:
            self.rx_error_count += 1
            return self.streaming

        text = _decode_ascii(data)
        self._consume_commands(text)
        self._consume_feedback(text)

        return self.streaming

    def drain_feedback(self, max_items=None):
        """Return queued feedback in receive order and remove it from the queue."""
        if max_items is None:
            count = len(self.feedback_queue)
        else:
            count = min(len(self.feedback_queue), max(0, int(max_items)))
        items = self.feedback_queue[:count]
        del self.feedback_queue[:count]
        return items

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
