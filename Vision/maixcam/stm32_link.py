"""UART output adapter for the STM32 Question 2 controller.

This module is deliberately separate from ball detection and tracking.  It
only translates the already-computed error/velocity state into the agreed
serial protocol and handles the STM32 start/stop commands.
"""


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
    """Non-blocking c2/ok command receiver and tracking-frame sender."""

    def __init__(self, serial_port, output_scale=1.0):
        self.serial_port = serial_port
        self.output_scale = float(output_scale)
        self.streaming = False
        self.command_tail = ""
        self.start_count = 0
        self.stop_count = 0
        self.rx_error_count = 0
        self.tx_error_count = 0
        self.tx_frame_count = 0

    @staticmethod
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

    def poll_commands(self):
        """Consume currently buffered UART bytes without blocking."""
        if self.serial_port is None:
            return self.streaming

        try:
            data = self.serial_port.read()
        except Exception:
            self.rx_error_count += 1
            return self.streaming

        text = self._decode_ascii(data).lower()
        for character in text:
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

        return self.streaming

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
