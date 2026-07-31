"""Non-blocking UDP telemetry and safe runtime configuration transport."""

import socket

from stream_protocol import (
    MAX_PACKET_BYTES,
    ProtocolError,
    apply_parameters,
    config_snapshot,
    decode_packet,
    encode_packet,
    make_config_ack,
    make_subscribe_ack,
    parse_subscribe_request,
    parse_set_config_request,
)


class UdpVisionLink:
    def __init__(
        self,
        session_id,
        telemetry_targets,
        telemetry_port,
        control_port,
        control_token,
    ):
        self.session_id = str(session_id)
        if isinstance(telemetry_targets, str):
            telemetry_targets = (telemetry_targets,)
        self.telemetry_targets = tuple(telemetry_targets)
        self.telemetry_port = int(telemetry_port)
        self.control_port = int(control_port)
        self.control_token = str(control_token)
        self.send_errors = 0
        self.control_errors = 0
        self.subscribers = set()

        self.sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sender.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.sender.setblocking(False)

        self.control = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.control.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.control.bind(("", self.control_port))
        self.control.setblocking(False)

    def send(self, packet):
        destinations = set(
            (str(host), self.telemetry_port)
            for host in self.telemetry_targets
        )
        destinations.update(self.subscribers)
        if not destinations:
            return False
        try:
            payload = encode_packet(packet)
        except Exception as exc:
            self.send_errors += 1
            print("telemetry encode failed:", exc)
            return False

        sent = False
        for destination in destinations:
            try:
                self.sender.sendto(payload, destination)
                sent = True
            except OSError:
                self.send_errors += 1
        return sent

    def _send_ack(self, address, packet):
        try:
            self.control.sendto(encode_packet(packet), address)
        except Exception as exc:
            self.control_errors += 1
            print("control ack failed:", exc)

    def poll_controls(
        self,
        detector,
        tracker,
        config_module,
        max_packets=4,
    ):
        events = []
        for _ in range(max(1, int(max_packets))):
            try:
                data, address = self.control.recvfrom(MAX_PACKET_BYTES + 1)
            except OSError:
                break

            request_id = "unknown"
            applied = {}
            errors = {}
            try:
                packet_type = decode_packet(data).get("type")
                if packet_type == "subscribe":
                    request_id, telemetry_port = parse_subscribe_request(
                        data, self.control_token
                    )
                    destination = (address[0], telemetry_port)
                    self.subscribers.add(destination)
                    ack = make_subscribe_ack(
                        self.session_id,
                        request_id,
                        True,
                        telemetry_port,
                    )
                    self._send_ack(address, ack)
                    events.append(
                        {
                            "request_id": request_id,
                            "ok": True,
                            "applied": {
                                "subscriber": destination,
                            },
                            "errors": {},
                            "source": address,
                        }
                    )
                    continue
                if packet_type == "set_config":
                    request_id, clean, errors = parse_set_config_request(
                        data, self.control_token
                    )
                    if not errors:
                        current = apply_parameters(
                            clean, detector, tracker, config_module
                        )
                        applied = clean
                        ok = True
                    else:
                        current = config_snapshot(detector, tracker)
                        ok = False
                else:
                    raise ProtocolError(
                        "invalid_type",
                        "expected subscribe or set_config packet",
                    )
            except ProtocolError as exc:
                self.control_errors += 1
                errors = {exc.code: str(exc)}
                current = config_snapshot(detector, tracker)
                ok = False
            except Exception as exc:
                self.control_errors += 1
                errors = {"internal_error": str(exc)}
                current = config_snapshot(detector, tracker)
                ok = False

            ack = make_config_ack(
                self.session_id,
                request_id,
                ok,
                applied,
                errors,
                current,
            )
            self._send_ack(address, ack)
            events.append(
                {
                    "request_id": request_id,
                    "ok": ok,
                    "applied": applied,
                    "errors": errors,
                    "source": address,
                }
            )
        return events

    def close(self):
        try:
            self.control.close()
        except Exception:
            pass
        try:
            self.sender.close()
        except Exception:
            pass
