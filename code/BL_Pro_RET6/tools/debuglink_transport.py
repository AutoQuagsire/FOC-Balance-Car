"""
DebugLink serial transport reusable by CLI, GUI, and scripts.

All wire-format parsing delegates to debuglink_parser.
Thread-safe for concurrent stream reading and command/response operations.

Usage::

    t = DebugLinkTransport()
    t.connect("COM33")
    print(t.ping())
    print(t.get_info())
    t.stream_start(rate_hz=100)
    # ... read frames via callback ...
    t.stream_stop()
    t.disconnect()
"""

import struct
import threading
import time

import serial

from debuglink_models import FastRingMeta, FastRingSample, LiveFrame
from debuglink_parser import (
    parse_fastring_chunk,
    parse_fastring_status,
    parse_status_stream,
)

# ---------------------------------------------------------------------------
# Protocol constants (mirror debug_link_protocol.h / debuglink_cli.py)
# ---------------------------------------------------------------------------

SOF0 = 0x5A
SOF1 = 0xA5
VERSION = 0x01
MAX_PAYLOAD = 240

MSG_PING_REQ = 0x01
MSG_GET_DEVICE_INFO_REQ = 0x02
MSG_STREAM_CONTROL_REQ = 0x10
MSG_GET_PARAM_REQ = 0x11
MSG_SET_PARAM_REQ = 0x12
MSG_POWER_STAGE_REQ = 0x15
MSG_ATTITUDE_CONTROL_REQ = 0x16
MSG_FAST_RING_REQ = 0x17
MSG_ACK = 0x80
MSG_NACK = 0x81
MSG_DEVICE_INFO_RSP = 0x82
MSG_PARAM_VALUE_RSP = 0x83
MSG_STATUS_STREAM = 0x90
MSG_FAST_RING_DATA = 0x93

PARAM_TYPE_FLOAT = 0x01
PARAM_SPEED_KP = 0x10
PARAM_SPEED_KI = 0x11
PARAM_SPEED_UNWIND_GAIN = 0x13
PARAM_ATTITUDE_KP = 0x20
PARAM_ATTITUDE_KD = 0x21
PARAM_ATTITUDE_IQ_LIMIT = 0x22
PARAM_ATTITUDE_SHUTDOWN_RAD = 0x23

OP_FASTRING_STATUS = 0x01
OP_FASTRING_SNAPSHOT = 0x02
OP_FASTRING_READ_CHUNK = 0x03


class TransportError(Exception):
    """Fatal transport error (disconnected, serial failure, protocol violation)."""


class TransportTimeout(TransportError):
    """Request timed out after all retries."""


class _FrameReader:
    """SOF-synchronized frame parser using the same algorithm as debuglink_cli."""

    def __init__(self):
        self.reset()

    def reset(self):
        self._state = 0
        self._header = bytearray()
        self._payload = bytearray()
        self._expected_len = 0
        self._crc_bytes = bytearray()

    @staticmethod
    def _crc16(data: bytes) -> int:
        crc = 0xFFFF
        for byte in data:
            crc ^= byte << 8
            for _ in range(8):
                if crc & 0x8000:
                    crc = ((crc << 1) ^ 0x1021) & 0xFFFF
                else:
                    crc = (crc << 1) & 0xFFFF
        return crc

    def feed(self, byte: int):
        """Feed one byte. Returns (msg_type, seq, payload) or None."""
        if self._state == 0:
            if byte == SOF0:
                self._state = 1
            return None

        if self._state == 1:
            if byte == SOF1:
                self._state = 2
                self._header = bytearray()
            else:
                self.reset()
            return None

        if self._state == 2:
            self._header.append(byte)
            if len(self._header) == 5:
                self._expected_len = struct.unpack_from("<H", self._header, 3)[0]
                if self._expected_len > MAX_PAYLOAD:
                    self.reset()
                    return None
                if self._expected_len == 0:
                    self._state = 4
                    self._crc_bytes = bytearray()
                else:
                    self._state = 3
                    self._payload = bytearray()
            return None

        if self._state == 3:
            self._payload.append(byte)
            if len(self._payload) >= self._expected_len:
                self._state = 4
                self._crc_bytes = bytearray()
            return None

        if self._state == 4:
            self._crc_bytes.append(byte)
            if len(self._crc_bytes) == 2:
                self._state = 0
                return self._check_frame()
            return None

        return None

    def _check_frame(self):
        body = bytes(self._header) + bytes(self._payload)
        crc_calc = self._crc16(body)
        crc_recv = struct.unpack_from("<H", self._crc_bytes)[0]
        if crc_calc != crc_recv:
            return None
        msg_type = self._header[1]
        seq = self._header[2]
        return (msg_type, seq, bytes(self._payload))


class DebugLinkTransport:
    """Serial transport that delegates all payload parsing to debuglink_parser.

    Thread safety: a single ``_serial_lock`` mutualizes all serial I/O.
    The stream background thread acquires it for each read cycle; request/
    response methods acquire it for the duration of send + receive.
    """

    def __init__(self):
        self._ser: serial.Serial | None = None
        self._cmd_reader = _FrameReader()
        self._stream_reader = _FrameReader()
        self._serial_lock = threading.Lock()
        self._seq = 0

        self._stream_thread: threading.Thread | None = None
        self._stream_stop_event = threading.Event()
        self._stream_callback = None
        self._stream_rate_hz: int | None = None

    @property
    def is_connected(self) -> bool:
        return self._ser is not None and self._ser.is_open

    def connect(self, port: str, baud: int = 921600) -> None:
        """Open *port* and prepare the link."""
        if self.is_connected:
            self.disconnect()

        try:
            self._ser = serial.Serial(
                port=port,
                baudrate=baud,
                timeout=0.05,
                write_timeout=1.0,
            )
        except (serial.SerialException, OSError) as e:
            raise TransportError(f"cannot open {port}: {e}") from e

        try:
            self._ser.dtr = False
            self._ser.rts = False
        except Exception:
            pass

        time.sleep(0.08)
        try:
            self._ser.reset_input_buffer()
            self._ser.reset_output_buffer()
        except Exception:
            pass

        self._cmd_reader.reset()
        self._stream_reader.reset()
        self._seq = 0

    def disconnect(self) -> None:
        """Stop streaming, close the serial port, and release resources."""
        self.stream_stop()
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None

    def _check_connected(self):
        if not self.is_connected:
            raise TransportError("not connected - call connect() first")

    @staticmethod
    def _crc16(data: bytes) -> int:
        crc = 0xFFFF
        for byte in data:
            crc ^= byte << 8
            for _ in range(8):
                if crc & 0x8000:
                    crc = ((crc << 1) ^ 0x1021) & 0xFFFF
                else:
                    crc = (crc << 1) & 0xFFFF
        return crc

    def _next_seq(self) -> int:
        self._seq = (self._seq + 1) & 0xFF
        return self._seq

    def _build_frame(self, msg_type: int, seq: int, payload: bytes) -> bytes:
        header = bytes([VERSION, msg_type, seq]) + struct.pack("<H", len(payload))
        body = header + payload
        crc = self._crc16(body)
        return bytes([SOF0, SOF1]) + body + struct.pack("<H", crc)

    def _recv_frame(self, reader: _FrameReader, timeout: float):
        """Read one complete frame using *reader* while holding ``_serial_lock``."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                waiting = self._ser.in_waiting
            except (serial.SerialException, OSError, AttributeError) as e:
                raise TransportError(f"serial port inaccessible: {e}") from e

            if waiting <= 0:
                time.sleep(0.001)
                continue

            try:
                data = self._ser.read(1)
            except (serial.SerialException, OSError) as e:
                raise TransportError(f"serial read error: {e}") from e

            if not data:
                continue

            result = reader.feed(data[0])
            if result is not None:
                return result
        return None

    def _request(
        self,
        msg_type: int,
        payload: bytes,
        accept_fn,
        timeout: float = 2.0,
        retries: int = 3,
        retry_delay_ms: int = 80,
    ):
        """Send a request and wait for a matching response."""
        self._check_connected()

        for attempt in range(1, retries + 1):
            with self._serial_lock:
                self._cmd_reader.reset()
                seq = self._next_seq()
                frame = self._build_frame(msg_type, seq, payload)

                try:
                    self._ser.reset_input_buffer()
                    self._ser.write(frame)
                    self._ser.flush()
                except (serial.SerialException, OSError) as e:
                    raise TransportError(f"serial write error: {e}") from e

                deadline = time.monotonic() + timeout
                while time.monotonic() < deadline:
                    remaining = deadline - time.monotonic()
                    result = self._recv_frame(self._cmd_reader, timeout=max(remaining, 0.01))
                    if result is None:
                        break

                    r_msg_type, _, r_payload = result
                    if r_msg_type == MSG_STATUS_STREAM:
                        continue
                    if accept_fn(r_msg_type):
                        return (r_msg_type, r_payload)

            if attempt < retries:
                time.sleep(retry_delay_ms / 1000.0)

        raise TransportTimeout(
            f"request 0x{msg_type:02X} timed out after {retries} attempt(s)"
        )

    def ping(self) -> bool:
        """Send PING_REQ, return ``True`` on ACK."""
        payload = struct.pack("<I", int(time.time() * 1000) & 0xFFFFFFFF)
        try:
            msg_type, resp = self._request(
                MSG_PING_REQ,
                payload,
                accept_fn=lambda mt: mt in (MSG_ACK, MSG_NACK),
            )
        except TransportError:
            return False

        if msg_type == MSG_ACK and len(resp) >= 2:
            return resp[0] == MSG_PING_REQ and resp[1] == 0
        return False

    def _set_bool_control(self, req_msg: int, enable: bool) -> bool:
        payload = struct.pack("<B", 1 if enable else 0)
        try:
            msg_type, resp = self._request(
                req_msg,
                payload,
                accept_fn=lambda mt: mt in (MSG_ACK, MSG_NACK),
            )
        except TransportError:
            return False

        if msg_type == MSG_ACK and len(resp) >= 2:
            return resp[0] == req_msg and resp[1] == 0
        return False

    def driver_enable(self, enable: bool) -> bool:
        """Enable or disable the power stage driver."""
        return self._set_bool_control(MSG_POWER_STAGE_REQ, enable)

    def balance_enable(self, enable: bool) -> bool:
        """Enable or disable attitude/speed balance control."""
        return self._set_bool_control(MSG_ATTITUDE_CONTROL_REQ, enable)

    def get_float_param(self, param_id: int) -> float:
        """Read a float parameter by DebugLink parameter ID."""
        msg_type, resp = self._request(
            MSG_GET_PARAM_REQ,
            struct.pack("<B", param_id),
            accept_fn=lambda mt: mt in (MSG_PARAM_VALUE_RSP, MSG_NACK),
        )

        if msg_type == MSG_NACK:
            reason = resp[1] if len(resp) >= 2 else 0
            raise TransportError(
                f"get_param 0x{param_id:02X} NACK reason={reason}"
            )

        if msg_type != MSG_PARAM_VALUE_RSP or len(resp) < 6:
            raise TransportError(
                f"get_param 0x{param_id:02X} unexpected response: "
                f"msg=0x{msg_type:02X} len={len(resp)}"
            )

        resp_id = resp[0]
        data_type = resp[1]
        if resp_id != param_id or data_type != PARAM_TYPE_FLOAT:
            raise TransportError(
                f"get_param 0x{param_id:02X} mismatched response: "
                f"id=0x{resp_id:02X} type=0x{data_type:02X}"
            )

        return struct.unpack_from("<f", resp, 2)[0]

    def set_float_param(self, param_id: int, value: float) -> bool:
        """Write a float parameter by DebugLink parameter ID."""
        payload = struct.pack("<BBf", param_id, PARAM_TYPE_FLOAT, float(value))
        msg_type, resp = self._request(
            MSG_SET_PARAM_REQ,
            payload,
            accept_fn=lambda mt: mt in (MSG_ACK, MSG_NACK),
        )

        if msg_type == MSG_NACK:
            reason = resp[1] if len(resp) >= 2 else 0
            raise TransportError(
                f"set_param 0x{param_id:02X} NACK reason={reason}"
            )

        if msg_type == MSG_ACK and len(resp) >= 2:
            return resp[0] == MSG_SET_PARAM_REQ and resp[1] == 0
        return False

    def get_speed_ki(self) -> float:
        """Read speed loop Ki (rad/rad)."""
        return self.get_float_param(PARAM_SPEED_KI)

    def set_speed_ki(self, value: float) -> bool:
        """Write speed loop Ki (rad/rad)."""
        return self.set_float_param(PARAM_SPEED_KI, value)

    def get_speed_kp(self) -> float:
        """Read speed loop Kp (rad/radps)."""
        return self.get_float_param(PARAM_SPEED_KP)

    def set_speed_kp(self, value: float) -> bool:
        """Write speed loop Kp (rad/radps)."""
        return self.set_float_param(PARAM_SPEED_KP, value)

    def get_speed_unwind_gain(self) -> float:
        """Read speed loop integral unwind gain."""
        return self.get_float_param(PARAM_SPEED_UNWIND_GAIN)

    def set_speed_unwind_gain(self, value: float) -> bool:
        """Write speed loop integral unwind gain."""
        return self.set_float_param(PARAM_SPEED_UNWIND_GAIN, value)

    def get_attitude_kp(self) -> float:
        """Read attitude loop Kp (A/rad)."""
        return self.get_float_param(PARAM_ATTITUDE_KP)

    def set_attitude_kp(self, value: float) -> bool:
        """Write attitude loop Kp (A/rad)."""
        return self.set_float_param(PARAM_ATTITUDE_KP, value)

    def get_attitude_kd(self) -> float:
        """Read attitude loop Kd (A/radps)."""
        return self.get_float_param(PARAM_ATTITUDE_KD)

    def set_attitude_kd(self, value: float) -> bool:
        """Write attitude loop Kd (A/radps)."""
        return self.set_float_param(PARAM_ATTITUDE_KD, value)

    def get_attitude_iq_limit(self) -> float:
        """Read attitude loop iq limit (A)."""
        return self.get_float_param(PARAM_ATTITUDE_IQ_LIMIT)

    def set_attitude_iq_limit(self, value: float) -> bool:
        """Write attitude loop iq limit (A)."""
        return self.set_float_param(PARAM_ATTITUDE_IQ_LIMIT, value)

    def get_attitude_shutdown_rad(self) -> float:
        """Read attitude shutdown threshold (rad)."""
        return self.get_float_param(PARAM_ATTITUDE_SHUTDOWN_RAD)

    def set_attitude_shutdown_rad(self, value: float) -> bool:
        """Write attitude shutdown threshold (rad)."""
        return self.set_float_param(PARAM_ATTITUDE_SHUTDOWN_RAD, value)

    def get_info(self) -> dict:
        """Send GET_DEVICE_INFO_REQ, return device info dict."""
        msg_type, resp = self._request(
            MSG_GET_DEVICE_INFO_REQ,
            b"",
            accept_fn=lambda mt: mt in (MSG_DEVICE_INFO_RSP, MSG_NACK),
        )

        if msg_type == MSG_NACK:
            reason = resp[1] if len(resp) >= 2 else 0
            raise TransportError(f"get_info NACK reason={reason}")

        if msg_type != MSG_DEVICE_INFO_RSP or len(resp) != 9:
            raise TransportError(
                f"get_info unexpected response: msg=0x{msg_type:02X} len={len(resp)}"
            )

        return {
            "device_type": resp[0],
            "proto_version": resp[1],
            "fw_major": resp[2],
            "fw_minor": resp[3],
            "fw_patch": resp[4],
            "cap_flags": struct.unpack_from("<H", resp, 5)[0],
            "max_payload": struct.unpack_from("<H", resp, 7)[0],
        }

    def set_stream_callback(self, callback):
        """Register a callback for incoming stream frames."""
        self._stream_callback = callback

    def stream_start(self, rate_hz: int = 100) -> bool:
        """Enable STATUS_STREAM telemetry at *rate_hz* (1-500 Hz)."""
        self._check_connected()
        if rate_hz <= 0:
            return False

        if self._stream_thread is not None and self._stream_thread.is_alive():
            if self._stream_rate_hz == rate_hz:
                return True
            self.stream_stop()

        payload = struct.pack("<BBHI", 1, 0, rate_hz, 0xFFFFFFFF)
        try:
            msg_type, resp = self._request(
                MSG_STREAM_CONTROL_REQ,
                payload,
                accept_fn=lambda mt: mt in (MSG_ACK, MSG_NACK),
            )
        except TransportError:
            return False

        if not (msg_type == MSG_ACK and len(resp) >= 2 and resp[1] == 0):
            return False

        self._stream_stop_event.clear()
        self._stream_thread = threading.Thread(
            target=self._stream_reader_thread,
            name="dl-stream",
            daemon=True,
        )
        self._stream_thread.start()
        self._stream_rate_hz = rate_hz
        return True

    def stream_stop(self) -> bool:
        """Stop STATUS_STREAM telemetry."""
        stop_rate_hz = self._stream_rate_hz if self._stream_rate_hz is not None else 100

        # 1) Stop the local reader thread.
        if self._stream_thread is not None:
            self._stream_stop_event.set()
            self._stream_thread.join(timeout=2.0)
            self._stream_thread = None

        # 2) Tell the device to stop.
        if not self.is_connected:
            return False

        payload = struct.pack("<BBHI", 0, 0, stop_rate_hz, 0)
        try:
            msg_type, resp = self._request(
                MSG_STREAM_CONTROL_REQ,
                payload,
                accept_fn=lambda mt: mt in (MSG_ACK, MSG_NACK),
                timeout=1.0,
                retries=2,
            )
            ok = msg_type == MSG_ACK and len(resp) >= 2 and resp[1] == 0
            if ok:
                self._stream_rate_hz = None
            return ok
        except TransportError:
            return False

    def _stream_reader_thread(self):
        """Background target: read frames, dispatch stream frames to callback."""
        while not self._stream_stop_event.is_set():
            with self._serial_lock:
                self._stream_reader.reset()
                try:
                    result = self._recv_frame(self._stream_reader, timeout=0.2)
                except TransportError:
                    break

            if result is None:
                continue

            msg_type, _, payload = result
            if msg_type != MSG_STATUS_STREAM:
                continue

            try:
                host_rx_time_ms = int(time.time() * 1000)
                frame = parse_status_stream(payload, host_rx_time_ms)
            except ValueError:
                continue

            cb = self._stream_callback
            if cb is not None:
                try:
                    cb(frame)
                except Exception:
                    pass

    def fastring_status(self) -> FastRingMeta:
        """Query the continuous dual-motor FastRing status."""
        payload = struct.pack("<B", OP_FASTRING_STATUS)
        msg_type, resp = self._request(
            MSG_FAST_RING_REQ,
            payload,
            accept_fn=lambda mt: mt == MSG_FAST_RING_DATA,
        )

        if msg_type != MSG_FAST_RING_DATA:
            raise TransportError(
                f"fastring_status unexpected response: msg=0x{msg_type:02X}"
            )

        try:
            return parse_fastring_status(resp)
        except ValueError as e:
            raise TransportError(f"fastring_status parse error: {e}") from e

    def fastring_snapshot(self) -> FastRingMeta:
        """Freeze the latest FastRing window and return its snapshot metadata."""
        payload = struct.pack("<B", OP_FASTRING_SNAPSHOT)
        msg_type, resp = self._request(
            MSG_FAST_RING_REQ,
            payload,
            accept_fn=lambda mt: mt == MSG_FAST_RING_DATA,
        )

        if msg_type != MSG_FAST_RING_DATA:
            raise TransportError(
                f"fastring_snapshot unexpected response: msg=0x{msg_type:02X}"
            )

        try:
            return parse_fastring_status(resp)
        except ValueError as e:
            raise TransportError(f"fastring_snapshot parse error: {e}") from e

    def fastring_read_chunk(
        self, snapshot_write_seq: int, start_idx: int, max_samples: int
    ) -> tuple[FastRingMeta, list[FastRingSample]]:
        """Read one chunk from a frozen FastRing snapshot window."""
        payload = struct.pack("<BIHB", OP_FASTRING_READ_CHUNK, snapshot_write_seq, start_idx, max_samples)
        msg_type, resp = self._request(
            MSG_FAST_RING_REQ,
            payload,
            accept_fn=lambda mt: mt == MSG_FAST_RING_DATA,
        )

        if msg_type != MSG_FAST_RING_DATA:
            raise TransportError(
                f"fastring_read_chunk unexpected response: msg=0x{msg_type:02X}"
            )

        try:
            return parse_fastring_chunk(resp)
        except ValueError as e:
            raise TransportError(f"fastring_read_chunk parse error: {e}") from e
