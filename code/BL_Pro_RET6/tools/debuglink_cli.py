#!/usr/bin/env python3
"""
DebugLink CLI tool.

Usage:
    python debuglink_cli.py --port COM33 ping
    python debuglink_cli.py --port COM33 info
    python debuglink_cli.py --port COM33 driver on
    python debuglink_cli.py --port COM33 balance on
    python debuglink_cli.py --port COM33 stream --rate 100

Requires: pyserial (pip install pyserial)
"""

import argparse
import signal
import struct
import sys
import time

import serial

from debuglink_parser import (
    parse_fastring_chunk,
    parse_fastring_status,
    parse_status_stream,
)

SOF0 = 0x5A
SOF1 = 0xA5
VERSION = 0x01

MSG_PING_REQ = 0x01
MSG_GET_DEVICE_INFO_REQ = 0x02
MSG_STREAM_CONTROL_REQ = 0x10
MSG_POWER_STAGE_REQ = 0x15
MSG_ATTITUDE_CONTROL_REQ = 0x16
MSG_FAST_RING_REQ = 0x17
MSG_ACK = 0x80
MSG_NACK = 0x81
MSG_DEVICE_INFO_RSP = 0x82
MSG_STATUS_STREAM = 0x90
MSG_FAST_RING_DATA = 0x93

OP_FASTRING_STATUS = 0x01
OP_FASTRING_SNAPSHOT = 0x02
OP_FASTRING_READ_CHUNK = 0x03

FASTCAP_SOURCE_LEFT = 0
FASTCAP_SOURCE_RIGHT = 1
FASTRING_CHUNK_MAX = 8  # (240 - 12) // 26

FOC_STATUS_FLAG_SPEED_FAULT_L = 1 << 0
FOC_STATUS_FLAG_SPEED_FAULT_R = 1 << 1
FOC_STATUS_FLAG_STACK_READY = 1 << 8
FOC_STATUS_FLAG_CONTROL_IT_ENABLED = 1 << 9
FOC_STATUS_FLAG_BUS_VALID = 1 << 10
FOC_STATUS_FLAG_CURRENT_LOOP_ACTIVE = 1 << 11
FOC_STATUS_FLAG_SPEED_LOOP_ENABLED = 1 << 12
FOC_STATUS_FLAG_CURRENT_LOOP_ENABLED = 1 << 13
FOC_STATUS_FLAG_POWER_STAGE_OFF = 1 << 14
FOC_STATUS_FLAG_ATTITUDE_CONTROL_ON = 1 << 15

# ── Parameter protocol ───────────────────────────────────────────────
MSG_GET_PARAM_REQ   = 0x11
MSG_SET_PARAM_REQ   = 0x12
MSG_SAVE_PARAMS_REQ = 0x13
MSG_PARAM_VALUE_RSP = 0x83

PARAM_TYPE_FLOAT = 0x01
PARAM_TYPE_UINT8 = 0x02

# name → (id, type, scale, description)
PARAM_TABLE = {
    "current_kp":           (0x01, "float", 1.0, "Current loop Kp"),
    "current_ki":           (0x02, "float", 1.0, "Current loop Ki"),
    "current_kd":           (0x03, "float", 1.0, "Current loop Kd"),
    "current_ilim":         (0x04, "float", 1.0, "Current loop ILim (A)"),
    "current_output_limit": (0x05, "float", 1.0, "Current loop output limit"),
    "current_i_err_min":    (0x06, "float", 1.0, "Current loop I_ERR_MIN"),
    "current_i_sep_ratio":  (0x07, "float", 1.0, "Current loop I_SEP_RATIO"),
    "current_mode":         (0x08, "uint8", 1.0, "Current PID mode (0=FFPI_V1, 1=Pure PI)"),
    "speed_kp":             (0x10, "float", 1.0, "Speed loop Kp (rad/radps)"),
    "speed_ki":             (0x11, "float", 1.0, "Speed loop Ki (rad/rad)"),
    "speed_pitch_limit":    (0x12, "float", 1.0, "Speed loop pitch limit (rad)"),
    "speed_unwind_gain":    (0x13, "float", 1.0, "Speed loop integral unwind gain"),
    "attitude_kp":          (0x20, "float", 1.0, "Attitude loop Kp (A/rad)"),
    "attitude_kd":          (0x21, "float", 1.0, "Attitude loop Kd (A/radps)"),
    "attitude_iq_limit":    (0x22, "float", 1.0, "Attitude loop Iq limit (A)"),
    "attitude_shutdown_rad":(0x23, "float", 1.0, "Attitude shutdown angle (rad)"),
}
PARAM_BY_ID = {v[0]: (k, *v[1:]) for k, v in PARAM_TABLE.items()}

DEFAULT_OPEN_RETRIES = 3
DEFAULT_CMD_RETRIES = 3
DEFAULT_SETTLE_MS = 80
DEFAULT_RETRY_DELAY_MS = 80


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def build_frame(msg_type: int, seq: int, payload: bytes) -> bytes:
    header = bytes([VERSION, msg_type, seq]) + struct.pack("<H", len(payload))
    body = header + payload
    crc = crc16_ccitt(body)
    return bytes([SOF0, SOF1]) + body + struct.pack("<H", crc)


def sleep_ms(delay_ms: int):
    time.sleep(max(delay_ms, 0) / 1000.0)


def format_foc_status(flags: int) -> str:
    labels = []
    if flags & FOC_STATUS_FLAG_SPEED_FAULT_L:
        labels.append("spdfltL")
    if flags & FOC_STATUS_FLAG_SPEED_FAULT_R:
        labels.append("spdfltR")
    if flags & FOC_STATUS_FLAG_STACK_READY:
        labels.append("stack")
    if flags & FOC_STATUS_FLAG_CONTROL_IT_ENABLED:
        labels.append("it")
    if flags & FOC_STATUS_FLAG_BUS_VALID:
        labels.append("bus")
    if flags & FOC_STATUS_FLAG_CURRENT_LOOP_ACTIVE:
        labels.append("iloop")
    if flags & FOC_STATUS_FLAG_SPEED_LOOP_ENABLED:
        labels.append("sloop")
    if flags & FOC_STATUS_FLAG_CURRENT_LOOP_ENABLED:
        labels.append("cloop")
    if flags & FOC_STATUS_FLAG_POWER_STAGE_OFF:
        labels.append("drv_off")
    if flags & FOC_STATUS_FLAG_ATTITUDE_CONTROL_ON:
        labels.append("bal")
    return ",".join(labels) if labels else "-"


class FrameReader:
    def __init__(self):
        self.reset()

    def reset(self):
        self.state = 0
        self.header = bytearray()
        self.payload = bytearray()
        self.expected_len = 0
        self.crc_bytes = bytearray()

    def feed(self, byte: int):
        if self.state == 0:
            if byte == SOF0:
                self.state = 1
            return None

        if self.state == 1:
            if byte == SOF1:
                self.state = 2
                self.header = bytearray()
            else:
                self.reset()
            return None

        if self.state == 2:
            self.header.append(byte)
            if len(self.header) == 5:
                self.expected_len = struct.unpack_from("<H", self.header, 3)[0]
                if self.expected_len > 240:
                    self.reset()
                    return None
                if self.expected_len == 0:
                    self.state = 4
                    self.crc_bytes = bytearray()
                else:
                    self.state = 3
                    self.payload = bytearray()
            return None

        if self.state == 3:
            self.payload.append(byte)
            if len(self.payload) >= self.expected_len:
                self.state = 4
                self.crc_bytes = bytearray()
            return None

        if self.state == 4:
            self.crc_bytes.append(byte)
            if len(self.crc_bytes) == 2:
                self.state = 0
                return self._check_frame()
            return None

        return None

    def _check_frame(self):
        body = bytes(self.header) + bytes(self.payload)
        crc_calc = crc16_ccitt(body)
        crc_recv = struct.unpack_from("<H", self.crc_bytes)[0]
        if crc_calc != crc_recv:
            print(f"[WARN] CRC mismatch: calc=0x{crc_calc:04X} recv=0x{crc_recv:04X}")
            return None
        msg_type = self.header[1]
        seq = self.header[2]
        return (msg_type, seq, bytes(self.payload))


def try_reset_input_buffer(ser: serial.Serial):
    try:
        ser.reset_input_buffer()
    except Exception:
        pass


def try_reset_output_buffer(ser: serial.Serial):
    try:
        ser.reset_output_buffer()
    except Exception:
        pass


def recv_frame(ser: serial.Serial, reader: FrameReader, timeout: float = 2.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        waiting = ser.in_waiting
        if waiting <= 0:
            time.sleep(0.001)
            continue

        data = ser.read(1)
        if not data:
            continue

        result = reader.feed(data[0])
        if result is not None:
            return result
    return None


def is_unsolicited_stream_frame(frame) -> bool:
    return frame is not None and frame[0] == MSG_STATUS_STREAM


def open_serial_port(port: str, baud: int, settle_ms: int, open_retries: int) -> serial.Serial:
    last_error = None

    for attempt in range(1, open_retries + 1):
        ser = None
        try:
            ser = serial.Serial(
                port=port,
                baudrate=baud,
                timeout=0.05,
                write_timeout=1.0,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False,
                exclusive=None,
            )

            print(f"[LINK] open {port} {baud} (attempt {attempt}/{open_retries})")

            try:
                ser.dtr = False
                ser.rts = False
            except Exception:
                pass

            sleep_ms(settle_ms)

            try_reset_input_buffer(ser)
            try_reset_output_buffer(ser)
            return ser
        except (serial.SerialException, OSError, PermissionError) as e:
            last_error = e
            if ser is not None:
                try:
                    ser.close()
                except Exception:
                    pass
            if attempt < open_retries:
                print(f"[WARN] open failed: {e}")
                sleep_ms(settle_ms)

    raise serial.SerialException(f"open {port} failed after {open_retries} attempts: {last_error}")


def send_frame_and_wait(
    ser: serial.Serial,
    reader: FrameReader,
    frame: bytes,
    label: str,
    timeout: float,
    retries: int,
    retry_delay_ms: int,
    accept_fn=None,
):
    last_error = None

    for attempt in range(1, retries + 1):
        try:
            if attempt == 1:
                try_reset_input_buffer(ser)
            ser.write(frame)
            ser.flush()

            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                remaining = deadline - time.monotonic()
                result = recv_frame(ser, reader, timeout=max(remaining, 0.01))
                if result is None:
                    break

                if is_unsolicited_stream_frame(result):
                    continue

                if accept_fn is None or accept_fn(result):
                    return result

                print(f"[WARN] ignored unexpected frame ({label}): msg=0x{result[0]:02X}")

            print(f"[RX] timeout ({label}, attempt {attempt}/{retries})")
        except (serial.SerialException, OSError, PermissionError) as e:
            last_error = e
            print(f"[WARN] serial error ({label}, attempt {attempt}/{retries}): {e}")

        if attempt < retries:
            sleep_ms(retry_delay_ms)

    if last_error is not None:
        raise serial.SerialException(last_error)

    return None


def cmd_ping(ser: serial.Serial, retries: int, retry_delay_ms: int):
    reader = FrameReader()
    seq = 1
    payload = struct.pack("<I", int(time.time() * 1000) & 0xFFFFFFFF)
    frame = build_frame(MSG_PING_REQ, seq, payload)

    print(f"[TX] PING_REQ seq={seq}")
    result = send_frame_and_wait(
        ser,
        reader,
        frame,
        "PING_REQ",
        2.0,
        retries,
        retry_delay_ms,
        accept_fn=lambda r: r[0] in (MSG_ACK, MSG_NACK),
    )
    if result is None:
        return False

    msg_type, _, resp_payload = result
    if msg_type == MSG_ACK and len(resp_payload) >= 2:
        req_msg = resp_payload[0]
        status = resp_payload[1]
        print(f"[RX] ACK req=0x{req_msg:02X} status={status}")
        return req_msg == MSG_PING_REQ and status == 0

    print(f"[RX] unexpected: msg=0x{msg_type:02X}")
    return False


def cmd_info(ser: serial.Serial, retries: int, retry_delay_ms: int):
    reader = FrameReader()
    seq = 2
    frame = build_frame(MSG_GET_DEVICE_INFO_REQ, seq, b"")

    print(f"[TX] GET_DEVICE_INFO_REQ seq={seq}")
    result = send_frame_and_wait(
        ser,
        reader,
        frame,
        "GET_DEVICE_INFO_REQ",
        2.0,
        retries,
        retry_delay_ms,
        accept_fn=lambda r: r[0] in (MSG_DEVICE_INFO_RSP, MSG_NACK),
    )
    if result is None:
        return False

    msg_type, _, payload = result

    if msg_type == MSG_NACK and len(payload) >= 2:
        print(f"[RX] NACK req=0x{payload[0]:02X} reason={payload[1]}")
        return False

    if msg_type != MSG_DEVICE_INFO_RSP or len(payload) != 9:
        print(f"[RX] unexpected: msg=0x{msg_type:02X} len={len(payload)}")
        return False

    device_type = payload[0]
    proto_ver = payload[1]
    fw_major = payload[2]
    fw_minor = payload[3]
    fw_patch = payload[4]
    cap_flags = struct.unpack_from("<H", payload, 5)[0]
    max_payload = struct.unpack_from("<H", payload, 7)[0]

    print("[RX] DEVICE_INFO_RSP")
    print(f"  device_type   = 0x{device_type:02X}")
    print(f"  proto_version = {proto_ver}")
    print(f"  fw            = {fw_major}.{fw_minor}.{fw_patch}")
    print(f"  cap_flags     = 0x{cap_flags:04X}")
    print(f"  max_payload   = {max_payload}")
    return True


def cmd_stream(ser: serial.Serial, rate: int, retries: int, retry_delay_ms: int):
    reader = FrameReader()

    start_payload = struct.pack("<BBHI", 1, 0, rate, 0xFFFFFFFF)
    start_frame = build_frame(MSG_STREAM_CONTROL_REQ, 3, start_payload)

    print(f"[TX] STREAM_CONTROL_REQ enable=1 rate={rate}Hz")
    result = send_frame_and_wait(
        ser,
        reader,
        start_frame,
        "STREAM_CONTROL_REQ start",
        2.0,
        retries,
        retry_delay_ms,
        accept_fn=lambda r: r[0] in (MSG_ACK, MSG_NACK),
    )
    if result is None:
        return False

    msg_type, _, resp_payload = result
    if msg_type == MSG_ACK and len(resp_payload) >= 2 and resp_payload[1] == 0:
        print(f"[STREAM] started rate={rate}Hz")
    elif msg_type == MSG_NACK and len(resp_payload) >= 2:
        print(f"[RX] NACK req=0x{resp_payload[0]:02X} reason={resp_payload[1]}")
        return False
    else:
        print(f"[RX] unexpected: msg=0x{msg_type:02X}")
        return False

    running = True

    def on_sigint(_sig, _frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, on_sigint)

    count = 0
    t_start = time.time()

    while running:
        result = recv_frame(ser, reader, timeout=1.0)
        if result is None:
            continue

        msg_type, _, payload = result
        if msg_type != MSG_STATUS_STREAM:
            continue

        try:
            host_rx_time_ms = int(time.time() * 1000)
            frame = parse_status_stream(payload, host_rx_time_ms)
        except ValueError as e:
            print(f"[WARN] stream parse error: {e}")
            continue

        tick_ms = frame.tick_ms
        pitch_target = frame.pitch_target_deg
        speed_p_term = frame.speed_p_term_deg
        speed_i_term = frame.speed_i_term_deg
        pitch_meas = frame.pitch_meas_deg
        pitch_rate = frame.pitch_rate_dps
        speed_target = frame.speed_target_radps
        speed_meas = frame.speed_meas_radps
        speed_raw = frame.speed_raw_radps
        attitude_p_term = frame.attitude_p_iq_cmd_a
        attitude_d_term = frame.attitude_d_iq_cmd_a
        iq_cmd = frame.iq_cmd_a
        iq_cmd_clamped = frame.iq_cmd_clamped_a
        speed_output_limit = frame.speed_output_limit_deg
        attitude_output_limit = frame.attitude_output_limit_a
        iq_l = frame.iq_l_a
        iq_r = frame.iq_r_a
        uq_l = frame.uq_l_v
        uq_r = frame.uq_r_v
        bus_v = frame.bus_v
        fault = frame.fault_flags
        fault_text = frame.fault_labels

        print(
            f"t={tick_ms:>8} pitch_target={pitch_target:>+6.2f}deg "
            f"speed_p_term={speed_p_term:>+6.2f}deg "
            f"speed_i_term={speed_i_term:>+6.2f}deg "
            f"pitch_meas={pitch_meas:>+6.2f}deg pitch_rate={pitch_rate:>+7.2f}dps "
            f"speed_target={speed_target:>+6.3f} speed_meas={speed_meas:>+6.3f} "
            f"speed_raw={speed_raw:>+6.3f} "
            f"attitude_p_term={attitude_p_term:>+5.3f} "
            f"attitude_d_term={attitude_d_term:>+5.3f} "
            f"iq_cmd={iq_cmd:>+5.3f} iq_cmd_clamped={iq_cmd_clamped:>+5.3f} "
            f"speed_output_limit={speed_output_limit:>+6.2f}deg "
            f"attitude_output_limit={attitude_output_limit:>+5.3f} "
            f"iq_l={iq_l:>+5.3f} iq_r={iq_r:>+5.3f} "
            f"uq_l={uq_l:>5.2f}V uq_r={uq_r:>5.2f}V "
            f"bus={bus_v:>6.2f}V fault=0x{fault:04X} [{fault_text}]"
        )
        count += 1

    elapsed = time.time() - t_start
    print(
        f"\n[STREAM] stopped. {count} frames in {elapsed:.1f}s "
        f"({count / elapsed if elapsed > 0 else 0:.1f} Hz)"
    )

    stop_payload = struct.pack("<BBHI", 0, 0, rate, 0)
    stop_frame = build_frame(MSG_STREAM_CONTROL_REQ, 4, stop_payload)
    result = send_frame_and_wait(
        ser,
        reader,
        stop_frame,
        "STREAM_CONTROL_REQ stop",
        1.0,
        retries,
        retry_delay_ms,
        accept_fn=lambda r: r[0] in (MSG_ACK, MSG_NACK),
    )
    if result and result[0] == MSG_ACK:
        print("[TX] STREAM_CONTROL_REQ enable=0 -> ACK")
    else:
        print("[TX] STREAM_CONTROL_REQ enable=0 -> no ACK")

    return True


def cmd_driver(ser: serial.Serial, enable: bool, retries: int, retry_delay_ms: int):
    reader = FrameReader()
    seq = 5
    payload = struct.pack("<B", 1 if enable else 0)
    frame = build_frame(MSG_POWER_STAGE_REQ, seq, payload)
    action = "on" if enable else "off"

    print(f"[TX] POWER_STAGE_REQ state={action}")
    result = send_frame_and_wait(
        ser,
        reader,
        frame,
        "POWER_STAGE_REQ",
        2.0,
        retries,
        retry_delay_ms,
        accept_fn=lambda r: r[0] in (MSG_ACK, MSG_NACK),
    )
    if result is None:
        return False

    msg_type, _, resp_payload = result
    if msg_type == MSG_ACK and len(resp_payload) >= 2:
        req_msg = resp_payload[0]
        status = resp_payload[1]
        print(f"[RX] ACK req=0x{req_msg:02X} status={status}")
        return req_msg == MSG_POWER_STAGE_REQ and status == 0

    if msg_type == MSG_NACK and len(resp_payload) >= 2:
        print(f"[RX] NACK req=0x{resp_payload[0]:02X} reason={resp_payload[1]}")
        return False

    print(f"[RX] unexpected: msg=0x{msg_type:02X}")
    return False


def cmd_balance(ser: serial.Serial, enable: bool, retries: int, retry_delay_ms: int):
    reader = FrameReader()
    seq = 6
    payload = struct.pack("<B", 1 if enable else 0)
    frame = build_frame(MSG_ATTITUDE_CONTROL_REQ, seq, payload)
    action = "on" if enable else "off"

    print(f"[TX] ATTITUDE_CONTROL_REQ state={action}")
    result = send_frame_and_wait(
        ser,
        reader,
        frame,
        "ATTITUDE_CONTROL_REQ",
        2.0,
        retries,
        retry_delay_ms,
        accept_fn=lambda r: r[0] in (MSG_ACK, MSG_NACK),
    )
    if result is None:
        return False

    msg_type, _, resp_payload = result
    if msg_type == MSG_ACK and len(resp_payload) >= 2:
        req_msg = resp_payload[0]
        status = resp_payload[1]
        print(f"[RX] ACK req=0x{req_msg:02X} status={status}")
        return req_msg == MSG_ATTITUDE_CONTROL_REQ and status == 0

    if msg_type == MSG_NACK and len(resp_payload) >= 2:
        print(f"[RX] NACK req=0x{resp_payload[0]:02X} reason={resp_payload[1]}")
        return False

    print(f"[RX] unexpected: msg=0x{msg_type:02X}")
    return False


def cmd_fastring_status(ser: serial.Serial, retries: int, retry_delay_ms: int):
    reader = FrameReader()
    seq = 30
    payload = struct.pack("<B", OP_FASTRING_STATUS)
    frame = build_frame(MSG_FAST_RING_REQ, seq, payload)

    print("[TX] FAST_RING_REQ STATUS")
    result = send_frame_and_wait(
        ser, reader, frame, "FASTRING_STATUS",
        2.0, retries, retry_delay_ms,
        accept_fn=lambda r: r[0] == MSG_FAST_RING_DATA,
    )
    if result is None:
        return None

    _, _, pld = result
    try:
        meta = parse_fastring_status(pld)
    except ValueError as e:
        print(f"[WARN] fastring status parse error: {e}")
        return None

    print("[RX] FAST_RING_DATA STATUS")
    print(f"  total_count  = {meta.total_count}")
    print(f"  capacity     = {meta.capacity}")
    print(f"  head         = {meta.head}")
    print(f"  write_seq    = {meta.write_seq}")
    return meta


def cmd_fastring_dump(
    ser: serial.Serial,
    out_file: str,
    retries: int,
    retry_delay_ms: int,
):
    snapshot_meta, all_samples = collect_fastring_snapshot(ser, retries, retry_delay_ms)
    if snapshot_meta is None:
        return False
    try:
        write_fastring_dual_csv(all_samples, out_file)
        print(f"[DUMP] wrote {len(all_samples)} dual samples to {out_file}")
        return True
    except OSError as e:
        print(f"[ERROR] cannot write {out_file}: {e}")
        return False


def cmd_fastring_split(
    ser: serial.Serial,
    left_out: str,
    right_out: str,
    retries: int,
    retry_delay_ms: int,
):
    snapshot_meta, all_samples = collect_fastring_snapshot(ser, retries, retry_delay_ms)
    if snapshot_meta is None:
        return False

    print(
        f"[DUMP] synchronized FastRing snapshot write_seq={snapshot_meta.write_seq} "
        f"samples={len(all_samples)}"
    )
    try:
        write_fastring_side_csv(all_samples, FASTCAP_SOURCE_LEFT, snapshot_meta.write_seq, left_out)
        write_fastring_side_csv(all_samples, FASTCAP_SOURCE_RIGHT, snapshot_meta.write_seq, right_out)
        print(f"[DUMP] wrote {len(all_samples)} left samples to {left_out}")
        print(f"[DUMP] wrote {len(all_samples)} right samples to {right_out}")
        return True
    except OSError as e:
        print(f"[ERROR] cannot write synchronized split CSVs: {e}")
        return False


def cmd_fastring_side(
    ser: serial.Serial,
    side: str,
    out_file: str,
    retries: int,
    retry_delay_ms: int,
):
    requested_source = FASTCAP_SOURCE_LEFT if side.upper() == "L" else FASTCAP_SOURCE_RIGHT
    side_label = "R" if requested_source == FASTCAP_SOURCE_RIGHT else "L"
    snapshot_meta, all_samples = collect_fastring_snapshot(ser, retries, retry_delay_ms)
    if snapshot_meta is None:
        return False

    print(
        f"[DUMP] FastRing snapshot write_seq={snapshot_meta.write_seq} "
        f"motor={side_label} samples={len(all_samples)}"
    )
    try:
        write_fastring_side_csv(all_samples, requested_source, snapshot_meta.write_seq, out_file)
        print(f"[DUMP] wrote {len(all_samples)} samples to {out_file}")
        return True
    except OSError as e:
        print(f"[ERROR] cannot write {out_file}: {e}")
        return False


def collect_fastring_snapshot(
    ser: serial.Serial,
    retries: int,
    retry_delay_ms: int,
):
    reader = FrameReader()
    seq_base = 31
    live_meta = cmd_fastring_status(ser, retries, retry_delay_ms)
    if live_meta is None:
        return None, None
    if live_meta.total_count == 0:
        print("[WARN] FastRing is empty")
        return None, None

    payload = struct.pack("<B", OP_FASTRING_SNAPSHOT)
    frame = build_frame(MSG_FAST_RING_REQ, seq_base & 0xFF, payload)
    seq_base += 1

    print("[TX] FAST_RING_REQ SNAPSHOT")
    result = send_frame_and_wait(
        ser, reader, frame, "FASTRING_SNAPSHOT",
        2.0, retries, retry_delay_ms,
        accept_fn=lambda r: r[0] == MSG_FAST_RING_DATA,
    )
    if result is None:
        return None, None

    _, _, pld = result
    try:
        meta = parse_fastring_status(pld)
    except ValueError as e:
        print(f"[WARN] fastring snapshot parse error: {e}")
        return None, None

    print(f"[DUMP] total_count={meta.total_count} capacity={meta.capacity} write_seq={meta.write_seq}")
    all_samples = []
    start_idx = 0
    while start_idx < meta.total_count:
        remaining = meta.total_count - start_idx
        max_req = min(FASTRING_CHUNK_MAX, remaining)
        payload = struct.pack("<BIHB", OP_FASTRING_READ_CHUNK, meta.write_seq, start_idx, max_req)
        frame = build_frame(MSG_FAST_RING_REQ, seq_base & 0xFF, payload)
        seq_base += 1

        result = send_frame_and_wait(
            ser, reader, frame, f"FASTRING_READ_CHUNK start={start_idx}",
            2.0, retries, retry_delay_ms,
            accept_fn=lambda r: r[0] == MSG_FAST_RING_DATA,
        )
        if result is None:
            print(f"[WARN] FASTRING_READ_CHUNK start={start_idx} timed out, retrying ...")
            continue

        _, _, pld = result
        try:
            chunk_meta, chunk_samples = parse_fastring_chunk(pld)
        except ValueError as e:
            print(f"[WARN] fastring chunk parse error: {e}")
            continue

        if chunk_meta.op_echo != OP_FASTRING_READ_CHUNK:
            print(f"[WARN] unexpected op_echo=0x{chunk_meta.op_echo:02X}, skipping")
            continue
        if chunk_meta.total_count != meta.total_count:
            print(f"[WARN] total_count mismatch: got {chunk_meta.total_count}, expected {meta.total_count}, skipping")
            continue
        if chunk_meta.write_seq != meta.write_seq:
            print(f"[WARN] write_seq changed: got {chunk_meta.write_seq}, expected {meta.write_seq}, skipping")
            continue
        if chunk_samples and chunk_samples[0].index != start_idx:
            print(f"[WARN] start_idx mismatch: got {chunk_samples[0].index}, expected {start_idx}, skipping")
            continue

        sample_count = len(chunk_samples)
        if sample_count == 0:
            break

        all_samples.extend(chunk_samples)
        start_idx += sample_count

    return meta, all_samples


def write_fastring_dual_csv(all_samples, out_file: str) -> None:
    with open(out_file, "w", newline="") as f:
        f.write(
            "idx,target_iq_l,iq_ref_l,filtered_iq_l,raw_iq_l,uq_final_l,"
            "target_iq_r,iq_ref_r,filtered_iq_r,raw_iq_r,uq_final_r,"
            "bus_v,sample_idx,status_flags\n"
        )
        for s in all_samples:
            f.write(
                f"{s.index},{s.target_iq_l_a:.6f},{s.iq_ref_l_a:.6f},{s.filtered_iq_l_a:.6f},"
                f"{s.raw_iq_l_a:.6f},{s.uq_final_l_v:.6f},{s.target_iq_r_a:.6f},{s.iq_ref_r_a:.6f},"
                f"{s.filtered_iq_r_a:.6f},{s.raw_iq_r_a:.6f},{s.uq_final_r_v:.6f},{s.bus_v:.6f},"
                f"{s.sample_idx},{s.status_flags}\n"
            )


def write_fastring_side_csv(all_samples, source: int, snapshot_write_seq: int, out_file: str) -> None:
    side_label = "R" if source == FASTCAP_SOURCE_RIGHT else "L"
    with open(out_file, "w", newline="") as f:
        f.write("idx,target_iq,iq_ref,filtered_iq,raw_iq,uq_final,source,capture_id\n")
        for idx, s in enumerate(all_samples):
            if source == FASTCAP_SOURCE_RIGHT:
                target_iq = s.target_iq_r_a
                iq_ref = s.iq_ref_r_a
                filtered_iq = s.filtered_iq_r_a
                raw_iq = s.raw_iq_r_a
                uq_final = s.uq_final_r_v
            else:
                target_iq = s.target_iq_l_a
                iq_ref = s.iq_ref_l_a
                filtered_iq = s.filtered_iq_l_a
                raw_iq = s.raw_iq_l_a
                uq_final = s.uq_final_l_v
            f.write(
                f"{idx},{target_iq:.6f},{iq_ref:.6f},{filtered_iq:.6f},"
                f"{raw_iq:.6f},{uq_final:.6f},{side_label},{snapshot_write_seq}\n"
            )


# ── Parameter helpers ────────────────────────────────────────────────

def _resolve_param(spec: str):
    """Resolve parameter name or hex ID string to (param_id, type, scale, name)."""
    if spec in PARAM_TABLE:
        pid, ptype, scale, _desc = PARAM_TABLE[spec]
        return pid, ptype, scale, spec
    try:
        pid = int(spec, 0)
    except ValueError:
        pass
    else:
        entry = PARAM_BY_ID.get(pid)
        if entry is not None:
            name, ptype, scale, _desc = entry
            return pid, ptype, scale, name
    raise ValueError(
        f"unknown parameter: {spec!r} — use 'param list' to see valid names and IDs"
    )


# ── Parameter commands ───────────────────────────────────────────────

def cmd_param_list():
    """Print the local parameter table (no device communication)."""
    print(f"{'ID':6s} {'NAME':27s} {'TYPE':6s}  DESCRIPTION")
    print("-" * 75)
    for name, (pid, ptype, _scale, desc) in sorted(PARAM_TABLE.items(),
                                                     key=lambda kv: kv[1][0]):
        print(f"0x{pid:02X}  {name:27s} {ptype:6s}  {desc}")
    return True


def cmd_param_get(ser: serial.Serial, param_spec: str,
                  retries: int, retry_delay_ms: int):
    """Send GET_PARAM_REQ and print the PARAM_VALUE_RSP."""
    try:
        pid, ptype, scale, name = _resolve_param(param_spec)
    except ValueError as e:
        print(f"[ERROR] {e}")
        return False

    reader = FrameReader()
    seq = 0x20
    frame = build_frame(MSG_GET_PARAM_REQ, seq, bytes([pid]))

    print(f"[TX] GET_PARAM_REQ {name} (0x{pid:02X})")
    result = send_frame_and_wait(
        ser, reader, frame, "GET_PARAM_REQ",
        2.0, retries, retry_delay_ms,
        accept_fn=lambda r: r[0] in (MSG_PARAM_VALUE_RSP, MSG_NACK),
    )
    if result is None:
        return False

    msg_type, _, pld = result
    if msg_type == MSG_NACK:
        reason = pld[1] if len(pld) >= 2 else "?"
        print(f"[RX] NACK req=0x{pld[0]:02X} reason={reason}")
        return False

    if len(pld) < 6:
        print(f"[WARN] short PARAM_VALUE_RSP: {len(pld)} bytes")
        return False

    resp_id = pld[0]
    resp_type = pld[1]
    raw_bytes = pld[2:6]

    if resp_type == PARAM_TYPE_FLOAT:
        raw_val = struct.unpack_from("<f", raw_bytes, 0)[0]
        scaled = raw_val * scale
        raw_disp = f"0x{raw_bytes.hex()}"
        val_disp = f"{scaled:.6f}"
    elif resp_type == PARAM_TYPE_UINT8:
        raw_val = raw_bytes[0]
        scaled = raw_val * scale
        raw_disp = str(int(raw_val))
        val_disp = str(int(scaled))
    else:
        print(f"[WARN] unknown data_type=0x{resp_type:02X} in response")
        return False

    resp_name = PARAM_BY_ID.get(resp_id, (f"0x{resp_id:02X}",))[0]
    print(f"[RX] PARAM_VALUE_RSP id=0x{resp_id:02X} name={resp_name} "
          f"raw={raw_disp} value={val_disp}")
    return True


def cmd_param_set(ser: serial.Serial, param_spec: str, value_str: str,
                  retries: int, retry_delay_ms: int):
    """Send SET_PARAM_REQ and wait for ACK/NACK."""
    try:
        pid, ptype, scale, name = _resolve_param(param_spec)
    except ValueError as e:
        print(f"[ERROR] {e}")
        return False

    if ptype == "float":
        user_val = float(value_str)
        raw_val = user_val / scale
        payload = struct.pack("<BBf", pid, PARAM_TYPE_FLOAT, raw_val)
    else:  # uint8
        user_val = float(value_str)
        raw_val = int(user_val / scale)
        if raw_val < 0 or raw_val > 255:
            print(f"[ERROR] uint8 value out of range: {raw_val} (from {value_str!r})")
            return False
        payload = struct.pack("<BBB3x", pid, PARAM_TYPE_UINT8, raw_val)

    reader = FrameReader()
    seq = 0x21
    frame = build_frame(MSG_SET_PARAM_REQ, seq, payload)

    print(f"[TX] SET_PARAM_REQ {name} (0x{pid:02X}) = {value_str}")
    result = send_frame_and_wait(
        ser, reader, frame, "SET_PARAM_REQ",
        2.0, retries, retry_delay_ms,
        accept_fn=lambda r: r[0] in (MSG_ACK, MSG_NACK),
    )
    if result is None:
        return False

    msg_type, _, pld = result
    if msg_type == MSG_ACK and len(pld) >= 2:
        print(f"[RX] ACK req=0x{pld[0]:02X} status={pld[1]}  ({name} updated)")
        return True

    if msg_type == MSG_NACK and len(pld) >= 2:
        print(f"[RX] NACK req=0x{pld[0]:02X} reason={pld[1]}")
        return False

    print(f"[RX] unexpected: msg=0x{msg_type:02X}")
    return False


def cmd_param_dump(ser: serial.Serial, retries: int, retry_delay_ms: int):
    """Iterate all known parameters, GET each one, and print a table."""
    reader = FrameReader()
    ok_all = True

    print(f"{'NAME':27s} {'VALUE':16s}")
    print("-" * 45)

    for name, (pid, ptype, scale, _desc) in sorted(PARAM_TABLE.items(),
                                                     key=lambda kv: kv[1][0]):
        seq = (0x30 + pid) & 0xFF
        frame = build_frame(MSG_GET_PARAM_REQ, seq, bytes([pid]))

        result = send_frame_and_wait(
            ser, reader, frame, f"GET_PARAM {name}",
            1.0, retries, retry_delay_ms,
            accept_fn=lambda r: r[0] in (MSG_PARAM_VALUE_RSP, MSG_NACK),
        )
        if result is None:
            print(f"  {name:27s} [timeout]")
            ok_all = False
            continue

        msg_type, _, pld = result
        if msg_type == MSG_NACK:
            reason = pld[1] if len(pld) >= 2 else "?"
            print(f"  {name:27s} [NACK reason={reason}]")
            ok_all = False
            continue

        if len(pld) < 6:
            print(f"  {name:27s} [short payload: {len(pld)}B]")
            ok_all = False
            continue

        if ptype == "float":
            raw_val = struct.unpack_from("<f", pld, 2)[0]
            print(f"  {name:27s} = {raw_val * scale:.6f}")
        else:
            raw_val = pld[2]
            print(f"  {name:27s} = {int(raw_val * scale)}")

    return ok_all


def main():
    parser = argparse.ArgumentParser(description="DebugLink CLI")
    parser.add_argument("--port", required=True, help="Serial port (e.g. COM33)")
    parser.add_argument("--baud", type=int, default=921600, help="Baud rate")
    parser.add_argument(
        "--open-retries",
        type=int,
        default=DEFAULT_OPEN_RETRIES,
        help="Retry count when opening the serial port",
    )
    parser.add_argument(
        "--retries",
        type=int,
        default=DEFAULT_CMD_RETRIES,
        help="Retry count for request/response commands",
    )
    parser.add_argument(
        "--settle-ms",
        type=int,
        default=DEFAULT_SETTLE_MS,
        help="Delay after open before first TX",
    )
    parser.add_argument(
        "--retry-delay-ms",
        type=int,
        default=DEFAULT_RETRY_DELAY_MS,
        help="Delay between command retries",
    )
    sub = parser.add_subparsers(dest="command")

    sub.add_parser("ping", help="Send PING_REQ")
    sub.add_parser("info", help="Get device info")
    p_driver = sub.add_parser("driver", help="Enable or disable the power stage")
    p_driver.add_argument("state", choices=("on", "off"), help="Target driver state")
    p_balance = sub.add_parser("balance", help="Enable or disable the attitude/speed control loop")
    p_balance.add_argument("state", choices=("on", "off"), help="Target balance state")

    p_stream = sub.add_parser("stream", help="Start status stream")
    p_stream.add_argument("--rate", type=int, default=100, help="Stream rate Hz")

    p_fastring = sub.add_parser("fastring", help="Continuous dual-motor ring buffer commands")
    fastring_sub = p_fastring.add_subparsers(dest="fastring_op")
    fastring_sub.add_parser("status", help="Query FastRing status")
    p_fr_dump = fastring_sub.add_parser("dump", help="Read latest FastRing window and write CSV")
    p_fr_dump.add_argument("--out", required=True, help="Output CSV file path")
    p_fr_side = fastring_sub.add_parser("side", help="Write one motor side from a FastRing snapshot")
    p_fr_side.add_argument("--side", required=True, choices=("L", "R"), help="Motor side")
    p_fr_side.add_argument("--out", required=True, help="Output CSV file path")
    p_fr_split = fastring_sub.add_parser(
        "split",
        help="Read one FastRing snapshot and write synchronized left/right CSV files",
    )
    p_fr_split.add_argument("--left-out", required=True, help="Left motor output CSV path")
    p_fr_split.add_argument("--right-out", required=True, help="Right motor output CSV path")

    # ── param ──
    p_param = sub.add_parser("param", help="Read/write runtime parameters")
    param_sub = p_param.add_subparsers(dest="param_op")
    param_sub.add_parser("list", help="List known parameters (local table, no device I/O)")
    p_pget = param_sub.add_parser("get", help="Read one parameter value from device")
    p_pget.add_argument("param", help="Parameter name or hex ID  (e.g. current_kp or 0x01)")
    p_pset = param_sub.add_parser("set", help="Write one parameter value on device")
    p_pset.add_argument("param", help="Parameter name or hex ID")
    p_pset.add_argument("value", help="New value (float for float params, int for uint8)")
    param_sub.add_parser("dump", help="Read all known parameters from device and print table")

    args = parser.parse_args()

    if args.command is None:
        parser.print_help()
        sys.exit(1)

    # Commands that do not require a serial connection
    if args.command == "param" and args.param_op == "list":
        cmd_param_list()
        sys.exit(0)

    ser = None
    ok = False
    try:
        ser = open_serial_port(args.port, args.baud, args.settle_ms, args.open_retries)
        if args.command == "ping":
            ok = cmd_ping(ser, args.retries, args.retry_delay_ms)
        elif args.command == "info":
            ok = cmd_info(ser, args.retries, args.retry_delay_ms)
        elif args.command == "driver":
            ok = cmd_driver(ser, args.state == "on", args.retries, args.retry_delay_ms)
        elif args.command == "balance":
            ok = cmd_balance(ser, args.state == "on", args.retries, args.retry_delay_ms)
        elif args.command == "stream":
            ok = cmd_stream(ser, args.rate, args.retries, args.retry_delay_ms)
        elif args.command == "fastring":
            if args.fastring_op is None:
                p_fastring.print_help()
                sys.exit(1)
            if args.fastring_op == "status":
                ok = cmd_fastring_status(ser, args.retries, args.retry_delay_ms) is not None
            elif args.fastring_op == "dump":
                ok = cmd_fastring_dump(ser, args.out, args.retries, args.retry_delay_ms)
            elif args.fastring_op == "side":
                ok = cmd_fastring_side(
                    ser, args.side, args.out, args.retries, args.retry_delay_ms
                )
            elif args.fastring_op == "split":
                ok = cmd_fastring_split(
                    ser,
                    args.left_out,
                    args.right_out,
                    args.retries,
                    args.retry_delay_ms,
                )
        elif args.command == "param":
            if args.param_op is None:
                p_param.print_help()
                sys.exit(1)
            if args.param_op == "list":
                ok = cmd_param_list()
            elif args.param_op == "get":
                ok = cmd_param_get(ser, args.param, args.retries, args.retry_delay_ms)
            elif args.param_op == "set":
                ok = cmd_param_set(ser, args.param, args.value,
                                   args.retries, args.retry_delay_ms)
            elif args.param_op == "dump":
                ok = cmd_param_dump(ser, args.retries, args.retry_delay_ms)
    except serial.SerialException as e:
        print(f"[ERROR] {e}")
    finally:
        if ser is not None:
            ser.close()

    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
