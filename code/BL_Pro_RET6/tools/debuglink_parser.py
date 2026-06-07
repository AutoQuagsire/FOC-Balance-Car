"""
DebugLink protocol parser.

This module converts wire payloads into strongly-typed model objects.
All layouts match the firmware in MyCode/System/debug_link.c.
"""

import struct

from debuglink_models import (
    FastRingMeta,
    FastRingSample,
    LiveFrame,
)

_FOC_FLAG_SPEED_FAULT_L = 1 << 0
_FOC_FLAG_SPEED_FAULT_R = 1 << 1
_FOC_FLAG_STACK_READY = 1 << 8
_FOC_FLAG_CONTROL_IT_ENABLED = 1 << 9
_FOC_FLAG_BUS_VALID = 1 << 10
_FOC_FLAG_CURRENT_LOOP_ACTIVE = 1 << 11
_FOC_FLAG_SPEED_LOOP_ENABLED = 1 << 12
_FOC_FLAG_CURRENT_LOOP_ENABLED = 1 << 13
_FOC_FLAG_POWER_STAGE_OFF = 1 << 14
_FOC_FLAG_ATTITUDE_CONTROL_ON = 1 << 15

_FLAG_TABLE = [
    (_FOC_FLAG_SPEED_FAULT_L, "spdfltL"),
    (_FOC_FLAG_SPEED_FAULT_R, "spdfltR"),
    (_FOC_FLAG_STACK_READY, "stack"),
    (_FOC_FLAG_CONTROL_IT_ENABLED, "it"),
    (_FOC_FLAG_BUS_VALID, "bus"),
    (_FOC_FLAG_CURRENT_LOOP_ACTIVE, "iloop"),
    (_FOC_FLAG_SPEED_LOOP_ENABLED, "sloop"),
    (_FOC_FLAG_CURRENT_LOOP_ENABLED, "cloop"),
    (_FOC_FLAG_POWER_STAGE_OFF, "drv_off"),
    (_FOC_FLAG_ATTITUDE_CONTROL_ON, "bal"),
]

_STREAM_PAYLOAD_SIZE_MIN = 42
_STREAM_PAYLOAD_SIZE_WITH_RAW_SPEED = 44
_FASTRING_STATUS_SIZE = 11
_FASTRING_CHUNK_HEADER_SIZE = 12
_FASTRING_SAMPLE_SIZE = 26


def _decode_fault_labels(flags: int) -> str:
    labels = [label for mask, label in _FLAG_TABLE if flags & mask]
    return ",".join(labels) if labels else "-"


def source_to_label(v: int) -> str:
    if v == 0:
        return "L"
    if v == 1:
        return "R"
    raise ValueError(f"invalid source value: {v} (expected 0 or 1)")


def parse_status_stream(payload: bytes, host_rx_time_ms: int) -> LiveFrame:
    if len(payload) < _STREAM_PAYLOAD_SIZE_MIN:
        raise ValueError(
            f"STATUS_STREAM payload too short: need at least {_STREAM_PAYLOAD_SIZE_MIN} bytes, got {len(payload)}"
        )

    tick_ms = struct.unpack_from("<I", payload, 0)[0]
    pitch_target_deg = struct.unpack_from("<h", payload, 4)[0] / 100.0
    speed_p_term_deg = struct.unpack_from("<h", payload, 6)[0] / 100.0
    speed_i_term_deg = struct.unpack_from("<h", payload, 8)[0] / 100.0
    pitch_meas_deg = struct.unpack_from("<h", payload, 10)[0] / 100.0
    pitch_rate_dps = struct.unpack_from("<h", payload, 12)[0] / 100.0
    speed_target_radps = struct.unpack_from("<h", payload, 14)[0] / 100.0
    speed_meas_radps = struct.unpack_from("<h", payload, 16)[0] / 100.0
    attitude_p_iq_cmd_a = struct.unpack_from("<h", payload, 18)[0] / 1000.0
    attitude_d_iq_cmd_a = struct.unpack_from("<h", payload, 20)[0] / 1000.0
    iq_cmd_a = struct.unpack_from("<h", payload, 22)[0] / 1000.0
    iq_cmd_clamped_a = struct.unpack_from("<h", payload, 24)[0] / 1000.0
    speed_output_limit_deg = struct.unpack_from("<h", payload, 26)[0] / 100.0
    attitude_output_limit_a = struct.unpack_from("<h", payload, 28)[0] / 1000.0
    iq_l_a = struct.unpack_from("<h", payload, 30)[0] / 1000.0
    iq_r_a = struct.unpack_from("<h", payload, 32)[0] / 1000.0
    uq_l_v = struct.unpack_from("<h", payload, 34)[0] / 1000.0
    uq_r_v = struct.unpack_from("<h", payload, 36)[0] / 1000.0
    bus_v = struct.unpack_from("<H", payload, 38)[0] / 1000.0
    fault_flags = struct.unpack_from("<H", payload, 40)[0]
    if len(payload) >= _STREAM_PAYLOAD_SIZE_WITH_RAW_SPEED:
        speed_raw_radps = struct.unpack_from("<h", payload, 42)[0] / 100.0
    else:
        speed_raw_radps = speed_meas_radps

    return LiveFrame(
        tick_ms=tick_ms,
        pitch_target_deg=pitch_target_deg,
        speed_p_term_deg=speed_p_term_deg,
        speed_i_term_deg=speed_i_term_deg,
        pitch_meas_deg=pitch_meas_deg,
        pitch_rate_dps=pitch_rate_dps,
        speed_target_radps=speed_target_radps,
        speed_meas_radps=speed_meas_radps,
        speed_raw_radps=speed_raw_radps,
        attitude_p_iq_cmd_a=attitude_p_iq_cmd_a,
        attitude_d_iq_cmd_a=attitude_d_iq_cmd_a,
        iq_cmd_a=iq_cmd_a,
        iq_cmd_clamped_a=iq_cmd_clamped_a,
        speed_output_limit_deg=speed_output_limit_deg,
        attitude_output_limit_a=attitude_output_limit_a,
        iq_l_a=iq_l_a,
        iq_r_a=iq_r_a,
        uq_l_v=uq_l_v,
        uq_r_v=uq_r_v,
        bus_v=bus_v,
        fault_flags=fault_flags,
        fault_labels=_decode_fault_labels(fault_flags),
        host_rx_time_ms=host_rx_time_ms,
    )


def parse_fastring_status(payload: bytes) -> FastRingMeta:
    if len(payload) < _FASTRING_STATUS_SIZE:
        raise ValueError(
            f"FASTRING_STATUS payload too short: need {_FASTRING_STATUS_SIZE} bytes, got {len(payload)}"
        )

    return FastRingMeta(
        op_echo=payload[0],
        total_count=struct.unpack_from("<H", payload, 1)[0],
        capacity=struct.unpack_from("<H", payload, 3)[0],
        head=struct.unpack_from("<H", payload, 5)[0],
        write_seq=struct.unpack_from("<I", payload, 7)[0],
    )


def parse_fastring_chunk(payload: bytes) -> tuple[FastRingMeta, list[FastRingSample]]:
    if len(payload) < _FASTRING_CHUNK_HEADER_SIZE:
        raise ValueError(
            f"FASTRING_CHUNK payload too short: need at least {_FASTRING_CHUNK_HEADER_SIZE} bytes, got {len(payload)}"
        )

    total_count = struct.unpack_from("<H", payload, 1)[0]
    capacity = struct.unpack_from("<H", payload, 3)[0]
    write_seq = struct.unpack_from("<I", payload, 5)[0]
    start_idx = struct.unpack_from("<H", payload, 9)[0]
    sample_count = payload[11]
    expected_len = _FASTRING_CHUNK_HEADER_SIZE + sample_count * _FASTRING_SAMPLE_SIZE
    if len(payload) != expected_len:
        raise ValueError(
            f"FASTRING_CHUNK payload length mismatch: expected {expected_len} bytes "
            f"for sample_count={sample_count}, got {len(payload)}"
        )

    samples: list[FastRingSample] = []
    offset = _FASTRING_CHUNK_HEADER_SIZE
    for i in range(sample_count):
        samples.append(
            FastRingSample(
                index=start_idx + i,
                target_iq_l_a=struct.unpack_from("<h", payload, offset + 0)[0] / 1000.0,
                iq_ref_l_a=struct.unpack_from("<h", payload, offset + 2)[0] / 1000.0,
                filtered_iq_l_a=struct.unpack_from("<h", payload, offset + 4)[0] / 1000.0,
                raw_iq_l_a=struct.unpack_from("<h", payload, offset + 6)[0] / 1000.0,
                uq_final_l_v=struct.unpack_from("<h", payload, offset + 8)[0] / 1000.0,
                target_iq_r_a=struct.unpack_from("<h", payload, offset + 10)[0] / 1000.0,
                iq_ref_r_a=struct.unpack_from("<h", payload, offset + 12)[0] / 1000.0,
                filtered_iq_r_a=struct.unpack_from("<h", payload, offset + 14)[0] / 1000.0,
                raw_iq_r_a=struct.unpack_from("<h", payload, offset + 16)[0] / 1000.0,
                uq_final_r_v=struct.unpack_from("<h", payload, offset + 18)[0] / 1000.0,
                bus_v=struct.unpack_from("<H", payload, offset + 20)[0] / 1000.0,
                sample_idx=struct.unpack_from("<H", payload, offset + 22)[0],
                status_flags=struct.unpack_from("<H", payload, offset + 24)[0],
            )
        )
        offset += _FASTRING_SAMPLE_SIZE

    return FastRingMeta(
        op_echo=payload[0],
        total_count=total_count,
        capacity=capacity,
        head=0,
        write_seq=write_seq,
    ), samples
