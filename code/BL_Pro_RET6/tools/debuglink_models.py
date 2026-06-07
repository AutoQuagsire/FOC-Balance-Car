"""
DebugLink data models — strongly-typed dataclasses for stream and fastring.

Wire-format scale: deg (x100), radps (x1000), A (x1000 or /1000), V (x1000).
All float fields are after scaling has been applied by the parser.
"""

from dataclasses import dataclass, field


@dataclass
class LiveFrame:
    """One decoded STATUS_STREAM (0x90) telemetry frame."""

    tick_ms: int
    pitch_target_deg: float
    speed_p_term_deg: float
    speed_i_term_deg: float
    pitch_meas_deg: float
    pitch_rate_dps: float
    speed_target_radps: float
    speed_meas_radps: float
    speed_raw_radps: float
    attitude_p_iq_cmd_a: float
    attitude_d_iq_cmd_a: float
    iq_cmd_a: float
    iq_cmd_clamped_a: float
    speed_output_limit_deg: float
    attitude_output_limit_a: float
    iq_l_a: float
    iq_r_a: float
    uq_l_v: float
    uq_r_v: float
    bus_v: float
    fault_flags: int
    fault_labels: str
    host_rx_time_ms: int


@dataclass
class FastRingMeta:
    """Metadata for the dual-motor continuous ring buffer."""

    op_echo: int
    total_count: int
    capacity: int
    head: int
    write_seq: int


@dataclass
class FastRingSample:
    """One dual-motor sample from the continuous FastRing buffer."""

    index: int
    target_iq_l_a: float
    iq_ref_l_a: float
    filtered_iq_l_a: float
    raw_iq_l_a: float
    uq_final_l_v: float
    target_iq_r_a: float
    iq_ref_r_a: float
    filtered_iq_r_a: float
    raw_iq_r_a: float
    uq_final_r_v: float
    bus_v: float
    sample_idx: int
    status_flags: int
