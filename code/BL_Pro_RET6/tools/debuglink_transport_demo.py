#!/usr/bin/env python3
"""
DebugLinkTransport smoke test for ping, get_info, and stream.

Usage::

    python debuglink_transport_demo.py --port COM33
    python debuglink_transport_demo.py --port COM33 --baud 921600 --rate 100
"""

import argparse
import sys
import threading

from debuglink_transport import DebugLinkTransport, TransportError


def main():
    parser = argparse.ArgumentParser(description="DebugLinkTransport demo")
    parser.add_argument("--port", required=True, help="Serial port, e.g. COM33")
    parser.add_argument("--baud", type=int, default=921600, help="Baud rate")
    parser.add_argument("--rate", type=int, default=100, help="Stream rate Hz")
    args = parser.parse_args()

    t = DebugLinkTransport()

    try:
        t.connect(args.port, args.baud)
        print(f"[DEMO] connected to {args.port} @ {args.baud}")
    except TransportError as e:
        print(f"[FATAL] {e}")
        sys.exit(1)

    ok = True

    try:
        ping_ok = t.ping()
        print(f"[DEMO] ping -> {'OK' if ping_ok else 'FAIL'}")
        if not ping_ok:
            ok = False

        try:
            info = t.get_info()
            print("[DEMO] device info:")
            print(f"  device_type   = 0x{info['device_type']:02X}")
            print(f"  proto_version = {info['proto_version']}")
            print(f"  fw            = {info['fw_major']}.{info['fw_minor']}.{info['fw_patch']}")
            print(f"  cap_flags     = 0x{info['cap_flags']:04X}")
            print(f"  max_payload   = {info['max_payload']}")
        except TransportError as e:
            print(f"[DEMO] get_info failed: {e}")
            ok = False

        first_frame_event = threading.Event()
        first_frame = None

        def on_frame(frame):
            nonlocal first_frame
            if first_frame is None:
                first_frame = frame
                first_frame_event.set()

        t.set_stream_callback(on_frame)

        stream_ok = t.stream_start(rate_hz=args.rate)
        print(f"[DEMO] stream_start(rate={args.rate}) -> {'OK' if stream_ok else 'FAIL'}")

        if stream_ok:
            got_frame = first_frame_event.wait(timeout=3.0)
            if got_frame and first_frame is not None:
                f = first_frame
                fault_hex = f"0x{f.fault_flags:04X}"
                print("[DEMO] first stream frame:")
                print(f"  tick_ms             = {f.tick_ms}")
                print(f"  pitch_target_deg    = {f.pitch_target_deg:+.2f}")
                print(f"  speed_p_term_deg    = {f.speed_p_term_deg:+.2f}")
                print(f"  speed_i_term_deg    = {f.speed_i_term_deg:+.2f}")
                print(f"  pitch_meas_deg      = {f.pitch_meas_deg:+.2f}")
                print(f"  pitch_rate_dps      = {f.pitch_rate_dps:+.2f}")
                print(f"  speed_target_radps  = {f.speed_target_radps:+.3f}")
                print(f"  speed_meas_radps    = {f.speed_meas_radps:+.3f}")
                print(f"  speed_raw_radps     = {f.speed_raw_radps:+.3f}")
                print(f"  attitude_p_iq_cmd_a = {f.attitude_p_iq_cmd_a:+.3f}")
                print(f"  attitude_d_iq_cmd_a = {f.attitude_d_iq_cmd_a:+.3f}")
                print(f"  iq_cmd_a            = {f.iq_cmd_a:+.3f}")
                print(f"  iq_cmd_clamped_a    = {f.iq_cmd_clamped_a:+.3f}")
                print(f"  speed_output_limit  = {f.speed_output_limit_deg:+.2f} deg")
                print(f"  attitude_out_limit  = {f.attitude_output_limit_a:+.3f}")
                print(f"  iq_l_a / iq_r_a     = {f.iq_l_a:+.3f} / {f.iq_r_a:+.3f}")
                print(f"  uq_l_v / uq_r_v     = {f.uq_l_v:.2f} / {f.uq_r_v:.2f}")
                print(f"  bus_v               = {f.bus_v:.2f}")
                print(f"  fault_flags         = {fault_hex} [{f.fault_labels}]")
            else:
                print("[DEMO] stream: no frame received within 3 s")
                ok = False

            stop_ok = t.stream_stop()
            print(f"[DEMO] stream_stop() -> {'OK' if stop_ok else 'FAIL (or already stopped)'}")

    finally:
        t.set_stream_callback(None)
        t.disconnect()
        print("[DEMO] disconnected")

    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
