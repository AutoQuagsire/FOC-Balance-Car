"""
DebugLink GUI scaffold (PySide6).

By default this runs against real DebugLink serial stream. Use ``--mock`` for
local UI-only rendering.
"""

from __future__ import annotations

import math
import os
import random
import sys
import csv
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from debuglink_models import (
    FastRingMeta,
    FastRingSample,
    LiveFrame,
)
from debuglink_transport import DebugLinkTransport, TransportError


_DLL_DIR_HANDLES: list[Any] = []


def _prepare_windows_qt_dll_search_path() -> None:
    if os.name != "nt":
        return
    if not hasattr(os, "add_dll_directory"):
        return

    candidates: list[Path] = []
    meipass = getattr(sys, "_MEIPASS", None)
    if meipass:
        mp = Path(meipass)
        candidates.extend(
            [
                mp,
                mp / "PySide6",
                mp / "shiboken6",
                mp / "_internal",
                mp / "_internal" / "PySide6",
                mp / "_internal" / "shiboken6",
            ]
        )

    # Source-run fallback: ensure site-package bin dirs are visible.
    try:
        import PySide6  # type: ignore

        pyside_dir = Path(PySide6.__file__).resolve().parent
        candidates.extend([pyside_dir, pyside_dir / "plugins"])
    except Exception:
        pass

    seen: set[str] = set()
    for p in candidates:
        pstr = str(p)
        if pstr in seen:
            continue
        seen.add(pstr)
        if p.exists():
            try:
                _DLL_DIR_HANDLES.append(os.add_dll_directory(pstr))
            except Exception:
                pass


_prepare_windows_qt_dll_search_path()

try:
    from PySide6.QtCore import QObject, QTimer, Signal
    from PySide6.QtWidgets import (
        QApplication,
        QCheckBox,
        QComboBox,
        QDoubleSpinBox,
        QFormLayout,
        QGridLayout,
        QGroupBox,
        QHBoxLayout,
        QLabel,
        QMainWindow,
        QPushButton,
        QSpinBox,
        QTabWidget,
        QTextEdit,
        QVBoxLayout,
        QWidget,
    )
except ImportError as exc:
    print("PySide6 is required. Install with: pip install PySide6")
    print(f"Import error: {exc}")
    sys.exit(1)


@dataclass
class LiveViewState:
    last_frame: LiveFrame | None = None
    paused: bool = False
    history_limit: int = 2000
    recording: bool = False
    record_duration_s: float = 5.0
    record_deadline_monotonic: float = 0.0
    record_rows: list[LiveFrame] = field(default_factory=list)
    record_auto_started_stream: bool = False
    record_output_path: str | None = None


@dataclass
class FastRingViewState:
    meta: FastRingMeta | None = None
    sample_count: int = 0
    dumping: bool = False
    busy: bool = False
    output_path: str | None = None


@dataclass
class ControlViewState:
    connected: bool = False
    streaming: bool = False
    connecting: bool = False
    mock_mode: bool = False
    port: str | None = None
    baud: int | None = None
    fw: str | None = None
    cap_flags: int | None = None
    last_message: str | None = None


class DebugLinkGateway(QObject):
    linkStateChanged = Signal(bool, object, object)  # connected, port, baud
    deviceInfoReady = Signal(dict)
    liveFrameReady = Signal(object)  # LiveFrame
    fastRingStatusReady = Signal(object)  # FastRingMeta
    fastRingChunkReady = Signal(object, list)  # FastRingMeta, list[FastRingSample]
    errorRaised = Signal(str)
    connectSucceeded = Signal(object, bool, object)  # transport, ping_ok, info
    connectFailed = Signal(str)
    fastRingFinished = Signal(str)

    def on_link_state(self, connected: bool, port: str | None, baud: int | None) -> None:
        self.linkStateChanged.emit(connected, port, baud)

    def on_device_info(
        self,
        device_type: int,
        proto_version: int,
        fw: str,
        cap_flags: int,
        max_payload: int,
    ) -> None:
        self.deviceInfoReady.emit(
            {
                "device_type": device_type,
                "proto_version": proto_version,
                "fw": fw,
                "cap_flags": cap_flags,
                "max_payload": max_payload,
            }
        )

    def on_live_frame(self, frame: LiveFrame) -> None:
        self.liveFrameReady.emit(frame)

    def on_fastring_status(self, meta: FastRingMeta) -> None:
        self.fastRingStatusReady.emit(meta)

    def on_fastring_chunk(self, meta: FastRingMeta, samples: list[FastRingSample]) -> None:
        self.fastRingChunkReady.emit(meta, samples)

    def on_error(self, message: str) -> None:
        self.errorRaised.emit(message)

    def on_connect_succeeded(
        self, transport: object, ping_ok: bool, info: dict | None
    ) -> None:
        self.connectSucceeded.emit(transport, ping_ok, info)

    def on_connect_failed(self, message: str) -> None:
        self.connectFailed.emit(message)


class MainWindow(QMainWindow):
    def __init__(
        self,
        gateway: DebugLinkGateway,
        mock: bool = False,
        port: str = "COM33",
        baud: int = 921600,
        rate: int = 100,
    ) -> None:
        super().__init__()
        self.gateway = gateway
        self.live_state = LiveViewState()
        self.fastring_state = FastRingViewState()
        self.control_state = ControlViewState()

        self._transport: DebugLinkTransport | None = None
        self._closing = False
        self._repo_root = Path(__file__).resolve().parents[1]

        self.setWindowTitle("DebugLink GUI (Scaffold)")
        self.resize(1100, 760)
        self._build_ui()
        self._bind_signals()
        self.configure_session(port, baud, rate, mock)
        self._apply_ui_state()

    def _build_ui(self) -> None:
        tabs = QTabWidget()
        tabs.addTab(self._build_live_tab(), "Live")
        tabs.addTab(self._build_fastring_tab(), "FastRing")
        tabs.addTab(self._build_control_tab(), "Control")
        self.setCentralWidget(tabs)

    def _build_live_tab(self) -> QWidget:
        root = QWidget()
        layout = QVBoxLayout(root)

        session_box = QGroupBox("Live Session")
        session_form = QFormLayout(session_box)
        self.live_mode = QLabel("standby")
        self.live_stream = QLabel("stopped")
        self.live_rate = QLabel("-")
        self.live_record = QLabel("idle")
        session_form.addRow("mode", self.live_mode)
        session_form.addRow("stream", self.live_stream)
        session_form.addRow("rate_hz", self.live_rate)
        session_form.addRow("record", self.live_record)
        layout.addWidget(session_box)

        live_actions = QHBoxLayout()
        self.btn_connect = QPushButton("Connect")
        self.btn_disconnect = QPushButton("Disconnect")
        self.btn_stream_start = QPushButton("Start Stream")
        self.btn_stream_stop = QPushButton("Stop Stream")
        self.btn_pause_view = QPushButton("Pause View")
        self.record_duration_spin = QDoubleSpinBox()
        self.record_duration_spin.setRange(1.0, 60.0)
        self.record_duration_spin.setSingleStep(0.5)
        self.record_duration_spin.setValue(self.live_state.record_duration_s)
        self.record_duration_spin.setSuffix(" s")
        self.btn_stream_record = QPushButton("Record Stream CSV")
        for btn in (
            self.btn_connect,
            self.btn_disconnect,
            self.btn_stream_start,
            self.btn_stream_stop,
            self.btn_pause_view,
        ):
            live_actions.addWidget(btn)
        live_actions.addWidget(QLabel("Record"))
        live_actions.addWidget(self.record_duration_spin)
        live_actions.addWidget(self.btn_stream_record)
        live_actions.addStretch(1)
        layout.addLayout(live_actions)

        summary = QGroupBox("Summary")
        sform = QFormLayout(summary)
        self.live_tick = QLabel("-")
        self.live_bus = QLabel("-")
        self.live_fault = QLabel("-")
        sform.addRow("tick_ms", self.live_tick)
        sform.addRow("bus_v", self.live_bus)
        sform.addRow("fault", self.live_fault)
        layout.addWidget(summary)

        grid = QGridLayout()
        self.live_labels: dict[str, QLabel] = {}
        fields = [
            "pitch_target_deg",
            "pitch_meas_deg",
            "pitch_rate_dps",
            "speed_target_radps",
            "speed_meas_radps",
            "speed_raw_radps",
            "speed_p_term_deg",
            "speed_i_term_deg",
            "iq_cmd_a",
            "iq_cmd_clamped_a",
            "attitude_p_iq_cmd_a",
            "attitude_d_iq_cmd_a",
            "attitude_output_limit_a",
            "iq_l_a",
            "iq_r_a",
            "uq_l_v",
            "uq_r_v",
        ]
        for idx, name in enumerate(fields):
            lbl_name = QLabel(name)
            lbl_val = QLabel("-")
            self.live_labels[name] = lbl_val
            row = idx // 2
            col = (idx % 2) * 2
            grid.addWidget(lbl_name, row, col)
            grid.addWidget(lbl_val, row, col + 1)
        box = QGroupBox("Live Fields")
        box.setLayout(grid)
        layout.addWidget(box)
        layout.addStretch(1)
        return root

    def _build_fastring_tab(self) -> QWidget:
        root = QWidget()
        layout = QVBoxLayout(root)

        action_row = QHBoxLayout()
        self.btn_fastring_status = QPushButton("FastRing Status")
        self.btn_fastring_dump_left = QPushButton("Dump L")
        self.btn_fastring_dump_right = QPushButton("Dump R")
        self.btn_fastring_dump_both = QPushButton("Dump Both")
        self.btn_fastring_dump = QPushButton("Dump Dual")
        for btn in (
            self.btn_fastring_status,
            self.btn_fastring_dump_left,
            self.btn_fastring_dump_right,
            self.btn_fastring_dump_both,
            self.btn_fastring_dump,
        ):
            action_row.addWidget(btn)
        action_row.addStretch(1)
        layout.addLayout(action_row)

        form_box = QGroupBox("Ring Status")
        form = QFormLayout(form_box)
        self.fr_total = QLabel("-")
        self.fr_capacity = QLabel("-")
        self.fr_head = QLabel("-")
        self.fr_write_seq = QLabel("-")
        self.fr_samples = QLabel("0")
        form.addRow("total_count", self.fr_total)
        form.addRow("capacity", self.fr_capacity)
        form.addRow("head", self.fr_head)
        form.addRow("write_seq", self.fr_write_seq)
        form.addRow("samples_in_view", self.fr_samples)
        layout.addWidget(form_box)

        self.fr_log = QTextEdit()
        self.fr_log.setReadOnly(True)
        self.fr_log.setPlaceholderText("FastRing events and summary will appear here.")
        layout.addWidget(self.fr_log)
        return root

    def _build_control_tab(self) -> QWidget:
        root = QWidget()
        layout = QVBoxLayout(root)

        info_box = QGroupBox("Device")
        form = QFormLayout(info_box)
        self.ctrl_conn = QLabel("disconnected")
        self.ctrl_mode = QLabel("standby")
        self.ctrl_stream = QLabel("stopped")
        self.ctrl_port = QLabel("-")
        self.ctrl_baud = QLabel("-")
        self.ctrl_fw = QLabel("-")
        self.ctrl_caps = QLabel("-")
        form.addRow("link", self.ctrl_conn)
        form.addRow("mode", self.ctrl_mode)
        form.addRow("stream", self.ctrl_stream)
        form.addRow("port", self.ctrl_port)
        form.addRow("baud", self.ctrl_baud)
        form.addRow("fw", self.ctrl_fw)
        form.addRow("cap_flags", self.ctrl_caps)
        layout.addWidget(info_box)

        session_box = QGroupBox("Session Defaults")
        session_form = QFormLayout(session_box)
        port_row = QHBoxLayout()
        self.ctrl_port_combo = QComboBox()
        self.ctrl_port_combo.setEditable(True)
        self.btn_refresh_ports = QPushButton("Refresh Ports")
        port_row.addWidget(self.ctrl_port_combo)
        port_row.addWidget(self.btn_refresh_ports)
        self.ctrl_rate_spin = QSpinBox()
        self.ctrl_rate_spin.setRange(1, 200)
        self.ctrl_rate_spin.setValue(100)
        self.ctrl_mock_checkbox = QCheckBox("Mock mode")
        session_form.addRow("serial_port", port_row)
        session_form.addRow("stream_rate_hz", self.ctrl_rate_spin)
        session_form.addRow("options", self.ctrl_mock_checkbox)
        layout.addWidget(session_box)

        speed_box = QGroupBox("Speed Loop")
        speed_form = QFormLayout(speed_box)
        self.speed_kp_spin = QDoubleSpinBox()
        self.speed_kp_spin.setRange(0.0, 1.0)
        self.speed_kp_spin.setDecimals(6)
        self.speed_kp_spin.setSingleStep(0.001)
        self.speed_kp_spin.setValue(0.0015)
        self.speed_kp_spin.setToolTip("Speed loop Kp in rad/radps. Use 0.000000 to disable P.")
        speed_kp_row = QHBoxLayout()
        self.btn_speed_kp_read = QPushButton("Read Kp")
        self.btn_speed_kp_apply = QPushButton("Apply Kp")
        speed_kp_row.addWidget(self.speed_kp_spin)
        speed_kp_row.addWidget(self.btn_speed_kp_read)
        speed_kp_row.addWidget(self.btn_speed_kp_apply)
        speed_form.addRow("speed_kp_rad_per_radps", speed_kp_row)

        self.speed_ki_spin = QDoubleSpinBox()
        self.speed_ki_spin.setRange(0.0, 1.0)
        self.speed_ki_spin.setDecimals(6)
        self.speed_ki_spin.setSingleStep(0.001)
        self.speed_ki_spin.setValue(0.005)
        self.speed_ki_spin.setToolTip("Speed loop Ki in rad/rad. Use 0.000000 to disable I.")
        speed_ki_row = QHBoxLayout()
        self.btn_speed_ki_read = QPushButton("Read Ki")
        self.btn_speed_ki_apply = QPushButton("Apply Ki")
        speed_ki_row.addWidget(self.speed_ki_spin)
        speed_ki_row.addWidget(self.btn_speed_ki_read)
        speed_ki_row.addWidget(self.btn_speed_ki_apply)
        speed_form.addRow("speed_ki_rad_per_rad", speed_ki_row)

        self.speed_unwind_gain_spin = QDoubleSpinBox()
        self.speed_unwind_gain_spin.setRange(0.1, 20.0)
        self.speed_unwind_gain_spin.setDecimals(3)
        self.speed_unwind_gain_spin.setSingleStep(0.1)
        self.speed_unwind_gain_spin.setValue(1.0)
        self.speed_unwind_gain_spin.setToolTip("Speed loop integral unwind gain.")
        speed_unwind_row = QHBoxLayout()
        self.btn_speed_unwind_read = QPushButton("Read Unwind")
        self.btn_speed_unwind_apply = QPushButton("Apply Unwind")
        speed_unwind_row.addWidget(self.speed_unwind_gain_spin)
        speed_unwind_row.addWidget(self.btn_speed_unwind_read)
        speed_unwind_row.addWidget(self.btn_speed_unwind_apply)
        speed_form.addRow("speed_unwind_gain", speed_unwind_row)
        layout.addWidget(speed_box)

        attitude_box = QGroupBox("Attitude Loop")
        attitude_form = QFormLayout(attitude_box)

        self.attitude_kp_spin = QDoubleSpinBox()
        self.attitude_kp_spin.setRange(0.0, 1000.0)
        self.attitude_kp_spin.setDecimals(4)
        self.attitude_kp_spin.setSingleStep(0.1)
        self.attitude_kp_spin.setValue(9.6)
        self.attitude_kp_spin.setToolTip("Attitude loop Kp in A/rad.")
        attitude_kp_row = QHBoxLayout()
        self.btn_attitude_kp_read = QPushButton("Read Kp")
        self.btn_attitude_kp_apply = QPushButton("Apply Kp")
        attitude_kp_row.addWidget(self.attitude_kp_spin)
        attitude_kp_row.addWidget(self.btn_attitude_kp_read)
        attitude_kp_row.addWidget(self.btn_attitude_kp_apply)
        attitude_form.addRow("attitude_kp_a_per_rad", attitude_kp_row)

        self.attitude_kd_spin = QDoubleSpinBox()
        self.attitude_kd_spin.setRange(0.0, 1000.0)
        self.attitude_kd_spin.setDecimals(4)
        self.attitude_kd_spin.setSingleStep(0.01)
        self.attitude_kd_spin.setValue(0.18)
        self.attitude_kd_spin.setToolTip("Attitude loop Kd in A/radps.")
        attitude_kd_row = QHBoxLayout()
        self.btn_attitude_kd_read = QPushButton("Read Kd")
        self.btn_attitude_kd_apply = QPushButton("Apply Kd")
        attitude_kd_row.addWidget(self.attitude_kd_spin)
        attitude_kd_row.addWidget(self.btn_attitude_kd_read)
        attitude_kd_row.addWidget(self.btn_attitude_kd_apply)
        attitude_form.addRow("attitude_kd_a_per_radps", attitude_kd_row)

        self.attitude_iq_limit_spin = QDoubleSpinBox()
        self.attitude_iq_limit_spin.setRange(0.001, 1000.0)
        self.attitude_iq_limit_spin.setDecimals(4)
        self.attitude_iq_limit_spin.setSingleStep(0.1)
        self.attitude_iq_limit_spin.setValue(1.6)
        self.attitude_iq_limit_spin.setToolTip("Attitude loop output limit in A.")
        attitude_iq_limit_row = QHBoxLayout()
        self.btn_attitude_iq_limit_read = QPushButton("Read Iq Limit")
        self.btn_attitude_iq_limit_apply = QPushButton("Apply Iq Limit")
        attitude_iq_limit_row.addWidget(self.attitude_iq_limit_spin)
        attitude_iq_limit_row.addWidget(self.btn_attitude_iq_limit_read)
        attitude_iq_limit_row.addWidget(self.btn_attitude_iq_limit_apply)
        attitude_form.addRow("attitude_iq_limit_a", attitude_iq_limit_row)

        self.attitude_shutdown_spin = QDoubleSpinBox()
        self.attitude_shutdown_spin.setRange(0.001, 3.2)
        self.attitude_shutdown_spin.setDecimals(4)
        self.attitude_shutdown_spin.setSingleStep(0.01)
        self.attitude_shutdown_spin.setValue(2.0)
        self.attitude_shutdown_spin.setToolTip("Attitude shutdown threshold in rad.")
        attitude_shutdown_row = QHBoxLayout()
        self.btn_attitude_shutdown_read = QPushButton("Read Shutdown")
        self.btn_attitude_shutdown_apply = QPushButton("Apply Shutdown")
        attitude_shutdown_row.addWidget(self.attitude_shutdown_spin)
        attitude_shutdown_row.addWidget(self.btn_attitude_shutdown_read)
        attitude_shutdown_row.addWidget(self.btn_attitude_shutdown_apply)
        attitude_form.addRow("attitude_shutdown_rad", attitude_shutdown_row)

        layout.addWidget(attitude_box)

        btn_row = QHBoxLayout()
        self.btn_ping = QPushButton("Ping")
        self.btn_driver_on = QPushButton("Driver On")
        self.btn_driver_off = QPushButton("Driver Off")
        self.btn_balance_on = QPushButton("Balance On")
        self.btn_balance_off = QPushButton("Balance Off")
        for btn in (
            self.btn_ping,
            self.btn_driver_on,
            self.btn_driver_off,
            self.btn_balance_on,
            self.btn_balance_off,
        ):
            btn_row.addWidget(btn)
        layout.addLayout(btn_row)

        self.ctrl_log = QTextEdit()
        self.ctrl_log.setReadOnly(True)
        self.ctrl_log.setPlaceholderText("Control events and errors.")
        layout.addWidget(self.ctrl_log)
        return root

    def configure_session(self, port: str, baud: int, rate_hz: int, mock_mode: bool) -> None:
        self.control_state.port = port
        self.control_state.baud = baud
        self.control_state.mock_mode = mock_mode
        self.ctrl_port.setText(port)
        self.ctrl_baud.setText(str(baud))
        self._refresh_serial_ports(preferred=port)
        self.ctrl_rate_spin.setValue(rate_hz)
        self.ctrl_mock_checkbox.setChecked(mock_mode)
        self.live_rate.setText(str(rate_hz))
        self._apply_ui_state()

    def set_streaming_state(self, streaming: bool) -> None:
        self.control_state.streaming = streaming
        self._apply_ui_state()

    def _bind_signals(self) -> None:
        self.gateway.linkStateChanged.connect(self._on_link_state)
        self.gateway.deviceInfoReady.connect(self._on_device_info)
        self.gateway.liveFrameReady.connect(self._on_live_frame)
        self.gateway.fastRingStatusReady.connect(self._on_fastring_status)
        self.gateway.fastRingChunkReady.connect(self._on_fastring_chunk)
        self.gateway.errorRaised.connect(self._on_error)
        self.gateway.connectSucceeded.connect(self._on_connect_succeeded)
        self.gateway.connectFailed.connect(self._on_connect_failed)
        self.gateway.fastRingFinished.connect(self._on_fastring_finished)

        self.btn_pause_view.clicked.connect(self._toggle_live_pause)
        self.btn_connect.clicked.connect(self._on_connect_clicked)
        self.btn_disconnect.clicked.connect(self._on_disconnect_clicked)
        self.btn_stream_start.clicked.connect(self._on_start_stream_clicked)
        self.btn_stream_stop.clicked.connect(self._on_stop_stream_clicked)
        self.btn_stream_record.clicked.connect(self._on_stream_record_clicked)

        self.btn_ping.clicked.connect(self._on_ping_clicked)
        self.btn_driver_on.clicked.connect(lambda: self._on_driver_clicked(True))
        self.btn_driver_off.clicked.connect(lambda: self._on_driver_clicked(False))
        self.btn_balance_on.clicked.connect(lambda: self._on_balance_clicked(True))
        self.btn_balance_off.clicked.connect(lambda: self._on_balance_clicked(False))
        self.btn_speed_kp_read.clicked.connect(self._on_speed_kp_read_clicked)
        self.btn_speed_kp_apply.clicked.connect(self._on_speed_kp_apply_clicked)
        self.btn_speed_ki_read.clicked.connect(self._on_speed_ki_read_clicked)
        self.btn_speed_ki_apply.clicked.connect(self._on_speed_ki_apply_clicked)
        self.btn_speed_unwind_read.clicked.connect(self._on_speed_unwind_read_clicked)
        self.btn_speed_unwind_apply.clicked.connect(self._on_speed_unwind_apply_clicked)
        self.btn_attitude_kp_read.clicked.connect(self._on_attitude_kp_read_clicked)
        self.btn_attitude_kp_apply.clicked.connect(self._on_attitude_kp_apply_clicked)
        self.btn_attitude_kd_read.clicked.connect(self._on_attitude_kd_read_clicked)
        self.btn_attitude_kd_apply.clicked.connect(self._on_attitude_kd_apply_clicked)
        self.btn_attitude_iq_limit_read.clicked.connect(
            self._on_attitude_iq_limit_read_clicked
        )
        self.btn_attitude_iq_limit_apply.clicked.connect(
            self._on_attitude_iq_limit_apply_clicked
        )
        self.btn_attitude_shutdown_read.clicked.connect(
            self._on_attitude_shutdown_read_clicked
        )
        self.btn_attitude_shutdown_apply.clicked.connect(
            self._on_attitude_shutdown_apply_clicked
        )
        self.btn_fastring_status.clicked.connect(self._on_fastring_status_clicked)
        self.btn_fastring_dump_left.clicked.connect(
            lambda: self._start_fastring_side_dump(0, "left.csv")
        )
        self.btn_fastring_dump_right.clicked.connect(
            lambda: self._start_fastring_side_dump(1, "right.csv")
        )
        self.btn_fastring_dump_both.clicked.connect(self._start_fastring_split_dump)
        self.btn_fastring_dump.clicked.connect(self._on_fastring_dump_clicked)
        self.btn_refresh_ports.clicked.connect(self._on_refresh_ports_clicked)
        self.ctrl_port_combo.currentTextChanged.connect(self._on_port_combo_changed)

    def _on_refresh_ports_clicked(self) -> None:
        preferred = self.ctrl_port_combo.currentText().strip() or self.control_state.port or "COM33"
        self._refresh_serial_ports(preferred=preferred)

    def _on_port_combo_changed(self, text: str) -> None:
        port = text.strip()
        if port:
            self.control_state.port = port
            self.ctrl_port.setText(port)

    def _refresh_serial_ports(self, preferred: str | None = None) -> None:
        preferred_port = (preferred or "").strip()
        existing_text = self.ctrl_port_combo.currentText().strip()
        if not preferred_port:
            preferred_port = existing_text or self.control_state.port or "COM33"

        ports: list[str] = []
        try:
            from serial.tools import list_ports

            ports = sorted([p.device for p in list_ports.comports()])
        except Exception as e:
            self._append_control_log(f"[WARN] list serial ports failed: {e}")

        current = preferred_port
        self.ctrl_port_combo.blockSignals(True)
        self.ctrl_port_combo.clear()
        if ports:
            self.ctrl_port_combo.addItems(ports)
        else:
            self.ctrl_port_combo.addItem(preferred_port or "COM33")

        if preferred_port and self.ctrl_port_combo.findText(preferred_port) < 0:
            self.ctrl_port_combo.addItem(preferred_port)

        if preferred_port:
            self.ctrl_port_combo.setCurrentText(preferred_port)
        elif self.ctrl_port_combo.count() > 0:
            current = self.ctrl_port_combo.itemText(0)
            self.ctrl_port_combo.setCurrentIndex(0)
        self.ctrl_port_combo.blockSignals(False)

        chosen = self.ctrl_port_combo.currentText().strip() or current
        if chosen:
            self.control_state.port = chosen
            self.ctrl_port.setText(chosen)

    def _on_connect_clicked(self) -> None:
        if self._transport is not None or self.control_state.connecting:
            self._append_control_log("[WARN] connect already in progress or active")
            return

        port = self.ctrl_port_combo.currentText().strip() or self.control_state.port or "COM33"
        self.control_state.port = port
        self.ctrl_port.setText(port)
        baud = self.control_state.baud or 921600

        self.control_state.connecting = True
        self._apply_ui_state()
        self._append_control_log(f"Connecting to {port} @ {baud}...")

        worker = threading.Thread(
            target=self._connect_worker,
            args=(port, baud),
            name="gui-connect",
            daemon=True,
        )
        worker.start()

    def _connect_worker(self, port: str, baud: int) -> None:
        transport = DebugLinkTransport()
        try:
            transport.connect(port, baud)
        except TransportError as e:
            self.gateway.on_connect_failed(f"connect failed: {e}")
            return

        ping_ok = transport.ping()
        info = None
        try:
            info = transport.get_info()
        except TransportError:
            info = None

        self.gateway.on_connect_succeeded(transport, ping_ok, info)

    def _on_connect_succeeded(
        self, transport: object, ping_ok: bool, info: dict | None
    ) -> None:
        if self._closing:
            try:
                transport.disconnect()  # type: ignore[union-attr]
            except Exception:
                pass
            return
        if not self.control_state.connecting:
            try:
                transport.disconnect()  # type: ignore[union-attr]
            except Exception:
                pass
            return

        self._transport = transport if isinstance(transport, DebugLinkTransport) else None
        self.control_state.connecting = False
        self.gateway.on_link_state(True, self.control_state.port, self.control_state.baud)
        self._append_control_log("Serial port opened.")
        self._append_control_log(f"Ping -> {'OK' if ping_ok else 'FAIL'}")
        if not ping_ok:
            self._append_control_log("[WARN] ping failed - device may not be responding")

        if info is not None:
            self.gateway.on_device_info(
                device_type=info["device_type"],
                proto_version=info["proto_version"],
                fw=f"{info['fw_major']}.{info['fw_minor']}.{info['fw_patch']}",
                cap_flags=info["cap_flags"],
                max_payload=info["max_payload"],
            )
            self._append_control_log(
                f"Device: type=0x{info['device_type']:02X} "
                f"fw={info['fw_major']}.{info['fw_minor']}.{info['fw_patch']} "
                f"caps=0x{info['cap_flags']:04X}"
            )
        else:
            self._append_control_log("[WARN] get_info failed")

        self._on_speed_kp_read_clicked()
        self._on_speed_ki_read_clicked()
        self._on_speed_unwind_read_clicked()
        self._on_attitude_kp_read_clicked()
        self._on_attitude_kd_read_clicked()
        self._on_attitude_iq_limit_read_clicked()
        self._on_attitude_shutdown_read_clicked()
        self._apply_ui_state()

    def _on_connect_failed(self, message: str) -> None:
        if self._closing:
            return
        self.control_state.connecting = False
        self._apply_ui_state()
        self._append_control_log(f"[ERROR] {message}")

    def _on_disconnect_clicked(self) -> None:
        self._append_control_log("Disconnecting...")
        if self.live_state.recording:
            self._finish_stream_recording("disconnected")
        self._stop_stream_internal()

        if self._transport is not None:
            try:
                self._transport.disconnect()
            except Exception as e:
                self._append_control_log(f"[WARN] disconnect error: {e}")
            self._transport = None

        mock_feeder = getattr(self, "mock_feeder", None)
        if mock_feeder is not None:
            try:
                mock_feeder.timer.stop()
            except Exception:
                pass

        self.gateway.on_link_state(False, self.control_state.port, self.control_state.baud)
        self._append_control_log("Disconnected.")

    def _on_start_stream_clicked(self) -> None:
        if self._transport is None:
            self._append_control_log("[WARN] not connected")
            return
        if self.control_state.streaming:
            self._append_control_log("[WARN] stream already running")
            return

        rate = self.ctrl_rate_spin.value()
        self._append_control_log(f"Starting stream @ {rate} Hz...")

        self._transport.set_stream_callback(self._on_stream_frame)
        ok = self._transport.stream_start(rate_hz=rate)
        if ok:
            self.control_state.streaming = True
            self._apply_ui_state()
            self._append_control_log("Stream started.")
        else:
            self._transport.set_stream_callback(None)
            self._append_control_log("[ERROR] stream start failed")

    def _on_stop_stream_clicked(self) -> None:
        self._append_control_log("Stopping stream...")
        if self.live_state.recording:
            self._finish_stream_recording("stopped manually")
        self._stop_stream_internal()
        self._append_control_log("Stream stopped.")

    def _on_stream_record_clicked(self) -> None:
        if self._transport is None:
            self._append_control_log("[WARN] not connected")
            return
        if self.live_state.recording:
            self._append_control_log("[WARN] stream recording already in progress")
            return

        duration_s = float(self.record_duration_spin.value())
        self.live_state.record_duration_s = duration_s

        auto_started = False
        if not self.control_state.streaming:
            self._append_control_log("Record requested: stream is stopped, starting stream first...")
            self._on_start_stream_clicked()
            if not self.control_state.streaming:
                self._append_control_log("[ERROR] cannot start recording because stream failed to start")
                return
            auto_started = True

        self.live_state.record_rows = []
        self.live_state.recording = True
        self.live_state.record_auto_started_stream = auto_started
        self.live_state.record_deadline_monotonic = time.monotonic() + duration_s
        self._append_control_log(f"Stream recording started ({duration_s:.1f}s)")
        self._apply_ui_state()

    def _finish_stream_recording(self, reason: str | None = None) -> None:
        if not self.live_state.recording:
            return

        rows = list(self.live_state.record_rows)
        auto_started = self.live_state.record_auto_started_stream
        self.live_state.recording = False
        self.live_state.record_rows = []
        self.live_state.record_auto_started_stream = False
        self.live_state.record_deadline_monotonic = 0.0

        output_path = None
        if len(rows) > 0:
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            output_path = self._resolve_stream_output_path(f"stream_{timestamp}.csv")
            self.live_state.record_output_path = str(output_path)
            try:
                self._write_stream_csv(rows, output_path)
                self._append_control_log(
                    f"Stream recording saved: {len(rows)} frames -> {output_path}"
                )
            except OSError as e:
                self._append_control_log(
                    f"[ERROR] cannot write stream CSV: {output_path} ({e})"
                )
        else:
            self._append_control_log("[WARN] stream recording finished with 0 frames")

        if reason is not None:
            self._append_control_log(f"Stream recording ended ({reason})")
        elif output_path is not None:
            self._append_control_log("Stream recording ended (duration reached)")

        if auto_started and self.control_state.streaming:
            self._append_control_log("Auto-stopping stream (recording started it)")
            self._stop_stream_internal()

        self._apply_ui_state()

    def _stop_stream_internal(self) -> None:
        if not self.control_state.streaming:
            return
        self.control_state.streaming = False
        self._apply_ui_state()

        if self._transport is not None:
            self._transport.set_stream_callback(None)
            try:
                self._transport.stream_stop()
            except Exception:
                pass

    def _on_ping_clicked(self) -> None:
        if self._transport is None:
            self._append_control_log("[WARN] not connected")
            return
        ok = self._transport.ping()
        self._append_control_log(f"Ping -> {'OK' if ok else 'FAIL'}")

    def _on_driver_clicked(self, enable: bool) -> None:
        if self._transport is None:
            self._append_control_log("[WARN] not connected")
            return
        action = "ON" if enable else "OFF"
        self._append_control_log(f"Driver {action}...")
        ok = self._transport.driver_enable(enable)
        self._append_control_log(f"Driver {action} -> {'OK' if ok else 'FAIL'}")

    def _on_balance_clicked(self, enable: bool) -> None:
        if self._transport is None:
            self._append_control_log("[WARN] not connected")
            return
        action = "ON" if enable else "OFF"
        self._append_control_log(f"Balance {action}...")
        ok = self._transport.balance_enable(enable)
        self._append_control_log(f"Balance {action} -> {'OK' if ok else 'FAIL'}")

    def _on_speed_kp_read_clicked(self) -> None:
        if self._transport is None:
            self._append_control_log("[WARN] not connected")
            return
        try:
            value = self._transport.get_speed_kp()
        except TransportError as e:
            self._append_control_log(f"[ERROR] read Speed Kp failed: {e}")
            return

        self.speed_kp_spin.setValue(value)
        self._append_control_log(f"Speed Kp <- {value:.6f}")

    def _on_speed_kp_apply_clicked(self) -> None:
        if self._transport is None:
            self._append_control_log("[WARN] not connected")
            return

        value = float(self.speed_kp_spin.value())
        try:
            ok = self._transport.set_speed_kp(value)
        except TransportError as e:
            self._append_control_log(f"[ERROR] apply Speed Kp failed: {e}")
            return

        self._append_control_log(
            f"Speed Kp -> {value:.6f} {'OK' if ok else 'FAIL'}"
        )

    def _on_speed_ki_read_clicked(self) -> None:
        if self._transport is None:
            self._append_control_log("[WARN] not connected")
            return
        try:
            value = self._transport.get_speed_ki()
        except TransportError as e:
            self._append_control_log(f"[ERROR] read Speed Ki failed: {e}")
            return

        self.speed_ki_spin.setValue(value)
        self._append_control_log(f"Speed Ki <- {value:.6f}")

    def _on_speed_ki_apply_clicked(self) -> None:
        if self._transport is None:
            self._append_control_log("[WARN] not connected")
            return

        value = float(self.speed_ki_spin.value())
        try:
            ok = self._transport.set_speed_ki(value)
        except TransportError as e:
            self._append_control_log(f"[ERROR] apply Speed Ki failed: {e}")
            return

        self._append_control_log(
            f"Speed Ki -> {value:.6f} {'OK' if ok else 'FAIL'}"
        )

    def _on_speed_unwind_read_clicked(self) -> None:
        if self._transport is None:
            self._append_control_log("[WARN] not connected")
            return
        try:
            value = self._transport.get_speed_unwind_gain()
        except TransportError as e:
            self._append_control_log(f"[ERROR] read Speed Unwind failed: {e}")
            return

        self.speed_unwind_gain_spin.setValue(value)
        self._append_control_log(f"Speed Unwind <- {value:.3f}")

    def _on_speed_unwind_apply_clicked(self) -> None:
        if self._transport is None:
            self._append_control_log("[WARN] not connected")
            return

        value = float(self.speed_unwind_gain_spin.value())
        try:
            ok = self._transport.set_speed_unwind_gain(value)
        except TransportError as e:
            self._append_control_log(f"[ERROR] apply Speed Unwind failed: {e}")
            return

        self._append_control_log(
            f"Speed Unwind -> {value:.3f} {'OK' if ok else 'FAIL'}"
        )

    def _on_attitude_kp_read_clicked(self) -> None:
        if self._transport is None:
            self._append_control_log("[WARN] not connected")
            return
        try:
            value = self._transport.get_attitude_kp()
        except TransportError as e:
            self._append_control_log(f"[ERROR] read Attitude Kp failed: {e}")
            return

        self.attitude_kp_spin.setValue(value)
        self._append_control_log(f"Attitude Kp <- {value:.4f}")

    def _on_attitude_kp_apply_clicked(self) -> None:
        if self._transport is None:
            self._append_control_log("[WARN] not connected")
            return

        value = float(self.attitude_kp_spin.value())
        try:
            ok = self._transport.set_attitude_kp(value)
        except TransportError as e:
            self._append_control_log(f"[ERROR] apply Attitude Kp failed: {e}")
            return

        self._append_control_log(
            f"Attitude Kp -> {value:.4f} {'OK' if ok else 'FAIL'}"
        )

    def _on_attitude_kd_read_clicked(self) -> None:
        if self._transport is None:
            self._append_control_log("[WARN] not connected")
            return
        try:
            value = self._transport.get_attitude_kd()
        except TransportError as e:
            self._append_control_log(f"[ERROR] read Attitude Kd failed: {e}")
            return

        self.attitude_kd_spin.setValue(value)
        self._append_control_log(f"Attitude Kd <- {value:.4f}")

    def _on_attitude_kd_apply_clicked(self) -> None:
        if self._transport is None:
            self._append_control_log("[WARN] not connected")
            return

        value = float(self.attitude_kd_spin.value())
        try:
            ok = self._transport.set_attitude_kd(value)
        except TransportError as e:
            self._append_control_log(f"[ERROR] apply Attitude Kd failed: {e}")
            return

        self._append_control_log(
            f"Attitude Kd -> {value:.4f} {'OK' if ok else 'FAIL'}"
        )

    def _on_attitude_iq_limit_read_clicked(self) -> None:
        if self._transport is None:
            self._append_control_log("[WARN] not connected")
            return
        try:
            value = self._transport.get_attitude_iq_limit()
        except TransportError as e:
            self._append_control_log(f"[ERROR] read Attitude Iq Limit failed: {e}")
            return

        self.attitude_iq_limit_spin.setValue(value)
        self._append_control_log(f"Attitude Iq Limit <- {value:.4f}")

    def _on_attitude_iq_limit_apply_clicked(self) -> None:
        if self._transport is None:
            self._append_control_log("[WARN] not connected")
            return

        value = float(self.attitude_iq_limit_spin.value())
        try:
            ok = self._transport.set_attitude_iq_limit(value)
        except TransportError as e:
            self._append_control_log(f"[ERROR] apply Attitude Iq Limit failed: {e}")
            return

        self._append_control_log(
            f"Attitude Iq Limit -> {value:.4f} {'OK' if ok else 'FAIL'}"
        )

    def _on_attitude_shutdown_read_clicked(self) -> None:
        if self._transport is None:
            self._append_control_log("[WARN] not connected")
            return
        try:
            value = self._transport.get_attitude_shutdown_rad()
        except TransportError as e:
            self._append_control_log(f"[ERROR] read Attitude Shutdown failed: {e}")
            return

        self.attitude_shutdown_spin.setValue(value)
        self._append_control_log(f"Attitude Shutdown <- {value:.4f} rad")

    def _on_attitude_shutdown_apply_clicked(self) -> None:
        if self._transport is None:
            self._append_control_log("[WARN] not connected")
            return

        value = float(self.attitude_shutdown_spin.value())
        try:
            ok = self._transport.set_attitude_shutdown_rad(value)
        except TransportError as e:
            self._append_control_log(f"[ERROR] apply Attitude Shutdown failed: {e}")
            return

        self._append_control_log(
            f"Attitude Shutdown -> {value:.4f} rad {'OK' if ok else 'FAIL'}"
        )

    def _on_stream_frame(self, frame: LiveFrame) -> None:
        self.gateway.liveFrameReady.emit(frame)

    def _on_fastring_status_clicked(self) -> None:
        if self._transport is None:
            self._append_fastring_log("[WARN] not connected")
            return
        try:
            meta = self._transport.fastring_status()
        except TransportError as e:
            self._append_fastring_log(f"[ERROR] fastring_status: {e}")
            return
        self.gateway.on_fastring_status(meta)

    def _on_fastring_dump_clicked(self) -> None:
        if self._transport is None:
            self._append_fastring_log("[WARN] not connected")
            return
        if self.fastring_state.busy or self.fastring_state.dumping:
            self._append_fastring_log("[WARN] dump already in progress")
            return

        self.fastring_state.busy = True
        self.fastring_state.dumping = True
        self.fastring_state.sample_count = 0
        output_path = self._resolve_fastring_output_path("fastring_dual.csv")
        self.fastring_state.output_path = str(output_path)
        self._apply_ui_state()
        self._append_fastring_log(f"FastRing dump starting -> {output_path}")

        worker = threading.Thread(
            target=self._fastring_dump_worker,
            args=(output_path,),
            name="gui-fastring-dump",
            daemon=True,
        )
        worker.start()

    def _start_fastring_side_dump(self, target_source: int, filename: str) -> None:
        if self._transport is None:
            self._append_fastring_log("[WARN] not connected")
            return
        if self.fastring_state.dumping or self.fastring_state.busy:
            self._append_fastring_log("[WARN] dump already in progress")
            return

        self.fastring_state.busy = True
        self.fastring_state.dumping = True
        self.fastring_state.sample_count = 0
        output_path = self._resolve_fastring_output_path(filename)
        self.fastring_state.output_path = str(output_path)
        self._apply_ui_state()
        self._append_fastring_log(
            f"Dump {filename} starting (target source={target_source}) -> {output_path}"
        )

        worker = threading.Thread(
            target=self._fastring_side_dump_worker,
            args=(target_source, output_path),
            name="gui-fastring-side-dump",
            daemon=True,
        )
        worker.start()

    def _start_fastring_split_dump(self) -> None:
        if self._transport is None:
            self._append_fastring_log("[WARN] not connected")
            return
        if self.fastring_state.dumping or self.fastring_state.busy:
            self._append_fastring_log("[WARN] dump already in progress")
            return

        self.fastring_state.busy = True
        self.fastring_state.dumping = True
        self.fastring_state.sample_count = 0
        self.fastring_state.output_path = str(self._repo_root / "current_loop_data")
        self._apply_ui_state()
        self._append_fastring_log(
            f"Dump Both starting -> {self._resolve_fastring_output_path('left.csv')} , {self._resolve_fastring_output_path('right.csv')}"
        )

        worker = threading.Thread(
            target=self._fastring_split_dump_worker,
            name="gui-fastring-split-dump",
            daemon=True,
        )
        worker.start()

    def _fastring_side_dump_worker(self, target_source: int, output_path: Path) -> None:
        try:
            snapshot_meta, all_samples = self._snapshot_and_collect_fastring()
            self._write_fastring_side_csv(
                all_samples, target_source, snapshot_meta.write_seq, output_path
            )
            self.gateway.fastRingFinished.emit(f"Dump complete -> {output_path}")
        except TransportError as e:
            self.gateway.fastRingFinished.emit(f"[ERROR] dump transport: {e}")
        except OSError as e:
            self.gateway.fastRingFinished.emit(
                f"[ERROR] file write failed: {output_path} ({e})"
            )

    def _fastring_split_dump_worker(self) -> None:
        left_path = self._resolve_fastring_output_path("left.csv")
        right_path = self._resolve_fastring_output_path("right.csv")
        try:
            snapshot_meta, all_samples = self._snapshot_and_collect_fastring()
            self._write_fastring_side_csv(
                all_samples, 0, snapshot_meta.write_seq, left_path
            )
            self.gateway.fastRingFinished.emit(f"Dump L complete -> {left_path}")
            self.fastring_state.sample_count = 0
            self._write_fastring_side_csv(
                all_samples, 1, snapshot_meta.write_seq, right_path
            )
            self.gateway.fastRingFinished.emit(f"Dump R complete -> {right_path}")
            self.gateway.fastRingFinished.emit(
                f"Dump Both complete -> {left_path} , {right_path}"
            )
        except TransportError as e:
            self.gateway.fastRingFinished.emit(f"[ERROR] dump both transport: {e}")
        except OSError as e:
            self.gateway.fastRingFinished.emit(f"[ERROR] dump both file write failed: {e}")

    def _fastring_dump_worker(self, output_path: Path) -> None:
        try:
            _, all_samples = self._snapshot_and_collect_fastring()
            self._write_fastring_dual_csv(all_samples, output_path)

            self.gateway.fastRingFinished.emit(
                f"FastRing dump complete: {len(all_samples)} samples -> {output_path}"
            )
        except TransportError as e:
            self.gateway.fastRingFinished.emit(f"[ERROR] fastring dump transport: {e}")
        except OSError as e:
            self.gateway.fastRingFinished.emit(
                f"[ERROR] fastring file write failed: {output_path} ({e})"
            )

    def _snapshot_and_collect_fastring(self) -> tuple[FastRingMeta, list[FastRingSample]]:
        live_meta = self._transport.fastring_status()
        self.gateway.on_fastring_status(live_meta)
        if live_meta.total_count == 0:
            raise TransportError("FastRing is empty")

        snapshot_meta = self._transport.fastring_snapshot()
        self.gateway.on_fastring_status(snapshot_meta)
        self.gateway.fastRingFinished.emit(
            f"[INFO] Snapshot frozen: total_count={snapshot_meta.total_count} write_seq={snapshot_meta.write_seq}"
        )

        all_samples: list[FastRingSample] = []
        chunk_size = 8
        start_idx = 0
        while start_idx < snapshot_meta.total_count:
            remaining = snapshot_meta.total_count - start_idx
            req = min(chunk_size, remaining)
            chunk_meta, samples = self._transport.fastring_read_chunk(
                snapshot_meta.write_seq, start_idx, req
            )
            if chunk_meta.total_count != snapshot_meta.total_count:
                raise TransportError("fastring dump aborted: total_count changed during read")
            if chunk_meta.write_seq != snapshot_meta.write_seq:
                raise TransportError("fastring dump aborted: write_seq changed during read")
            if samples and samples[0].index != start_idx:
                raise TransportError("fastring dump aborted: start_idx mismatch during read")

            all_samples.extend(samples)
            self.gateway.on_fastring_chunk(chunk_meta, samples)
            start_idx += len(samples)
            if len(samples) == 0:
                break

        if len(all_samples) != snapshot_meta.total_count:
            raise TransportError(
                f"fastring dump incomplete: expected {snapshot_meta.total_count} samples, got {len(all_samples)}"
            )
        return snapshot_meta, all_samples

    def _write_fastring_dual_csv(
        self, all_samples: list[FastRingSample], output_path: Path
    ) -> None:
        with open(output_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(
                [
                    "idx",
                    "target_iq_l",
                    "iq_ref_l",
                    "filtered_iq_l",
                    "raw_iq_l",
                    "uq_final_l",
                    "target_iq_r",
                    "iq_ref_r",
                    "filtered_iq_r",
                    "raw_iq_r",
                    "uq_final_r",
                    "bus_v",
                    "sample_idx",
                    "status_flags",
                ]
            )
            for s in all_samples:
                writer.writerow(
                    [
                        s.index,
                        s.target_iq_l_a,
                        s.iq_ref_l_a,
                        s.filtered_iq_l_a,
                        s.raw_iq_l_a,
                        s.uq_final_l_v,
                        s.target_iq_r_a,
                        s.iq_ref_r_a,
                        s.filtered_iq_r_a,
                        s.raw_iq_r_a,
                        s.uq_final_r_v,
                        s.bus_v,
                        s.sample_idx,
                        s.status_flags,
                    ]
                )

    def _write_fastring_side_csv(
        self,
        all_samples: list[FastRingSample],
        target_source: int,
        snapshot_write_seq: int,
        output_path: Path,
    ) -> None:
        side_label = "R" if target_source == 1 else "L"
        with open(output_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(
                [
                    "idx",
                    "target_iq",
                    "iq_ref",
                    "filtered_iq",
                    "raw_iq",
                    "uq_final",
                    "source",
                    "capture_id",
                ]
            )
            for idx, s in enumerate(all_samples):
                if target_source == 1:
                    row = [
                        idx,
                        s.target_iq_r_a,
                        s.iq_ref_r_a,
                        s.filtered_iq_r_a,
                        s.raw_iq_r_a,
                        s.uq_final_r_v,
                        side_label,
                        snapshot_write_seq,
                    ]
                else:
                    row = [
                        idx,
                        s.target_iq_l_a,
                        s.iq_ref_l_a,
                        s.filtered_iq_l_a,
                        s.raw_iq_l_a,
                        s.uq_final_l_v,
                        side_label,
                        snapshot_write_seq,
                    ]
                writer.writerow(row)

    def _append_control_log(self, message: str) -> None:
        self.ctrl_log.append(message)

    def _append_fastring_log(self, message: str) -> None:
        self.fr_log.append(message)

    def _reset_live_view(self) -> None:
        self.live_tick.setText("-")
        self.live_bus.setText("-")
        self.live_fault.setText("-")
        for lbl in self.live_labels.values():
            lbl.setText("-")

    def _reset_fastring_view(self) -> None:
        self.fr_total.setText("-")
        self.fr_capacity.setText("-")
        self.fr_head.setText("-")
        self.fr_write_seq.setText("-")
        self.fr_samples.setText("0")
        self.fastring_state.meta = None
        self.fastring_state.sample_count = 0
        self.fastring_state.busy = False
        self.fastring_state.dumping = False
        self.fastring_state.output_path = None

    def _resolve_fastring_output_path(self, filename: str) -> Path:
        output_dir = self._repo_root / "current_loop_data"
        output_dir.mkdir(parents=True, exist_ok=True)
        return output_dir / filename

    def _resolve_stream_output_path(self, filename: str) -> Path:
        output_dir = self._repo_root / "stream_data"
        output_dir.mkdir(parents=True, exist_ok=True)
        return output_dir / filename

    def _write_stream_csv(self, rows: list[LiveFrame], output_path: Path) -> None:
        with open(output_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(
                [
                    "idx",
                    "host_rx_time_ms",
                    "tick_ms",
                    "pitch_target_deg",
                    "speed_p_term_deg",
                    "speed_i_term_deg",
                    "pitch_meas_deg",
                    "pitch_rate_dps",
                    "speed_target_radps",
                    "speed_meas_radps",
                    "speed_raw_radps",
                    "attitude_p_iq_cmd_a",
                    "attitude_d_iq_cmd_a",
                    "iq_cmd_a",
                    "iq_cmd_clamped_a",
                    "speed_output_limit_deg",
                    "attitude_output_limit_a",
                    "iq_l_a",
                    "iq_r_a",
                    "uq_l_v",
                    "uq_r_v",
                    "bus_v",
                    "fault_flags",
                    "fault_labels",
                ]
            )
            for idx, frame in enumerate(rows):
                writer.writerow(
                    [
                        idx,
                        frame.host_rx_time_ms,
                        frame.tick_ms,
                        frame.pitch_target_deg,
                        frame.speed_p_term_deg,
                        frame.speed_i_term_deg,
                        frame.pitch_meas_deg,
                        frame.pitch_rate_dps,
                        frame.speed_target_radps,
                        frame.speed_meas_radps,
                        frame.speed_raw_radps,
                        frame.attitude_p_iq_cmd_a,
                        frame.attitude_d_iq_cmd_a,
                        frame.iq_cmd_a,
                        frame.iq_cmd_clamped_a,
                        frame.speed_output_limit_deg,
                        frame.attitude_output_limit_a,
                        frame.iq_l_a,
                        frame.iq_r_a,
                        frame.uq_l_v,
                        frame.uq_r_v,
                        frame.bus_v,
                        frame.fault_flags,
                        frame.fault_labels,
                    ]
                )

    def _toggle_live_pause(self) -> None:
        self.live_state.paused = not self.live_state.paused
        self.btn_pause_view.setText("Resume View" if self.live_state.paused else "Pause View")
        self._append_control_log(
            "Live view paused" if self.live_state.paused else "Live view resumed"
        )

    def _apply_ui_state(self) -> None:
        connected = self.control_state.connected
        streaming = self.control_state.streaming
        connecting = self.control_state.connecting
        fastring_busy = self.fastring_state.busy or self.fastring_state.dumping
        busy = connecting or fastring_busy

        self.ctrl_conn.setText(
            "connecting..." if connecting else ("connected" if connected else "disconnected")
        )
        self.ctrl_mode.setText("mock" if self.control_state.mock_mode else "real")
        self.ctrl_stream.setText("running" if streaming else "stopped")
        self.live_mode.setText("mock" if self.control_state.mock_mode else "real")
        self.live_stream.setText("running" if streaming else "stopped")
        self.live_rate.setText(str(self.ctrl_rate_spin.value()))
        if self.live_state.recording:
            remain = max(0.0, self.live_state.record_deadline_monotonic - time.monotonic())
            self.live_record.setText(
                f"recording ({remain:.1f}s, n={len(self.live_state.record_rows)})"
            )
            self.btn_stream_record.setText("Recording...")
        else:
            self.live_record.setText("idle")
            self.btn_stream_record.setText("Record Stream CSV")

        self.btn_connect.setEnabled(not connected and not connecting)
        self.btn_disconnect.setEnabled(connected and not busy)
        self.btn_stream_start.setEnabled(connected and not streaming and not busy)
        self.btn_stream_stop.setEnabled(connected and streaming and not busy)
        self.btn_pause_view.setEnabled(connected or self.control_state.mock_mode)
        self.record_duration_spin.setEnabled(
            connected and (not self.live_state.recording) and (not busy)
        )
        self.btn_stream_record.setEnabled(
            connected and (not self.live_state.recording) and (not busy)
        )

        self.btn_ping.setEnabled(connected and not busy)
        self.btn_driver_on.setEnabled(connected and not busy)
        self.btn_driver_off.setEnabled(connected and not busy)
        self.btn_balance_on.setEnabled(connected and not busy)
        self.btn_balance_off.setEnabled(connected and not busy)
        self.speed_kp_spin.setEnabled(connected and not busy)
        self.btn_speed_kp_read.setEnabled(connected and not busy)
        self.btn_speed_kp_apply.setEnabled(connected and not busy)
        self.speed_ki_spin.setEnabled(connected and not busy)
        self.btn_speed_ki_read.setEnabled(connected and not busy)
        self.btn_speed_ki_apply.setEnabled(connected and not busy)
        self.speed_unwind_gain_spin.setEnabled(connected and not busy)
        self.btn_speed_unwind_read.setEnabled(connected and not busy)
        self.btn_speed_unwind_apply.setEnabled(connected and not busy)
        self.attitude_kp_spin.setEnabled(connected and not busy)
        self.btn_attitude_kp_read.setEnabled(connected and not busy)
        self.btn_attitude_kp_apply.setEnabled(connected and not busy)
        self.attitude_kd_spin.setEnabled(connected and not busy)
        self.btn_attitude_kd_read.setEnabled(connected and not busy)
        self.btn_attitude_kd_apply.setEnabled(connected and not busy)
        self.attitude_iq_limit_spin.setEnabled(connected and not busy)
        self.btn_attitude_iq_limit_read.setEnabled(connected and not busy)
        self.btn_attitude_iq_limit_apply.setEnabled(connected and not busy)
        self.attitude_shutdown_spin.setEnabled(connected and not busy)
        self.btn_attitude_shutdown_read.setEnabled(connected and not busy)
        self.btn_attitude_shutdown_apply.setEnabled(connected and not busy)

        self.btn_fastring_status.setEnabled(connected and not streaming and not busy)
        self.btn_fastring_dump_left.setEnabled(connected and not streaming and not busy)
        self.btn_fastring_dump_right.setEnabled(connected and not streaming and not busy)
        self.btn_fastring_dump_both.setEnabled(connected and not streaming and not busy)
        self.btn_fastring_dump.setEnabled(connected and not streaming and not busy)
        self.ctrl_port_combo.setEnabled(not connected and not busy)
        self.btn_refresh_ports.setEnabled(not connected and not busy)
        self.ctrl_rate_spin.setEnabled(not connected and not busy)
        self.ctrl_mock_checkbox.setEnabled(False)

    def _on_link_state(self, connected: bool, port: str | None, baud: int | None) -> None:
        self.control_state.connected = connected
        self.control_state.port = port
        self.control_state.baud = baud
        self.ctrl_port.setText(port or "-")
        self.ctrl_baud.setText(str(baud) if baud is not None else "-")
        if not connected:
            self._refresh_serial_ports(preferred=port or self.control_state.port or "COM33")
        if not connected:
            if self.live_state.recording:
                self._finish_stream_recording("link lost")
            self.control_state.streaming = False
            self.live_state.last_frame = None
            self._reset_live_view()
            self._reset_fastring_view()
        self._apply_ui_state()

    def _on_device_info(self, info: dict) -> None:
        self.control_state.fw = info.get("fw")
        self.control_state.cap_flags = info.get("cap_flags")
        self.ctrl_fw.setText(str(info.get("fw", "-")))
        caps = info.get("cap_flags")
        self.ctrl_caps.setText(f"0x{caps:04X}" if isinstance(caps, int) else "-")
        self._apply_ui_state()

    def _on_live_frame(self, frame: LiveFrame) -> None:
        if self.live_state.recording:
            self.live_state.record_rows.append(frame)
            if time.monotonic() >= self.live_state.record_deadline_monotonic:
                self._finish_stream_recording()
            else:
                self._apply_ui_state()

        if self.live_state.paused:
            return
        self.live_state.last_frame = frame
        self.live_tick.setText(str(frame.tick_ms))
        self.live_bus.setText(f"{frame.bus_v:.2f} V")
        self.live_fault.setText(f"0x{frame.fault_flags:04X} [{frame.fault_labels}]")
        for key, lbl in self.live_labels.items():
            val = getattr(frame, key)
            if isinstance(val, float):
                lbl.setText(f"{val:+.3f}")
            else:
                lbl.setText(str(val))

    def _on_fastring_status(self, meta: FastRingMeta) -> None:
        self.fastring_state.meta = meta
        self.fr_total.setText(str(meta.total_count))
        self.fr_capacity.setText(str(meta.capacity))
        self.fr_head.setText(str(meta.head))
        self.fr_write_seq.setText(str(meta.write_seq))
        self.fr_log.append(
            f"STATUS total={meta.total_count} capacity={meta.capacity} head={meta.head} write_seq={meta.write_seq}"
        )
        self._apply_ui_state()

    def _on_fastring_chunk(self, meta: FastRingMeta, samples: list[FastRingSample]) -> None:
        self.fastring_state.meta = meta
        self.fastring_state.sample_count += len(samples)
        self.fr_samples.setText(str(self.fastring_state.sample_count))
        self._apply_ui_state()

    def _on_error(self, message: str) -> None:
        self.control_state.last_message = message
        self._append_control_log(message)

    def _on_fastring_finished(self, message: str) -> None:
        self._append_fastring_log(message)
        self.fastring_state.busy = False
        self.fastring_state.dumping = False
        self._apply_ui_state()

    def closeEvent(self, event: Any) -> None:
        self._closing = True
        if self.live_state.recording:
            self._finish_stream_recording("window closed")
        self._stop_stream_internal()
        if self._transport is not None:
            try:
                self._transport.disconnect()
            except Exception:
                pass
            self._transport = None
        mock_feeder = getattr(self, "mock_feeder", None)
        if mock_feeder is not None:
            try:
                mock_feeder.timer.stop()
            except Exception:
                pass
        super().closeEvent(event)


class MockFeeder(QObject):
    def __init__(self, gateway: DebugLinkGateway) -> None:
        super().__init__()
        self.gateway = gateway
        self.tick = 0
        self.phase = 0.0
        self.fastring_sent = False
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._step)
        self.timer.start(50)
        self.gateway.on_link_state(True, "COM33", 921600)
        self.gateway.on_device_info(0x01, 1, "0.1.0", 0x00E3, 240)

    def _step(self) -> None:
        self.tick += 50
        self.phase += 0.08
        frame = LiveFrame(
            tick_ms=self.tick,
            pitch_target_deg=0.5 * math.sin(self.phase),
            speed_p_term_deg=0.4 * math.sin(self.phase * 1.1),
            speed_i_term_deg=0.2 * math.sin(self.phase * 0.4),
            pitch_meas_deg=0.8 * math.sin(self.phase + 0.2),
            pitch_rate_dps=2.5 * math.cos(self.phase),
            speed_target_radps=0.0,
            speed_meas_radps=0.4 * math.sin(self.phase * 0.6),
            speed_raw_radps=-0.4 * math.sin(self.phase * 0.6),
            attitude_p_iq_cmd_a=0.2 * math.sin(self.phase * 1.3),
            attitude_d_iq_cmd_a=0.1 * math.cos(self.phase * 1.4),
            iq_cmd_a=0.25 * math.sin(self.phase * 1.2),
            iq_cmd_clamped_a=max(-0.2, min(0.2, 0.25 * math.sin(self.phase * 1.2))),
            speed_output_limit_deg=4.58,
            attitude_output_limit_a=1.2,
            iq_l_a=0.1 * math.sin(self.phase * 1.8) + random.uniform(-0.02, 0.02),
            iq_r_a=0.1 * math.cos(self.phase * 1.6) + random.uniform(-0.02, 0.02),
            uq_l_v=1.5 * math.sin(self.phase * 1.2),
            uq_r_v=1.5 * math.cos(self.phase * 1.1),
            bus_v=19.95 + random.uniform(-0.02, 0.02),
            fault_flags=0x3F00,
            fault_labels="stack,it,bus,iloop,sloop,cloop",
            host_rx_time_ms=int(time.time() * 1000),
        )
        self.gateway.on_live_frame(frame)

        if not self.fastring_sent and self.tick > 1500:
            meta = FastRingMeta(
                op_echo=0x03,
                total_count=128,
                capacity=512,
                head=128,
                write_seq=12,
            )
            self.gateway.on_fastring_status(meta)
            chunk = []
            for i in range(64):
                chunk.append(
                    FastRingSample(
                        index=i,
                        target_iq_l_a=0.8 * math.sin(i * 0.08),
                        iq_ref_l_a=0.8 * math.sin(i * 0.08 + 0.02),
                        filtered_iq_l_a=0.75 * math.sin(i * 0.08 + 0.03),
                        raw_iq_l_a=0.75 * math.sin(i * 0.08 + 0.03) + random.uniform(-0.05, 0.05),
                        uq_final_l_v=2.0 * math.sin(i * 0.08),
                        target_iq_r_a=-0.7 * math.sin(i * 0.08),
                        iq_ref_r_a=-0.7 * math.sin(i * 0.08 + 0.02),
                        filtered_iq_r_a=-0.65 * math.sin(i * 0.08 + 0.03),
                        raw_iq_r_a=-0.65 * math.sin(i * 0.08 + 0.03) + random.uniform(-0.05, 0.05),
                        uq_final_r_v=-1.8 * math.sin(i * 0.08),
                        bus_v=19.95 + random.uniform(-0.02, 0.02),
                        sample_idx=i,
                        status_flags=0x3F00,
                    )
                )
            self.gateway.on_fastring_chunk(meta, chunk)
            self.fastring_sent = True


def main() -> int:
    import argparse

    parser = argparse.ArgumentParser(description="DebugLink GUI")
    parser.add_argument("--port", default="COM33", help="serial port (default: COM33)")
    parser.add_argument("--baud", type=int, default=921600, help="baudrate (default: 921600)")
    parser.add_argument("--rate", type=int, default=100, help="stream rate Hz (default: 100)")
    parser.add_argument("--mock", action="store_true", help="use mock data instead of real serial")
    args = parser.parse_args()

    app = QApplication(sys.argv)
    gateway = DebugLinkGateway()
    window = MainWindow(
        gateway,
        mock=args.mock,
        port=args.port,
        baud=args.baud,
        rate=args.rate,
    )
    window.show()

    if args.mock:
        window.mock_feeder = MockFeeder(gateway)  # type: ignore[attr-defined]
        window.set_streaming_state(True)
        window.ctrl_log.append("Running in MOCK mode.")
    else:
        window.ctrl_log.append(f"Ready. Click Connect to open {args.port} @ {args.baud}.")

    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
