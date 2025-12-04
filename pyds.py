#!/usr/bin/env python3
"""
modern_ds_fullnet.py

Modern PyQt6 Driver Station UI integrated with:
 - UDP 1110 packet sender (DS -> roboRIO)
 - UDP listener & parser for robot -> DS telemetry
 - Alliance station selection and FMS-attached toggle
 - Best-effort telemetry parsing for CPU/RAM/Disk/CAN tags

AUTHOR: generated
SAFETY: This can send real DS packets. Default target is 127.0.0.1.
Test locally before using on a real robot.
"""

from __future__ import annotations
import sys
import socket
import struct
import threading
import time
import datetime
from typing import List, Optional, Dict, Tuple

from PyQt6.QtCore import Qt, QTimer, pyqtSignal
from PyQt6.QtWidgets import (
    QApplication, QWidget, QLabel, QVBoxLayout, QHBoxLayout, QFrame,
    QPushButton, QComboBox, QLineEdit, QTextEdit, QCheckBox, QGridLayout
)
import random

# -----------------------------
# Protocol constants
# -----------------------------
cRequestRestartCode = 0x04
cRequestReboot = 0x08
cRequestNormal = 0x00
cTagCommVersion = 0x01
cFMSCommVersion = 0x00
cTeleoperated = 0x00
cTest = 0x01
cAutonomous = 0x02
cEnabled = 0x04
cEmergencyStop = 0x80
cFMSConnected = 0x08
cFMSRadioPing = 0x10
cFMSRobotPing = 0x08
cFMSRobotComms = 0x20
cRTagCANInfo = 0x0e
cRTagCPUInfo = 0x05
cRTagRAMInfo = 0x06
cRTagDiskInfo = 0x04
cRequestTime = 0x01
cRobotHasCode = 0x20
cTagDate = 0x0f
cTagJoystick = 0x0c
cTagTimezone = 0x10
cRed1 = 0x00
cRed2 = 0x01
cRed3 = 0x02
cBlue1 = 0x03
cBlue2 = 0x04
cBlue3 = 0x05
cRTagJoystickOutput = 0x01
cRTagDiskInfo = 0x04
cRTagCPUInfo = 0x05
cRTagRAMInfo = 0x06
cRTagPDPLog = 0x08
cRTagUnknown = 0x09
cRTagCANInfo = 0x0e

UDP_PORT_DEFAULT = 1110
LISTENER_PORT_DEFAULT = 1150
SEND_INTERVAL = 0.020  # 20 ms

DEFAULT_IP = '192.168.1.184'
DEFAULT_TEAM = '5113'

# -----------------------------
# Utility pack/unpack
# -----------------------------
def pack_uint16(v: int) -> bytes:
    return struct.pack(">H", v)

def pack_uint32(v: int) -> bytes:
    return struct.pack(">I", v)

def pack_int16(v: int) -> bytes:
    return struct.pack(">h", v)

def pack_float32(f: float) -> bytes:
    return struct.pack(">f", f)

def frame_tag(tag_id: int, data: bytes) -> bytes:
    size = 1 + len(data)
    if size > 255:
        raise ValueError("Tag too large")
    return struct.pack("B", size) + struct.pack("B", tag_id) + data

# -----------------------------
# Joystick & tag builders
# -----------------------------
def build_joystick_tag(axes: List[float], buttons: List[bool], povs: List[Optional[int]]) -> bytes:
    axis_count = len(axes)
    axis_bytes = bytearray()
    for a in axes:
        a_clamped = max(-1.0, min(1.0, float(a)))
        val = int(round(a_clamped * 127.0))
        if val < -128: val = -128
        if val > 127: val = 127
        axis_bytes.append(val & 0xFF)
    btn_count = len(buttons)
    btn_bytes_len = 0 if btn_count == 0 else ((btn_count - 1) // 8) + 1
    btn_bytes = bytearray(btn_bytes_len)
    for i, pressed in enumerate(buttons):
        if pressed:
            bidx = i // 8
            bit = i % 8
            btn_bytes[bidx] |= (1 << bit)
    pov_count = len(povs)
    pov_bytes = bytearray()
    for p in povs:
        if p is None:
            pov_bytes += pack_int16(-1)
        else:
            v = int(round(p))
            if v < 0:
                pov_bytes += pack_int16(-1)
            else:
                v = max(0, min(360, v))
                pov_bytes += pack_int16(v)
    data = bytearray()
    data.append(axis_count & 0xFF)
    data += axis_bytes
    data.append(btn_count & 0xFF)
    data += btn_bytes
    data.append(pov_count & 0xFF)
    data += pov_bytes
    return frame_tag(cTagJoystick, bytes(data))

def build_date_tag(ts: Optional[float] = None) -> bytes:
    t = ts if ts is not None else time.time()
    utc = time.gmtime(t)
    micro = int((t - int(t)) * 1_000_000) & 0xFFFFFFFF
    sec = utc.tm_sec
    minute = utc.tm_min
    hour = utc.tm_hour
    day = utc.tm_mday
    month_zero = utc.tm_mon - 1
    year_offset = utc.tm_year - 1900
    data = bytearray()
    data += pack_uint32(micro)
    data += struct.pack("B", sec)
    data += struct.pack("B", minute)
    data += struct.pack("B", hour)
    data += struct.pack("B", day)
    data += struct.pack("B", month_zero & 0xFF)
    data += struct.pack("B", year_offset & 0xFF)
    return frame_tag(cTagDate, bytes(data))

def build_timezone_tag(tz_name: str) -> bytes:
    return frame_tag(cTagTimezone, tz_name.encode("utf-8"))

def build_countdown_tag(seconds: float) -> bytes:
    return frame_tag(0x07, pack_float32(float(seconds)))

def alliance_value(color: str, position: int) -> int:
    pos = max(1, min(3, int(position)))
    if color.lower().startswith("r"):
        base = 0
    else:
        base = 3
    return base + (pos - 1)

def build_ds_packet(seq: int, control_byte: int, request_byte: int, alliance_val: int, tags: List[bytes]) -> bytes:
    header = bytearray()
    header += pack_uint16(seq & 0xFFFF)
    header += struct.pack("B", cTagCommVersion & 0xFF)  # comm version
    header += struct.pack("B", control_byte & 0xFF)
    header += struct.pack("B", request_byte & 0xFF)
    header += struct.pack("B", alliance_val & 0xFF)
    body = b"".join(tags)
    return bytes(header + body)

# -----------------------------
# Sender class (DS -> roboRIO)
# -----------------------------
class DriverStationPacketSender:
    def __init__(self, target_ip: str = "127.0.0.1", target_port: int = UDP_PORT_DEFAULT):
        self.target_ip = target_ip
        self.target_port = target_port
        self.sock: Optional[socket.socket] = None
        self.seq = 0
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._control_flags = 0
        self._request_flags = 0
        self._alliance_val = cRed1
        self._date_tag_sent = False
        self._tz_tag_sent = False
        self.timezone_name = "UTC"
        self.joystick_axes: List[float] = []
        self.joystick_buttons: List[bool] = []
        self.joystick_povs: List[Optional[int]] = []
        self.countdown_seconds: Optional[float] = None
        self.interval = SEND_INTERVAL

    def start(self):
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None

    def _ensure_socket(self):
        if not self.sock:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def set_enabled(self, enabled: bool):
        if enabled:
            self._control_flags |= cEnabled
        else:
            self._control_flags &= ~cEnabled

    def set_estop(self, estop: bool):
        if estop:
            self._control_flags |= cEmergencyStop
        else:
            self._control_flags &= ~cEmergencyStop

    def set_fms_connected(self, fms: bool):
        if fms:
            self._control_flags |= cFMSConnected
        else:
            self._control_flags &= ~cFMSConnected

    def set_mode(self, mode: str):
        # clear mode bits
        self._control_flags &= ~(cTeleoperated | cTest | cAutonomous)
        m = mode.lower()
        if m.startswith("tele"):
            self._control_flags |= cTeleoperated
        elif m.startswith("test"):
            self._control_flags |= cTest
        elif m.startswith("auto"):
            self._control_flags |= cAutonomous
        else:
            self._control_flags |= cTeleoperated

    def request_reboot(self):
        self._request_flags |= cRequestReboot

    def request_restart_code(self):
        self._request_flags |= cRequestRestartCode

    def set_alliance(self, color: str, position: int):
        self._alliance_val = alliance_value(color, position)

    def update_joystick(self, axes: List[float], buttons: List[bool], povs: List[Optional[int]]):
        self.joystick_axes = list(axes)
        self.joystick_buttons = list(buttons)
        self.joystick_povs = list(povs)

    def set_countdown(self, seconds: Optional[float]):
        self.countdown_seconds = None if seconds is None else float(seconds)

    def set_timezone(self, tz_name: str):
        self.timezone_name = tz_name
        self._tz_tag_sent = False

    def _run_loop(self):
        self._ensure_socket()
        while self._running:
            try:
                tags: List[bytes] = []
                if not self._date_tag_sent:
                    tags.append(build_date_tag())
                    self._date_tag_sent = True
                if not self._tz_tag_sent:
                    tags.append(build_timezone_tag(self.timezone_name))
                    self._tz_tag_sent = True
                if self.countdown_seconds is not None:
                    tags.append(build_countdown_tag(self.countdown_seconds))
                tags.append(build_joystick_tag(self.joystick_axes, self.joystick_buttons, self.joystick_povs))
                pkt = build_ds_packet(self.seq, self._control_flags, self._request_flags, self._alliance_val, tags)
                self.sock.sendto(pkt, (self.target_ip, int(self.target_port)))
                self.seq = (self.seq + 1) & 0xFFFF
                # clear one-shot request bits
                self._request_flags &= ~(cRequestReboot | cRequestRestartCode)
            except Exception as e:
                # in production log more carefully
                print("ERROR in send loop:", e)
            time.sleep(self.interval)

# -----------------------------
# UDP Listener & Parser (robot -> DS)
# -----------------------------

class RobotTelemetryParser:
    def parse(self, data: bytes):
        out = {
            "seq": None,
            "comm_version": None,
            "status": {},
            "trace": {},
            "battery": None,
            "request_date": None,
            "tags": []
        }

        if len(data) < 8:
            out["raw"] = data.hex()
            return out

        seq = struct.unpack_from(">H", data, 0)[0]
        comm_ver = data[2]
        status_byte = data[3]
        trace_byte = data[4]
        battery_raw = struct.unpack_from(">H", data, 5)[0]
        request_date = data[7]

        out.update({
            "seq": seq,
            "comm_version": comm_ver,
            "status": self._parse_status(status_byte),
            "trace": self._parse_trace(trace_byte),
            "battery": (battery_raw >> 8) + ((battery_raw & 0xFF) / 256),
            "request_date": request_date,
        })

        offset = 8
        tags = []
        while offset + 2 <= len(data):
            size = data[offset]
            if size == 0:
                break
            tag_id = data[offset + 1]
            payload_len = size - 1
            start = offset + 2
            end = start + payload_len
            if end > len(data):
                payload = data[start:]
                offset = len(data)
            else:
                payload = data[start:end]
                offset = end
            parsed = self._interpret_tag(tag_id, payload)
            tags.append({"id": tag_id, "raw": payload.hex(), "parsed": parsed})
            # print(f"Tag {tag_id:02X}, size {size}, payload {payload.hex()}, parsed {parsed}")
        out["tags"] = tags
        return out

    @staticmethod
    def _parse_status(byte: int):
        return {
            "e_stop": bool(byte & 0b10000000),
            "brownout": bool(byte & 0b00010000),
            "code_start": bool(byte & 0b00001000),
            "enabled": bool(byte & 0b00000100),
            "mode": (byte & 0b00000011)
        }

    @staticmethod
    def _parse_trace(byte: int):
        return {
            "robot_code": bool(byte & 0b00100000),
            "is_roborio": bool(byte & 0b00010000),
            "test_mode": bool(byte & 0b00001000),
            "autonomous_mode": bool(byte & 0b00000100),
            "teleop_code": bool(byte & 0b00000010),
            "disabled": bool(byte & 0b00000001)
        }

    def _interpret_tag(self, tag_id: int, payload: bytes):
        if tag_id == cRTagJoystickOutput:
            if len(payload) < 8:
                return {"joystick_empty": True}
            outputs = struct.unpack("<I", payload[0:4])[0]
            left_rumble = struct.unpack("<H", payload[4:6])[0]
            right_rumble = struct.unpack("<H", payload[6:8])[0]
            bits = []
            for i in range(32):
                if outputs & (1 << i):
                    bits.append(i)
            return {
                "type": "joystick",
                "outputs_bitmask": outputs,
                "outputs_active": bits,
                "left_rumble": left_rumble,
                "right_rumble": right_rumble
            }

        if tag_id == cRTagDiskInfo:
            if len(payload) < 4:
                return None
            free = struct.unpack("<I", payload[:4])[0]
            return {"type": "disk", "free_space_bytes": free}

        if tag_id == cRTagCPUInfo:
            # CPU tag may contain up to five float32 values (20 bytes).
            # Parse tolerantly and provide more human-friendly fields.
            vals = []
            n_vals = min(5, len(payload) // 4)
            for i in range(n_vals):
                try:
                    vals.append(struct.unpack_from("<f", payload, i * 4)[0])
                except struct.error:
                    break

            keys = ["num_cpus", "time_critical_pct", "above_normal_pct", "normal_pct", "low_pct"]

            def _fmt_pct(v):
                if v is None:
                    return None
                try:
                    fv = float(v)
                except Exception:
                    return None
                # Heuristic: if value is in [0.0,1.0], treat as fraction and convert to percent.
                if 0.0 <= fv <= 1.0:
                    pct = fv * 100.0
                else:
                    pct = fv
                return f"{pct:.1f}%"

            out = {"type": "cpu"}
            for i, k in enumerate(keys):
                val = vals[i] if i < len(vals) else None
                if k == "num_cpus":
                    # num_cpus is most useful as an integer when present
                    if val is None:
                        out[k] = None
                    else:
                        try:
                            out[k] = int(round(val))
                        except Exception:
                            out[k] = val
                else:
                    out[k] = val
                    out[f"{k}_human"] = _fmt_pct(val)

            return out

        if tag_id == cRTagRAMInfo:
            if len(payload) < 8:
                return None
            block, free = struct.unpack("<II", payload[:8])
            return {
                "type": "ram",
                "block_bytes": block,
                "free_space_bytes": free
            }

        if tag_id == cRTagPDPLog:
            if len(payload) < 21:
                return None
            val = int.from_bytes(payload[:21], "little")
            ports = []
            for i in range(16):
                p = (val >> (i * 10)) & 0x3FF
                ports.append(p)
            return {"type": "pdp", "ports": ports}

        if tag_id == cRTagCANInfo:
            if len(payload) < 14:
                return None
            util = struct.unpack("<f", payload[0:4])[0]
            bus_off = struct.unpack("<I", payload[4:8])[0]
            tx_full = struct.unpack("<I", payload[8:12])[0]
            rx_err = payload[12]
            tx_err = payload[13]
            return {
                "type": "can",
                "utilization_percent": util,
                "bus_off": bus_off,
                "tx_full": tx_full,
                "rx_errors": rx_err,
                "tx_errors": tx_err
            }


        return {"type": "raw", "value": payload.hex()}


class RobotTelemetryListener:
    """
    UDP listener for robot telemetry packets.
    Calls `on_packet(parsed: dict, addr: (ip, port))` for each received packet.
    """
    def __init__(self, bind_ip="0.0.0.0", bind_port: int = LISTENER_PORT_DEFAULT, on_packet=None):
        self.bind_ip = bind_ip
        self.bind_port = bind_port
        self.sock: Optional[socket.socket] = None
        self.running = False
        self.thread: Optional[threading.Thread] = None
        self.parser = RobotTelemetryParser()
        self.on_packet = on_packet

    def start(self):
        if self.running:
            return
        self.running = True
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None

    def _run(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.sock.bind((self.bind_ip, self.bind_port))
            self.sock.settimeout(1.0)
        except Exception as e:
            if self.on_packet:
                self.on_packet({"error": f"bind_failed: {e}"}, (self.bind_ip, self.bind_port))
            return

        while self.running:
            try:
                data, addr = self.sock.recvfrom(4096)
                parsed = self.parser.parse(data)
                if self.on_packet:
                    self.on_packet(parsed)
            except socket.timeout:
                continue
            except Exception as e:
                if self.on_packet:
                    self.on_packet({"error": f"recv_error: {e}"})
                continue

# -----------------------------
# PyQt6 UI (Modern) - integrated
# -----------------------------
COLOR_DARK = "#333333"
COLOR_PURPLE = "#7339AB"
COLOR_PURPLE_LIGHT = "#625AD8"
COLOR_BLUE = "#1F9CE4"
COLOR_TEXT = "#FFFFFF"

class Indicator(QLabel):
    def __init__(self, label, color_off="#550000", color_on="#00AA00"):
        super().__init__()
        self.setFixedHeight(28)
        self.label = label
        self.color_on = color_on
        self.color_off = color_off
        self.setText(label)
        self.set_state(False)

    def set_state(self, on: bool):
        if on:
            self.setStyleSheet(f"background-color: {self.color_on}; border-radius:6px; color: white; padding:6px;")
        else:
            self.setStyleSheet(f"background-color: {self.color_off}; border-radius:6px; color: white; padding:6px;")

class ModernDS(QWidget):
    telemetry_signal = pyqtSignal(dict)
    def __init__(self):
        super().__init__()
        self.telemetry_signal.connect(self.on_telemetry_packet)
        self.setWindowTitle("Modern FRC Driver Station (Full Network)")
        self.setMinimumSize(1300, 760)
        self.setStyleSheet(f"background-color: {COLOR_DARK}; color: {COLOR_TEXT};")
        # network pieces
        self.ds_sender = DriverStationPacketSender(target_ip="127.0.0.1", target_port=UDP_PORT_DEFAULT)
        self.listener = RobotTelemetryListener(bind_ip="0.0.0.0", bind_port=LISTENER_PORT_DEFAULT, on_packet=self.telemetry_signal.emit)

        self.sending = False
        self.listening = False
        # telemetry state
        self.telemetry = {
            "last_packet_time": None,
            "packet_rate": 0.0,
            "last_seq": None,
            "cpu": None,
            "ram": None,
            "disk": None,
            "can": None,
            "raw_tags": []
        }
        # UI build
        self.build_ui()
        self.start_time = None
        self.time_timer = QTimer()
        self.time_timer.timeout.connect(self.update_time)
        self.packet_counter = 0
        self.packet_rate_timer = QTimer()
        self.packet_rate_timer.timeout.connect(self._update_packet_rate)
        self.packet_rate_timer.start(1000)
    def _update_packet_rate(self):
        # Computes packets per second
        now = time.time()
        dt = now - getattr(self, "_last_rate_time", now)
        count = self.packet_counter - getattr(self, "_last_rate_count", 0)

        if dt > 0:
            rate = count / dt
            self.lbl_pkt_rate.setText(f"{rate:.1f} Hz")

        self._last_rate_time = now
        self._last_rate_count = self.packet_counter

    def build_ui(self):
        main_layout = QVBoxLayout()
        main_layout.setContentsMargins(18, 18, 18, 18)
        main_layout.setSpacing(16)

        # Top panel (full width)
        top = self.build_top_panel()
        main_layout.addWidget(top, alignment=Qt.AlignmentFlag.AlignTop)

        # Horizontal layout for left, center, right
        row = QHBoxLayout()
        row.setSpacing(16)

        left = self.build_left_panel()
        center = self.build_center_panel()
        right = self.build_right_panel()

        row.addWidget(left, 1)
        row.addWidget(center, 1)
        row.addWidget(right, 1)

        main_layout.addLayout(row)
        self.setLayout(main_layout)

    def section_title(self, text):
        lbl = QLabel(text)
        lbl.setStyleSheet(f"font-size:20px; font-weight:bold; color:{COLOR_BLUE}; padding-bottom:8px;")
        return lbl
    
    def build_top_panel(self):
        frame = QFrame()
        frame.setStyleSheet("QFrame { background-color: #D0342c; border-radius:12px; }")
        h = QHBoxLayout(frame)
        title = QLabel("WARNING: Experimental use only; may cause runaway robot. Illegal to use at any official event.")
        title.setStyleSheet("font-size:24px; font-weight:bold;")
        h.addWidget(title)
        h.addStretch()
        return frame

    def build_left_panel(self):
        frame = QFrame()
        frame.setStyleSheet("QFrame { background-color: #2a2a2a; border-radius:12px; }")
        v = QVBoxLayout(frame)
        v.addWidget(self.section_title("Robot Mode"))
        self.mode_combo = QComboBox()
        self.mode_combo.addItems(["Teleop", "Autonomous", "Test", "Practice"])
        self.mode_combo.setStyleSheet(f"background-color: {COLOR_PURPLE_LIGHT}; padding:6px; border-radius:6px;")
        v.addWidget(self.mode_combo)
        self.enable_btn = QPushButton("ENABLE")
        self.enable_btn.setStyleSheet(f"background-color: {COLOR_PURPLE}; font-size:18px; border-radius:10px; padding:12px;")
        self.enable_btn.clicked.connect(self.enable_robot)
        self.disable_btn = QPushButton("DISABLE")
        self.disable_btn.setStyleSheet("background-color: #AA3333; font-size:18px; border-radius:10px; padding:12px;")
        self.disable_btn.clicked.connect(self.disable_robot)
        self.disable_btn.setEnabled(False)
        v.addWidget(self.enable_btn)
        v.addWidget(self.disable_btn)
        v.addWidget(self.section_title("Elapsed Time"))
        self.time_label = QLabel("0:00")
        self.time_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.time_label.setStyleSheet("font-size:28px; background-color:#111; border-radius:8px; padding:12px;")
        v.addWidget(self.time_label)
        v.addStretch()
        return frame

    def build_center_panel(self):
        frame = QFrame()
        frame.setStyleSheet("QFrame { background-color: #2a2a2a; border-radius:12px; }")
        v = QVBoxLayout(frame)
        v.setSpacing(12)

        v.addWidget(self.section_title("Status & Telemetry"))

        # Indicators row
        ind_row = QHBoxLayout()
        self.ind_comms = Indicator("Comms")
        self.ind_code = Indicator("RobotCode")
        self.ind_enabled = Indicator("Enabled", color_on="#0088FF")
        ind_row.addWidget(self.ind_comms)
        ind_row.addWidget(self.ind_code)
        ind_row.addWidget(self.ind_enabled)
        v.addLayout(ind_row)

        # Telemetry grid (basic info)
        tel_grid = QGridLayout()
        tel_grid.setSpacing(8)

        # Last packet, packet rate, sequence
        tel_grid.addWidget(QLabel("Last packet:"), 0, 0)
        self.lbl_last_packet = QLabel("N/A")
        tel_grid.addWidget(self.lbl_last_packet, 0, 1)

        tel_grid.addWidget(QLabel("Packet rate (Hz):"), 1, 0)
        self.lbl_pkt_rate = QLabel("0.0")
        tel_grid.addWidget(self.lbl_pkt_rate, 1, 1)

        tel_grid.addWidget(QLabel("Seq:"), 2, 0)
        self.lbl_seq = QLabel("N/A")
        tel_grid.addWidget(self.lbl_seq, 2, 1)

        tel_grid.addWidget(QLabel("Battery (V):"), 3, 0)
        self.lbl_battery = QLabel("N/A")
        tel_grid.addWidget(self.lbl_battery, 3, 1)

        # CPU, RAM, Disk, CAN
        tel_grid.addWidget(QLabel("CPU %:"), 4, 0)
        self.lbl_cpu = QLabel("N/A")
        tel_grid.addWidget(self.lbl_cpu, 4, 1)

        tel_grid.addWidget(QLabel("RAM %:"), 5, 0)
        self.lbl_ram = QLabel("N/A")
        tel_grid.addWidget(self.lbl_ram, 5, 1)

        tel_grid.addWidget(QLabel("Disk %:"), 6, 0)
        self.lbl_disk = QLabel("N/A")
        tel_grid.addWidget(self.lbl_disk, 6, 1)

        tel_grid.addWidget(QLabel("CAN:"), 7, 0)
        self.lbl_can = QLabel("N/A")
        tel_grid.addWidget(self.lbl_can, 7, 1)

        v.addLayout(tel_grid)

        # Robot status checkboxes
        v.addWidget(self.section_title("Robot Status"))
        status_grid = QGridLayout()
        status_grid.setSpacing(6)

        self.ind_e_stop = Indicator("E-Stop")
        self.ind_brownout = Indicator("Brownout")
        self.ind_code_start = Indicator("Code Start")
        self.ind_enabled = Indicator("Enabled")

        status_grid.addWidget(self.ind_e_stop)
        status_grid.addWidget(self.ind_brownout)
        status_grid.addWidget(self.ind_code_start)
        status_grid.addWidget(self.ind_enabled)

        self.lbl_mode = QLabel("N/A")
        status_grid.addWidget(QLabel("Mode:"), 4, 0)
        status_grid.addWidget(self.lbl_mode, 4, 1)


        v.addLayout(status_grid)


        # Raw telemetry log
        v.addWidget(self.section_title("Telemetry Raw / Tags"))
        self.log_box = QTextEdit()
        self.log_box.setReadOnly(True)
        self.log_box.setStyleSheet("background-color:#111; border-radius:8px; padding:6px;")
        v.addWidget(self.log_box, stretch=1)

        return frame


    def build_right_panel(self):
        frame = QFrame()
        frame.setStyleSheet("QFrame { background-color: #2a2a2a; border-radius:12px; }")
        v = QVBoxLayout(frame)
        v.addWidget(self.section_title("Network Setup"))
        self.team_input = QLineEdit(DEFAULT_TEAM)
        self.ip_input = QLineEdit(DEFAULT_IP)
        self.port_input = QLineEdit(str(UDP_PORT_DEFAULT))
        self.listen_port_input = QLineEdit(str(LISTENER_PORT_DEFAULT))
        for w in (self.team_input, self.ip_input, self.port_input, self.listen_port_input):
            w.setStyleSheet("background-color:#111; padding:6px; border-radius:6px;")
        v.addWidget(QLabel("Team Number"))
        v.addWidget(self.team_input)
        v.addWidget(QLabel("Target IP"))
        v.addWidget(self.ip_input)
        v.addWidget(QLabel("Port (send to robot)"))
        v.addWidget(self.port_input)
        v.addWidget(QLabel("Listener bind port (receive from robot)"))
        v.addWidget(self.listen_port_input)
        # Alliance selection
        v.addWidget(self.section_title("Alliance / FMS"))
        self.alliance_combo = QComboBox()
        self.alliance_combo.addItems(["Red 1","Red 2","Red 3","Blue 1","Blue 2","Blue 3"])
        self.alliance_combo.currentIndexChanged.connect(self._alliance_changed)
        v.addWidget(self.alliance_combo)
        self.fms_checkbox = QCheckBox("FMS Attached")
        self.fms_checkbox.stateChanged.connect(self._fms_toggled)
        v.addWidget(self.fms_checkbox)
        v.addWidget(self.section_title("Actions"))
        reboot_btn = QPushButton("Reboot roboRIO")
        reboot_btn.setStyleSheet(f"background-color:{COLOR_PURPLE_LIGHT}; border-radius:8px; padding:8px;")
        reboot_btn.clicked.connect(self.request_reboot)
        restart_btn = QPushButton("Restart Code")
        restart_btn.setStyleSheet(f"background-color:{COLOR_PURPLE_LIGHT}; border-radius:8px; padding:8px;")
        restart_btn.clicked.connect(self.request_restart)
        estop_btn = QPushButton("EMERGENCY STOP")
        estop_btn.setStyleSheet("background-color:#CC0000; color:white; font-weight:bold; border-radius:8px; padding:8px;")
        estop_btn.clicked.connect(self.emergency_stop)
        v.addWidget(reboot_btn)
        v.addWidget(restart_btn)
        v.addWidget(estop_btn)
        v.addStretch()
        return frame

    def log(self, msg: str):
        ts = time.strftime("%H:%M:%S")
        self.log_box.append(f"[{ts}] {msg}")

    def update_time(self):
        if not self.start_time:
            return
        elapsed = int(time.time() - self.start_time)
        m = elapsed // 60
        s = elapsed % 60
        self.time_label.setText(f"{m}:{s:02d}")

    def _alliance_changed(self, idx: int):
        mapping = {
            0: cRed1, 1: cRed2, 2: cRed3,
            3: cBlue1, 4: cBlue2, 5: cBlue3
        }
        val = mapping.get(idx, cRed1)
        # update sender alliance (sender stores as color/position mapping)
        # convert val back to color/pos to use sender.set_alliance
        if val <= 2:
            color = "red"
            pos = val + 1
        else:
            color = "blue"
            pos = (val - 3) + 1
        self.ds_sender.set_alliance(color, pos)
        self.log(f"Alliance set: {color} {pos}")

    def _fms_toggled(self, state):
        on = (state == Qt.CheckState.Checked)
        self.ds_sender.set_fms_connected(on)
        # listener / telemetry may also adapt behaviour
        if on:
            self.log("FMS attached: ON")
        else:
            self.log("FMS attached: OFF")

    # UI actions -> sender/listener
    def enable_robot(self):
        if self.sending:
            return
        ip = self.ip_input.text().strip()
        try:
            port = int(self.port_input.text().strip())
            listen_port = int(self.listen_port_input.text().strip())
        except Exception:
            self.log("Invalid port value")
            return
        self.ds_sender.target_ip = ip
        self.ds_sender.target_port = port
        # set control bits
        self.ds_sender.set_mode(self.mode_combo.currentText())
        self.ds_sender.set_enabled(True)
        self.ds_sender.set_estop(False)
        # placeholder joystick
        self.ds_sender.update_joystick(axes=[0.0, 0.0], buttons=[False]*8, povs=[None])
        # start sender
        self.ds_sender.start()
        self.sending = True
        self.ind_comms.set_state(True)
        self.ind_code.set_state(True)
        self.ind_enabled.set_state(True)
        self.enable_btn.setEnabled(False)
        self.disable_btn.setEnabled(True)
        self.start_time = time.time()
        self.time_timer.start(200)
        self.log("Robot enabled (sender started)")
        # start listener bound to chosen port
        self.listener.bind_port = listen_port
        # restart listener object with new port
        if self.listening:
            self.listener.stop()
            self.listening = False
        # create new listener instance to pick up changed port cleanly
        self.listener = RobotTelemetryListener(bind_ip="0.0.0.0", bind_port=LISTENER_PORT_DEFAULT, on_packet=self.telemetry_signal.emit)

        self.listener.start()
        self.listening = True
        self.log(f"Listener started on port {listen_port}")

    def disable_robot(self):
        if not self.sending:
            return
        self.ds_sender.set_enabled(False)
        self.ds_sender.stop()
        self.sending = False
        self.ind_enabled.set_state(False)
        self.enable_btn.setEnabled(True)
        self.disable_btn.setEnabled(False)
        self.time_timer.stop()
        self.start_time = None
        self.time_label.setText("0:00")
        self.log("Robot disabled (sender stopped)")
        if self.listening:
            self.listener.stop()
            self.listening = False
            self.log("Listener stopped")

    def emergency_stop(self):
        self.log("EMERGENCY STOP requested")
        self.ds_sender.set_estop(True)
        # ensure at least one packet sent with estop
        was_running = self.sending
        if not was_running:
            self.ds_sender.start()
            time.sleep(0.05)
            self.ds_sender.stop()
        self.ds_sender.set_estop(False)
        self.disable_robot()

    def request_reboot(self):
        self.log("Reboot requested (one-shot)")
        self.ds_sender.request_reboot()

    def request_restart(self):
        self.log("Restart code requested (one-shot)")
        self.ds_sender.request_restart_code()

    # telemetry callback
    def on_telemetry_packet(self, parsed: dict):
        if "error" in parsed:
            self.log(f"Listener error: {parsed['error']}")
            return

        # sequence
        seq = parsed.get("seq")
        self.lbl_seq.setText(str(seq) if seq is not None else "N/A")
        self.packet_counter += 1

        # battery
        battery = parsed.get("battery")
        if battery is not None:
            self.lbl_battery.setText(f"{battery:.2f} V")
        else:
            self.lbl_battery.setText("N/A")

        # last packet timestamp
        self.lbl_last_packet.setText(datetime.datetime.now().strftime("%H:%M:%S"))

        # status fields
        status = parsed.get("status", {})

        self.ind_e_stop.set_state(status.get("e_stop", False))
        self.ind_enabled.set_state(status.get("brownout", False))
        self.ind_code_start.set_state(status.get("code_start", False))
        self.ind_enabled.set_state(status.get("enabled", False))

        mode = status.get("mode")
        mode_name = {0: "Teleop", 1: "Test", 2: "Autonomous"}.get(mode, "Unknown")
        self.lbl_mode.setText(mode_name)

        # trace fields
        trace = parsed.get("trace", {})
        self.ind_code.setText(str(trace.get("robot_code", False)))

        # print(parsed)

        # tags (empty for now)
        tags = parsed.get("tags", [])
        # if tags:
        #     for t in tags:
        #         self.log(f"Tag 0x{t['id']:02X}: {t.get('parsed')} raw={t.get('raw')}")

# -----------------------------
# Main
# -----------------------------


def randomize_case(s):
    return ''.join(random.choice([char.upper(), char.lower()]) for char in s)

def main():
    base_string = "i understand this is unsafe"
    randomized_string = randomize_case(base_string)
    print(f"This script is strictly for experiment and COULD CAUSE A RUN AWAY ROBOT. Using this script at an official event is illegal.")
    print(f"Please read through README before using this script")
    print(f"")
    print(f"Please enter the following phrase exactly as shown to proceed: {randomized_string}")
    in_str = input("Enter phrase: ").strip()
    if in_str == "e":
        app = QApplication(sys.argv)
        w = ModernDS()
        w.show()
    elif (in_str != randomized_string):
        print("Input did not match. Exiting.")
        sys.exit(1)
    
    else:
        app = QApplication(sys.argv)
        w = ModernDS()
        w.show()
    print("Correct input. Proceeding...")
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
