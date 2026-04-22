#!/usr/bin/env python3
from __future__ import annotations

import math
import struct
import sys
import time
from collections import deque
from dataclasses import dataclass
from typing import Deque, Dict, Iterable, Optional, Tuple

import pyqtgraph as pg
import serial
import serial.tools.list_ports
from PyQt5.QtCore import QObject, QThread, QTimer, Qt, pyqtSignal, pyqtSlot
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFormLayout,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSpinBox,
    QSplitter,
    QTabWidget,
    QTextEdit,
    QToolButton,
    QVBoxLayout,
    QWidget,
)


TAIL = b"\xff\xff"
WORK_HEADER = b"\x5a\xa5"
PRESET_HEADER = b"\x6a\xa6"
MOTOR_PID_HEADER = b"\x7a\xa7"
MOTOR_LEVEL_PID_HEADER = b"\x7a\xb7"
HEAT_PID_HEADER = b"\x9a\xa9"

WORK_COMMANDS: Dict[str, Tuple[int, str]] = {
    "heat_start": (0x1041, "加热开始"),
    "heat_stop": (0x1030, "加热停止"),
    "press_start": (0x1005, "挤压开始"),
    "press_stop": (0x1034, "挤压停止"),
    "auto_start": (0x1037, "自动开始"),
    "auto_stop": (0x1038, "自动停止"),
    "soft_button": (0x1040, "软按键"),
    "screen_alive": (0x1051, "屏幕存在/心跳"),
    "screen_power_on": (0x1050, "屏幕开机"),
    "screen_ack": (0x1052, "屏幕应答"),
    "set_temperature": (0x1053, "设置目标温度"),
    "set_pressure": (0x1054, "设置目标压力"),
    "factory_mode": (0x1055, "工厂模式"),
    "report_count": (0x1056, "上报次数"),
    "finish": (0x8900, "完成信号"),
}

PRESET_COMMANDS: Dict[str, Tuple[int, str]] = {
    "read_slot": (0x1042, "读取预设槽"),
    "select_slot": (0x1044, "选择预设槽"),
    "save_temperature": (0x1039, "保存预设温度"),
    "save_pressure": (0x1040, "保存预设压力"),
    "save_time": (0x1041, "保存预设时间"),
    "save_other": (0x1043, "保存其他参数"),
    "clear_eeprom": (0x1046, "清 EEPROM"),
    "clear_eye_eeprom": (0x1047, "清眼部 EEPROM"),
}

INCOMING_WORK_NAMES = {
    0x2005: "压力上报(挤压)",
    0x2047: "压力上报(自动)",
    0x2041: "温度上报(加热)",
    0x2037: "温度上报(自动)",
    0x2050: "电量上报",
    0x2051: "退出工作模式",
    0x2052: "计时开始",
    0x2055: "眼部检测",
    0x2056: "计时停止",
    0x2057: "新眼部",
    0x2058: "加热功率",
    0x2059: "负载触发状态",
    0x9000: "工作次数",
    0x9100: "眼部无效",
}

INCOMING_PRESET_NAMES = {
    0x00A0: "加热次数",
    0x00A1: "挤压次数",
    0x00A2: "自动次数",
    0x00A4: "当前预设压力",
    0x00A5: "当前预设温度",
    0x00A6: "当前预设时间",
    0x00A8: "预设温度",
    0x00A9: "预设压力",
    0x00AA: "预设时间",
    0x00B0: "眼部次数",
}

WORK_BY_CODE = {code: name for code, name in WORK_COMMANDS.values()}
PRESET_BY_CODE = {code: name for code, name in PRESET_COMMANDS.values()}


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = ((crc >> 1) ^ 0xA001) if (crc & 1) else (crc >> 1)
    return crc & 0xFFFF


def hex_dump(data: bytes) -> str:
    return " ".join(f"{b:02X}" for b in data)


def build_work_frame(cmd_type: int, value: float = 0.0) -> bytes:
    body = struct.pack("<BBB BB f", 0x5A, 0xA5, 0x0D, cmd_type >> 8, cmd_type & 0xFF, float(value))
    return body + struct.pack("<HBB", crc16_modbus(body), 0xFF, 0xFF)


def build_preset_frame(cmd_type: int, value: int = 0) -> bytes:
    body = struct.pack("<BBB BB H", 0x6A, 0xA6, 0x0B, cmd_type >> 8, cmd_type & 0xFF, int(value) & 0xFFFF)
    return body + struct.pack("<HBB", crc16_modbus(body), 0xFF, 0xFF)


def build_pid_frame(header: bytes, kp: float, ki: float, kd: float, setpoint: float) -> bytes:
    body = header + struct.pack("<Bffff", 0x17, float(kp), float(ki), float(kd), float(setpoint))
    return body + struct.pack("<HBB", crc16_modbus(body), 0xFF, 0xFF)


@dataclass
class ParsedFrame:
    header: bytes
    raw: bytes
    valid_crc: bool
    cmd_type: Optional[int] = None
    value: Optional[float] = None
    preset_value: Optional[int] = None
    pid_values: Optional[Tuple[float, float, float, float]] = None

    @property
    def kind(self) -> str:
        if self.header == WORK_HEADER:
            return "WORK"
        if self.header == PRESET_HEADER:
            return "PRESET"
        if self.header == MOTOR_PID_HEADER:
            return "MOTOR_PID"
        if self.header == MOTOR_LEVEL_PID_HEADER:
            return "MOTOR_LEVEL_PID"
        if self.header == HEAT_PID_HEADER:
            return "HEAT_PID"
        return "UNKNOWN"

    @property
    def name(self) -> str:
        if self.kind == "WORK" and self.cmd_type is not None:
            return INCOMING_WORK_NAMES.get(self.cmd_type, WORK_BY_CODE.get(self.cmd_type, f"0x{self.cmd_type:04X}"))
        if self.kind == "PRESET" and self.cmd_type is not None:
            return INCOMING_PRESET_NAMES.get(self.cmd_type, PRESET_BY_CODE.get(self.cmd_type, f"0x{self.cmd_type:04X}"))
        if self.kind == "MOTOR_PID":
            return "电机 PID"
        if self.kind == "MOTOR_LEVEL_PID":
            return "电机分档 PID"
        if self.kind == "HEAT_PID":
            return "加热 PID"
        return "未知帧"


class ProtocolParser:
    def __init__(self) -> None:
        self.buffer = bytearray()

    def feed(self, data: bytes) -> Iterable[ParsedFrame]:
        self.buffer.extend(data)
        frames = []
        while True:
            start = self._find_next_header()
            if start is None:
                if len(self.buffer) > 2:
                    del self.buffer[:-2]
                break
            if start:
                del self.buffer[:start]
            tail = self.buffer.find(TAIL, 2)
            if tail < 0:
                break
            raw = bytes(self.buffer[: tail + 2])
            del self.buffer[: tail + 2]
            frame = self._parse(raw)
            if frame:
                frames.append(frame)
        return frames

    def _find_next_header(self) -> Optional[int]:
        indexes = [self.buffer.find(h) for h in (WORK_HEADER, PRESET_HEADER, MOTOR_PID_HEADER, MOTOR_LEVEL_PID_HEADER, HEAT_PID_HEADER)]
        indexes = [i for i in indexes if i >= 0]
        return min(indexes) if indexes else None

    def _parse(self, raw: bytes) -> Optional[ParsedFrame]:
        if len(raw) < 7:
            return None
        header = raw[:2]
        rx_crc = struct.unpack_from("<H", raw, len(raw) - 4)[0]
        valid = crc16_modbus(raw[:-4]) == rx_crc
        try:
            if header == WORK_HEADER and len(raw) == 13:
                _, _, _, hi, lo, value, _, _, _ = struct.unpack("<BBB BB f H BB", raw)
                return ParsedFrame(header, raw, valid, (hi << 8) | lo, value=value)
            if header == PRESET_HEADER and len(raw) == 11:
                _, _, _, hi, lo, value, _, _, _ = struct.unpack("<BBB BB H H BB", raw)
                return ParsedFrame(header, raw, valid, (hi << 8) | lo, preset_value=value)
            if header in (MOTOR_PID_HEADER, MOTOR_LEVEL_PID_HEADER, HEAT_PID_HEADER) and len(raw) == 23:
                return ParsedFrame(header, raw, valid, pid_values=struct.unpack_from("<ffff", raw, 3))
        except struct.error:
            return ParsedFrame(header, raw, False)
        return ParsedFrame(header, raw, valid)


class SerialWorker(QObject):
    connected = pyqtSignal(str)
    disconnected = pyqtSignal(str)
    received = pyqtSignal(object)
    raw_received = pyqtSignal(bytes)
    raw_sent = pyqtSignal(bytes, str)
    error = pyqtSignal(str)

    def __init__(self) -> None:
        super().__init__()
        self.serial: Optional[serial.Serial] = None
        self.timer: Optional[QTimer] = None
        self.parser = ProtocolParser()

    @pyqtSlot(str, int)
    def open(self, port: str, baudrate: int) -> None:
        self.close()
        try:
            self.serial = serial.Serial(port=port, baudrate=baudrate, timeout=0, write_timeout=0.2)
        except serial.SerialException as exc:
            self.error.emit(f"打开串口失败: {exc}")
            return
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.poll)
        self.timer.start(20)
        self.connected.emit(f"{port} @ {baudrate}")

    @pyqtSlot()
    def close(self) -> None:
        if self.timer:
            self.timer.stop()
            self.timer.deleteLater()
            self.timer = None
        if self.serial:
            port = self.serial.port
            try:
                self.serial.close()
            finally:
                self.serial = None
                self.disconnected.emit(str(port))

    @pyqtSlot(bytes, str)
    def send(self, frame: bytes, description: str = "") -> None:
        if not self.serial or not self.serial.is_open:
            self.error.emit("串口未连接")
            return
        try:
            self.serial.write(frame)
            self.raw_sent.emit(frame, description)
        except serial.SerialException as exc:
            self.error.emit(f"发送失败: {exc}")

    @pyqtSlot()
    def poll(self) -> None:
        if not self.serial:
            return
        try:
            count = self.serial.in_waiting
            data = self.serial.read(count) if count else b""
        except serial.SerialException as exc:
            self.error.emit(f"接收失败: {exc}")
            self.close()
            return
        if not data:
            return
        self.raw_received.emit(data)
        for frame in self.parser.feed(data):
            self.received.emit(frame)


class ValueCard(QFrame):
    def __init__(self, title: str, unit: str = "") -> None:
        super().__init__()
        self.unit = unit
        self.setObjectName("ValueCard")
        layout = QVBoxLayout(self)
        layout.setContentsMargins(12, 10, 12, 10)
        self.title_label = QLabel(title)
        self.title_label.setObjectName("CardTitle")
        self.value_label = QLabel("--")
        self.value_label.setObjectName("CardValue")
        layout.addWidget(self.title_label)
        layout.addWidget(self.value_label)

    def set_value(self, value: object) -> None:
        text = f"{value:.2f}" if isinstance(value, float) else str(value)
        self.value_label.setText(f"{text} {self.unit}" if self.unit else text)


class MainWindow(QMainWindow):
    send_frame = pyqtSignal(bytes, str)
    open_serial = pyqtSignal(str, int)
    close_serial = pyqtSignal()

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("SLK-01 调试上位机")
        self.resize(1320, 820)
        self.start_time = time.monotonic()
        self.last_eye_time: Optional[float] = None
        self.temperature_points: Deque[Tuple[float, float]] = deque(maxlen=2400)
        self.pressure_points: Deque[Tuple[float, float]] = deque(maxlen=2400)
        self.stress_sent_count = 0
        self.stress_rx_count = 0
        self.stress_error_count = 0
        self.stress_no_rx_count = 0
        self.stress_last_rx_count = 0
        self.stress_sequence_index = 0
        self._setup_worker()
        self._setup_ui()
        self._setup_timers()
        self.refresh_ports()

    def _setup_worker(self) -> None:
        self.worker_thread = QThread(self)
        self.worker = SerialWorker()
        self.worker.moveToThread(self.worker_thread)
        self.worker.connected.connect(self.on_connected)
        self.worker.disconnected.connect(self.on_disconnected)
        self.worker.received.connect(self.on_frame)
        self.worker.raw_received.connect(self.on_raw_received)
        self.worker.raw_sent.connect(self.on_raw_sent)
        self.worker.error.connect(self.on_error)
        self.open_serial.connect(self.worker.open)
        self.close_serial.connect(self.worker.close)
        self.send_frame.connect(self.worker.send)
        self.worker_thread.start()

    def _setup_ui(self) -> None:
        root = QWidget()
        self.setCentralWidget(root)
        root_layout = QVBoxLayout(root)
        root_layout.setContentsMargins(12, 12, 12, 12)
        root_layout.setSpacing(10)
        root_layout.addWidget(self._build_top_bar())

        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(self._build_left_panel())
        splitter.addWidget(self._build_right_panel())
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)
        splitter.setSizes([410, 900])
        root_layout.addWidget(splitter, 1)
        self.statusBar().showMessage("未连接")
        self._apply_style()

    def _build_top_bar(self) -> QWidget:
        bar = QFrame()
        bar.setObjectName("TopBar")
        layout = QHBoxLayout(bar)
        layout.setContentsMargins(12, 10, 12, 10)
        layout.setSpacing(8)
        title = QLabel("SLK-01 调试上位机")
        title.setObjectName("AppTitle")
        layout.addWidget(title)
        layout.addStretch(1)
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(180)
        self.refresh_button = QToolButton()
        self.refresh_button.setText("刷新")
        self.refresh_button.clicked.connect(self.refresh_ports)
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["115200", "9600", "57600", "230400", "460800", "921600"])
        self.connect_button = QPushButton("连接")
        self.connect_button.clicked.connect(self.toggle_connection)
        self.connection_label = QLabel("离线")
        self.connection_label.setObjectName("ConnectionBadge")
        layout.addWidget(QLabel("串口"))
        layout.addWidget(self.port_combo)
        layout.addWidget(self.refresh_button)
        layout.addWidget(QLabel("波特率"))
        layout.addWidget(self.baud_combo)
        layout.addWidget(self.connect_button)
        layout.addWidget(self.connection_label)
        return bar

    def _build_left_panel(self) -> QWidget:
        tabs = QTabWidget()
        tabs.setMinimumWidth(400)
        tabs.addTab(self._build_work_tab(), "控制")
        tabs.addTab(self._build_pid_tab(), "PID")
        tabs.addTab(self._build_preset_tab(), "预设")
        return tabs

    def _build_work_tab(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(10, 10, 10, 10)
        target_group = QGroupBox("目标参数")
        target_layout = QFormLayout(target_group)
        self.temp_set = self._double_spin(0, 100, 42.5, " ℃", 0.1)
        self.press_set = self._double_spin(0, 800, 150, " mmHg", 1)
        target_layout.addRow("目标温度", self.temp_set)
        target_layout.addRow("目标压力", self.press_set)
        target_buttons = QHBoxLayout()
        target_buttons.addWidget(self._button("下发温度", lambda: self.send_work("set_temperature", self.temp_set.value())))
        target_buttons.addWidget(self._button("下发压力", lambda: self.send_work("set_pressure", self.press_set.value())))
        target_layout.addRow(target_buttons)
        layout.addWidget(target_group)

        mode_group = QGroupBox("工作模式")
        grid = QGridLayout(mode_group)
        buttons = [
            ("加热开始", self.start_heat),
            ("加热停止", lambda: self.send_work("heat_stop")),
            ("挤压开始", self.start_press),
            ("挤压停止", lambda: self.send_work("press_stop")),
            ("自动开始", self.start_auto),
            ("自动停止", lambda: self.send_work("auto_stop")),
            ("软按键", lambda: self.send_work("soft_button")),
            ("完成信号", lambda: self.send_work("finish")),
            ("工厂模式", lambda: self.send_work("factory_mode")),
            ("上报次数", lambda: self.send_work("report_count")),
            ("屏幕开机", lambda: self.send_work("screen_power_on")),
            ("屏幕应答", lambda: self.send_work("screen_ack")),
        ]
        for index, (text, callback) in enumerate(buttons):
            grid.addWidget(self._button(text, callback), index // 2, index % 2)
        layout.addWidget(mode_group)

        heartbeat_group = QGroupBox("心跳")
        heartbeat_layout = QHBoxLayout(heartbeat_group)
        self.heartbeat_check = QCheckBox("自动发送屏幕存在信号")
        self.heartbeat_check.toggled.connect(self.on_heartbeat_toggled)
        self.heartbeat_interval = QSpinBox()
        self.heartbeat_interval.setRange(200, 10000)
        self.heartbeat_interval.setValue(1000)
        self.heartbeat_interval.setSuffix(" ms")
        heartbeat_layout.addWidget(self.heartbeat_check)
        heartbeat_layout.addWidget(self.heartbeat_interval)
        heartbeat_layout.addWidget(self._button("发送一次", lambda: self.send_work("screen_alive")))
        layout.addWidget(heartbeat_group)

        eye_group = QGroupBox("眼盾在线检测")
        eye_layout = QGridLayout(eye_group)
        self.eye_detail_label = QLabel("未检测")
        self.eye_detail_label.setWordWrap(True)
        eye_layout.addWidget(self.eye_detail_label, 0, 0, 1, 2)
        eye_layout.addWidget(self._button("检测眼盾", self.check_eye_shield), 1, 0)
        eye_layout.addWidget(self._button("查询眼盾次数", lambda: self.send_work("report_count")), 1, 1)
        layout.addWidget(eye_group)

        custom_group = QGroupBox("自定义工作命令")
        custom_layout = QHBoxLayout(custom_group)
        self.custom_work_cmd = QLineEdit("1051")
        self.custom_work_value = self._double_spin(-100000, 100000, 0, "", 0.1)
        custom_layout.addWidget(QLabel("0x"))
        custom_layout.addWidget(self.custom_work_cmd)
        custom_layout.addWidget(self.custom_work_value)
        custom_layout.addWidget(self._button("发送", self.send_custom_work))
        layout.addWidget(custom_group)
        stress_group = QGroupBox("命令暴力测试")
        stress_layout = QGridLayout(stress_group)
        self.stress_interval = QSpinBox()
        self.stress_interval.setRange(20, 2000)
        self.stress_interval.setValue(80)
        self.stress_interval.setSuffix(" ms")
        self.stress_total = QSpinBox()
        self.stress_total.setRange(1, 100000)
        self.stress_total.setValue(300)
        self.stress_button = QPushButton("开始暴力测试")
        self.stress_button.clicked.connect(self.toggle_stress_test)
        self.stress_status_label = QLabel("未开始")
        self.stress_status_label.setObjectName("Hint")
        self.stress_status_label.setWordWrap(True)
        stress_layout.addWidget(QLabel("间隔"), 0, 0)
        stress_layout.addWidget(self.stress_interval, 0, 1)
        stress_layout.addWidget(QLabel("总次数"), 1, 0)
        stress_layout.addWidget(self.stress_total, 1, 1)
        stress_layout.addWidget(self.stress_button, 2, 0, 1, 2)
        stress_layout.addWidget(self.stress_status_label, 3, 0, 1, 2)
        layout.addWidget(stress_group)
        layout.addStretch(1)
        return page

    def _build_pid_tab(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(10, 10, 10, 10)
        heat_group = QGroupBox("温度 PID")
        heat_form = QFormLayout(heat_group)
        self.heat_kp = self._double_spin(-10000, 10000, 15, "", 0.001, 3)
        self.heat_ki = self._double_spin(-10000, 10000, 0.4, "", 0.001, 3)
        self.heat_kd = self._double_spin(-10000, 10000, 50, "", 0.001, 3)
        self.heat_sp = self._double_spin(-10000, 10000, 42.5, " ℃", 0.1)
        heat_form.addRow("Kp", self.heat_kp)
        heat_form.addRow("Ki", self.heat_ki)
        heat_form.addRow("Kd", self.heat_kd)
        heat_form.addRow("Setpoint", self.heat_sp)
        heat_form.addRow(self._button("写入温度 PID", self.send_heat_pid))
        layout.addWidget(heat_group)

        motor_group = QGroupBox("压力/电机 PID")
        motor_form = QFormLayout(motor_group)
        self.motor_kp = self._double_spin(-10000, 10000, 300, "", 0.001, 3)
        self.motor_ki = self._double_spin(-10000, 10000, 0, "", 0.001, 3)
        self.motor_kd = self._double_spin(-10000, 10000, 0, "", 0.001, 3)
        self.motor_sp = self._double_spin(-10000, 10000, 150, " mmHg", 1)
        motor_form.addRow("Kp", self.motor_kp)
        motor_form.addRow("Ki", self.motor_ki)
        motor_form.addRow("Kd", self.motor_kd)
        motor_form.addRow("Setpoint", self.motor_sp)
        motor_form.addRow(self._button("写入压力 PID", self.send_motor_pid))
        layout.addWidget(motor_group)

        motor_level_group = QGroupBox("压力分档 PID（0x7AB7）")
        motor_level_form = QFormLayout(motor_level_group)
        self.motor_level = QComboBox()
        self.motor_level.addItems(["150", "250", "350", "450", "550"])
        self.motor_level.setCurrentText("150")
        self.motor_level_kp = self._double_spin(-10000, 10000, 200, "", 0.001, 3)
        self.motor_level_ki = self._double_spin(-10000, 10000, 0, "", 0.001, 3)
        self.motor_level_kd = self._double_spin(-10000, 10000, 0, "", 0.001, 3)
        motor_level_form.addRow("挡位(mmHg)", self.motor_level)
        motor_level_form.addRow("Kp", self.motor_level_kp)
        motor_level_form.addRow("Ki", self.motor_level_ki)
        motor_level_form.addRow("Kd", self.motor_level_kd)
        motor_level_form.addRow(self._button("写入分档 PID", self.send_motor_level_pid))
        layout.addWidget(motor_level_group)
        hint = QLabel("PID 帧直接调用固件 PID_Init；温度输出限幅由固件设为 0..255，压力/电机输出限幅由固件设为 -50000..50000。")
        hint.setObjectName("Hint")
        hint.setWordWrap(True)
        layout.addWidget(hint)
        layout.addStretch(1)
        return page

    def _build_preset_tab(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(10, 10, 10, 10)
        slot_group = QGroupBox("预设槽")
        slot_layout = QFormLayout(slot_group)
        self.slot_spin = QSpinBox()
        self.slot_spin.setRange(0, 3)
        self.slot_spin.setValue(1)
        slot_buttons = QHBoxLayout()
        slot_buttons.addWidget(self._button("读取槽", lambda: self.send_preset("read_slot", self.slot_spin.value())))
        slot_buttons.addWidget(self._button("选择槽", lambda: self.send_preset("select_slot", self.slot_spin.value())))
        slot_layout.addRow("槽位", self.slot_spin)
        slot_layout.addRow(slot_buttons)
        layout.addWidget(slot_group)

        values_group = QGroupBox("保存预设")
        values_form = QFormLayout(values_group)
        self.preset_temp = QSpinBox()
        self.preset_temp.setRange(0, 100)
        self.preset_temp.setValue(42)
        self.preset_temp.setSuffix(" ℃")
        self.preset_press = QSpinBox()
        self.preset_press.setRange(0, 1000)
        self.preset_press.setValue(150)
        self.preset_press.setSuffix(" mmHg")
        self.preset_time = QSpinBox()
        self.preset_time.setRange(0, 600)
        self.preset_time.setValue(1)
        self.preset_time.setSuffix(" s")
        values_form.addRow("温度", self.preset_temp)
        values_form.addRow("压力", self.preset_press)
        values_form.addRow("时间", self.preset_time)
        save_buttons = QGridLayout()
        save_buttons.addWidget(self._button("保存温度", lambda: self.send_preset("save_temperature", self.preset_temp.value())), 0, 0)
        save_buttons.addWidget(self._button("保存压力", lambda: self.send_preset("save_pressure", self.preset_press.value())), 0, 1)
        save_buttons.addWidget(self._button("保存时间", lambda: self.send_preset("save_time", self.preset_time.value())), 1, 0)
        save_buttons.addWidget(self._button("保存全部", self.save_all_presets), 1, 1)
        values_form.addRow(save_buttons)
        layout.addWidget(values_group)

        eeprom_group = QGroupBox("EEPROM")
        eeprom_layout = QGridLayout(eeprom_group)
        eeprom_layout.addWidget(self._button("清 EEPROM", lambda: self.confirm_and_send_preset("clear_eeprom")), 0, 0)
        eeprom_layout.addWidget(self._button("清眼部 EEPROM", lambda: self.confirm_and_send_preset("clear_eye_eeprom")), 0, 1)
        layout.addWidget(eeprom_group)

        custom_group = QGroupBox("自定义预设命令")
        custom_layout = QHBoxLayout(custom_group)
        self.custom_preset_cmd = QLineEdit("1042")
        self.custom_preset_value = QSpinBox()
        self.custom_preset_value.setRange(0, 65535)
        custom_layout.addWidget(QLabel("0x"))
        custom_layout.addWidget(self.custom_preset_cmd)
        custom_layout.addWidget(self.custom_preset_value)
        custom_layout.addWidget(self._button("发送", self.send_custom_preset))
        layout.addWidget(custom_group)
        layout.addStretch(1)
        return page

    def _build_right_panel(self) -> QWidget:
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(10)
        cards = QHBoxLayout()
        self.temp_card = ValueCard("当前温度", "℃")
        self.press_card = ValueCard("当前压力", "mmHg")
        self.power_card = ValueCard("加热功率", "%")
        self.load_card = ValueCard("负载模式")
        self.soc_card = ValueCard("电量", "%")
        self.eye_card = ValueCard("眼部状态")
        self.event_card = ValueCard("最近事件")
        for card in (self.temp_card, self.press_card, self.power_card, self.load_card, self.soc_card, self.eye_card, self.event_card):
            cards.addWidget(card)
        layout.addLayout(cards)
        self.load_detail_label = QLabel("负载条件：等待稳定")
        self.load_detail_label.setObjectName("Hint")
        self.load_detail_label.setWordWrap(True)
        layout.addWidget(self.load_detail_label)

        plot_splitter = QSplitter(Qt.Vertical)
        self.temp_plot, self.temp_curve = self._create_plot("温度曲线", "℃", "#ef476f")
        self.press_plot, self.press_curve = self._create_plot("压力曲线", "mmHg", "#118ab2")
        plot_splitter.addWidget(self.temp_plot)
        plot_splitter.addWidget(self.press_plot)
        plot_splitter.setStretchFactor(0, 1)
        plot_splitter.setStretchFactor(1, 1)
        layout.addWidget(plot_splitter, 1)

        log_group = QGroupBox("收发日志")
        log_layout = QVBoxLayout(log_group)
        toolbar = QHBoxLayout()
        toolbar.addStretch(1)
        toolbar.addWidget(self._button("清空日志", lambda: self.log.clear()))
        toolbar.addWidget(self._button("清空曲线", self.clear_curves))
        log_layout.addLayout(toolbar)
        self.log = QTextEdit()
        self.log.setReadOnly(True)
        self.log.setMinimumHeight(160)
        self.log.setFont(QFont("Consolas", 9))
        log_layout.addWidget(self.log)
        layout.addWidget(log_group)
        return panel

    def _setup_timers(self) -> None:
        self.plot_timer = QTimer(self)
        self.plot_timer.timeout.connect(self.update_plots)
        self.plot_timer.start(150)
        self.heartbeat_timer = QTimer(self)
        self.heartbeat_timer.timeout.connect(lambda: self.send_work("screen_alive", log_quiet=True))
        self.stress_timer = QTimer(self)
        self.stress_timer.timeout.connect(self.send_stress_command)
        self.eye_timeout_timer = QTimer(self)
        self.eye_timeout_timer.timeout.connect(self.update_eye_timeout)
        self.eye_timeout_timer.start(1000)

    def _apply_style(self) -> None:
        self.setStyleSheet(
            """
            QWidget { font-family: "Microsoft YaHei", "Segoe UI", sans-serif; font-size: 13px; color: #202225; }
            QMainWindow, QWidget { background: #f4f6f8; }
            #TopBar, #ValueCard, QGroupBox { background: #ffffff; border: 1px solid #d7dde5; border-radius: 6px; }
            #AppTitle { font-size: 20px; font-weight: 700; color: #1d3557; }
            #ConnectionBadge { padding: 5px 10px; border-radius: 6px; background: #eef1f5; color: #4b5563; }
            #CardTitle { color: #64748b; font-size: 12px; }
            #CardValue { color: #111827; font-size: 22px; font-weight: 700; }
            #Hint { color: #64748b; padding: 8px; }
            QPushButton, QToolButton { background: #1d4ed8; color: white; border: 0; border-radius: 5px; padding: 7px 12px; }
            QPushButton:hover, QToolButton:hover { background: #1e40af; }
            QPushButton:pressed, QToolButton:pressed { background: #1e3a8a; }
            QLineEdit, QComboBox, QSpinBox, QDoubleSpinBox {
                background: white; border: 1px solid #cbd5e1; border-radius: 5px; padding: 5px 7px; min-height: 22px;
            }
            QTabWidget::pane { border: 1px solid #d7dde5; background: white; border-radius: 6px; }
            QTabBar::tab { background: #e8edf3; padding: 8px 18px; border-top-left-radius: 5px; border-top-right-radius: 5px; margin-right: 2px; }
            QTabBar::tab:selected { background: white; color: #1d4ed8; font-weight: 700; }
            QGroupBox { margin-top: 10px; padding: 10px; font-weight: 700; }
            QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 4px; }
            QTextEdit { background: #101827; color: #dbeafe; border: 0; border-radius: 5px; }
            """
        )

    def _button(self, text: str, callback) -> QPushButton:
        button = QPushButton(text)
        button.clicked.connect(callback)
        return button

    def _double_spin(
        self,
        minimum: float,
        maximum: float,
        value: float,
        suffix: str = "",
        step: float = 1.0,
        decimals: int = 2,
    ) -> QDoubleSpinBox:
        spin = QDoubleSpinBox()
        spin.setRange(minimum, maximum)
        spin.setDecimals(decimals)
        spin.setSingleStep(step)
        spin.setValue(value)
        spin.setSuffix(suffix)
        return spin

    def _create_plot(self, title: str, unit: str, color: str):
        widget = pg.PlotWidget()
        widget.setBackground("w")
        widget.setTitle(title, color="#1f2937", size="12pt")
        widget.setLabel("left", unit)
        widget.setLabel("bottom", "时间", units="s")
        widget.showGrid(x=True, y=True, alpha=0.25)
        curve = widget.plot([], [], pen=pg.mkPen(color=color, width=2))
        return widget, curve

    def refresh_ports(self) -> None:
        current_device = self.port_combo.currentData()
        self.port_combo.clear()
        for port in serial.tools.list_ports.comports():
            self.port_combo.addItem(f"{port.device}  {port.description}", port.device)
        if current_device:
            index = self.port_combo.findData(current_device)
            if index >= 0:
                self.port_combo.setCurrentIndex(index)

    def toggle_connection(self) -> None:
        if self.connect_button.text() == "连接":
            port = self.port_combo.currentData()
            if not port:
                QMessageBox.warning(self, "串口", "没有可用串口")
                return
            self.open_serial.emit(str(port), int(self.baud_combo.currentText()))
        else:
            self.close_serial.emit()

    def on_connected(self, text: str) -> None:
        self.connect_button.setText("断开")
        self.connection_label.setText("在线")
        self.connection_label.setStyleSheet("background:#dcfce7;color:#166534;")
        self.statusBar().showMessage(f"已连接 {text}")
        self.append_log("INFO", f"已连接 {text}")

    def on_disconnected(self, port: str) -> None:
        if self.stress_timer.isActive():
            self.stop_stress_test("串口断开，暴力测试已停止")
        self.connect_button.setText("连接")
        self.connection_label.setText("离线")
        self.connection_label.setStyleSheet("")
        self.statusBar().showMessage("未连接")
        if port:
            self.append_log("INFO", f"已断开 {port}")

    def on_error(self, message: str) -> None:
        if self.stress_timer.isActive():
            self.stress_error_count += 1
            self.update_stress_status()
        self.append_log("ERR", message)
        self.statusBar().showMessage(message)

    def on_raw_sent(self, frame: bytes, description: str) -> None:
        suffix = f"  {description}" if description else ""
        self.append_log("TX", f"{hex_dump(frame)}{suffix}")

    def on_raw_received(self, data: bytes) -> None:
        self.append_log("RXRAW", hex_dump(data), faint=True)

    def append_log(self, prefix: str, text: str, faint: bool = False) -> None:
        if faint:
            return
        timestamp = time.strftime("%H:%M:%S")
        color = {"TX": "#7dd3fc", "RX": "#bbf7d0", "ERR": "#fecaca", "INFO": "#fde68a"}.get(prefix, "#dbeafe")
        self.log.append(f'<span style="color:#94a3b8;">[{timestamp}]</span> <span style="color:{color};">[{prefix}]</span> {text}')
        self.log.verticalScrollBar().setValue(self.log.verticalScrollBar().maximum())

    def update_heat_load_status(self, status: int) -> None:
        items = [
            (0x01, "启动屏蔽完成"),
            (0x02, "目标附近稳定"),
            (0x04, "负载检测已武装"),
            (0x08, "温度低于目标"),
            (0x10, "仍在目标附近"),
            (0x20, "检测到掉温"),
            (0x40, "掉温已确认"),
        ]
        active = [name for bit, name in items if status & bit]

        if status & 0x80:
            self.load_card.set_value("已进入")
            self.event_card.set_value("进入负载模式")
        elif active:
            self.load_card.set_value(f"{len(active)} 项")
        else:
            self.load_card.set_value("未触发")

        detail = " / ".join(active) if active else "等待稳定"
        if status & 0x80:
            detail = f"{detail} / 已进入负载模式"
        self.load_card.value_label.setToolTip(detail)
        self.load_detail_label.setText(f"负载条件：{detail}")

    def on_frame(self, frame: ParsedFrame) -> None:
        if self.stress_timer.isActive():
            self.stress_rx_count += 1
            self.update_stress_status()
        crc_text = "OK" if frame.valid_crc else "CRC_ERR"
        detail = frame.name
        if frame.value is not None and math.isfinite(frame.value):
            detail += f" value={frame.value:.3f}"
        if frame.preset_value is not None:
            detail += f" value={frame.preset_value}"
        if frame.pid_values is not None:
            kp, ki, kd, sp = frame.pid_values
            detail += f" Kp={kp:.3f} Ki={ki:.3f} Kd={kd:.3f} SP={sp:.2f}"
        self.append_log("RX", f"{frame.kind} {crc_text} {detail}  {hex_dump(frame.raw)}")
        if not frame.valid_crc:
            return

        now = time.monotonic() - self.start_time
        if frame.kind == "WORK" and frame.cmd_type is not None:
            if frame.cmd_type in (0x2041, 0x2037) and frame.value is not None:
                self.temperature_points.append((now, frame.value))
                self.temp_card.set_value(frame.value)
            elif frame.cmd_type in (0x2005, 0x2047) and frame.value is not None:
                self.pressure_points.append((now, frame.value))
                self.press_card.set_value(frame.value)
            elif frame.cmd_type == 0x2050 and frame.value is not None:
                self.soc_card.set_value(frame.value)
            elif frame.cmd_type == 0x2058 and frame.value is not None:
                self.power_card.set_value(frame.value)
            elif frame.cmd_type == 0x2059 and frame.value is not None:
                self.update_heat_load_status(int(frame.value))
            elif frame.cmd_type == 0x2055 and frame.value is not None:
                self.last_eye_time = time.monotonic()
                if frame.value >= 0.5:
                    self.eye_card.set_value("正常在线")
                    self.eye_detail_label.setText("眼盾检测正常，EYE_status=1，可以启动加热/挤压/自动流程。")
                else:
                    self.eye_card.set_value("异常/离线")
                    self.eye_detail_label.setText("眼盾检测异常，EYE_status=0；固件会拒绝加热、挤压和自动开始命令。")
            elif frame.cmd_type == 0x2057:
                self.last_eye_time = time.monotonic()
                self.eye_card.set_value("新眼盾")
                self.eye_detail_label.setText("收到新眼盾事件。")
            elif frame.cmd_type == 0x9100:
                self.last_eye_time = time.monotonic()
                self.eye_card.set_value("眼盾无效")
                self.eye_detail_label.setText("收到眼盾无效事件。")
            else:
                self.event_card.set_value(frame.name)
        elif frame.kind == "PRESET" and frame.cmd_type is not None:
            self.event_card.set_value(f"{frame.name}: {frame.preset_value}")

    def send_work(self, key: str, value: float = 0.0, log_quiet: bool = False) -> None:
        cmd_type, name = WORK_COMMANDS[key]
        description = "" if log_quiet else f"{name} value={value:g}"
        self.send_frame.emit(build_work_frame(cmd_type, value), description)

    def toggle_stress_test(self) -> None:
        if self.stress_timer.isActive():
            self.stop_stress_test("手动停止")
            return
        if self.connect_button.text() == "连接":
            QMessageBox.warning(self, "命令暴力测试", "请先连接串口")
            return
        self.stress_sent_count = 0
        self.stress_rx_count = 0
        self.stress_error_count = 0
        self.stress_no_rx_count = 0
        self.stress_last_rx_count = 0
        self.stress_sequence_index = 0
        self.stress_button.setText("停止暴力测试")
        self.append_log("INFO", "命令暴力测试开始：默认发送心跳/应答/目标值/上报次数，不启动加热或挤压")
        self.update_stress_status()
        self.stress_timer.start(self.stress_interval.value())
        self.send_stress_command()

    def stop_stress_test(self, reason: str = "完成") -> None:
        self.stress_timer.stop()
        self.stress_button.setText("开始暴力测试")
        self.update_stress_status(reason)
        self.append_log("INFO", f"命令暴力测试停止：{reason}")

    def update_stress_status(self, reason: str = "") -> None:
        text = (
            f"TX={self.stress_sent_count}/{self.stress_total.value()}  "
            f"RX={self.stress_rx_count}  ERR={self.stress_error_count}  "
            f"NO_RX_STEP={self.stress_no_rx_count}"
        )
        if reason:
            text = f"{reason} | {text}"
        self.stress_status_label.setText(text)

    def send_stress_command(self) -> None:
        if self.stress_sent_count >= self.stress_total.value():
            self.stop_stress_test("达到总次数")
            return

        if self.stress_sent_count > 0 and self.stress_rx_count == self.stress_last_rx_count:
            self.stress_no_rx_count += 1
        self.stress_last_rx_count = self.stress_rx_count

        sequence = (
            ("screen_alive", 0.0),
            ("screen_ack", 0.0),
            ("set_temperature", self.temp_set.value()),
            ("set_pressure", self.press_set.value()),
            ("report_count", 0.0),
        )
        key, value = sequence[self.stress_sequence_index % len(sequence)]
        self.stress_sequence_index += 1
        self.stress_sent_count += 1
        self.send_work(key, value, log_quiet=True)
        self.update_stress_status()

    def start_heat(self) -> None:
        self.send_work("heat_start")
        QTimer.singleShot(160, lambda: self.send_work("soft_button"))
        QTimer.singleShot(320, lambda: self.send_work("set_temperature", self.temp_set.value()))
        self.event_card.set_value("加热启动流程")
        self.append_log("INFO", "加热开始已按固件流程发送：PRE_HEAT -> 软按键 -> 目标温度")

    def start_press(self) -> None:
        pressure = self.press_set.value()
        self.send_work("press_start", pressure)
        QTimer.singleShot(160, lambda: self.send_work("soft_button"))
        self.event_card.set_value("挤压启动流程")
        self.append_log("INFO", "挤压开始已按固件流程发送：PRE_PRESS -> 软按键")

    def start_auto(self) -> None:
        pressure = self.press_set.value()
        self.send_work("auto_start", pressure)
        QTimer.singleShot(160, lambda: self.send_work("soft_button"))
        QTimer.singleShot(320, lambda: self.send_work("set_temperature", self.temp_set.value()))
        self.event_card.set_value("自动启动流程")
        self.append_log("INFO", "自动开始已按固件流程发送：PRE_AUTO -> 软按键 -> 目标温度")

    def check_eye_shield(self) -> None:
        self.last_eye_time = None
        self.eye_card.set_value("检测中")
        self.eye_detail_label.setText("已发送屏幕开机和心跳命令，等待下位机 0x2055 眼盾检测上报。")
        self.send_work("screen_power_on")
        QTimer.singleShot(120, lambda: self.send_work("screen_alive"))

    def send_preset(self, key: str, value: int = 0) -> None:
        cmd_type, name = PRESET_COMMANDS[key]
        self.send_frame.emit(build_preset_frame(cmd_type, value), f"{name} value={value}")

    def send_custom_work(self) -> None:
        try:
            cmd_type = int(self.custom_work_cmd.text().strip(), 16)
        except ValueError:
            QMessageBox.warning(self, "命令", "工作命令必须是 16 进制，例如 1051")
            return
        self.send_frame.emit(build_work_frame(cmd_type, self.custom_work_value.value()), f"自定义工作命令 0x{cmd_type:04X}")

    def send_custom_preset(self) -> None:
        try:
            cmd_type = int(self.custom_preset_cmd.text().strip(), 16)
        except ValueError:
            QMessageBox.warning(self, "命令", "预设命令必须是 16 进制，例如 1042")
            return
        self.send_frame.emit(build_preset_frame(cmd_type, self.custom_preset_value.value()), f"自定义预设命令 0x{cmd_type:04X}")

    def send_heat_pid(self) -> None:
        frame = build_pid_frame(HEAT_PID_HEADER, self.heat_kp.value(), self.heat_ki.value(), self.heat_kd.value(), self.heat_sp.value())
        self.send_frame.emit(frame, "写入温度 PID")

    def send_motor_pid(self) -> None:
        frame = build_pid_frame(MOTOR_PID_HEADER, self.motor_kp.value(), self.motor_ki.value(), self.motor_kd.value(), self.motor_sp.value())
        self.send_frame.emit(frame, "写入压力/电机 PID")

    def send_motor_level_pid(self) -> None:
        level = float(self.motor_level.currentText())
        frame = build_pid_frame(
            MOTOR_LEVEL_PID_HEADER,
            self.motor_level_kp.value(),
            self.motor_level_ki.value(),
            self.motor_level_kd.value(),
            level,
        )
        self.send_frame.emit(frame, f"写入分档 PID level={int(level)}")

    def save_all_presets(self) -> None:
        self.send_preset("save_temperature", self.preset_temp.value())
        self.send_preset("save_pressure", self.preset_press.value())
        self.send_preset("save_time", self.preset_time.value())

    def confirm_and_send_preset(self, key: str) -> None:
        _, name = PRESET_COMMANDS[key]
        result = QMessageBox.question(self, name, f"确认执行“{name}”？", QMessageBox.Yes | QMessageBox.No)
        if result == QMessageBox.Yes:
            self.send_preset(key, 0)

    def on_heartbeat_toggled(self, checked: bool) -> None:
        if checked:
            self.heartbeat_timer.start(self.heartbeat_interval.value())
            self.send_work("screen_alive", log_quiet=True)
        else:
            self.heartbeat_timer.stop()

    def update_plots(self) -> None:
        if self.temperature_points:
            x, y = zip(*self.temperature_points)
            self.temp_curve.setData(x, y)
            self.temp_plot.setXRange(max(0, x[-1] - 120), max(10, x[-1]), padding=0)
        if self.pressure_points:
            x, y = zip(*self.pressure_points)
            self.press_curve.setData(x, y)
            self.press_plot.setXRange(max(0, x[-1] - 120), max(10, x[-1]), padding=0)

    def update_eye_timeout(self) -> None:
        if self.last_eye_time is None:
            return
        if time.monotonic() - self.last_eye_time > 3.0:
            self.eye_card.set_value("未收到上报")
            self.eye_detail_label.setText("超过 3 秒未收到 0x2055 眼盾状态；请检查眼盾连接，或点击“检测眼盾”重新触发自检。")
            self.last_eye_time = None

    def clear_curves(self) -> None:
        self.temperature_points.clear()
        self.pressure_points.clear()
        self.temp_curve.setData([], [])
        self.press_curve.setData([], [])
        self.start_time = time.monotonic()

    def closeEvent(self, event) -> None:
        self.close_serial.emit()
        self.worker_thread.quit()
        self.worker_thread.wait(1500)
        super().closeEvent(event)


def main() -> int:
    pg.setConfigOptions(antialias=True)
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
