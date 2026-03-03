"""
ARC Extension Board - Real-time USB CDC Monitor & Logger
- 데이터 구조: UsbStreamData_t (H10 + Trunk IMU + IMU Hub 6 sensors)
- 제어: "AGRB MON START" / "AGRB MON STOP"
- 실시간 그래프 6개 + CSV 자동 저장 + 리뷰 뷰어(14 graphs)
"""

import sys
import os
import glob
import csv
import struct
import time
import serial
import serial.tools.list_ports
from PyQt5 import QtCore, QtWidgets
from PyQt5.QtCore import pyqtSignal, pyqtSlot

import pyqtgraph as pg
import numpy as np
from scipy import interpolate

# ===================== 설정 =====================
UPDATE_INTERVAL_MS = 30
WINDOW_SIZE_SAMPLES = 1000
FLUSH_EVERY = 500

SOF_VALUE = 0xAA55
DEFAULT_BAUD = 921600
DEFAULT_TIMEOUT = 1.0  # seconds

DEFAULT_SAVE_FOLDER = "S017"

# 출력 파일명 목록 (드롭다운 선택용, 필요시 추가/삭제)
OUTPUT_FILE_NAMES = [
    "IMU_Npose.csv",
    "level_075mps_lv0_01.csv",
    "level_075mps_lv0_02.csv",
    "level_100mps_lv0_01.csv",
    "level_100mps_lv0_02.csv",
    "level_125mps_lv0_01.csv",
    "level_125mps_lv0_02.csv",
    "stopandgo_lv0_01.csv",
    "stopandgo_lv0_02.csv",
    "stopandgo_lv0_03.csv",
    "stopandgo_lv0_04.csv",
    "accel_sine_lv0_01.csv",
    "accel_sine_lv0_02.csv",
    "stand_lv0_01.csv",
    "stand_lv0_02.csv",
    "dynamicpose_lv0_01.csv",
    "dynamicpose_lv0_02.csv",
    "incline_10deg_lv0_01.csv",
    "incline_10deg_lv0_02.csv",
    "decline_5deg_lv0_01.csv",
    "decline_5deg_lv0_02.csv",
]

# 후처리(0 스파이크 보간)에서 제외할 센서
EXCLUDED_POSTPROCESS_SENSORS = {}

# SavingData_t 구조 (user_app_offset.c 기준):
# uint16_t sof, len
# uint32_t loopCnt
# uint8_t h10Mode, h10AssistLevel, SmartAssist, imuHubConnected
# 8 floats (H10 robot)
# 12 floats (H10 IMU)
# 10 floats (Trunk IMU)
# 28 floats (FSR Left 14 + Right 14)
# 60 floats (IMU Hub 6×10)
# uint16_t crc

STRUCT_FMT = '<IBBBB118f'  # loopCnt + 4×uint8 + 118 floats (8+12+10+28+60)
PAYLOAD_SIZE = struct.calcsize(STRUCT_FMT)
EXPECTED_TOTAL_PACKET_SIZE = 2 + 2 + PAYLOAD_SIZE + 2

# 디버깅: 계산된 크기 출력
print(f"[DEBUG] PAYLOAD_SIZE = {PAYLOAD_SIZE} bytes")
print(f"[DEBUG] EXPECTED_TOTAL_PACKET_SIZE = {EXPECTED_TOTAL_PACKET_SIZE} bytes")

CSV_HEADER = (
    "LoopCnt,H10Mode,H10AssistLevel,SmartAssist,ImuHubConnected,"
    "LeftHipAngle,RightHipAngle,LeftThighAngle,RightThighAngle,"
    "LeftHipTorque,RightHipTorque,LeftHipMotorAngle,RightHipMotorAngle,"
    "LeftHipImuGlobalAccX,LeftHipImuGlobalAccY,LeftHipImuGlobalAccZ,"
    "LeftHipImuGlobalGyrX,LeftHipImuGlobalGyrY,LeftHipImuGlobalGyrZ,"
    "RightHipImuGlobalAccX,RightHipImuGlobalAccY,RightHipImuGlobalAccZ,"
    "RightHipImuGlobalGyrX,RightHipImuGlobalGyrY,RightHipImuGlobalGyrZ,"
    "TrunkIMU_LocalAccX,TrunkIMU_LocalAccY,TrunkIMU_LocalAccZ,"
    "TrunkIMU_LocalGyrX,TrunkIMU_LocalGyrY,TrunkIMU_LocalGyrZ,"
    "TrunkIMU_QuatW,TrunkIMU_QuatX,TrunkIMU_QuatY,TrunkIMU_QuatZ,"
    "LeftFSR1,LeftFSR2,LeftFSR3,LeftFSR4,LeftFSR5,LeftFSR6,"
    "LeftFSR7,LeftFSR8,LeftFSR9,LeftFSR10,LeftFSR11,LeftFSR12,"
    "LeftFSR13,LeftFSR14,"
    "RightFSR1,RightFSR2,RightFSR3,RightFSR4,RightFSR5,RightFSR6,"
    "RightFSR7,RightFSR8,RightFSR9,RightFSR10,RightFSR11,RightFSR12,"
    "RightFSR13,RightFSR14,"
    "L_THIGH_IMU_QuatW,L_THIGH_IMU_QuatX,L_THIGH_IMU_QuatY,L_THIGH_IMU_QuatZ,"
    "L_THIGH_IMU_AccX,L_THIGH_IMU_AccY,L_THIGH_IMU_AccZ,"
    "L_THIGH_IMU_GyrX,L_THIGH_IMU_GyrY,L_THIGH_IMU_GyrZ,"
    "L_SHANK_IMU_QuatW,L_SHANK_IMU_QuatX,L_SHANK_IMU_QuatY,L_SHANK_IMU_QuatZ,"
    "L_SHANK_IMU_AccX,L_SHANK_IMU_AccY,L_SHANK_IMU_AccZ,"
    "L_SHANK_IMU_GyrX,L_SHANK_IMU_GyrY,L_SHANK_IMU_GyrZ,"
    "L_FOOT_IMU_QuatW,L_FOOT_IMU_QuatX,L_FOOT_IMU_QuatY,L_FOOT_IMU_QuatZ,"
    "L_FOOT_IMU_AccX,L_FOOT_IMU_AccY,L_FOOT_IMU_AccZ,"
    "L_FOOT_IMU_GyrX,L_FOOT_IMU_GyrY,L_FOOT_IMU_GyrZ,"
    "R_THIGH_IMU_QuatW,R_THIGH_IMU_QuatX,R_THIGH_IMU_QuatY,R_THIGH_IMU_QuatZ,"
    "R_THIGH_IMU_AccX,R_THIGH_IMU_AccY,R_THIGH_IMU_AccZ,"
    "R_THIGH_IMU_GyrX,R_THIGH_IMU_GyrY,R_THIGH_IMU_GyrZ,"
    "R_SHANK_IMU_QuatW,R_SHANK_IMU_QuatX,R_SHANK_IMU_QuatY,R_SHANK_IMU_QuatZ,"
    "R_SHANK_IMU_AccX,R_SHANK_IMU_AccY,R_SHANK_IMU_AccZ,"
    "R_SHANK_IMU_GyrX,R_SHANK_IMU_GyrY,R_SHANK_IMU_GyrZ,"
    "R_FOOT_IMU_QuatW,R_FOOT_IMU_QuatX,R_FOOT_IMU_QuatY,R_FOOT_IMU_QuatZ,"
    "R_FOOT_IMU_AccX,R_FOOT_IMU_AccY,R_FOOT_IMU_AccZ,"
    "R_FOOT_IMU_GyrX,R_FOOT_IMU_GyrY,R_FOOT_IMU_GyrZ"
)
CSV_COLS = CSV_HEADER.split(",")


def crc16_modbus(data: bytes, init_val: int = 0xFFFF) -> int:
    crc = init_val
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
            crc &= 0xFFFF
    return crc


def decode_packet(data_tuple):
    """[I,B,B,B,B] + 118f → list[123]"""
    expected_len = 5 + 118
    if len(data_tuple) != expected_len:
        raise ValueError(f"Unexpected data length: {len(data_tuple)}")
    row = [0] * expected_len
    row[0] = int(data_tuple[0])  # LoopCnt
    row[1] = int(data_tuple[1])  # H10Mode
    row[2] = int(data_tuple[2])  # H10AssistLevel
    row[3] = int(data_tuple[3])  # SmartAssist
    row[4] = int(data_tuple[4])  # ImuHubConnected
    for i in range(5, expected_len):
        row[i] = float(data_tuple[i])
    return row


def row_to_csv_line(row):
    int_indices = {0, 1, 2, 3, 4}
    parts = []
    for idx, val in enumerate(row):
        if idx in int_indices:
            parts.append(str(int(val)))
        else:
            parts.append(f"{float(val):.4f}")
    return ",".join(parts)


def detect_zero_spike_rows(data_array):
    """
    IMU Hub 6개 센서 각각에서 값이 모두 0인 행 검출
    Returns: dict {sensor_id: [row_indices]}, dict {sensor_id: [col_indices]}
    """
    # IMU Hub 데이터 컬럼: L_THIGH, L_SHANK, L_FOOT, R_THIGH, R_SHANK, R_FOOT 각각 10개 (Quat 4 + Acc 3 + Gyr 3)
    col_names = CSV_COLS
    
    # 센서 이름 매핑
    sensor_names = ["L_THIGH_IMU", "L_SHANK_IMU", "L_FOOT_IMU", "R_THIGH_IMU", "R_SHANK_IMU", "R_FOOT_IMU"]
    
    # 센서별로 컬럼 인덱스 저장 (후처리 제외 센서는 건너뜀)
    sensor_columns = {}
    for sensor_id in range(6):
        cols = []
        sensor_name = sensor_names[sensor_id]
        if sensor_name in EXCLUDED_POSTPROCESS_SENSORS:
            continue
        for suffix in ["QuatW", "QuatX", "QuatY", "QuatZ", "AccX", "AccY", "AccZ", "GyrX", "GyrY", "GyrZ"]:
            name = f"{sensor_name}_{suffix}"
            if name in col_names:
                cols.append(col_names.index(name))
        sensor_columns[sensor_id] = cols
    
    # 센서별로 0 스파이크 검출
    zero_rows_per_sensor = {}
    total_zeros = 0
    
    for sensor_id, cols in sensor_columns.items():
        zero_rows = []
        for row_idx in range(data_array.shape[0]):
            # 해당 센서의 모든 값이 0인지 확인
            sensor_all_zero = True
            for col_idx in cols:
                if col_idx < data_array.shape[1]:
                    if abs(data_array[row_idx, col_idx]) > 1e-6:
                        sensor_all_zero = False
                        break
            if sensor_all_zero and len(cols) > 0:  # 컬럼이 있고 모두 0일 때만
                zero_rows.append(row_idx)
        
        if zero_rows:
            zero_rows_per_sensor[sensor_id] = zero_rows
            total_zeros += len(zero_rows)
    
    return zero_rows_per_sensor, sensor_columns, total_zeros


def interpolate_zero_rows(data_array, zero_rows_per_sensor, sensor_columns):
    """
    센서별로 앞뒤 1틱 데이터로 선형보간 (간단하고 빠름)
    Returns: corrected data_array (copy)
    """
    corrected = data_array.copy()
    
    for sensor_id, zero_rows in zero_rows_per_sensor.items():
        cols = sensor_columns[sensor_id]
        
        for row_idx in zero_rows:
            prev_idx = row_idx - 1
            next_idx = row_idx + 1
            
            # 앞뒤 1틱 데이터가 있는 경우만 보간
            if prev_idx >= 0 and next_idx < data_array.shape[0]:
                # 앞뒤가 0 스파이크가 아닌지 확인 (모든 값이 0인지)
                prev_all_zero = True
                next_all_zero = True
                
                for col_idx in cols:
                    if col_idx < data_array.shape[1]:
                        if abs(data_array[prev_idx, col_idx]) > 1e-6:
                            prev_all_zero = False
                        if abs(data_array[next_idx, col_idx]) > 1e-6:
                            next_all_zero = False
                
                prev_valid = not prev_all_zero
                next_valid = not next_all_zero
                
                # 앞뒤가 모두 유효하면 평균값으로 보간
                if prev_valid and next_valid:
                    for col_idx in cols:
                        if col_idx < data_array.shape[1]:
                            prev_val = data_array[prev_idx, col_idx]
                            next_val = data_array[next_idx, col_idx]
                            corrected[row_idx, col_idx] = (prev_val + next_val) / 2.0
                elif prev_valid:
                    # 앞쪽만 유효 - 앞쪽 값 복사
                    for col_idx in cols:
                        if col_idx < data_array.shape[1]:
                            corrected[row_idx, col_idx] = data_array[prev_idx, col_idx]
                elif next_valid:
                    # 뒤쪽만 유효 - 뒤쪽 값 복사
                    for col_idx in cols:
                        if col_idx < data_array.shape[1]:
                            corrected[row_idx, col_idx] = data_array[next_idx, col_idx]
            elif prev_idx >= 0:
                # 뒤쪽 데이터 없음 - 앞쪽 값 복사
                for col_idx in cols:
                    if col_idx < data_array.shape[1]:
                        corrected[row_idx, col_idx] = data_array[prev_idx, col_idx]
            elif next_idx < data_array.shape[0]:
                # 앞쪽 데이터 없음 - 뒤쪽 값 복사
                for col_idx in cols:
                    if col_idx < data_array.shape[1]:
                        corrected[row_idx, col_idx] = data_array[next_idx, col_idx]
    
    return corrected


# ===================== Serial Worker =====================

class SerialWorker(QtCore.QObject):
    data_received = pyqtSignal(list)
    status_msg = pyqtSignal(str)
    finished = pyqtSignal()
    connection_failed = pyqtSignal(str)
    packet_error_count = pyqtSignal(int)

    def __init__(self, port_name, baudrate=DEFAULT_BAUD, timeout=DEFAULT_TIMEOUT, parent=None):
        super().__init__(parent)
        self.port_name = port_name
        self.baudrate = baudrate
        self.timeout = timeout
        self._running = False
        self._error_count = 0

    def _inc_error(self, reason: str = ""):
        self._error_count += 1
        self.packet_error_count.emit(self._error_count)
        if reason:
            self.status_msg.emit(f"Packet error #{self._error_count}: {reason}")

    @pyqtSlot()
    def run(self):
        self._running = True
        ser = None
        try:
            try:
                ser = serial.Serial(self.port_name, self.baudrate, timeout=self.timeout)
                self.status_msg.emit(f"Connected to {self.port_name}")
                self.status_msg.emit(f"Expected packet size: {EXPECTED_TOTAL_PACKET_SIZE} bytes")
            except Exception as e:
                err_msg = f"Serial open error: {e}"
                self.status_msg.emit(err_msg)
                self.connection_failed.emit(err_msg)
                return

            try:
                ser.write(b"AGRB MON START")
                self.status_msg.emit("Sent: AGRB MON START")
            except Exception as e:
                self.status_msg.emit(f"Failed to send start cmd: {e}")

            buf = bytearray()

            while self._running:
                try:
                    chunk = ser.read(4096)  # 더 큰 버퍼로 한 번에 많이 읽기
                except serial.SerialException as e:
                    self.status_msg.emit(f"Serial read error: {e}")
                    break

                if not chunk:
                    continue

                buf.extend(chunk)

                while True:
                    if len(buf) < 4:
                        break

                    sof = buf[0] | (buf[1] << 8)
                    if sof != SOF_VALUE:
                        self._inc_error(f"SOF mismatch (got 0x{sof:04X}, expected 0x{SOF_VALUE:04X}) - buf[0:4]={buf[:4].hex()}")
                        del buf[0]
                        continue

                    length = buf[2] | (buf[3] << 8)
                    if length != EXPECTED_TOTAL_PACKET_SIZE:
                        self._inc_error(f"Length mismatch (got {length}, expected {EXPECTED_TOTAL_PACKET_SIZE}) - buf[0:8]={buf[:8].hex()}")
                        del buf[0]
                        continue

                    if len(buf) < length:
                        break

                    packet = bytes(buf[:length])
                    del buf[:length]

                    had_error = False
                    reason = ""
                    row = None

                    if len(packet) != EXPECTED_TOTAL_PACKET_SIZE:
                        had_error = True
                        reason = f"Packet size mismatch"
                    else:
                        # CRC 검증 건너뜀 (펌웨어에서 CRC=0으로 설정 중)
                        # 성능 향상을 위해 CRC 계산 자체를 생략
                        
                        # CRC와 관계없이 데이터 파싱 시도
                        payload = packet[4:-2]
                        if len(payload) != PAYLOAD_SIZE:
                            had_error = True
                            reason = f"Payload size mismatch"
                        else:
                            try:
                                data_tuple = struct.unpack(STRUCT_FMT, payload)
                                row = decode_packet(data_tuple)
                            except struct.error as e:
                                had_error = True
                                reason = f"Struct unpack error: {e}"
                            except ValueError as e:
                                had_error = True
                                reason = f"Decode error: {e}"

                    if had_error:
                        self._inc_error(reason)
                        continue

                    if row is not None:
                        self.data_received.emit(row)

        finally:
            if ser is not None and ser.is_open:
                try:
                    ser.write(b"AGRB MON STOP")
                    time.sleep(0.1)
                    self.status_msg.emit("Sent: AGRB MON STOP")
                except Exception:
                    pass
                ser.close()
                self.status_msg.emit("Serial port closed")
            self.finished.emit()

    def stop(self):
        self._running = False


# ===================== CSV Review Widget (14 graphs) =====================

class CsvReviewWidget(QtWidgets.QWidget):
    def __init__(self, csv_path=None, parent=None):
        super().__init__(parent)
        self.csv_path = csv_path
        self.is_processed = False  # 후처리 여부 플래그

        self.groups = [
            ["LeftHipAngle", "RightHipAngle", "LeftHipTorque", "RightHipTorque"],  # 1
            ["LeftHipImuGlobalAccX","LeftHipImuGlobalAccY","LeftHipImuGlobalAccZ",
             "RightHipImuGlobalAccX","RightHipImuGlobalAccY","RightHipImuGlobalAccZ"],  # 2
            ["LeftHipImuGlobalGyrX","LeftHipImuGlobalGyrY","LeftHipImuGlobalGyrZ",
             "RightHipImuGlobalGyrX","RightHipImuGlobalGyrY","RightHipImuGlobalGyrZ"],  # 3
            ["TrunkIMU_LocalAccX","TrunkIMU_LocalAccY","TrunkIMU_LocalAccZ"],  # 4
            ["TrunkIMU_LocalGyrX","TrunkIMU_LocalGyrY","TrunkIMU_LocalGyrZ"],  # 5
            ["TrunkIMU_QuatW","TrunkIMU_QuatX","TrunkIMU_QuatY","TrunkIMU_QuatZ"],  # 6
            ["L_THIGH_IMU_QuatW","L_THIGH_IMU_QuatX","L_THIGH_IMU_QuatY","L_THIGH_IMU_QuatZ"],  # 7
            ["L_THIGH_IMU_AccX","L_THIGH_IMU_AccY","L_THIGH_IMU_AccZ"],  # 8
            ["L_THIGH_IMU_GyrX","L_THIGH_IMU_GyrY","L_THIGH_IMU_GyrZ"],  # 9
            ["L_SHANK_IMU_QuatW","L_SHANK_IMU_QuatX","L_SHANK_IMU_QuatY","L_SHANK_IMU_QuatZ",
             "L_FOOT_IMU_QuatW","L_FOOT_IMU_QuatX","L_FOOT_IMU_QuatY","L_FOOT_IMU_QuatZ"],  # 10
            ["R_THIGH_IMU_QuatW","R_THIGH_IMU_QuatX","R_THIGH_IMU_QuatY","R_THIGH_IMU_QuatZ",
             "R_SHANK_IMU_QuatW","R_SHANK_IMU_QuatX","R_SHANK_IMU_QuatY","R_SHANK_IMU_QuatZ"],  # 11
            ["R_FOOT_IMU_QuatW","R_FOOT_IMU_QuatX","R_FOOT_IMU_QuatY","R_FOOT_IMU_QuatZ"],  # 12
            ["L_SHANK_IMU_AccX","L_SHANK_IMU_AccY","L_SHANK_IMU_AccZ",
             "L_FOOT_IMU_AccX","L_FOOT_IMU_AccY","L_FOOT_IMU_AccZ"],  # 13
            ["R_THIGH_IMU_AccX","R_THIGH_IMU_AccY","R_THIGH_IMU_AccZ",
             "R_SHANK_IMU_AccX","R_SHANK_IMU_AccY","R_SHANK_IMU_AccZ",
             "R_FOOT_IMU_AccX","R_FOOT_IMU_AccY","R_FOOT_IMU_AccZ"],  # 14
        ]

        self.col_index = {name: i for i, name in enumerate(CSV_COLS)}
        self.loop_cnt = None
        self.data = None
        self._first_loopcnt = None
        
        if csv_path:
            self._load_csv(csv_path)
        
        self._build_ui()
        
        if csv_path:
            self._plot_all()
            self._relayout()

    def _load_csv(self, path):
        with open(path, "r", encoding="utf-8") as f:
            reader = csv.reader(f)
            header = next(reader)
        arr = np.genfromtxt(path, delimiter=",", skip_header=1)
        if arr.ndim == 1:
            arr = arr.reshape(1, -1)
        self.data = arr
        self.loop_cnt = self.data[:, 0]
        if self.loop_cnt.size > 0:
            self._first_loopcnt = self.loop_cnt[0]
        else:
            self._first_loopcnt = 0
        
        # col_index 재구성 (헤더가 변경되었을 수 있으므로)
        self.col_index = {name: i for i, name in enumerate(CSV_COLS)}

    def _build_ui(self):
        layout = QtWidgets.QHBoxLayout(self)

        left = QtWidgets.QVBoxLayout()
        layout.addLayout(left, 0)

        left.addWidget(QtWidgets.QLabel("<b>Select graphs</b>"))
        self.checkboxes = []
        for i in range(14):
            cb = QtWidgets.QCheckBox(f"Graph {i+1}")
            cb.setChecked(True)
            cb.stateChanged.connect(self._on_visibility_changed)
            self.checkboxes.append(cb)
            left.addWidget(cb)

        hl = QtWidgets.QHBoxLayout()
        btn_sel = QtWidgets.QPushButton("Select All")
        btn_clr = QtWidgets.QPushButton("Clear All")
        btn_sel.clicked.connect(lambda: self._set_all(True))
        btn_clr.clicked.connect(lambda: self._set_all(False))
        hl.addWidget(btn_sel)
        hl.addWidget(btn_clr)
        left.addLayout(hl)
        
        # 후처리 버튼 추가
        self.btn_postprocess = QtWidgets.QPushButton("후처리\n(0 스파이크 보간)")
        self.btn_postprocess.setMinimumHeight(60)
        self.btn_postprocess.clicked.connect(self._on_postprocess_clicked)
        left.addWidget(self.btn_postprocess)
        
        # 후처리 결과 레이블
        self.postprocess_label = QtWidgets.QLabel("")
        self.postprocess_label.setWordWrap(True)
        self.postprocess_label.setStyleSheet("QLabel { color: green; }")
        left.addWidget(self.postprocess_label)
        
        left.addStretch()

        self.grid = QtWidgets.QGridLayout()
        self.grid.setContentsMargins(6, 6, 6, 6)
        self.grid.setHorizontalSpacing(8)
        self.grid.setVerticalSpacing(8)
        layout.addLayout(self.grid, 1)

        self.plots = []
        for i in range(14):
            pw = pg.PlotWidget()
            pw.showGrid(x=True, y=True)
            pw.setLabel("bottom", "Time (sec)")
            pw.setLabel("left", "Value")
            pw.getAxis('left').enableAutoSIPrefix(False)
            pw.getAxis('bottom').enableAutoSIPrefix(False)
            preview = ", ".join(self.groups[i][:4]) + (", ..." if len(self.groups[i]) > 4 else "")
            pw.setTitle(f"Graph {i+1}  [{preview}]")
            self.plots.append(pw)
            r = i // 4
            c = i % 4
            self.grid.addWidget(pw, r, c)

    def _plot_all(self):
        if self.data is None or self.loop_cnt is None:
            return
            
        # x축: 상대 시간 (초) - 500Hz = 2ms/tick
        if self._first_loopcnt is None and self.loop_cnt.size > 0:
            self._first_loopcnt = self.loop_cnt[0]
        
        if self.loop_cnt.size > 0:
            deltas = np.diff(self.loop_cnt, prepend=self.loop_cnt[0])
            deltas_valid = np.where((deltas > 0) & (deltas < 50000), deltas, 0)
            x_vals = np.cumsum(deltas_valid) * 0.002  # 500Hz = 2ms = 0.002s
        else:
            x_vals = np.array([])
        
        for i, pw in enumerate(self.plots):
            pw.clear()
            # 기존 legend 제거
            if hasattr(pw, 'plotItem') and pw.plotItem is not None:
                legend_items = [item for item in pw.plotItem.items if isinstance(item, pg.LegendItem)]
                for legend in legend_items:
                    pw.plotItem.legend = None
                    pw.plotItem.removeItem(legend)
            
            legend = pw.addLegend()
            hues = max(8, len(self.groups[i]))
            for k, name in enumerate(self.groups[i]):
                if name not in self.col_index:
                    continue
                col = self.col_index[name]
                if col >= self.data.shape[1]:
                    continue
                y = self.data[:, col]
                color = pg.intColor(k, hues=hues)
                pw.plot(x_vals, y, pen=color, name=name)
            if x_vals.size > 0:
                pw.setXRange(float(x_vals[0]), float(x_vals[-1]), padding=0.02)
                pw.enableAutoRange(axis='y', enable=True)

    def _on_visibility_changed(self, state):
        for i, cb in enumerate(self.checkboxes):
            self.plots[i].setVisible(cb.isChecked())
        self._relayout()

    def _set_all(self, value: bool):
        for cb in self.checkboxes:
            cb.setChecked(value)
    
    def _on_postprocess_clicked(self):
        """후처리: 0 스파이크 선형보간"""
        # 진행 다이얼로그 생성
        progress = QtWidgets.QProgressDialog("후처리 중...", "취소", 0, 100, self)
        progress.setWindowTitle("데이터 후처리")
        progress.setWindowModality(QtCore.Qt.WindowModal)
        progress.setMinimumDuration(0)
        progress.setValue(0)
        QtWidgets.QApplication.processEvents()
        
        try:
            # 1단계: 0 스파이크 검출
            progress.setLabelText("0 스파이크 검출 중...")
            progress.setValue(20)
            QtWidgets.QApplication.processEvents()
            
            zero_rows_per_sensor, sensor_columns, total_zeros = detect_zero_spike_rows(self.data)
            
            if total_zeros == 0:
                QtWidgets.QMessageBox.information(
                    self, "후처리 완료", 
                    "IMU Hub 데이터에서 0 스파이크가 발견되지 않았습니다."
                )
                progress.close()
                return
            
            # 센서별 검출 결과 출력
            sensor_names = ["L_THIGH_IMU", "L_SHANK_IMU", "L_FOOT_IMU", "R_THIGH_IMU", "R_SHANK_IMU", "R_FOOT_IMU"]
            detail_msg = []
            for sensor_id, zero_rows in zero_rows_per_sensor.items():
                detail_msg.append(f"{sensor_names[sensor_id]}: {len(zero_rows)}개")
            
            # 2단계: 선형보간
            progress.setLabelText(f"{total_zeros}개 스파이크 보간 중...")
            progress.setValue(50)
            QtWidgets.QApplication.processEvents()
            
            corrected_data = interpolate_zero_rows(self.data, zero_rows_per_sensor, sensor_columns)
            
            # 3단계: 파일 저장
            progress.setLabelText("후처리 결과 저장 중...")
            progress.setValue(70)
            QtWidgets.QApplication.processEvents()
            
            base_name = os.path.splitext(self.csv_path)[0]
            processed_path = f"{base_name}_processed.csv"
            
            with open(processed_path, "w", encoding="utf-8", newline="") as f:
                f.write(CSV_HEADER + "\n")
                for row in corrected_data:
                    # int 컬럼과 float 컬럼 구분
                    parts = []
                    for idx, val in enumerate(row):
                        if idx in {0, 1, 2, 3}:
                            parts.append(str(int(val)))
                        else:
                            parts.append(f"{float(val):.4f}")
                    f.write(",".join(parts) + "\n")
            
            # 4단계: 데이터 다시 로드
            progress.setLabelText("후처리 데이터 로딩 중...")
            progress.setValue(90)
            QtWidgets.QApplication.processEvents()
            
            self.csv_path = processed_path
            self._load_csv(processed_path)
            self.is_processed = True
            
            # 5단계: 그래프 다시 그리기
            progress.setLabelText("그래프 업데이트 중...")
            progress.setValue(95)
            QtWidgets.QApplication.processEvents()
            
            # 그래프 완전히 다시 그리기
            self._plot_all()
            self._relayout()
            QtWidgets.QApplication.processEvents()
            
            progress.setValue(100)
            
            # 결과 표시
            result_msg = (
                f"✓ 후처리 완료!\n"
                f"총 수정: {total_zeros}개\n"
                + "\n".join(detail_msg) + "\n"
                f"저장:\n{os.path.basename(processed_path)}"
            )
            self.postprocess_label.setText(result_msg)
            
            QtWidgets.QMessageBox.information(
                self, "후처리 완료",
                f"0 스파이크 총 {total_zeros}개를 선형보간으로 수정했습니다.\n\n"
                + "\n".join(detail_msg) + "\n\n"
                f"저장 위치: {processed_path}"
            )
            
        except Exception as e:
            import traceback
            QtWidgets.QMessageBox.critical(
                self, "후처리 오류",
                f"후처리 중 오류가 발생했습니다:\n{e}\n\n{traceback.format_exc()}"
            )
        finally:
            progress.close()

    def _relayout(self):
        while self.grid.count():
            item = self.grid.takeAt(0)
            w = item.widget()
            if w:
                w.setParent(None)

        visible_idxs = [i for i, cb in enumerate(self.checkboxes) if cb.isChecked()]
        n = len(visible_idxs)

        if n == 0:
            return

        if n <= 4:
            cols, rows = 1, n
            max_cols, max_rows = 1, 4
        elif n <= 6:
            cols, rows = 2, 3
            max_cols, max_rows = 2, 4
        elif n <= 8:
            cols, rows = 2, 4
            max_cols, max_rows = 2, 4
        elif n <= 12:
            cols, rows = 3, 4
            max_cols, max_rows = 3, 4
        else:
            cols, rows = 4, 4
            max_cols, max_rows = 4, 4

        for order, idx in enumerate(visible_idxs[:14]):
            r = order // cols
            c = order % cols
            self.grid.addWidget(self.plots[idx], r, c)
            self.plots[idx].setVisible(True)

        for c in range(max_cols):
            self.grid.setColumnStretch(c, 0)
        for r in range(max_rows):
            self.grid.setRowStretch(r, 0)
        for c in range(cols):
            self.grid.setColumnStretch(c, 1)
        for r in range(rows):
            self.grid.setRowStretch(r, 1)

        for i in range(14):
            if i not in visible_idxs:
                self.plots[i].setVisible(False)


# ===================== Main Window (Tabbed Interface) =====================

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ARC Extension Board - Real-time Monitor & Review")
        self.resize(1500, 900)

        self.data_all = []
        self._recv_count = 0
        self._first_loopcnt = None  # 첫 번째 LoopCnt 값 저장
        self._prev_loopcnt = None   # 이전 LoopCnt (sync 판별용)
        self._sync_active = False   # sync ON 상태 플래그
        self._drop_detected = False # 패킷 드롭 감지 플래그
        self._drop_gap = 0          # 현재 누적 드롭 수
        self.default_idx = CSV_COLS.index("LeftHipAngle")

        self.serial_thread = None
        self.serial_worker = None

        self.log_file = None
        self._written_lines = 0
        self._last_log_path = None
        self._pending_lines = []

        self.plot_groups = []
        self._build_plot_groups()
        self._build_ui()

        pg.setConfigOptions(antialias=True)
        self.refresh_ports()

        self.plot_timer = QtCore.QTimer(self)
        self.plot_timer.timeout.connect(self.update_all_plots)
        self.plot_timer.start(UPDATE_INTERVAL_MS)

    def _build_plot_groups(self):
        g1 = ["LeftHipAngle", "RightHipAngle"]
        g2 = ["TrunkIMU_QuatW","TrunkIMU_QuatX","TrunkIMU_QuatY","TrunkIMU_QuatZ"]
        g3 = ["L_THIGH_IMU_QuatW","L_THIGH_IMU_QuatX","L_THIGH_IMU_QuatY","L_THIGH_IMU_QuatZ"]
        g4 = ["L_SHANK_IMU_QuatW","L_SHANK_IMU_QuatX","L_SHANK_IMU_QuatY","L_SHANK_IMU_QuatZ"]
        g5 = ["L_FOOT_IMU_QuatW","L_FOOT_IMU_QuatX","L_FOOT_IMU_QuatY","L_FOOT_IMU_QuatZ"]
        g6 = ["R_THIGH_IMU_QuatW","R_THIGH_IMU_QuatX","R_THIGH_IMU_QuatY","R_THIGH_IMU_QuatZ"]
        g7 = ["R_SHANK_IMU_QuatW","R_SHANK_IMU_QuatX","R_SHANK_IMU_QuatY","R_SHANK_IMU_QuatZ"]
        g8 = ["R_FOOT_IMU_QuatW","R_FOOT_IMU_QuatX","R_FOOT_IMU_QuatY","R_FOOT_IMU_QuatZ"]
        self.plot_groups = [g1, g2, g3, g4, g5, g6, g7, g8]

    def _build_ui(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        main_layout = QtWidgets.QVBoxLayout(central)
        
        # 탭 위젯 생성
        self.tab_widget = QtWidgets.QTabWidget()
        main_layout.addWidget(self.tab_widget)
        
        # Tab 0: 실시간 모니터링
        self.monitor_tab = QtWidgets.QWidget()
        self.tab_widget.addTab(self.monitor_tab, "Real-time Monitor")
        self._build_monitor_tab()
        
        # Tab 1: CSV 리뷰 (빈 위젯으로 시작)
        self.review_widget = CsvReviewWidget(parent=self)
        self.tab_widget.addTab(self.review_widget, "CSV Review")
        
        self.status_bar = self.statusBar()
        self.status_bar.showMessage("Ready")
    
    def _build_monitor_tab(self):
        vbox = QtWidgets.QVBoxLayout(self.monitor_tab)

        top = QtWidgets.QHBoxLayout()
        vbox.addLayout(top)

        top.addWidget(QtWidgets.QLabel("Serial Port:"))
        self.combo_port = QtWidgets.QComboBox()
        top.addWidget(self.combo_port)

        self.btn_refresh = QtWidgets.QPushButton("Refresh")
        self.btn_refresh.clicked.connect(self.refresh_ports)
        top.addWidget(self.btn_refresh)

        self.btn_connect = QtWidgets.QPushButton("Connect & Start")
        self.btn_connect.clicked.connect(self.on_connect_clicked)
        top.addWidget(self.btn_connect)

        self.btn_disconnect = QtWidgets.QPushButton("Disconnect & Stop")
        self.btn_disconnect.clicked.connect(self.on_disconnect_clicked)
        self.btn_disconnect.setEnabled(False)
        top.addWidget(self.btn_disconnect)

        top.addStretch()

        top.addWidget(QtWidgets.QLabel("Save folder:"))
        self.edit_folder = QtWidgets.QLineEdit(DEFAULT_SAVE_FOLDER)
        self.edit_folder.setMinimumWidth(80)
        self.edit_folder.setMaximumWidth(150)
        top.addWidget(self.edit_folder)

        top.addWidget(QtWidgets.QLabel("Output file:"))
        self.combo_filename = QtWidgets.QComboBox()
        self.combo_filename.setEditable(True)
        self.combo_filename.addItems(OUTPUT_FILE_NAMES)
        self.combo_filename.setMinimumWidth(300)
        top.addWidget(self.combo_filename)

        self.btn_browse = QtWidgets.QPushButton("Browse…")
        self.btn_browse.clicked.connect(self.on_browse_clicked)
        top.addWidget(self.btn_browse)
        
        self.btn_open_csv = QtWidgets.QPushButton("Open CSV")
        self.btn_open_csv.clicked.connect(self.on_open_csv_clicked)
        top.addWidget(self.btn_open_csv)

        self.mode_label = QtWidgets.QLabel("H10Mode: -, AssistLevel: -, ImuHub: -")
        font = self.mode_label.font()
        font.setPointSize(font.pointSize() + 2)
        font.setBold(True)
        self.mode_label.setFont(font)
        vbox.addWidget(self.mode_label)

        info_row = QtWidgets.QHBoxLayout()
        self.error_label = QtWidgets.QLabel("Packet errors: 0")
        info_row.addWidget(self.error_label)

        self.drop_label = QtWidgets.QLabel("")
        self.drop_label.setStyleSheet(
            "QLabel { color: red; font-weight: bold; font-size: 14px; }"
        )
        info_row.addWidget(self.drop_label)
        info_row.addStretch()
        vbox.addLayout(info_row)

        grid = QtWidgets.QGridLayout()
        vbox.addLayout(grid, stretch=1)

        # Row 0: LoopCnt - Recv 차이 그래프 (2열 span)
        self.diff_plot = pg.PlotWidget()
        self.diff_plot.showGrid(x=True, y=True)
        self.diff_plot.setLabel("bottom", "Time (sec)")
        self.diff_plot.setLabel("left", "Gap")
        self.diff_plot.getAxis('left').enableAutoSIPrefix(False)
        self.diff_plot.getAxis('bottom').enableAutoSIPrefix(False)
        self.diff_plot.setTitle("LoopCnt - Recv Gap (Packet Drop)")
        grid.addWidget(self.diff_plot, 0, 0, 1, 2)

        # Row 1~4: 기존 8개 그래프
        self.plot_widgets = []
        for i in range(8):
            pw = pg.PlotWidget()
            pw.showGrid(x=True, y=True)
            pw.setLabel("bottom", "Time (sec)")
            pw.setLabel("left", "Value")
            pw.getAxis('left').enableAutoSIPrefix(False)
            pw.getAxis('bottom').enableAutoSIPrefix(False)
            preview = ", ".join(self.plot_groups[i][:4])
            if len(self.plot_groups[i]) > 4:
                preview += ", ..."
            pw.setTitle(f"Graph {i+1}  [{preview}]")
            self.plot_widgets.append(pw)
            r = i // 2 + 1
            c = i % 2
            grid.addWidget(pw, r, c)

    def refresh_ports(self):
        self.combo_port.clear()
        ports = serial.tools.list_ports.comports()
        for p in ports:
            self.combo_port.addItem(p.device)
        if not ports:
            self.combo_port.addItem("(no ports)")
        self.status_bar.showMessage("Port list refreshed")

    def _open_log_file(self):
        folder = (self.edit_folder.text() or "").strip()
        filename = (self.combo_filename.currentText() or "").strip()
        if not filename:
            filename = OUTPUT_FILE_NAMES[0]
            self.combo_filename.setCurrentText(filename)
        if folder:
            os.makedirs(folder, exist_ok=True)
            filepath = os.path.join(folder, filename)
        else:
            filepath = filename
        self.log_file = open(filepath, "w", encoding="utf-8", newline="")
        self.log_file.write(CSV_HEADER + "\n")
        self._written_lines = 0
        self._pending_lines = []
        self._last_log_path = os.path.abspath(filepath)
        self.status_bar.showMessage(f"Logging to: {filepath}")

    def _flush_pending_lines(self):
        if self.log_file and self._pending_lines:
            try:
                self.log_file.writelines(self._pending_lines)
            except Exception as e:
                self.status_bar.showMessage(f"File write error: {e}")
            self._pending_lines.clear()

    def _close_log_file(self):
        if self.log_file:
            try:
                self._flush_pending_lines()
                self.log_file.flush()
                self.log_file.close()
            except Exception:
                pass
            self.log_file = None
            self.status_bar.showMessage("Log file closed")

    def _find_latest_csv(self):
        if self._last_log_path and os.path.isfile(self._last_log_path):
            return self._last_log_path
        pattern = os.path.join(os.getcwd(), "arc_data_log_*.csv")
        files = glob.glob(pattern)
        if not files:
            return None
        files.sort(key=lambda p: os.path.getmtime(p), reverse=True)
        return files[0]

    def _open_review_viewer(self):
        path = self._find_latest_csv()
        if not path:
            QtWidgets.QMessageBox.information(self, "Info", "No CSV file found.")
            return
        
        # CSV 리뷰 위젯에 데이터 로드
        self.review_widget.csv_path = path
        self.review_widget._load_csv(path)
        self.review_widget._plot_all()
        self.review_widget._relayout()
        
        # CSV 리뷰 탭으로 자동 전환
        self.tab_widget.setCurrentIndex(1)
        self.status_bar.showMessage(f"Review loaded: {os.path.basename(path)}")

    def on_browse_clicked(self):
        fname, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "Select output CSV", self.combo_filename.currentText(),
            "CSV Files (*.csv);;All Files (*)")
        if fname:
            self.combo_filename.setCurrentText(fname)
    
    def on_open_csv_clicked(self):
        """기존 CSV 파일을 열어서 리뷰 위젯에 표시"""
        fname, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Open CSV File", os.getcwd(),
            "CSV Files (*.csv);;All Files (*)")
        if fname and os.path.isfile(fname):
            # CSV 리뷰 위젯에 데이터 로드
            self.review_widget.csv_path = fname
            self.review_widget._load_csv(fname)
            self.review_widget._plot_all()
            self.review_widget._relayout()
            
            # CSV 리뷰 탭으로 전환
            self.tab_widget.setCurrentIndex(1)
            self.status_bar.showMessage(f"Opened: {fname}")

    def on_connect_clicked(self):
        port_name = self.combo_port.currentText()
        if not port_name or port_name == "(no ports)":
            QtWidgets.QMessageBox.warning(self, "Warning", "No port selected.")
            return
        if self.serial_thread is not None:
            QtWidgets.QMessageBox.information(self, "Info", "Already connected.")
            return

        try:
            self._open_log_file()
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "Error", f"Failed to open log:\n{e}")
            return
        
        # 데이터 초기화
        self.data_all = []
        self._first_loopcnt = None
        self._prev_loopcnt = None
        self._sync_active = False
        self._drop_detected = False
        self._drop_gap = 0
        self._recv_count = 0

        self.serial_thread = QtCore.QThread()
        self.serial_worker = SerialWorker(port_name, DEFAULT_BAUD, DEFAULT_TIMEOUT)
        self.serial_worker.moveToThread(self.serial_thread)

        self.serial_thread.started.connect(self.serial_worker.run)
        self.serial_worker.data_received.connect(self.on_data_received)
        self.serial_worker.status_msg.connect(self.on_status_msg)
        self.serial_worker.finished.connect(self.on_serial_finished)
        self.serial_worker.connection_failed.connect(self.on_connection_failed)
        self.serial_worker.packet_error_count.connect(self.on_packet_error_count)

        self.serial_thread.start()
        self.plot_timer.start(UPDATE_INTERVAL_MS)

        self.btn_connect.setEnabled(False)
        self.btn_disconnect.setEnabled(True)
        self.status_bar.showMessage(f"Connecting to {port_name}...")
        self.error_label.setText("Packet errors: 0")

    def on_disconnect_clicked(self):
        if self.serial_worker is not None:
            self.serial_worker.stop()

    @pyqtSlot(list)
    def on_data_received(self, row):
        current_loopcnt = row[0]

        # Sync 상태 판별: loopCnt가 변하면 sync ON, 안 변하면 sync OFF
        if self._prev_loopcnt is not None:
            if current_loopcnt != self._prev_loopcnt:
                self._sync_active = True
            else:
                self._sync_active = False
        self._prev_loopcnt = current_loopcnt

        # 모드/Sync 정보는 항상 업데이트 (연결 상태 확인용)
        try:
            sync_str = "ON" if self._sync_active else "OFF"
            self.mode_label.setText(
                f"H10Mode: {int(row[1])}, AssistLevel: {int(row[2])}, "
                f"ImuHub: {int(row[3])}, Sync: {sync_str}"
            )
        except Exception:
            pass

        # Sync OFF면 데이터 저장/플롯 안 함
        if not self._sync_active:
            return

        # 첫 번째 LoopCnt 저장
        if self._first_loopcnt is None:
            self._first_loopcnt = current_loopcnt

        self.data_all.append(row)
        self._recv_count += 1

        # 패킷 드롭 감지: expected = first + (recv_count - 1), gap = current - expected
        if self._first_loopcnt is not None:
            expected = self._first_loopcnt + (self._recv_count - 1)
            gap = int(current_loopcnt - expected)
            if gap > 0:
                self._drop_detected = True
                self._drop_gap = gap
                self.drop_label.setText(
                    f"Loop Count Gap Detected!  Gap: {gap}"
                )

        if self.log_file:
            try:
                line = row_to_csv_line(row) + "\n"
                self._pending_lines.append(line)
                self._written_lines += 1
                if len(self._pending_lines) >= FLUSH_EVERY:
                    self._flush_pending_lines()
            except Exception as e:
                self.status_bar.showMessage(f"File write error: {e}")

        # 상태바 업데이트는 100개마다 한 번만 (GUI 성능 개선)
        if self._recv_count % 100 == 0:
            try:
                v = row[self.default_idx]
                self.status_bar.showMessage(
                    f"Recv #{self._recv_count}, LoopCnt={row[0]}, LeftHipAngle={v:.4f}"
                )
            except Exception:
                self.status_bar.showMessage(f"Recv #{self._recv_count}")

    def update_all_plots(self):
        if not self.data_all:
            return
        rows = self.data_all[-WINDOW_SIZE_SAMPLES:]
        if not rows:
            return
        
        # x축: 누적 시간 (초) - loopCnt reset 무시 (500Hz = 2ms/tick)
        loop_cnts = [r[0] for r in rows]
        x_vals = [0.0]
        for i in range(1, len(loop_cnts)):
            delta = loop_cnts[i] - loop_cnts[i - 1]
            if 0 < delta < 50000:   # 정상 증가
                x_vals.append(x_vals[-1] + delta * 0.002)
            else:                    # reset / 정지 → x 고정
                x_vals.append(x_vals[-1])

        # 차이 그래프: (loopCnt - first_loopCnt) - recv_index
        self.diff_plot.clear()
        if self._first_loopcnt is not None:
            total_before = len(self.data_all) - len(rows)
            gap_vals = []
            for i, r in enumerate(rows):
                recv_idx = total_before + i
                expected = self._first_loopcnt + recv_idx
                gap_vals.append(int(r[0] - expected))
            self.diff_plot.plot(x_vals, gap_vals, pen=pg.mkPen('r', width=2))
            if x_vals:
                self.diff_plot.setXRange(min(x_vals), max(x_vals), padding=0.01)

        for plot_idx, pw in enumerate(self.plot_widgets):
            pw.clear()
            group = self.plot_groups[plot_idx]
            legend = pw.addLegend()
            hues = max(8, len(group))
            for k, name in enumerate(group):
                try:
                    col_idx = CSV_COLS.index(name)
                except ValueError:
                    continue
                y_vals = [r[col_idx] for r in rows]
                color = pg.intColor(k, hues=hues)
                pw.plot(x_vals, y_vals, pen=color, name=name)
            if x_vals:
                pw.setXRange(min(x_vals), max(x_vals), padding=0.01)

    @pyqtSlot(str)
    def on_status_msg(self, msg):
        self.status_bar.showMessage(msg)

    @pyqtSlot(str)
    def on_connection_failed(self, msg):
        self._close_log_file()
        QtWidgets.QMessageBox.critical(self, "Connection Failed", msg)

    @pyqtSlot()
    def on_serial_finished(self):
        self.plot_timer.stop()

        if self.serial_thread is not None:
            self.serial_thread.quit()
            self.serial_thread.wait()
            self.serial_thread = None
        self.serial_worker = None

        self._close_log_file()

        self.btn_connect.setEnabled(True)
        self.btn_disconnect.setEnabled(False)
        self.status_bar.showMessage("Disconnected")

        self._open_review_viewer()

    @pyqtSlot(int)
    def on_packet_error_count(self, n):
        self.error_label.setText(f"Packet errors: {n}")

    def closeEvent(self, event):
        if self.serial_worker is not None:
            self.serial_worker.stop()
        if self.serial_thread is not None:
            self.serial_thread.quit()
            self.serial_thread.wait()
            self.serial_thread = None
        self._close_log_file()
        event.accept()


# ===================== Main Entry =====================

def main():
    app = QtWidgets.QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
