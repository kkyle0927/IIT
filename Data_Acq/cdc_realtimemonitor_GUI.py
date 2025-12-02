import sys
import os
import glob
import csv
import struct
import time
import serial
import serial.tools.list_ports
from datetime import datetime

from PyQt5 import QtCore, QtWidgets
from PyQt5.QtCore import pyqtSignal, pyqtSlot, Qt

import pyqtgraph as pg
import numpy as np

# ===================== 0) 백엔드 고정 설정 =====================
UPDATE_INTERVAL_MS = 1000       # 실시간 그래프 갱신 주기 (ms)
WINDOW_SIZE_SAMPLES = 1000      # 실시간 플롯 최근 샘플 수

# 2ms, 패킷 약 300 bytes라고 가정하면 1초에 약 500패킷 → 500라인 모아서 한 번에 write
FLUSH_EVERY = 500               # CSV 라인 500개 모을 때마다 디스크에 기록

# ===================== 1) 패킷/CRC 설정 (펌웨어와 동일 포맷) =====================

# C 구조체
# typedef struct __attribute__((packed)) {
#     uint16_t sof;      // 0xAA55
#     uint16_t len;      // 전체 패킷 길이 (sizeof(SavingData_t))
#
#     uint32_t loopCnt;
#     uint8_t  h10Mode;
#     uint8_t  h10AssistLevel;
#     uint8_t  SmartAssist;
#
#     float ... (총 58개)
#
#     uint16_t crc;      // sof~(crc 바로 앞까지)에 대한 CRC16 (0xFFFF, poly 0xA001)
# } SavingData_t;

SOF_VALUE = 0xAA55

# payload 부분: loopCnt + h10Mode + h10AssistLevel + SmartAssist + 58 floats
STRUCT_FMT = '<IBBB58f'
PAYLOAD_SIZE = struct.calcsize(STRUCT_FMT)  # 4 + 1 + 1 + 1 + 58*4 = 239 bytes

# 전체 패킷 예상 길이: sof(2) + len(2) + payload + crc(2)
EXPECTED_TOTAL_PACKET_SIZE = 2 + 2 + PAYLOAD_SIZE + 2

CSV_HEADER = (
    "LoopCnt,H10Mode,H10AssistLevel,SmartAssist,"
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
    "RightFSR13,RightFSR14"
)
CSV_COLS = CSV_HEADER.split(",")

DEFAULT_BAUD = 921600
DEFAULT_TIMEOUT = 1.0  # sec


def crc16_modbus(data: bytes, init_val: int = 0xFFFF) -> int:
    """
    MCU의 CalcCrc16과 동일한 CRC16 (poly 0xA001, init 0xFFFF, LSB-first).
    """
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
    """[I,B,B,B] + 58f → list[62]"""
    expected_len = 4 + 58
    if len(data_tuple) != expected_len:
        raise ValueError(f"Unexpected data length: {len(data_tuple)}")
    row = [0] * expected_len
    row[0] = int(data_tuple[0])  # LoopCnt
    row[1] = int(data_tuple[1])  # H10Mode
    row[2] = int(data_tuple[2])  # H10AssistLevel
    row[3] = int(data_tuple[3])  # SmartAssist
    for i in range(4, expected_len):
        row[i] = float(data_tuple[i])
    return row


def row_to_csv_line(row):
    """row(list[62]) → CSV string (정수 4개 + float 소수점4자리)."""
    int_indices = {0, 1, 2, 3}
    parts = []
    for idx, val in enumerate(row):
        if idx in int_indices:
            parts.append(str(int(val)))
        else:
            parts.append(f"{float(val):.4f}")
    return ",".join(parts)


# ===================== 2) 시리얼 워커 =====================

class SerialWorker(QtCore.QObject):
    data_received = pyqtSignal(list)
    status_msg = pyqtSignal(str)
    finished = pyqtSignal()
    connection_failed = pyqtSignal(str)
    packet_error_count = pyqtSignal(int)   # 패킷 에러 누적 개수 신호

    def __init__(self, port_name, baudrate=DEFAULT_BAUD, timeout=DEFAULT_TIMEOUT, parent=None):
        super().__init__(parent)
        self.port_name = port_name
        self.baudrate = baudrate
        self.timeout = timeout
        self._running = False
        self._error_count = 0

    def _inc_error(self, reason: str = ""):
        """패킷 검증 실패 시 호출 (누적 개수 증가 + 신호 송신)."""
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
                self.status_msg.emit(f"Expected total packet size: {EXPECTED_TOTAL_PACKET_SIZE} bytes")
            except Exception as e:
                err_msg = f"Serial open error: {e}"
                self.status_msg.emit(err_msg)
                self.connection_failed.emit(err_msg)
                return

            # 모니터링 시작 명령
            try:
                ser.write(b"AGRB MON START")
                self.status_msg.emit("Sent: AGRB MON START")
            except Exception as e:
                self.status_msg.emit(f"Failed to send start cmd: {e}")

            buf = bytearray()

            while self._running:
                try:
                    # 한 번에 조금 더 크게 읽어 효율/지터 개선 (256 → 512)
                    chunk = ser.read(512)
                except serial.SerialException as e:
                    self.status_msg.emit(f"Serial read error: {e}")
                    break

                if not chunk:
                    continue

                buf.extend(chunk)

                # 가능한 한 많이 패킷 파싱
                while True:
                    # 최소 헤더(sof + len) 필요
                    if len(buf) < 4:
                        break

                    # SOF 체크 (little-endian uint16)
                    sof = buf[0] | (buf[1] << 8)
                    if sof != SOF_VALUE:
                        # 프레이밍 에러: 1바이트씩 밀면서 재동기화
                        self._inc_error("SOF mismatch - resync")
                        del buf[0]
                        continue

                    # len 읽기
                    length = buf[2] | (buf[3] << 8)

                    # 길이 sanity check
                    if length != EXPECTED_TOTAL_PACKET_SIZE:
                        self._inc_error(f"Length mismatch (got {length}, expected {EXPECTED_TOTAL_PACKET_SIZE})")
                        del buf[0]
                        continue

                    # 전체 패킷이 다 도착했는지 확인
                    if len(buf) < length:
                        # 아직 덜 들어옴 → 다음 read까지 대기
                        break

                    # ---- 여기서부터는 "한 패킷" 단위 처리 ----
                    packet = bytes(buf[:length])
                    del buf[:length]

                    had_error = False
                    reason = ""
                    row = None

                    # 1) 패킷 길이 검증
                    if len(packet) != EXPECTED_TOTAL_PACKET_SIZE:
                        had_error = True
                        reason = f"Packet size mismatch (got {len(packet)}, expected {EXPECTED_TOTAL_PACKET_SIZE})"
                    else:
                        # 2) CRC 검증: sof~(crc 직전까지)
                        data_without_crc = packet[:-2]
                        recv_crc = packet[-2] | (packet[-1] << 8)
                        calc_crc = crc16_modbus(data_without_crc)

                        if recv_crc != calc_crc:
                            had_error = True
                            reason = "CRC mismatch"
                        else:
                            # 3) payload 추출: [sof(2), len(2)] 이후 ~ crc 직전
                            payload = packet[4:-2]
                            if len(payload) != PAYLOAD_SIZE:
                                had_error = True
                                reason = f"Payload size mismatch (got {len(payload)}, expected {PAYLOAD_SIZE})"
                            else:
                                # 4) struct.unpack + decode
                                try:
                                    data_tuple = struct.unpack(STRUCT_FMT, payload)
                                    row = decode_packet(data_tuple)
                                except struct.error as e:
                                    had_error = True
                                    reason = f"Struct unpack error: {e}"
                                except ValueError as e:
                                    had_error = True
                                    reason = f"Decode error: {e}"

                    # 5) 이 패킷에 에러가 하나라도 있었으면 1회만 카운트
                    if had_error:
                        self._inc_error(reason)
                        continue

                    # 6) 정상 패킷이면 row emit
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


# ===================== 3) CSV 리뷰 뷰어 (8개 그래프 + 선택 레이아웃) =====================

class CsvReviewDialog(QtWidgets.QDialog):
    """
    - 가장 최근 CSV 파일을 불러와 8개 그래프(1~8번) 표시
    - 좌측 체크박스 8개로 표시/숨김 선택
    - 선택 개수에 따라 배치 변경:
        * n <= 4 → 1열(세로)
        * 5 <= n <= 6 → 2×3
        * 7 <= n <= 8 → 2×4
    - 모든 플롯 크기 균일
    """
    def __init__(self, csv_path, parent=None):
        super().__init__(parent)
        self.setWindowTitle(f"CSV Review — {os.path.basename(csv_path)}")
        self.resize(1500, 900)
        self.csv_path = csv_path

        # 8개 플롯의 변수 그룹
        self.groups = [
            ["LeftHipAngle", "RightHipAngle", "LeftHipTorque", "RightHipTorque"],  # 1
            ["LeftHipImuGlobalAccX","LeftHipImuGlobalAccY","LeftHipImuGlobalAccZ",
             "RightHipImuGlobalAccX","RightHipImuGlobalAccY","RightHipImuGlobalAccZ"],  # 2
            ["LeftHipImuGlobalGyrX","LeftHipImuGlobalGyrY","LeftHipImuGlobalGyrZ",
             "RightHipImuGlobalGyrX","RightHipImuGlobalGyrY","RightHipImuGlobalGyrZ"],  # 3
            ["TrunkIMU_LocalAccX","TrunkIMU_LocalAccY","TrunkIMU_LocalAccZ"],  # 4
            ["TrunkIMU_LocalGyrX","TrunkIMU_LocalGyrY","TrunkIMU_LocalGyrZ"],  # 5
            ["TrunkIMU_QuatW","TrunkIMU_QuatX","TrunkIMU_QuatY","TrunkIMU_QuatZ"],  # 6
            [f"LeftFSR{i}" for i in range(1, 15)],  # 7
            [f"RightFSR{i}" for i in range(1, 15)],  # 8
        ]

        # 데이터 로드
        self.col_index = {name: i for i, name in enumerate(CSV_COLS)}
        self.loop_cnt = None
        self.data = None
        self._load_csv(csv_path)

        # UI
        self._build_ui()
        self._plot_all()
        self._relayout()  # 초기 체크 상태(전부 체크)에 맞춰 균일 배치

    def _load_csv(self, path):
        # 헤더 스킵 후 빠르게 로드
        with open(path, "r", encoding="utf-8") as f:
            reader = csv.reader(f)
            header = next(reader)
        arr = np.genfromtxt(path, delimiter=",", skip_header=1)
        if arr.ndim == 1:
            arr = arr.reshape(1, -1)
        self.data = arr
        self.loop_cnt = self.data[:, 0]

    def _build_ui(self):
        layout = QtWidgets.QHBoxLayout(self)

        # 좌측: 체크박스
        left = QtWidgets.QVBoxLayout()
        layout.addLayout(left, 0)

        left.addWidget(QtWidgets.QLabel("<b>Select graphs to display</b>"))
        self.checkboxes = []
        names = [f"Graph {i+1}" for i in range(8)]
        for i, name in enumerate(names):
            cb = QtWidgets.QCheckBox(name)
            cb.setChecked(True)  # 기본 전체 표시
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
        left.addStretch()

        # 우측: 그리드 (동적)
        self.grid = QtWidgets.QGridLayout()
        self.grid.setContentsMargins(6, 6, 6, 6)
        self.grid.setHorizontalSpacing(8)
        self.grid.setVerticalSpacing(8)
        layout.addLayout(self.grid, 1)

        self.plots = []
        for i in range(8):
            pw = pg.PlotWidget()
            pw.showGrid(x=True, y=True)
            pw.setLabel("bottom", "LoopCnt")
            pw.setLabel("left", "Value")

            # 균일 사이즈 보장
            sp = pw.sizePolicy()
            sp.setHorizontalPolicy(QtWidgets.QSizePolicy.Expanding)
            sp.setVerticalPolicy(QtWidgets.QSizePolicy.Expanding)
            pw.setSizePolicy(sp)
            pw.setMinimumSize(10, 10)

            preview = ", ".join(self.groups[i][:4]) + (", ..." if len(self.groups[i]) > 4 else "")
            title_txt = f"Graph {i+1}" + (f"  [{preview}]" if self.groups[i] else "")
            pw.setTitle(title_txt)
            self.plots.append(pw)

            # 임시 배치(실제는 _relayout에서 재배치)
            r = i // 4
            c = i % 4
            self.grid.addWidget(pw, r, c)

    def _plot_all(self):
        # 정적 전체 길이 플로팅
        for i, pw in enumerate(self.plots):
            pw.clear()
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
                pw.plot(self.loop_cnt, y, pen=color, name=name)
            if self.loop_cnt.size:
                pw.setXRange(float(self.loop_cnt.min()), float(self.loop_cnt.max()), padding=0.01)

    def _on_visibility_changed(self, state):
        # 보이기/숨김 적용 + 레이아웃 갱신
        for i, cb in enumerate(self.checkboxes):
            self.plots[i].setVisible(cb.isChecked())
        self._relayout()

    def _set_all(self, value: bool):
        for cb in self.checkboxes:
            cb.setChecked(value)

    def _relayout(self):
        """
        선택 개수에 따라 균일 배치:
        - n <= 4  → 1열(세로)
        - 5 <= n <= 6 → 2×3
        - 7 <= n <= 8 → 2×4
        """
        # 1) 기존 항목 제거
        while self.grid.count():
            item = self.grid.takeAt(0)
            w = item.widget()
            if w:
                w.setParent(None)

        # 2) 보이는 플롯 인덱스
        visible_idxs = [i for i, cb in enumerate(self.checkboxes) if cb.isChecked()]
        n = len(visible_idxs)

        if n == 0:
            # 아무것도 선택 안한 경우
            for c in range(4):
                self.grid.setColumnStretch(c, 0)
            for r in range(4):
                self.grid.setRowStretch(r, 0)
            return

        # 3) 열/행 결정
        if n <= 4:
            cols = 1
            rows = n
            max_cols, max_rows = 1, 4
        elif n <= 6:
            cols = 2
            rows = 3
            max_cols, max_rows = 2, 4
        else:  # 7~8
            cols = 2
            rows = 4
            max_cols, max_rows = 2, 4

        # 4) 재배치
        for order, idx in enumerate(visible_idxs[:8]):  # 최대 8개
            r = order // cols
            c = order % cols
            self.grid.addWidget(self.plots[idx], r, c)
            self.plots[idx].setVisible(True)

        # 5) 균일 크기 유지: stretch 재설정
        for c in range(max_cols):
            self.grid.setColumnStretch(c, 0)
        for r in range(max_rows):
            self.grid.setRowStretch(r, 0)
        for c in range(cols):
            self.grid.setColumnStretch(c, 1)
        for r in range(rows):
            self.grid.setRowStretch(r, 1)

        # 6) 보이지 않는 플롯은 숨김 유지
        for i in range(8):
            if i not in visible_idxs:
                self.plots[i].setVisible(False)


# ===================== 4) 메인 GUI (실시간: 6개 플롯) =====================

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("USB CDC Real-time Monitor & Auto Logger (6 Plots)")
        self.resize(1400, 900)

        # 버퍼 & 상태
        self.data_all = []
        self._recv_count = 0

        # 기본 표시 변수(LeftHipAngle)
        self.default_idx = CSV_COLS.index("LeftHipAngle")

        # 스레드
        self.serial_thread = None
        self.serial_worker = None

        # 파일 로깅 핸들
        self.log_file = None
        self._written_lines = 0
        self._last_log_path = None  # 최근 저장 파일
        self._pending_lines = []    # FLUSH_EVERY개 모았다가 writelines

        # 플롯 그룹(6개: 1,4,5,6,7,8)
        self.plot_groups = []
        self._build_plot_groups()

        self._build_ui()
        pg.setConfigOptions(antialias=True)
        self.refresh_ports()

        # 타이머
        self.plot_timer = QtCore.QTimer(self)
        self.plot_timer.timeout.connect(self.update_all_plots)
        self.plot_timer.start(UPDATE_INTERVAL_MS)

    def _build_plot_groups(self):
        g1 = ["LeftHipAngle", "RightHipAngle", "LeftHipTorque", "RightHipTorque"]
        g4 = ["TrunkIMU_LocalAccX","TrunkIMU_LocalAccY","TrunkIMU_LocalAccZ"]
        g5 = ["TrunkIMU_LocalGyrX","TrunkIMU_LocalGyrY","TrunkIMU_LocalGyrZ"]
        g6 = ["TrunkIMU_QuatW","TrunkIMU_QuatX","TrunkIMU_QuatY","TrunkIMU_QuatZ"]
        g7 = [f"LeftFSR{i}" for i in range(1, 15)]
        g8 = [f"RightFSR{i}" for i in range(1, 15)]
        self.plot_groups = [g1, g4, g5, g6, g7, g8]

    def _build_ui(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        vbox = QtWidgets.QVBoxLayout(central)

        # 상단 컨트롤 라인
        top = QtWidgets.QHBoxLayout()
        vbox.addLayout(top)

        top.addWidget(QtWidgets.QLabel("Serial Port:"))
        self.combo_port = QtWidgets.QComboBox()
        top.addWidget(self.combo_port)

        self.btn_refresh = QtWidgets.QPushButton("Refresh")
        self.btn_refresh.clicked.connect(self.refresh_ports)
        top.addWidget(self.btn_refresh)

        self.btn_connect = QtWidgets.QPushButton("Connect & Start Logging")
        self.btn_connect.clicked.connect(self.on_connect_clicked)
        top.addWidget(self.btn_connect)

        self.btn_disconnect = QtWidgets.QPushButton("Disconnect & Stop")
        self.btn_disconnect.clicked.connect(self.on_disconnect_clicked)
        self.btn_disconnect.setEnabled(False)
        top.addWidget(self.btn_disconnect)

        top.addStretch()

        top.addWidget(QtWidgets.QLabel("Output file:"))
        default_name = f"cdc_monitoring_data_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        self.edit_filename = QtWidgets.QLineEdit(default_name)
        self.edit_filename.setMinimumWidth(320)
        top.addWidget(self.edit_filename)

        self.btn_browse = QtWidgets.QPushButton("Browse…")
        self.btn_browse.clicked.connect(self.on_browse_clicked)
        top.addWidget(self.btn_browse)

        # 모드 라벨
        self.mode_label = QtWidgets.QLabel("H10Mode: -,  H10AssistLevel: -,  SmartAssist: -")
        font = self.mode_label.font()
        font.setPointSize(font.pointSize() + 2)
        font.setBold(True)
        self.mode_label.setFont(font)
        vbox.addWidget(self.mode_label)

        # 패킷 에러 카운트 라벨
        self.error_label = QtWidgets.QLabel("Packet errors: 0")
        vbox.addWidget(self.error_label)

        # 2x3 플롯 (실시간)
        grid = QtWidgets.QGridLayout()
        vbox.addLayout(grid, stretch=1)

        self.plot_widgets = []
        for i in range(len(self.plot_groups)):
            pw = pg.PlotWidget()
            pw.showGrid(x=True, y=True)
            pw.setLabel("bottom", "LoopCnt")
            pw.setLabel("left", "Value")
            title_txt = f"Graph {i+1}"
            preview = ", ".join(self.plot_groups[i][:4])
            if len(self.plot_groups[i]) > 4:
                preview += ", ..."
            title_txt += f"  [{preview}]"
            pw.setTitle(title_txt)
            self.plot_widgets.append(pw)
            r = i // 3
            c = i % 3
            grid.addWidget(pw, r, c)

        self.status_bar = self.statusBar()
        self.status_bar.showMessage("Ready")

    # ---------- 포트/파일 ----------
    def refresh_ports(self):
        self.combo_port.clear()
        ports = serial.tools.list_ports.comports()
        for p in ports:
            self.combo_port.addItem(p.device)
        if not ports:
            self.combo_port.addItem("(no ports)")
        self.status_bar.showMessage("Port list refreshed")

    def _open_log_file(self):
        filename = (self.edit_filename.text() or "").strip()
        if not filename:
            filename = f"cdc_monitoring_data_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
            self.edit_filename.setText(filename)
        self.log_file = open(filename, "w", encoding="utf-8", newline="")
        self.log_file.write(CSV_HEADER + "\n")
        self._written_lines = 0
        self._pending_lines = []   # 새 세션 시작 시 버퍼 초기화
        self._last_log_path = os.path.abspath(filename)
        self.status_bar.showMessage(f"Logging to: {filename} (flush_every={FLUSH_EVERY})")

    def _flush_pending_lines(self):
        """메모리에 모아둔 라인들을 실제 파일에 기록."""
        if self.log_file and self._pending_lines:
            try:
                self.log_file.writelines(self._pending_lines)
            except Exception as e:
                self.status_bar.showMessage(f"File write error (flush): {e}")
            self._pending_lines.clear()

    def _close_log_file(self):
        if self.log_file:
            try:
                # 남은 버퍼 먼저 기록
                self._flush_pending_lines()
                self.log_file.flush()
                self.log_file.close()
            except Exception:
                pass
            self.log_file = None
            self.status_bar.showMessage("Log file closed")

    def _find_latest_csv(self):
        """
        최근 저장 파일 경로:
          1) 현재 세션 파일(_last_log_path)이 있으면 우선 사용
          2) 없으면 실행 폴더에서 패턴 검색
        """
        if self._last_log_path and os.path.isfile(self._last_log_path):
            return self._last_log_path
        pattern = os.path.join(os.getcwd(), "cdc_monitoring_data_log_*.csv")
        files = glob.glob(pattern)
        if not files:
            return None
        files.sort(key=lambda p: os.path.getmtime(p), reverse=True)
        return files[0]

    def _open_review_viewer(self):
        path = self._find_latest_csv()
        if not path:
            QtWidgets.QMessageBox.information(self, "Info", "No CSV file found to review.")
            return
        dlg = CsvReviewDialog(path, self)
        dlg.setModal(False)
        dlg.show()

    # ---------- 버튼 ----------
    def on_browse_clicked(self):
        fname, _ = QtWidgets.QFileDialog.getSaveFileName(
            self,
            "Select output CSV file",
            self.edit_filename.text(),
            "CSV Files (*.csv);;All Files (*)"
        )
        if fname:
            self.edit_filename.setText(fname)

    def on_connect_clicked(self):
        port_name = self.combo_port.currentText()
        if not port_name or port_name == "(no ports)":
            QtWidgets.QMessageBox.warning(self, "Warning", "No serial port selected.")
            return
        if self.serial_thread is not None:
            QtWidgets.QMessageBox.information(self, "Info", "Already connected.")
            return

        try:
            self._open_log_file()
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "Error", f"Failed to open log file:\n{e}")
            return

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

        self.btn_connect.setEnabled(False)
        self.btn_disconnect.setEnabled(True)
        self.status_bar.showMessage(f"Connecting to {port_name} ...")
        # 에러 카운터 초기화
        self.error_label.setText("Packet errors: 0")

    def on_disconnect_clicked(self):
        if self.serial_worker is not None:
            self.serial_worker.stop()

    # ---------- 데이터/플롯 ----------
    @pyqtSlot(list)
    def on_data_received(self, row):
        self.data_all.append(row)
        self._recv_count += 1

        # CSV 파일에 바로 쓰지 않고, 메모리 버퍼에 모았다가 FLUSH_EVERY마다 writelines
        if self.log_file:
            try:
                line = row_to_csv_line(row) + "\n"
                self._pending_lines.append(line)
                self._written_lines += 1
                if len(self._pending_lines) >= FLUSH_EVERY:
                    self._flush_pending_lines()
            except Exception as e:
                self.status_bar.showMessage(f"File write error: {e}")

        try:
            self.mode_label.setText(
                f"H10Mode: {int(row[1])},  H10AssistLevel: {int(row[2])},  SmartAssist: {int(row[3])}"
            )
        except Exception:
            pass

        try:
            v = row[self.default_idx]
            self.status_bar.showMessage(
                f"Recv #{self._recv_count}, LoopCnt={row[0]}, LeftHipAngle={v:.4f}"
            )
        except Exception:
            self.status_bar.showMessage(f"Recv #{self._recv_count}, LoopCnt={row[0]}")

    def update_all_plots(self):
        if not self.data_all:
            return
        rows = self.data_all[-WINDOW_SIZE_SAMPLES:]
        if not rows:
            return
        x_vals = [r[0] for r in rows]

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

    # ---------- 스레드/파일 종료 & 기타 슬롯 ----------
    @pyqtSlot(str)
    def on_status_msg(self, msg):
        self.status_bar.showMessage(msg)

    @pyqtSlot(str)
    def on_connection_failed(self, msg):
        self._close_log_file()
        QtWidgets.QMessageBox.critical(self, "Serial Connection Failed", msg)

    @pyqtSlot()
    def on_serial_finished(self):
        # 스레드 정리
        if self.serial_thread is not None:
            self.serial_thread.quit()
            self.serial_thread.wait()
            self.serial_thread = None
        self.serial_worker = None

        # 파일 닫기 (내부에서 pending flush)
        self._close_log_file()

        # 버튼 상태
        self.btn_connect.setEnabled(True)
        self.btn_disconnect.setEnabled(False)
        self.status_bar.showMessage("Disconnected")

        # 저장된 최신 CSV 열어 리뷰 뷰어 표시
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


# ===================== 5) 엔트리 포인트 =====================

def main():
    app = QtWidgets.QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
