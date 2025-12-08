import sys
import os
import csv

from PyQt5 import QtWidgets
from PyQt5.QtCore import Qt

import pyqtgraph as pg
import numpy as np


class CsvReviewWindow(QtWidgets.QMainWindow):
    """
    - CSV 파일을 로드해서 LoopCnt 기준으로 10개 그래프 표시
    - 좌측에서 Graph 1~10 선택, Left/Right FSR 채널(1~14) 선택 가능
    - 선택 그래프 개수에 따라 레이아웃 자동 변경:
        * n <= 4   → 1열(세로)
        * 5 <= 6   → 2×3
        * 7 ~ 10   → 2×5
    - x축은 항상 LoopCnt (초기값을 0으로 정규화한 값) 사용
    """

    def __init__(self, csv_path=None, parent=None):
        super().__init__(parent)
        self.setWindowTitle("CSV Review Viewer")
        self.resize(1600, 900)

        self.csv_path = csv_path
        self.csv_cols = None
        self.data = None
        self.loop_cnt = None          # 정규화된 LoopCnt (첫값을 0으로 맞춘 것)
        self.loop_cnt_raw = None      # 원본 LoopCnt (필요하면 디버깅용으로 사용 가능)
        self.col_index = {}

        # 그룹 정의 (1~8번 그래프)
        self.groups = [
            ["LeftHipAngle", "RightHipAngle", "LeftHipTorque", "RightHipTorque"],  # Graph 1
            ["LeftHipImuGlobalAccX", "LeftHipImuGlobalAccY", "LeftHipImuGlobalAccZ",
             "RightHipImuGlobalAccX", "RightHipImuGlobalAccY", "RightHipImuGlobalAccZ"],  # 2
            ["LeftHipImuGlobalGyrX", "LeftHipImuGlobalGyrY", "LeftHipImuGlobalGyrZ",
             "RightHipImuGlobalGyrX", "RightHipImuGlobalGyrY", "RightHipImuGlobalGyrGz"],  # 3
            ["TrunkIMU_LocalAccX", "TrunkIMU_LocalAccY", "TrunkIMU_LocalAccZ"],  # 4
            ["TrunkIMU_LocalGyrX", "TrunkIMU_LocalGyrY", "TrunkIMU_LocalGyrZ"],  # 5
            ["TrunkIMU_QuatW", "TrunkIMU_QuatX", "TrunkIMU_QuatY", "TrunkIMU_QuatZ"],  # 6
            [f"LeftFSR{i}" for i in range(1, 15)],   # 7 (FSR Left 1~14)
            [f"RightFSR{i}" for i in range(1, 15)],  # 8 (FSR Right 1~14)
            [],  # 9: LoopCnt 증가량 (특수 처리)
            [],  # 10: LoopCnt (정규화된 값, x축과 동일)
        ]

        # FSR 채널 이름
        self.left_fsr_names = [f"LeftFSR{i}" for i in range(1, 15)]
        self.right_fsr_names = [f"RightFSR{i}" for i in range(1, 15)]
        self.left_fsr_cbs = []
        self.right_fsr_cbs = []

        self._build_ui()

        if csv_path is not None:
            self.load_csv(csv_path)

    # ---------------- UI 구성 ----------------
    def _build_ui(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)

        main_layout = QtWidgets.QVBoxLayout(central)

        # 상단 바: 파일 경로 + Open 버튼
        top_layout = QtWidgets.QHBoxLayout()
        main_layout.addLayout(top_layout)

        top_layout.addWidget(QtWidgets.QLabel("CSV File:"))
        self.label_path = QtWidgets.QLabel("(no file loaded)")
        self.label_path.setTextInteractionFlags(Qt.TextSelectableByMouse)
        top_layout.addWidget(self.label_path, stretch=1)

        btn_open = QtWidgets.QPushButton("Open CSV…")
        btn_open.clicked.connect(self._on_open_csv_clicked)
        top_layout.addWidget(btn_open)

        # 아래: 좌측 컨트롤 + 우측 플롯 그리드
        body_layout = QtWidgets.QHBoxLayout()
        main_layout.addLayout(body_layout, stretch=1)

        # 좌측 컨트롤
        left = QtWidgets.QVBoxLayout()
        body_layout.addLayout(left, 0)

        # Graph 선택
        left.addWidget(QtWidgets.QLabel("<b>Select graphs to display</b>"))
        self.checkboxes = []
        graph_names = [f"Graph {i+1}" for i in range(10)]
        for i, name in enumerate(graph_names):
            cb = QtWidgets.QCheckBox(name)
            cb.setChecked(True)
            cb.stateChanged.connect(self._on_graph_visibility_changed)
            self.checkboxes.append(cb)
            left.addWidget(cb)

        # Graph 9, 10 설명 붙여주기 (툴팁)
        self.checkboxes[8].setToolTip("Graph 9: LoopCnt increment (LoopCnt[n] - LoopCnt[n-1])")
        self.checkboxes[9].setToolTip("Graph 10: LoopCnt (normalized, first value = 0)")

        hl_graph = QtWidgets.QHBoxLayout()
        btn_sel_all = QtWidgets.QPushButton("Select All")
        btn_clr_all = QtWidgets.QPushButton("Clear All")
        btn_sel_all.clicked.connect(lambda: self._set_all_graphs(True))
        btn_clr_all.clicked.connect(lambda: self._set_all_graphs(False))
        hl_graph.addWidget(btn_sel_all)
        hl_graph.addWidget(btn_clr_all)
        left.addLayout(hl_graph)

        left.addSpacing(15)

        # FSR Left 선택
        gb_left_fsr = QtWidgets.QGroupBox("Left FSR channels (Graph 7)")
        v_fsr_left = QtWidgets.QVBoxLayout(gb_left_fsr)
        grid_left = QtWidgets.QGridLayout()
        v_fsr_left.addLayout(grid_left)

        for i, name in enumerate(self.left_fsr_names):
            cb = QtWidgets.QCheckBox(name.replace("Left", ""))  # "FSR1" 이런 느낌
            cb.setChecked(True)
            cb.stateChanged.connect(self._on_fsr_selection_changed)
            self.left_fsr_cbs.append(cb)
            r = i // 4
            c = i % 4
            grid_left.addWidget(cb, r, c)

        hl_left_fsr = QtWidgets.QHBoxLayout()
        btn_l_all = QtWidgets.QPushButton("All")
        btn_l_clr = QtWidgets.QPushButton("Clear")
        btn_l_all.clicked.connect(lambda: self._set_left_fsr(True))
        btn_l_clr.clicked.connect(lambda: self._set_left_fsr(False))
        v_fsr_left.addLayout(hl_left_fsr)
        hl_left_fsr.addWidget(btn_l_all)
        hl_left_fsr.addWidget(btn_l_clr)

        left.addWidget(gb_left_fsr)

        # FSR Right 선택
        gb_right_fsr = QtWidgets.QGroupBox("Right FSR channels (Graph 8)")
        v_fsr_right = QtWidgets.QVBoxLayout(gb_right_fsr)
        grid_right = QtWidgets.QGridLayout()
        v_fsr_right.addLayout(grid_right)

        for i, name in enumerate(self.right_fsr_names):
            cb = QtWidgets.QCheckBox(name.replace("Right", ""))  # "FSR1" 형식
            cb.setChecked(True)
            cb.stateChanged.connect(self._on_fsr_selection_changed)
            self.right_fsr_cbs.append(cb)
            r = i // 4
            c = i % 4
            grid_right.addWidget(cb, r, c)

        hl_right_fsr = QtWidgets.QHBoxLayout()
        btn_r_all = QtWidgets.QPushButton("All")
        btn_r_clr = QtWidgets.QPushButton("Clear")
        btn_r_all.clicked.connect(lambda: self._set_right_fsr(True))
        btn_r_clr.clicked.connect(lambda: self._set_right_fsr(False))
        v_fsr_right.addLayout(hl_right_fsr)
        hl_right_fsr.addWidget(btn_r_all)
        hl_right_fsr.addWidget(btn_r_clr)

        left.addWidget(gb_right_fsr)

        left.addStretch()

        # 우측 플롯 그리드
        self.grid = QtWidgets.QGridLayout()
        self.grid.setContentsMargins(6, 6, 6, 6)
        self.grid.setHorizontalSpacing(8)
        self.grid.setVerticalSpacing(8)
        body_layout.addLayout(self.grid, 1)

        self.plots = []
        for i in range(10):
            pw = pg.PlotWidget()
            pw.showGrid(x=True, y=True)
            pw.setLabel("bottom", "LoopCnt (normalized)")
            pw.setLabel("left", "Value")

            sp = pw.sizePolicy()
            sp.setHorizontalPolicy(QtWidgets.QSizePolicy.Expanding)
            sp.setVerticalPolicy(QtWidgets.QSizePolicy.Expanding)
            pw.setSizePolicy(sp)
            pw.setMinimumSize(10, 10)

            # 그래프별 제목
            if i == 8:
                title_txt = "Graph 9  [LoopCnt increment (ΔLoopCnt)]"
            elif i == 9:
                title_txt = "Graph 10 [LoopCnt (normalized)]"
            else:
                preview = ", ".join(self.groups[i][:4]) + (", ..." if len(self.groups[i]) > 4 else "")
                title_txt = f"Graph {i+1}" + (f"  [{preview}]" if self.groups[i] else "")

            pw.setTitle(title_txt)

            self.plots.append(pw)

            # 임시 배치 (실제는 _relayout에서 다시)
            r = i // 5
            c = i % 5
            self.grid.addWidget(pw, r, c)

    # ---------------- CSV 로드 ----------------
    def load_csv(self, path):
        """CSV 로드 + 이상치 필터 (LoopCnt 제외) + LoopCnt 정규화 + 플롯 갱신"""
        if not os.path.isfile(path):
            QtWidgets.QMessageBox.warning(self, "Warning", f"File not found:\n{path}")
            return

        try:
            with open(path, "r", encoding="utf-8") as f:
                reader = csv.reader(f)
                header = next(reader)

            csv_cols = header
            arr = np.genfromtxt(path, delimiter=",", skip_header=1)

            if arr.ndim == 1:
                arr = arr.reshape(1, -1)

        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "Error", f"Failed to load CSV:\n{e}")
            return

        # LoopCnt 인덱스
        try:
            loop_idx = csv_cols.index("LoopCnt")
        except ValueError:
            QtWidgets.QMessageBox.critical(self, "Error", "CSV header has no 'LoopCnt' column.")
            return

        # ----------------------------
        #   이상치 제거 (LoopCnt 제외)
        # ----------------------------
        abs_arr = np.abs(arr)

        # LoopCnt 컬럼만 따로 제거 (조건 적용 X)
        abs_arr_no_loop = np.delete(abs_arr, loop_idx, axis=1)

        # NaN / Inf / |value| > 1e5 (LoopCnt 제외)
        bad_mask = (
            np.isnan(arr).any(axis=1) |
            np.isinf(arr).any(axis=1) |
            (abs_arr_no_loop > 1e5).any(axis=1)
        )

        if bad_mask.any():
            first_bad = np.argmax(bad_mask)
            arr_clean = arr[:first_bad]
        else:
            arr_clean = arr

        if arr_clean.size == 0:
            QtWidgets.QMessageBox.warning(
                self, "Warning",
                "All data removed due to outliers (excluding LoopCnt).\nNo valid rows left to plot."
            )
            return

        # 최종 데이터 저장
        self.csv_path = path
        self.csv_cols = csv_cols
        self.data = arr_clean
        self.loop_cnt_raw = self.data[:, loop_idx].copy()

        # ---------- LoopCnt 정규화: 첫 값을 0으로 맞추기 ----------
        base = self.loop_cnt_raw[0]
        self.loop_cnt = self.loop_cnt_raw - base

        self.col_index = {name: i for i, name in enumerate(self.csv_cols)}
        self.label_path.setText(path)

        # 전체 플롯 갱신
        self._plot_all()
        self._relayout()

    # ---------------- 버튼/콜백 ----------------
    def _on_open_csv_clicked(self):
        fname, _ = QtWidgets.QFileDialog.getOpenFileName(
            self,
            "Select CSV file",
            "",
            "CSV Files (*.csv);;All Files (*)"
        )
        if not fname:
            return
        self.load_csv(fname)

    def _on_graph_visibility_changed(self, state):
        # 체크 상태를 플롯 visible 상태에 반영하고 레이아웃 갱신
        for i, cb in enumerate(self.checkboxes):
            self.plots[i].setVisible(cb.isChecked())
        self._relayout()

    def _set_all_graphs(self, value: bool):
        for cb in self.checkboxes:
            cb.setChecked(value)

    def _set_left_fsr(self, value: bool):
        for cb in self.left_fsr_cbs:
            cb.setChecked(value)

    def _set_right_fsr(self, value: bool):
        for cb in self.right_fsr_cbs:
            cb.setChecked(value)

    def _on_fsr_selection_changed(self, state):
        """FSR 채널 선택 변경 시 Graph 7/8만 다시 그리기"""
        if self.data is None:
            return
        self._plot_single(6)  # Graph index 6 (Left FSR)
        self._plot_single(7)  # Graph index 7 (Right FSR)

    # ---------------- FSR 선택 유틸 ----------------
    def _get_active_left_fsr(self):
        names = []
        for name, cb in zip(self.left_fsr_names, self.left_fsr_cbs):
            if cb.isChecked():
                names.append(name)
        return names

    def _get_active_right_fsr(self):
        names = []
        for name, cb in zip(self.right_fsr_names, self.right_fsr_cbs):
            if cb.isChecked():
                names.append(name)
        return names

    # ---------------- 플롯 그리기 ----------------
    def _plot_all(self):
        """현재 로드된 CSV 전체 길이에 대해 모든 그래프 재플롯."""
        if self.data is None or self.loop_cnt is None:
            return
        for i in range(10):
            self._plot_single(i)

    def _plot_single(self, idx: int):
        if self.data is None or self.loop_cnt is None:
            return
        if idx < 0 or idx >= len(self.plots):
            return

        pw = self.plots[idx]
        pw.clear()

        # 기존 legend 제거 (중복 방지)
        if hasattr(pw, 'legend') and pw.legend is not None:
            try:
                pw.legend.scene().removeItem(pw.legend)
            except Exception:
                pass
            pw.legend = None

        legend = pw.addLegend()

        # x축: 항상 정규화된 LoopCnt
        x = self.loop_cnt
        print(f"total duration (ms) : {self.loop_cnt[-1] * 2}")
        # ----- 특수 그래프 (9, 10) -----
        if idx == 8:  # Graph 9: LoopCnt increment (ΔLoopCnt)
            if x.size < 2:
                return
            # 첫 값은 0으로 두고, 이후는 현재-이전
            y = np.diff(x, prepend=x[0])
            color = pg.intColor(0, hues=1)
            pw.plot(x, y, pen=color, name="ΔLoopCnt")
            if x.size:
                pw.setXRange(float(x.min()), float(x.max()), padding=0.01)
            return

        if idx == 9:  # Graph 10: LoopCnt (normalized)
            y = x.copy()
            color = pg.intColor(0, hues=1)
            pw.plot(x, y, pen=color, name="LoopCnt")
            if x.size:
                pw.setXRange(float(x.min()), float(x.max()), padding=0.01)
            return

        # ----- 일반 그래프 (1~8) -----
        # 그룹 이름 선택 (FSR일 경우 선택 채널만)
        if idx == 6:  # Graph 7: Left FSR
            base_group = self.groups[idx]
            active_names = [n for n in self._get_active_left_fsr() if n in base_group]
        elif idx == 7:  # Graph 8: Right FSR
            base_group = self.groups[idx]
            active_names = [n for n in self._get_active_right_fsr() if n in base_group]
        else:
            active_names = self.groups[idx]

        if not active_names:
            return

        hues = max(8, len(active_names))
        for k, name in enumerate(active_names):
            if name not in self.col_index:
                continue
            col = self.col_index[name]
            if col >= self.data.shape[1]:
                continue
            y = self.data[:, col]
            color = pg.intColor(k, hues=hues)
            pw.plot(x, y, pen=color, name=name)

        if x.size:
            pw.setXRange(float(x.min()), float(x.max()), padding=0.01)

    # ---------------- 레이아웃 재배치 ----------------
    def _relayout(self):
        """
        선택된 그래프 수에 따라 레이아웃 자동 배치:
          - n <= 4    → 1열(세로)
          - 5 <= 6    → 2×3
          - 7 ~ 10    → 2×5
        """
        # 기존 항목 제거
        while self.grid.count():
            item = self.grid.takeAt(0)
            w = item.widget()
            if w:
                w.setParent(None)

        visible_idxs = [i for i, cb in enumerate(self.checkboxes) if cb.isChecked()]
        n = len(visible_idxs)

        if n == 0:
            # 아무 것도 선택 안 된 경우: stretch 초기화
            for c in range(5):
                self.grid.setColumnStretch(c, 0)
            for r in range(5):
                self.grid.setRowStretch(r, 0)
            return

        # 열/행 결정
        if n <= 4:
            cols = 1
            rows = n
            max_cols, max_rows = 1, 5
        elif n <= 6:
            cols = 2
            rows = 3
            max_cols, max_rows = 2, 5
        else:  # 7~10
            cols = 2
            rows = 5
            max_cols, max_rows = 2, 5

        # 플롯 재배치
        for order, idx in enumerate(visible_idxs[:10]):
            r = order // cols
            c = order % cols
            self.grid.addWidget(self.plots[idx], r, c)
            self.plots[idx].setVisible(True)

        # stretch 균일 설정
        for c in range(max_cols):
            self.grid.setColumnStretch(c, 0)
        for r in range(max_rows):
            self.grid.setRowStretch(r, 0)

        for c in range(cols):
            self.grid.setColumnStretch(c, 1)
        for r in range(rows):
            self.grid.setRowStretch(r, 1)

        # 선택 안 된 그래프는 숨김
        for i in range(10):
            if i not in visible_idxs:
                self.plots[i].setVisible(False)


def main():
    app = QtWidgets.QApplication(sys.argv)
    pg.setConfigOptions(antialias=True)

    # 시작할 때 한 번 CSV 선택 (원하면 취소 후 나중에 Open CSV 버튼으로 불러도 됨)
    fname, _ = QtWidgets.QFileDialog.getOpenFileName(
        None,
        "Select CSV file (optional)",
        "",
        "CSV Files (*.csv);;All Files (*)"
    )
    if not fname:
        fname = None

    win = CsvReviewWindow(csv_path=fname)
    win.show()

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
