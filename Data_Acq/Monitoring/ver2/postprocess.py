"""
후처리 배치 스크립트 - IMU Hub 0 스파이크 선형보간
지정 폴더 내 모든 원본 CSV를 일괄 후처리하여 _processed.csv로 저장
"""

import os
import glob
import numpy as np

# ===================== 설정 =====================
SAVE_FOLDER = "S016"  # 후처리할 폴더명

# 후처리(0 스파이크 보간)에서 제외할 센서
EXCLUDED_POSTPROCESS_SENSORS = {}

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


# ===================== 후처리 함수 =====================

def detect_zero_spike_rows(data_array):
    """
    IMU Hub 6개 센서 각각에서 값이 모두 0인 행 검출
    Returns: dict {sensor_id: [row_indices]}, dict {sensor_id: [col_indices]}, int total_zeros
    """
    col_names = CSV_COLS
    sensor_names = ["L_THIGH_IMU", "L_SHANK_IMU", "L_FOOT_IMU", "R_THIGH_IMU", "R_SHANK_IMU", "R_FOOT_IMU"]

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

    zero_rows_per_sensor = {}
    total_zeros = 0

    for sensor_id, cols in sensor_columns.items():
        zero_rows = []
        for row_idx in range(data_array.shape[0]):
            sensor_all_zero = True
            for col_idx in cols:
                if col_idx < data_array.shape[1]:
                    if abs(data_array[row_idx, col_idx]) > 1e-6:
                        sensor_all_zero = False
                        break
            if sensor_all_zero and len(cols) > 0:
                zero_rows.append(row_idx)

        if zero_rows:
            zero_rows_per_sensor[sensor_id] = zero_rows
            total_zeros += len(zero_rows)

    return zero_rows_per_sensor, sensor_columns, total_zeros


def interpolate_zero_rows(data_array, zero_rows_per_sensor, sensor_columns):
    """
    센서별로 앞뒤 1틱 데이터로 선형보간
    Returns: corrected data_array (copy)
    """
    corrected = data_array.copy()

    for sensor_id, zero_rows in zero_rows_per_sensor.items():
        cols = sensor_columns[sensor_id]

        for row_idx in zero_rows:
            prev_idx = row_idx - 1
            next_idx = row_idx + 1

            if prev_idx >= 0 and next_idx < data_array.shape[0]:
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

                if prev_valid and next_valid:
                    for col_idx in cols:
                        if col_idx < data_array.shape[1]:
                            prev_val = data_array[prev_idx, col_idx]
                            next_val = data_array[next_idx, col_idx]
                            corrected[row_idx, col_idx] = (prev_val + next_val) / 2.0
                elif prev_valid:
                    for col_idx in cols:
                        if col_idx < data_array.shape[1]:
                            corrected[row_idx, col_idx] = data_array[prev_idx, col_idx]
                elif next_valid:
                    for col_idx in cols:
                        if col_idx < data_array.shape[1]:
                            corrected[row_idx, col_idx] = data_array[next_idx, col_idx]
            elif prev_idx >= 0:
                for col_idx in cols:
                    if col_idx < data_array.shape[1]:
                        corrected[row_idx, col_idx] = data_array[prev_idx, col_idx]
            elif next_idx < data_array.shape[0]:
                for col_idx in cols:
                    if col_idx < data_array.shape[1]:
                        corrected[row_idx, col_idx] = data_array[next_idx, col_idx]

    return corrected


def save_processed_csv(path, data_array):
    """후처리 결과를 CSV로 저장"""
    with open(path, "w", encoding="utf-8", newline="") as f:
        f.write(CSV_HEADER + "\n")
        for row in data_array:
            parts = []
            for idx, val in enumerate(row):
                if idx in {0, 1, 2, 3}:
                    parts.append(str(int(val)))
                else:
                    parts.append(f"{float(val):.4f}")
            f.write(",".join(parts) + "\n")


def process_single_file(csv_path):
    """단일 CSV 파일 후처리. 수정 개수 반환 (0이면 스파이크 없음)"""
    sensor_names = ["L_THIGH_IMU", "L_SHANK_IMU", "L_FOOT_IMU", "R_THIGH_IMU", "R_SHANK_IMU", "R_FOOT_IMU"]

    arr = np.genfromtxt(csv_path, delimiter=",", skip_header=1)
    if arr.ndim == 1:
        arr = arr.reshape(1, -1)

    zero_rows_per_sensor, sensor_columns, total_zeros = detect_zero_spike_rows(arr)

    if total_zeros == 0:
        return 0, {}

    corrected = interpolate_zero_rows(arr, zero_rows_per_sensor, sensor_columns)

    base_name = os.path.splitext(csv_path)[0]
    out_path = f"{base_name}_processed.csv"
    save_processed_csv(out_path, corrected)

    detail = {}
    for sensor_id, rows in zero_rows_per_sensor.items():
        detail[sensor_names[sensor_id]] = len(rows)

    return total_zeros, detail


# ===================== 메인 =====================

def main():
    folder = SAVE_FOLDER
    if not os.path.isdir(folder):
        print(f"[ERROR] '{folder}' not found")
        return

    csv_files = sorted(glob.glob(os.path.join(folder, "*.csv")))
    # _processed.csv 제외
    csv_files = [f for f in csv_files if not f.endswith("_processed.csv")]

    if not csv_files:
        print(f"[INFO] No CSV files in '{folder}'")
        return

    print(f"=== Batch Post-processing: {folder}/ ===")
    print(f"Found {len(csv_files)} file(s)\n")

    total_fixed = 0
    for csv_path in csv_files:
        name = os.path.basename(csv_path)
        print(f"  {name} ... ", end="", flush=True)

        try:
            n_fixed, detail = process_single_file(csv_path)
        except Exception as e:
            print(f"ERROR: {e}")
            continue

        if n_fixed == 0:
            print("no zero spikes")
        else:
            detail_str = ", ".join(f"{k}: {v}" for k, v in detail.items())
            print(f"fixed {n_fixed} ({detail_str})")
            total_fixed += n_fixed

    print(f"\n=== Done. Total fixed: {total_fixed} ===")


if __name__ == "__main__":
    main()
