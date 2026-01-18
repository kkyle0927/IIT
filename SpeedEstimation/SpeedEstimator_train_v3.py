# SpeedEstimator_train.py
# =========================
# Imports & backend setup
# =========================
import os
os.environ["TORCHDYNAMO_DISABLE"] = "1"
os.environ["TORCHINDUCTOR_DISABLE"] = "1"
os.environ["KMP_DUPLICATE_LIB_OK"] = "TRUE"

import h5py
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader

# PyTorch backend: 성능 최적화
torch.backends.cudnn.benchmark = True
torch.backends.cuda.matmul.allow_tf32 = True
torch.backends.cudnn.allow_tf32 = True
if torch.cuda.is_available() and hasattr(torch, "set_float32_matmul_precision"):
    torch.set_float32_matmul_precision("medium")

# Matplotlib (헤드리스 시 안전)
import matplotlib
if os.environ.get("DISPLAY", "") == "" and os.name != "nt":
    matplotlib.use("Agg")
import multiprocessing as mp
import matplotlib.pyplot as plt

from itertools import product
import wandb
from tqdm import tqdm
import time
from datetime import timedelta

# =========================
# Options: 여기만 고치면 됩니다
# =========================
CONFIG = {
    # ===== 공통 데이터/학습 설정 =====
    "path": r"C:\Users\ChanyoungKo\OneDrive - 한국과학기술원 기계공학과\Kyle\KAIST\ExoLab\Research and projects\2025_Intention Intelligence Team\NetworkCode\251030_combined_data.h5",
    "data_length": 12000,
    "sub_names": ['S004','S005','S006','S007','S008','S009','S010','S011'],
    "train_subs": ['S007','S008','S009','S010','S011'],
    "val_subs":   ['S006'],
    "test_subs":  ['S004'],
    "cond_names": [
        'accel_sine','asym_30deg','asym_60deg','cadence_120p','cadence_90p',
        'crouch','decline_5deg','incline_10deg','level_08mps','level_12mps','level_16mps'
    ],
    "use_conds": [
        'accel_sine','cadence_120p','cadence_90p',
        'decline_5deg','incline_10deg','level_08mps','level_12mps','level_16mps'
    ],
    # 효율 옵션
    "eff_batch_size": 2048,     # 유효 배치(gradient accumulation 목표). VRAM 부족하면 유지
    "num_workers": 8,      # CPU 코어 기반 자동 설정
    "prefetch_factor": 4,       # 워커당 프리패치
    "live_plot": False,         # GUI 오버헤드 제거 권장

    # 학습 하이퍼
    "batch_size": 1024, "val_batch_size": 1024,
    "epochs": 20, "patience": 5,
    "dropout_p": 0.3, "attn_heads": 1,
    "lr": 3e-4, "weight_decay": 1e-2,
    "cos_T0": 10, "cos_Tmult": 1, "eta_min": 1e-5,
    "huber_delta": 0.5,

    # 단일 실행 기본값
    "L": 180,         # 공통 시퀀스 길이
    "S": 3,
    "d_model": 192,
    "lstm_h": 512,
    "out_dim": 2,

    # ===== 모델별 그리드 정의 =====
    "models": {
        # 1) TCN + GRU
        "TCN_GRU": {
            "grid": {
                "d":     [192],
                "h":     [256],
                "L":     [91],   # ← 여기
                "S":     [3],
                "kset":  [(5,3)],
                "blocks":[3]
            }
        },

        "GRU_Stack": {
            "grid": {
                "h":      [192, 164, 128],                 # hidden 크기 고정
                "layers": [3, 2, 1],   # 여기만 sweep
                "L":      [90, 120, 140, 160, 180],
                "S":      [3]
            }
        },

        # 3) Pure Tiny-TCN
        "TinyTCN": {
            "grid": {
                "d":     [192],
                "blocks":[3],
                "L":     [91],   # ← 여기
                "S":     [3],
                "kset":  [(5,3)]
            }
        },

        # 4) TCN + LSTM
        "TCN_LSTM": {
            "grid": {
                "d":    [192],
                "h":    [256],
                "L":    [91],    # ← 여기
                "S":    [3],
                "kset": [(5,3)]
            }
        },

        # 5) Small TCN + Tiny Self-Attention
        "SmallTCN_LSTM_Attn": {
            "grid": {
                "d":    [192],
                "h":    [192],
                "L":    [91],    # ← 여기
                "S":    [3],
                "kset": [(5,3)]
            }
        }
    },

    # 어떤 모델들을 돌릴지 선택 (비교용 전체)
    "run_list": ["TCN_GRU", "GRU_Stack", "TinyTCN", "SmallTCN_LSTM_Attn"],

    # 저장 규칙
    "save_prefix": "SE_",
}


# 테스트 스크립트 호환용 전역 이름 유지
Sub_names  = CONFIG["sub_names"]
Cond_names = CONFIG["cond_names"]

# =========================
# Feature 사양: 여기서 입력을 선택하세요
# =========================
DEFAULT_FEAT_SPEC = {
    "include": {
        "stanceL": False, "stanceR": False, "dc": False,
        "acc": True, "gyr": True, "fsrL": False, "fsrR": False, "kin": True,
        "grf": False   # 추가: GRF 사용 여부
    },

    # IMU
    "imu": {
        "use_tilt": True,
        "acc_axes": ["X","Y","Z"],
        "gyr_axes": ["X","Y","Z"],
        "acc_div_g": True
    },

    # FSR
    "fsr": {
        "gate_with_stance": True,
        "L_channels": [1,2,3,4,5,6,7,8],
        "R_channels": [1,2,3,4,5,6,7,8],
        "use_norm": True,
        "summary": { "sumL": True, "sumR": True, "diffLR": True, "sumLR": True }
    },

    # Kinematics
    "kin": {
        "gate_with_stance": False,
        "use": {
            # Hip FE (기존)
            "hip_fe_deg_L": True,  "hip_fe_deg_R": True,
            "hip_fe_vel_L": True,  "hip_fe_vel_R": True,

            # Hip AA, Rotation 추가
            "hip_aa_deg_L": False,  "hip_aa_deg_R": False,
            "hip_aa_vel_L": False,  "hip_aa_vel_R": False,
            "hip_rot_deg_L": False, "hip_rot_deg_R": False,
            "hip_rot_vel_L": False, "hip_rot_vel_R": False,

            # Knee FE 추가
            "knee_deg_L": False,    "knee_deg_R": False,
            "knee_vel_L": False,    "knee_vel_R": False,

            # Ankle FE 추가
            "ankle_deg_L": False,   "ankle_deg_R": False,
            "ankle_vel_L": False,   "ankle_vel_R": False,
        },
        "units": { "angles_in_deg": True, "angvel_in_deg_s": True },
        "dimless": {
            "hip_forward": False,          # ω cosθ √(L/g)
            "hip_vertical": False,         # ω sinθ √(L/g)
            "leg_length_key": "leg_length_m"
        }
    },

    # GRF 측정값 사용 옵션
    "grf": {
        "use_norm": True,                # BW로 나눔
        "gate_with_stance": False,       # 스탠스로 게이팅
        "left":  {"Fx": True, "Fy": True, "Fz": True},
        "right": {"Fx": True, "Fy": True, "Fz": True},
        "summary": {                     # 요약 채널 선택
            "L_Fz": False, "R_Fz": False,
            "sum_Fz": False, "diff_Fz": False
        }
    },

    # Target 무차원화
    "target": {
        "dimless_vy": True,   # True면 vY를 √(gL)로 나눠서 학습/저장
        "dimless_vz": True    # True면 vZ를 √(gL)로 나눠서 학습/저장
    }
}

# ---------------- 유틸 ----------------
def alloc_mat(data_length, n_cond): 
    return np.empty((data_length, n_cond), dtype=np.float32)

def fill_col_1d(dset, out, col, data_length):
    n = min(len(dset), data_length)
    out[:n, col] = dset[:n].astype(np.float32)
    if n < data_length:
        out[n:, col] = np.nan

def fill_marker_xyz(dset2d, out_x, out_y, out_z, col, data_length):
    T = dset2d.shape[0]
    n = min(T, data_length)
    buf = dset2d[:n, :].astype(np.float32) / 1000.0  # mm -> m 가정시
    out_x[:n, col], out_y[:n, col], out_z[:n, col] = buf[:,0], buf[:,1], buf[:,2]
    if n < data_length:
        out_x[n:, col] = np.nan; out_y[n:, col] = np.nan; out_z[n:, col] = np.nan


# ---------------- 데이터 로드 ----------------
def load_all_subjects(path=None, sub_names=None, cond_names=None, data_length=None):
    path = path or CONFIG["path"]
    sub_names = sub_names or CONFIG["sub_names"]
    cond_names = cond_names or CONFIG["cond_names"]
    data_length = data_length or CONFIG["data_length"]

    Sub = {}
    with h5py.File(path, 'r') as f:
        for sub_idx, sub_name in enumerate(sub_names, start=1):
            print(f"Processing {sub_name}...", flush=True)
            Sub[sub_idx] = {}

            # Insole FSR
            for side, prefix in [('Left','L'), ('Right','R')]:
                for k in range(1,9):
                    key = f'fsr{prefix}{k}'
                    arr = alloc_mat(data_length, len(cond_names))
                    for c, cond in enumerate(cond_names):
                        d = f[f"{sub_name}/{cond}/trial_01/Insole/{side}/{key}"]
                        fill_col_1d(d, arr, c, data_length)
                    Sub[sub_idx][key] = arr

            # Back IMU
            for out_key, h5key in [('BackIMUAccX','Accel_X'), ('BackIMUAccY','Accel_Y'), ('BackIMUAccZ','Accel_Z'),
                                   ('BackIMUGyrX','Gyro_X'),  ('BackIMUGyrY','Gyro_Y'),  ('BackIMUGyrZ','Gyro_Z')]:
                arr = alloc_mat(data_length, len(cond_names))
                for c, cond in enumerate(cond_names):
                    d = f[f"{sub_name}/{cond}/trial_01/Back_imu/{h5key}"]
                    fill_col_1d(d, arr, c, data_length)
                Sub[sub_idx][out_key] = arr

            # Encoder
            for out_key, h5key in [('H10encL','incPosLH'), ('H10encR','incPosRH')]:
                arr = alloc_mat(data_length, len(cond_names))
                for c, cond in enumerate(cond_names):
                    d = f[f"{sub_name}/{cond}/trial_01/Robot/{h5key}"]
                    fill_col_1d(d, arr, c, data_length)
                Sub[sub_idx][out_key] = arr

            # GRF (이미 로드되어 있었음)
            for out_key, lr, comp in [('LGRFx','left','Fx'), ('LGRFy','left','Fy'), ('LGRFz','left','Fz'),
                                      ('RGRFx','right','Fx'),('RGRFy','right','Fy'),('RGRFz','right','Fz')]:
                arr = alloc_mat(data_length, len(cond_names))
                for c, cond in enumerate(cond_names):
                    d = f[f"{sub_name}/{cond}/trial_01/MoCap/grf_measured/{lr}/force/{comp}"]
                    fill_col_1d(d, arr, c, data_length)
                Sub[sub_idx][out_key] = arr

            # Kinematics (각도)
            Sub[sub_idx]['mcdeg'] = {}
            mcdeg_map = [
                ('LhipFE','hip_flexion_l'), ('LhipAA','hip_adduction_l'), ('LhipRot','hip_rotation_l'),
                ('RhipFE','hip_flexion_r'), ('RhipAA','hip_adduction_r'), ('RhipRot','hip_rotation_r'),
                ('LkneeFE','knee_angle_l'), ('RkneeFE','knee_angle_r'),
                ('LankleFE','ankle_angle_l'), ('RankleFE','ankle_angle_r')
            ]
            for out_key, h5key in mcdeg_map:
                arr = alloc_mat(data_length, len(cond_names))
                for c, cond in enumerate(cond_names):
                    d = f[f"{sub_name}/{cond}/trial_01/MoCap/kin_q/{h5key}"]
                    fill_col_1d(d, arr, c, data_length)
                Sub[sub_idx]['mcdeg'][out_key] = arr

            # Kinematics (각속도)
            Sub[sub_idx]['mcangvel'] = {}
            for out_key, h5key in mcdeg_map:
                arr = alloc_mat(data_length, len(cond_names))
                for c, cond in enumerate(cond_names):
                    d = f[f"{sub_name}/{cond}/trial_01/MoCap/kin_qdot/{h5key}"]
                    fill_col_1d(d, arr, c, data_length)
                Sub[sub_idx]['mcangvel'][out_key] = arr

            # Markers
            Sub[sub_idx]['mcmarker'] = {}
            for base_key, h5key in [('BackIMU_L','IMU_H10_L'), ('BackIMU_R','IMU_H10_R'), ('BackIMU_T','IMU_H10_Top')]:
                mx = alloc_mat(data_length, len(cond_names))
                my = alloc_mat(data_length, len(cond_names))
                mz = alloc_mat(data_length, len(cond_names))
                for c, cond in enumerate(cond_names):
                    d2 = f[f"{sub_name}/{cond}/trial_01/MoCap/markers/data/{h5key}"]
                    fill_marker_xyz(d2, mx, my, mz, c, data_length)
                Sub[sub_idx]['mcmarker'][f'{base_key}x'] = mx
                Sub[sub_idx]['mcmarker'][f'{base_key}y'] = my
                Sub[sub_idx]['mcmarker'][f'{base_key}z'] = mz

            # sub_info
            obj = f.get(f"{sub_name}/sub_info", None)
            mass_val = np.nan
            leg_val = np.nan

            if obj is None:
                print("[warn: sub_info missing]")
            elif isinstance(obj, h5py.Dataset):
                try:
                    mass_val = float(np.array(obj[()]).squeeze())
                except Exception as e:
                    print(f"[warn: sub_info dataset parse failed: {e}]")
            else:
                ds_m = obj.get('mass_kg', None)
                if isinstance(ds_m, h5py.Dataset):
                    mass_val = float(np.array(ds_m[()]).squeeze())
                elif 'mass_kg' in obj.attrs:
                    mass_val = float(obj.attrs['mass_kg'])
                else:
                    print(f"[warn] {sub_name}: 'mass_kg' not found")

                ds_l = obj.get('leg_length_m', None)
                if isinstance(ds_l, h5py.Dataset):
                    leg_val = float(np.array(ds_l[()]).squeeze())
                elif 'leg_length_m' in obj.attrs:
                    leg_val = float(obj.attrs['leg_length_m'])
                else:
                    print(f"[warn] {sub_name}: 'leg_length_m' not found")

            Sub[sub_idx]['mass_kg'] = np.float32(mass_val)
            Sub[sub_idx]['leg_length_m'] = np.float32(leg_val)

            # pitch_tread
            arr = alloc_mat(data_length, len(cond_names))
            for c, cond in enumerate(cond_names):
                try:
                    d = f[f"{sub_name}/{cond}/trial_01/Common/pitch_tread"]
                    fill_col_1d(d, arr, c, data_length)
                except KeyError:
                    arr[:, c] = 0.0
            Sub[sub_idx]['pitch_tread'] = arr

            # true vel/acc
            for out_key, h5key in [('true_velY','v_Y_true'), ('true_velZ','v_Z_true'),
                                   ('true_accY','a_Y_true'), ('true_accZ','a_Z_true')]:
                arr = alloc_mat(data_length, len(cond_names))
                for c, cond in enumerate(cond_names):
                    d = f[f"{sub_name}/{cond}/trial_01/Common/{h5key}"]
                    fill_col_1d(d, arr, c, data_length)
                Sub[sub_idx][out_key] = arr

            # contact from GRF (bit mask: 1=R, 2=L, 3=both, 0=none)
            arr = alloc_mat(data_length, len(cond_names))
            Lz, Rz = Sub[sub_idx]['LGRFz'], Sub[sub_idx]['RGRFz']
            thres_GRF = 30
            for c in range(len(cond_names)):
                left_contact  = (Lz[:, c] > thres_GRF).astype(np.int32)
                right_contact = (Rz[:, c] > thres_GRF).astype(np.int32)
                stage = np.zeros_like(left_contact, dtype=np.int32)
                for t in range(len(stage)):
                    code = 0
                    if right_contact[t]:
                        code += 1   # R
                    if left_contact[t]:
                        code += 2   # L
                    if code == 0 and t > 0:
                        # 이전 상태 유지 여부는 취향에 따라
                        stage[t] = 0
                    else:
                        stage[t] = code
                arr[:, c] = stage.astype(np.float32)
            Sub[sub_idx]['contact'] = arr

    print("전체 완료")
    return Sub

# ---------------- High-frequency tick filter ----------------
def _median3_along_time(A: np.ndarray) -> np.ndarray:
    if A.ndim == 1:
        x0 = A
        xm = np.concatenate([A[:1], A[:-1]], axis=0)
        xp = np.concatenate([A[1:], A[-1:]], axis=0)
        return np.median(np.stack([xm, x0, xp], axis=0), axis=0).astype(np.float32)
    elif A.ndim == 2:
        xm = np.vstack([A[:1, :], A[:-1, :]])
        x0 = A
        xp = np.vstack([A[1:, :], A[-1:, :]])
        return np.median(np.stack([xm, x0, xp], axis=0), axis=0).astype(np.float32)
    else:
        return A

def apply_tick_filter(Sub_dict, passes: int = 1, skip_keys=("contact",)):
    for _, D in Sub_dict.items():
        for key, val in list(D.items()):
            if key in skip_keys:
                continue
            if isinstance(val, np.ndarray):
                if val.dtype.kind in ("f",):
                    arr = val
                    for _ in range(max(1, passes)):
                        arr = _median3_along_time(arr)
                    D[key] = arr
            elif isinstance(val, dict):
                for k2, v2 in list(val.items()):
                    if isinstance(v2, np.ndarray) and v2.dtype.kind in ("f",):
                        arr = v2
                        for _ in range(max(1, passes)):
                            arr = _median3_along_time(arr)
                        D[key][k2] = arr

# ---------------- 시각화 (선택) ----------------
def visualize_model_inputs_all_seq(Sub, cfg, mu, sd, fs=100, seconds=30):
    import numpy as np, matplotlib.pyplot as plt

    T_show = int(seconds * fs)
    L_long = min(T_show, cfg["data_length"])

    cond_ids = _conds_to_ids(cfg["cond_names"], cfg["use_conds"])
    name_to_idx = {n: i + 1 for i, n in enumerate(cfg["sub_names"])}

    # 짧은 표기 이름 후보
    spec = DEFAULT_FEAT_SPEC
    feat_names = []

    # Kinematics 표기 일부
    if spec["include"].get("kin", False):
        feat_names.extend([
            "LHdeg","RHdeg","LHvel","RHvel",  # 최소 표기. 실제 F와 안 맞을 수 있으나 자동 생성 보완됨.
        ])

    if spec["include"].get("acc", False):
        for ax in spec["imu"]["acc_axes"]:
            feat_names.append({"X":"IMUAX","Y":"IMUAY","Z":"IMUAZ"}[ax])

    if spec["include"].get("gyr", False):
        for ax in spec["imu"]["gyr_axes"]:
            feat_names.append({"X":"IMUGX","Y":"IMUGY","Z":"IMUGZ"}[ax])

    if spec["include"].get("fsrL", False):
        for k in spec["fsr"]["L_channels"]:
            feat_names.append(f"fsrL{k}")
    if spec["include"].get("fsrR", False):
        for k in spec["fsr"]["R_channels"]:
            feat_names.append(f"fsrR{k}")

    sm = spec["fsr"].get("summary", {})
    if sm.get("sumL", False):   feat_names.append("sumL")
    if sm.get("sumR", False):   feat_names.append("sumR")
    if sm.get("diffLR", False): feat_names.append("diffLR")
    if sm.get("sumLR", False):  feat_names.append("sumLR")

    if spec["include"].get("dc", False):       feat_names.append("dc")
    if spec["include"].get("stanceL", False):  feat_names.append("stanceL")
    if spec["include"].get("stanceR", False):  feat_names.append("stanceR")

    # GRF 간단 표기
    if spec["include"].get("grf", False):
        if spec["grf"]["left"]["Fx"]:  feat_names.append("LGRFx")
        if spec["grf"]["left"]["Fy"]:  feat_names.append("LGRFy")
        if spec["grf"]["left"]["Fz"]:  feat_names.append("LGRFz")
        if spec["grf"]["right"]["Fx"]: feat_names.append("RGRFx")
        if spec["grf"]["right"]["Fy"]: feat_names.append("RGRFy")
        if spec["grf"]["right"]["Fz"]: feat_names.append("RGRFz")

    for sub_name in cfg["sub_names"]:
        sub_idx = name_to_idx[sub_name]
        ds = SpeedDataset(Sub, use_sub_indices=[sub_idx],
                          L=L_long, S=L_long,
                          use_cond_ids=cond_ids,
                          fit_norm=False, mu=mu, sd=sd, apply_norm=False)
        if len(ds) == 0:
            continue

        x = ds.X[0].numpy()     # (L, F)
        L, F = x.shape

        # 이름 길이 F에 맞춤
        names = (feat_names[:F] + [f"f{i}" for i in range(len(feat_names), F)])[:F]

        half = (F + 1) // 2
        t = np.arange(L) / fs

        def _make_fig(data, start, end, title):
            n = end - start
            fig, axes = plt.subplots(n, 1, figsize=(14, max(2.0, n*1.2)), sharex=True)
            if n == 1: axes = [axes]
            for ax, fi in zip(axes, range(start, end)):
                ax.plot(t, data[:, fi])
                ax.set_ylabel(names[fi])
                ax.grid(True, linestyle=":", linewidth=0.5)
            axes[-1].set_xlabel("Time [s]")
            fig.suptitle(title, x=0.01, ha="left")
            plt.tight_layout(rect=[0, 0, 1, 0.96])
            return fig

        fig1 = _make_fig(x, 0, half,  f"[{sub_name}] Model inputs 1/2 (0–{seconds}s, F={F})")
        fig2 = _make_fig(x, half, F,  f"[{sub_name}] Model inputs 2/2 (0–{seconds}s, F={F})")

        plt.show(block=True)
        plt.close(fig1); plt.close(fig2)


# ---------------- NaN 보정 ----------------
def count_and_fix_nans_in_Sub(Sub_dict):
    def fill_nans(arr):
        if not np.isnan(arr).any():
            return arr
        arr = arr.copy()
        n = arr.shape[0]
        for i in range(n):
            if np.isnan(arr[i]):
                prev_val = arr[i-1] if i>0 and not np.isnan(arr[i-1]) else None
                next_val = arr[i+1] if i<n-1 and not np.isnan(arr[i+1]) else None
                if prev_val is not None and next_val is not None:
                    arr[i] = (prev_val + next_val) / 2
                elif prev_val is not None:
                    arr[i] = prev_val
                elif next_val is not None:
                    arr[i] = next_val
                else:
                    arr[i] = 0.0
        return arr

    total_nan_before = 0; total_nan_after = 0
    print("===== NaN 검사 시작 =====")
    for sub_idx, D in Sub_dict.items():
        for key, val in D.items():
            if isinstance(val, dict):
                for k2, v2 in val.items():
                    if isinstance(v2, np.ndarray):
                        n_before = np.isnan(v2).sum(); total_nan_before += n_before
                        if n_before>0:
                            v2_fixed = np.apply_along_axis(fill_nans, 0, v2)
                            Sub_dict[sub_idx][key][k2] = v2_fixed
                            total_nan_after += np.isnan(v2_fixed).sum()
            elif isinstance(val, np.ndarray):
                n_before = np.isnan(val).sum(); total_nan_before += n_before
                if n_before>0:
                    v_fixed = np.apply_along_axis(fill_nans, 0, val)
                    Sub_dict[sub_idx][key] = v_fixed
                    total_nan_after += np.isnan(v_fixed).sum()
    print("\n===== NaN 통계 =====")
    print(f"보정 전 총 NaN 개수: {total_nan_before}")
    print(f"보정 후 총 NaN 개수: {total_nan_after}")
    print("NaN 보정 완료 ✅")

# ---------------- IMU 전처리 ----------------
def preprocess_back_imu(Sub_dict, pitch_is_deg=True):
    acc_div_g = DEFAULT_FEAT_SPEC.get("imu", {}).get("acc_div_g", False)

    for _, D in Sub_dict.items():
        if 'mcmarker' not in D or 'pitch_tread' not in D:
            continue

        Lx = D['mcmarker']['BackIMU_Lx']; Ly = D['mcmarker']['BackIMU_Ly']; Lz = D['mcmarker']['BackIMU_Lz']
        Rx = D['mcmarker']['BackIMU_Rx']; Ry = D['mcmarker']['BackIMU_Ry']; Rz = D['mcmarker']['BackIMU_Rz']
        Tx = D['mcmarker']['BackIMU_Tx']; Ty = D['mcmarker']['BackIMU_Ty']; Tz = D['mcmarker']['BackIMU_Tz']
        pitch_all = D['pitch_tread']

        for prefix in ['BackIMUAcc', 'BackIMUGyr']:
            if not all(k in D for k in (f'{prefix}X', f'{prefix}Y', f'{prefix}Z')):
                continue

            X_all = np.nan_to_num(D[f'{prefix}X'])
            Y_all = np.nan_to_num(D[f'{prefix}Y'])
            Z_all = np.nan_to_num(D[f'{prefix}Z'])
            Tn, Cn = X_all.shape

            outX = np.empty_like(X_all, dtype=np.float32)
            outY = np.empty_like(Y_all, dtype=np.float32)
            outZ = np.empty_like(Z_all, dtype=np.float32)

            for c in range(Cn):
                QL = np.stack([Lx[:,c], Ly[:,c], Lz[:,c]], axis=1)
                QR = np.stack([Rx[:,c], Ry[:,c], Rz[:,c]], axis=1)
                QT = np.stack([Tx[:,c], Ty[:,c], Tz[:,c]], axis=1)

                QL = np.nan_to_num(QL); QR = np.nan_to_num(QR); QT = np.nan_to_num(QT)
                vLR = QR - QL
                yI  = vLR / (np.linalg.norm(vLR, axis=1, keepdims=True) + 1e-12)
                vLT = QT - QL
                zI  = -np.cross(vLR, vLT)
                zI  = zI / (np.linalg.norm(zI, axis=1, keepdims=True) + 1e-12)
                xI  = np.cross(yI, zI)
                xI  = xI / (np.linalg.norm(xI, axis=1, keepdims=True) + 1e-12)

                dataL = np.stack([X_all[:,c], Y_all[:,c], Z_all[:,c]], axis=1)

                a_T = (dataL[:,0:1]*xI) + (dataL[:,1:2]*yI) + (dataL[:,2:3]*zI)

                theta = pitch_all[:, c]
                if pitch_is_deg:
                    theta = np.deg2rad(theta)
                cth, sth = np.cos(theta), np.sin(theta)

                gX = a_T[:, 0]
                gY =  cth * a_T[:,1] - sth * a_T[:,2]
                gZ =  sth * a_T[:,1] + cth * a_T[:,2]

                outX[:, c] = gX.astype(np.float32)
                outY[:, c] = gY.astype(np.float32)
                outZ[:, c] = gZ.astype(np.float32)

            if acc_div_g and prefix == 'BackIMUAcc':
                outX /= 9.80665
                outY /= 9.80665
                outZ /= 9.80665

            D[f'{prefix}X_tilt'] = outX
            D[f'{prefix}Y_tilt'] = outY
            D[f'{prefix}Z_tilt'] = outZ

# ---------------- 무차원 선속도 파생 ----------------
def preprocess_linearvelocity(D, c, spec, hipL, hipR, hipVL, hipVR):
    out = []

    dimless = spec["kin"].get("dimless", {})
    use_fwd = dimless.get("hip_forward", False)
    use_ver = dimless.get("hip_vertical", False)
    if not (use_fwd or use_ver):
        return out

    units = spec["kin"].get("units", {})
    in_deg   = units.get("angles_in_deg", True)
    in_deg_s = units.get("angvel_in_deg_s", True)

    def deg2rad(arr): 
        return arr * (np.pi/180.0) if arr is not None and in_deg else arr

    hipL_r  = deg2rad(hipL)
    hipR_r  = deg2rad(hipR)
    hipVL_r = hipVL * (np.pi/180.0) if (hipVL is not None and in_deg_s) else hipVL
    hipVR_r = hipVR * (np.pi/180.0) if (hipVR is not None and in_deg_s) else hipVR

    cosL = np.cos(hipL_r) if hipL_r is not None else None
    cosR = np.cos(hipR_r) if hipR_r is not None else None
    sinL = np.sin(hipL_r) if hipL_r is not None else None
    sinR = np.sin(hipR_r) if hipR_r is not None else None

    g = 9.80665
    L_key = dimless.get("leg_length_key", "leg_length_m")
    L_leg = float(D.get(L_key, np.nan))
    if not np.isfinite(L_leg) or L_leg <= 0:
        L_leg = 1.0
    scale = np.sqrt(L_leg / g)

    if use_fwd:
        L_fwd = hipVL_r * (cosL if cosL is not None else 1.0) * scale if hipVL_r is not None else None
        R_fwd = hipVR_r * (cosR if cosR is not None else 1.0) * scale if hipVR_r is not None else None
        if L_fwd is not None: out.append(L_fwd)
        if R_fwd is not None: out.append(R_fwd)

    if use_ver:
        L_ver = hipVL_r * (sinL if sinL is not None else 0.0) * scale if hipVL_r is not None else None
        R_ver = hipVR_r * (sinR if sinR is not None else 0.0) * scale if hipVR_r is not None else None
        if L_ver is not None: out.append(L_ver)
        if R_ver is not None: out.append(R_ver)

    return out

# ---------------- 정규화 유틸 ----------------
def normalize_fsr_by_mass(Sub, sub_idx, mass_kg):
    bw = float(mass_kg) * 9.80665
    if not np.isfinite(bw) or bw <= 0:
        print(f"[warn] invalid mass ({mass_kg}) -> skipped FSR normalization")
        return
    D = Sub[sub_idx]
    for side in ('L', 'R'):
        for k in range(1, 9):
            raw_key  = f'fsr{side}{k}'
            norm_key = f'fsr{side}{k}_norm'
            if raw_key in D:
                D[norm_key] = D[raw_key] / bw

def normalize_grf_by_mass(Sub, sub_idx, mass_kg):
    """GRF를 체중(BW)으로 나눈 값을 별도 키 '*_norm'으로 저장."""
    bw = float(mass_kg) * 9.80665
    if not np.isfinite(bw) or bw <= 0:
        print(f"[warn] invalid mass ({mass_kg}) -> skipped GRF normalization")
        return
    D = Sub[sub_idx]
    for k in ['LGRFx','LGRFy','LGRFz','RGRFx','RGRFy','RGRFz']:
        if k in D:
            D[k + "_norm"] = D[k] / bw

# ---------------- Dataset ----------------
def _get_if(spec_use: bool, arr):
    return arr if spec_use and (arr is not None) else None

def _append_if(lst, arr):
    if arr is not None:
        lst.append(arr)

def _names_to_indices(name_pool, picked_names):
    name_to_idx1 = {n:i+1 for i,n in enumerate(name_pool)}  # 1-based sub index
    return [name_to_idx1[n] for n in picked_names]

def _conds_to_ids(cond_pool, picked_names):
    cond_to_id0 = {n:i for i,n in enumerate(cond_pool)}     # 0-based cond id
    return [cond_to_id0[n] for n in picked_names]

def build_feat(D, c, cont, spec=DEFAULT_FEAT_SPEC):
    inc = spec["include"]

    # --- stance & 변화율 ---
    stanceL = np.isin(cont, [2,3]).astype(np.float32)[:, None] if inc["stanceL"] else None
    stanceR = np.isin(cont, [1,3]).astype(np.float32)[:, None] if inc["stanceR"] else None
    dc      = np.abs(np.diff(cont, prepend=cont[0])).astype(np.float32)[:, None] if inc["dc"] else None

    feats = []

    # --- Kinematics ---
    if inc.get("kin", False):
        kin_use  = spec["kin"]["use"]
        KQ  = D['mcdeg']; KQD = D['mcangvel']  # 각도, 각속도

        # 편의 접근자
        def g(name): return KQ.get(name)[:, c:c+1] if name in KQ else None
        def gv(name): return KQD.get(name)[:, c:c+1] if name in KQD else None

        # Hip FE
        _append_if(feats, _get_if(kin_use.get("hip_fe_deg_L", True), g('LhipFE')))
        _append_if(feats, _get_if(kin_use.get("hip_fe_deg_R", True), g('RhipFE')))
        _append_if(feats, _get_if(kin_use.get("hip_fe_vel_L", True), gv('LhipFE')))
        _append_if(feats, _get_if(kin_use.get("hip_fe_vel_R", True), gv('RhipFE')))

        # Hip AA
        _append_if(feats, _get_if(kin_use.get("hip_aa_deg_L", True), g('LhipAA')))
        _append_if(feats, _get_if(kin_use.get("hip_aa_deg_R", True), g('RhipAA')))
        _append_if(feats, _get_if(kin_use.get("hip_aa_vel_L", True), gv('LhipAA')))
        _append_if(feats, _get_if(kin_use.get("hip_aa_vel_R", True), gv('RhipAA')))

        # Hip Rotation
        _append_if(feats, _get_if(kin_use.get("hip_rot_deg_L", True), g('LhipRot')))
        _append_if(feats, _get_if(kin_use.get("hip_rot_deg_R", True), g('RhipRot')))
        _append_if(feats, _get_if(kin_use.get("hip_rot_vel_L", True), gv('LhipRot')))
        _append_if(feats, _get_if(kin_use.get("hip_rot_vel_R", True), gv('RhipRot')))

        # Knee FE
        _append_if(feats, _get_if(kin_use.get("knee_deg_L", True), g('LkneeFE')))
        _append_if(feats, _get_if(kin_use.get("knee_deg_R", True), g('RkneeFE')))
        _append_if(feats, _get_if(kin_use.get("knee_vel_L", True), gv('LkneeFE')))
        _append_if(feats, _get_if(kin_use.get("knee_vel_R", True), gv('RkneeFE')))

        # Ankle FE
        _append_if(feats, _get_if(kin_use.get("ankle_deg_L", True), g('LankleFE')))
        _append_if(feats, _get_if(kin_use.get("ankle_deg_R", True), g('RankleFE')))
        _append_if(feats, _get_if(kin_use.get("ankle_vel_L", True), gv('LankleFE')))
        _append_if(feats, _get_if(kin_use.get("ankle_vel_R", True), gv('RankleFE')))

        # 힙 무차원 선속도 특징 추가 (수평/수직 토글)
        hipL  = KQ.get('LhipFE')[:, c:c+1] if 'LhipFE' in KQ else None
        hipR  = KQ.get('RhipFE')[:, c:c+1] if 'RhipFE' in KQ else None
        hipVL = KQD.get('LhipFE')[:, c:c+1] if 'LhipFE' in KQD else None
        hipVR = KQD.get('RhipFE')[:, c:c+1] if 'RhipFE' in KQD else None
        for x in preprocess_linearvelocity(D, c, spec, hipL, hipR, hipVL, hipVR):
            _append_if(feats, x)

    # --- IMU ---
    if inc["acc"] or inc["gyr"]:
        use_tilt = spec["imu"]["use_tilt"]
        def _key(base, axis): return f"{base}{axis}_tilt" if use_tilt else f"{base}{axis}"

        if inc["acc"]:
            for ax in spec["imu"]["acc_axes"]:
                acc = D[_key("BackIMUAcc", ax)][:, c:c+1]
                _append_if(feats, acc)

        if inc["gyr"]:
            for ax in spec["imu"]["gyr_axes"]:
                gyr = D[_key("BackIMUGyr", ax)][:, c:c+1]
                _append_if(feats, gyr)

    # --- FSR ---
    if inc["fsrL"] or inc["fsrR"]:
        gate = spec["fsr"]["gate_with_stance"]
        use_norm = spec["fsr"]["use_norm"]

        fsrL_list, fsrR_list = [], []

        def _fsr(side, k):
            raw = f"fsr{side}{k}_norm" if use_norm and (f"fsr{side}{k}_norm" in D) else f"fsr{side}{k}"
            return D[raw][:, c:c+1]

        if inc["fsrL"]:
            for k in spec["fsr"]["L_channels"]:
                x = _fsr('L', k)
                if gate and stanceL is not None: x = x * stanceL
                fsrL_list.append(x); _append_if(feats, x)

        if inc["fsrR"]:
            for k in spec["fsr"]["R_channels"]:
                x = _fsr('R', k)
                if gate and stanceR is not None: x = x * stanceR
                fsrR_list.append(x); _append_if(feats, x)

        # summary
        fsrL_mat = np.hstack(fsrL_list) if len(fsrL_list) > 0 else None
        fsrR_mat = np.hstack(fsrR_list) if len(fsrR_list) > 0 else None

        sumL = np.sum(fsrL_mat, axis=1, keepdims=True) if fsrL_mat is not None else None
        sumR = np.sum(fsrR_mat, axis=1, keepdims=True) if fsrR_mat is not None else None

        sm = spec["fsr"].get("summary", {})
        if sm.get("sumL", False) and (sumL is not None): _append_if(feats, sumL)
        if sm.get("sumR", False) and (sumR is not None): _append_if(feats, sumR)
        if (sumL is not None) and (sumR is not None):
            if sm.get("diffLR", False): _append_if(feats, sumL - sumR)
            if sm.get("sumLR",  False): _append_if(feats, sumL + sumR)

    # --- GRF ---
    if inc.get("grf", False):
        grf_spec = spec["grf"]
        use_norm = grf_spec.get("use_norm", True)
        gate     = grf_spec.get("gate_with_stance", False)

        def _pick_grf(base_key):
            key = base_key + "_norm" if use_norm and (base_key + "_norm" in D) else base_key
            return D.get(key, None)

        # Left
        if grf_spec["left"].get("Fx", True):
            x = _pick_grf("LGRFx");  x = x[:, c:c+1] if x is not None else None
            if x is not None and gate and stanceL is not None: x = x * stanceL
            _append_if(feats, x)
        if grf_spec["left"].get("Fy", True):
            y = _pick_grf("LGRFy");  y = y[:, c:c+1] if y is not None else None
            if y is not None and gate and stanceL is not None: y = y * stanceL
            _append_if(feats, y)
        if grf_spec["left"].get("Fz", True):
            z = _pick_grf("LGRFz");  z = z[:, c:c+1] if z is not None else None
            if z is not None and gate and stanceL is not None: z = z * stanceL
            _append_if(feats, z)

        # Right
        if grf_spec["right"].get("Fx", True):
            x = _pick_grf("RGRFx");  x = x[:, c:c+1] if x is not None else None
            if x is not None and gate and stanceR is not None: x = x * stanceR
            _append_if(feats, x)
        if grf_spec["right"].get("Fy", True):
            y = _pick_grf("RGRFy");  y = y[:, c:c+1] if y is not None else None
            if y is not None and gate and stanceR is not None: y = y * stanceR
            _append_if(feats, y)
        if grf_spec["right"].get("Fz", True):
            z = _pick_grf("RGRFz");  z = z[:, c:c+1] if z is not None else None
            if z is not None and gate and stanceR is not None: z = z * stanceR
            _append_if(feats, z)

        # Summary Fz
        Lz = _pick_grf("LGRFz"); Rz = _pick_grf("RGRFz")
        if Lz is not None: Lz = Lz[:, c:c+1]
        if Rz is not None: Rz = Rz[:, c:c+1]
        sm = grf_spec.get("summary", {})
        if sm.get("L_Fz", False) and Lz is not None: _append_if(feats, Lz)
        if sm.get("R_Fz", False) and Rz is not None: _append_if(feats, Rz)
        if Lz is not None and Rz is not None:
            if sm.get("sum_Fz", False):  _append_if(feats, Lz + Rz)
            if sm.get("diff_Fz", False): _append_if(feats, Lz - Rz)

    # --- dc, stance 열 추가 ---
    for x in (dc, stanceL, stanceR):
        _append_if(feats, x)

    # --- 합치기 ---
    if len(feats) == 0:
        X = np.zeros((0,1), dtype=np.float32)
    else:
        X = np.hstack(feats).astype(np.float32)
        X = np.nan_to_num(X, nan=0.0, posinf=0.0, neginf=0.0)

    # --- 출력 (vY, vZ) ---
    g = 9.80665
    dimless_cfg = spec["kin"]["dimless"]
    L_key  = dimless_cfg.get("leg_length_key", "leg_length_m")
    L_leg  = float(D.get(L_key, 1.0))
    sqrt_gL = np.sqrt(g * max(L_leg, 1e-6))

    tgt = spec.get("target", {})
    use_dimless_vy = tgt.get("dimless_vy", True)
    use_dimless_vz = tgt.get("dimless_vz", False)

    vY_out = D['true_velY'][:, c] / sqrt_gL if use_dimless_vy else D['true_velY'][:, c]
    vZ_out = D['true_velZ'][:, c] / sqrt_gL if use_dimless_vz else D['true_velZ'][:, c]

    Y = np.stack([vY_out, vZ_out], axis=1).astype(np.float32)
    mask = ~np.isnan(Y).any(axis=1)
    return X[mask], Y[mask]


def get_feat_names_from_spec(D, spec=DEFAULT_FEAT_SPEC):
    """
    build_feat()가 만드는 feature 순서와 동일한 순서로
    채널 이름 리스트를 생성한다.
    D : Sub[sub_idx] 딕셔너리
    """
    inc = spec["include"]
    names = []

    # --- Kinematics ---
    if inc.get("kin", False):
        kin_use = spec["kin"]["use"]
        KQ  = D.get('mcdeg', {})
        KQD = D.get('mcangvel', {})

        def has_q(name):  return name in KQ
        def has_qd(name): return name in KQD

        # Hip FE
        if kin_use.get("hip_fe_deg_L", True) and has_q('LhipFE'):
            names.append("hip_fe_deg_L")
        if kin_use.get("hip_fe_deg_R", True) and has_q('RhipFE'):
            names.append("hip_fe_deg_R")
        if kin_use.get("hip_fe_vel_L", True) and has_qd('LhipFE'):
            names.append("hip_fe_vel_L")
        if kin_use.get("hip_fe_vel_R", True) and has_qd('RhipFE'):
            names.append("hip_fe_vel_R")

        # Hip AA
        if kin_use.get("hip_aa_deg_L", False) and has_q('LhipAA'):
            names.append("hip_aa_deg_L")
        if kin_use.get("hip_aa_deg_R", False) and has_q('RhipAA'):
            names.append("hip_aa_deg_R")
        if kin_use.get("hip_aa_vel_L", False) and has_qd('LhipAA'):
            names.append("hip_aa_vel_L")
        if kin_use.get("hip_aa_vel_R", False) and has_qd('RhipAA'):
            names.append("hip_aa_vel_R")

        # Hip Rotation
        if kin_use.get("hip_rot_deg_L", False) and has_q('LhipRot'):
            names.append("hip_rot_deg_L")
        if kin_use.get("hip_rot_deg_R", False) and has_q('RhipRot'):
            names.append("hip_rot_deg_R")
        if kin_use.get("hip_rot_vel_L", False) and has_qd('LhipRot'):
            names.append("hip_rot_vel_L")
        if kin_use.get("hip_rot_vel_R", False) and has_qd('RhipRot'):
            names.append("hip_rot_vel_R")

        # Knee FE
        if kin_use.get("knee_deg_L", True) and has_q('LkneeFE'):
            names.append("knee_deg_L")
        if kin_use.get("knee_deg_R", True) and has_q('RkneeFE'):
            names.append("knee_deg_R")
        if kin_use.get("knee_vel_L", True) and has_qd('LkneeFE'):
            names.append("knee_vel_L")
        if kin_use.get("knee_vel_R", True) and has_qd('RkneeFE'):
            names.append("knee_vel_R")

        # Ankle FE
        if kin_use.get("ankle_deg_L", True) and has_q('LankleFE'):
            names.append("ankle_deg_L")
        if kin_use.get("ankle_deg_R", True) and has_q('RankleFE'):
            names.append("ankle_deg_R")
        if kin_use.get("ankle_vel_L", True) and has_qd('LankleFE'):
            names.append("ankle_vel_L")
        if kin_use.get("ankle_vel_R", True) and has_qd('RankleFE'):
            names.append("ankle_vel_R")

        # 힙 무차원 선속도 (지금 spec에서는 False지만 일반화)
        dimless = spec["kin"].get("dimless", {})
        use_fwd = dimless.get("hip_forward", False)
        use_ver = dimless.get("hip_vertical", False)
        if use_fwd:
            if has_qd('LhipFE'):
                names.append("hip_forward_L")
            if has_qd('RhipFE'):
                names.append("hip_forward_R")
        if use_ver:
            if has_qd('LhipFE'):
                names.append("hip_vertical_L")
            if has_qd('RhipFE'):
                names.append("hip_vertical_R")

    # --- IMU ---
    if inc["acc"] or inc["gyr"]:
        imu_spec = spec["imu"]
        # build_feat에서는 use_tilt만 키 결정에 쓰이고, 채널 수는 acc_axes/gyr_axes로 결정
        if inc["acc"]:
            for ax in imu_spec["acc_axes"]:
                names.append(f"IMUAcc{ax}")
        if inc["gyr"]:
            for ax in imu_spec["gyr_axes"]:
                names.append(f"IMUGyr{ax}")

    # --- FSR ---
    if inc["fsrL"] or inc["fsrR"]:
        fsr_spec = spec["fsr"]
        # 개별 채널
        if inc["fsrL"]:
            for k in fsr_spec["L_channels"]:
                names.append(f"fsrL{k}")
        if inc["fsrR"]:
            for k in fsr_spec["R_channels"]:
                names.append(f"fsrR{k}")
        # summary
        sm = fsr_spec.get("summary", {})
        if sm.get("sumL", False):
            names.append("fsr_sumL")
        if sm.get("sumR", False):
            names.append("fsr_sumR")
        if sm.get("diffLR", False):
            names.append("fsr_diffLR")
        if sm.get("sumLR", False):
            names.append("fsr_sumLR")

    # --- GRF ---
    if inc.get("grf", False):
        grf_spec = spec["grf"]

        # left
        if grf_spec["left"].get("Fx", True):
            names.append("LGRFx")
        if grf_spec["left"].get("Fy", True):
            names.append("LGRFy")
        if grf_spec["left"].get("Fz", True):
            names.append("LGRFz")
        # right
        if grf_spec["right"].get("Fx", True):
            names.append("RGRFx")
        if grf_spec["right"].get("Fy", True):
            names.append("RGRFy")
        if grf_spec["right"].get("Fz", True):
            names.append("RGRFz")

        smg = grf_spec.get("summary", {})
        if smg.get("L_Fz", False):
            names.append("L_Fz")
        if smg.get("R_Fz", False):
            names.append("R_Fz")
        if smg.get("sum_Fz", False):
            names.append("sum_Fz")
        if smg.get("diff_Fz", False):
            names.append("diff_Fz")

    # --- dc, stance ---
    if inc["dc"]:
        names.append("dc")
    if inc["stanceL"]:
        names.append("stanceL")
    if inc["stanceR"]:
        names.append("stanceR")

    return names

def plot_input_worker(X, t, feat_names, sub_name, cond_name):
    """
    별도 프로세스에서 실행되는 플롯 함수.
    feature 이름을 보고 Kinematics / IMU / GRF / stance 그룹으로 나눠서
    그룹별 figure, 채널별 subplot으로 그림.
    """
    F = X.shape[1]

    # ---- 이름 기반 그룹핑 ----
    def classify(name: str) -> str:
        if name.startswith(("hip_", "knee_", "ankle_")):
            return "Kinematics"
        if ("IMUAcc" in name) or name.startswith(("IMUAX","IMUAY","IMUAZ")):
            return "IMU Acc"
        if ("IMUGyr" in name) or name.startswith(("IMUGX","IMUGY","IMUGZ")):
            return "IMU Gyr"
        if "GRF" in name:
            return "GRF"
        if "stance" in name or name == "dc":
            return "Gait state / stance"
        return "Other"

    groups = {}
    for idx, nm in enumerate(feat_names):
        g = classify(nm)
        groups.setdefault(g, []).append(idx)

    # ---- 그룹별 figure 생성 ----
    for gname, idxs in groups.items():
        n = len(idxs)
        if n == 0:
            continue

        fig, axes = plt.subplots(
            n, 1,
            figsize=(14, max(2.0, 1.2 * n)),
            sharex=True
        )
        if n == 1:
            axes = [axes]

        for ax, fi in zip(axes, idxs):
            ax.plot(t, X[:, fi])
            ax.set_ylabel(feat_names[fi])
            ax.grid(True, linestyle=":", linewidth=0.5)

        axes[-1].set_xlabel("Time [s]")
        fig.suptitle(
            f"{gname} — {sub_name} | {cond_name}",
            x=0.01, ha="left"
        )
        plt.tight_layout(rect=[0, 0, 1, 0.96])

    # 이 프로세스에서는 block=True로 두고 사용자가 닫을 때까지 유지
    plt.show()

def show_plot_in_background(X, t, feat_names, sub_name, cond_name):
    ctx = mp.get_context("spawn")  # Windows용
    p = ctx.Process(
        target=plot_input_worker,
        args=(X, t, feat_names, sub_name, cond_name)
    )
    p.daemon = False
    p.start()


def visualize_input_oneshot(Sub, cfg, sub_name, cond_name, fs=100, seconds=20):
    """
    특정 subject_name / cond_name 에 대해
    DEFAULT_FEAT_SPEC에서 'include'로 켜 놓은 입력만
    20초 길이 동안 플롯한다.
    """
    if sub_name not in cfg["sub_names"]:
        raise ValueError(f"unknown sub_name: {sub_name}")
    if cond_name not in cfg["cond_names"]:
        raise ValueError(f"unknown cond_name: {cond_name}")

    sub_idx = cfg["sub_names"].index(sub_name) + 1  # Sub dict는 1-based key
    cond_idx = cfg["cond_names"].index(cond_name)

    D = Sub[sub_idx]
    if "contact" not in D:
        raise RuntimeError("'contact'가 없습니다. load_all_subjects 이후 전체 전처리까지 끝난 다음에 호출해야 합니다.")

    cont = D["contact"][:, cond_idx]

    # 실제 모델에 들어가는 feature/target 생성
    X, Y = build_feat(D, cond_idx, cont, spec=DEFAULT_FEAT_SPEC)  # X: (T,F)
    if X.shape[0] == 0:
        print("해당 조건에서 usable sample이 없습니다.")
        return

    # 채널 이름: spec 기반으로 생성
    feat_names = get_feat_names_from_spec(D, spec=DEFAULT_FEAT_SPEC)
    F = X.shape[1]
    if len(feat_names) != F:
        print(f"[warn] feat_names({len(feat_names)}) != X.shape[1]({F}), generic 이름으로 대체")
        feat_names = [f"f{i}" for i in range(F)]

    # 20초 구간
    T_show = min(int(seconds * fs), X.shape[0])
    Xs = X[:T_show]
    t = np.arange(T_show) / fs

    print(f"[visualize_input_oneshot] {sub_name} / {cond_name} | T={Xs.shape[0]}, F={F}")

    half = (F + 1) // 2

    def _make_fig(data, start, end, title):
        n = end - start
        fig, axes = plt.subplots(n, 1, figsize=(14, max(2.0, n * 1.2)), sharex=True)
        if n == 1:
            axes = [axes]
        for ax, fi in zip(axes, range(start, end)):
            ax.plot(t, data[:, fi])
            ax.set_ylabel(feat_names[fi])
            ax.grid(True, linestyle=":", linewidth=0.5)
        axes[-1].set_xlabel("Time [s]")
        fig.suptitle(title, x=0.01, ha="left")
        plt.tight_layout(rect=[0, 0, 1, 0.96])
        return fig

    fig1 = _make_fig(
        Xs, 0, half,
        f"[{sub_name} | {cond_name}] Model input (0–{seconds}s) 1/2 (F={F})"
    )
    fig2 = None
    if half < F:
        fig2 = _make_fig(
            Xs, half, F,
            f"[{sub_name} | {cond_name}] Model input (0–{seconds}s) 2/2 (F={F})"
        )
    print(f"[visualize_input_oneshot] {sub_name} / {cond_name} | T={Xs.shape[0]}, F={F}")
    show_plot_in_background(Xs, t, feat_names, sub_name, cond_name)



class SpeedDataset(Dataset):
    def __init__(self, Sub_dict, use_sub_indices, L=10, S=5,
                 use_cond_ids=None, fit_norm=False, mu=None, sd=None, apply_norm=True):
        X_seq, Y_seq, feats_for_norm = [], [], []

        for i, D in Sub_dict.items():
            if i not in use_sub_indices: 
                continue
            T, C = D['contact'].shape
            cond_ids = range(C) if use_cond_ids is None else use_cond_ids
            for c in cond_ids:
                cont = D['contact'][:, c]
                X, Y = build_feat(D, c, cont)
                if len(X) < L:
                    continue
                feats_for_norm.append(X)
                for s in range(0, len(X)-L+1, S):
                    X_seq.append(X[s:s+L])
                    Y_seq.append(Y[s+L-1])

        if len(X_seq) == 0:
            self.X = torch.empty(0, L, 1, dtype=torch.float32)
            self.Y = torch.empty(0, 2, dtype=torch.float32)
            if fit_norm:
                self.mu, self.sd = np.zeros((1,1),np.float32), np.ones((1,1),np.float32)
            return

        X_seq = np.stack(X_seq); Y_seq = np.stack(Y_seq)

        if fit_norm:
            cat = np.concatenate(feats_for_norm, axis=0)
            mu = cat.mean(axis=0, keepdims=True)
            sd = cat.std(axis=0, keepdims=True) + 1e-6
            self.mu, self.sd = mu, sd
        else:
            self.mu, self.sd = mu, sd

        if apply_norm:
            assert (mu is not None) and (sd is not None), "mu/sd 누락"
            X_seq = (X_seq - mu) / sd

        self.X = torch.from_numpy(X_seq)
        self.Y = torch.from_numpy(Y_seq)

    def __len__(self): 
        return self.X.shape[0]

    def __getitem__(self, idx): 
        return self.X[idx], self.Y[idx]

# ---------------- Model ----------------

# Depthwise separable conv: depthwise(groups=C) → pointwise(1x1)
class DWSepConv1d(nn.Module):
    def __init__(self, C, k, dilation=1, causal=False, p=0.0):
        super().__init__()
        pad = (k-1)*dilation
        if causal:
            self.pad = (pad, 0)
        else:
            self.pad = (pad//2, pad - pad//2)
        self.dw = nn.Conv1d(C, C, kernel_size=k, dilation=dilation,
                            padding=0, groups=C, bias=False)
        self.pw = nn.Conv1d(C, C, kernel_size=1, bias=False)
        self.act = nn.GELU()
        self.bn  = nn.BatchNorm1d(C)
        self.do  = nn.Dropout(p)
        self.causal = causal
    def forward(self, x):
        # x: (B, C, L)
        x = nn.functional.pad(x, self.pad)
        x = self.dw(x)
        x = self.pw(x)
        x = self.act(x)
        x = self.bn(x)
        return self.do(x)

class TCNBlock(nn.Module):
    def __init__(self, C, k_list=(5,3), dilations=(1,2,4), causal=False, p=0.0):
        super().__init__()
        layers = []
        for d in dilations:
            for k in k_list:
                layers += [DWSepConv1d(C, k, dilation=d, causal=causal, p=p)]
        self.net = nn.Sequential(*layers)
        self.proj = nn.Identity()
    def forward(self, x):
        return self.net(x) + x  # residual

class TinyAttn(nn.Module):
    def __init__(self, H):
        super().__init__()
        self.q = nn.Linear(H, H, bias=False)
        self.k = nn.Linear(H, H, bias=False)
        self.v = nn.Linear(H, H, bias=False)
        self.out = nn.Linear(H, H, bias=False)
    def forward(self, h):                 # h: (B, L, H)
        q = self.q(h[:, -1:, :])         # (B, 1, H)
        k = self.k(h)                     # (B, L, H)
        v = self.v(h)                     # (B, L, H)
        att = torch.softmax(torch.matmul(q, k.transpose(1,2))/ (h.size(-1)**0.5), dim=-1)  # (B,1,L)
        out = torch.matmul(att, v)        # (B,1,H)
        return self.out(out.squeeze(1))   # (B,H)

class Model_TCN_GRU(nn.Module):
    # Linear(F→d) → TCN×(2~3) → GRU(h) → MLP
    def __init__(self, F, d=128, h=160, p=0.2, blocks=3, kset=(5,3)):
        super().__init__()
        self.proj = nn.Sequential(nn.Linear(F, d), nn.GELU(), nn.Dropout(p), nn.LayerNorm(d))
        self.tcns = nn.ModuleList([TCNBlock(d, k_list=kset, dilations=(1,2,4), causal=False, p=p) for _ in range(blocks)])
        self.gru  = nn.GRU(input_size=d, hidden_size=h, batch_first=True)
        self.head = nn.Sequential(nn.Linear(h, h//2), nn.GELU(), nn.Dropout(p), nn.Linear(h//2, 2))
    def forward(self, x):                 # (B, L, F)
        h = self.proj(x)                  # (B, L, d)
        hc = h.transpose(1,2)             # (B, d, L)
        for t in self.tcns: hc = t(hc)
        h = hc.transpose(1,2)             # (B, L, d)
        h, _ = self.gru(h)                # (B, L, h)
        return self.head(h[:, -1, :])

class Model_GRU2(nn.Module):
    # Conv1d(F→F, k=3, causal) → GRU×2 → MLP
    def __init__(self, F, h=160, p=0.2):
        super().__init__()
        self.mix = nn.Conv1d(F, F, kernel_size=3, padding=2, dilation=2, groups=1, bias=False)  # 가벼운 채널믹싱
        self.bn  = nn.BatchNorm1d(F)
        self.act = nn.GELU()
        self.gru1 = nn.GRU(input_size=F, hidden_size=h, batch_first=True)
        self.gru2 = nn.GRU(input_size=h, hidden_size=h, batch_first=True)
        self.head = nn.Sequential(nn.Linear(h, h//2), nn.GELU(), nn.Dropout(p), nn.Linear(h//2, 2))
    def forward(self, x):                 # (B, L, F)
        z = self.act(self.bn(self.mix(x.transpose(1,2)))).transpose(1,2)
        h, _ = self.gru1(z)
        h, _ = self.gru2(h)
        return self.head(h[:, -1, :])


class Model_GRU_Stack(nn.Module):
    """
    순수 GRU + MLP
    GRU layer 수를 grid로 스윕하는 용도.
    """
    def __init__(self, F, h=288, layers=2, p=0.3):
        super().__init__()
        self.gru = nn.GRU(
            input_size=F,
            hidden_size=h,
            num_layers=layers,
            batch_first=True,
            dropout=p if layers > 1 else 0.0  # PyTorch 규칙: num_layers>1일 때만 dropout 동작
        )
        self.head = nn.Sequential(
            nn.Linear(h, h // 2),
            nn.GELU(),
            nn.Dropout(p),
            nn.Linear(h // 2, 2)
        )

    def forward(self, x):  # x: (B, L, F)
        out, _ = self.gru(x)     # out: (B, L, h)
        last = out[:, -1, :]     # 마지막 타임스텝
        return self.head(last)   # (B, 2)


class Model_TinyTCN(nn.Module):
    # Linear(F→d) → [DW-Sep TCN]×blocks → GAP(time) → MLP
    def __init__(self, F, d=128, blocks=3, p=0.2, kset=(5,3)):
        super().__init__()
        self.proj = nn.Sequential(nn.Linear(F, d), nn.GELU(), nn.Dropout(p), nn.LayerNorm(d))
        self.tcns = nn.ModuleList([TCNBlock(d, k_list=kset, dilations=(1,2,4), causal=False, p=p) for _ in range(blocks)])
        self.head = nn.Sequential(nn.Linear(d, d//2), nn.GELU(), nn.Dropout(p), nn.Linear(d//2, 2))
    def forward(self, x):
        h = self.proj(x).transpose(1,2)   # (B,d,L)
        for t in self.tcns: h = t(h)
        h = h.mean(dim=-1)                # GAP over time
        return self.head(h)

class Model_TCN_LSTM(nn.Module):
    # Linear → TCN×3 → LSTM(h) → MLP
    def __init__(self, F, d=128, h=192, p=0.2, kset=(5,3)):
        super().__init__()
        self.proj = nn.Sequential(nn.Linear(F, d), nn.GELU(), nn.Dropout(p), nn.LayerNorm(d))
        self.tcns = nn.ModuleList([TCNBlock(d, k_list=kset, dilations=(1,2,4), causal=False, p=p) for _ in range(3)])
        self.lstm = nn.LSTM(input_size=d, hidden_size=h, batch_first=True)
        self.head = nn.Sequential(nn.Linear(h, h//2), nn.GELU(), nn.Dropout(p), nn.Linear(h//2, 2))
    def forward(self, x):
        h = self.proj(x).transpose(1,2)
        for t in self.tcns: h = t(h)
        h = h.transpose(1,2)
        h, _ = self.lstm(h)
        return self.head(h[:, -1, :])

class Model_SmallTCN_LSTM_Attn(nn.Module):
    # Linear → TCN → LSTM → TinyAttn → MLP
    def __init__(self, F, d=128, h=160, p=0.2, kset=(5,3)):
        super().__init__()
        self.proj = nn.Sequential(nn.Linear(F, d), nn.GELU(), nn.Dropout(p), nn.LayerNorm(d))
        self.tcn  = TCNBlock(d, k_list=kset, dilations=(1,2,4), causal=False, p=p)
        self.lstm = nn.LSTM(input_size=d, hidden_size=h, batch_first=True)
        self.attn = TinyAttn(h)
        self.head = nn.Sequential(nn.Linear(h, h//2), nn.GELU(), nn.Dropout(p), nn.Linear(h//2, 2))
    def forward(self, x):
        h = self.proj(x).transpose(1,2)
        h = self.tcn(h).transpose(1,2)
        h, _ = self.lstm(h)
        h = self.attn(h)                  # (B,H)
        return self.head(h)

def build_model_by_arch(arch: str, input_dim: int, hp: dict, p: float):
    kset = tuple(hp.get("kset", (5,3)))
    if arch == "TCN_GRU":
        return Model_TCN_GRU(F=input_dim,
                             d=hp.get("d",128), h=hp.get("h",160),
                             p=p, blocks=hp.get("blocks",3), kset=kset)
    if arch == "GRU2":
        return Model_GRU2(F=input_dim, h=hp.get("h",160), p=p)
    if arch == "GRU_Stack":
        return Model_GRU_Stack(
            F=input_dim,
            h=hp.get("h", 288),
            layers=hp.get("layers", 2),
            p=p
        )
    if arch == "TinyTCN":
        return Model_TinyTCN(F=input_dim,
                             d=hp.get("d",128), blocks=hp.get("blocks",3),
                             p=p, kset=kset)
    if arch == "TCN_LSTM":
        return Model_TCN_LSTM(F=input_dim,
                              d=hp.get("d",128), h=hp.get("h",192),
                              p=p, kset=kset)
    if arch == "SmallTCN_LSTM_Attn":
        return Model_SmallTCN_LSTM_Attn(F=input_dim,
                                        d=hp.get("d",128), h=hp.get("h",160),
                                        p=p, kset=kset)
    # 기본: 기존 SpeedNet (TCN+LSTM+MHA)
    if arch == "TCN_LSTM_MHA":
        return SpeedNet(input_dim=input_dim,
                        d_model=hp.get("d", CONFIG.get("d_model",192)),
                        lstm_h=hp.get("h", CONFIG.get("lstm_h",512)),
                        out_dim=CONFIG.get("out_dim",2),
                        p=p, attn_heads=CONFIG.get("attn_heads",1))
    raise ValueError(f"unknown arch: {arch}")

def iter_grid(grid: dict):
    keys = list(grid.keys())
    vals = [grid[k] for k in keys]
    for combo in product(*vals):
        yield dict(zip(keys, combo))
        
class SpeedNet(nn.Module):
    def __init__(self, input_dim, d_model, lstm_h, out_dim, p=0.2, attn_heads=4):
        super().__init__()
        self.input_dim = input_dim

        # 입력 투영
        self.proj = nn.Sequential(
            nn.Linear(input_dim, d_model),
            nn.GELU(),
            nn.Dropout(p),
            nn.LayerNorm(d_model)
        )

        # TCN 블록(배치정규화 포함)
        def tblock(d):
            return nn.Sequential(
                nn.Conv1d(d_model, d_model, kernel_size=5, padding=2*d, dilation=d),
                nn.GELU(),
                nn.BatchNorm1d(d_model),
                nn.Conv1d(d_model, d_model, kernel_size=3, padding=1*d, dilation=d),
                nn.GELU(),
                nn.Dropout(p)
            )
        self.tcn1, self.tcn2, self.tcn3 = tblock(1), tblock(2), tblock(4)
        self.tcn_ln = nn.LayerNorm(d_model)

        # 시퀀스 인코더
        self.bi_lstm = nn.LSTM(d_model, lstm_h, batch_first=True, bidirectional=False)

        # 주의집중층 2단계 + 잔차
        self.attn1 = nn.MultiheadAttention(embed_dim=lstm_h, num_heads=attn_heads, dropout=p, batch_first=False)
        self.attn2 = nn.MultiheadAttention(embed_dim=lstm_h, num_heads=attn_heads, dropout=p, batch_first=False)

        # 회귀 헤드 확장
        self.regressor = nn.Sequential(
            nn.Linear(lstm_h, lstm_h // 2), nn.GELU(), nn.Dropout(p),
            nn.Linear(lstm_h // 2, lstm_h // 4), nn.GELU(), nn.Dropout(max(p/2, 0.05)),
            nn.Linear(lstm_h // 4, out_dim)
        )

    def forward(self, x):
        if x.ndim == 2:
            x = x.unsqueeze(1)

        h = self.proj(x)            # (B, L, d_model)

        # TCN with residuals
        h_c = h.transpose(1, 2)     # (B, d_model, L)
        r = h_c
        h_c = self.tcn1(h_c) + r; r = h_c
        h_c = self.tcn2(h_c) + r; r = h_c
        h_c = self.tcn3(h_c) + r

        h = h_c.transpose(1, 2)     # (B, L, d_model)
        h = self.tcn_ln(h)

        # LSTM
        h, _ = self.bi_lstm(h)      # (B, L, lstm_h)

        # MHA × 2 + residual
        qkv = h.transpose(0, 1)     # (L, B, lstm_h)
        a1, _ = self.attn1(qkv, qkv, qkv)
        a2, _ = self.attn2(a1, a1, a1)
        a = a1 + a2

        feat = a[-1]                # 마지막 타임스텝 (B, lstm_h)
        return self.regressor(feat) # (B, out_dim)

# ---------------- Live plot ----------------
class LivePlot:
    def __init__(self):
        self.fig, self.ax = plt.subplots()
        self.losses = []
        self.best_loss = float('inf')
        self.line, = self.ax.plot([], [], label='Val Loss')
        self.best_line = None
        self.ax.set_xlim(0,1)
        self.ax.set_xlabel('Epoch')
        self.ax.set_ylabel('Loss')
        self.ax.legend(loc='upper right')
        plt.ion(); plt.show()

    def update(self, epoch, loss):
        self.losses.append(loss)
        self.line.set_xdata(range(1, len(self.losses)+1))
        self.line.set_ydata(self.losses)
        if loss < self.best_loss:
            self.best_loss = loss
            if self.best_line:
                self.best_line.remove()
            self.best_line = self.ax.axhline(y=self.best_loss, linestyle='--', label='Best')
        self.ax.set_xlim(0, len(self.losses)+1)
        self.ax.relim(); self.ax.autoscale_view(scalex=False, scaley=True)
        self.ax.legend(loc='upper right')
        plt.pause(0.05)

# ---------------- Helper ----------------
def assert_disjoint(cfg):
    tr, va, te = set(cfg["train_subs"]), set(cfg["val_subs"]), set(cfg.get("test_subs", []))
    assert tr.isdisjoint(va) and tr.isdisjoint(te) and va.isdisjoint(te), \
        f"Split overlap: train∩val={tr&va}, train∩test={tr&te}, val∩test={va&te}"

def evaluate(model, dl, device, loss_fn):
    model.eval(); tot = 0.0; n = 0
    with torch.no_grad():
        for Xb, Yb in dl:
            Xb, Yb = Xb.to(device, non_blocking=True), Yb.to(device, non_blocking=True)
            tot += loss_fn(model(Xb), Yb).item()
            n   += 1
    return tot / max(1, n)

# ---------------- Train loop ----------------
def train_model(Sub, cfg, device="cpu", model_fn=None):
    assert_disjoint(cfg)
    run = wandb.init(
        project=os.environ.get("WANDB_PROJECT","SpeedEstimator"),
        entity=os.environ.get("WANDB_ENTITY", None),
        name=os.path.splitext(os.path.basename(cfg.get("save_name", "run.pt")))[0],
        group=cfg.get("arch","SpeedNet"),
        config=cfg
    )

    # 기본 save_name 자동 생성
    if "save_name" not in cfg:
        arch = cfg.get("arch", "TCN_LSTM_MHA")
        cfg["save_name"] = f"{cfg['save_prefix']}{arch}_L{cfg.get('L',192)}_S{cfg.get('S',3)}.pt"

    # 피험자 인덱스
    train_sub_indices = _names_to_indices(cfg["sub_names"], cfg["train_subs"])
    val_sub_indices   = _names_to_indices(cfg["sub_names"], cfg["val_subs"])
    test_sub_indices  = _names_to_indices(cfg["sub_names"], cfg.get("test_subs", [])) if cfg.get("test_subs") else []

    # 조건 id
    cond_ids = _conds_to_ids(cfg["cond_names"], cfg["use_conds"])

    # Dataset
    ds_train = SpeedDataset(Sub, use_sub_indices=train_sub_indices, L=cfg["L"], S=cfg["S"],
                            use_cond_ids=cond_ids, fit_norm=True)
    mu, sd = ds_train.mu, ds_train.sd
    ds_val  = SpeedDataset(Sub, use_sub_indices=val_sub_indices,   L=cfg["L"], S=cfg["S"],
                           use_cond_ids=cond_ids, fit_norm=False, mu=mu, sd=sd)
    ds_test = None
    if test_sub_indices:
        ds_test = SpeedDataset(Sub, use_sub_indices=test_sub_indices, L=cfg["L"], S=cfg["S"],
                               use_cond_ids=cond_ids, fit_norm=False, mu=mu, sd=sd)

    # === Gradient Accumulation 설정 ===
    per_device_bs = int(cfg["batch_size"])
    global_bs     = int(cfg.get("eff_batch_size", per_device_bs))
    accum_steps   = max(1, (global_bs + per_device_bs - 1) // per_device_bs)  # ceil
    if run is not None:
        wandb.config.update(
            {"accum_steps": accum_steps, "effective_global_bs": accum_steps*per_device_bs},
            allow_val_change=True
        )

    # ---- Dataloaders ----
    pin = (device.type=="cuda")

    # === num_workers / prefetch 설정 ===
    if str(cfg.get("num_workers","auto")).lower() == "auto":
        _cpu = os.cpu_count() or 4
        _nw = max(2, _cpu // 2)
    else:
        _nw = int(cfg["num_workers"])
    _pf = int(cfg.get("prefetch_factor", 2))

    dl  = DataLoader(
        ds_train, batch_size=cfg["batch_size"], shuffle=True,
        num_workers=_nw, pin_memory=pin, persistent_workers=True,
        prefetch_factor=_pf
    )
    vdl = DataLoader(
        ds_val, batch_size=cfg["val_batch_size"], shuffle=False,
        num_workers=_nw, pin_memory=pin, persistent_workers=True,
        prefetch_factor=_pf
    )

    # test loader: single-process (Windows 충돌 회피)
    tdl = None
    if ds_test:
        tdl = DataLoader(
            ds_test, batch_size=cfg["val_batch_size"], shuffle=False,
            num_workers=0, pin_memory=False  # prefetch_factor/persistent_workers 넣지 않음
        )

    # Model
    input_dim = ds_train.X.shape[2]
    if model_fn is None:
        model = SpeedNet(input_dim=input_dim,
                         d_model=cfg.get("d_model",192),
                         lstm_h=cfg.get("lstm_h",512),
                         out_dim=cfg.get("out_dim",2),
                         p=cfg["dropout_p"],
                         attn_heads=cfg.get("attn_heads",1)).to(device)
    else:
        model = model_fn(input_dim).to(device)

    optimizer = optim.AdamW(
        model.parameters(), lr=cfg["lr"], weight_decay=cfg["weight_decay"],
        fused=True if torch.cuda.is_available() else False
    )
    scheduler = optim.lr_scheduler.CosineAnnealingWarmRestarts(
        optimizer, T_0=cfg["cos_T0"], T_mult=cfg["cos_Tmult"], eta_min=cfg["eta_min"]
    )
    # 회귀용 손실: MSE
    loss_fn = nn.MSELoss()

    use_amp = (device.type=="cuda")
    from torch.amp import GradScaler, autocast   # 최신 권장 API
    scaler = GradScaler('cuda', enabled=use_amp)

    best_val = float('inf')
    patience = cfg["patience"]
    wait = 0

    # 라이브 플롯
    plotter = LivePlot() if cfg.get("live_plot", True) else None

    # 임시 베스트 체크포인트 경로
    out_dir = os.path.dirname(cfg["save_name"]) or "."
    base = os.path.splitext(os.path.basename(cfg["save_name"]))[0]
    tmp_best = os.path.join(out_dir, f".__best_{base}.pt")

    # ETA용 지수이동평균
    ema_epoch = None

    for epoch in range(1, cfg["epochs"]+1):
        model.train(); run_loss = 0.0
        t_epoch0 = time.perf_counter()  # 에폭 타이머 시작
        optimizer.zero_grad(set_to_none=True)

        for b, (Xb, Yb) in enumerate(tqdm(dl, desc=f"Epoch {epoch}/{cfg['epochs']}")):
            Xb, Yb = Xb.to(device, non_blocking=True), Yb.to(device, non_blocking=True)

            with autocast('cuda', enabled=use_amp):
                pred = model(Xb)
                loss = loss_fn(pred, Yb)

            run_loss += loss.item()  # 로깅용 원래 loss

            # === 누적 역전파 ===
            if use_amp:
                scaler.scale(loss / accum_steps).backward()
            else:
                (loss / accum_steps).backward()

            # 이번 마이크로배치가 누적 스텝의 끝인지 판단
            do_step = ((b + 1) % accum_steps == 0) or ((b + 1) == len(dl))
            if do_step:
                if use_amp:
                    scaler.unscale_(optimizer)
                torch.nn.utils.clip_grad_norm_(model.parameters(), 1.0)

                if use_amp:
                    scaler.step(optimizer)
                    scaler.update()
                else:
                    optimizer.step()

                optimizer.zero_grad(set_to_none=True)

                # 스케줄러는 실제 옵티마이저 스텝에만 1회
                t_cur = (epoch - 1) + (b + 1) / max(1, len(dl))
                scheduler.step(t_cur)

        # 검증
        model.eval(); vloss = 0.0
        with torch.no_grad(), autocast('cuda', enabled=use_amp):
            for Xb, Yb in tqdm(vdl, desc="Validation", leave=False):
                Xb, Yb = Xb.to(device, non_blocking=True), Yb.to(device, non_blocking=True)
                vloss += loss_fn(model(Xb), Yb).item()

        train_avg = run_loss / max(1, len(dl))
        val_avg   = vloss    / max(1, len(vdl))

        # ETA 계산 및 출력
        epoch_sec = time.perf_counter() - t_epoch0
        ema_epoch = epoch_sec if ema_epoch is None else 0.3*epoch_sec + 0.7*ema_epoch
        rem_epochs = cfg["epochs"] - epoch
        eta_sec = max(0.0, ema_epoch * rem_epochs)

        print(
            f"Epoch {epoch}/{cfg['epochs']} | train {train_avg:.6f} | val {val_avg:.6f} "
            f"| epoch {epoch_sec:.1f}s | ETA {str(timedelta(seconds=int(eta_sec)))}"
        )
        wandb.log({
            "epoch": epoch,
            "train_loss": train_avg,
            "val_loss": val_avg,
            "best_val": best_val,
            "lr": optimizer.param_groups[0]["lr"],
            "time/epoch_sec": epoch_sec,
            "time/eta_sec": eta_sec,
        })

        # 라이브 플롯
        if plotter:
            try:
                plotter.update(epoch, val_avg)
            except Exception:
                pass

        # early stopping + 임시 체크포인트 저장
        if val_avg + 1e-6 < best_val:
            best_val = val_avg; wait = 0
            torch.save({'state_dict': model.state_dict(), 'mu': mu, 'sd': sd, 'input_dim': input_dim}, tmp_best)
        else:
            wait += 1
            if wait >= patience:
                print(f"Early stopping at epoch {epoch}")
                break

    # 최종 저장
    ckpt = torch.load(tmp_best, map_location=device)
    model.load_state_dict(ckpt['state_dict'])

    torch.save({
        "model_state": model.state_dict(),
        "mu": mu, "sd": sd, "input_dim": input_dim,
        "arch": cfg.get("arch", "TCN_LSTM_MHA"),
        "hp": cfg.get("hp", {})  # ← 수정
    }, cfg["save_name"])

    try:
        os.remove(tmp_best)
    except Exception:
        pass

    # 테스트 세트 평가 로그
    if tdl is not None and len(ds_test) > 0:
        test_loss = safe_evaluate(model, tdl, device, loss_fn)
        wandb.log({"test_loss": test_loss})
        wandb.summary["test_loss"] = test_loss

    wandb.finish()
    return model, mu, sd


def count_total_runs(base_cfg):
    total = 0
    for arch in base_cfg.get("run_list", []):
        grid = base_cfg["models"][arch]["grid"]
        # 그리드 조합 수
        n = 1
        for k, v in grid.items():
            n *= len(v)
        total += n
    return total

def run_sweeps(Sub, base_cfg, device):
    from datetime import timedelta
    results = []

    total_runs = count_total_runs(base_cfg)
    run_idx = 0
    ema_run_sec = None  # 전역 러닝타임 EMA

    for arch in base_cfg.get("run_list", []):
        grid = base_cfg["models"][arch]["grid"]

        for hp in iter_grid(grid):
            run_idx += 1
            # 현재 조합의 L,S 반영
            cfg = dict(base_cfg)
            cfg["arch"] = arch
            cfg["L"] = hp.get("L", cfg.get("L", 192))
            cfg["S"] = hp.get("S", cfg.get("S", 3))
            cfg["hp"] = hp

            # 저장 이름 생성
            tag_parts = []
            for k in sorted(hp.keys()):
                if k in ("L","S"): 
                    continue
                v = hp[k]
                if isinstance(v, tuple):
                    v = "x".join(map(str, v))
                tag_parts.append(f"{k}{v}")
            tag = "_".join(tag_parts) if tag_parts else "base"
            cfg["save_name"] = f"{cfg['save_prefix']}{arch}_L{cfg['L']}_S{cfg['S']}_{tag}.pt"

            # 팩토리
            def model_factory(input_dim, _arch=arch, _hp=hp, _p=cfg["dropout_p"]):
                return build_model_by_arch(_arch, input_dim, _hp, _p)

            print(f"[RUN {run_idx}/{total_runs}] {arch} | hp={hp} | save={cfg['save_name']}")

            t0 = time.perf_counter()
            model, mu, sd = train_model(Sub, cfg, device=device, model_fn=model_factory)
            run_sec = time.perf_counter() - t0

            # 전역 ETA 갱신
            ema_run_sec = run_sec if ema_run_sec is None else 0.3*run_sec + 0.7*ema_run_sec
            rem = total_runs - run_idx
            global_eta = timedelta(seconds=int(max(0.0, ema_run_sec * rem)))

            print(f"[DONE {run_idx}/{total_runs}] took {run_sec:.1f}s | Global ETA {global_eta}")

            # W&B에도 기록(선택)
            try:
                wandb.log({
                    "sweep/run_index": run_idx,
                    "sweep/total_runs": total_runs,
                    "sweep/this_run_sec": run_sec,
                    "sweep/global_eta_sec": max(0.0, (ema_run_sec or run_sec) * rem),
                })
            except Exception:
                pass

            results.append((arch, hp, cfg["save_name"]))

    print("[SWEEP COMPLETE]")
    return results

def safe_evaluate(model, dl, device, loss_fn):
    try:
        return evaluate(model, dl, device, loss_fn)
    except RuntimeError as e:
        msg = str(e)
        if "DataLoader worker" in msg or "exited unexpectedly" in msg:
            # 단일 프로세스 로더로 재시도
            from torch.utils.data import DataLoader
            dl_fb = DataLoader(
                dl.dataset, batch_size=dl.batch_size, shuffle=False,
                num_workers=0, pin_memory=False
            )
            return evaluate(model, dl_fb, device, loss_fn)
        raise

# ---------------- main ----------------
if __name__ == "__main__":
    cfg = CONFIG
    assert_disjoint(cfg)

    Sub = load_all_subjects(path=cfg["path"], sub_names=cfg["sub_names"],
                            cond_names=cfg["cond_names"], data_length=cfg["data_length"])
    count_and_fix_nans_in_Sub(Sub)
    preprocess_back_imu(Sub, pitch_is_deg=True)

    for i, D in Sub.items():
        mass = float(D.get('mass_kg', np.nan))
        normalize_fsr_by_mass(Sub, i, mass)
        normalize_grf_by_mass(Sub, i, mass)

    apply_tick_filter(Sub, passes=2)
    
    visualize_input_oneshot(Sub, cfg, sub_name="S005", cond_name="level_08mps")
    visualize_input_oneshot(Sub, cfg, sub_name="S005", cond_name="level_12mps")
    
    import torch.multiprocessing as mp
    mp.set_start_method("spawn", force=True)

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print("현재 학습 장치:", device)

    # 스윕 실행 또는 단일 실행
    if cfg.get("run_list"):
        _ = run_sweeps(Sub, cfg, device)
    else:
        # 단일 실행: 기존 SpeedNet
        def default_factory(input_dim):
            return build_model_by_arch("TCN_LSTM_MHA", input_dim, hp={}, p=cfg["dropout_p"])
        model, mu, sd = train_model(Sub, cfg, device=device, model_fn=default_factory)
