"""Databank H5 motion-data integrity check.

Walks every `combined_data_*.h5` in this folder and runs a comprehensive
battery of structural, statistical and physics-plausibility checks.

For a new h5 file, just place it in this directory and run:
    python Databank/data_integrity_check.py

Outputs:
  Databank/integrity_report/
    report.md         - human-readable summary
    summary.json      - raw stats / flags / verdicts
    plots/
      <file_stem>/
        coverage.png             - per-subject × cond coverage
        lr_symmetry.png          - L/R RMS ratios per subject
        imu_mount.png            - trunk-IMU mount orientation per subject
        anomalies_summary.png    - one-glance grid of every flagged finding
        <subject>/
          timeseries_<cond>_<asl>.png
          gaitcycle_<cond>_<asl>.png  (gait conditions only)
"""
from __future__ import annotations

import json
import shutil
import subprocess
import textwrap
from collections import defaultdict
from pathlib import Path

import h5py
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

# ============================================================== CONFIG

DATABANK = Path(__file__).resolve().parent
OUT_DIR = DATABANK / "integrity_report"
PLOT_DIR = OUT_DIR / "plots"
PLOT_SUBJECT = "S001"  # whose detailed timeseries / gaitcycle plots to draw

# expected protocol per condition: assist levels + trial count.
# (defines what 'compliance' means for each subject)
EXPECTED_CONDITIONS = {
    # cond            speed    pitch  kind     asls/lv0 trials
    "level_075mps":  dict(speed=0.75, pitch=0.0,  kind="level",   asls=("lv0",), trials_per_asl=2),
    "level_100mps":  dict(speed=1.00, pitch=0.0,  kind="level",   asls=("lv0",), trials_per_asl=2),
    "level_125mps":  dict(speed=1.25, pitch=0.0,  kind="level",   asls=("lv0",), trials_per_asl=2),
    "incline_10deg": dict(speed=None, pitch=10.0, kind="incline", asls=("lv0",), trials_per_asl=2),
    "decline_5deg":  dict(speed=None, pitch=-5.0, kind="decline", asls=("lv0",), trials_per_asl=2),
    "accel_sine":    dict(speed=None, pitch=0.0,  kind="sine",    asls=("lv0",), trials_per_asl=2),
    "stopandgo":     dict(speed=None, pitch=0.0,  kind="sng",     asls=("lv0",), trials_per_asl=4),
    "stand":         dict(speed=None, pitch=None, kind="stand",   asls=("lv0",), trials_per_asl=2),
    "dynamicpose":   dict(speed=None, pitch=None, kind="dyn",     asls=("lv0",), trials_per_asl=2),
}
GAIT_KINDS = {"level", "incline", "decline", "sine", "sng"}
# Required top-level groups in every trial.
REQUIRED_TRIAL_GROUPS = {"common", "robot", "treadmill", "forceplate", "mocap"}

# ---- thresholds ----
TH = dict(
    nan_frac_warn=0.01,
    nan_frac_fail=0.05,
    flat_var=1e-12,
    dt_jitter_cv=0.05,
    dt_gap_factor=5,
    loop_step_tol=1,
    speed_tol_frac=0.20,
    pitch_tol_deg=2.0,
    mad_zscore=4.0,
    lr_ratio_lo=0.7,
    lr_ratio_hi=1.3,
    sub_height_mm=(1400, 2100),
    sub_weight_kg=(30, 150),
    sub_age_yr=(10, 100),
    grf_hs_threshold_n=50.0,
    grf_min_cycle_s=0.5,
    grf_max_cycle_s=2.0,
    motor_hip_corr_min=0.6,
    grf_weight_match_tol=0.5,
    # extended checks
    quat_mag_dev=0.05,                # |q| must be in [0.95, 1.05]
    fp_baseline_drift_n=100.0,        # |head − tail| GRF, sliding-window
    imu_bias_drift_dps=5.0,           # |mean gyro| over trial
    phase_mismatch_frac=0.15,         # |robot/GRF stride-freq − 1|
    gait_stage_invalid_frac=0.05,     # invalid transitions ratio
    imu_tilt_warn_deg=8.0,            # subject trunk-IMU mount tilt
    imu_lateral_lean_deg=5.0,
    subject_wide_hip_dev_deg=10.0,    # subject hip baseline vs cohort
    # outlier detection on per-subject mean of an mocap angle, |Δ from cohort
    # median| in degrees.
    abs_dev_outlier_deg=8.0,
)

# ---- channel sets ----

TS_PANELS = [
    ("treadmill/left/speed_leftbelt",  "speed_L (m/s)"),
    ("treadmill/right/speed_rightbelt","speed_R (m/s)"),
    ("treadmill/pitch",                "pitch (deg)"),
    ("forceplate/grf/left/z",          "GRF_L_z (N)"),
    ("forceplate/grf/right/z",         "GRF_R_z (N)"),
    ("mocap/kin_q/hip_flexion_l",      "hip_L (deg)"),
    ("mocap/kin_q/hip_flexion_r",      "hip_R (deg)"),
    ("mocap/kin_q/knee_angle_l",       "knee_L (deg)"),
    ("mocap/kin_q/knee_angle_r",       "knee_R (deg)"),
    ("mocap/kin_q/ankle_angle_l",      "ankle_L (deg)"),
    ("mocap/kin_q/ankle_angle_r",      "ankle_R (deg)"),
    ("robot/left/torque",              "robot tau_L (Nm)"),
    ("robot/right/torque",             "robot tau_R (Nm)"),
    ("robot/left/motor_angle",         "motor_L (deg)"),
    ("robot/right/motor_angle",        "motor_R (deg)"),
    ("robot/imu/back_imu/accel_x",     "back_acc_x (m/s²)"),
    ("robot/imu/back_imu/accel_y",     "back_acc_y (m/s²)"),
    ("robot/imu/back_imu/accel_z",     "back_acc_z (m/s²)"),
    ("robot/imu/back_imu/gyro_x",      "back_gyr_x (dps)"),
    ("robot/imu/back_imu/gyro_y",      "back_gyr_y (dps)"),
]

GC_PANELS = [
    ("mocap/kin_q/hip_flexion_l",  "hip_L"),
    ("mocap/kin_q/hip_flexion_r",  "hip_R"),
    ("mocap/kin_q/knee_angle_l",   "knee_L"),
    ("mocap/kin_q/knee_angle_r",   "knee_R"),
    ("mocap/kin_q/ankle_angle_l",  "ankle_L"),
    ("mocap/kin_q/ankle_angle_r",  "ankle_R"),
    ("forceplate/grf/left/z",      "GRF_L_z"),
    ("forceplate/grf/right/z",     "GRF_R_z"),
    ("robot/left/torque",          "tau_L"),
    ("robot/right/torque",         "tau_R"),
    ("robot/imu/back_imu/accel_x", "back_acc_x"),
    ("robot/imu/back_imu/accel_z", "back_acc_z"),
]

DIST_CHANNELS = [
    "mocap/kin_q/hip_flexion_l", "mocap/kin_q/hip_flexion_r",
    "mocap/kin_q/knee_angle_l",  "mocap/kin_q/knee_angle_r",
    "mocap/kin_q/ankle_angle_l", "mocap/kin_q/ankle_angle_r",
    "forceplate/grf/left/z",     "forceplate/grf/right/z",
    "robot/left/torque",         "robot/right/torque",
    "robot/imu/back_imu/accel_x","robot/imu/back_imu/accel_y","robot/imu/back_imu/accel_z",
    "robot/imu/back_imu/gyro_x", "robot/imu/back_imu/gyro_y", "robot/imu/back_imu/gyro_z",
    "treadmill/left/speed_leftbelt", "treadmill/right/speed_rightbelt",
    "treadmill/pitch",
]

LR_PAIRS = [
    ("mocap/kin_q/knee_angle_l",  "mocap/kin_q/knee_angle_r",  "knee"),
    ("mocap/kin_q/hip_flexion_l", "mocap/kin_q/hip_flexion_r", "hip"),
    ("mocap/kin_q/ankle_angle_l", "mocap/kin_q/ankle_angle_r", "ankle"),
    ("forceplate/grf/left/z",     "forceplate/grf/right/z",    "GRF_z"),
    ("robot/left/torque",         "robot/right/torque",        "robot_tau"),
    ("robot/left/motor_angle",    "robot/right/motor_angle",   "motor_angle"),
]

# ============================================================== UTILITIES


def file_in_use(path: Path) -> bool:
    try:
        r = subprocess.run(["fuser", str(path)], capture_output=True, text=True)
        return bool(r.stdout.strip() or r.stderr.strip())
    except FileNotFoundError:
        return False


def safe_get(grp: h5py.Group, path: str):
    """Return dataset[:] at `path` inside `grp` or None.

    Transparently handles legacy `robot/back_imu/...` / `robot/hip_imu/...`
    layout used by older H5 exports alongside the newer
    `robot/imu/back_imu/...` / `robot/imu/hip_imu/...` layout.
    """
    candidates = [path]
    if "robot/imu/back_imu/" in path:
        candidates.append(path.replace("robot/imu/back_imu/", "robot/back_imu/"))
    elif "robot/back_imu/" in path:
        candidates.append(path.replace("robot/back_imu/", "robot/imu/back_imu/"))
    if "robot/imu/hip_imu/" in path:
        candidates.append(path.replace("robot/imu/hip_imu/", "robot/hip_imu/"))
    elif "robot/hip_imu/" in path:
        candidates.append(path.replace("robot/hip_imu/", "robot/imu/hip_imu/"))
    for cand in candidates:
        node = grp
        ok = True
        for p in cand.split("/"):
            if p not in node:
                ok = False
                break
            node = node[p]
        if not ok or not isinstance(node, h5py.Dataset):
            continue
        try:
            return node[:]
        except Exception:
            return None
    return None


def list_subjects(h: h5py.File):
    return sorted([k for k in h.keys() if k.startswith("S")])


def list_conditions(grp: h5py.Group):
    return sorted([k for k in grp.keys() if k != "sub_info"])


def channel_stats(x: np.ndarray) -> dict:
    if x is None or x.size == 0:
        return dict(n=0, nan_frac=1.0, mean=np.nan, std=np.nan,
                    p5=np.nan, p95=np.nan, range=np.nan, var=np.nan)
    nan_frac = float(np.mean(~np.isfinite(x)))
    xx = x[np.isfinite(x)]
    if xx.size == 0:
        return dict(n=int(x.size), nan_frac=nan_frac, mean=np.nan, std=np.nan,
                    p5=np.nan, p95=np.nan, range=np.nan, var=np.nan)
    return dict(
        n=int(x.size), nan_frac=nan_frac,
        mean=float(np.mean(xx)), std=float(np.std(xx)),
        p5=float(np.percentile(xx, 5)), p95=float(np.percentile(xx, 95)),
        range=float(np.ptp(xx)), var=float(np.var(xx)),
    )


def detect_heel_strikes(grf_z: np.ndarray, fs: float) -> np.ndarray:
    if grf_z is None or grf_z.size == 0:
        return np.array([], dtype=int)
    above = np.abs(grf_z) > TH["grf_hs_threshold_n"]
    rising = np.where(np.diff(above.astype(np.int8)) == 1)[0] + 1
    if rising.size < 2:
        return rising
    min_n = int(TH["grf_min_cycle_s"] * fs)
    max_n = int(TH["grf_max_cycle_s"] * fs)
    keep = [rising[0]]
    for idx in rising[1:]:
        if min_n <= idx - keep[-1] <= max_n:
            keep.append(idx)
        elif idx - keep[-1] > max_n:
            keep.append(idx)
    return np.array(keep, dtype=int)


def gait_cycle_average(sig, hs, fs, n_pts=101):
    if sig is None or hs.size < 2:
        return None, None, 0
    cycles = []
    min_n = int(TH["grf_min_cycle_s"] * fs)
    max_n = int(TH["grf_max_cycle_s"] * fs)
    x_new = np.linspace(0, 1, n_pts)
    for a, b in zip(hs[:-1], hs[1:]):
        if not (min_n <= b - a <= max_n):
            continue
        seg = sig[a:b]
        if seg.size < 10 or not np.all(np.isfinite(seg)):
            continue
        x_old = np.linspace(0, 1, seg.size)
        cycles.append(np.interp(x_new, x_old, seg))
    if not cycles:
        return None, None, 0
    arr = np.stack(cycles, axis=0)
    return arr.mean(0), arr.std(0), arr.shape[0]


def stride_freq_grf(grf_z, fs=100.0):
    """Stride frequency from GRF rising-edge crossings."""
    if grf_z is None or grf_z.size < 100:
        return float("nan")
    above = np.abs(grf_z) > TH["grf_hs_threshold_n"]
    rising = np.where(np.diff(above.astype(int)) == 1)[0]
    if rising.size < 2:
        return float("nan")
    return fs / float(np.median(np.diff(rising)))


def dom_freq(x, fs=100.0, lo_skip=5):
    """Dominant frequency of x via FFT magnitude peak (>0 Hz)."""
    if x is None:
        return float("nan")
    xx = x[np.isfinite(x)]
    if xx.size < 4096:
        return float("nan")
    seg = xx[:4096] - xx[:4096].mean()
    spec = np.abs(np.fft.rfft(seg))
    spec[:lo_skip] = 0
    freqs = np.fft.rfftfreq(4096, d=1 / fs)
    return float(freqs[spec.argmax()])


# ============================================================== CHECKS


def check_subject_info(h, subj):
    si = h.get(f"{subj}/sub_info")
    out = dict(present=si is not None, flags=[])
    if si is None:
        out["flags"].append("missing sub_info group")
        return out

    def _val(name):
        v = si.get(name)
        if v is None:
            return None
        v = v[()]
        if isinstance(v, bytes):
            v = v.decode()
        try:
            return float(v)
        except Exception:
            return None

    age = _val("age"); height = _val("height"); weight = _val("weight"); sex = _val("sex")
    out.update(age=age, height=height, weight=weight, sex=sex)

    lo, hi = TH["sub_age_yr"]
    if age is None or not (lo <= age <= hi):
        out["flags"].append(f"age out of range: {age}")
    lo, hi = TH["sub_height_mm"]
    if height is None or not (lo <= height <= hi):
        out["flags"].append(f"height out of range: {height}")
    lo, hi = TH["sub_weight_kg"]
    if weight is None or not (lo <= weight <= hi):
        out["flags"].append(f"weight out of range: {weight}")
    if sex not in (0, 1):
        out["flags"].append(f"sex not in {{0,1}}: {sex}")
    for side in ("left", "right"):
        leg = si.get(f"{side}/leg length")
        if leg is None:
            out["flags"].append(f"missing {side}/leg length"); continue
        v = leg[()]
        if isinstance(v, bytes):
            v = v.decode()
        try:
            v = float(v)
            if not (300 <= v <= 1200):
                out["flags"].append(f"{side} leg length out of range: {v}")
        except Exception:
            out["flags"].append(f"{side} leg length unparsable: {leg[()]}")
    return out


def check_trial_groups(trial_grp):
    """Check that all required top-level groups are present in the trial."""
    present = set(trial_grp.keys())
    missing = sorted(REQUIRED_TRIAL_GROUPS - present)
    return missing


def check_quaternion_magnitude(trial_grp):
    qw = safe_get(trial_grp, "robot/imu/back_imu/quat_w")
    qx = safe_get(trial_grp, "robot/imu/back_imu/quat_x")
    qy = safe_get(trial_grp, "robot/imu/back_imu/quat_y")
    qz = safe_get(trial_grp, "robot/imu/back_imu/quat_z")
    if any(q is None for q in (qw, qx, qy, qz)):
        return None, None
    n = min(map(len, (qw, qx, qy, qz)))
    mag = np.sqrt(qw[:n] ** 2 + qx[:n] ** 2 + qy[:n] ** 2 + qz[:n] ** 2)
    mag = mag[np.isfinite(mag)]
    if mag.size == 0:
        return None, None
    m = float(np.mean(mag))
    flag = abs(m - 1.0) > TH["quat_mag_dev"]
    return m, flag


def check_gait_stage_order(trial_grp):
    """Return (n_invalid_transitions, n_total) — valid cycle is 1→2→3→4→1."""
    x = safe_get(trial_grp, "common/gait_stage")
    if x is None:
        return 0, 0
    x = x[np.isfinite(x)].astype(int)
    if x.size < 2:
        return 0, 0
    change = np.where(np.diff(x) != 0)[0]
    seq = x[np.concatenate([[0], change + 1])]
    n_trans = len(seq) - 1
    if n_trans < 1:
        return 0, 0
    diffs = np.mod(seq[1:] - seq[:-1], 4)
    n_invalid = int(np.sum(diffs != 1))
    return n_invalid, n_trans


def check_forceplate_drift(trial_grp, ch="forceplate/grf/left/z"):
    x = safe_get(trial_grp, ch)
    if x is None or x.size < 200:
        return None, None
    n = x.size
    w = max(50, n // 20)
    head = float(np.nanmean(np.abs(x[:w])))
    tail = float(np.nanmean(np.abs(x[-w:])))
    drift = tail - head
    flag = abs(drift) > TH["fp_baseline_drift_n"]
    return drift, flag


def check_imu_bias(trial_grp, ch="robot/imu/back_imu/gyro_z"):
    x = safe_get(trial_grp, ch)
    if x is None:
        return None, None
    m = float(np.nanmean(x))
    flag = abs(m) > TH["imu_bias_drift_dps"]
    return m, flag


def check_phase_mismatch(trial_grp):
    grf = safe_get(trial_grp, "forceplate/grf/right/z")
    thi = safe_get(trial_grp, "robot/right/thigh_angle")
    if grf is None or thi is None:
        return None, None, None
    f_grf = stride_freq_grf(grf)
    f_rob = dom_freq(thi)
    if not (np.isfinite(f_grf) and np.isfinite(f_rob)) or f_grf == 0:
        return f_grf, f_rob, None
    ratio = f_rob / f_grf
    flag = abs(ratio - 1.0) > TH["phase_mismatch_frac"]
    return f_grf, f_rob, flag


def check_quality(trial_grp, weight_kg, cond_kind):
    flags = []
    is_gait = cond_kind in GAIT_KINDS

    # ---- structural: time / sampling ----
    t = safe_get(trial_grp, "common/time")
    if t is None or t.size < 2:
        flags.append("missing or empty common/time")
        fs = float("nan")
    else:
        dt = np.diff(t.astype(float))
        med = float(np.median(dt))
        cv = float(np.std(dt) / med) if med > 0 else float("inf")
        fs = 1000.0 / med if med > 0 else float("nan")  # time in ms
        if cv > TH["dt_jitter_cv"]:
            flags.append(f"dt jitter cv={cv:.3f}")
        if dt.size and dt.max() > TH["dt_gap_factor"] * med:
            flags.append(f"time gap: max dt = {dt.max():.1f} ms (median {med:.1f})")
        if (dt <= 0).any():
            flags.append("non-monotonic time")

    # ---- loop_count step consistency ----
    lc = safe_get(trial_grp, "common/loop_count")
    if lc is None or lc.size < 2:
        flags.append("missing common/loop_count")
    else:
        dlc = np.diff(lc.astype(np.int64))
        med = int(np.median(dlc))
        if (np.abs(dlc - med) > TH["loop_step_tol"]).any():
            n_bad = int((np.abs(dlc - med) > TH["loop_step_tol"]).sum())
            flags.append(f"loop_count step jumps x{n_bad} (median step {med})")

    # ---- per-channel NaN / flat-line ----
    FLAT_OK = {"treadmill/pitch"}
    if not is_gait:
        FLAT_OK |= {"treadmill/left/speed_leftbelt", "treadmill/right/speed_rightbelt"}
    bad_channels = []
    for ch in DIST_CHANNELS:
        x = safe_get(trial_grp, ch)
        if x is None:
            bad_channels.append((ch, "missing"))
            continue
        nf = float(np.mean(~np.isfinite(x)))
        if nf > TH["nan_frac_fail"]:
            bad_channels.append((ch, f"nan_frac={nf:.3f}"))
        elif nf > TH["nan_frac_warn"]:
            flags.append(f"nan warn {ch} nf={nf:.3f}")
        xx = x[np.isfinite(x)]
        if ch not in FLAT_OK and xx.size and float(np.var(xx)) < TH["flat_var"]:
            bad_channels.append((ch, "flat"))

    # ---- cross-sensor: GRF total peak ~ body weight (gait conditions only) ----
    if weight_kg is not None and is_gait:
        gl = safe_get(trial_grp, "forceplate/grf/left/z")
        gr = safe_get(trial_grp, "forceplate/grf/right/z")
        if gl is not None and gr is not None:
            tot = np.abs(gl) + np.abs(gr)
            tot = tot[np.isfinite(tot)]
            if tot.size:
                peak = float(np.percentile(tot, 99))
                bw = weight_kg * 9.81
                ratio = peak / bw if bw > 0 else float("nan")
                if not (TH["grf_weight_match_tol"] <= ratio <= 2.5):
                    flags.append(f"GRF total peak / body-weight = {ratio:.2f} "
                                 f"(peak {peak:.0f} N, bw {bw:.0f} N)")

    # ---- cross-sensor: motor_angle vs hip_flexion (gait, non-stopandgo) ----
    if is_gait and cond_kind != "sng":
        for side in ("left", "right"):
            m = safe_get(trial_grp, f"robot/{side}/motor_angle")
            hipf = safe_get(trial_grp, f"mocap/kin_q/hip_flexion_{side[0]}")
            if m is None or hipf is None:
                continue
            n = min(m.size, hipf.size)
            if n < 100:
                continue
            a = m[:n]; b = hipf[:n]
            m_ok = np.isfinite(a) & np.isfinite(b)
            if m_ok.sum() < 100:
                continue
            c = float(np.corrcoef(a[m_ok] - a[m_ok].mean(),
                                   b[m_ok] - b[m_ok].mean())[0, 1])
            if c < TH["motor_hip_corr_min"]:
                flags.append(f"motor_angle({side}) vs hip_flexion corr={c:.2f}")

    # ---- extended checks ----
    qmag, qflag = check_quaternion_magnitude(trial_grp)
    if qflag:
        flags.append(f"quaternion |q|={qmag:.3f} (expected ≈1)")

    n_inv, n_total = check_gait_stage_order(trial_grp)
    if n_total >= 10 and n_inv / n_total > TH["gait_stage_invalid_frac"]:
        flags.append(f"gait_stage invalid transitions {n_inv}/{n_total} "
                     f"({n_inv/n_total*100:.1f}%)")

    fp_drift, fp_flag = check_forceplate_drift(trial_grp)
    if fp_flag:
        flags.append(f"forceplate baseline drift |Δ|={abs(fp_drift):.0f} N")

    bias, b_flag = check_imu_bias(trial_grp)
    if b_flag:
        flags.append(f"IMU gyro_z bias = {bias:+.2f} dps")

    if is_gait and cond_kind not in ("sine",):  # sine has time-varying stride freq
        f_grf, f_rob, p_flag = check_phase_mismatch(trial_grp)
        if p_flag:
            flags.append(f"robot/GRF stride-freq mismatch  "
                         f"f_GRF={f_grf:.2f}, f_robot={f_rob:.2f}, "
                         f"ratio={f_rob/f_grf:.2f}")

    return dict(fs_hz=fs, flags=flags, bad_channels=bad_channels,
                quaternion_mag=qmag, gait_stage_invalid=(n_inv, n_total),
                fp_baseline_drift=fp_drift, imu_gyro_z_bias=bias)


def check_condition_label(trial_grp, cond):
    spec = EXPECTED_CONDITIONS.get(cond)
    if spec is None:
        return [f"unknown condition '{cond}'"]
    flags = []
    sl = safe_get(trial_grp, "treadmill/left/speed_leftbelt")
    sr = safe_get(trial_grp, "treadmill/right/speed_rightbelt")
    pitch = safe_get(trial_grp, "treadmill/pitch")

    if spec["pitch"] is not None and pitch is not None:
        p = float(np.nanmean(pitch))
        if abs(p - spec["pitch"]) > TH["pitch_tol_deg"]:
            flags.append(f"pitch mean={p:.2f} deg, expected {spec['pitch']:+.1f}")

    if spec["kind"] in ("stand", "dyn"):
        if sl is not None and sr is not None:
            spd = 0.5 * (sl + sr)
            spd = spd[np.isfinite(spd)]
            if spd.size and float(np.mean(np.abs(spd))) > 0.3:
                flags.append(f"{spec['kind']}: treadmill moving "
                             f"(mean |speed|={np.mean(np.abs(spd)):.2f})")
        return flags

    if sl is None or sr is None:
        return flags
    spd = 0.5 * (sl + sr); spd = spd[np.isfinite(spd)]
    if spd.size == 0:
        flags.append("treadmill speed all-NaN"); return flags

    if spec["kind"] == "level" and spec["speed"] is not None:
        m = float(np.mean(spd))
        tol = spec["speed"] * TH["speed_tol_frac"]
        if abs(m - spec["speed"]) > tol:
            flags.append(f"mean speed={m:.2f} m/s, expected {spec['speed']:.2f} ±{tol:.2f}")
    elif spec["kind"] in ("incline", "decline"):
        if float(np.mean(spd)) < 0.3:
            flags.append(f"{spec['kind']}: low mean speed (<0.3 m/s)")
    elif spec["kind"] == "sine":
        if float(np.std(spd)) < 0.05:
            flags.append(f"sine: speed std={np.std(spd):.3f} (expected oscillation)")
    elif spec["kind"] == "sng":
        thr = 0.1
        above = spd > thr
        n_cross = int(np.sum(np.diff(above.astype(int)) == 1))
        if n_cross < 2:
            flags.append(f"stopandgo: only {n_cross} go-segments")
    return flags


def check_imu_mount(h, subj):
    """Estimate trunk-IMU mount tilt from gravity vector (mean accel over walking).
    Returns (source_trial_label, ax, ay, az, |g|, tilt_deg, lateral_lean_deg, fwdback_lean_deg, flag).
    """
    sources = [("stand", "lv0", "trial_01"),
               ("level_100mps", "lv0", "trial_01"),
               ("level_075mps", "lv0", "trial_01")]
    for cond, asl, tr in sources:
        try:
            tg = h[subj][cond][asl][tr]
        except KeyError:
            continue
        ax = safe_get(tg, "robot/imu/back_imu/accel_x")
        ay = safe_get(tg, "robot/imu/back_imu/accel_y")
        az = safe_get(tg, "robot/imu/back_imu/accel_z")
        if ax is None or ay is None or az is None:
            continue
        ax_m = float(np.nanmean(ax)); ay_m = float(np.nanmean(ay)); az_m = float(np.nanmean(az))
        mag = float(np.sqrt(ax_m ** 2 + ay_m ** 2 + az_m ** 2))
        # IMU x-axis is mounted along gravity; tilt = angle off vertical
        tilt = float(np.degrees(np.arctan2(np.sqrt(ay_m ** 2 + az_m ** 2), ax_m)))
        lr   = float(np.degrees(np.arctan2(ay_m, ax_m)))
        fb   = float(np.degrees(np.arctan2(az_m, ax_m)))
        flags = []
        if tilt > TH["imu_tilt_warn_deg"]:
            flags.append(f"trunk-IMU mount tilt {tilt:.1f}° off vertical")
        if abs(lr) > TH["imu_lateral_lean_deg"]:
            flags.append(f"trunk-IMU lateral lean {lr:+.1f}°")
        return dict(source=f"{cond}/{asl}/{tr}", ax=ax_m, ay=ay_m, az=az_m,
                    g_mag=mag, tilt_deg=tilt, lateral_lean=lr, fwdback_lean=fb,
                    flags=flags)
    return None


def check_protocol_compliance(coverage_for_subj):
    """Compare a subject's coverage to EXPECTED_CONDITIONS.
    Returns lists of missing trials, extra trials, and missing conditions."""
    issues = []
    for cond, spec in EXPECTED_CONDITIONS.items():
        if cond not in coverage_for_subj:
            issues.append(f"missing condition: {cond}")
            continue
        for asl in spec["asls"]:
            if asl not in coverage_for_subj[cond]:
                issues.append(f"{cond}: missing assist level {asl}")
                continue
            trials = coverage_for_subj[cond][asl]
            n = spec["trials_per_asl"]
            expected = [f"trial_{i:02d}" for i in range(1, n + 1)]
            missing = [t for t in expected if t not in trials]
            extra = [t for t in trials if t not in expected]
            if missing:
                issues.append(f"{cond}/{asl}: missing trials {missing}")
            if extra:
                issues.append(f"{cond}/{asl}: extra trials {extra}")
    return issues


def cross_subject_zscore(all_stats):
    """Robust z-score outliers per (cond, asl, channel) on subject-mean."""
    SETPOINT_CHANNELS = {
        "treadmill/pitch",
        "treadmill/left/speed_leftbelt",
        "treadmill/right/speed_rightbelt",
    }
    flags = defaultdict(list)
    for cond, by_asl in all_stats.items():
        for asl, by_ch in by_asl.items():
            for ch, by_subj in by_ch.items():
                if ch in SETPOINT_CHANNELS:
                    continue
                vals = {s: float(np.mean([t["mean"] for t in trials
                                          if np.isfinite(t["mean"])]))
                        for s, trials in by_subj.items()
                        if any(np.isfinite(t["mean"]) for t in trials)}
                if len(vals) < 3:
                    continue
                arr = np.fromiter(vals.values(), dtype=float)
                med = float(np.median(arr))
                mad = float(np.median(np.abs(arr - med)))
                # require MAD ≥ max(1e-6, 1e-3·|median|) — otherwise ~constant
                floor = max(1e-6, abs(med) * 1e-3)
                if mad < floor:
                    continue
                scale = 1.4826 * mad
                for s, v in vals.items():
                    z = abs(v - med) / scale
                    if z > TH["mad_zscore"]:
                        flags[s].append(dict(cond=cond, asl=asl, ch=ch,
                                             value=v, cohort_med=med, z=float(z)))
    return flags


def subject_wide_offsets(h, subjects, channels):
    """For each (subject, channel): mean over ALL the subject's trials.
    Identifies subject-wide hip/knee baseline offsets (S004/S007/S017 type).
    Returns dict ch -> sorted list of (subject, mean) and the cohort median.
    """
    out = {}
    for ch in channels:
        rows = []
        for s in subjects:
            per_trial = []
            for cond in h[s]:
                if cond == "sub_info":
                    continue
                for asl in h[s][cond]:
                    for tr in h[s][cond][asl]:
                        x = safe_get(h[s][cond][asl][tr], f"mocap/kin_q/{ch}")
                        if x is None:
                            continue
                        per_trial.append(float(np.nanmean(x)))
            if per_trial:
                rows.append((s, float(np.mean(per_trial))))
        if not rows:
            continue
        rows.sort(key=lambda r: r[1])
        arr = np.array([r[1] for r in rows])
        med = float(np.median(arr))
        out[ch] = dict(rows=rows, cohort_med=med,
                       outliers=[s for s, v in rows
                                 if abs(v - med) > TH["abs_dev_outlier_deg"]])
    return out


# ============================================================== PLOTS


def plot_coverage(file_label, coverage, out_png):
    cells = []
    for cond, spec in EXPECTED_CONDITIONS.items():
        for asl in spec["asls"]:
            cells.append(f"{cond}|{asl}")
    cells.sort()
    subjects = sorted(coverage.keys())
    Z = np.full((len(subjects), len(cells)), -1.0)
    for i, s in enumerate(subjects):
        for j, c in enumerate(cells):
            cond, asl = c.split("|")
            try:
                Z[i, j] = float(len(coverage[s][cond][asl]))
            except KeyError:
                Z[i, j] = -1
    fig_w = max(10, 0.45 * len(cells))
    fig_h = max(4, 0.4 * len(subjects) + 1)
    fig, ax = plt.subplots(figsize=(fig_w, fig_h))
    cmap = plt.get_cmap("YlGn").copy(); cmap.set_under("crimson")
    im = ax.imshow(Z, aspect="auto", cmap=cmap, vmin=0, vmax=max(2, Z.max()))
    ax.set_yticks(range(len(subjects))); ax.set_yticklabels(subjects)
    ax.set_xticks(range(len(cells)))
    ax.set_xticklabels([c.replace("|", "/") for c in cells], rotation=60, ha="right")
    for i in range(Z.shape[0]):
        for j in range(Z.shape[1]):
            v = Z[i, j]
            ax.text(j, i, "—" if v < 0 else str(int(v)),
                    ha="center", va="center",
                    color="white" if v < 0 else "black",
                    fontsize=8, weight="bold" if v < 0 else "normal")
    fig.colorbar(im, ax=ax, label="# trials  (red — = missing)")
    ax.set_title(f"{file_label}: coverage")
    fig.tight_layout()
    fig.savefig(out_png, dpi=110); plt.close(fig)


def plot_lr_summary(per_subject_lr, out_png, file_label):
    if not per_subject_lr:
        return
    pairs_present = sorted({lbl for d in per_subject_lr.values() for lbl in d.keys()})
    subjects = sorted(per_subject_lr.keys())
    fig, ax = plt.subplots(figsize=(max(6, 0.6 * len(subjects)), 4))
    width = 0.8 / max(1, len(pairs_present))
    for i, lbl in enumerate(pairs_present):
        vals = [per_subject_lr[s].get(lbl, np.nan) for s in subjects]
        x = np.arange(len(subjects)) + (i - len(pairs_present) / 2 + 0.5) * width
        ax.bar(x, vals, width=width, label=lbl)
    ax.axhline(1.0, color="k", lw=0.5)
    ax.axhline(TH["lr_ratio_lo"], color="r", lw=0.5, ls="--")
    ax.axhline(TH["lr_ratio_hi"], color="r", lw=0.5, ls="--")
    ax.set_xticks(range(len(subjects))); ax.set_xticklabels(subjects, rotation=45)
    ax.set_ylabel("L/R RMS ratio (level conds)")
    ax.set_title(f"{file_label}: L/R symmetry")
    ax.legend(fontsize=7, ncol=2)
    fig.tight_layout(); fig.savefig(out_png, dpi=110); plt.close(fig)


def plot_imu_mount(per_subject_mount, out_png, file_label):
    """Bar chart of trunk-IMU mount tilt per subject."""
    if not per_subject_mount:
        return
    subjects = sorted(per_subject_mount.keys())
    tilts = [per_subject_mount[s]["tilt_deg"] for s in subjects]
    lr    = [per_subject_mount[s]["lateral_lean"] for s in subjects]
    fb    = [per_subject_mount[s]["fwdback_lean"] for s in subjects]
    mags  = [per_subject_mount[s]["g_mag"] for s in subjects]
    fig, axes = plt.subplots(2, 2, figsize=(14, 7),
                             gridspec_kw=dict(hspace=0.5, wspace=0.25))
    ax = axes[0, 0]
    colors = ["red" if t > TH["imu_tilt_warn_deg"] else "tab:gray" for t in tilts]
    ax.bar(subjects, tilts, color=colors)
    ax.axhspan(0, TH["imu_tilt_warn_deg"], color="green", alpha=0.10,
               label=f"≤{TH['imu_tilt_warn_deg']}° plausible")
    ax.set_ylabel("tilt off vertical (deg)"); ax.legend(fontsize=8)
    ax.set_title("(A) trunk-IMU total mount tilt")
    ax.set_xticklabels(subjects, rotation=45, fontsize=7)
    ax.tick_params(labelsize=8); ax.grid(alpha=0.3, axis="y")

    ax = axes[0, 1]
    colors = ["red" if abs(v) > TH["imu_lateral_lean_deg"] else "tab:gray" for v in lr]
    ax.bar(subjects, lr, color=colors); ax.axhline(0, color="k", lw=0.5)
    ax.axhspan(-TH["imu_lateral_lean_deg"], TH["imu_lateral_lean_deg"],
               color="green", alpha=0.10)
    ax.set_ylabel("lateral lean (deg)\n(+ = right)")
    ax.set_title("(B) lateral lean component")
    ax.set_xticklabels(subjects, rotation=45, fontsize=7)
    ax.tick_params(labelsize=8); ax.grid(alpha=0.3, axis="y")

    ax = axes[1, 0]
    ax.bar(subjects, fb, color="tab:gray"); ax.axhline(0, color="k", lw=0.5)
    ax.set_ylabel("forward/back lean (deg)\n(+ = forward)")
    ax.set_title("(C) forward-back lean component")
    ax.set_xticklabels(subjects, rotation=45, fontsize=7)
    ax.tick_params(labelsize=8); ax.grid(alpha=0.3, axis="y")

    ax = axes[1, 1]
    ax.bar(subjects, mags, color="tab:gray")
    ax.axhline(9.81, color="green", ls="--", lw=0.8, label="ideal |g|=9.81")
    ax.set_ylabel("|g| (m/s²)")
    ax.set_title("(D) gravity magnitude (sanity — IMU calibration)")
    ax.set_xticklabels(subjects, rotation=45, fontsize=7)
    ax.tick_params(labelsize=8); ax.grid(alpha=0.3, axis="y")
    ax.legend(fontsize=8)

    fig.suptitle(f"{file_label}: trunk-IMU mount orientation per subject",
                 fontsize=12)
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    fig.savefig(out_png, dpi=110); plt.close(fig)


def plot_anomalies_summary(report, swo, out_png):
    """Single-PNG summary of all flagged anomalies for this file."""
    fr = report
    file_stem = Path(fr["file"]).stem

    # Aggregate anomalies
    fail_trials = []; warn_trials = []
    for s, sr in fr.get("subjects", {}).items():
        for cond, by_asl in sr.get("conditions", {}).items():
            if not isinstance(by_asl, dict):
                continue
            for asl, by_tr in by_asl.items():
                if not isinstance(by_tr, dict):
                    continue
                for tr, info in by_tr.items():
                    if info.get("verdict") == "FAIL":
                        fail_trials.append((s, cond, asl, tr, info))
                    elif info.get("verdict") == "WARN":
                        warn_trials.append((s, cond, asl, tr, info))

    fig = plt.figure(figsize=(20, 16))
    gs = fig.add_gridspec(3, 1, hspace=0.4,
                          height_ratios=[1.5, 1.5, 1.5],
                          top=0.96, bottom=0.04, left=0.04, right=0.98)

    # (1) FAIL/WARN list
    ax = fig.add_subplot(gs[0]); ax.axis("off")
    txt = [f"FAIL trials ({len(fail_trials)}):"]
    for s, c, a, tr, info in fail_trials[:30]:
        bad = info.get("bad_channels", [])[:3]
        txt.append(f"   {s}/{c}/{a}/{tr}: " + ", ".join(f"{ch}({why})" for ch, why in bad))
    txt.append("")
    txt.append(f"WARN trials ({len(warn_trials)}):")
    for s, c, a, tr, info in warn_trials[:40]:
        flags = (info.get("condition_flags", []) + info.get("quality_flags", []))[:3]
        txt.append(f"   {s}/{c}/{a}/{tr}: " + " | ".join(flags))
    ax.text(0.01, 0.98, "\n".join(txt), fontsize=8, family="monospace",
            transform=ax.transAxes, va="top")
    ax.set_title("Per-trial verdicts (top 30 FAIL + 40 WARN)", loc="left")

    # (2) subject-wide hip/knee offsets
    ax = fig.add_subplot(gs[1])
    ch = "hip_flexion_l"
    info = swo.get(ch)
    if info:
        labels = [r[0] for r in info["rows"]]
        vals = [r[1] for r in info["rows"]]
        med = info["cohort_med"]; outs = set(info["outliers"])
        colors = ["red" if s in outs else "tab:gray" for s in labels]
        ax.bar(labels, vals, color=colors)
        ax.axhline(med, ls="--", lw=1, color="0.2",
                   label=f"cohort median = {med:+.1f}°")
        ax.set_xticklabels(labels, rotation=70, fontsize=8)
        for tick, s in zip(ax.get_xticklabels(), labels):
            if s in outs:
                tick.set_color("red"); tick.set_fontweight("bold")
        ax.set_ylabel("subject-mean hip_flexion_l (deg)")
        ax.set_title(f"Subject-wide hip baseline offsets — |Δ from cohort median| > "
                     f"{TH['abs_dev_outlier_deg']}° outliers in red", loc="left")
        ax.tick_params(labelsize=8); ax.grid(alpha=0.3, axis="y"); ax.legend(fontsize=8)

    # (3) Coverage gaps + protocol compliance + sub_info
    ax = fig.add_subplot(gs[2]); ax.axis("off")
    gaps_txt = ["Subject coverage / protocol compliance issues:"]
    for s in sorted(fr["subjects"].keys()):
        sr = fr["subjects"][s]
        gaps = sr.get("coverage_gaps", [])
        proto = sr.get("protocol_issues", [])
        sif = sr.get("sub_info", {}).get("flags", [])
        lr = sr.get("lr_symmetry_flags", [])
        mount = sr.get("imu_mount_flags", [])
        all_items = gaps + proto + sif + lr + mount
        if all_items:
            wrapped = textwrap.fill("; ".join(all_items), width=160,
                                    subsequent_indent=" " * 8)
            gaps_txt.append(f"   {s}: {wrapped}")
    if len(gaps_txt) == 1:
        gaps_txt.append("   (none)")
    ax.text(0.01, 0.98, "\n".join(gaps_txt), fontsize=8, family="monospace",
            transform=ax.transAxes, va="top")
    ax.set_title("Subject-level issues", loc="left")

    fig.suptitle(f"{file_stem}: anomalies summary", fontsize=13)
    fig.savefig(out_png, dpi=110); plt.close(fig)


def plot_timeseries(trial_grp, out_png, title):
    t = safe_get(trial_grp, "common/time")
    if t is None:
        return
    t = t.astype(float) / 1000.0
    n = len(TS_PANELS); rows = 5; cols = (n + rows - 1) // rows
    fig, axes = plt.subplots(rows, cols, figsize=(3 * cols, 1.7 * rows), sharex=True)
    axes = axes.ravel()
    for ax, (ch, label) in zip(axes, TS_PANELS):
        x = safe_get(trial_grp, ch)
        if x is None:
            ax.set_title(f"{label} [missing]", fontsize=8, color="red"); ax.axis("off"); continue
        m = min(len(t), len(x))
        ax.plot(t[:m], x[:m], lw=0.5)
        ax.set_title(label, fontsize=8); ax.tick_params(labelsize=7); ax.grid(alpha=0.3)
    for ax in axes[len(TS_PANELS):]:
        ax.axis("off")
    fig.suptitle(title, fontsize=10); fig.supxlabel("time (s)", fontsize=9)
    fig.tight_layout(rect=[0, 0.02, 1, 0.97])
    fig.savefig(out_png, dpi=110); plt.close(fig)


def plot_gait_cycle(trial_grp, out_png, title):
    fs = 100.0
    t = safe_get(trial_grp, "common/time")
    if t is not None and t.size > 1:
        dt_ms = float(np.median(np.diff(t.astype(float))))
        if dt_ms > 0:
            fs = 1000.0 / dt_ms
    grfL = safe_get(trial_grp, "forceplate/grf/left/z")
    grfR = safe_get(trial_grp, "forceplate/grf/right/z")
    hsL = detect_heel_strikes(grfL, fs)
    hsR = detect_heel_strikes(grfR, fs)
    if hsL.size < 3 and hsR.size < 3:
        return
    n = len(GC_PANELS); rows = 4; cols = (n + rows - 1) // rows
    fig, axes = plt.subplots(rows, cols, figsize=(3 * cols, 1.8 * rows))
    axes = axes.ravel()
    x_pct = np.linspace(0, 100, 101)
    for ax, (ch, label) in zip(axes, GC_PANELS):
        sig = safe_get(trial_grp, ch)
        if sig is None:
            ax.set_title(f"{label} [missing]", fontsize=8, color="red"); ax.axis("off"); continue
        hs = hsR if (ch.endswith("_r") or "/right/" in ch) else hsL
        mu, sd, nc = gait_cycle_average(sig, hs, fs)
        if mu is None:
            ax.set_title(f"{label} [n=0]", fontsize=8, color="orange"); ax.axis("off"); continue
        ax.plot(x_pct, mu, "C0", lw=1.0)
        ax.fill_between(x_pct, mu - sd, mu + sd, color="C0", alpha=0.2)
        ax.set_title(f"{label} (n={nc})", fontsize=8)
        ax.tick_params(labelsize=7); ax.grid(alpha=0.3)
    for ax in axes[len(GC_PANELS):]:
        ax.axis("off")
    fig.suptitle(title, fontsize=10); fig.supxlabel("gait cycle (%)", fontsize=9)
    fig.tight_layout(rect=[0, 0.02, 1, 0.97])
    fig.savefig(out_png, dpi=110); plt.close(fig)


# ============================================================== PIPELINE


def process_file(path, all_stats):
    label = path.stem
    out = dict(file=str(path.name), subjects={}, file_flags=[])

    try:
        h = h5py.File(path, "r")
    except Exception as e:
        out["file_flags"].append(f"open failed: {e}")
        return out

    subjects = list_subjects(h)
    coverage = {s: {} for s in subjects}
    per_subject_lr = {}
    per_subject_mount = {}

    file_plot_dir = PLOT_DIR / label
    file_plot_dir.mkdir(parents=True, exist_ok=True)

    for subj in subjects:
        sinfo = check_subject_info(h, subj)
        weight = sinfo.get("weight")
        subj_report = dict(sub_info=sinfo, conditions={})

        for cond in list_conditions(h[subj]):
            if cond not in EXPECTED_CONDITIONS:
                subj_report["conditions"][cond] = dict(unexpected=True)
                continue
            cond_report = {}
            cond_grp = h[subj][cond]
            for asl in cond_grp.keys():
                asl_grp = cond_grp[asl]
                trials = sorted(asl_grp.keys())
                coverage.setdefault(subj, {}).setdefault(cond, {})[asl] = trials
                asl_report = {}
                for tr in trials:
                    tg = asl_grp[tr]

                    # trial-level group missing
                    missing_groups = check_trial_groups(tg)
                    lab_flags = check_condition_label(tg, cond)
                    qres = check_quality(tg, weight, EXPECTED_CONDITIONS[cond]["kind"])

                    bad_channels = list(qres["bad_channels"])
                    if missing_groups:
                        for g in missing_groups:
                            bad_channels.append((f"{g}/*", "missing trial group"))

                    verdict = "PASS"
                    if bad_channels:
                        verdict = "FAIL"
                    if lab_flags or qres["flags"]:
                        if verdict == "PASS":
                            verdict = "WARN"

                    asl_report[tr] = dict(
                        verdict=verdict,
                        fs_hz=qres["fs_hz"],
                        condition_flags=lab_flags,
                        quality_flags=qres["flags"],
                        bad_channels=bad_channels,
                        missing_groups=missing_groups,
                        quaternion_mag=qres.get("quaternion_mag"),
                        gait_stage_invalid=qres.get("gait_stage_invalid"),
                        fp_baseline_drift=qres.get("fp_baseline_drift"),
                        imu_gyro_z_bias=qres.get("imu_gyro_z_bias"),
                    )

                    # per-channel stats accumulator (for cross-subject z-score)
                    for ch in DIST_CHANNELS:
                        x = safe_get(tg, ch)
                        if x is None:
                            continue
                        s = channel_stats(x)
                        all_stats.setdefault(cond, {}).setdefault(asl, {}) \
                                 .setdefault(ch, {}).setdefault(subj, []).append(s)

                cond_report[asl] = asl_report
            subj_report["conditions"][cond] = cond_report

            # coverage gaps within condition
            expected_asls = set(EXPECTED_CONDITIONS[cond]["asls"])
            present = set(cond_report.keys())
            missing = expected_asls - present
            if missing:
                subj_report.setdefault("coverage_gaps", []).append(
                    f"{cond}: missing assist levels {sorted(missing)}")

        # protocol compliance per subject
        proto = check_protocol_compliance(coverage.get(subj, {}))
        if proto:
            subj_report["protocol_issues"] = proto

        # missing entire conditions
        missing_conds = set(EXPECTED_CONDITIONS.keys()) - set(subj_report["conditions"].keys())
        if missing_conds:
            subj_report.setdefault("coverage_gaps", []).append(
                f"missing conditions: {sorted(missing_conds)}")

        # L/R symmetry on level conds
        lr = {}
        for cond in ("level_075mps", "level_100mps", "level_125mps"):
            for asl in ("lv0", "lv4", "lv7"):
                try:
                    tg = h[subj][cond][asl]["trial_01"]
                except (KeyError, TypeError):
                    continue
                for ch_l, ch_r, lbl in LR_PAIRS:
                    a = safe_get(tg, ch_l); b = safe_get(tg, ch_r)
                    if a is None or b is None:
                        continue
                    a = a[np.isfinite(a)]; b = b[np.isfinite(b)]
                    if a.size < 100 or b.size < 100:
                        continue
                    rms_a = float(np.sqrt(np.mean(a ** 2)))
                    rms_b = float(np.sqrt(np.mean(b ** 2)))
                    if rms_b < 1e-9:
                        continue
                    lr.setdefault(lbl, []).append(rms_a / rms_b)
        per_subject_lr[subj] = {k: float(np.mean(v)) for k, v in lr.items() if v}
        symm = [f"{lbl} L/R RMS ratio={r:.2f}" for lbl, r in per_subject_lr[subj].items()
                if not (TH["lr_ratio_lo"] <= r <= TH["lr_ratio_hi"])]
        if symm:
            subj_report.setdefault("lr_symmetry_flags", []).extend(symm)

        # IMU mount tilt
        mount = check_imu_mount(h, subj)
        if mount is not None:
            per_subject_mount[subj] = mount
            if mount["flags"]:
                subj_report.setdefault("imu_mount_flags", []).extend(mount["flags"])

        out["subjects"][subj] = subj_report

    # subject-wide hip/knee baseline offsets
    swo = subject_wide_offsets(h, subjects, [
        "hip_flexion_l", "hip_flexion_r",
        "knee_angle_l",  "knee_angle_r",
    ])
    out["subject_wide_offsets"] = {
        ch: dict(rows=info["rows"], cohort_med=info["cohort_med"],
                 outliers=info["outliers"]) for ch, info in swo.items()
    }
    for ch, info in swo.items():
        for s in info["outliers"]:
            out["subjects"].setdefault(s, {}).setdefault("subject_wide_offsets", []).append(
                f"{ch}: subject mean = "
                f"{dict(info['rows'])[s]:+.1f}° vs cohort {info['cohort_med']:+.1f}°"
            )

    # ---- plots ----
    plot_coverage(label, coverage, file_plot_dir / "coverage.png")
    plot_lr_summary(per_subject_lr, file_plot_dir / "lr_symmetry.png", label)
    plot_imu_mount(per_subject_mount, file_plot_dir / "imu_mount.png", label)

    if PLOT_SUBJECT in subjects:
        subj_dir = file_plot_dir / PLOT_SUBJECT
        subj_dir.mkdir(parents=True, exist_ok=True)
        for cond in list_conditions(h[PLOT_SUBJECT]):
            if cond not in EXPECTED_CONDITIONS:
                continue
            for asl in h[PLOT_SUBJECT][cond].keys():
                trials = sorted(h[PLOT_SUBJECT][cond][asl].keys())
                if not trials:
                    continue
                tg = h[PLOT_SUBJECT][cond][asl][trials[0]]
                plot_timeseries(tg, subj_dir / f"timeseries_{cond}_{asl}.png",
                                f"{PLOT_SUBJECT} {cond}/{asl}/{trials[0]}")
                if EXPECTED_CONDITIONS[cond]["kind"] in {"level", "incline", "decline"}:
                    plot_gait_cycle(tg, subj_dir / f"gaitcycle_{cond}_{asl}.png",
                                    f"{PLOT_SUBJECT} {cond}/{asl}/{trials[0]}  (gait-cycle averaged)")

    out["coverage"] = coverage
    out["per_subject_lr"] = per_subject_lr
    out["per_subject_mount"] = per_subject_mount

    # one-glance anomalies summary
    plot_anomalies_summary(out, swo, file_plot_dir / "anomalies_summary.png")

    h.close()
    return out


def maybe_rename_pending_file():
    """If `combined_data_<NN>.h5` (no leading 'S') has finished uploading,
    rename to `combined_data_S<NN>.h5` (file holds S001..S<NN>)."""
    for src in DATABANK.glob("combined_data_*.h5"):
        # skip already-renamed files
        if src.stem.startswith("combined_data_S"):
            continue
        # numeric suffix?
        suffix = src.stem.replace("combined_data_", "")
        if not suffix.isdigit():
            continue
        dst = src.with_name(f"combined_data_S{int(suffix):03d}.h5")
        if file_in_use(src):
            print(f"[info] {src.name} is being written; skipping for now.")
            continue
        try:
            with h5py.File(src, "r") as h:
                _ = list(h.keys())
        except Exception as e:
            print(f"[info] {src.name} not yet a valid H5 ({e}); skipping rename.")
            continue
        print(f"[info] renaming {src.name} -> {dst.name}")
        shutil.move(str(src), str(dst))


# ============================================================== REPORT


def write_markdown(report, cross_z, out_md):
    lines = []
    lines.append("# Databank Integrity Report\n")
    lines.append(f"Generated by `data_integrity_check.py`. Plot subject = `{PLOT_SUBJECT}`.\n")

    for fr in report:
        lines.append(f"\n## {fr['file']}\n")
        if fr.get("file_flags"):
            for f in fr["file_flags"]:
                lines.append(f"- ⚠ {f}")
            continue

        lines.append("### Coverage\n")
        lines.append(f"![coverage](plots/{Path(fr['file']).stem}/coverage.png)\n")

        lines.append("### Anomalies summary\n")
        lines.append(f"![anomalies](plots/{Path(fr['file']).stem}/anomalies_summary.png)\n")

        lines.append("### IMU mount per subject\n")
        lines.append(f"![imu_mount](plots/{Path(fr['file']).stem}/imu_mount.png)\n")

        lines.append("### L/R symmetry (level conditions)\n")
        lines.append(f"![lr](plots/{Path(fr['file']).stem}/lr_symmetry.png)\n")

        # Per-subject summary
        lines.append("### Per-subject summary\n")
        lines.append("| subject | age | height | weight | sub_info | coverage gaps | "
                     "protocol issues | LR ratios | IMU mount | subject-wide hip/knee |")
        lines.append("|---|---|---|---|---|---|---|---|---|---|")
        for s in sorted(fr["subjects"].keys()):
            sr = fr["subjects"][s]
            si = sr.get("sub_info", {})
            lines.append("| {s} | {a} | {h} | {w} | {sif} | {cg} | {pi} | {lr} | {mi} | {sw} |".format(
                s=s,
                a=si.get("age"), h=si.get("height"), w=si.get("weight"),
                sif="; ".join(si.get("flags", [])) or "",
                cg="<br>".join(sr.get("coverage_gaps", [])) or "",
                pi="<br>".join(sr.get("protocol_issues", [])) or "",
                lr="<br>".join(sr.get("lr_symmetry_flags", [])) or "",
                mi="<br>".join(sr.get("imu_mount_flags", [])) or "",
                sw="<br>".join(sr.get("subject_wide_offsets", [])) or "",
            ))
        lines.append("")

        # FAIL/WARN trial table
        lines.append("### Trial verdicts (FAIL / WARN only)\n")
        lines.append("| subject | cond | asl | trial | verdict | flags |")
        lines.append("|---|---|---|---|---|---|")
        for s in sorted(fr["subjects"].keys()):
            sr = fr["subjects"][s]
            for cond, by_asl in sr.get("conditions", {}).items():
                if not isinstance(by_asl, dict):
                    continue
                for asl, by_tr in by_asl.items():
                    if not isinstance(by_tr, dict):
                        continue
                    for tr, info in by_tr.items():
                        if info["verdict"] == "PASS":
                            continue
                        all_flags = (info.get("condition_flags", [])
                                     + info.get("quality_flags", [])
                                     + [f"{ch}: {why}" for ch, why in info.get("bad_channels", [])])
                        lines.append(f"| {s} | {cond} | {asl} | {tr} | {info['verdict']} "
                                     f"| {'<br>'.join(all_flags)} |")
        lines.append("")

        # plot_subject section
        if PLOT_SUBJECT in fr["subjects"]:
            lines.append(f"### Plots for {PLOT_SUBJECT}\n")
            sd = PLOT_DIR / Path(fr["file"]).stem / PLOT_SUBJECT
            for png in sorted(sd.glob("*.png")):
                rel = png.relative_to(OUT_DIR)
                lines.append(f"- ![{png.stem}]({rel})")
            lines.append("")

    # Cross-subject distribution outliers
    lines.append("\n## Cross-subject distribution outliers (robust z-score on channel means)\n")
    if not cross_z:
        lines.append("(no outliers above threshold)\n")
    else:
        lines.append(f"Threshold: |z| > {TH['mad_zscore']} (MAD-based robust z-score over subject-mean).\n")
        lines.append("| subject | cond | asl | channel | value | cohort median | z |")
        lines.append("|---|---|---|---|---|---|---|")
        for subj in sorted(cross_z.keys()):
            for f in sorted(cross_z[subj], key=lambda x: -x["z"])[:30]:
                lines.append(f"| {subj} | {f['cond']} | {f['asl']} | {f['ch']} "
                             f"| {f['value']:.3f} | {f['cohort_med']:.3f} | {f['z']:.2f} |")
        lines.append("")

    out_md.write_text("\n".join(lines))


def main():
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    PLOT_DIR.mkdir(parents=True, exist_ok=True)

    maybe_rename_pending_file()

    h5_files = sorted(p for p in DATABANK.glob("combined_data_*.h5"))
    # If both `<X>.h5` and `<X>_clean.h5` exist, use the cleaned version.
    cleaned_stems = {p.stem.removesuffix("_clean") for p in h5_files
                     if p.stem.endswith("_clean")}
    h5_files = [p for p in h5_files if p.stem not in cleaned_stems]
    # Also skip any non-renamed numeric files (combined_data_<NN>.h5).
    h5_files = [p for p in h5_files
                if not p.stem.replace("combined_data_", "").isdigit()]
    if not h5_files:
        print("[warn] no completed H5 files found.")
        return

    print(f"[info] processing {len(h5_files)} file(s): {[p.name for p in h5_files]}")

    all_stats = {}
    reports = []
    for p in h5_files:
        if file_in_use(p):
            print(f"[info] {p.name} is open by another process; skipping.")
            reports.append(dict(file=p.name,
                                file_flags=["file in use by another process"]))
            continue
        print(f"[info] >>> {p.name}")
        reports.append(process_file(p, all_stats))

    cross_z = cross_subject_zscore(all_stats)

    (OUT_DIR / "summary.json").write_text(
        json.dumps(dict(reports=reports, cross_subject_outliers=cross_z),
                   indent=2, default=str))

    write_markdown(reports, cross_z, OUT_DIR / "report.md")

    n_fail = n_warn = n_pass = 0
    for fr in reports:
        for s, sr in fr.get("subjects", {}).items():
            for cond, by_asl in sr.get("conditions", {}).items():
                if not isinstance(by_asl, dict):
                    continue
                for asl, by_tr in by_asl.items():
                    if not isinstance(by_tr, dict):
                        continue
                    for tr, info in by_tr.items():
                        v = info.get("verdict")
                        if v == "FAIL": n_fail += 1
                        elif v == "WARN": n_warn += 1
                        elif v == "PASS": n_pass += 1
    print(f"\n[done] PASS={n_pass}  WARN={n_warn}  FAIL={n_fail}")
    print(f"[done] report : {OUT_DIR/'report.md'}")
    print(f"[done] json   : {OUT_DIR/'summary.json'}")
    print(f"[done] plots  : {PLOT_DIR}")


if __name__ == "__main__":
    main()
