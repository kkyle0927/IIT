"""
Input Feature Integrity Check
- Basic sanity: NaN, Inf, extreme values, per-subject stats
- Cross-subject consistency: same condition, compare distributions
- Biomechanical validity: ROM, GRF range, derivative consistency
- Time-series quality: jumps, drift, noise
Saves results to SpeedEstimation/input_feat_check/ and SpeedEstimation_robot/input_feat_check/
"""

import sys, os
import numpy as np
import h5py
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from pathlib import Path
from collections import defaultdict
import warnings
warnings.filterwarnings('ignore')

# ---------- paths ----------
H5_PATH = Path("/home/chanyoungko/IIT/Databank/combined_data_S008.h5")
OUT_BASE = Path("/home/chanyoungko/IIT/SpeedEstimation/input_feat_check")
OUT_ROBOT = Path("/home/chanyoungko/IIT/SpeedEstimation_robot/input_feat_check")

SUBJECTS = [f"S{i:03d}" for i in range(1, 9)]
CONDITIONS = ["level_075mps", "level_100mps", "level_125mps",
              "accel_sine", "decline_5deg", "incline_10deg", "stopandgo"]
FS = 100  # Hz

# ---------- feature definitions ----------
# Baseline (SpeedEstimation) raw H5 features
BASELINE_RAW = {
    "mocap/kin_q": ["hip_flexion_l", "hip_flexion_r", "knee_angle_l", "knee_angle_r",
                    "ankle_angle_l", "ankle_angle_r"],
    "robot/back_imu": ["accel_x", "accel_y", "accel_z", "gyro_x", "gyro_y", "gyro_z",
                        "quat_w", "quat_x", "quat_y", "quat_z"],
    "robot/left": ["hip_angle", "thigh_angle", "torque"],
    "robot/right": ["hip_angle", "thigh_angle", "torque"],
    "forceplate/grf/left": ["x", "y", "z"],
    "forceplate/grf/right": ["x", "y", "z"],
    "treadmill/left": ["speed_leftbelt"],
}

# Biomechanical limits (degrees for angles, m/s^2 for acc, N for GRF, etc.)
BIO_LIMITS = {
    # mocap joint angles (degrees)
    "hip_flexion_l":  (-40, 80),   "hip_flexion_r":  (-40, 80),
    "knee_angle_l":   (-10, 90),   "knee_angle_r":   (-10, 90),
    "ankle_angle_l":  (-50, 40),   "ankle_angle_r":  (-50, 40),
    # robot exo angles (degrees)
    "hip_angle":      (-40, 80),
    "thigh_angle":    (-40, 80),
    # IMU accel (m/s^2) - ~1-2g typical during walking
    "accel_x": (-30, 30), "accel_y": (-30, 30), "accel_z": (-30, 30),
    # IMU gyro (rad/s)
    "gyro_x": (-10, 10), "gyro_y": (-10, 10), "gyro_z": (-10, 10),
    # quaternion
    "quat_w": (-1.1, 1.1), "quat_x": (-1.1, 1.1), "quat_y": (-1.1, 1.1), "quat_z": (-1.1, 1.1),
    # torque (Nm)
    "torque": (-5, 5),
    # GRF (N) - z is typically negative (downward)
    "grf_z": (-2000, 100),
    "grf_xy": (-500, 500),
    # speed (m/s)
    "speed_leftbelt": (-0.5, 2.5),
}

def get_bio_limit(gpath, feat_name):
    """Get biomechanical limits for a feature."""
    if feat_name in BIO_LIMITS:
        return BIO_LIMITS[feat_name]
    if "grf" in gpath and feat_name == "z":
        return BIO_LIMITS["grf_z"]
    if "grf" in gpath and feat_name in ("x", "y"):
        return BIO_LIMITS["grf_xy"]
    return None


def load_all_trials(f):
    """Load all trial data paths from H5."""
    trials = []
    for sub in SUBJECTS:
        if sub not in f:
            continue
        for cond in CONDITIONS:
            if cond not in f[sub]:
                continue
            for lv in f[sub][cond].keys():
                lv_grp = f[sub][cond][lv]
                if not isinstance(lv_grp, h5py.Group):
                    continue
                for trial_name in lv_grp.keys():
                    trial_grp = lv_grp[trial_name]
                    if isinstance(trial_grp, h5py.Group):
                        trials.append({
                            "sub": sub, "cond": cond, "lv": lv,
                            "trial": trial_name, "grp": trial_grp,
                            "path": f"{sub}/{cond}/{lv}/{trial_name}"
                        })
    return trials


def check_basic_sanity(trials, out_dir):
    """Check NaN, Inf, extreme values for all raw features."""
    print("\n[1/4] Basic Sanity Check...")
    results = []
    all_stats = defaultdict(lambda: defaultdict(list))  # feat -> sub -> [stats]

    for t in trials:
        grp = t["grp"]
        sub = t["sub"]
        for gpath, feats in BASELINE_RAW.items():
            for feat in feats:
                try:
                    data_grp = grp
                    for p in gpath.split("/"):
                        data_grp = data_grp[p]
                    if feat not in data_grp:
                        continue
                    d = data_grp[feat][:]
                except (KeyError, ValueError):
                    continue

                n = len(d)
                n_nan = np.sum(np.isnan(d))
                n_inf = np.sum(np.isinf(d))
                dmin, dmax = np.nanmin(d), np.nanmax(d)
                dmean, dstd = np.nanmean(d), np.nanstd(d)

                label = f"{gpath}/{feat}"
                all_stats[label][sub].append({
                    "trial": t["path"], "n": n, "nan": n_nan, "inf": n_inf,
                    "min": dmin, "max": dmax, "mean": dmean, "std": dstd
                })

                # Flag issues
                flags = []
                if n_nan > 0: flags.append(f"NaN={n_nan}")
                if n_inf > 0: flags.append(f"Inf={n_inf}")
                bio_lim = get_bio_limit(gpath, feat)
                if bio_lim:
                    if dmin < bio_lim[0]: flags.append(f"min={dmin:.2f}<{bio_lim[0]}")
                    if dmax > bio_lim[1]: flags.append(f"max={dmax:.2f}>{bio_lim[1]}")

                if flags:
                    results.append({
                        "trial": t["path"], "feature": label,
                        "flags": ", ".join(flags),
                        "min": dmin, "max": dmax, "mean": dmean, "std": dstd
                    })

    # Write report
    with open(out_dir / "01_basic_sanity.txt", "w") as f:
        f.write("=" * 100 + "\n")
        f.write("BASIC SANITY CHECK — NaN, Inf, Out-of-range\n")
        f.write("=" * 100 + "\n\n")

        if not results:
            f.write("No issues found.\n")
        else:
            f.write(f"Found {len(results)} flagged entries:\n\n")
            for r in results:
                f.write(f"  {r['trial']:50s} | {r['feature']:35s} | {r['flags']}\n")

        # Summary stats table
        f.write("\n\n" + "=" * 100 + "\n")
        f.write("FEATURE SUMMARY STATISTICS (aggregated across all trials)\n")
        f.write("=" * 100 + "\n\n")
        f.write(f"{'Feature':40s} | {'N_trials':>8s} | {'Min':>10s} | {'Max':>10s} | {'Mean':>10s} | {'Std':>10s} | {'NaN_tot':>8s} | {'Inf_tot':>8s}\n")
        f.write("-" * 130 + "\n")
        for label in sorted(all_stats.keys()):
            sub_data = all_stats[label]
            all_entries = [e for subs in sub_data.values() for e in subs]
            n_trials = len(all_entries)
            total_nan = sum(e["nan"] for e in all_entries)
            total_inf = sum(e["inf"] for e in all_entries)
            gmin = min(e["min"] for e in all_entries)
            gmax = max(e["max"] for e in all_entries)
            gmean = np.mean([e["mean"] for e in all_entries])
            gstd = np.mean([e["std"] for e in all_entries])
            f.write(f"{label:40s} | {n_trials:8d} | {gmin:10.4f} | {gmax:10.4f} | {gmean:10.4f} | {gstd:10.4f} | {total_nan:8d} | {total_inf:8d}\n")

    print(f"  -> {len(results)} issues found. Saved to 01_basic_sanity.txt")
    return all_stats


def check_cross_subject(trials, all_stats, out_dir):
    """Compare feature distributions across subjects for same condition."""
    print("\n[2/4] Cross-Subject Consistency Check...")

    # Collect per-subject per-condition stats
    cond_sub_stats = defaultdict(lambda: defaultdict(lambda: defaultdict(list)))

    for t in trials:
        grp = t["grp"]
        sub, cond = t["sub"], t["cond"]
        for gpath, feats in BASELINE_RAW.items():
            for feat in feats:
                try:
                    data_grp = grp
                    for p in gpath.split("/"):
                        data_grp = data_grp[p]
                    if feat not in data_grp:
                        continue
                    d = data_grp[feat][:]
                except (KeyError, ValueError):
                    continue
                label = f"{gpath}/{feat}"
                cond_sub_stats[cond][label][sub].append(np.nanmean(d))

    # Detect outlier subjects (mean deviates > 3 std from group)
    outliers = []
    with open(out_dir / "02_cross_subject.txt", "w") as f:
        f.write("=" * 100 + "\n")
        f.write("CROSS-SUBJECT CONSISTENCY — Outlier Detection (|z| > 3)\n")
        f.write("=" * 100 + "\n\n")

        for cond in sorted(cond_sub_stats.keys()):
            for label in sorted(cond_sub_stats[cond].keys()):
                sub_means = {}
                for sub, vals in cond_sub_stats[cond][label].items():
                    sub_means[sub] = np.mean(vals)
                if len(sub_means) < 3:
                    continue
                arr = np.array(list(sub_means.values()))
                mu, sigma = np.mean(arr), np.std(arr)
                if sigma < 1e-10:
                    continue
                for sub, val in sub_means.items():
                    z = (val - mu) / sigma
                    if abs(z) > 3:
                        outliers.append({"cond": cond, "feat": label, "sub": sub,
                                         "val": val, "z": z, "group_mean": mu, "group_std": sigma})
                        f.write(f"  {cond:15s} | {label:35s} | {sub} | val={val:.4f} | z={z:.2f} (mean={mu:.4f}, std={sigma:.4f})\n")

        if not outliers:
            f.write("No outlier subjects detected (all within 3σ).\n")
        else:
            f.write(f"\nTotal: {len(outliers)} outlier entries\n")

    # Box plots per condition for key features
    key_feats = ["mocap/kin_q/hip_flexion_l", "robot/left/hip_angle",
                 "robot/back_imu/accel_x", "forceplate/grf/left/z",
                 "treadmill/left/speed_leftbelt", "robot/left/torque"]

    fig, axes = plt.subplots(2, 3, figsize=(18, 10))
    axes = axes.flatten()
    for idx, feat_label in enumerate(key_feats):
        ax = axes[idx]
        # Collect per-subject means for level_100mps
        cond = "level_100mps"
        if cond in cond_sub_stats and feat_label in cond_sub_stats[cond]:
            sub_data = cond_sub_stats[cond][feat_label]
            subs_sorted = sorted(sub_data.keys())
            data_list = [sub_data[s] for s in subs_sorted]
            ax.boxplot(data_list, labels=subs_sorted)
            ax.set_title(feat_label.split("/")[-1] + f" ({cond})", fontsize=9)
            ax.tick_params(axis='x', rotation=45)
        else:
            ax.set_title(f"{feat_label} (no data)")
    plt.suptitle("Cross-Subject Consistency (level_100mps)", fontsize=12)
    plt.tight_layout()
    plt.savefig(out_dir / "02_cross_subject_boxplot.png", dpi=150)
    plt.close()

    print(f"  -> {len(outliers)} outlier entries. Saved to 02_cross_subject.txt")


def check_biomechanical(trials, out_dir):
    """Check biomechanical validity and derivative consistency."""
    print("\n[3/4] Biomechanical Validity Check...")

    issues = []
    deriv_consistency = []

    for t in trials:
        grp = t["grp"]
        sub = t["sub"]

        # --- Joint angle ROM check ---
        for side, side_name in [("left", "l"), ("right", "r")]:
            # Mocap
            for joint in ["hip_flexion", "knee_angle", "ankle_angle"]:
                key = f"{joint}_{side_name}"
                try:
                    d = grp[f"mocap/kin_q/{key}"][:]
                    lim = BIO_LIMITS.get(key)
                    if lim and (np.nanmin(d) < lim[0] or np.nanmax(d) > lim[1]):
                        issues.append(f"{t['path']} | mocap/{key}: [{np.nanmin(d):.1f}, {np.nanmax(d):.1f}] outside {lim}")
                except KeyError:
                    pass

            # Robot exo
            for joint in ["hip_angle", "thigh_angle"]:
                try:
                    d = grp[f"robot/{side}/{joint}"][:]
                    lim = BIO_LIMITS.get(joint)
                    if lim and (np.nanmin(d) < lim[0] or np.nanmax(d) > lim[1]):
                        issues.append(f"{t['path']} | robot/{side}/{joint}: [{np.nanmin(d):.1f}, {np.nanmax(d):.1f}] outside {lim}")
                except KeyError:
                    pass

        # --- GRF check: z should be mostly negative (downward) or zero ---
        for side in ["left", "right"]:
            try:
                gz = grp[f"forceplate/grf/{side}/z"][:]
                pct_positive = np.mean(gz > 50) * 100
                if pct_positive > 5:
                    issues.append(f"{t['path']} | grf/{side}/z: {pct_positive:.1f}% > 50N (expect mostly ≤0)")
            except KeyError:
                pass

        # --- Derivative consistency: angle vs numerical derivative ---
        for side in ["left", "right"]:
            try:
                angle = grp[f"robot/{side}/hip_angle"][:]
                num_dot = np.gradient(angle) * FS  # numerical derivative
                # Check if derivative is reasonable (not too large)
                max_dot = np.max(np.abs(num_dot))
                if max_dot > 2000:  # deg/s, very high
                    issues.append(f"{t['path']} | robot/{side}/hip_angle numerical dot max={max_dot:.0f} deg/s (suspicious)")
                deriv_consistency.append({
                    "trial": t["path"], "side": side,
                    "angle_std": np.std(angle), "dot_max": max_dot
                })
            except KeyError:
                pass

        # --- Speed vs condition sanity ---
        try:
            speed = grp["treadmill/left/speed_leftbelt"][:]
            cond = t["cond"]
            if "level_075" in cond:
                expected = 0.75
            elif "level_100" in cond:
                expected = 1.0
            elif "level_125" in cond:
                expected = 1.25
            else:
                expected = None

            if expected is not None:
                # Trim first/last 5 sec (ramp up/down)
                trim = int(5 * FS)
                if len(speed) > 2 * trim:
                    mid_speed = speed[trim:-trim]
                    actual_mean = np.mean(mid_speed)
                    if abs(actual_mean - expected) > 0.15:
                        issues.append(f"{t['path']} | speed mean={actual_mean:.3f}, expected≈{expected} (diff={abs(actual_mean-expected):.3f})")
        except KeyError:
            pass

        # --- Quaternion norm check (should be ~1) ---
        try:
            qw = grp["robot/back_imu/quat_w"][:]
            qx = grp["robot/back_imu/quat_x"][:]
            qy = grp["robot/back_imu/quat_y"][:]
            qz = grp["robot/back_imu/quat_z"][:]
            qnorm = np.sqrt(qw**2 + qx**2 + qy**2 + qz**2)
            norm_err = np.max(np.abs(qnorm - 1.0))
            if norm_err > 0.05:
                issues.append(f"{t['path']} | quat norm max error={norm_err:.4f} (expect ≈1.0)")
        except KeyError:
            pass

    with open(out_dir / "03_biomechanical.txt", "w") as f:
        f.write("=" * 100 + "\n")
        f.write("BIOMECHANICAL VALIDITY CHECK\n")
        f.write("=" * 100 + "\n\n")
        if not issues:
            f.write("No biomechanical issues detected.\n")
        else:
            f.write(f"Found {len(issues)} issues:\n\n")
            for iss in issues:
                f.write(f"  {iss}\n")

    print(f"  -> {len(issues)} issues. Saved to 03_biomechanical.txt")


def check_timeseries(trials, out_dir):
    """Time-series quality: jumps, drift, noise. Plot representative trials."""
    print("\n[4/4] Time-Series Quality Check...")

    jump_issues = []

    # Jump thresholds per feature type
    jump_thresh = {
        "hip_angle": 15, "thigh_angle": 15,  # degrees per sample (100Hz)
        "hip_flexion": 15, "knee_angle": 15, "ankle_angle": 15,
        "accel": 20, "gyro": 3,
        "torque": 0.5,
        "grf_z": 300, "grf_xy": 200,
        "speed": 0.3,
    }

    def get_jump_thresh(gpath, feat):
        if "grf" in gpath and feat == "z":
            return jump_thresh["grf_z"]
        if "grf" in gpath:
            return jump_thresh["grf_xy"]
        if "speed" in feat:
            return jump_thresh["speed"]
        for key in jump_thresh:
            if key in feat:
                return jump_thresh[key]
        return None

    for t in trials:
        grp = t["grp"]
        for gpath, feats in BASELINE_RAW.items():
            for feat in feats:
                try:
                    data_grp = grp
                    for p in gpath.split("/"):
                        data_grp = data_grp[p]
                    d = data_grp[feat][:]
                except (KeyError, ValueError):
                    continue

                diffs = np.abs(np.diff(d))
                thresh = get_jump_thresh(gpath, feat)
                if thresh is None:
                    continue

                n_jumps = np.sum(diffs > thresh)
                if n_jumps > 0:
                    max_jump = np.max(diffs)
                    jump_issues.append({
                        "trial": t["path"],
                        "feat": f"{gpath}/{feat}",
                        "n_jumps": n_jumps,
                        "max_jump": max_jump,
                        "thresh": thresh
                    })

    with open(out_dir / "04_timeseries_jumps.txt", "w") as f:
        f.write("=" * 100 + "\n")
        f.write("TIME-SERIES QUALITY — Jump/Discontinuity Detection\n")
        f.write("=" * 100 + "\n\n")
        if not jump_issues:
            f.write("No significant jumps detected.\n")
        else:
            f.write(f"Found {len(jump_issues)} features with jumps:\n\n")
            f.write(f"{'Trial':50s} | {'Feature':35s} | {'N_jumps':>8s} | {'Max_jump':>10s} | {'Threshold':>10s}\n")
            f.write("-" * 120 + "\n")
            for j in sorted(jump_issues, key=lambda x: -x["max_jump"]):
                f.write(f"{j['trial']:50s} | {j['feat']:35s} | {j['n_jumps']:8d} | {j['max_jump']:10.4f} | {j['thresh']:10.4f}\n")

    print(f"  -> {len(jump_issues)} features with jumps. Saved to 04_timeseries_jumps.txt")

    # ---------- Plot representative time-series ----------
    # Pick S001/level_100mps/lv0/trial_01 as representative
    print("  -> Generating time-series plots...")

    plot_trials = []
    for t in trials:
        if t["sub"] == "S001" and t["cond"] == "level_100mps" and t["lv"] == "lv0":
            plot_trials.append(t)
            break

    if not plot_trials:
        print("  [WARN] Could not find S001/level_100mps/lv0 for plotting")
        return

    t = plot_trials[0]
    grp = t["grp"]
    n_samples = grp["robot/left/hip_angle"].shape[0]
    time = np.arange(n_samples) / FS

    # Plot 1: All baseline features
    feat_groups = [
        ("Mocap Joint Angles (deg)", "mocap/kin_q",
         ["hip_flexion_l", "hip_flexion_r", "knee_angle_l", "knee_angle_r", "ankle_angle_l", "ankle_angle_r"]),
        ("Robot Exo Angles (deg)", None,
         [("robot/left/hip_angle", "L hip"), ("robot/left/thigh_angle", "L thigh"),
          ("robot/right/hip_angle", "R hip"), ("robot/right/thigh_angle", "R thigh")]),
        ("Back IMU Accel (m/s²)", "robot/back_imu",
         ["accel_x", "accel_y", "accel_z"]),
        ("Back IMU Gyro (rad/s)", "robot/back_imu",
         ["gyro_x", "gyro_y", "gyro_z"]),
        ("GRF Z (N)", None,
         [("forceplate/grf/left/z", "left"), ("forceplate/grf/right/z", "right")]),
        ("Torque (Nm) & Speed (m/s)", None,
         [("robot/left/torque", "L torque"), ("robot/right/torque", "R torque"),
          ("treadmill/left/speed_leftbelt", "speed")]),
    ]

    fig, axes = plt.subplots(len(feat_groups), 1, figsize=(16, 3 * len(feat_groups)), sharex=True)
    # Show only first 10 seconds for clarity
    show_sec = 10
    show_n = min(show_sec * FS, n_samples)
    t_show = time[:show_n]

    for idx, (title, base_path, feat_list) in enumerate(feat_groups):
        ax = axes[idx]
        for feat_item in feat_list:
            if isinstance(feat_item, tuple):
                full_path, label = feat_item
                try:
                    dg = grp
                    for p in full_path.split("/"):
                        dg = dg[p]
                    d = dg[:][:show_n]
                    ax.plot(t_show, d, label=label, alpha=0.8, linewidth=0.8)
                except KeyError:
                    pass
            else:
                try:
                    dg = grp
                    for p in base_path.split("/"):
                        dg = dg[p]
                    d = dg[feat_item][:][:show_n]
                    ax.plot(t_show, d, label=feat_item, alpha=0.8, linewidth=0.8)
                except KeyError:
                    pass
        ax.set_title(title, fontsize=10)
        ax.legend(fontsize=7, ncol=3, loc='upper right')
        ax.grid(True, alpha=0.3)

    axes[-1].set_xlabel("Time (s)")
    plt.suptitle(f"Time-Series Overview — {t['path']} (first {show_sec}s)", fontsize=12)
    plt.tight_layout()
    plt.savefig(out_dir / "04_timeseries_overview.png", dpi=150)
    plt.close()

    # Plot 2: Angle vs numerical derivative consistency
    fig, axes = plt.subplots(2, 2, figsize=(14, 8))
    for col, side in enumerate(["left", "right"]):
        try:
            angle = grp[f"robot/{side}/hip_angle"][:][:show_n]
            num_dot = np.gradient(angle) * FS
            axes[0, col].plot(t_show, angle, label="hip_angle", linewidth=0.8)
            axes[0, col].set_title(f"Robot {side} hip_angle (deg)")
            axes[0, col].legend(fontsize=8)
            axes[0, col].grid(True, alpha=0.3)
            axes[1, col].plot(t_show, num_dot, label="numerical d/dt", linewidth=0.8, color='orange')
            axes[1, col].set_title(f"Robot {side} hip_angle numerical derivative (deg/s)")
            axes[1, col].legend(fontsize=8)
            axes[1, col].grid(True, alpha=0.3)
        except KeyError:
            pass
    axes[-1, 0].set_xlabel("Time (s)")
    axes[-1, 1].set_xlabel("Time (s)")
    plt.suptitle(f"Derivative Consistency — {t['path']}", fontsize=12)
    plt.tight_layout()
    plt.savefig(out_dir / "04_derivative_consistency.png", dpi=150)
    plt.close()

    # Plot 3: All subjects same condition overlay (hip_angle left, speed)
    fig, axes = plt.subplots(2, 1, figsize=(16, 8))
    for t2 in trials:
        if t2["cond"] != "level_100mps" or t2["lv"] != "lv0":
            continue
        grp2 = t2["grp"]
        try:
            d = grp2["robot/left/hip_angle"][:]
            n2 = min(show_n, len(d))
            axes[0].plot(np.arange(n2)/FS, d[:n2], label=t2["sub"], alpha=0.7, linewidth=0.8)
        except KeyError:
            pass
        try:
            d = grp2["treadmill/left/speed_leftbelt"][:]
            n2 = min(show_n, len(d))
            axes[1].plot(np.arange(n2)/FS, d[:n2], label=t2["sub"], alpha=0.7, linewidth=0.8)
        except KeyError:
            pass

    axes[0].set_title("Robot Left Hip Angle — All Subjects (level_100mps/lv0)")
    axes[0].legend(fontsize=8, ncol=4)
    axes[0].grid(True, alpha=0.3)
    axes[0].set_ylabel("Angle (deg)")
    axes[1].set_title("Treadmill Speed — All Subjects (level_100mps/lv0)")
    axes[1].legend(fontsize=8, ncol=4)
    axes[1].grid(True, alpha=0.3)
    axes[1].set_ylabel("Speed (m/s)")
    axes[1].set_xlabel("Time (s)")
    plt.tight_layout()
    plt.savefig(out_dir / "04_cross_subject_timeseries.png", dpi=150)
    plt.close()

    # Plot 4: Distribution histograms for key features
    fig, axes = plt.subplots(3, 3, figsize=(15, 12))
    hist_feats = [
        ("robot/left", "hip_angle"), ("robot/left", "thigh_angle"), ("robot/left", "torque"),
        ("robot/back_imu", "accel_x"), ("robot/back_imu", "gyro_x"),
        ("mocap/kin_q", "hip_flexion_l"), ("mocap/kin_q", "knee_angle_l"),
        ("forceplate/grf/left", "z"), ("treadmill/left", "speed_leftbelt"),
    ]
    for idx, (gpath, feat) in enumerate(hist_feats):
        ax = axes[idx // 3, idx % 3]
        for sub in SUBJECTS:
            vals = []
            for t2 in trials:
                if t2["sub"] != sub:
                    continue
                try:
                    dg = t2["grp"]
                    for p in gpath.split("/"):
                        dg = dg[p]
                    vals.append(dg[feat][:])
                except KeyError:
                    pass
            if vals:
                all_vals = np.concatenate(vals)
                ax.hist(all_vals, bins=100, alpha=0.4, label=sub, density=True)
        ax.set_title(f"{gpath}/{feat}", fontsize=9)
        ax.legend(fontsize=6, ncol=2)
    plt.suptitle("Feature Distributions by Subject (all trials)", fontsize=12)
    plt.tight_layout()
    plt.savefig(out_dir / "04_feature_distributions.png", dpi=150)
    plt.close()

    print("  -> Saved time-series plots.")


def main():
    print("=" * 60)
    print("INPUT FEATURE INTEGRITY CHECK")
    print("=" * 60)
    print(f"H5: {H5_PATH}")
    print(f"Output: {OUT_BASE}")

    f = h5py.File(H5_PATH, "r")
    trials = load_all_trials(f)
    print(f"Loaded {len(trials)} trials across {len(SUBJECTS)} subjects, {len(CONDITIONS)} conditions")

    # Run all checks (save to SpeedEstimation)
    all_stats = check_basic_sanity(trials, OUT_BASE)
    check_cross_subject(trials, all_stats, OUT_BASE)
    check_biomechanical(trials, OUT_BASE)
    check_timeseries(trials, OUT_BASE)

    # Copy text reports to SpeedEstimation_robot too
    import shutil
    for fname in OUT_BASE.glob("*"):
        shutil.copy2(fname, OUT_ROBOT / fname.name)
    print(f"\n-> Results also copied to {OUT_ROBOT}")

    f.close()
    print("\nDone.")


if __name__ == "__main__":
    main()
