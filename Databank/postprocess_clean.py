"""Databank H5 post-processing: single-sample spike removal + interpolation.

For every numeric dataset in every trial, detect outlier samples that are
physically impossible (or far from the local distribution) and replace them by
linear interpolation from neighbours.

Two detection layers, applied in order:

  1. Absolute physical bound per channel category (catches the catastrophic
     1e30-style overflow spikes regardless of context).
  2. Hampel filter (rolling median / MAD) for milder spikes.

Set-point channels (treadmill/pitch, treadmill/*/speed_*, common/mode,
common/assist_level) are skipped — they are intentionally constant.

Run:
  python Databank/postprocess_clean.py            # default in→out paths
  python Databank/postprocess_clean.py --inplace  # overwrite (NOT recommended)
"""

from __future__ import annotations

import argparse
import json
import shutil
from pathlib import Path

import h5py
import numpy as np

DATABANK = Path(__file__).resolve().parent

# Channels intentionally constant — never touched.
SETPOINT_CHANNELS = (
    "treadmill/pitch",
    "treadmill/left/speed_leftbelt",
    "treadmill/right/speed_rightbelt",
    "common/mode",
    "common/assist_level",
)

# (path-substring → absolute |·| upper bound).  First match wins.
# Bounds are intentionally generous — they exist only to catch the catastrophic
# overflow spikes (1e10~1e34).  Mid-range outliers are caught by the Hampel
# filter instead, so legitimate physical extremes are preserved.
ABS_BOUNDS = [
    ("robot/left/torque",                  1.0e4),     # Nm (real ≲ 50)
    ("robot/right/torque",                 1.0e4),
    ("robot/left/motor_angle",             1.0e4),     # deg (real ≲ 720)
    ("robot/right/motor_angle",            1.0e4),
    ("robot/left/hip_angle",               1.0e4),
    ("robot/right/hip_angle",              1.0e4),
    ("robot/left/thigh_angle",             1.0e4),
    ("robot/right/thigh_angle",            1.0e4),
    ("robot/imu/back_imu/accel",           1.0e4),     # m/s^2 (real ≲ 30)
    ("robot/imu/back_imu/gyro",            1.0e5),     # dps (real ≲ 2000)
    ("robot/imu/back_imu/quat",            10.0),      # unit quaternion
    ("robot/imu/hip_imu",                  1.0e5),
    ("robot/back_imu/accel",               1.0e4),     # legacy layout
    ("robot/back_imu/gyro",                1.0e5),
    ("robot/back_imu/quat",                10.0),
    ("robot/hip_imu",                      1.0e5),
    ("mocap/kin_q",                        1.0e4),     # deg
    ("mocap/joint_moment",                 1.0e7),     # may be in N·mm
    ("mocap/marker",                       1.0e5),     # mm
    ("forceplate/grf",                     1.0e5),     # N
    ("forceplate/cop",                     1.0e5),     # mm
    ("forceplate/moment",                  1.0e7),     # likely N·mm (real ≲ 5e5)
    ("treadmill/left/distance",            1.0e9),
    ("treadmill/right/distance",           1.0e9),
    ("robot/insole",                       1.0e9),     # FSR ADC counts
]

HAMPEL_WINDOW = 7        # samples on each side
HAMPEL_K = 8.0           # threshold in MAD-σ units (conservative)


def is_setpoint(path: str) -> bool:
    return path in SETPOINT_CHANNELS


def abs_bound_for(path: str) -> float | None:
    for sub, lim in ABS_BOUNDS:
        if sub in path:
            return float(lim)
    return None


def interp_replace(x: np.ndarray, mask_bad: np.ndarray) -> np.ndarray:
    """Replace samples where mask_bad is True by linear interp from neighbours."""
    out = x.astype(np.float64, copy=True)
    n = out.size
    if n == 0 or not mask_bad.any():
        return out
    good = ~mask_bad & np.isfinite(x)
    if good.sum() < 2:
        out[mask_bad] = np.nan
        return out
    idx = np.arange(n)
    out[mask_bad] = np.interp(idx[mask_bad], idx[good], x[good])
    return out


def detect_abs_outliers(x: np.ndarray, bound: float) -> np.ndarray:
    """True at samples whose |x| exceeds the bound or is non-finite."""
    return ~np.isfinite(x) | (np.abs(x) > bound)


def detect_hampel(x: np.ndarray, win: int = HAMPEL_WINDOW, k: float = HAMPEL_K) -> np.ndarray:
    """Hampel filter — flag samples > k*MAD from local rolling median.

    Pure-numpy implementation; for n~20k it runs in well under a second.
    """
    n = x.size
    if n < 2 * win + 1:
        return np.zeros(n, dtype=bool)
    pad = np.pad(x, win, mode="edge")
    # build sliding window matrix (n × (2*win+1))
    idx = np.arange(2 * win + 1)[None, :] + np.arange(n)[:, None]
    win_mat = pad[idx]
    med = np.median(win_mat, axis=1)
    mad = np.median(np.abs(win_mat - med[:, None]), axis=1)
    sigma = 1.4826 * mad
    sigma[sigma == 0] = 1e-12
    return np.abs(x - med) > k * sigma


def process_dataset(ds: h5py.Dataset, path: str, log: list) -> None:
    """In-place clean a 1-D float dataset.  Log entries describe each replacement."""
    if ds.dtype.kind not in "fiu":
        return  # not numeric
    if is_setpoint(path):
        return
    try:
        x = ds[:]
    except Exception:
        return
    if x.ndim != 1 or x.size < 4:
        return

    bound = abs_bound_for(path)
    abs_bad = detect_abs_outliers(x.astype(float), bound) if bound is not None \
              else ~np.isfinite(x.astype(float))
    n_abs = int(abs_bad.sum())

    # Hampel is restricted to a ±HAMPEL_WIN_AROUND_SPIKE window around each
    # absolute outlier — Hampel false-positives on fast-varying signals like
    # hip_angle in stopandgo otherwise. If no abs spike, skip Hampel entirely.
    HAMPEL_WIN_AROUND_SPIKE = 200
    if n_abs == 0:
        hampel_bad = np.zeros_like(abs_bad)
    else:
        x_for_hamp = x.astype(float).copy()
        x_for_hamp[abs_bad] = np.median(x_for_hamp[~abs_bad]) if (~abs_bad).any() else 0.0
        hampel_full = detect_hampel(x_for_hamp)
        # Restrict to neighbourhood of any abs-spike sample.
        spike_idx = np.where(abs_bad)[0]
        nbr = np.zeros(x.size, dtype=bool)
        for s_i in spike_idx:
            lo = max(0, s_i - HAMPEL_WIN_AROUND_SPIKE)
            hi = min(x.size, s_i + HAMPEL_WIN_AROUND_SPIKE + 1)
            nbr[lo:hi] = True
        hampel_bad = hampel_full & nbr
    hampel_only = hampel_bad & ~abs_bad
    n_hamp = int(hampel_only.sum())

    bad = abs_bad | hampel_bad
    if not bad.any():
        return

    cleaned = interp_replace(x.astype(float), bad)
    # Cast back if integer dtype
    if ds.dtype.kind in "iu":
        cleaned = np.rint(cleaned).astype(ds.dtype)
    ds[...] = cleaned

    # log a few representative samples
    bad_idx = np.where(bad)[0]
    log.append(dict(
        path=path,
        n=int(x.size),
        n_abs=n_abs,
        n_hampel=n_hamp,
        first_bad_indices=bad_idx[:8].tolist(),
        original_extreme=float(np.max(np.abs(x.astype(float)))) if n_abs else None,
        bound=bound,
    ))


def walk_and_clean(out: h5py.File, prefix: str, log: list) -> None:
    """Recursively descend into out (open in 'r+'), cleaning every dataset."""
    obj = out[prefix] if prefix else out
    if isinstance(obj, h5py.Dataset):
        process_dataset(obj, prefix.lstrip("/"), log)
        return
    for k in obj.keys():
        sub = f"{prefix}/{k}" if prefix else k
        walk_and_clean(out, sub, log)


def clean_file(src: Path, dst: Path) -> dict:
    print(f"[copy] {src.name} -> {dst.name} ({src.stat().st_size/1e9:.2f} GB)")
    shutil.copy2(src, dst)
    log: list = []
    with h5py.File(dst, "r+") as out:
        walk_and_clean(out, "", log)
    summary = dict(file=dst.name, n_datasets_modified=len(log), entries=log)
    return summary


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--inplace", action="store_true",
                    help="modify the input files directly (NOT recommended)")
    ap.add_argument("--files", nargs="*", default=None,
                    help="explicit input file paths (default: all combined_data_*.h5)")
    args = ap.parse_args()

    if args.files:
        srcs = [Path(p) for p in args.files]
    else:
        srcs = sorted(p for p in DATABANK.glob("combined_data_*.h5") if "_clean" not in p.name)

    summaries = []
    for src in srcs:
        if args.inplace:
            dst = src
            print(f"[warn] inplace: writing back to {src.name}")
            log: list = []
            with h5py.File(src, "r+") as out:
                walk_and_clean(out, "", log)
            summaries.append(dict(file=src.name, n_datasets_modified=len(log), entries=log))
        else:
            dst = src.with_name(src.stem + "_clean.h5")
            summaries.append(clean_file(src, dst))

    out_log = DATABANK / "integrity_report" / "postprocess_log.json"
    out_log.parent.mkdir(parents=True, exist_ok=True)
    out_log.write_text(json.dumps(summaries, indent=2, default=str))

    # console summary: only datasets that actually had corrections
    print(f"\n[done] log: {out_log}")
    for s in summaries:
        print(f"\n{s['file']}: {s['n_datasets_modified']} datasets modified")
        # list top ones
        worst = sorted(s["entries"], key=lambda e: -(e["n_abs"] + e["n_hampel"]))[:25]
        for e in worst:
            tag = []
            if e["n_abs"]:    tag.append(f"abs={e['n_abs']}")
            if e["n_hampel"]: tag.append(f"hampel={e['n_hampel']}")
            extra = f"  orig max|·|={e['original_extreme']:.2e}" if e["original_extreme"] else ""
            print(f"  {e['path']}  ({', '.join(tag)}){extra}")


if __name__ == "__main__":
    main()
