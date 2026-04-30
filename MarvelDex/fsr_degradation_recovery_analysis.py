#!/usr/bin/env python3
"""
Single-FSR degradation recovery analysis driven by GRF peak scaling.

What this script does
1. Loads `combined_data_S008.h5`
2. Extracts same-foot gait cycles from GRF stance onsets
3. Builds a constant scale-up factor from 10-cycle mean peak ratios
   for one FSR channel at a time
4. Compares raw vs corrected FSR/weight waveforms against a GRF-scaled
   reference waveform using avg+-std envelopes
5. Saves composite plots and CSV summaries for:
   - within-subject: first 10 cycles vs middle 10 cycles
   - between-subject: previous subject first 10 cycles vs current subject first 10 cycles

Default analysis target:
    condition = level_100mps
    level     = lv0
    trial     = trial_01
    sensor    = fsr2
"""

from __future__ import annotations

import argparse
import csv
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import h5py
import numpy as np

# Matplotlib may not have a writable default config path in this environment.
os.environ.setdefault("MPLCONFIGDIR", "/tmp/fsr_degradation_recovery_mpl")
import matplotlib.pyplot as plt


DEFAULT_H5 = Path("/home/chanyoungko/IIT/Databank/combined_data_S008.h5")
DEFAULT_SUBJECTS = [f"S{i:03d}" for i in range(1, 9)]
DEFAULT_WITHIN_PLOT_SUBJECTS = ["S001", "S002", "S003"]
DEFAULT_SENSOR_INDEX = 2


@dataclass
class CycleData:
    fsr_wave: np.ndarray
    grf_wave: np.ndarray
    peak_fsr_norm: float
    peak_grf_norm: float
    start_idx: int
    end_idx: int


@dataclass
class WindowStats:
    mean_wave: np.ndarray
    std_wave: np.ndarray
    mean_peak_fsr_norm: float
    std_peak_fsr_norm: float
    mean_peak_grf_norm: float
    std_peak_grf_norm: float


@dataclass
class WindowSelection:
    cycles: list[CycleData]
    start_cycle_1based: int
    end_cycle_1based: int


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--h5", type=Path, default=DEFAULT_H5, help="Path to combined_data_S008.h5")
    parser.add_argument("--condition", default="level_100mps", help="Condition name")
    parser.add_argument("--level", default="lv0", help="Level name")
    parser.add_argument("--trial", default="trial_01", help="Trial name")
    parser.add_argument("--subjects", nargs="+", default=DEFAULT_SUBJECTS, help="Subjects to analyze in order")
    parser.add_argument(
        "--within-plot-subjects",
        nargs="+",
        default=DEFAULT_WITHIN_PLOT_SUBJECTS,
        help="Subjects shown in within-subject waveform grid",
    )
    parser.add_argument(
        "--sensor-index",
        type=int,
        default=DEFAULT_SENSOR_INDEX,
        help="Analyze one sensor channel at a time: fsrL<idx> / fsrR<idx>",
    )
    parser.add_argument("--n-cycles", type=int, default=10, help="Cycles per comparison window")
    parser.add_argument("--resample-points", type=int, default=200, help="Samples per resampled gait cycle")
    parser.add_argument("--stance-threshold", type=float, default=50.0, help="GRF stance threshold in N")
    parser.add_argument("--min-cycle-len", type=int, default=50, help="Minimum onset-to-onset cycle length")
    parser.add_argument("--max-cycle-len", type=int, default=300, help="Maximum onset-to-onset cycle length")
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=None,
        help="Output directory. Defaults to ./analysis_outputs/fsr_recovery_<condition>_<level>_<trial>",
    )
    return parser.parse_args()


def make_out_dir(args: argparse.Namespace) -> Path:
    if args.out_dir is not None:
        out_dir = args.out_dir
    else:
        stem = f"fsr_recovery_fsr{args.sensor_index}_{args.condition}_{args.level}_{args.trial}"
        out_dir = Path(__file__).resolve().parent / "analysis_outputs" / stem
    out_dir.mkdir(parents=True, exist_ok=True)
    return out_dir


def decode_scalar(ds: h5py.Dataset) -> float:
    value = ds[()]
    if isinstance(value, bytes):
        return float(value.decode())
    return float(value)


def resample_1d(x: np.ndarray, n_points: int) -> np.ndarray:
    x = np.asarray(x, dtype=np.float32)
    if len(x) == 1:
        return np.full(n_points, x[0], dtype=np.float32)
    x_old = np.linspace(0.0, 1.0, len(x))
    x_new = np.linspace(0.0, 1.0, n_points)
    return np.interp(x_new, x_old, x).astype(np.float32)


def detect_stance_onsets(grf_positive_n: np.ndarray, threshold_n: float) -> np.ndarray:
    stance = (grf_positive_n > threshold_n).astype(np.int8)
    starts = np.where(np.diff(np.r_[0, stance]) == 1)[0]
    # If the recording begins in stance, the first cycle is partial and gets dropped.
    if len(starts) and starts[0] == 0:
        starts = starts[1:]
    return starts


def extract_cycles(
    trial_grp: h5py.Group,
    side: str,
    sensor_index: int,
    weight_kg: float,
    threshold_n: float,
    min_cycle_len: int,
    max_cycle_len: int,
    n_points: int,
) -> list[CycleData]:
    grf_positive_n = np.clip(
        -np.asarray(trial_grp["forceplate"]["grf"][side]["z"][:], dtype=np.float32),
        0.0,
        None,
    )
    prefix = "L" if side == "left" else "R"
    sensor_key = f"fsr{prefix}{sensor_index}"
    if sensor_key not in trial_grp["robot"]["insole"][side]:
        raise KeyError(f"Missing sensor {sensor_key} in trial group")
    fsr_wave_raw = np.asarray(trial_grp["robot"]["insole"][side][sensor_key][:], dtype=np.float32)
    starts = detect_stance_onsets(grf_positive_n, threshold_n=threshold_n)
    cycles: list[CycleData] = []
    for idx in range(len(starts) - 1):
        start = int(starts[idx])
        end = int(starts[idx + 1])
        if not (min_cycle_len <= end - start <= max_cycle_len):
            continue
        grf_wave = grf_positive_n[start:end] / weight_kg
        fsr_wave = fsr_wave_raw[start:end] / weight_kg
        cycles.append(
            CycleData(
                fsr_wave=resample_1d(fsr_wave, n_points=n_points),
                grf_wave=resample_1d(grf_wave, n_points=n_points),
                peak_fsr_norm=float(np.max(fsr_wave)),
                peak_grf_norm=float(np.max(grf_wave)),
                start_idx=start,
                end_idx=end,
            )
        )
    return cycles


def get_first_window(cycles: list[CycleData], n_cycles: int) -> WindowSelection:
    if len(cycles) < n_cycles:
        raise ValueError(f"Need at least {n_cycles} cycles, found {len(cycles)}")
    return WindowSelection(
        cycles=cycles[:n_cycles],
        start_cycle_1based=1,
        end_cycle_1based=n_cycles,
    )


def get_middle_window(cycles: list[CycleData], n_cycles: int) -> WindowSelection:
    if len(cycles) < n_cycles:
        raise ValueError(f"Need at least {n_cycles} cycles, found {len(cycles)}")
    mid = len(cycles) // 2
    start = max(0, mid - n_cycles // 2)
    end = start + n_cycles
    if end > len(cycles):
        end = len(cycles)
        start = end - n_cycles
    return WindowSelection(
        cycles=cycles[start:end],
        start_cycle_1based=start + 1,
        end_cycle_1based=end,
    )


def get_last_window(cycles: list[CycleData], n_cycles: int) -> WindowSelection:
    if len(cycles) < n_cycles:
        raise ValueError(f"Need at least {n_cycles} cycles, found {len(cycles)}")
    start = len(cycles) - n_cycles
    end = len(cycles)
    return WindowSelection(
        cycles=cycles[start:end],
        start_cycle_1based=start + 1,
        end_cycle_1based=end,
    )


def stack_waves(cycles: Iterable[CycleData], key: str) -> np.ndarray:
    return np.stack([getattr(cycle, key) for cycle in cycles], axis=0)


def get_window_stats(cycles: list[CycleData]) -> WindowStats:
    fsr_waves = stack_waves(cycles, "fsr_wave")
    peak_fsr = np.array([cycle.peak_fsr_norm for cycle in cycles], dtype=np.float64)
    peak_grf = np.array([cycle.peak_grf_norm for cycle in cycles], dtype=np.float64)
    return WindowStats(
        mean_wave=fsr_waves.mean(axis=0),
        std_wave=fsr_waves.std(axis=0),
        mean_peak_fsr_norm=float(peak_fsr.mean()),
        std_peak_fsr_norm=float(peak_fsr.std()),
        mean_peak_grf_norm=float(peak_grf.mean()),
        std_peak_grf_norm=float(peak_grf.std()),
    )


def rmse(a: np.ndarray, b: np.ndarray) -> float:
    a = np.asarray(a, dtype=np.float64)
    b = np.asarray(b, dtype=np.float64)
    return float(np.sqrt(np.mean((a - b) ** 2)))


def peak_ratio_scale(ref_peak: float, target_peak: float) -> float:
    eps = 1e-12
    if abs(target_peak) < eps:
        if abs(ref_peak) < eps:
            return 1.0
        return float("nan")
    if abs(ref_peak) < eps:
        return 0.0
    return float(ref_peak / target_peak)


def scale_fsr_stats(stats: WindowStats, scale_factor: float) -> WindowStats:
    if not np.isfinite(scale_factor):
        nan_wave = np.full_like(stats.mean_wave, np.nan, dtype=np.float64)
        return WindowStats(
            mean_wave=nan_wave.copy(),
            std_wave=nan_wave.copy(),
            mean_peak_fsr_norm=float("nan"),
            std_peak_fsr_norm=float("nan"),
            mean_peak_grf_norm=stats.mean_peak_grf_norm,
            std_peak_grf_norm=stats.std_peak_grf_norm,
        )
    return WindowStats(
        mean_wave=stats.mean_wave * scale_factor,
        std_wave=stats.std_wave * scale_factor,
        mean_peak_fsr_norm=stats.mean_peak_fsr_norm * scale_factor,
        std_peak_fsr_norm=stats.std_peak_fsr_norm * scale_factor,
        mean_peak_grf_norm=stats.mean_peak_grf_norm,
        std_peak_grf_norm=stats.std_peak_grf_norm,
    )


def compute_scale_factor(ref_stats: WindowStats, target_stats: WindowStats) -> float:
    return peak_ratio_scale(
        ref_peak=ref_stats.mean_peak_fsr_norm,
        target_peak=target_stats.mean_peak_fsr_norm,
    )


def expected_stats(ref_stats: WindowStats, target_stats: WindowStats) -> WindowStats:
    grf_ratio = target_stats.mean_peak_grf_norm / ref_stats.mean_peak_grf_norm
    return WindowStats(
        mean_wave=ref_stats.mean_wave * grf_ratio,
        std_wave=ref_stats.std_wave * grf_ratio,
        mean_peak_fsr_norm=ref_stats.mean_peak_fsr_norm * grf_ratio,
        std_peak_fsr_norm=ref_stats.std_peak_fsr_norm * grf_ratio,
        mean_peak_grf_norm=target_stats.mean_peak_grf_norm,
        std_peak_grf_norm=target_stats.std_peak_grf_norm,
    )


def corrected_stats(target_stats: WindowStats, scale_factor: float) -> WindowStats:
    if not np.isfinite(scale_factor):
        nan_wave = np.full_like(target_stats.mean_wave, np.nan, dtype=np.float64)
        return WindowStats(
            mean_wave=nan_wave.copy(),
            std_wave=nan_wave.copy(),
            mean_peak_fsr_norm=float("nan"),
            std_peak_fsr_norm=float("nan"),
            mean_peak_grf_norm=target_stats.mean_peak_grf_norm,
            std_peak_grf_norm=target_stats.std_peak_grf_norm,
        )
    return WindowStats(
        mean_wave=target_stats.mean_wave * scale_factor,
        std_wave=target_stats.std_wave * scale_factor,
        mean_peak_fsr_norm=target_stats.mean_peak_fsr_norm * scale_factor,
        std_peak_fsr_norm=target_stats.std_peak_fsr_norm * scale_factor,
        mean_peak_grf_norm=target_stats.mean_peak_grf_norm,
        std_peak_grf_norm=target_stats.std_peak_grf_norm,
    )


def summarize_comparison(
    name: str,
    side: str,
    sensor_key: str,
    ref_window: WindowSelection,
    target_window: WindowSelection,
    ref_anchor_scale: float = 1.0,
    ref_anchor_subject: str | None = None,
) -> dict[str, float | str | np.ndarray]:
    ref_cycles = ref_window.cycles
    target_cycles = target_window.cycles
    raw_ref_stats = get_window_stats(ref_cycles)
    ref_stats = scale_fsr_stats(raw_ref_stats, ref_anchor_scale)
    target_stats = get_window_stats(target_cycles)
    scale_factor = compute_scale_factor(ref_stats, target_stats)
    exp_stats = expected_stats(ref_stats, target_stats)
    corr_stats = corrected_stats(target_stats, scale_factor)

    return {
        "name": name,
        "side": side,
        "sensor_key": sensor_key,
        "ref_cycle_range": f"{ref_window.start_cycle_1based}-{ref_window.end_cycle_1based}",
        "target_cycle_range": f"{target_window.start_cycle_1based}-{target_window.end_cycle_1based}",
        "ref_anchor_scale": ref_anchor_scale,
        "ref_anchor_subject": ref_anchor_subject or "",
        "scale_factor": scale_factor,
        "ref_stats": ref_stats,
        "raw_ref_stats": raw_ref_stats,
        "target_stats": target_stats,
        "expected_stats": exp_stats,
        "corrected_stats": corr_stats,
        "mean_wave_rmse_raw": rmse(target_stats.mean_wave, exp_stats.mean_wave),
        "mean_wave_rmse_corrected": rmse(corr_stats.mean_wave, exp_stats.mean_wave),
        "std_wave_rmse_raw": rmse(target_stats.std_wave, exp_stats.std_wave),
        "std_wave_rmse_corrected": rmse(corr_stats.std_wave, exp_stats.std_wave),
        "peak_mean_abs_error_raw": abs(target_stats.mean_peak_fsr_norm - exp_stats.mean_peak_fsr_norm),
        "peak_mean_abs_error_corrected": abs(corr_stats.mean_peak_fsr_norm - exp_stats.mean_peak_fsr_norm),
        "peak_std_abs_error_raw": abs(target_stats.std_peak_fsr_norm - exp_stats.std_peak_fsr_norm),
        "peak_std_abs_error_corrected": abs(corr_stats.std_peak_fsr_norm - exp_stats.std_peak_fsr_norm),
    }


def plot_band(ax: plt.Axes, x: np.ndarray, mean: np.ndarray, std: np.ndarray, color: str, label: str, alpha: float) -> None:
    ax.plot(x, mean, color=color, linewidth=2.0, label=label)
    ax.fill_between(x, mean - std, mean + std, color=color, alpha=alpha)


def collect_wave_extrema(*row_groups: list[dict[str, object]]) -> tuple[float, float]:
    mins: list[float] = []
    maxs: list[float] = []
    for rows in row_groups:
        for row in rows:
            for key in ["raw_ref_stats", "ref_stats", "target_stats", "corrected_stats"]:
                stats = row[key]
                if not isinstance(stats, WindowStats):
                    continue
                mean = np.asarray(stats.mean_wave, dtype=np.float64)
                std = np.asarray(stats.std_wave, dtype=np.float64)
                arr_min = mean - std
                arr_max = mean + std
                if np.any(np.isfinite(arr_min)):
                    mins.append(float(np.nanmin(arr_min)))
                if np.any(np.isfinite(arr_max)):
                    maxs.append(float(np.nanmax(arr_max)))
    if not mins or not maxs:
        return 0.0, 1.0
    y_min = min(mins)
    y_max = max(maxs)
    pad = 0.05 * max(1e-6, y_max - y_min)
    return y_min - pad, y_max + pad


def draw_overlay_panel(ax: plt.Axes, summary: dict[str, object], title: str, sensor_label: str) -> None:
    x = np.linspace(0.0, 100.0, len(summary["ref_stats"].mean_wave))
    ref_stats: WindowStats = summary["ref_stats"]  # type: ignore[assignment]
    target_stats: WindowStats = summary["target_stats"]  # type: ignore[assignment]
    expected: WindowStats = summary["expected_stats"]  # type: ignore[assignment]
    corrected: WindowStats = summary["corrected_stats"]  # type: ignore[assignment]

    plot_band(ax, x, ref_stats.mean_wave, ref_stats.std_wave, "#4c78a8", "Reference", 0.18)
    plot_band(ax, x, expected.mean_wave, expected.std_wave, "#222222", "Expected", 0.10)
    plot_band(ax, x, target_stats.mean_wave, target_stats.std_wave, "#f58518", "Raw target", 0.18)
    plot_band(ax, x, corrected.mean_wave, corrected.std_wave, "#54a24b", "Corrected target", 0.18)

    scale = summary["scale_factor"]
    raw_rmse = summary["mean_wave_rmse_raw"]
    cor_rmse = summary["mean_wave_rmse_corrected"]
    ax.set_title(
        f"{title} | scale={scale:.3f} | mean RMSE {raw_rmse:.3f}->{cor_rmse:.3f}",
        fontsize=10,
    )
    ax.set_xlabel("Gait cycle (%)")
    ax.set_ylabel(f"{sensor_label} / weight")
    ax.grid(True, alpha=0.20)


def save_summary_csv(rows: list[dict[str, object]], csv_path: Path, mode: str) -> None:
    fields = [
        "mode",
        "name",
        "side",
        "sensor_key",
        "ref_cycle_range",
        "target_cycle_range",
        "ref_anchor_scale",
        "ref_anchor_subject",
        "scale_factor",
        "mean_wave_rmse_raw",
        "mean_wave_rmse_corrected",
        "std_wave_rmse_raw",
        "std_wave_rmse_corrected",
        "peak_mean_abs_error_raw",
        "peak_mean_abs_error_corrected",
        "peak_std_abs_error_raw",
        "peak_std_abs_error_corrected",
        "ref_mean_peak_grf_norm",
        "target_mean_peak_grf_norm",
        "ref_mean_peak_fsr_norm",
        "target_mean_peak_fsr_norm",
        "expected_mean_peak_fsr_norm",
        "corrected_mean_peak_fsr_norm",
    ]
    with csv_path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            ref_stats: WindowStats = row["ref_stats"]  # type: ignore[assignment]
            target_stats: WindowStats = row["target_stats"]  # type: ignore[assignment]
            expected: WindowStats = row["expected_stats"]  # type: ignore[assignment]
            corrected: WindowStats = row["corrected_stats"]  # type: ignore[assignment]
            writer.writerow(
                {
                    "mode": mode,
                    "name": row["name"],
                    "side": row["side"],
                    "sensor_key": row["sensor_key"],
                    "ref_cycle_range": row["ref_cycle_range"],
                    "target_cycle_range": row["target_cycle_range"],
                    "ref_anchor_scale": row["ref_anchor_scale"],
                    "ref_anchor_subject": row["ref_anchor_subject"],
                    "scale_factor": row["scale_factor"],
                    "mean_wave_rmse_raw": row["mean_wave_rmse_raw"],
                    "mean_wave_rmse_corrected": row["mean_wave_rmse_corrected"],
                    "std_wave_rmse_raw": row["std_wave_rmse_raw"],
                    "std_wave_rmse_corrected": row["std_wave_rmse_corrected"],
                    "peak_mean_abs_error_raw": row["peak_mean_abs_error_raw"],
                    "peak_mean_abs_error_corrected": row["peak_mean_abs_error_corrected"],
                    "peak_std_abs_error_raw": row["peak_std_abs_error_raw"],
                    "peak_std_abs_error_corrected": row["peak_std_abs_error_corrected"],
                    "ref_mean_peak_grf_norm": ref_stats.mean_peak_grf_norm,
                    "target_mean_peak_grf_norm": target_stats.mean_peak_grf_norm,
                    "ref_mean_peak_fsr_norm": ref_stats.mean_peak_fsr_norm,
                    "target_mean_peak_fsr_norm": target_stats.mean_peak_fsr_norm,
                    "expected_mean_peak_fsr_norm": expected.mean_peak_fsr_norm,
                    "corrected_mean_peak_fsr_norm": corrected.mean_peak_fsr_norm,
                }
            )


def draw_before_after_panel(
    ax_before: plt.Axes,
    ax_after: plt.Axes,
    middle_right_row: dict[str, object],
    end_right_row: dict[str, object],
    subject: str,
    sensor_label: str,
    y_limits: tuple[float, float],
) -> None:
    x = np.linspace(0.0, 100.0, len(middle_right_row["ref_stats"].mean_wave))
    beginning_raw: WindowStats = middle_right_row["raw_ref_stats"]  # type: ignore[assignment]
    beginning_after: WindowStats = middle_right_row["ref_stats"]  # type: ignore[assignment]
    middle_target: WindowStats = middle_right_row["target_stats"]  # type: ignore[assignment]
    middle_corrected: WindowStats = middle_right_row["corrected_stats"]  # type: ignore[assignment]
    end_target: WindowStats = end_right_row["target_stats"]  # type: ignore[assignment]
    end_corrected: WindowStats = end_right_row["corrected_stats"]  # type: ignore[assignment]
    ref_cycle_range = middle_right_row["ref_cycle_range"]
    middle_cycle_range = middle_right_row["target_cycle_range"]
    end_cycle_range = end_right_row["target_cycle_range"]
    ref_anchor_scale = float(middle_right_row["ref_anchor_scale"])
    ref_anchor_subject = middle_right_row["ref_anchor_subject"]

    beginning_before_label = f"right_beginning (cycles {ref_cycle_range})"
    if ref_anchor_subject and np.isfinite(ref_anchor_scale) and abs(ref_anchor_scale - 1.0) > 1e-9:
        beginning_after_label = (
            f"right_beginning (cycles {ref_cycle_range}, "
            f"scaled to {ref_anchor_subject} x{ref_anchor_scale:.2f})"
        )
    elif ref_anchor_subject and not np.isfinite(ref_anchor_scale):
        beginning_after_label = (
            f"right_beginning (cycles {ref_cycle_range}, "
            f"scaling to {ref_anchor_subject} unavailable)"
        )
    else:
        beginning_after_label = beginning_before_label

    plot_band(
        ax_before,
        x,
        beginning_raw.mean_wave,
        beginning_raw.std_wave,
        "#2ca02c",
        beginning_before_label,
        0.10,
    )
    plot_band(
        ax_before,
        x,
        middle_target.mean_wave,
        middle_target.std_wave,
        "#d62728",
        f"right_middle_raw (cycles {middle_cycle_range})",
        0.18,
    )
    plot_band(
        ax_after,
        x,
        beginning_after.mean_wave,
        beginning_after.std_wave,
        "#2ca02c",
        beginning_after_label,
        0.10,
    )
    plot_band(
        ax_after,
        x,
        middle_corrected.mean_wave,
        middle_corrected.std_wave,
        "#d62728",
        f"right_middle_corrected (cycles {middle_cycle_range})",
        0.18,
    )
    plot_band(
        ax_before,
        x,
        end_target.mean_wave,
        end_target.std_wave,
        "#9467bd",
        f"right_end_raw (cycles {end_cycle_range})",
        0.18,
    )
    plot_band(
        ax_after,
        x,
        end_corrected.mean_wave,
        end_corrected.std_wave,
        "#9467bd",
        f"right_end_corrected (cycles {end_cycle_range})",
        0.18,
    )

    ax_before.set_title(
        (
            f"{subject} | right | before compensation\n"
            f"all curves raw | beginning {ref_cycle_range}, middle {middle_cycle_range}, end {end_cycle_range}"
        ),
        fontsize=10,
    )
    ax_after.set_title(
        (
            f"{subject} | right | after compensation\n"
            f"beginning anchor applied only here | middle scale {middle_right_row['scale_factor']:.3f}, "
            f"end scale {end_right_row['scale_factor']:.3f}"
        ),
        fontsize=10,
    )
    for ax in (ax_before, ax_after):
        ax.set_xlabel("Gait cycle (%)")
        ax.set_ylabel(f"{sensor_label} / weight")
        ax.grid(True, alpha=0.20)
        ax.set_ylim(*y_limits)


def plot_within_subject_grid(
    middle_rows: list[dict[str, object]],
    end_rows: list[dict[str, object]],
    subjects: list[str],
    out_path: Path,
    sensor_label: str,
    y_limits: tuple[float, float],
) -> None:
    fig, axes = plt.subplots(
        nrows=len(subjects),
        ncols=2,
        figsize=(15, 3.0 * len(subjects)),
        dpi=180,
        sharex=True,
        sharey=True,
    )
    if len(subjects) == 1:
        axes = np.asarray([axes])
    for ridx, sub in enumerate(subjects):
        middle_right_row = next(item for item in middle_rows if item["name"] == sub and item["side"] == "right")
        end_right_row = next(item for item in end_rows if item["name"] == sub and item["side"] == "right")
        draw_before_after_panel(
            ax_before=axes[ridx, 0],
            ax_after=axes[ridx, 1],
            middle_right_row=middle_right_row,
            end_right_row=end_right_row,
            subject=sub,
            sensor_label=sensor_label,
            y_limits=y_limits,
        )
        if ridx == 0:
            axes[ridx, 0].legend(loc="upper right", fontsize=8, frameon=False, ncol=2)
            axes[ridx, 1].legend(loc="upper right", fontsize=8, frameon=False, ncol=2)
    fig.suptitle(
        "Within-subject compensation on shared-insole block (S001-S003): before vs after",
        fontsize=15,
        y=0.995,
    )
    fig.tight_layout(rect=[0, 0, 1, 0.985])
    fig.savefig(out_path, bbox_inches="tight")
    plt.close(fig)


def plot_between_subject_grid(
    rows: list[dict[str, object]],
    pair_names: list[str],
    out_path: Path,
    sensor_label: str,
    y_limits: tuple[float, float],
) -> None:
    fig, axes = plt.subplots(
        nrows=len(pair_names),
        ncols=2,
        figsize=(15, 3.0 * len(pair_names)),
        dpi=180,
        sharex=True,
        sharey=True,
    )
    if len(pair_names) == 1:
        axes = np.asarray([axes])
    side_order = ["left", "right"]
    for ridx, pair_name in enumerate(pair_names):
        for cidx, side in enumerate(side_order):
            ax = axes[ridx, cidx]
            row = next(item for item in rows if item["name"] == pair_name and item["side"] == side)
            draw_overlay_panel(ax, row, title=f"{pair_name} {side}", sensor_label=sensor_label)
            ax.set_ylim(*y_limits)
            if ridx == 0 and cidx == 1:
                ax.legend(loc="upper right", fontsize=8, frameon=False)
    fig.suptitle(
        "Between-subject sequential compensation: first 10 cycles vs previous subject",
        fontsize=15,
        y=0.995,
    )
    fig.tight_layout(rect=[0, 0, 1, 0.985])
    fig.savefig(out_path, bbox_inches="tight")
    plt.close(fig)


def plot_summary_metrics(rows: list[dict[str, object]], labels: list[str], out_path: Path, title: str) -> None:
    side_order = ["left", "right"]
    x = np.arange(len(labels))
    width = 0.18
    fig, axes = plt.subplots(2, 2, figsize=(16, 10), dpi=180)

    def pick(metric: str, side: str) -> np.ndarray:
        vals = []
        for label in labels:
            row = next(item for item in rows if item["name"] == label and item["side"] == side)
            vals.append(float(row[metric]))
        return np.array(vals, dtype=np.float64)

    for side, color in zip(side_order, ["#4c78a8", "#f58518"]):
        axes[0, 0].plot(x, pick("scale_factor", side), marker="o", linewidth=2.0, color=color, label=side)
    axes[0, 0].axhline(1.0, color="#444444", linestyle="--", linewidth=1.0)
    axes[0, 0].set_title("Scale factor")
    axes[0, 0].set_ylabel("Correction scale")
    axes[0, 0].grid(True, alpha=0.20)
    axes[0, 0].legend(frameon=False)

    offsets = {
        ("left", "raw"): -1.5 * width,
        ("left", "corrected"): -0.5 * width,
        ("right", "raw"): 0.5 * width,
        ("right", "corrected"): 1.5 * width,
    }
    colors = {
        ("left", "raw"): "#9ecae9",
        ("left", "corrected"): "#3182bd",
        ("right", "raw"): "#fdd0a2",
        ("right", "corrected"): "#e6550d",
    }

    for side in side_order:
        raw = pick("mean_wave_rmse_raw", side)
        corrected = pick("mean_wave_rmse_corrected", side)
        axes[0, 1].bar(x + offsets[(side, "raw")], raw, width=width, color=colors[(side, "raw")], label=f"{side} raw")
        axes[0, 1].bar(
            x + offsets[(side, "corrected")],
            corrected,
            width=width,
            color=colors[(side, "corrected")],
            label=f"{side} corrected",
        )
    axes[0, 1].set_title("Mean waveform RMSE to expected")
    axes[0, 1].set_ylabel("RMSE")
    axes[0, 1].grid(True, axis="y", alpha=0.20)
    axes[0, 1].legend(frameon=False, ncol=2)

    for side in side_order:
        raw = pick("std_wave_rmse_raw", side)
        corrected = pick("std_wave_rmse_corrected", side)
        axes[1, 0].bar(x + offsets[(side, "raw")], raw, width=width, color=colors[(side, "raw")], label=f"{side} raw")
        axes[1, 0].bar(
            x + offsets[(side, "corrected")],
            corrected,
            width=width,
            color=colors[(side, "corrected")],
            label=f"{side} corrected",
        )
    axes[1, 0].set_title("Std waveform RMSE to expected")
    axes[1, 0].set_ylabel("RMSE")
    axes[1, 0].grid(True, axis="y", alpha=0.20)

    for side in side_order:
        raw = pick("peak_std_abs_error_raw", side)
        corrected = pick("peak_std_abs_error_corrected", side)
        axes[1, 1].bar(x + offsets[(side, "raw")], raw, width=width, color=colors[(side, "raw")], label=f"{side} raw")
        axes[1, 1].bar(
            x + offsets[(side, "corrected")],
            corrected,
            width=width,
            color=colors[(side, "corrected")],
            label=f"{side} corrected",
        )
    axes[1, 1].set_title("Peak std absolute error")
    axes[1, 1].set_ylabel("Abs error")
    axes[1, 1].grid(True, axis="y", alpha=0.20)

    for ax in axes.ravel():
        ax.set_xticks(x)
        ax.set_xticklabels(labels, rotation=45, ha="right")

    fig.suptitle(title, fontsize=15, y=0.99)
    fig.tight_layout(rect=[0, 0, 1, 0.975])
    fig.savefig(out_path, bbox_inches="tight")
    plt.close(fig)


def main() -> None:
    args = parse_args()
    out_dir = make_out_dir(args)
    within_plot_subjects = [sub for sub in args.within_plot_subjects if sub in args.subjects]
    sensor_label = f"FSR{args.sensor_index}"

    within_rows: list[dict[str, object]] = []
    within_end_rows: list[dict[str, object]] = []
    between_rows: list[dict[str, object]] = []
    pair_names: list[str] = []

    with h5py.File(args.h5, "r") as f:
        subject_cycles: dict[str, dict[str, list[CycleData]]] = {}
        first_windows: dict[str, dict[str, WindowSelection]] = {}
        for sub in args.subjects:
            if sub not in f:
                raise KeyError(f"Subject {sub} not found in {args.h5}")
            weight_kg = decode_scalar(f[sub]["sub_info"]["weight"])
            trial_grp = f[sub][args.condition][args.level][args.trial]
            subject_cycles[sub] = {}
            first_windows[sub] = {}
            for side in ["left", "right"]:
                prefix = "L" if side == "left" else "R"
                sensor_key = f"fsr{prefix}{args.sensor_index}"
                cycles = extract_cycles(
                    trial_grp=trial_grp,
                    side=side,
                    sensor_index=args.sensor_index,
                    weight_kg=weight_kg,
                    threshold_n=args.stance_threshold,
                    min_cycle_len=args.min_cycle_len,
                    max_cycle_len=args.max_cycle_len,
                    n_points=args.resample_points,
                )
                if len(cycles) < args.n_cycles:
                    raise ValueError(
                        f"{sub} {side} has only {len(cycles)} usable cycles for "
                        f"{args.condition}/{args.level}/{args.trial}"
                    )
                subject_cycles[sub][side] = cycles
                first_windows[sub][side] = get_first_window(cycles, args.n_cycles)

        within_right_anchor_scales: dict[str, float] = {sub: 1.0 for sub in args.subjects}
        within_right_anchor_subjects: dict[str, str] = {sub: "" for sub in args.subjects}
        if within_plot_subjects:
            anchor_base_sub = within_plot_subjects[0]
            anchor_peak = get_window_stats(first_windows[anchor_base_sub]["right"].cycles).mean_peak_fsr_norm
            within_right_anchor_subjects[anchor_base_sub] = anchor_base_sub
            for sub in within_plot_subjects[1:]:
                sub_peak = get_window_stats(first_windows[sub]["right"].cycles).mean_peak_fsr_norm
                within_right_anchor_scales[sub] = peak_ratio_scale(anchor_peak, sub_peak)
                within_right_anchor_subjects[sub] = anchor_base_sub

        for sub in args.subjects:
            for side in ["left", "right"]:
                prefix = "L" if side == "left" else "R"
                sensor_key = f"fsr{prefix}{args.sensor_index}"
                cycles = subject_cycles[sub][side]
                ref_window = first_windows[sub][side]
                middle_window = get_middle_window(cycles, args.n_cycles)
                end_window = get_last_window(cycles, args.n_cycles)
                ref_anchor_scale = 1.0
                ref_anchor_subject = ""
                if side == "right" and sub in within_plot_subjects:
                    ref_anchor_scale = within_right_anchor_scales[sub]
                    ref_anchor_subject = within_right_anchor_subjects[sub]
                within_rows.append(
                    summarize_comparison(
                        name=sub,
                        side=side,
                        sensor_key=sensor_key,
                        ref_window=ref_window,
                        target_window=middle_window,
                        ref_anchor_scale=ref_anchor_scale,
                        ref_anchor_subject=ref_anchor_subject,
                    )
                )
                within_end_rows.append(
                    summarize_comparison(
                        name=sub,
                        side=side,
                        sensor_key=sensor_key,
                        ref_window=ref_window,
                        target_window=end_window,
                        ref_anchor_scale=ref_anchor_scale,
                        ref_anchor_subject=ref_anchor_subject,
                    )
                )

        for prev_sub, curr_sub in zip(args.subjects[:-1], args.subjects[1:]):
            pair_name = f"{prev_sub}->{curr_sub}"
            pair_names.append(pair_name)
            for side in ["left", "right"]:
                prefix = "L" if side == "left" else "R"
                sensor_key = f"fsr{prefix}{args.sensor_index}"
                ref_window = first_windows[prev_sub][side]
                target_window = first_windows[curr_sub][side]
                between_rows.append(
                    summarize_comparison(
                        name=pair_name,
                        side=side,
                        sensor_key=sensor_key,
                        ref_window=ref_window,
                        target_window=target_window,
                    )
                )

    # Use a fixed shared y-axis for waveform plots so all subjects/pairs are directly comparable.
    waveform_y_limits = (0.0, 1.0)

    save_summary_csv(within_rows, out_dir / "within_subject_summary.csv", mode="within_subject")
    save_summary_csv(between_rows, out_dir / "between_subject_summary.csv", mode="between_subject")

    plot_within_subject_grid(
        within_rows,
        within_end_rows,
        subjects=within_plot_subjects,
        out_path=out_dir / "within_subject_waveform_grid.png",
        sensor_label=sensor_label,
        y_limits=waveform_y_limits,
    )
    plot_between_subject_grid(
        between_rows,
        pair_names=pair_names,
        out_path=out_dir / "between_subject_waveform_grid.png",
        sensor_label=sensor_label,
        y_limits=waveform_y_limits,
    )
    plot_summary_metrics(
        within_rows,
        labels=args.subjects,
        out_path=out_dir / "within_subject_summary_metrics.png",
        title="Within-subject summary metrics",
    )
    plot_summary_metrics(
        between_rows,
        labels=pair_names,
        out_path=out_dir / "between_subject_summary_metrics.png",
        title="Between-subject sequential summary metrics",
    )

    print(f"[done] outputs saved to: {out_dir}")
    for file_name in [
        "within_subject_summary.csv",
        "between_subject_summary.csv",
        "within_subject_waveform_grid.png",
        "between_subject_waveform_grid.png",
        "within_subject_summary_metrics.png",
        "between_subject_summary_metrics.png",
    ]:
        print(out_dir / file_name)


if __name__ == "__main__":
    main()
