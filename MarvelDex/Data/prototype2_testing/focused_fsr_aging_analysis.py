from pathlib import Path

import matplotlib
matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


SCRIPT_DIR = Path(__file__).resolve().parent
DATA_DIR = SCRIPT_DIR.parent
RESULT_DIR = SCRIPT_DIR / "result_focus"
RESULT_DIR.mkdir(exist_ok=True)

SAMPLING_RATE = 500
WINDOW_SEC = 5.0
HEEL_CH = 2
TOE_CH = 7

PLOT_CONDITIONS = ["before_level", "after_level"]
TEMPORAL_LABELS = {
    "before_level": "S1-Level (fresh)",
    "after_level": "S2-Level (aged)",
}
BODY_MASS_KG = {
    "before_level": 82.0,
    "after_level": 62.0,
}
FILES = {
    "before_level": DATA_DIR / "before" / "level_075mps_lv0_01_processed.csv",
    "after_level": DATA_DIR / "after" / "level_075mps_lv0_01_processed.csv",
}

LEFT_FSR = [f"LeftFSR{i}" for i in range(1, 15)]
RIGHT_FSR = [f"RightFSR{i}" for i in range(1, 15)]
FOOT_CHANNELS = {"Left": LEFT_FSR, "Right": RIGHT_FSR}


def savefig(name: str) -> None:
    path = RESULT_DIR / f"{name}.png"
    plt.tight_layout()
    plt.savefig(path, dpi=160, bbox_inches="tight")
    plt.close("all")
    print(f"Saved {path}")


def detect_walking_onset(df: pd.DataFrame, fs: int = SAMPLING_RATE) -> int:
    hip = (df["LeftHipAngle"] + df["RightHipAngle"]) / 2.0
    rolling_std = hip.rolling(window=fs, min_periods=fs).std()
    initial_std = hip.iloc[:fs].std()
    threshold = max(initial_std * 5.0, 0.5)
    exceed = rolling_std[rolling_std > threshold].index
    if len(exceed) == 0:
        return 0
    return max(0, int(exceed[0]) - fs // 2)


def choose_arch_sensor(df: pd.DataFrame, onset: int, side: str) -> int:
    standing = df.iloc[:onset]
    walking = df.iloc[onset:]
    best_sensor = 1
    best_dynamic = None

    for sensor_num, ch in enumerate(FOOT_CHANNELS[side], start=1):
        baseline = float(standing[ch].mean()) if len(standing) else 0.0
        walk_p95 = float(np.percentile(walking[ch], 95)) if len(walking) else 0.0
        dynamic = walk_p95 - baseline
        if best_dynamic is None or dynamic < best_dynamic:
            best_dynamic = dynamic
            best_sensor = sensor_num

    return best_sensor


def build_plot_cache(data: dict[str, pd.DataFrame], normalize_by_mass: bool) -> tuple[dict, float, float]:
    cache = {}
    channel_max = 0.0
    sum_max = 0.0

    for condition in PLOT_CONDITIONS:
        df = data[condition]
        onset = detect_walking_onset(df)
        norm = BODY_MASS_KG[condition] if normalize_by_mass else 1.0
        left_arch = choose_arch_sensor(df, onset, "Left")
        right_arch = choose_arch_sensor(df, onset, "Right")

        time = np.arange(len(df), dtype=float) / SAMPLING_RATE
        start_idx = onset
        end_idx = min(len(df), onset + int(WINDOW_SEC * SAMPLING_RATE))
        window = slice(start_idx, end_idx)

        series = {
            "t": time[window] - (onset / SAMPLING_RATE),
            "left_heel": df[f"LeftFSR{HEEL_CH}"].to_numpy(dtype=float)[window] / norm,
            "left_toe": df[f"LeftFSR{TOE_CH}"].to_numpy(dtype=float)[window] / norm,
            "left_arch": df[f"LeftFSR{left_arch}"].to_numpy(dtype=float)[window] / norm,
            "right_heel": df[f"RightFSR{HEEL_CH}"].to_numpy(dtype=float)[window] / norm,
            "right_toe": df[f"RightFSR{TOE_CH}"].to_numpy(dtype=float)[window] / norm,
            "right_arch": df[f"RightFSR{right_arch}"].to_numpy(dtype=float)[window] / norm,
            "left_sum": df[LEFT_FSR].sum(axis=1).to_numpy(dtype=float)[window] / norm,
            "right_sum": df[RIGHT_FSR].sum(axis=1).to_numpy(dtype=float)[window] / norm,
            "left_arch_num": left_arch,
            "right_arch_num": right_arch,
        }
        cache[condition] = series

        channel_max = max(
            channel_max,
            float(np.max(series["left_heel"])),
            float(np.max(series["left_toe"])),
            float(np.max(series["left_arch"])),
            float(np.max(series["right_heel"])),
            float(np.max(series["right_toe"])),
            float(np.max(series["right_arch"])),
        )
        sum_max = max(
            sum_max,
            float(np.max(series["left_sum"])),
            float(np.max(series["right_sum"])),
        )

        print(
            f"{condition}: onset={onset / SAMPLING_RATE:.2f}s, "
            f"Left arch=FSR{left_arch}, Right arch=FSR{right_arch}"
        )

    return cache, channel_max, sum_max


def plot_time_series_panel(data: dict[str, pd.DataFrame], normalize_by_mass: bool, output_name: str) -> None:
    plot_cache, channel_max, sum_max = build_plot_cache(data, normalize_by_mass)
    fig, axes = plt.subplots(len(PLOT_CONDITIONS), 3, figsize=(21, 6.2), sharex=False)
    if len(PLOT_CONDITIONS) == 1:
        axes = np.array([axes])

    channel_ylim = (0.0, channel_max * 1.05 if channel_max > 0 else 1.0)
    sum_ylim = (0.0, sum_max * 1.05 if sum_max > 0 else 1.0)
    channel_ylabel = "FSR / body mass (ADC/kg)" if normalize_by_mass else "FSR (ADC)"
    sum_ylabel = "FSR sum / body mass (ADC/kg)" if normalize_by_mass else "FSR sum (ADC)"

    for row, condition in enumerate(PLOT_CONDITIONS):
        ax_left = axes[row, 0]
        ax_right = axes[row, 1]
        ax_sum = axes[row, 2]
        s = plot_cache[condition]

        ax_left.plot(s["t"], s["left_heel"], color="#60a5fa", linewidth=0.9, alpha=0.9, label=f"Heel FSR{HEEL_CH}")
        ax_left.plot(s["t"], s["left_toe"], color="#0f766e", linewidth=0.9, alpha=0.9, label=f"Toe FSR{TOE_CH}")
        ax_left.plot(s["t"], s["left_arch"], color="#7c3aed", linewidth=0.9, alpha=0.9, label=f"Arch FSR{s['left_arch_num']}")

        ax_right.plot(s["t"], s["right_heel"], color="#fca5a5", linewidth=0.9, alpha=0.9, label=f"Heel FSR{HEEL_CH}")
        ax_right.plot(s["t"], s["right_toe"], color="#f59e0b", linewidth=0.9, alpha=0.9, label=f"Toe FSR{TOE_CH}")
        ax_right.plot(s["t"], s["right_arch"], color="#9333ea", linewidth=0.9, alpha=0.9, label=f"Arch FSR{s['right_arch_num']}")

        ax_sum.plot(s["t"], s["left_sum"], color="#1d4ed8", linewidth=1.2, alpha=0.9, label="Left sum")
        ax_sum.plot(s["t"], s["right_sum"], color="#dc2626", linewidth=1.2, alpha=0.9, label="Right sum")

        for ax in [ax_left, ax_right, ax_sum]:
            ax.axvline(0.0, color="black", linestyle="--", linewidth=1.0, alpha=0.7)
            ax.set_xlim(0.0, WINDOW_SEC)
            ax.grid(alpha=0.2)

        ax_left.set_ylim(*channel_ylim)
        ax_right.set_ylim(*channel_ylim)
        ax_sum.set_ylim(*sum_ylim)
        ax_left.text(0.03, channel_ylim[1] * 0.92, "walk onset", fontsize=8, ha="left", va="top")
        ax_right.text(0.03, channel_ylim[1] * 0.92, "walk onset", fontsize=8, ha="left", va="top")
        ax_sum.text(0.03, sum_ylim[1] * 0.92, "walk onset", fontsize=8, ha="left", va="top")

        ax_left.set_title(f"{TEMPORAL_LABELS[condition]}: Left")
        ax_right.set_title(f"{TEMPORAL_LABELS[condition]}: Right")
        ax_sum.set_title(f"{TEMPORAL_LABELS[condition]}: FSR sum")
        ax_left.set_ylabel(channel_ylabel)
        ax_right.set_ylabel(channel_ylabel)
        ax_sum.set_ylabel(sum_ylabel)

        if row == 0:
            ax_left.legend(fontsize=8, loc="upper right")
            ax_right.legend(fontsize=8, loc="upper right")
            ax_sum.legend(fontsize=8, loc="upper right")

    axes[-1, 0].set_xlabel("Time since walking onset (s)")
    axes[-1, 1].set_xlabel("Time since walking onset (s)")
    axes[-1, 2].set_xlabel("Time since walking onset (s)")
    savefig(output_name)


print("Loading processed datasets...")
data = {key: pd.read_csv(path) for key, path in FILES.items()}
plot_time_series_panel(data, normalize_by_mass=True, output_name="fsr_time_series_4panel")
plot_time_series_panel(data, normalize_by_mass=False, output_name="fsr_time_series_4panel_raw")
