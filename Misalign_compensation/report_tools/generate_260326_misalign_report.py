from __future__ import annotations

import json
import math
import re
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Iterable

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import seaborn as sns

from reportlab.lib import colors
from reportlab.lib.enums import TA_CENTER, TA_LEFT
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
from reportlab.lib.units import cm
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import (
    Image,
    KeepTogether,
    PageBreak,
    Paragraph,
    SimpleDocTemplate,
    Spacer,
    Table,
    TableStyle,
)


ROOT = Path("/home/chanyoungko/IIT/Misalign_compensation")
ASSET_DIR = ROOT / "260326_Misalign_정리_assets"
OUTPUT_PDF = ROOT / "260326_Misalign_정리.pdf"
LATEX_BUILD_DIR = Path("/tmp/misalign_report_build")

if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


def ensure_dirs() -> None:
    ASSET_DIR.mkdir(parents=True, exist_ok=True)


def setup_fonts():
    font_dir = Path("/home/chanyoungko/.local/lib/python3.13/site-packages/koreanize_matplotlib/fonts")
    regular = font_dir / "NanumGothic.ttf"
    bold = font_dir / "NanumGothicBold.ttf"
    if not regular.exists() or not bold.exists():
        raise FileNotFoundError(f"Nanum fonts not found in {font_dir}")
    pdfmetrics.registerFont(TTFont("NanumGothic", str(regular)))
    pdfmetrics.registerFont(TTFont("NanumGothicBold", str(bold)))


def styles():
    ss = getSampleStyleSheet()
    ss.add(
        ParagraphStyle(
            name="KRTitle",
            parent=ss["Title"],
            fontName="NanumGothicBold",
            fontSize=22,
            leading=34,
            alignment=TA_CENTER,
            spaceAfter=12,
        )
    )
    ss.add(
        ParagraphStyle(
            name="KRHeading1",
            parent=ss["Heading1"],
            fontName="NanumGothicBold",
            fontSize=16,
            leading=26,
            spaceAfter=8,
            spaceBefore=10,
        )
    )
    ss.add(
        ParagraphStyle(
            name="KRHeading2",
            parent=ss["Heading2"],
            fontName="NanumGothicBold",
            fontSize=12.5,
            leading=20,
            spaceAfter=6,
            spaceBefore=8,
        )
    )
    ss.add(
        ParagraphStyle(
            name="KRBody",
            parent=ss["BodyText"],
            fontName="NanumGothic",
            fontSize=9.5,
            leading=16,
            alignment=TA_LEFT,
            wordWrap="CJK",
            spaceAfter=4,
            spaceBefore=0,
        )
    )
    ss.add(
        ParagraphStyle(
            name="KRSmall",
            parent=ss["BodyText"],
            fontName="NanumGothic",
            fontSize=8.2,
            leading=13,
            alignment=TA_LEFT,
            wordWrap="CJK",
            spaceAfter=3,
            spaceBefore=0,
        )
    )
    ss.add(
        ParagraphStyle(
            name="KRCaption",
            parent=ss["Italic"],
            fontName="NanumGothic",
            fontSize=8.2,
            leading=12,
            alignment=TA_CENTER,
            textColor=colors.HexColor("#444444"),
            spaceAfter=6,
            spaceBefore=2,
        )
    )
    ss.add(
        ParagraphStyle(
            name="KRTableHeader",
            parent=ss["BodyText"],
            fontName="NanumGothicBold",
            fontSize=8.0,
            leading=12.5,
            alignment=TA_CENTER,
            wordWrap="CJK",
            spaceAfter=0,
            spaceBefore=0,
        )
    )
    ss.add(
        ParagraphStyle(
            name="KRTableCell",
            parent=ss["BodyText"],
            fontName="NanumGothic",
            fontSize=7.6,
            leading=11.5,
            alignment=TA_LEFT,
            wordWrap="CJK",
            spaceAfter=0,
            spaceBefore=0,
        )
    )
    return ss


def para(text: str, style_name: str, ss):
    return Paragraph(text.replace("\n", "<br/>"), ss[style_name])


def as_float(x):
    try:
        return float(x)
    except Exception:
        return math.nan


def load_metrics_rows(base_dir: Path):
    rows = []
    if not base_dir.exists():
        return rows
    for d in sorted(base_dir.iterdir()):
        if not d.is_dir():
            continue
        metrics_path = d / "metrics.json"
        if not metrics_path.exists():
            continue
        obj = json.loads(metrics_path.read_text())
        if obj.get("test_mae") is None:
            continue
        rows.append(
            {
                "name": d.name,
                "mae": as_float(obj.get("test_mae")),
                "rmse": as_float(obj.get("test_rmse")),
                "params": as_float(obj.get("n_params")),
                "model_size_mb": as_float(obj.get("model_size_mb")),
                "host_latency_ms": as_float(obj.get("host_latency_ms")),
                "host_max_freq_hz": as_float(obj.get("host_max_freq_hz")),
            }
        )
    return rows


def archive_rows(archive_name: str, prefix: str | None = None):
    rows = load_metrics_rows(ROOT / "experiments" / "archive_season2" / archive_name)
    if prefix:
        rows = [r for r in rows if Path(r["name"]).name.startswith(prefix)]
    return pd.DataFrame(rows).sort_values(["mae", "rmse", "name"]).reset_index(drop=True)


def archive_rows_for(season: int, archive_name: str, prefix: str | None = None):
    rows = load_metrics_rows(ROOT / "experiments" / f"archive_season{season}" / archive_name)
    if prefix:
        rows = [r for r in rows if Path(r["name"]).name.startswith(prefix)]
    df = pd.DataFrame(rows)
    if df.empty:
        return df
    return df.sort_values(["mae", "rmse", "name"]).reset_index(drop=True)


def aggregate_experiment_metrics(exp_root: Path) -> pd.DataFrame:
    rows = load_metrics_rows(exp_root)
    grouped = {}
    for row in rows:
        name = Path(str(row["name"])).name
        base = re.sub(r"_Test-.*$", "", name)
        grouped.setdefault(base, []).append(row)
    out = []
    for base, items in grouped.items():
        out.append(
            {
                "name": base,
                "n_runs": len(items),
                "mae_mean": float(np.mean([r["mae"] for r in items])),
                "mae_std": float(np.std([r["mae"] for r in items], ddof=0)),
                "rmse_mean": float(np.mean([r["rmse"] for r in items])),
                "host_latency_ms": float(np.mean([r["host_latency_ms"] for r in items])),
                "host_max_freq_hz": float(np.mean([r["host_max_freq_hz"] for r in items])),
                "model_size_mb": float(np.mean([r["model_size_mb"] for r in items])),
            }
        )
    if not out:
        return pd.DataFrame()
    return pd.DataFrame(out).sort_values(["mae_mean", "rmse_mean", "name"]).reset_index(drop=True)


def final_loso_summary():
    return pd.read_csv(ROOT / "compare_result" / "archive_season2" / "archive_6" / "final_loso" / "summary_mean_std.csv")


def full_loso_summary_for(archive_name: str):
    path = ROOT / "compare_result" / "archive_season2" / archive_name / "final_loso" / "summary_mean_std.csv"
    if not path.exists():
        return pd.DataFrame()
    return pd.read_csv(path)


def full_loso_fold_for(archive_name: str):
    path = ROOT / "compare_result" / "archive_season2" / archive_name / "final_loso" / "fold_metrics.csv"
    if not path.exists():
        return pd.DataFrame()
    return pd.read_csv(path)


def reproducibility_summary():
    return pd.read_csv(ROOT / "compare_result" / "archive_season2" / "archive_6" / "reproducibility" / "summary_by_model.csv")


def reproducibility_by_seed():
    return pd.read_csv(ROOT / "compare_result" / "archive_season2" / "archive_6" / "reproducibility" / "summary_by_model_seed.csv")


def reproducibility_pairs():
    return pd.read_csv(ROOT / "compare_result" / "archive_season2" / "archive_6" / "reproducibility" / "paired_mae_by_subject_seed.csv")


def historical_global_summary():
    return pd.read_csv(ROOT / "compare_result" / "final_ablation_summary.csv").sort_values(["mae", "rmse", "group"]).reset_index(drop=True)


def final_global_baseline():
    df = pd.read_csv(ROOT / "compare_result" / "final_ablation_summary.csv")
    row = df[df["group"] == "exp_A5_res_noise_001"].iloc[0]
    return {
        "name": row["group"],
        "mae": float(row["mae"]),
        "rmse": float(row["rmse"]),
    }


def get_archive_best(df: pd.DataFrame):
    if df.empty:
        return None
    row = df.iloc[0]
    return {
        "name": row["name"],
        "mae": float(row["mae"]),
        "rmse": float(row["rmse"]),
    }


def safe_best(df: pd.DataFrame):
    if df is None or df.empty:
        return {"name": "-", "mae": math.nan, "rmse": math.nan}
    row = df.sort_values(["mae", "rmse"]).iloc[0]
    return {"name": row["name"], "mae": float(row["mae"]), "rmse": float(row["rmse"])}


def safe_best_full(df: pd.DataFrame):
    if df is None or df.empty:
        return {"experiment": "-", "mae_mean": math.nan, "rmse_mean": math.nan}
    row = df.sort_values(["mae_mean", "rmse_mean"]).iloc[0]
    return {"experiment": row["experiment"], "mae_mean": float(row["mae_mean"]), "rmse_mean": float(row["rmse_mean"])}


def pretty_name(name: str) -> str:
    n = Path(str(name)).name
    n = re.sub(r"^exp_[A-Z0-9]+_", "", n)
    n = re.sub(r"_Test-.*$", "", n)
    repl = {
        "ctrl_teacher_kinpower": "teacher kinpower baseline",
        "ctrl_tiny_kinpower_w100": "tiny kinpower baseline",
        "tiny_kinpower_w100": "tiny kinpower w100",
        "tiny_loadgate_kinpower_w100_smooth": "tiny load-gated kinpower smooth",
        "ctrl_quat_res_noise001": "quat residual noise001 baseline",
        "ctrl_quat_res_kin_power": "quat residual kin+power baseline",
        "feat_quat_kinematics": "quat + kinematic derivatives",
        "feat_quat_mech_power": "quat + mech_power",
        "feat_quat_load_coupling": "quat + load_coupling",
        "feat_kinpower_load_coupling": "kinpower + load_coupling",
        "ref_euler_quat": "Euler + quaternion reference",
        "repr_rawimu6": "raw trunk IMU 6ch",
        "repr_rawimu6_quat": "raw trunk IMU 6ch + quat",
        "repr_gravity6": "gravity-aligned IMU 6ch",
        "repr_gravity6_quat": "gravity-aligned IMU 6ch + quat",
        "repr_symyaw6": "symmetry-yaw IMU 6ch",
        "repr_symyaw6_quat": "symmetry-yaw IMU 6ch + quat",
        "repr_bodypca6": "PCA body-frame IMU 6ch",
        "repr_bodypca6_quat": "PCA body-frame IMU 6ch + quat",
        "feat_worksplit_teacher": "teacher + interface worksplit",
        "feat_hysteresis_teacher": "teacher + hysteresis",
        "feat_impedance_teacher": "teacher + effective impedance",
        "feat_lag_teacher": "teacher + interface lag",
        "feat_phase_teacher": "teacher + phase-locked features",
        "feat_trunk_consistency_teacher": "teacher + trunk consistency",
        "feat_best_physics_teacher": "teacher + all selected physics",
        "tiny_best_physics_w100": "tiny + selected physics",
        "tiny_phase_trunk_w100": "tiny + phase + trunk consistency",
        "feat_norm_worksplit_teacher": "teacher + normalized worksplit",
        "feat_short_worksplit_teacher": "teacher + short-window worksplit",
        "feat_norm_load_coupling_teacher": "teacher + normalized load coupling",
        "feat_normload_worksplit_teacher": "teacher + normalized load + worksplit",
        "ctrl_teacher_thigh": "teacher thigh baseline",
        "ctrl_tiny_thigh": "tiny thigh baseline",
        "lin_grouped_physics": "grouped linear physics-only",
        "fact_teacher_full": "factorized teacher full",
        "fact_teacher_physonly": "factorized teacher physics-only",
        "fact_tiny_full": "factorized tiny full",
    }
    return repl.get(n, n.replace("_", " "))


def make_barplot(df: pd.DataFrame, x: str, y: str, out: Path, title: str, xlabel: str, figsize=(8, 4)):
    plt.figure(figsize=figsize)
    sns.barplot(data=df, x=x, y=y, color="#4C78A8")
    plt.title(title)
    plt.xlabel(xlabel)
    plt.ylabel("")
    plt.tight_layout()
    plt.savefig(out, dpi=180)
    plt.close()


def insert_breaks_for_plot(data, offsets, fs=100.0):
    data = np.squeeze(data)
    if offsets is None or len(offsets) < 2:
        return data, np.arange(len(data)) / fs
    segments = []
    total_len = len(data)
    for i in range(len(offsets)):
        start = offsets[i]
        end = offsets[i + 1] if i < len(offsets) - 1 else total_len
        if start >= end:
            continue
        segments.append(data[start:end])
        if i < len(offsets) - 1:
            segments.append(np.array([np.nan], dtype=float))
    if not segments:
        return data, np.arange(len(data)) / fs
    new_data = np.concatenate(segments)
    new_t = np.arange(len(new_data)) / fs
    return new_data, new_t


def make_representative_trajectory_plot(exp_dir: Path, out_path: Path, title: str):
    import compare_results as cr
    try:
        import torch
    except ImportError:
        if out_path.exists():
            return
        raise

    res = cr.load_and_evaluate(exp_dir, torch.device("cpu"), return_seqs=True, return_extras=False)
    if not res or "y_true_seq" not in res or "y_pred_seq" not in res:
        return

    y_true = np.squeeze(res["y_true_seq"])
    y_pred = np.squeeze(res["y_pred_seq"])
    robot_ref = np.squeeze(res.get("robot_ref_seq")) if res.get("robot_ref_seq") is not None else None
    offsets = res.get("series_offsets")

    if y_true.ndim > 1:
        y_true = y_true[:, 0]
    if y_pred.ndim > 1:
        y_pred = y_pred[:, 0]
    if robot_ref is not None and robot_ref.ndim > 1:
        robot_ref = robot_ref[:, 0]

    y_true_plot, t_plot = insert_breaks_for_plot(y_true, offsets)
    y_pred_plot, _ = insert_breaks_for_plot(y_pred, offsets)
    robot_plot, _ = insert_breaks_for_plot(robot_ref, offsets) if robot_ref is not None else (None, None)

    fig, axes = plt.subplots(2, 1, figsize=(12, 10))
    axes[0].plot(t_plot, y_true_plot, label="Ground Truth", alpha=0.8, color="black", linewidth=1.4)
    axes[0].plot(t_plot, y_pred_plot, label="Prediction", alpha=0.9, color="red", linestyle="--", linewidth=1.2)
    if robot_plot is not None:
        axes[0].plot(t_plot, robot_plot, label="Calibrated Robot Angle", alpha=0.85, color="#1f77b4", linestyle="-.", linewidth=1.0)
    if offsets is not None:
        for offset in offsets:
            if offset == 0:
                continue
            axes[0].axvline(x=offset / 100.0, color="gray", linestyle=":", alpha=0.35)
    axes[0].set_title(f"{title} - Full Duration")
    axes[0].set_xlabel("Time (s)")
    axes[0].set_ylabel("Hip Angle (deg)")
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    start_t, end_t = 10.0, 25.0
    if t_plot[-1] < 15.0:
        start_t, end_t = 0.0, float(t_plot[-1])
    elif t_plot[-1] < 25.0:
        start_t, end_t = 0.0, 15.0
    mask = (t_plot >= start_t) & (t_plot <= end_t)
    if np.sum(mask) > 0:
        axes[1].plot(t_plot[mask], y_true_plot[mask], label="Ground Truth", color="black", alpha=0.8, linewidth=1.4)
        axes[1].plot(t_plot[mask], y_pred_plot[mask], label="Prediction", color="red", linestyle="--", alpha=0.9, linewidth=1.2)
        if robot_plot is not None:
            axes[1].plot(t_plot[mask], robot_plot[mask], label="Calibrated Robot Angle", color="#1f77b4", linestyle="-.", alpha=0.85, linewidth=1.0)
        axes[1].set_title(f"Zoomed ({start_t:.0f}s - {end_t:.0f}s)")
        axes[1].set_xlabel("Time (s)")
        axes[1].set_ylabel("Hip Angle (deg)")
        axes[1].legend()
        axes[1].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(out_path, dpi=180)
    plt.close()


def make_figures():
    global_hist = historical_global_summary()
    a1 = archive_rows("archive_1")
    a2 = archive_rows("archive_2")
    a3 = archive_rows("archive_3")
    a4 = archive_rows("archive_4")
    a5 = archive_rows("archive_5")
    a6_screen = archive_rows("archive_6", prefix="exp_S2A6_")
    a7_screen = archive_rows("archive_7", prefix="exp_S2A7_")
    a8_screen = archive_rows("archive_8", prefix="exp_S2A8_")
    a7_full = full_loso_summary_for("archive_7")
    a8_full = full_loso_summary_for("archive_8")
    s3_summary = aggregate_experiment_metrics(ROOT / "experiments" / "archive_season3" / "archive_1")

    if not global_hist.empty:
        top20 = global_hist.head(20).copy()
        top20["label"] = top20["group"].apply(pretty_name)
        plt.figure(figsize=(9.6, 7.2))
        sns.barplot(data=top20.sort_values("mae", ascending=True), x="mae", y="label", color="#1D3557")
        plt.xlabel("MAE")
        plt.ylabel("")
        plt.title("Historical Top-20 Models by MAE (shared hip-angle target)")
        plt.tight_layout()
        plt.savefig(ASSET_DIR / "historical_top20_mae.png", dpi=180)
        plt.close()

    if not s3_summary.empty:
        plt.figure(figsize=(8.8, 4.6))
        sns.barplot(data=s3_summary.sort_values("mae_mean"), x="mae_mean", y="name", color="#6D597A")
        plt.xlabel("MAE mean")
        plt.ylabel("")
        plt.title("Season3 Archive 1 Summary (human-thigh target)")
        plt.tight_layout()
        plt.savefig(ASSET_DIR / "season3_archive1_summary.png", dpi=180)
        plt.close()

    progress_rows = []
    for label, df in [
        ("A1", a1),
        ("A2", a2),
        ("A3", a3),
        ("A4", a4),
        ("A5", a5),
        ("A6(screen)", a6_screen),
        ("A7(screen)", a7_screen),
        ("A8(screen)", a8_screen),
    ]:
        if not df.empty:
            progress_rows.append({"archive": label, "mae": float(df.iloc[0]["mae"])})
    progress_df = pd.DataFrame(progress_rows)
    plt.figure(figsize=(7.2, 3.8))
    sns.barplot(data=progress_df, x="archive", y="mae", color="#3A86FF")
    plt.title("Season2 Archive Best MAE (single-LOSO screening)")
    plt.ylabel("MAE")
    plt.xlabel("")
    plt.tight_layout()
    plt.savefig(ASSET_DIR / "season2_archive_progress.png", dpi=180)
    plt.close()

    if not a3.empty:
        make_barplot(a3.head(9), "mae", "name", ASSET_DIR / "archive3_representation.png", "Archive 3 Representation Comparison", "MAE", figsize=(8.8, 4.8))
    if not a4.empty:
        make_barplot(a4.head(9), "mae", "name", ASSET_DIR / "archive4_biomech_screening.png", "Archive 4 Biomechanical Feature Screening", "MAE", figsize=(8.8, 4.8))
    if not a5.empty:
        make_barplot(a5.head(11), "mae", "name", ASSET_DIR / "archive5_misalign_screening.png", "Archive 5 Misalignment-aware Feature Screening", "MAE", figsize=(8.8, 5.4))
    if not a7_screen.empty:
        make_barplot(a7_screen.head(11), "mae", "name", ASSET_DIR / "archive7_physics_screening.png", "Archive 7 Physics Feature Screening", "MAE", figsize=(9.4, 5.2))
    if not a8_screen.empty:
        make_barplot(a8_screen.head(7), "mae", "name", ASSET_DIR / "archive8_physics_screening.png", "Archive 8 Physics Feature Re-screening", "MAE", figsize=(9.0, 4.8))

    fl = final_loso_summary().copy()
    fl["label"] = fl["experiment"].replace(
        {
            "exp_S2A6F_ctrl_teacher_kinpower": "teacher",
            "exp_S2A6F_tiny_kinpower_w100": "tiny",
            "exp_S2A6F_tiny_loadgate_kinpower_w100_smooth": "tiny_loadgate",
        }
    )
    plt.figure(figsize=(6.8, 4.4))
    sns.scatterplot(data=fl, x="model_size_mb_mean", y="mae_mean", hue="label", style="label", s=120)
    for _, r in fl.iterrows():
        plt.text(r["model_size_mb_mean"] * 1.01, r["mae_mean"] + 0.002, str(r["label"]), fontsize=9)
    plt.xlabel("Model size (MB)")
    plt.ylabel("MAE mean (8-subject LOSO)")
    plt.title("Archive 6 Accuracy-Size Tradeoff")
    plt.tight_layout()
    plt.savefig(ASSET_DIR / "archive6_tradeoff_size_mae.png", dpi=180)
    plt.close()

    plt.figure(figsize=(6.8, 4.4))
    sns.scatterplot(data=fl, x="host_latency_ms_mean", y="mae_mean", hue="label", style="label", s=120)
    for _, r in fl.iterrows():
        plt.text(r["host_latency_ms_mean"] * 1.01, r["mae_mean"] + 0.002, str(r["label"]), fontsize=9)
    plt.xlabel("Host latency (ms)")
    plt.ylabel("MAE mean (8-subject LOSO)")
    plt.title("Archive 6 Accuracy-Latency Tradeoff")
    plt.tight_layout()
    plt.savefig(ASSET_DIR / "archive6_tradeoff_latency_mae.png", dpi=180)
    plt.close()

    seed_df = reproducibility_by_seed().copy()
    seed_df["label"] = seed_df["model"].replace({"teacher_kinpower": "teacher", "tiny_kinpower_w100": "tiny"})
    plt.figure(figsize=(6.5, 4.0))
    sns.lineplot(data=seed_df, x="seed", y="mae_mean", hue="label", marker="o")
    plt.fill_between([], [], [])  # no-op to keep backend stable
    plt.ylabel("MAE mean")
    plt.xlabel("Random seed")
    plt.title("Archive 6 Reproducibility by Seed")
    plt.tight_layout()
    plt.savefig(ASSET_DIR / "archive6_seed_mae.png", dpi=180)
    plt.close()

    pair_df = reproducibility_pairs().copy()
    pair_df["subject_seed"] = pair_df["subject"] + "_seed" + pair_df["seed"].astype(str)
    plt.figure(figsize=(7.6, 4.2))
    sns.barplot(data=pair_df.sort_values("tiny_minus_teacher_mae"), x="subject_seed", y="tiny_minus_teacher_mae", color="#FB5607")
    plt.axhline(0.0, color="black", linewidth=1)
    plt.xticks(rotation=60, ha="right", fontsize=8)
    plt.ylabel("Tiny - Teacher MAE")
    plt.xlabel("")
    plt.title("Archive 6 Pairwise MAE Difference (negative is better for tiny)")
    plt.tight_layout()
    plt.savefig(ASSET_DIR / "archive6_pairwise_mae_diff.png", dpi=180)
    plt.close()

    if not a7_full.empty:
        plt.figure(figsize=(8.4, 4.6))
        sns.barplot(data=a7_full.sort_values("mae_mean"), x="mae_mean", y="experiment", color="#4C78A8")
        plt.xlabel("MAE mean")
        plt.ylabel("")
        plt.title("Archive 7 Full LOSO Comparison")
        plt.tight_layout()
        plt.savefig(ASSET_DIR / "archive7_full_loso.png", dpi=180)
        plt.close()

    if not a8_full.empty:
        plt.figure(figsize=(8.8, 4.8))
        sns.barplot(data=a8_full.sort_values("mae_mean"), x="mae_mean", y="experiment", color="#2A9D8F")
        plt.xlabel("MAE mean")
        plt.ylabel("")
        plt.title("Archive 8 Full LOSO Comparison")
        plt.tight_layout()
        plt.savefig(ASSET_DIR / "archive8_full_loso.png", dpi=180)
        plt.close()

        fold8 = full_loso_fold_for("archive_8")
        if not fold8.empty:
            best_candidates = a8_full.sort_values("mae_mean").head(3)["experiment"].tolist()
            fold8 = fold8[fold8["experiment"].isin(best_candidates + ["exp_S2A8F_ctrl_teacher_kinpower"])].copy()
            plt.figure(figsize=(10.8, 4.6))
            sns.barplot(data=fold8, x="fold", y="test_mae", hue="experiment")
            plt.xticks(rotation=35, ha="right")
            plt.ylabel("MAE")
            plt.xlabel("")
            plt.title("Archive 8 Full LOSO Subject-wise MAE")
            plt.tight_layout()
            plt.savefig(ASSET_DIR / "archive8_subjectwise_mae.png", dpi=180)
            plt.close()

    combo_rows = []
    for src, label in [
        (fl, "A6"),
        (a7_full, "A7"),
        (a8_full, "A8"),
    ]:
        if src is None or src.empty:
            continue
        if "experiment" in src.columns:
            for _, r in src.iterrows():
                combo_rows.append({
                    "archive": label,
                    "experiment": r["experiment"],
                    "mae_mean": float(r["mae_mean"]),
                    "lat": float(r["host_latency_ms_mean"]),
                    "size": float(r["model_size_mb_mean"]),
                })
    if combo_rows:
        combo_df = pd.DataFrame(combo_rows).sort_values("mae_mean")
        plt.figure(figsize=(7.2, 4.6))
        sns.scatterplot(data=combo_df, x="size", y="mae_mean", hue="archive", style="archive", s=90)
        plt.xlabel("Model size (MB)")
        plt.ylabel("MAE mean")
        plt.title("Full LOSO Candidate Accuracy-Size Overview")
        plt.tight_layout()
        plt.savefig(ASSET_DIR / "full_loso_candidate_size_mae.png", dpi=180)
        plt.close()

        plt.figure(figsize=(7.2, 4.6))
        sns.scatterplot(data=combo_df, x="lat", y="mae_mean", hue="archive", style="archive", s=90)
        plt.xlabel("Host latency (ms)")
        plt.ylabel("MAE mean")
        plt.title("Full LOSO Candidate Accuracy-Latency Overview")
        plt.tight_layout()
        plt.savefig(ASSET_DIR / "full_loso_candidate_latency_mae.png", dpi=180)
        plt.close()

    log_path = ROOT / "logs" / "Misalign_AS2_A6_eval_4_3122.out"
    if log_path.exists():
        log_lines = log_path.read_text().splitlines()
        fi_sections = {}
        current = None
        values = []
        for line in log_lines:
            if "Processing: exp_S2A6F_ctrl_teacher_kinpower_Test-m2_S008_seed42" in line:
                current = "teacher"
                values = []
                fi_sections[current] = values
            elif "Processing: exp_S2A6F_tiny_kinpower_w100_Test-m2_S008_seed42" in line:
                current = "tiny"
                values = []
                fi_sections[current] = values
            elif current and "Feature " in line and "Importance =" in line:
                parts = line.strip().split("Importance =")
                idx = int(parts[0].split("Feature")[1].split("/")[0].strip())
                val = float(parts[1].strip())
                values.append((idx, val))

        feature_names = [
            "roll", "pitch", "yaw",
            "left_hip_angle", "right_hip_angle",
            "left_torque", "right_torque",
            "left_thigh_angle", "right_thigh_angle",
            "quat_w", "quat_x", "quat_y", "quat_z",
            "left_thigh_angle_dot", "left_torque_dot",
            "right_thigh_angle_dot", "right_torque_dot",
            "mech_power_left", "mech_power_right",
        ]
        for key, items in fi_sections.items():
            if not items:
                continue
            fi_df = pd.DataFrame(
                [{"feature": feature_names[i - 1], "importance": v} for i, v in items]
            ).sort_values("importance", ascending=False).head(10)
            plt.figure(figsize=(6.8, 3.8))
            sns.barplot(data=fi_df, x="importance", y="feature", color="#8338EC")
            plt.xlabel("Permutation importance")
            plt.ylabel("")
            plt.title(f"Top 10 Feature Importance ({key})")
            plt.tight_layout()
            plt.savefig(ASSET_DIR / f"archive6_feature_importance_{key}.png", dpi=180)
            plt.close()

    rep_specs = [
        (ROOT / "experiments/archive_season2/archive_6/exp_S2A6F_ctrl_teacher_kinpower_Test-m2_S008_seed42", ASSET_DIR / "trajectory_a6_teacher.png", "A6 Teacher"),
        (ROOT / "experiments/archive_season2/archive_6/exp_S2A6F_tiny_kinpower_w100_Test-m2_S008_seed42", ASSET_DIR / "trajectory_a6_tiny.png", "A6 Tiny"),
        (ROOT / "experiments/archive_season2/archive_8/exp_S2A8_feat_short_worksplit_teacher_Test-m2_S008_seed42", ASSET_DIR / "trajectory_a8_short_worksplit.png", "A8 Short Worksplit"),
        (ROOT / "experiments/archive_season2/archive_8/exp_S2A8F_feat_norm_worksplit_teacher_Test-m2_S008_seed42", ASSET_DIR / "trajectory_a8f_norm_worksplit.png", "A8F Norm Worksplit"),
    ]
    for exp_dir, out_path, title in rep_specs:
        try:
            if exp_dir.exists():
                make_representative_trajectory_plot(exp_dir, out_path, title)
        except Exception:
            pass



def fmt(v, nd=4):
    if isinstance(v, str):
        return v
    if pd.isna(v):
        return "-"
    return f"{float(v):.{nd}f}"


def img(path: Path, width_cm: float, caption: str, ss, max_height_cm: float = 10.0):
    if not path.exists():
        return []
    out = []
    width = min(width_cm, 14.0) * cm
    max_height = max_height_cm * cm
    im = Image(str(path))
    im._restrictSize(width, max_height)
    im.hAlign = "CENTER"
    out.append(im)
    out.append(para(caption, "KRCaption", ss))
    return out


def table_from_rows(rows: list[list], col_widths=None, font_size=8.0, ss=None):
    cooked = []
    for r_idx, row in enumerate(rows):
        cooked_row = []
        for cell in row:
            text = str(cell).replace("\n", "<br/>")
            if ss is not None:
                style = ss["KRTableHeader"] if r_idx == 0 else ss["KRTableCell"]
                cooked_row.append(Paragraph(text, style))
            else:
                cooked_row.append(cell)
        cooked.append(cooked_row)

    tbl = Table(cooked, colWidths=col_widths, repeatRows=1)
    tbl.setStyle(
        TableStyle(
            [
                ("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#DCE6F1")),
                ("GRID", (0, 0), (-1, -1), 0.3, colors.grey),
                ("VALIGN", (0, 0), (-1, -1), "TOP"),
                ("ALIGN", (0, 0), (-1, 0), "CENTER"),
                ("ROWBACKGROUNDS", (0, 1), (-1, -1), [colors.white, colors.HexColor("#F7F9FC")]),
                ("LEFTPADDING", (0, 0), (-1, -1), 4),
                ("RIGHTPADDING", (0, 0), (-1, -1), 4),
                ("TOPPADDING", (0, 0), (-1, -1), 4),
                ("BOTTOMPADDING", (0, 0), (-1, -1), 4),
            ]
        )
    )
    return tbl


def on_page(canvas, doc):
    canvas.setFont("NanumGothic", 8)
    canvas.setFillColor(colors.HexColor("#666666"))
    canvas.drawRightString(A4[0] - 1.3 * cm, 1.0 * cm, f"{canvas.getPageNumber()}")
    canvas.drawString(1.5 * cm, 1.0 * cm, "260326_Misalign_정리")


def build_report():
    setup_fonts()
    ensure_dirs()
    make_figures()

    ss = styles()
    story = []

    baseline = final_global_baseline()
    a1 = archive_rows("archive_1")
    a2 = archive_rows("archive_2")
    a3 = archive_rows("archive_3")
    a4 = archive_rows("archive_4")
    a5 = archive_rows("archive_5")
    a6_screen = archive_rows("archive_6", prefix="exp_S2A6_")
    fl = final_loso_summary().copy()
    repro = reproducibility_summary().copy()
    repro_seed = reproducibility_by_seed().copy()
    pair_summary = pd.read_csv(ROOT / "compare_result" / "archive_season2" / "archive_6" / "reproducibility" / "paired_mae_summary.csv")

    story.append(Spacer(1, 1.3 * cm))
    story.append(para("260326 Misalign Compensation 정리 보고서", "KRTitle", ss))
    story.append(para("프로젝트 전 기간 실험, 모델 구조, 입력 feature, 전처리, 학습 방법론, 결과, 실시간 배포 가능성을 통합 정리한 내부 보고서", "KRBody", ss))
    story.append(Spacer(1, 0.5 * cm))
    story.append(para("작성 목적", "KRHeading1", ss))
    story.append(
        para(
            "이 보고서는 Misalign_compensation 프로젝트에서 season1부터 season2 archive_6 reproducibility 검증까지 수행한 실험을 한 문서에 모아, "
            "어떤 가설이 성공했고 어떤 가설이 실패했는지, 최종적으로 어떤 모델을 논문 메인 후보로 삼는 것이 맞는지를 정리하기 위해 작성했다. "
            "또한 논문 작성 전에 필요한 수치, 대표 그림, 입력 feature 생성 방식, 모델 경량화와 실시간 추론 가능성까지 동시에 검토할 수 있게 하는 것을 목표로 한다.",
            "KRBody",
            ss,
        )
    )
    story.append(para("문서 구성 계획", "KRHeading2", ss))
    story.append(
        para(
            "1) 문제 정의와 데이터 구조, 2) 입력 feature와 전처리 구현, 3) 모델 구조와 학습 방법론, 4) archive별 가설·대안·결과, "
            "5) 최종 full LOSO 및 seed 재현성 검증, 6) 실시간 추론 가능성 분석, 7) 논문 메시지와 한계 순서로 정리한다.",
            "KRBody",
            ss,
        )
    )

    story.append(PageBreak())
    story.append(para("1. 프로젝트 개요", "KRHeading1", ss))
    story.append(
        para(
            "이 프로젝트의 목표는 hip assistive wearable robot과 사람 hip joint 사이의 misalignment 및 interface compliance로 인해 발생하는 "
            "robot 측정값과 실제 human hip kinematics 사이의 차이를 소프트웨어적으로 보정하는 것이다. "
            "출력은 사람의 좌·우 hip flexion angle이며, 학습은 기본적으로 residual target 방식으로 설정되었다. "
            "즉 mocap 기반 human hip angle에서 pelvis pitch와 robot hip angle(보정 offset 포함)을 뺀 잔차를 예측하고, "
            "평가 시에는 이를 절대 각도로 복원한다.",
            "KRBody",
            ss,
        )
    )
    story.append(
        para(
            f"공정 비교 기준 strongest historical baseline은 {baseline['name']}이며 single-LOSO 기준 MAE {baseline['mae']:.4f}, RMSE {baseline['rmse']:.4f}이다. "
            "이 baseline은 season1 archive_5의 residual + noise 계열 실험에서 나왔다.",
            "KRBody",
            ss,
        )
    )

    story.append(para("2. 데이터셋 및 예측 태스크", "KRHeading1", ss))
    story.append(
        para(
            "학습 데이터는 combined_data_S008.h5이며 최상위 subject는 S001~S008의 8명이다. 조건은 7개(accel_sine, decline_5deg, incline_10deg, "
            "level_075mps, level_100mps, level_125mps, stopandgo), misalignment level은 lv0, lv4, lv7의 3개, 총 trial 수는 169개다. "
            "config에서는 data source prefix를 m2_로 붙여 m2_S001 같은 이름으로 사용하지만, 실제 H5 내부 그룹명은 S001 형식이다.",
            "KRBody",
            ss,
        )
    )
    data_table = [
        ["항목", "값"],
        ["피험자 수", "8"],
        ["조건 수", "7"],
        ["misalignment level", "lv0 / lv4 / lv7"],
        ["총 trial 수", "169"],
        ["샘플링 주파수", "100 Hz"],
        ["출력", "mocap/kin_q의 hip_flexion_l, hip_flexion_r (pelvis 기준 상대각)"],
        ["기본 평가 방식", "single LOSO screening → 최종 후보 full LOSO"],
    ]
    story.append(table_from_rows(data_table, col_widths=[4.2 * cm, 11.5 * cm], font_size=8.5, ss=ss))
    story.append(Spacer(1, 0.2 * cm))
    story.append(
        para(
            "single LOSO 단계에서는 주로 m2_S008을 test subject로 고정해 빠르게 방향성을 보고, 최종 archive_6에서는 8-subject full LOSO로 마무리했다.",
            "KRSmall",
            ss,
        )
    )

    story.append(para("3. 입력 feature와 전처리 구현", "KRHeading1", ss))
    story.append(
        para(
            "입력 feature는 raw sensor를 그대로 쓰는 경우와, 도메인 지식을 반영한 derived feature를 추가하는 경우로 나뉜다. "
            "전처리와 derived feature 생성은 모두 model_training.py 안에서 수행되며, trial-level cache를 사용해 반복 계산을 줄이도록 개선되었다.",
            "KRBody",
            ss,
        )
    )
    raw_table = [
        ["원천 그룹", "채널", "설명"],
        ["robot/back_imu", "accel_x/y/z, gyro_x/y/z, quat_w/x/y/z", "trunk IMU"],
        ["robot/left, robot/right", "hip_angle, thigh_angle, torque", "좌우 로봇 관절/토크"],
        ["mocap/kin_q", "hip_flexion_l/r, pelvis_list", "정답 및 calibration 기준"],
        ["treadmill/left,right", "belt speed", "body-frame / overground accel 유도"],
        ["sub_info", "height, weight", "일부 실험에서 사용 가능하지만 최종 후보에서는 미사용"],
    ]
    story.append(table_from_rows(raw_table, col_widths=[4.0 * cm, 5.8 * cm, 5.9 * cm], font_size=7.8, ss=ss))
    story.append(para("3.1 공통 전처리", "KRHeading2", ss))
    story.append(
        para(
            "- quaternion sign continuity: q와 -q의 동치성 때문에 시간축에서 부호가 튀는 문제를 막기 위해 canonicalization을 수행한다.<br/>"
            "- complementary filter Euler: trunk IMU accel+gyro를 사용해 roll/pitch/yaw를 추정한다.<br/>"
            "- derivative: thigh_angle_dot, torque_dot는 시간 미분 후 30 Hz 4차 저역통과 필터를 적용한다.<br/>"
            "- calibration: robot thigh/hip 계열은 초기 구간에서 mocap thigh 상대각과 평균을 맞춰 offset을 추정한다.<br/>"
            "- residual target: output은 pelvis 기준 hip angle에서 robot hip angle과 calibration offset을 뺀 잔차로 학습하고, 평가 시 절대각으로 복원한다.",
            "KRBody",
            ss,
        )
    )
    story.append(para("3.2 Derived feature 종류", "KRHeading2", ss))
    feat_rows = [
        ["derived/euler", "3", "Complementary filter로 계산한 roll/pitch/yaw", "낮음", "최종 후보에 사용"],
        ["derived/body_frame_imu", "7", "quat→global, gravity 제거, 1.5 s sliding PCA로 heading 추정, treadmill accel 보상", "높음", "archive_1~2"],
        ["derived/gravity_aligned_imu", "7", "초기 standing 구간의 z_up만 고정, yaw는 고정하지 않음", "중간", "archive_3"],
        ["derived/symmetry_heading_imu", "7~8", "gravity-aligned 후 좌우 symmetry signal로 yaw 후보를 online search", "중간", "archive_3"],
        ["derived/deflection", "2", "motor_angle - hip_angle", "낮음", "archive_4~5"],
        ["derived/asymmetry", "3", "좌우 hip/torque/thigh difference", "낮음", "archive_4"],
        ["derived/mech_power", "2", "torque × hip_angle_dot", "낮음", "A5/A6 최종 계열"],
        ["derived/relative_kinematics", "8", "trunk pitch와 robot hip/thigh 및 속도 차이", "낮음", "archive_5"],
        ["derived/compliance_proxy", "8", "deflection, |torque|×|deflection|, torque/(|thighdot|+eps), thigh power", "낮음", "archive_5"],
        ["derived/load_coupling", "6", "torque × (pitch-gap / rate-gap / hip-thigh-gap)", "낮음", "archive_5~6 gating"],
    ]
    story.append(table_from_rows(feat_rows, col_widths=[4.6 * cm, 1.2 * cm, 7.1 * cm, 1.6 * cm, 2.0 * cm], font_size=7.1, ss=ss))
    story.append(
        para(
            "핵심은 final candidate에서 사용된 전처리가 모두 online-friendly라는 점이다. 최종 tiny/teacher 모델은 body-frame이나 symmetry-yaw 같은 무거운 heading 추정 없이, "
            "Euler, quaternion, robot kinematics, torque derivative, mech_power만 사용한다. 이 구성은 100 Hz online 추론을 고려했을 때 현실적인 계산량이다.",
            "KRBody",
            ss,
        )
    )

    story.append(PageBreak())
    story.append(para("4. 모델 구조", "KRHeading1", ss))
    story.append(
        para(
            "기본 backbone은 TCN_MLP이다. TCN encoder는 dilation이 2배씩 증가하는 TemporalBlock을 순차로 쌓고, 마지막 시점 context를 MLP head에 넣어 예측한다. "
            "이후 archive 탐색 과정에서 TCN_GRU, StanceGatedTCN, LoadGatedTCN 같은 변형도 실험했지만 최종적으로 남은 후보는 standard TCN 2종이다.",
            "KRBody",
            ss,
        )
    )
    model_rows = [
        ["모델", "구조", "입력 window", "channels", "kernel", "head", "파라미터", "모델 크기(MB)"],
        ["Teacher", "TCN_MLP", "200", "[32, 64, 64, 128]", "5", "[64]", "224,072", "0.8548"],
        ["Tiny", "TCN_MLP", "100", "[16, 24, 24, 48]", "3", "[32]", "22,720", "0.0867"],
        ["Tiny+LoadGate", "LoadGatedTCN", "100", "[16, 24, 24, 48]", "3", "[32]", "23,597", "0.0900"],
    ]
    story.append(table_from_rows(model_rows, col_widths=[2.4 * cm, 3.1 * cm, 1.9 * cm, 3.6 * cm, 1.2 * cm, 1.4 * cm, 2.3 * cm, 2.0 * cm], font_size=7.6, ss=ss))
    story.append(para("4.1 LoadGatedTCN", "KRHeading2", ss))
    story.append(
        para(
            "LoadGatedTCN은 작은 gating MLP를 추가해 선택된 load-related channel들에서 gate를 계산하고, 이를 이용해 전체 입력을 time-step별로 scaling한다. "
            "직관적으로는 load/compliance가 큰 구간에서 trunk vs robot feature의 신뢰도를 조절하려는 설계였다. "
            "그러나 최종 full LOSO에서는 accuracy와 latency 둘 다 standard tiny보다 열세였다.",
            "KRBody",
            ss,
        )
    )

    story.append(para("5. 학습 및 평가 방법론", "KRHeading1", ss))
    story.append(
        para(
            "공통 학습 설정은 batch 128, epoch 30, learning rate 0.001, weight decay 0.01, AMP, cosine scheduler, early stopping이다. "
            "screening 단계에서는 single LOSO로 빠르게 비교하고, 최종 후보는 full LOSO 8 folds로 검증했다. "
            "평가 시 compare_results.py는 trajectory, condition plot, permutation feature importance, MAE/RMSE/R²/lag를 계산한다.",
            "KRBody",
            ss,
        )
    )
    train_rows = [
        ["항목", "설정"],
        ["optimizer", "AdamW 계열 (lr=0.001, wd=0.01)"],
        ["epochs", "30"],
        ["early stop", "patience=10, warmup=10"],
        ["augmentation", "Gaussian noise std=0.01"],
        ["loss", "기본 회귀 loss + 실험에 따라 smoothness/standing weight 등"],
        ["train stride", "5"],
        ["inference stride", "1"],
        ["metrics", "MAE, RMSE, Huber, R², lag, host latency, model size"],
        ["feature importance", "Permutation importance (compare_results.py)"],
    ]
    story.append(table_from_rows(train_rows, col_widths=[4.3 * cm, 11.4 * cm], font_size=8.1, ss=ss))

    story.append(PageBreak())
    story.append(para("6. Archive별 진행 요약", "KRHeading1", ss))
    story.append(
        para(
            "아래 표는 각 archive가 노린 기여 축, 비교한 대안, 성공한 가설과 실패한 가설을 정리한 것이다. "
            "복잡한 실험 설계에서 바로 결론을 내리지 않고, 성공/실패 가설을 분리한 뒤 최종안으로 압축하는 현재 운영 방식에 맞춰 정리했다.",
            "KRBody",
            ss,
        )
    )
    archive_table = [
        ["Archive", "주요 기여 축", "비교한 대안", "성공한 가설", "실패한 가설", "대표 결과"],
        ["A1", "accuracy / body-frame", "bodyframe core, gait/asym, mech+kin, standing-weight, biomech-fusion", "mech+kin 추가는 core보다 개선", "body-frame 자체가 강한 baseline을 넘지 못함", "S008 기준 bodyframe_mechpower_kinematics 2.2165"],
        ["A2", "accuracy / controlled representation", "quat residual, bodyframe core, bodyframe+quat, longer context, GRU", "quat residual + mech_kin이 강함", "body-frame replacement는 일관되게 열세", "quat_mech_kin 2.0297"],
        ["A3", "accuracy / representation study", "raw IMU, gravity-only, symmetry yaw, PCA body frame, +quat", "raw+quat, gravity+quat는 쓸 만함", "symmetry yaw, PCA heading은 부적절", "ref_euler_quat 2.0336"],
        ["A4", "accuracy / 기존 biomech feature screening", "kinematics, mech_power, deflection, asymmetry, all_biomech", "kinematics가 baseline보다 소폭 개선", "많은 handcrafted feature는 additive gain이 약함", "feat_quat_kinematics 2.0425"],
        ["A5", "accuracy / misalignment-aware domain feature", "relative_kinematics, compliance_proxy, load_coupling, all_misalign", "load_coupling만 부분적으로 유망", "relkin/compliance는 대부분 성능 악화", "ctrl_quat_res_kin_power 2.0229"],
        ["A6", "latency / parameter count / deployability", "teacher, tiny, tiny+loadgate, ultratiny", "tiny 모델이 경량성과 속도 장점 확보", "load gating이 accuracy 우위를 만들지 못함", "full LOSO tiny 2.6344 vs teacher 2.6369(seed42)"],
    ]
    story.append(table_from_rows(archive_table, col_widths=[1.2 * cm, 2.8 * cm, 4.0 * cm, 3.0 * cm, 3.6 * cm, 2.3 * cm], font_size=6.7, ss=ss))
    story.extend(img(ASSET_DIR / "season2_archive_progress.png", 16.0, "그림 1. season2 archive별 최고 single-LOSO MAE 진행", ss))
    story.extend(img(ASSET_DIR / "archive3_representation.png", 16.0, "그림 2. archive_3 representation comparison", ss))
    story.extend(img(ASSET_DIR / "archive4_biomech_screening.png", 16.0, "그림 3. archive_4 biomechanical feature screening", ss))
    story.extend(img(ASSET_DIR / "archive5_misalign_screening.png", 16.0, "그림 4. archive_5 misalignment-aware feature screening", ss))

    story.append(PageBreak())
    story.append(para("7. 최종 후보 및 full LOSO 결과", "KRHeading1", ss))
    story.append(
        para(
            "최종적으로 논문용 full LOSO 검증까지 간 후보는 3개였다. teacher_kinpower는 가장 큰 정확도 기준 모델, tiny_kinpower_w100은 경량 배포 후보, "
            "tiny_loadgate_kinpower_w100_smooth는 domain-aware gating 구조 후보였다.",
            "KRBody",
            ss,
        )
    )
    final_rows = [["모델", "n_folds", "MAE mean", "MAE std", "RMSE mean", "Params", "Size(MB)", "Latency(ms)", "Max freq(Hz)"]]
    label_map = {
        "exp_S2A6F_ctrl_teacher_kinpower": "Teacher",
        "exp_S2A6F_tiny_kinpower_w100": "Tiny",
        "exp_S2A6F_tiny_loadgate_kinpower_w100_smooth": "Tiny+LoadGate",
    }
    for _, r in fl.sort_values(["mae_mean", "rmse_mean"]).iterrows():
        final_rows.append(
            [
                label_map.get(r["experiment"], r["experiment"]),
                str(int(r["n_folds"])),
                fmt(r["mae_mean"], 4),
                fmt(r["mae_std"], 4),
                fmt(r["rmse_mean"], 4),
                f"{int(r['params_mean']):,}",
                fmt(r["model_size_mb_mean"], 4),
                fmt(r["host_latency_ms_mean"], 4),
                fmt(r["host_max_freq_hz_mean"], 1),
            ]
        )
    story.append(table_from_rows(final_rows, col_widths=[3.0 * cm, 1.4 * cm, 2.1 * cm, 2.0 * cm, 2.2 * cm, 2.3 * cm, 1.9 * cm, 2.0 * cm, 2.1 * cm], font_size=7.4, ss=ss))
    story.append(
        para(
            "seed 42 한 번만 보면 Tiny가 Teacher와 거의 동급 또는 약간 우세한 것처럼 보였다. "
            "그러나 재현성 검증을 추가하면 이 결론은 약해진다.",
            "KRBody",
            ss,
        )
    )
    repro_rows = [["모델", "n_runs", "n_seeds", "MAE mean", "MAE std", "RMSE mean", "Latency(ms)", "Size(MB)"]]
    for _, r in repro.sort_values(["mae_mean", "rmse_mean"]).iterrows():
        repro_rows.append(
            [
                "Teacher" if r["model"] == "teacher_kinpower" else "Tiny",
                str(int(r["n_runs"])),
                str(int(r["n_seeds"])),
                fmt(r["mae_mean"], 4),
                fmt(r["mae_std"], 4),
                fmt(r["rmse_mean"], 4),
                fmt(r["host_latency_ms_mean"], 4),
                fmt(r["model_size_mb_mean"], 4),
            ]
        )
    story.append(table_from_rows(repro_rows, col_widths=[3.0 * cm, 1.8 * cm, 1.8 * cm, 2.2 * cm, 2.0 * cm, 2.2 * cm, 2.0 * cm, 2.0 * cm], font_size=7.6, ss=ss))
    p = pair_summary.iloc[0]
    story.append(
        para(
            f"3-seed × 8-subject 총 {int(p['n_pairs'])}쌍 비교에서 Tiny가 Teacher보다 MAE가 낮은 경우는 {int(p['tiny_better_count'])}회, "
            f"Teacher가 더 좋은 경우는 {int(p['teacher_better_count'])}회였다. Tiny−Teacher 평균 MAE 차이는 {float(p['tiny_minus_teacher_mae_mean']):.4f}로, "
            "재현성까지 포함하면 Teacher가 accuracy 기준으로는 여전히 더 안전한 선택이다.",
            "KRBody",
            ss,
        )
    )
    story.extend(img(ASSET_DIR / "archive6_tradeoff_size_mae.png", 14.5, "그림 5. archive_6 full LOSO accuracy-size tradeoff", ss))
    story.extend(img(ASSET_DIR / "archive6_tradeoff_latency_mae.png", 14.5, "그림 6. archive_6 full LOSO accuracy-latency tradeoff", ss))
    story.extend(img(ASSET_DIR / "archive6_seed_mae.png", 14.5, "그림 7. seed별 MAE mean", ss))
    story.extend(img(ASSET_DIR / "archive6_pairwise_mae_diff.png", 15.2, "그림 8. subject×seed pair별 Tiny-Teacher MAE 차이", ss))

    story.append(PageBreak())
    story.append(para("8. 대표 trajectory와 feature importance", "KRHeading1", ss))
    story.append(
        para(
            "최종 candidate의 qualitative plot은 S008 representative evaluation에서 다시 생성했다. "
            "단일 subject S008에서는 Teacher가 MAE 1.8440, Tiny가 MAE 1.9617로 Teacher가 더 좋았지만, 두 모델 모두 stopandgo와 경사 조건에서도 추세를 잘 따라간다.",
            "KRBody",
            ss,
        )
    )
    story.extend(img(ROOT / "compare_result/archive_season2/archive_6/exp_S2A6F_ctrl_teacher_kinpower_Test-m2_S008_seed42/trajectory.png", 15.6, "그림 9. Teacher representative trajectory (S008)", ss))
    story.extend(img(ROOT / "compare_result/archive_season2/archive_6/exp_S2A6F_tiny_kinpower_w100_Test-m2_S008_seed42/trajectory.png", 15.6, "그림 10. Tiny representative trajectory (S008)", ss))
    story.extend(img(ASSET_DIR / "archive6_feature_importance_teacher.png", 14.8, "그림 11. Teacher top-10 feature importance", ss))
    story.extend(img(ASSET_DIR / "archive6_feature_importance_tiny.png", 14.8, "그림 12. Tiny top-10 feature importance", ss))
    fi_table = [
        ["순위", "Teacher 중요 feature", "Tiny 중요 feature"],
        ["1", "left_hip_angle", "left_hip_angle"],
        ["2", "right_hip_angle", "right_hip_angle"],
        ["3", "right_thigh_angle", "right_torque"],
        ["4", "right_torque", "left_thigh_angle"],
        ["5", "left_thigh_angle", "left_torque"],
        ["6", "left_torque", "right_thigh_angle"],
        ["7", "left_thigh_angle_dot", "left_thigh_angle_dot"],
        ["8", "right_thigh_angle_dot", "right_thigh_angle_dot"],
    ]
    story.append(table_from_rows(fi_table, col_widths=[1.3 * cm, 7.0 * cm, 7.0 * cm], font_size=8.0, ss=ss))
    story.append(
        para(
            "feature importance 결과는 두 모델 모두 robot hip angle, robot torque, thigh angle, thigh angle derivative에 가장 크게 의존함을 보여준다. "
            "즉 최종 성능은 복잡한 body-frame 정렬보다 robot-human interface의 직접 관찰량과 그 시간변화율, 그리고 mech_power 같은 경량 derived feature에 의해 좌우된다.",
            "KRBody",
            ss,
        )
    )

    story.append(PageBreak())
    story.append(para("9. 실시간 추론 가능성 및 계산 부담 분석", "KRHeading1", ss))
    story.append(
        para(
            "최종 논문에서 strong contribution을 accuracy-only가 아니라 deployable system으로 잡으려면, 전처리와 모델 추론이 실제 online loop에서 감당 가능한지 명확히 보여줘야 한다. "
            "현재 결과 기준으로 final Tiny는 그 조건을 만족한다.",
            "KRBody",
            ss,
        )
    )
    rt_rows = [
        ["구성 요소", "최종 Tiny 기준 구현", "복잡도 / 부담", "실시간성 판단"],
        ["Euler 추정", "Complementary filter (acc+gyro)", "sample당 O(1)", "충분히 가능"],
        ["Quaternion 사용", "raw quat_wxyz 직접 입력", "추가 계산 거의 없음", "충분히 가능"],
        ["Derivative", "thigh_angle_dot, torque_dot", "sample당 차분 1회 + 저역통과", "가능"],
        ["mech_power", "torque × hip_angle_dot", "sample당 곱셈 2회", "가능"],
        ["Body-frame/PCA heading", "최종 후보에서 미사용", "window PCA와 고유값 분해 필요", "기본 후보 제외가 타당"],
        ["Tiny inference", "window=100 TCN", "2.0984 ms host latency", "100 Hz에 충분"],
    ]
    story.append(table_from_rows(rt_rows, col_widths=[3.0 * cm, 5.0 * cm, 4.0 * cm, 3.0 * cm], font_size=7.3, ss=ss))
    story.append(
        para(
            "100 Hz 제어 루프를 가정하면 주기당 예산은 10 ms이다. Tiny의 host latency는 약 2.10 ms이므로 모델 추론만 놓고 보면 예산의 약 21% 수준이다. "
            "전처리도 complementary filter, finite difference, mech_power처럼 O(1) 연산 위주이기 때문에 전체 파이프라인은 online 배치가 아니라 sample-by-sample 상태 갱신 형태로 구현 가능하다. "
            "반면 body-frame PCA heading은 window 누적 통계와 eigen decomposition이 들어가므로, 본 프로젝트의 실시간 후보에서 제외한 판단이 타당하다.",
            "KRBody",
            ss,
        )
    )
    story.append(
        para(
            "sensor reduction 관점의 새로운 contribution은 이번 실험에서 확보하지 못했다. 최종 Tiny 역시 trunk IMU quat/euler와 robot 좌우 hip_angle, thigh_angle, torque를 그대로 사용한다. "
            "따라서 논문에서는 sensor reduction보다는 deployability와 accuracy-efficiency tradeoff를 강조하는 편이 맞다.",
            "KRBody",
            ss,
        )
    )

    story.append(PageBreak())
    story.append(para("10. 최종 정리: 무엇이 성공했고 무엇이 실패했는가", "KRHeading1", ss))
    story.append(para("성공한 가설", "KRHeading2", ss))
    story.append(
        para(
            "- orientation-preserving 입력(euler + quat + robot kinematics)은 끝까지 강했다.<br/>"
            "- mech_power와 kinematic derivative는 가벼운 계산량 대비 실용적인 이득을 주었다.<br/>"
            "- 큰 teacher 모델이 아니어도 tiny causal TCN이 상당한 정확도를 유지할 수 있었다.<br/>"
            "- 최종 Tiny는 파라미터와 모델 크기를 약 90% 줄이고도 100 Hz online 적용 가능성을 확보했다.",
            "KRBody",
            ss,
        )
    )
    story.append(para("실패한 가설", "KRHeading2", ss))
    story.append(
        para(
            "- body-frame이나 heading normalization이 이 문제의 핵심 해결책일 것이라는 가설은 지지되지 않았다.<br/>"
            "- symmetry-yaw, PCA body-frame, load-aware gating은 최종 accuracy winner가 되지 못했다.<br/>"
            "- handcrafted biomechanical feature를 많이 넣으면 strong baseline을 넘을 것이라는 가설도 지지되지 않았다.<br/>"
            "- misalignment-specific feature 중 일부(load_coupling)는 부분적으로 가능성이 있었지만, 전체적으로 additive gain은 약했다.",
            "KRBody",
            ss,
        )
    )
    story.append(para("논문 메인 메시지 제안", "KRHeading2", ss))
    story.append(
        para(
            "현재 결과에 가장 잘 맞는 논문 메시지는 다음과 같다. "
            "첫째, hip misalignment compensation은 aggressive frame normalization보다 orientation-preserving sensor fusion이 더 안정적이다. "
            "둘째, biomechanics-informed tiny causal TCN은 large teacher 대비 약 2% 수준의 정확도 손해로 10배 작은 모델 크기와 더 나은 deployability를 제공한다. "
            "셋째, 따라서 본 프로젝트의 strongest contribution은 new body-frame proposal이 아니라 real-time deployable misalignment compensation system이다.",
            "KRBody",
            ss,
        )
    )

    story.append(para("11. 논문 작성 권고안", "KRHeading1", ss))
    paper_rows = [
        ["항목", "권고안"],
        ["메인 baseline", "Teacher kinpower"],
        ["proposed model", "Tiny kinpower w100"],
        ["메인 contribution 축", "accuracy-efficiency tradeoff, deployability, online feasibility"],
        ["보조 contribution", "body-frame / normalization negative results, representation study"],
        ["메인 표", "Teacher vs Tiny vs Tiny+LoadGate full LOSO mean±std"],
        ["메인 그림", "tradeoff plot, representative trajectory, feature importance, paired seed plot"],
        ["supplementary", "archive_3/4/5 ablation, 실패한 feature family 비교"],
    ]
    story.append(table_from_rows(paper_rows, col_widths=[4.3 * cm, 11.3 * cm], font_size=8.1, ss=ss))
    story.append(
        para(
            "정리하면, 본 프로젝트는 정확도 absolute winner를 새로 만든 것은 아니지만, wearable robot misalignment compensation 문제에서 "
            "어떤 입력 representation과 어떤 경량 구조가 실제로 살아남는지 충분히 정리했다. "
            "특히 final Tiny는 논문 메인 모델로 사용할 가치가 있다.",
            "KRBody",
            ss,
        )
    )

    story.append(para("12. 논문화 시 가져갈 수 있는 contribution", "KRHeading1", ss))
    story.append(
        para(
            "논문 contribution은 결과가 실제로 지지하는 주장 위에만 세워야 한다. 현재 프로젝트 결과를 기준으로 보면, strong contribution과 weak contribution은 분리해서 서술하는 것이 맞다.",
            "KRBody",
            ss,
        )
    )
    story.append(para("12.1 강하게 주장 가능한 contribution", "KRHeading2", ss))
    contrib_rows = [
        ["기여 유형", "현재 결과가 지지하는 내용", "근거"],
        ["문제 정식화", "hip exoskeleton misalignment/compliance 보정을 software-only residual correction 문제로 정식화", "robot hip/thigh/torque + trunk IMU만으로 human hip angle 보정 수행"],
        ["representation insight", "aggressive heading normalization보다 orientation-preserving representation이 더 안정적임을 실험적으로 보임", "archive_2~3에서 body-frame, PCA heading, symmetry yaw가 strong baseline을 넘지 못함"],
        ["accuracy-efficiency tradeoff", "biomechanics-informed tiny causal TCN이 teacher 대비 약 2% 수준 정확도 손해로 약 90% 모델 축소 달성", "archive_6 final_loso + reproducibility 결과"],
        ["deployability", "전처리와 모델 추론 모두 100 Hz online 추론을 목표로 유지 가능한 계산량으로 구성", "final 후보는 Euler, derivative, mech_power, TCN 위주로 구성되고 body-frame PCA는 제외"],
    ]
    story.append(table_from_rows(contrib_rows, col_widths=[3.0 * cm, 7.1 * cm, 6.0 * cm], font_size=7.3, ss=ss))
    story.append(
        para(
            "위 네 가지 중 논문 메인 contribution으로 가장 적합한 것은 세 번째와 네 번째다. "
            "즉 본 연구의 핵심은 새로 복잡한 feature를 더 넣어서 최고 정확도를 만든 것이 아니라, 실제 배포 가능한 경량 모델이 큰 teacher와 비교해 어느 정도 정확도를 유지하는지, "
            "그리고 어떤 representation 설계가 misalignment compensation에 유리한지를 구조적으로 보여준 점에 있다.",
            "KRBody",
            ss,
        )
    )
    story.append(para("12.2 약하게만 주장해야 하는 contribution", "KRHeading2", ss))
    weak_rows = [
        ["주장", "왜 약한가"],
        ["새 body-frame 방법이 성능을 개선한다", "archive_2~3에서 일관되게 지지되지 않음"],
        ["misalignment-aware handcrafted feature가 baseline을 확실히 넘는다", "archive_4~5에서 additive gain이 약하고 불안정함"],
        ["load-aware gating이 최종 해법이다", "archive_6 full LOSO와 seed 재현성에서 Tiny/Teacher보다 열세"],
        ["Tiny가 Teacher와 완전히 동급 정확도다", "seed 42에서는 비슷했지만 3-seed 재현성에선 Teacher 우세"],
    ]
    story.append(table_from_rows(weak_rows, col_widths=[6.0 * cm, 10.1 * cm], font_size=7.8, ss=ss))
    story.append(para("12.3 논문용 contribution 문장 초안", "KRHeading2", ss))
    story.append(
        para(
            "아래 문장들은 현재 결과를 과장하지 않으면서도 논문화 시 직접 사용할 수 있는 수준의 contribution 초안이다.",
            "KRBody",
            ss,
        )
    )
    story.append(
        para(
            "1) We formulate hip-angle misalignment compensation in a wearable robot as a software-only residual correction problem using robot-mounted sensing and trunk IMU.<br/>"
            "2) We provide a controlled representation study showing that orientation-preserving sensor fusion is more reliable than aggressive heading-normalized body-frame representations for treadmill walking.<br/>"
            "3) We show that a biomechanics-informed tiny causal TCN preserves most of the accuracy of a larger teacher model while reducing parameter count and model size by about 90%, enabling practical real-time deployment.",
            "KRBody",
            ss,
        )
    )
    story.append(para("12.4 추천 논문 스토리", "KRHeading2", ss))
    story.append(
        para(
            "논문 메인 스토리는 다음처럼 가져가는 것이 가장 자연스럽다. "
            "첫째, 기존 하드웨어를 바꾸지 않고도 software-only compensation으로 misalignment 문제를 줄일 수 있다. "
            "둘째, 이 문제에서는 body-frame 정규화가 핵심이 아니라 orientation-preserving fusion과 lightweight temporal modeling이 더 중요하다. "
            "셋째, 최종 tiny 모델은 absolute best accuracy를 갱신하지는 못했지만, 실시간 배포 관점에서 훨씬 설득력 있는 accuracy-efficiency tradeoff를 제공한다. "
            "즉 이 논문은 새로운 복잡한 센서 융합 알고리즘 제안보다, deployable misalignment compensation system과 그 설계 원칙을 제시하는 논문으로 쓰는 것이 맞다.",
            "KRBody",
            ss,
        )
    )

    doc = SimpleDocTemplate(
        str(OUTPUT_PDF),
        pagesize=A4,
        leftMargin=1.5 * cm,
        rightMargin=1.5 * cm,
        topMargin=1.5 * cm,
        bottomMargin=1.5 * cm,
        title="260326_Misalign_정리",
        author="OpenAI Codex",
    )
    doc.build(story, onFirstPage=on_page, onLaterPages=on_page)


def latex_escape(text: str) -> str:
    s = str(text)
    replacements = {
        "\\": r"\textbackslash{}",
        "&": r"\&",
        "%": r"\%",
        "$": r"\$",
        "#": r"\#",
        "_": r"\_",
        "{": r"\{",
        "}": r"\}",
        "~": r"\textasciitilde{}",
        "^": r"\textasciicircum{}",
    }
    for src, dst in replacements.items():
        s = s.replace(src, dst)
    return s


def latex_par(text: str) -> str:
    return latex_escape(text).replace("\n", "\n\n")


def latex_table(headers, rows, colspec: str, caption: str, size: str = r"\small") -> str:
    lines = [
        r"\begin{table}[H]",
        r"\centering",
        size,
        r"\renewcommand{\arraystretch}{1.18}",
        rf"\begin{{tabular}}{{{colspec}}}",
        r"\toprule",
        " & ".join(latex_escape(h) for h in headers) + r" \\",
        r"\midrule",
    ]
    for row in rows:
        lines.append(" & ".join(latex_escape(c) for c in row) + r" \\")
    lines.extend(
        [
            r"\bottomrule",
            r"\end{tabular}",
            rf"\caption{{{latex_escape(caption)}}}",
            r"\end{table}",
        ]
    )
    return "\n".join(lines)


def latex_longtable(headers, rows, colspec: str, caption: str, size: str = r"\scriptsize") -> str:
    lines = [
        size,
        r"\renewcommand{\arraystretch}{1.14}",
        rf"\begin{{longtable}}{{{colspec}}}",
        rf"\caption{{{latex_escape(caption)}}}\\",
        r"\toprule",
        " & ".join(latex_escape(h) for h in headers) + r" \\",
        r"\midrule",
        r"\endfirsthead",
        r"\toprule",
        " & ".join(latex_escape(h) for h in headers) + r" \\",
        r"\midrule",
        r"\endhead",
        r"\bottomrule",
        r"\endfoot",
    ]
    for row in rows:
        lines.append(" & ".join(latex_escape(c) for c in row) + r" \\")
    lines.append(r"\end{longtable}")
    return "\n".join(lines)


def latex_figure(rel_path: str, caption: str, width: str = "0.92\\linewidth") -> str:
    if rel_path.startswith("assets/"):
        asset_path = ASSET_DIR / Path(rel_path).name
        if not asset_path.exists():
            return ""
    return "\n".join(
        [
            r"\begin{figure}[H]",
            r"\centering",
            rf"\includegraphics[width={width}]{{{rel_path}}}",
            rf"\caption{{{latex_escape(caption)}}}",
            r"\end{figure}",
        ]
    )


def prepare_latex_assets() -> None:
    if LATEX_BUILD_DIR.exists():
        shutil.rmtree(LATEX_BUILD_DIR)
    LATEX_BUILD_DIR.mkdir(parents=True, exist_ok=True)

    links = {
        "assets": ASSET_DIR,
        "teacher_plot": ROOT / "compare_result/archive_season2/archive_6/exp_S2A6F_ctrl_teacher_kinpower_Test-m2_S008_seed42",
        "tiny_plot": ROOT / "compare_result/archive_season2/archive_6/exp_S2A6F_tiny_kinpower_w100_Test-m2_S008_seed42",
    }
    for name, target in links.items():
        link = LATEX_BUILD_DIR / name
        if link.exists() or link.is_symlink():
            link.unlink()
        link.symlink_to(target)


def write_and_compile_latex(tex: str) -> None:
    prepare_latex_assets()
    tex_path = LATEX_BUILD_DIR / "report.tex"
    tex_path.write_text(tex)
    cmd = ["xelatex", "-interaction=nonstopmode", "-halt-on-error", "report.tex"]
    for _ in range(2):
        result = subprocess.run(cmd, cwd=LATEX_BUILD_DIR, capture_output=True, text=True)
        if result.returncode != 0:
            raise RuntimeError(
                "XeLaTeX build failed\nSTDOUT:\n"
                + result.stdout
                + "\nSTDERR:\n"
                + result.stderr
            )
    shutil.copy2(LATEX_BUILD_DIR / "report.pdf", OUTPUT_PDF)


def build_report_latex():
    ensure_dirs()
    make_figures()

    baseline = final_global_baseline()
    global_hist = historical_global_summary()
    a1 = archive_rows("archive_1")
    a2 = archive_rows("archive_2")
    a3 = archive_rows("archive_3", prefix="exp_S2A3_")
    a4 = archive_rows("archive_4", prefix="exp_S2A4_")
    a5 = archive_rows("archive_5", prefix="exp_S2A5_")
    a6_screen = archive_rows("archive_6", prefix="exp_S2A6_")
    a7 = archive_rows("archive_7", prefix="exp_S2A7_")
    a8 = archive_rows("archive_8", prefix="exp_S2A8_")
    s3 = aggregate_experiment_metrics(ROOT / "experiments" / "archive_season3" / "archive_1")
    fl6 = final_loso_summary().copy()
    fl7 = full_loso_summary_for("archive_7")
    fl8 = full_loso_summary_for("archive_8")
    folds8 = full_loso_fold_for("archive_8")
    repro = reproducibility_summary().copy()
    repro_seed = reproducibility_by_seed().copy()
    pair_summary = pd.read_csv(ROOT / "compare_result" / "archive_season2" / "archive_6" / "reproducibility" / "paired_mae_summary.csv")
    pair_row = pair_summary.iloc[0]

    archive_best_rows = [
        ("A1", safe_best(a1), "body-frame + biomech screening"),
        ("A2", safe_best(a2), "controlled representation / architecture"),
        ("A3", safe_best(a3), "representation study"),
        ("A4", safe_best(a4), "biomechanical feature screening"),
        ("A5", safe_best(a5), "misalignment-aware feature screening"),
        ("A6(screen)", safe_best(a6_screen), "deployable model screening"),
        ("A7(screen)", safe_best(a7), "physics feature screening"),
        ("A8(screen)", safe_best(a8), "physics feature re-screening"),
    ]

    def archive_inventory_rows(df: pd.DataFrame):
        rows = []
        if df is None or df.empty:
            return rows
        for _, r in df.sort_values(["mae", "rmse"]).iterrows():
            rows.append([
                pretty_name(r["name"]),
                fmt(r["mae"], 4),
                fmt(r["rmse"], 4),
                f"{int(r['params']):,}" if not pd.isna(r["params"]) else "-",
                fmt(r["model_size_mb"], 4),
                fmt(r["host_latency_ms"], 4),
                fmt(r["host_max_freq_hz"], 1),
            ])
        return rows

    def full_inventory_rows(df: pd.DataFrame):
        rows = []
        if df is None or df.empty:
            return rows
        for _, r in df.sort_values(["mae_mean", "rmse_mean"]).iterrows():
            rows.append([
                pretty_name(r["experiment"]),
                fmt(r["mae_mean"], 4),
                fmt(r["mae_std"], 4),
                fmt(r["rmse_mean"], 4),
                f"{int(r['params_mean']):,}" if not pd.isna(r["params_mean"]) else "-",
                fmt(r["model_size_mb_mean"], 4),
                fmt(r["host_latency_ms_mean"], 4),
                fmt(r["host_max_freq_hz_mean"], 1),
            ])
        return rows

    headers_final = ["모델", "n_folds", "MAE mean", "MAE std", "RMSE mean", "Params", "Size(MB)", "Latency(ms)", "Max freq(Hz)"]
    rows_final = []
    label_map = {
        "exp_S2A6F_ctrl_teacher_kinpower": "Teacher",
        "exp_S2A6F_tiny_kinpower_w100": "Tiny",
        "exp_S2A6F_tiny_loadgate_kinpower_w100_smooth": "Tiny+LoadGate",
    }
    for _, r in fl6.sort_values(["mae_mean", "rmse_mean"]).iterrows():
        rows_final.append([
            label_map.get(r["experiment"], pretty_name(r["experiment"])),
            str(int(r["n_folds"])),
            fmt(r["mae_mean"], 4),
            fmt(r["mae_std"], 4),
            fmt(r["rmse_mean"], 4),
            f"{int(r['params_mean']):,}",
            fmt(r["model_size_mb_mean"], 4),
            fmt(r["host_latency_ms_mean"], 4),
            fmt(r["host_max_freq_hz_mean"], 1),
        ])

    headers_repro = ["모델", "n_runs", "n_seeds", "MAE mean", "MAE std", "RMSE mean", "Latency(ms)", "Size(MB)"]
    rows_repro = []
    for _, r in repro.sort_values(["mae_mean", "rmse_mean"]).iterrows():
        rows_repro.append([
            "Teacher" if r["model"] == "teacher_kinpower" else "Tiny",
            str(int(r["n_runs"])),
            str(int(r["n_seeds"])),
            fmt(r["mae_mean"], 4),
            fmt(r["mae_std"], 4),
            fmt(r["rmse_mean"], 4),
            fmt(r["host_latency_ms_mean"], 4),
            fmt(r["model_size_mb_mean"], 4),
        ])

    a8_compare_rows = []
    if not fl8.empty:
        for _, r in fl8.sort_values(["mae_mean", "rmse_mean"]).iterrows():
            rel = ""
            if r["experiment"] != "exp_S2A8F_ctrl_teacher_kinpower":
                base = fl8[fl8["experiment"] == "exp_S2A8F_ctrl_teacher_kinpower"].iloc[0]
                rel = f"{(float(r['mae_mean']) - float(base['mae_mean'])):+.4f}"
            a8_compare_rows.append([
                pretty_name(r["experiment"]),
                fmt(r["mae_mean"], 4),
                fmt(r["mae_std"], 4),
                fmt(r["rmse_mean"], 4),
                fmt(r["host_latency_ms_mean"], 4),
                rel if rel else "baseline",
            ])

    a8_subject_rows = []
    if not folds8.empty:
        pivot = folds8.pivot(index="fold", columns="experiment", values="test_mae")
        best_cols = [c for c in [
            "exp_S2A8F_ctrl_teacher_kinpower",
            "exp_S2A8F_feat_norm_worksplit_teacher",
            "exp_S2A8F_feat_trunk_consistency_teacher",
            "exp_S2A8F_feat_short_worksplit_teacher",
        ] if c in pivot.columns]
        for fold in pivot.index:
            row = [fold]
            for col in best_cols:
                row.append(fmt(pivot.loc[fold, col], 4))
            a8_subject_rows.append(row)

    archive_summary_rows = [[
        "Archive", "기여 축", "최고 실험", "MAE", "RMSE", "핵심 해석"
    ]]
    archive_explanations = {
        "A1": "body-frame 단독은 약했고 mech+kin 보조가 가장 유의미",
        "A2": "controlled 비교에서도 body-frame replacement는 실패",
        "A3": "orientation-preserving representation이 더 안정적",
        "A4": "기존 biomech feature 중 derivative 계열만 약하게 유효",
        "A5": "load_coupling만 부분적으로 가능성, 나머지는 불안정",
        "A6(screen)": "deployability 중심으로 tiny가 최종 후보군에 진입",
        "A7(screen)": "physics feature가 single-LOSO에서 강한 신호를 보임",
        "A8(screen)": "physics feature를 재설계했더니 S008에서는 baseline을 확실히 이김",
    }
    for label, best, axis in archive_best_rows:
        archive_summary_rows.append([
            label,
            axis,
            pretty_name(best["name"]),
            fmt(best["mae"], 4),
            fmt(best["rmse"], 4),
            archive_explanations[label],
        ])

    hist_top_rows = []
    for rank, (_, r) in enumerate(global_hist.head(25).iterrows(), start=1):
        hist_top_rows.append([
            str(rank),
            pretty_name(r["group"]),
            fmt(r["mae"], 4),
            fmt(r["rmse"], 4),
            fmt(r["r2"], 4),
            fmt(r["lag_ms"], 2),
            "공유 hip-angle target",
        ])

    historical_model_rows = []
    for _, r in global_hist.iterrows():
        historical_model_rows.append([
            pretty_name(r["group"]),
            fmt(r["mae"], 4),
            fmt(r["rmse"], 4),
            fmt(r["r2"], 4),
            fmt(r["lag_ms"], 2),
            fmt(r["mae_pct_change"], 2),
        ])

    strong_model_rows = [
        [
            "A5_res_noise_001",
            "TCN residual baseline",
            "Euler + quat + robot hip/thigh/torque",
            "window 200, channels [32,64,64,128], kernel 5",
            "historical single-LOSO strongest",
        ],
        [
            "A6 Teacher",
            "TCN_MLP teacher",
            "Euler + quat + robot hip/thigh/torque + derivatives + mech_power",
            "window 200, channels [32,64,64,128], kernel 5",
            "full-LOSO accuracy anchor",
        ],
        [
            "A6 Tiny",
            "TCN_MLP tiny",
            "same as teacher",
            "window 100, channels [16,24,24,48], kernel 3",
            "full-LOSO best deployable model",
        ],
        [
            "A8 Norm Worksplit Teacher",
            "teacher + physics feature",
            "teacher input + normalized interface worksplit",
            "teacher backbone + interface_workloop 6ch",
            "physics-feature full-LOSO best",
        ],
        [
            "S3 Factorized Tiny Full",
            "FactorizedBranchTCN tiny",
            "raw/orientation branch + geometry/compliance/hysteresis branches",
            "window 100, factorized branches, channels [12,16,16,24]",
            "season3 human-thigh target best",
        ],
    ]

    season4_candidate_rows = [
        [
            "A6 Tiny kinpower",
            "accuracy-efficiency / deployability",
            "full LOSO 2.6344, 0.0867 MB, 2.25 ms",
            "season4 기본 anchor 후보",
        ],
        [
            "A6 Teacher kinpower",
            "accuracy anchor",
            "full LOSO 2.6369, strongest robust reference",
            "새 실험은 모두 이 모델 대비 설명",
        ],
        [
            "A8 norm worksplit / trunk consistency",
            "domain-knowledge / interpretability",
            "physics feature가 internal baseline을 소폭 개선",
            "단일 family만 좁게 채택, large concat 금지",
        ],
        [
            "S3 factorized tiny full",
            "structure / target reformulation",
            "human-thigh target에서 가장 나았지만 절대 성능은 낮음",
            "season4에서 raw branch 유지형 factorization만 검토",
        ],
    ]

    glossary_rows = [
        ["용어", "정의", "보고서에서의 의미"],
        ["Robot angle", "로봇 센서가 직접 측정한 hip / thigh 관련 관절각", "입력의 기본 뼈대"],
        ["Calibrated robot angle", "offset/스케일 보정 후 robot angle", "사람 hip angle에 더 가깝게 맞춘 기준선"],
        ["Ground-truth human hip angle", "mocap/kin_q에서 얻은 실제 hip flexion", "최종 supervision"],
        ["Residual target", "ground truth - calibrated robot reference", "모델이 직접 예측하는 보정량"],
        ["Single LOSO", "한 subject만 test로 두는 screening setup", "빠른 가설 검증용"],
        ["Full LOSO", "8 subjects 전체를 각각 test로 두는 최종 검증", "논문용 일반화 평가"],
        ["Teacher", "큰 TCN_MLP baseline", "정확도 기준 strong reference"],
        ["Tiny", "경량 TCN_MLP baseline", "deployable reference"],
        ["Physics feature", "worksplit, load_coupling, trunk_consistency 등", "물리 해석 가능한 derived feature family"],
    ]

    protocol_rows = [
        ["구분", "Single LOSO screening", "Full LOSO validation"],
        ["목적", "가설 방향성 빠르게 확인", "subject generalization 최종 검증"],
        ["test subject", "주로 m2_S008", "m2_S001 ~ m2_S008 전체"],
        ["validation", "고정된 1개 subject", "각 fold마다 test 제외 나머지 중 val 설정"],
        ["해석 방식", "상대 비교와 가설 선별", "논문 메인 수치와 평균±표준편차"],
        ["위험", "subject-specific gain 착시", "계산량 증가"],
    ]

    feature_dictionary_rows = [
        ["입력/출력", "채널 예시", "계산식/생성 방법", "왜 썼는지"],
        ["quat_wxyz", "quat_w, quat_x, quat_y, quat_z", "raw quaternion", "절대 orientation 보존"],
        ["derived/euler", "roll, pitch, yaw", "complementary filter", "직관적 orientation state"],
        ["robot hip/thigh/torque", "left/right hip_angle, thigh_angle, torque", "raw sensor", "misalignment 보정의 기본 관측량"],
        ["thigh_angle_dot", "left/right thigh angular velocity", "finite difference + smoothing", "동역학 변화율 반영"],
        ["torque_dot", "left/right torque derivative", "finite difference + smoothing", "부하 변화율 반영"],
        ["mech_power", "left/right mechanical power", "torque × hip_angle_dot", "일/에너지 흐름 반영"],
        ["load_coupling", "torque-weighted gap features", "torque × (pitch-gap / hip-thigh-gap)", "부하와 mismatch의 결합"],
        ["interface_workloop", "W+, W-, Wnet", "rolling positive/negative work split", "에너지 손실과 hysteresis 요약"],
        ["trunk_consistency", "gyro-vs-kinematics residual", "trunk orientation과 robot motion의 불일치", "sensor fusion consistency 반영"],
        ["effective_impedance", "k, c, b, pred_err", "online RLS", "인터페이스 동역학 상태 근사"],
        ["target", "left/right hip residual", "GT - calibrated robot angle", "직접 각도 대신 보정량 예측"],
    ]

    archive_hypothesis_rows = {
        "A1-A3": [
            ["가설", "더 물리적으로 정렬된 body-frame / heading-normalized IMU가 raw/orientation-preserving 입력보다 좋을 것이다"],
            ["실험 설계", "body-frame, gravity-only, symmetry-yaw, hybrid, context 확장 등 비교"],
            ["성공/실패", "body-frame 단독 대체는 실패, raw+quat / euler+quat가 더 안정적"],
            ["판단", "이 문제는 aggressive normalization보다 orientation 정보 보존이 중요"],
        ],
        "A4-A5": [
            ["가설", "biomechanical / misalignment-aware handcrafted feature를 strong baseline 위에 얹으면 additive gain이 날 것이다"],
            ["실험 설계", "kinematics, mech_power, asymmetry, deflection, load_coupling, relkin, compliance proxy 비교"],
            ["성공/실패", "derivative와 일부 load_coupling만 부분 성공, 대량 concat은 반복적으로 실패"],
            ["판단", "feature family는 선별적으로 써야 하고, 많이 합친다고 좋아지지 않음"],
        ],
        "A6": [
            ["가설", "정확도를 크게 잃지 않으면서도 훨씬 가벼운 deployable 모델을 만들 수 있다"],
            ["실험 설계", "teacher / tiny / loadgate 비교, full LOSO + 3-seed 재현성"],
            ["성공/실패", "tiny는 teacher와 거의 비슷한 정확도로 10배 작은 모델을 달성, loadgate는 최종 winner 실패"],
            ["판단", "deployability 관점의 strongest story는 tiny causal TCN"],
        ],
        "A7-A8": [
            ["가설", "physics feature를 제대로 설계하면 최종 baseline을 넘길 수 있다"],
            ["실험 설계", "worksplit, hysteresis, impedance, lag, trunk consistency를 screening 후 full LOSO 재검증"],
            ["성공/실패", "single LOSO에서는 강한 gain, full LOSO에서는 internal baseline 소폭 개선까지만 확인"],
            ["판단", "physics feature는 가능성이 있으나 strongest project-wide baseline까지는 아직 못 넘음"],
        ],
    }

    tex_parts = [
        r"\documentclass[11pt,a4paper]{article}",
        r"\usepackage[margin=1in]{geometry}",
        r"\usepackage{graphicx}",
        r"\usepackage{booktabs}",
        r"\usepackage{longtable}",
        r"\usepackage{float}",
        r"\usepackage{hyperref}",
        r"\usepackage{amsmath}",
        r"\usepackage{array}",
        r"\usepackage{fontspec}",
        r"\hypersetup{colorlinks=true, linkcolor=blue, urlcolor=blue}",
        r"\setlength{\parskip}{0.45em}",
        r"\setlength{\parindent}{0pt}",
        r"\setlength{\emergencystretch}{3em}",
        r"\sloppy",
        r"\renewcommand{\arraystretch}{1.18}",
        r"\setmainfont[Path=/home/chanyoungko/.local/lib/python3.13/site-packages/koreanize_matplotlib/fonts/,UprightFont=NanumGothic.ttf,BoldFont=NanumGothicBold.ttf]{NanumGothic}",
        r"\graphicspath{{assets/}}",
        r"\begin{document}",
        r"\begin{titlepage}",
        r"\centering",
        r"{\LARGE 260326\_Misalign\_정리\par}",
        r"\vspace{0.8cm}",
        r"{\Large Misalign\_compensation 전체 실험 종합 보고서\par}",
        r"\vspace{0.8cm}",
        rf"{{\large Repository: {latex_escape(str(ROOT))}\par}}",
        r"\vspace{0.4cm}",
        r"{\large Updated: 2026-04-02\par}",
        r"\vfill",
        rf"{{\large {latex_escape('이 문서는 Misalign_compensation 프로젝트에서 수행한 season1/season2 실험 전체를 다시 정리한 확장판 보고서이다. 데이터 구조, 전처리, 입력 feature 생성, 모델 구조, 학습 전략, archive별 가설과 결과, full LOSO 검증, 재현성, 실시간 추론 가능성, 논문화 시 contribution까지 한 번에 파악할 수 있도록 구성했다.')} \par}}",
        r"\vfill",
        r"\end{titlepage}",
        r"\tableofcontents",
        r"\clearpage",
        r"\section{요약}",
        latex_par(
            "이 프로젝트는 wearable hip exoskeleton에서 발생하는 human--robot misalignment/compliance로 인해 robot이 측정한 hip/thigh 신호와 실제 human hip angle 사이에 생기는 체계적 오차를 소프트웨어적으로 보정하는 문제를 다룬다. "
            "핵심은 단순 regression이 아니라 `robot이 이미 알고 있는 것`과 `mocap이 알려주는 실제 값` 사이의 residual을 학습하는 것이다."
        ),
        latex_par(
            "지금까지의 전체 흐름을 한 문장으로 요약하면 다음과 같다. 초반에는 body-frame과 heading normalization 같은 '더 물리적인 좌표계'를 탐색했지만, 중반 이후 결과는 orientation-preserving sensor fusion과 경량 biomechanical feature가 더 안정적이라는 쪽으로 수렴했다. "
            "후반에는 accuracy breakthrough보다 deployability와 full LOSO generalization을 같이 만족하는 구성을 찾는 것으로 방향이 바뀌었다."
        ),
        latex_par(
            f"historical strongest single-LOSO reference는 {baseline['name']} (MAE {baseline['mae']:.4f}, RMSE {baseline['rmse']:.4f})이고, final validated full LOSO 결과에서는 archive_6 Tiny가 MAE 2.6344, Teacher가 MAE 2.6369로 가장 강한 두 기준점으로 남았다. "
            "latest archive_8 full LOSO에서는 physics feature가 internal teacher baseline 대비는 개선을 만들었지만, 아직 project-wide strongest baseline을 넘지는 못했다."
        ),
        r"\section{먼저 알아야 할 용어와 읽는 법}",
        latex_par(
            "이번 보고서는 실험 수가 많고 용어가 자주 바뀌기 때문에, 먼저 몇 가지 핵심 용어를 고정해서 읽는 것이 좋다. 아래 표는 이후 모든 section에서 반복해서 나오는 표현을 같은 뜻으로 쓰기 위한 사전이다."
        ),
        latex_table(
            glossary_rows[0],
            glossary_rows[1:],
            r">{\raggedright\arraybackslash}p{0.18\linewidth}>{\raggedright\arraybackslash}p{0.34\linewidth}>{\raggedright\arraybackslash}p{0.34\linewidth}",
            "핵심 용어 정리",
            size=r"\footnotesize",
        ),
        latex_par(
            "특히 residual target과 calibrated robot angle의 관계를 이해하는 것이 중요하다. 이 프로젝트는 절대 hip angle을 처음부터 끝까지 직접 맞추는 문제로 보지 않고, calibrated robot angle을 기준선으로 두고 그 위에 남는 오차를 예측하는 보정 문제로 본다. "
            "즉 모델 출력은 '사람 hip angle 그 자체'가 아니라 '기준선에서 얼마나 더 보정해야 하는가'다."
        ),
        r"\section{문제 정의와 데이터}",
        latex_par(
            "문제는 hip assistive wearable robot이 측정한 robot-side hip/thigh angle과 torque만으로는 실제 사람의 hip flexion angle을 정확히 알 수 없다는 데서 시작된다. "
            "실제 착용 환경에서는 sensor가 사람이 아니라 로봇에 붙어 있고, 사람과 로봇이 완전히 강체처럼 붙어 있지 않기 때문에 상대운동과 compliance가 존재한다. "
            "따라서 오차는 단순 노이즈가 아니라 human--robot interface dynamics에서 생기는 구조적 residual로 보는 것이 맞다."
        ),
        latex_par(
            "데이터는 combined_data_S008.h5이고, subject는 8명(S001--S008), condition은 7개(level 0.75/1.0/1.25 mps, accel sine, decline 5 deg, incline 10 deg, stop-and-go), misalignment level은 lv0/lv4/lv7이다. "
            "샘플링은 100 Hz이며 출력은 mocap/kin\\_q의 hip\\_flexion\\_l, hip\\_flexion\\_r이다."
        ),
        latex_table(
            ["항목", "값"],
            [
                ["피험자 수", "8"],
                ["조건 수", "7"],
                ["misalignment level", "lv0 / lv4 / lv7"],
                ["총 trial 수", "169"],
                ["샘플링 주파수", "100 Hz"],
                ["출력", "mocap/kin_q의 hip_flexion_l, hip_flexion_r"],
                ["screening 방식", "single LOSO (주로 m2_S008)"],
                ["최종 검증 방식", "full LOSO 8 folds + 일부 seed 재현성"],
            ],
            r">{\raggedright\arraybackslash}p{0.26\linewidth}>{\raggedright\arraybackslash}p{0.64\linewidth}",
            "데이터셋과 검증 프로토콜 요약",
        ),
        latex_par(
            "실험 결과를 읽을 때는 single LOSO와 full LOSO를 반드시 구분해야 한다. screening 단계에서 좋아 보였던 feature가 full LOSO에서 사라진 사례가 여러 번 나왔기 때문이다. 따라서 single LOSO는 어디까지나 'signal detection' 단계로 읽어야 하고, full LOSO가 최종 결론을 결정한다."
        ),
        latex_table(
            protocol_rows[0],
            protocol_rows[1:],
            r">{\raggedright\arraybackslash}p{0.16\linewidth}>{\raggedright\arraybackslash}p{0.34\linewidth}>{\raggedright\arraybackslash}p{0.34\linewidth}",
            "single LOSO와 full LOSO의 역할 구분",
            size=r"\footnotesize",
        ),
        r"\section{역사적 전체 성능 순위와 살아남은 모델}",
        latex_par(
            "archive_season4를 설계하려면 먼저 '무엇이 역사적으로 실제로 살아남았는가'를 한눈에 봐야 한다. "
            "아래 표는 season1--season2에서 동일한 shared hip-angle target 기준으로 평가된 모델들을 성능 순으로 정렬한 것이다. "
            "season3는 target 자체가 human thigh residual로 바뀌므로 이 표와 직접 섞지 않고 뒤에서 별도로 정리한다."
        ),
        latex_table(
            ["순위", "실험", "MAE", "RMSE", "R2", "lag(ms)", "비고"],
            hist_top_rows,
            r">{\centering\arraybackslash}p{0.06\linewidth}>{\raggedright\arraybackslash}p{0.40\linewidth}>{\raggedright\arraybackslash}p{0.08\linewidth}>{\raggedright\arraybackslash}p{0.08\linewidth}>{\raggedright\arraybackslash}p{0.08\linewidth}>{\raggedright\arraybackslash}p{0.08\linewidth}>{\raggedright\arraybackslash}p{0.14\linewidth}",
            "역사적 상위 25개 모델 성능 순위 (shared hip-angle target)",
            size=r"\scriptsize",
        ),
        latex_figure("assets/historical_top20_mae.png", "historical top-20 models by MAE"),
        latex_par(
            "이 순위를 보면 상위권은 일관되게 orientation-preserving 입력을 유지한 residual TCN 계열이 차지한다. "
            "즉 역사적으로 살아남은 축은 body-frame 대체가 아니라 Euler + quaternion + robot hip/thigh/torque + 경량 derived feature 구조다."
        ),
        latex_table(
            ["대표 모델", "모델 구조", "입력 구성", "구조 세부", "왜 좋은 결과를 보였는가"],
            strong_model_rows,
            r">{\raggedright\arraybackslash}p{0.16\linewidth}>{\raggedright\arraybackslash}p{0.18\linewidth}>{\raggedright\arraybackslash}p{0.26\linewidth}>{\raggedright\arraybackslash}p{0.20\linewidth}>{\raggedright\arraybackslash}p{0.16\linewidth}",
            "좋은 결과를 보인 대표 모델과 구조 요약",
            size=r"\scriptsize",
        ),
        r"\section{입력 feature, 전처리, 출력 구성}",
        latex_par(
            "입력은 크게 네 계층으로 나눌 수 있다. (1) trunk IMU raw signal과 quaternion, (2) robot hip/thigh/torque 원천 신호, (3) 미분 및 power 같은 경량 derived feature, (4) body-frame, load-coupling, trunk-consistency처럼 특정 가설을 반영한 실험용 derived feature다."
        ),
        latex_par(
            "공통 전처리는 quaternion sign continuity, complementary filter 기반 Euler angle, finite difference+저역통과 기반 derivative, calibration offset 추정, 그리고 residual target 구성이다. "
            "특히 residual target은 이 프로젝트의 핵심 설계다. 모델이 직접 절대 hip angle을 맞추는 대신, calibrated robot reference와 실제 human hip angle 사이의 차이를 예측함으로써, 문제를 '보정량 추정'으로 바꾼다."
        ),
        latex_table(
            ["feature group", "예시 채널", "생성 방식", "계산 부담", "프로젝트 내 역할"],
            [
                ["derived/euler", "roll, pitch, yaw", "complementary filter", "낮음", "최종 teacher/tiny 핵심"],
                ["robot kinematics", "hip_angle, thigh_angle, torque", "raw signal", "매우 낮음", "모든 archive의 기본 골격"],
                ["quat", "quat_wxyz", "raw signal", "매우 낮음", "orientation-preserving baseline"],
                ["derivative", "thigh_angle_dot, torque_dot", "finite diff + LPF", "낮음", "A4 이후 꾸준히 채택"],
                ["mech_power", "left/right power", "torque × hip_angle_dot", "낮음", "A5/A6 최종 후보에 포함"],
                ["body_frame_imu", "acc/gyro in body frame", "global rotation + gravity remove + PCA heading", "높음", "A1--A3 실험용"],
                ["gravity_aligned_imu", "gravity-only aligned acc/gyro", "z_up만 정렬", "중간", "A3 실험용"],
                ["symmetry_heading_imu", "symmetry-yaw aligned IMU", "yaw 후보 search", "중간", "A3 실험용"],
                ["load_coupling", "torque × gap", "torque와 gap의 곱", "낮음", "A5/A8 physics family"],
                ["interface_workloop", "pos/neg/net work", "rolling work split", "낮음", "A7/A8 physics family"],
                ["trunk_consistency", "gyro minus thighdot / pitchdot", "trunk-robot consistency residual", "낮음", "A7/A8 physics family"],
                ["effective_impedance", "k,c,b,pred_err", "online RLS", "중간", "A7 physics family"],
            ],
            r">{\raggedright\arraybackslash}p{0.18\linewidth}>{\raggedright\arraybackslash}p{0.18\linewidth}>{\raggedright\arraybackslash}p{0.25\linewidth}>{\raggedright\arraybackslash}p{0.10\linewidth}>{\raggedright\arraybackslash}p{0.21\linewidth}",
            "입력 feature 계층과 프로젝트 내 역할",
            size=r"\footnotesize",
        ),
        latex_table(
            feature_dictionary_rows[0],
            feature_dictionary_rows[1:],
            r">{\raggedright\arraybackslash}p{0.17\linewidth}>{\raggedright\arraybackslash}p{0.22\linewidth}>{\raggedright\arraybackslash}p{0.28\linewidth}>{\raggedright\arraybackslash}p{0.23\linewidth}",
            "feature dictionary: 무엇을 어떻게 만들었고 왜 넣었는가",
            size=r"\scriptsize",
        ),
        latex_par(
            "중요한 해석은 이것이다. 최종적으로 살아남은 feature는 대부분 O(1) 또는 짧은 rolling 통계 수준이다. body-frame PCA heading, symmetry-yaw, heavy latent-state 추정처럼 계산량이 더 큰 표현은 final candidate에서 제외되었다. "
            "즉 이 프로젝트는 후반으로 갈수록 accuracy와 함께 실시간성 제약을 더 강하게 반영하게 되었다."
        ),
        r"\section{모델 구조와 학습 전략}",
        latex_par(
            "기본 backbone은 TCN_MLP다. dilation이 증가하는 temporal convolution block으로 context를 인코딩한 뒤, 마지막 시점 feature를 작은 MLP head에 넣는다. teacher는 window 200, channels [32,64,64,128], kernel 5를 쓰고, tiny는 window 100, channels [16,24,24,48], kernel 3으로 경량화한다."
        ),
        latex_par(
            "학습 공통 설정은 batch 128, epochs 30, lr 0.001, wd 0.01, AMP, cosine scheduler, early stopping이다. screening archive는 single LOSO로 방향성을 빠르게 확인하고, 최종 후보 archive는 full LOSO와 seed 재현성까지 본다."
        ),
        latex_table(
            ["모델", "구조", "입력 window", "channels", "kernel", "head", "파라미터", "모델 크기(MB)"],
            [
                ["Teacher", "TCN_MLP", "200", "[32,64,64,128]", "5", "[64]", "224,072", "0.8548"],
                ["Tiny", "TCN_MLP", "100", "[16,24,24,48]", "3", "[32]", "22,720", "0.0867"],
                ["Tiny+LoadGate", "LoadGatedTCN", "100", "[16,24,24,48]", "3", "[32]", "23,597", "0.0900"],
            ],
            r">{\raggedright\arraybackslash}p{0.14\linewidth}>{\raggedright\arraybackslash}p{0.16\linewidth}>{\raggedright\arraybackslash}p{0.10\linewidth}>{\raggedright\arraybackslash}p{0.18\linewidth}>{\raggedright\arraybackslash}p{0.07\linewidth}>{\raggedright\arraybackslash}p{0.08\linewidth}>{\raggedright\arraybackslash}p{0.13\linewidth}>{\raggedright\arraybackslash}p{0.10\linewidth}",
            "최종 모델 family 구조 비교",
            size=r"\footnotesize",
        ),
        latex_par(
            "LoadGatedTCN은 load-related 입력에서 게이트를 계산해 전체 입력을 time-step별로 scaling하는 구조였다. 하지만 full LOSO와 재현성 검증에서 standard tiny를 넘지 못했다. 따라서 gating은 흥미로운 negative result로 남았고, 최종 메인 모델은 standard tiny causal TCN으로 수렴했다."
        ),
        latex_par(
            "모델 구조를 읽는 가장 쉬운 방법은 teacher와 tiny를 같은 backbone의 두 스케일 버전으로 보는 것이다. teacher는 '이 정도면 충분히 강한 큰 모델'을 의미하고, tiny는 같은 입력 표현을 유지하되 window와 채널 수를 줄여 배포성을 노린 버전이다. "
            "따라서 두 모델의 차이는 아이디어 차이보다 '같은 문제를 얼마나 작은 계산량으로 풀 수 있느냐'에 있다."
        ),
        r"\section{전체 archive 흐름 요약}",
        latex_par(
            "archive를 시간순으로 보면 프로젝트의 가설 정제 과정이 보인다. A1--A3는 representation과 좌표계 문제, A4--A5는 biomechanical/misalignment feature 문제, A6는 deployability 문제, A7--A8은 physics feature의 직접적 효과를 다시 검증하는 문제였다."
        ),
        latex_table(
            archive_summary_rows[0],
            archive_summary_rows[1:],
            r">{\raggedright\arraybackslash}p{0.09\linewidth}>{\raggedright\arraybackslash}p{0.18\linewidth}>{\raggedright\arraybackslash}p{0.22\linewidth}>{\raggedright\arraybackslash}p{0.08\linewidth}>{\raggedright\arraybackslash}p{0.08\linewidth}>{\raggedright\arraybackslash}p{0.26\linewidth}",
            "archive별 최고 결과와 핵심 해석",
            size=r"\footnotesize",
        ),
        latex_figure("assets/season2_archive_progress.png", "season2 archive별 최고 single-LOSO MAE 진행"),
        latex_par(
            "A1과 A2의 핵심 질문은 'body-frame이나 더 물리적인 표현이 더 좋을까'였다. 결과는 아니었다. A3는 이 질문을 더 엄격하게 검증했고, raw+quat 또는 gravity+quat가 더 강했다. "
            "A4와 A5는 '그러면 misalignment-aware biomechanical feature가 핵심 아닐까'를 시험했지만, additive gain은 약했다. A6는 방향을 바꿔 tiny deployable model을 찾는 단계였다. "
            "A7과 A8은 다시 physics feature를 정교하게 다듬어, 정말 physics-informed feature가 baseline을 이길 수 있는지를 재검증한 단계다."
        ),
        latex_table(
            ["항목", "내용"],
            archive_hypothesis_rows["A1-A3"],
            r">{\raggedright\arraybackslash}p{0.16\linewidth}>{\raggedright\arraybackslash}p{0.74\linewidth}",
            "Archive 1--3: 가설-설계-결과-판단",
            size=r"\footnotesize",
        ),
        r"\subsection{Archive 1--3: representation과 body-frame 탐색}",
        latex_figure("assets/archive3_representation.png", "archive 3 representation comparison"),
        latex_par(
            "A1과 A2에서는 body-frame, gait/asymmetry, biomech fusion, longer context, hybrid representation 등을 비교했다. 결론은 body-frame 단독 대체는 강하지 않고, quat/euler를 보존한 baseline이 더 안정적이라는 점이었다. "
            "A3에서 raw trunk IMU, gravity-only, symmetry-yaw, PCA body-frame을 통제해서 비교했을 때도 같은 메시지가 반복되었다. 특히 treadmill 환경에서는 heading normalization이 생각만큼 유효하지 않았다."
        ),
        r"\subsection{Archive 4--5: biomechanical feature screening}",
        latex_figure("assets/archive4_biomech_screening.png", "archive 4 biomechanical feature screening"),
        latex_figure("assets/archive5_misalign_screening.png", "archive 5 misalignment-aware feature screening"),
        latex_par(
            "A4에서는 kinematics, mech_power, deflection, asymmetry 등을 teacher-style baseline 위에 올려봤다. 이 단계에서 derivative와 mech_power는 살아남았지만, feature를 많이 넣는다고 자동으로 좋아지지는 않았다. "
            "A5에서는 relative_kinematics, compliance_proxy, load_coupling처럼 misalignment-specific feature를 설계했지만, 그중 generalization 가능성이 있었던 것은 load_coupling 정도였다."
        ),
        latex_table(
            ["항목", "내용"],
            archive_hypothesis_rows["A4-A5"],
            r">{\raggedright\arraybackslash}p{0.16\linewidth}>{\raggedright\arraybackslash}p{0.74\linewidth}",
            "Archive 4--5: 가설-설계-결과-판단",
            size=r"\footnotesize",
        ),
        r"\subsection{Archive 6: deployability로의 수렴}",
        latex_figure("assets/archive6_tradeoff_size_mae.png", "archive 6 accuracy-size tradeoff"),
        latex_figure("assets/archive6_tradeoff_latency_mae.png", "archive 6 accuracy-latency tradeoff"),
        latex_par(
            "A6의 목표 축은 명확했다. accuracy만이 아니라 latency, parameter count, deployability를 같이 본다. 결과적으로 tiny는 teacher보다 약 90% 작은 모델 크기와 훨씬 작은 parameter count를 가지면서, full LOSO에서 accuracy 손해를 작게 유지했다. "
            "이 결과 때문에 프로젝트의 strongest claim은 정확도 absolute best보다, deployable misalignment compensation 시스템 쪽으로 이동했다."
        ),
        latex_table(
            headers_final,
            rows_final,
            r">{\raggedright\arraybackslash}p{0.14\linewidth}>{\raggedright\arraybackslash}p{0.07\linewidth}>{\raggedright\arraybackslash}p{0.10\linewidth}>{\raggedright\arraybackslash}p{0.09\linewidth}>{\raggedright\arraybackslash}p{0.10\linewidth}>{\raggedright\arraybackslash}p{0.12\linewidth}>{\raggedright\arraybackslash}p{0.09\linewidth}>{\raggedright\arraybackslash}p{0.10\linewidth}>{\raggedright\arraybackslash}p{0.10\linewidth}",
            "archive 6 final full LOSO 결과",
            size=r"\footnotesize",
        ),
        latex_table(
            headers_repro,
            rows_repro,
            r">{\raggedright\arraybackslash}p{0.18\linewidth}>{\raggedright\arraybackslash}p{0.10\linewidth}>{\raggedright\arraybackslash}p{0.10\linewidth}>{\raggedright\arraybackslash}p{0.12\linewidth}>{\raggedright\arraybackslash}p{0.11\linewidth}>{\raggedright\arraybackslash}p{0.12\linewidth}>{\raggedright\arraybackslash}p{0.12\linewidth}>{\raggedright\arraybackslash}p{0.10\linewidth}",
            "archive 6 3-seed 재현성 요약",
        ),
        latex_figure("assets/archive6_seed_mae.png", "archive 6 seed별 MAE mean"),
        latex_figure("assets/archive6_pairwise_mae_diff.png", "subject×seed pair별 Tiny-Teacher MAE 차이"),
        latex_par(
            f"재현성까지 포함하면 tiny가 teacher를 완전히 이긴다고 말하긴 어렵다. 총 {int(pair_row['n_pairs'])}쌍의 subject×seed 비교에서 Tiny가 더 좋은 경우는 {int(pair_row['tiny_better_count'])}회, Teacher가 더 좋은 경우는 {int(pair_row['teacher_better_count'])}회였다. "
            f"하지만 Tiny는 accuracy 손해를 매우 작게 유지하면서 모델 크기와 배포 부담을 크게 줄였기 때문에, 논문 메인 proposed model로는 오히려 더 설득력 있다."
        ),
        latex_table(
            ["항목", "내용"],
            archive_hypothesis_rows["A6"],
            r">{\raggedright\arraybackslash}p{0.16\linewidth}>{\raggedright\arraybackslash}p{0.74\linewidth}",
            "Archive 6: 가설-설계-결과-판단",
            size=r"\footnotesize",
        ),
        r"\subsection{Archive 7--8: physics feature 재검증}",
        latex_figure("assets/archive7_physics_screening.png", "archive 7 physics feature screening"),
        latex_figure("assets/archive8_physics_screening.png", "archive 8 physics feature re-screening"),
        latex_par(
            "A7은 interface work split, hysteresis, effective impedance, lag, phase, trunk consistency를 직접 비교한 archive였다. single-LOSO에서는 worksplit과 impedance가 매우 강해 보였다. "
            "하지만 full LOSO로 올리면 그 gain이 줄어들었다. 이 때문에 A8에서는 worksplit과 load_coupling을 정규화/짧은 window로 재설계하고, trunk consistency와 lag를 다시 좁게 검증했다."
        ),
        latex_par(
            "A8 single-LOSO에서는 physics feature가 정말로 baseline을 크게 이겼다. short worksplit, normalized load coupling, trunk consistency, normalized worksplit가 모두 baseline 대비 13% 이상 개선을 보였다. "
            "하지만 A8 full LOSO에서는 그 신호가 줄어들었다. 그럼에도 normalized worksplit와 trunk consistency는 archive_8 내부 baseline보다 조금 더 좋은 평균을 보였다."
        ),
        latex_figure("assets/archive7_full_loso.png", "archive 7 full LOSO comparison"),
        latex_figure("assets/archive8_full_loso.png", "archive 8 full LOSO comparison"),
        latex_table(
            ["모델", "MAE mean", "MAE std", "RMSE mean", "Latency(ms)", "A8 baseline 대비"],
            a8_compare_rows,
            r">{\raggedright\arraybackslash}p{0.32\linewidth}>{\raggedright\arraybackslash}p{0.10\linewidth}>{\raggedright\arraybackslash}p{0.10\linewidth}>{\raggedright\arraybackslash}p{0.10\linewidth}>{\raggedright\arraybackslash}p{0.12\linewidth}>{\raggedright\arraybackslash}p{0.14\linewidth}",
            "archive 8 full LOSO 결과 요약",
            size=r"\footnotesize",
        ),
        latex_par(
            "A8 full LOSO의 핵심 해석은 이렇다. physics feature는 분명히 완전히 죽지 않았고, normalized worksplit나 trunk consistency처럼 일부 family는 baseline보다 소폭 개선을 만들었다. "
            "하지만 그 개선은 아직 project-wide strongest baseline(A6 teacher/tiny)을 넘는 수준은 아니다. 즉 physics feature가 완전히 틀린 방향은 아니지만, 지금 단계에서는 strongest final claim으로 쓰기엔 한 단계 부족하다."
        ),
        latex_figure("assets/archive8_subjectwise_mae.png", "archive 8 full LOSO subject-wise MAE"),
        latex_par(
            "subject-wise로 보면 A8 physics candidate의 gain은 특정 subject에서 더 강하고, 다른 subject에서는 baseline과 비슷하거나 불리하다. 이런 패턴은 physics feature가 single-LOSO에서 크게 좋아 보이다가 full LOSO에서 average gain이 줄어든 이유를 설명해 준다."
        ),
        latex_table(
            ["항목", "내용"],
            archive_hypothesis_rows["A7-A8"],
            r">{\raggedright\arraybackslash}p{0.16\linewidth}>{\raggedright\arraybackslash}p{0.74\linewidth}",
            "Archive 7--8: 가설-설계-결과-판단",
            size=r"\footnotesize",
        ),
        r"\section{Season3 target 재정의 실험}",
        latex_par(
            "season3는 앞선 season1--2와 문제 정의 자체가 다르다. output을 기존 hip flexion residual에서 human thigh residual로 재정의하고, "
            "baseline을 calibrated robot thigh로 두었다. 구체적으로 human thigh는 pelvis\\_tilt + hip\\_flexion으로 정의했고, 그 위에 physics branch를 분리한 factorized 구조가 유효한지 확인했다."
        ),
        latex_par(
            "이 실험군은 절대 성능을 season2와 바로 비교하기보다는, 'physics 정보를 raw branch와 분리했을 때 target 재정의 문제에서 도움이 되는가'를 보는 exploratory stage로 읽어야 한다. "
            "즉 season3는 strongest final model을 만들기보다, season4에서 structure-oriented archive를 설계할 때 어떤 branch가 살아남는지를 보기 위한 bridge 역할에 가깝다."
        ),
        latex_table(
            ["모델", "MAE mean", "MAE std", "RMSE mean", "Latency(ms)", "Size(MB)", "해석"],
            [
                [
                    pretty_name(r["name"]),
                    fmt(r["mae_mean"], 4),
                    fmt(r["mae_std"], 4),
                    fmt(r["rmse_mean"], 4),
                    fmt(r["host_latency_ms"], 4),
                    fmt(r["model_size_mb"], 4),
                    "season3 internal best" if idx == 0 else (
                        "plain tiny baseline" if "ctrl_tiny" in r["name"] else (
                            "factorized teacher full" if "fact_teacher_full" in r["name"] else (
                                "teacher baseline" if "ctrl_teacher" in r["name"] else (
                                    "linear grouped physics only" if "lin_grouped" in r["name"] else "physics-only ablation"
                                )
                            )
                        )
                    ),
                ]
                for idx, (_, r) in enumerate(s3.sort_values(["mae_mean", "rmse_mean"]).iterrows())
            ],
            r">{\raggedright\arraybackslash}p{0.28\linewidth}>{\raggedright\arraybackslash}p{0.09\linewidth}>{\raggedright\arraybackslash}p{0.09\linewidth}>{\raggedright\arraybackslash}p{0.10\linewidth}>{\raggedright\arraybackslash}p{0.10\linewidth}>{\raggedright\arraybackslash}p{0.09\linewidth}>{\raggedright\arraybackslash}p{0.19\linewidth}",
            "season3 archive_1 요약 (human-thigh target, 3-subject mini-LOSO × 2 seeds)",
            size=r"\scriptsize",
        ),
        latex_figure("assets/season3_archive1_summary.png", "season3 archive_1 summary"),
        latex_par(
            "season3의 핵심 결론은 세 가지다. 첫째, physics-only 또는 grouped-linear baseline은 시각적으로도 수치적으로도 부족했다. 둘째, factorized full 구조는 plain baseline보다 약간 좋아졌지만 calibrated robot thigh를 충분히 벗어나지는 못했다. "
            "셋째, season3 best인 factorized tiny full조차 season2 strongest baseline보다 절대 수치는 훨씬 뒤진다. 따라서 season3는 최종 모델이 아니라 'physics-aware branch를 어떻게 다뤄야 하는가'에 대한 구조적 시사점으로 사용하는 것이 맞다."
        ),
        r"\section{대표 trajectory와 정성 분석}",
        latex_figure("assets/archive6_teacher_trajectory_with_robot.png", "A6 Teacher representative trajectory (prediction / ground truth / calibrated robot angle)"),
        latex_figure("assets/archive6_tiny_trajectory_with_robot.png", "A6 Tiny representative trajectory (prediction / ground truth / calibrated robot angle)"),
        latex_figure("assets/trajectory_a8_short_worksplit.png", "A8 short worksplit representative trajectory"),
        latex_figure("assets/trajectory_a8f_norm_worksplit.png", "A8F normalized worksplit representative trajectory"),
        latex_figure("assets/archive6_feature_importance_teacher.png", "Teacher top-10 feature importance"),
        latex_figure("assets/archive6_feature_importance_tiny.png", "Tiny top-10 feature importance"),
        latex_par(
            "Teacher와 Tiny의 feature importance는 공통적으로 robot hip angle, torque, thigh angle, derivative, 그리고 일부 orientation 정보를 가장 중요하게 사용하고 있음을 보여준다. "
            "즉 이 문제는 결국 body-frame을 얼마나 정교하게 만들었느냐보다, robot-human interface에서 바로 측정되는 관찰량과 orientation 정보를 얼마나 적절히 보존했느냐의 문제에 더 가깝다."
        ),
        r"\section{실시간 추론 가능성}",
        latex_par(
            "본 프로젝트에서 최종적으로 살아남은 모델은 나중에 실시간 inference에 들어가야 한다는 제약 아래 설계되었다. 따라서 전처리부터 모델 추론까지 전체 파이프라인의 계산량이 가벼워야 한다. "
            "최종 tiny와 teacher는 complementary filter Euler, quaternion raw input, derivative, mech_power만 사용한다. 이들은 모두 sample당 O(1) 혹은 짧은 rolling 통계 수준이다."
        ),
        latex_table(
            ["구성 요소", "최종 구현", "계산 부담", "판단"],
            [
                ["Euler 추정", "Complementary filter", "sample당 O(1)", "채택"],
                ["Quaternion 사용", "raw quat_wxyz 직접 입력", "매우 낮음", "채택"],
                ["Derivative", "finite difference + LPF", "낮음", "채택"],
                ["mech_power", "torque × hip_angle_dot", "매우 낮음", "채택"],
                ["body-frame heading", "PCA / symmetry yaw", "중간~높음", "최종 후보 제외"],
                ["effective impedance", "online RLS", "중간", "research feature로만 사용"],
                ["Tiny inference", "window 100 causal TCN", "약 2.10 ms", "100 Hz 가능"],
            ],
            r">{\raggedright\arraybackslash}p{0.22\linewidth}>{\raggedright\arraybackslash}p{0.28\linewidth}>{\raggedright\arraybackslash}p{0.18\linewidth}>{\raggedright\arraybackslash}p{0.18\linewidth}",
            "실시간 추론 가능성 관점의 계산 부담 분석",
        ),
        latex_par(
            "100 Hz loop를 가정하면 주기당 예산은 10 ms다. tiny의 host latency는 약 2.10 ms이므로 추론만 놓고는 예산의 약 21% 수준이다. "
            "전처리 역시 O(1) 수준이라 sample-by-sample online 구현이 가능하다. 반면 body-frame PCA heading은 고유값 분해가 필요하기 때문에 final online candidate에서 제외한 판단이 타당하다."
        ),
        latex_figure("assets/full_loso_candidate_size_mae.png", "full LOSO candidate accuracy-size overview"),
        latex_figure("assets/full_loso_candidate_latency_mae.png", "full LOSO candidate accuracy-latency overview"),
        r"\section{논문화 시 contribution과 메시지}",
        latex_par(
            "지금 결과를 과장 없이 정리하면, 가장 강하게 주장할 수 있는 contribution은 세 가지다. "
            "첫째, wearable hip exoskeleton misalignment compensation을 software-only residual correction 문제로 정식화했다는 점이다. "
            "둘째, orientation-preserving sensor fusion이 aggressive body-frame normalization보다 더 안정적임을 controlled archive 실험으로 보였다는 점이다. "
            "셋째, biomechanics-informed tiny causal TCN이 큰 teacher 모델에 비해 매우 작은 모델 크기와 실시간 배포 가능성을 확보하면서도 상당한 정확도를 유지했다는 점이다."
        ),
        latex_table(
            ["기여 유형", "강하게 주장 가능한 내용", "왜 강한가"],
            [
                ["문제 정식화", "software-only residual correction으로 misalignment 보정", "프로젝트 전체가 일관되게 이 formulation 위에서 성공/실패를 설명함"],
                ["representation insight", "orientation-preserving fusion > aggressive normalization", "A2--A3에서 반복적으로 관찰됨"],
                ["accuracy-efficiency tradeoff", "tiny가 약 90% 모델 축소로도 경쟁력 유지", "A6 full LOSO + reproducibility로 검증됨"],
                ["deployability", "전처리와 추론이 online-friendly하게 유지됨", "최종 후보가 모두 O(1) 혹은 짧은 rolling 계산 위주"],
            ],
            r">{\raggedright\arraybackslash}p{0.18\linewidth}>{\raggedright\arraybackslash}p{0.42\linewidth}>{\raggedright\arraybackslash}p{0.30\linewidth}",
            "논문화 시 강하게 주장 가능한 contribution",
            size=r"\footnotesize",
        ),
        latex_par(
            "반대로 약하게만 주장해야 하는 contribution도 있다. physics feature가 의미 있는 single-LOSO 신호를 주는 것은 맞지만, 아직 strongest full LOSO baseline을 넘었다고 말할 수는 없다. "
            "body-frame superiority나 load-gated final winner 같은 주장도 현재 결과로는 지지되지 않는다."
        ),
        r"\section{한 페이지 최종 메시지}",
        latex_table(
            ["항목", "최종 정리"],
            [
                ["가장 강한 전체 결론", "orientation-preserving + lightweight biomechanical feature가 가장 안정적으로 살아남았다"],
                ["정확도 기준 strongest validated baseline", "Archive 6 Tiny / Teacher full LOSO"],
                ["physics feature 쪽 핵심 결론", "single LOSO strong signal은 있었고, full LOSO에서도 internal baseline 소폭 개선은 확인됐지만 project-wide 최고는 아직 아님"],
                ["실패한 방향", "body-frame 대체, aggressive heading normalization, feature 대량 concat, loadgate final winner"],
                ["살아남은 입력 구성", "robot hip/thigh/torque + quat/euler + derivative + mech_power"],
                ["배포 관점 결론", "Tiny causal TCN이 현재 가장 현실적인 최종 모델"],
                ["논문 메시지 추천", "misalignment compensation의 robust representation과 deployable model design을 밝힌 study"],
            ],
            r">{\raggedright\arraybackslash}p{0.24\linewidth}>{\raggedright\arraybackslash}p{0.66\linewidth}",
            "보고서 전체를 한 페이지로 압축한 최종 메시지",
            size=r"\footnotesize",
        ),
        latex_par(
            "따라서 추천 논문 스토리는 다음과 같다. "
            "메인 모델은 teacher가 아니라 tiny로 두고, teacher는 strongest accuracy reference로 둔다. "
            "초반 archive의 negative result(body-frame / normalization 실패), 중반 archive의 feature screening, 후반 archive의 deployability와 full LOSO 검증을 모두 하나의 연속된 탐색 이야기로 묶는다. "
            "즉 이 논문은 '새로운 복잡한 feature를 넣어서 SOTA를 찍은 논문'보다, '어떤 표현과 어떤 경량 구조가 실제 wearable robot misalignment compensation에 살아남는지 밝힌 논문'으로 쓰는 것이 맞다."
        ),
        r"\section{Archive Season4 설계에 바로 쓸 정리}",
        latex_par(
            "archive\\_season4를 시작할 때는 단순히 '성능이 가장 좋았던 실험' 하나만 가져가는 것이 아니라, 역사적으로 검증된 축과 exploratory하게 signal만 보인 축을 분리해야 한다. "
            "이 보고서 기준으로 season4 설계의 출발점은 A6 Tiny/Teacher를 anchor baseline으로 두고, A8의 physics family와 season3의 factorized 구조를 좁게 결합하는 것이다."
        ),
        latex_table(
            ["후보", "노리는 기여 축", "현재까지의 근거", "season4에서의 사용 원칙"],
            season4_candidate_rows,
            r">{\raggedright\arraybackslash}p{0.22\linewidth}>{\raggedright\arraybackslash}p{0.18\linewidth}>{\raggedright\arraybackslash}p{0.28\linewidth}>{\raggedright\arraybackslash}p{0.24\linewidth}",
            "season4 설계용 핵심 후보 정리",
            size=r"\footnotesize",
        ),
        latex_par(
            "따라서 season4의 우선순위는 다음처럼 정리된다. "
            "첫째, A6 tiny/teacher 입력 구성을 anchor로 고정한다. 둘째, physics feature는 normalized worksplit 또는 trunk consistency처럼 full LOSO에서 internal baseline 개선을 보인 단일 family만 사용한다. "
            "셋째, season3의 factorized 구조는 raw branch를 유지한 full model에만 한정해 검토하고, physics-only나 grouped-linear baseline은 재시도하지 않는다. "
            "넷째, season4의 메인 목표는 absolute best score보다 'physics-aware structure가 anchor 대비 왜 더 낫거나 덜 나쁜지 설명 가능한 결과'를 만드는 것이다."
        ),
        r"\appendix",
        r"\section{Archive별 실험 인벤토리}",
        latex_par("아래 표들은 각 archive에서 실제로 학습한 실험들을 결과와 함께 빠짐없이 정리한 것이다. screening과 final full LOSO를 분리해 실었다."),
        r"\subsection{역사적 전체 single-LOSO 성능 순위}",
        latex_longtable(
            ["실험", "MAE", "RMSE", "R2", "lag(ms)", "MAE change(%)"],
            historical_model_rows,
            r">{\raggedright\arraybackslash}p{0.46\linewidth}>{\raggedright\arraybackslash}p{0.08\linewidth}>{\raggedright\arraybackslash}p{0.08\linewidth}>{\raggedright\arraybackslash}p{0.08\linewidth}>{\raggedright\arraybackslash}p{0.08\linewidth}>{\raggedright\arraybackslash}p{0.12\linewidth}",
            "season1--season2 shared hip-angle target historical ranking",
        ),
        r"\subsection{Season3 archive 1 inventory}",
        latex_longtable(
            ["모델", "MAE mean", "MAE std", "RMSE mean", "Latency", "MaxFreq", "MB"],
            [
                [
                    pretty_name(r["name"]),
                    fmt(r["mae_mean"], 4),
                    fmt(r["mae_std"], 4),
                    fmt(r["rmse_mean"], 4),
                    fmt(r["host_latency_ms"], 4),
                    fmt(r["host_max_freq_hz"], 1),
                    fmt(r["model_size_mb"], 4),
                ]
                for _, r in s3.sort_values(["mae_mean", "rmse_mean"]).iterrows()
            ],
            r">{\raggedright\arraybackslash}p{0.32\linewidth}>{\raggedright\arraybackslash}p{0.10\linewidth}>{\raggedright\arraybackslash}p{0.10\linewidth}>{\raggedright\arraybackslash}p{0.10\linewidth}>{\raggedright\arraybackslash}p{0.10\linewidth}>{\raggedright\arraybackslash}p{0.10\linewidth}>{\raggedright\arraybackslash}p{0.08\linewidth}",
            "season3 archive_1 human-thigh target inventory",
        ),
        r"\subsection{Archive 1 실험 목록}",
        latex_longtable(["실험", "MAE", "RMSE", "Params", "MB", "Latency", "MaxFreq"], archive_inventory_rows(a1), r">{\raggedright\arraybackslash}p{0.42\linewidth}>{\raggedright\arraybackslash}p{0.08\linewidth}>{\raggedright\arraybackslash}p{0.08\linewidth}>{\raggedright\arraybackslash}p{0.12\linewidth}>{\raggedright\arraybackslash}p{0.08\linewidth}>{\raggedright\arraybackslash}p{0.08\linewidth}>{\raggedright\arraybackslash}p{0.08\linewidth}", "Archive 1 experiment inventory"),
        r"\subsection{Archive 2 실험 목록}",
        latex_longtable(["실험", "MAE", "RMSE", "Params", "MB", "Latency", "MaxFreq"], archive_inventory_rows(a2), r">{\raggedright\arraybackslash}p{0.42\linewidth}>{\raggedright\arraybackslash}p{0.08\linewidth}>{\raggedright\arraybackslash}p{0.08\linewidth}>{\raggedright\arraybackslash}p{0.12\linewidth}>{\raggedright\arraybackslash}p{0.08\linewidth}>{\raggedright\arraybackslash}p{0.08\linewidth}>{\raggedright\arraybackslash}p{0.08\linewidth}", "Archive 2 experiment inventory"),
        r"\subsection{Archive 3 실험 목록}",
        latex_longtable(["실험", "MAE", "RMSE", "Params", "MB", "Latency", "MaxFreq"], archive_inventory_rows(a3), r">{\raggedright\arraybackslash}p{0.42\linewidth}>{\raggedright\arraybackslash}p{0.08\linewidth}>{\raggedright\arraybackslash}p{0.08\linewidth}>{\raggedright\arraybackslash}p{0.12\linewidth}>{\raggedright\arraybackslash}p{0.08\linewidth}>{\raggedright\arraybackslash}p{0.08\linewidth}>{\raggedright\arraybackslash}p{0.08\linewidth}", "Archive 3 experiment inventory"),
        r"\subsection{Archive 4 실험 목록}",
        latex_longtable(["실험", "MAE", "RMSE", "Params", "MB", "Latency", "MaxFreq"], archive_inventory_rows(a4), r">{\raggedright\arraybackslash}p{0.42\linewidth}>{\raggedright\arraybackslash}p{0.08\linewidth}>{\raggedright\arraybackslash}p{0.08\linewidth}>{\raggedright\arraybackslash}p{0.12\linewidth}>{\raggedright\arraybackslash}p{0.08\linewidth}>{\raggedright\arraybackslash}p{0.08\linewidth}>{\raggedright\arraybackslash}p{0.08\linewidth}", "Archive 4 experiment inventory"),
        r"\subsection{Archive 5 실험 목록}",
        latex_longtable(["실험", "MAE", "RMSE", "Params", "MB", "Latency", "MaxFreq"], archive_inventory_rows(a5), r">{\raggedright\arraybackslash}p{0.42\linewidth}>{\raggedright\arraybackslash}p{0.08\linewidth}>{\raggedright\arraybackslash}p{0.08\linewidth}>{\raggedright\arraybackslash}p{0.12\linewidth}>{\raggedright\arraybackslash}p{0.08\linewidth}>{\raggedright\arraybackslash}p{0.08\linewidth}>{\raggedright\arraybackslash}p{0.08\linewidth}", "Archive 5 experiment inventory"),
        r"\subsection{Archive 6 screening 실험 목록}",
        latex_longtable(["실험", "MAE", "RMSE", "Params", "MB", "Latency", "MaxFreq"], archive_inventory_rows(a6_screen), r">{\raggedright\arraybackslash}p{0.42\linewidth}>{\raggedright\arraybackslash}p{0.08\linewidth}>{\raggedright\arraybackslash}p{0.08\linewidth}>{\raggedright\arraybackslash}p{0.12\linewidth}>{\raggedright\arraybackslash}p{0.08\linewidth}>{\raggedright\arraybackslash}p{0.08\linewidth}>{\raggedright\arraybackslash}p{0.08\linewidth}", "Archive 6 screening inventory"),
        r"\subsection{Archive 7 screening 실험 목록}",
        latex_longtable(["실험", "MAE", "RMSE", "Params", "MB", "Latency", "MaxFreq"], archive_inventory_rows(a7), r">{\raggedright\arraybackslash}p{0.42\linewidth}>{\raggedright\arraybackslash}p{0.08\linewidth}>{\raggedright\arraybackslash}p{0.08\linewidth}>{\raggedright\arraybackslash}p{0.12\linewidth}>{\raggedright\arraybackslash}p{0.08\linewidth}>{\raggedright\arraybackslash}p{0.08\linewidth}>{\raggedright\arraybackslash}p{0.08\linewidth}", "Archive 7 screening inventory"),
        r"\subsection{Archive 8 screening 실험 목록}",
        latex_longtable(["실험", "MAE", "RMSE", "Params", "MB", "Latency", "MaxFreq"], archive_inventory_rows(a8), r">{\raggedright\arraybackslash}p{0.42\linewidth}>{\raggedright\arraybackslash}p{0.08\linewidth}>{\raggedright\arraybackslash}p{0.08\linewidth}>{\raggedright\arraybackslash}p{0.12\linewidth}>{\raggedright\arraybackslash}p{0.08\linewidth}>{\raggedright\arraybackslash}p{0.08\linewidth}>{\raggedright\arraybackslash}p{0.08\linewidth}", "Archive 8 screening inventory"),
        r"\subsection{Archive 6 full LOSO inventory}",
        latex_longtable(["모델", "MAE mean", "MAE std", "RMSE mean", "Params", "MB", "Latency", "MaxFreq"], full_inventory_rows(fl6), r">{\raggedright\arraybackslash}p{0.30\linewidth}>{\raggedright\arraybackslash}p{0.08\linewidth}>{\raggedright\arraybackslash}p{0.08\linewidth}>{\raggedright\arraybackslash}p{0.10\linewidth}>{\raggedright\arraybackslash}p{0.12\linewidth}>{\raggedright\arraybackslash}p{0.08\linewidth}>{\raggedright\arraybackslash}p{0.08\linewidth}>{\raggedright\arraybackslash}p{0.08\linewidth}", "Archive 6 full LOSO inventory"),
        r"\subsection{Archive 7 full LOSO inventory}",
        latex_longtable(["모델", "MAE mean", "MAE std", "RMSE mean", "Params", "MB", "Latency", "MaxFreq"], full_inventory_rows(fl7), r">{\raggedright\arraybackslash}p{0.30\linewidth}>{\raggedright\arraybackslash}p{0.08\linewidth}>{\raggedright\arraybackslash}p{0.08\linewidth}>{\raggedright\arraybackslash}p{0.10\linewidth}>{\raggedright\arraybackslash}p{0.12\linewidth}>{\raggedright\arraybackslash}p{0.08\linewidth}>{\raggedright\arraybackslash}p{0.08\linewidth}>{\raggedright\arraybackslash}p{0.08\linewidth}", "Archive 7 full LOSO inventory"),
        r"\subsection{Archive 8 full LOSO inventory}",
        latex_longtable(["모델", "MAE mean", "MAE std", "RMSE mean", "Params", "MB", "Latency", "MaxFreq"], full_inventory_rows(fl8), r">{\raggedright\arraybackslash}p{0.30\linewidth}>{\raggedright\arraybackslash}p{0.08\linewidth}>{\raggedright\arraybackslash}p{0.08\linewidth}>{\raggedright\arraybackslash}p{0.10\linewidth}>{\raggedright\arraybackslash}p{0.12\linewidth}>{\raggedright\arraybackslash}p{0.08\linewidth}>{\raggedright\arraybackslash}p{0.08\linewidth}>{\raggedright\arraybackslash}p{0.08\linewidth}", "Archive 8 full LOSO inventory"),
        r"\end{document}",
    ]

    write_and_compile_latex("\n\n".join(tex_parts))


if __name__ == "__main__":
    build_report_latex()
