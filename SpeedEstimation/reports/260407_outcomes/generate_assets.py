from __future__ import annotations

import json
import os
import sys
from pathlib import Path

ROOT = Path("/home/chanyoungko/IIT/SpeedEstimation")
os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib-speedest")
sys.path.insert(0, str(ROOT))

import h5py
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import torch
import yaml
from torch.utils.data import DataLoader
from torch.utils.data import Subset

from core.datasets import LazyWindowDataset
from core.models import build_model_from_cfg
from data.loader import extract_condition_data_v2, fs
def parse_vars(var_list_from_yaml):
    parsed = []
    for item in var_list_from_yaml:
        if isinstance(item, list) and len(item) == 2:
            parsed.append((item[0], item[1]))
        elif isinstance(item, tuple) and len(item) == 2:
            parsed.append(item)
    return parsed


def get_feature_names(cfg, input_vars):
    return [f"{gp}/{v}" for gp, vs in input_vars for v in vs]


def parse_subject_prefix(s):
    if s.startswith("m1_"):
        return "m1_", s[3:]
    if s.startswith("m2_"):
        return "m2_", s[3:]
    if s.startswith("m3_"):
        return "m3_", s[3:]
    return None, s


def get_data_sources_from_config(cfg):
    sources = {}
    raw_sources = cfg.get("shared", {}).get("data_sources", {}) or cfg.get("data_sources", {})
    if not isinstance(raw_sources, dict):
        return sources
    for _, val in raw_sources.items():
        if isinstance(val, dict) and "prefix" in val:
            sources[val["prefix"]] = val
    return sources

REPORT_DIR = ROOT / "reports" / "260407_outcomes"
ASSET_DIR = REPORT_DIR / "assets"


def ensure_dirs():
    REPORT_DIR.mkdir(parents=True, exist_ok=True)
    ASSET_DIR.mkdir(parents=True, exist_ok=True)


def focal_models():
    return [
        ("A1_best_raw", "exp_S3A2_A1_teacher_raw_full_Test-m2_S008_seed42"),
        ("A2_best_imuint", "exp_S3A2_A2_teacher_imuint_full_Test-m2_S008_seed42"),
        ("B4_interaction_contribution", "exp_S3A2_B4_deploy_plus_interaction_Test-m2_S008_seed42"),
        ("C2_compact_fusion_contribution", "exp_S3A2_C2_fusion_imu_stride_compact_Test-m2_S008_seed42"),
    ]


def load_summary_rows():
    rows = []
    for label, exp_name in focal_models():
        exp_dir = ROOT / "experiments" / exp_name
        metrics = json.load(open(exp_dir / "metrics.json"))
        row = {"label": label, "exp_name": exp_name}
        row.update(metrics)
        rows.append(row)
    df = pd.DataFrame(rows)
    df.to_csv(ASSET_DIR / "selected_model_summary.csv", index=False)
    return df


def make_comparison_figures(df: pd.DataFrame):
    plot_df = df.sort_values("test_mae")
    colors = ["#2f5597", "#5b9bd5", "#ed7d31", "#70ad47"]

    plt.figure(figsize=(10, 5))
    plt.bar(plot_df["label"], plot_df["test_mae"], color=colors)
    plt.ylabel("MAE")
    plt.title("Selected models: MAE")
    plt.xticks(rotation=15, ha="right")
    plt.tight_layout()
    plt.savefig(ASSET_DIR / "selected_models_mae.png", dpi=180)
    plt.close()

    plt.figure(figsize=(10, 5))
    plt.bar(plot_df["label"], plot_df["n_params"], color=colors)
    plt.ylabel("Number of parameters")
    plt.title("Selected models: parameter count")
    plt.xticks(rotation=15, ha="right")
    plt.tight_layout()
    plt.savefig(ASSET_DIR / "selected_models_params.png", dpi=180)
    plt.close()

    plt.figure(figsize=(10, 5))
    plt.scatter(df["n_params"], df["test_mae"], s=120, c=colors)
    for _, r in df.iterrows():
        plt.annotate(r["label"], (r["n_params"], r["test_mae"]), xytext=(6, 4), textcoords="offset points")
    plt.xlabel("Number of parameters")
    plt.ylabel("MAE")
    plt.title("Accuracy-efficiency tradeoff")
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(ASSET_DIR / "selected_models_pareto.png", dpi=180)
    plt.close()


def _collect_test_windows(cfg: dict, exp_dir: Path):
    scaler = np.load(exp_dir / "scaler.npz")
    mean = scaler["mean"]
    scale = scaler["scale"]

    input_vars = parse_vars(cfg["shared"]["input_vars"])
    output_vars = parse_vars(cfg["shared"]["output_vars"])
    feature_names = get_feature_names(cfg, input_vars)

    sub_name = cfg["03_eval"]["split"]["loso_subjects"][0]
    prefix_found, real_sub = parse_subject_prefix(sub_name)
    data_sources = get_data_sources_from_config(cfg)
    if prefix_found and prefix_found in data_sources:
        data_path = data_sources[prefix_found]["path"]
    else:
        data_path = str(ROOT / "combined_data_S008.h5")

    x_list, y_list = [], []
    with h5py.File(data_path, "r") as f:
        available_conds = list(f[real_sub].keys())
        actual_conds = []
        for c in cfg["shared"]["conditions"]:
            if c in available_conds:
                actual_conds.append(c)
            elif c == "level":
                actual_conds.extend([ac for ac in available_conds if ac.startswith("level_")])
            else:
                actual_conds.extend([ac for ac in available_conds if ac.startswith(c)])
        actual_conds = list(dict.fromkeys(actual_conds))

        for cond in actual_conds:
            xs, ys = extract_condition_data_v2(
                f, real_sub, cond,
                [(gp, vs) for gp, vs in input_vars],
                [(gp, vs) for gp, vs in output_vars],
                lpf_cutoff=cfg["shared"].get("lpf_cutoff", 0.5),
                lpf_order=cfg["shared"].get("lpf_order", 4),
                fs=fs,
                input_lpf_per_group=cfg.get("02_train", {}).get("data", {}).get("input_lpf_per_group"),
            )
            x_list.extend(xs)
            y_list.extend(ys)

    x = np.concatenate(x_list, axis=0)
    y = np.concatenate(y_list, axis=0)
    x = (x - mean) / (scale + 1e-8)

    data_cfg = cfg["02_train"]["data"]
    ds = LazyWindowDataset(
        [x], [y],
        data_cfg["window_size"],
        len(data_cfg.get("est_tick_ranges", [5])),
        3,
        target_mode="sequence",
        est_tick_ranges=data_cfg.get("est_tick_ranges", [5]),
    )
    max_windows = 1024
    if len(ds) > max_windows:
        idx = np.linspace(0, len(ds) - 1, max_windows, dtype=int)
        ds = Subset(ds, idx.tolist())
    loader = DataLoader(ds, batch_size=256, shuffle=False, num_workers=0)
    return loader, feature_names, x.shape[1]


def calculate_sampled_permutation_importance(model, dataloader, feature_names, device, max_batches=2):
    model.eval()
    xs, ys = [], []
    for i, (xb, yb) in enumerate(dataloader):
        xs.append(xb)
        ys.append(yb)
        if i + 1 >= max_batches:
            break

    xb = torch.cat(xs, dim=0).to(device)
    yb = torch.cat(ys, dim=0).to(device)

    with torch.no_grad():
        pred_base = model(xb)
        if pred_base.dim() == 3 and pred_base.shape[1] == 1:
            pred_base = pred_base.squeeze(1)
        if yb.dim() == 3 and yb.shape[1] == 1:
            yb_s = yb.squeeze(1)
        else:
            yb_s = yb
        base_mae = torch.mean(torch.abs(pred_base - yb_s)).item()

        importances = []
        for i in range(len(feature_names)):
            xb_perm = xb.clone()
            idx = torch.randperm(xb_perm.size(0), device=device)
            xb_perm[:, :, i] = xb_perm[idx, :, i]
            pred_perm = model(xb_perm)
            if pred_perm.dim() == 3 and pred_perm.shape[1] == 1:
                pred_perm = pred_perm.squeeze(1)
            perm_mae = torch.mean(torch.abs(pred_perm - yb_s)).item()
            importances.append(perm_mae - base_mae)
    return np.array(importances, dtype=np.float64), base_mae


def compute_feature_importance():
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    top_rows = []

    for label, exp_name in focal_models():
        print(f"[FI] {label}")
        exp_dir = ROOT / "experiments" / exp_name
        cfg = yaml.safe_load(open(exp_dir / "config.yaml"))
        loader, feature_names, input_dim = _collect_test_windows(cfg, exp_dir)

        model_cfg = cfg["02_train"]["model"]
        model = build_model_from_cfg(
            model_cfg,
            input_dim=input_dim,
            output_dim=1,
            horizon=1,
            use_input_norm=cfg["02_train"]["data"].get("normalize", True),
        )
        ckpt = torch.load(exp_dir / "model.pt", map_location=device)
        model.load_state_dict(ckpt["state_dict"] if isinstance(ckpt, dict) and "state_dict" in ckpt else ckpt, strict=False)
        model.to(device).eval()

        imps, base_mae = calculate_sampled_permutation_importance(model, loader, feature_names, device, max_batches=2)
        imp_df = pd.DataFrame({"feature": feature_names, "importance": imps}).sort_values("importance", ascending=False)
        imp_df["base_mae_sampled"] = base_mae
        imp_df.to_csv(ASSET_DIR / f"{label}_feature_importance.csv", index=False)

        top20 = imp_df.head(20).iloc[::-1]
        plt.figure(figsize=(10, max(6, 0.32 * len(top20))))
        plt.barh(top20["feature"], top20["importance"], color="#4e79a7")
        plt.xlabel("MAE increase")
        plt.title(f"Feature importance: {label}")
        plt.tight_layout()
        plt.savefig(ASSET_DIR / f"{label}_feature_importance_top20.png", dpi=180)
        plt.close()

        for _, row in imp_df.head(20).iterrows():
            top_rows.append({
                "label": label,
                "feature": row["feature"],
                "importance": float(row["importance"]),
            })

    pd.DataFrame(top_rows).to_csv(ASSET_DIR / "selected_model_feature_importance_top20.csv", index=False)


def main():
    ensure_dirs()
    df = load_summary_rows()
    make_comparison_figures(df)
    compute_feature_importance()
    print(f"[DONE] assets written to {ASSET_DIR}")


if __name__ == "__main__":
    main()
