import os
import gc
import copy
import time
import sys
import platform
import datetime
import random
import json

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
import matplotlib.pyplot as plt
import yaml
from pathlib import Path
from torch.utils.data import DataLoader, WeightedRandomSampler
from tqdm import tqdm

from core.models import build_model_from_cfg
from core.losses import get_custom_losses
from core.datasets import LazyWindowDataset, inverse_target_transform
from data.loader import get_dataset_metadata


def build_condition_sampler(dataset, dataset_meta, condition_weights):
    if not condition_weights or not dataset_meta:
        return None

    weights = []
    unique_conds = set()
    for sample_idx, _ in dataset.indices:
        cond = dataset_meta[sample_idx].get("condition")
        unique_conds.add(cond)
        weights.append(float(condition_weights.get(cond, 1.0)))

    if not weights or max(weights) == min(weights) == 1.0:
        return None

    print(f"[INFO] Condition-weighted sampling enabled: {condition_weights}")
    print(f"[INFO] Observed training conditions: {sorted(c for c in unique_conds if c is not None)}")
    return WeightedRandomSampler(
        weights=torch.as_tensor(weights, dtype=torch.double),
        num_samples=len(weights),
        replacement=True,
    )


def compute_overlap_consistency_loss(pred, source_idx, start_idx, input_window, est_tick_ranges=None, window_output=1):
    if source_idx is None or start_idx is None:
        return torch.tensor(0.0, device=pred.device)

    if pred.dim() == 2:
        pred_steps = pred.unsqueeze(1)
    elif pred.dim() == 3:
        pred_steps = pred
    else:
        return torch.tensor(0.0, device=pred.device)

    if est_tick_ranges:
        offsets = [int(k) - 1 for k in est_tick_ranges]
    else:
        offsets = list(range(pred_steps.shape[1] if pred_steps.shape[1] > 1 else int(window_output)))

    groups = {}
    src_list = source_idx.detach().cpu().tolist()
    start_list = start_idx.detach().cpu().tolist()
    for batch_i, (src_i, start_i) in enumerate(zip(src_list, start_list)):
        for step_i, step_offset in enumerate(offsets[:pred_steps.shape[1]]):
            abs_idx = int(start_i) + int(input_window) + int(step_offset)
            key = (int(src_i), abs_idx)
            groups.setdefault(key, []).append(pred_steps[batch_i, step_i])

    losses = []
    for preds in groups.values():
        if len(preds) < 2:
            continue
        stacked = torch.stack(preds, dim=0)
        losses.append(torch.mean((stacked - stacked.mean(dim=0, keepdim=True)) ** 2))

    if not losses:
        return torch.tensor(0.0, device=pred.device)
    return torch.stack(losses).mean()


class LiveTrainingPlotter:
    def __init__(self):
        self.fig, self.ax = plt.subplots(figsize=(8, 5))
        self.ax.set_xlabel('Epoch')
        self.ax.set_ylabel('Loss')
        self.ax.grid(True)
        plt.ion()
        plt.show(block=False)
        self.train_line, = self.ax.plot([], [], 'b-', label='Train Loss')
        self.val_line,   = self.ax.plot([], [], 'r--', label='Val Loss')
        self.ax.legend()
        self.epochs = []
        self.train_losses = []
        self.val_losses = []

    def start_session(self, model_name, seed):
        self.ax.clear()
        self.ax.set_xlabel('Epoch')
        self.ax.set_ylabel('Loss')
        self.ax.set_title(f"Training {model_name} (seed={seed})")
        self.ax.grid(True)
        self.train_line, = self.ax.plot([], [], 'b-', label='Train Loss')
        self.val_line,   = self.ax.plot([], [], 'r--', label='Val Loss')
        self.ax.legend()
        self.epochs = []
        self.train_losses = []
        self.val_losses = []
        plt.pause(0.1)

    def update_epoch(self, epoch, train_loss, val_loss):
        self.epochs.append(epoch)
        self.train_losses.append(train_loss)
        self.val_losses.append(val_loss)
        self.train_line.set_data(self.epochs, self.train_losses)
        self.val_line.set_data(self.epochs, self.val_losses)
        self.ax.relim()
        self.ax.autoscale_view()
        self.fig.canvas.draw()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()


class ExperimentLogger:
    def __init__(self, log_dir):
        self.log_dir = Path(log_dir)
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.csv_path = self.log_dir / "train_log.csv"
        self.log_file = open(self.csv_path, "w")
        self.log_file.write("epoch,train_loss,val_loss,lr,time\n")

    def log_epoch(self, epoch, train_loss, val_loss, lr, duration):
        self.log_file.write(f"{epoch},{train_loss:.6f},{val_loss:.6f},{lr:.2e},{duration:.2f}\n")
        self.log_file.flush()

    def close(self):
        self.log_file.close()

    def save_env(self):
        with open(self.log_dir / "env.txt", "w") as f:
            f.write(f"Date: {datetime.datetime.now()}\n")
            f.write(f"Python: {sys.version}\n")
            f.write(f"Platform: {platform.platform()}\n")

    def save_metrics(self, metrics_dict):
        def convert(o):
            if isinstance(o, np.integer): return int(o)
            if isinstance(o, np.floating): return float(o)
            if isinstance(o, np.ndarray): return o.tolist()
            return str(o)

        with open(self.log_dir / "metrics.json", "w") as f:
            json.dump(metrics_dict, f, indent=4, default=convert)

    def save_config(self, config):
        with open(self.log_dir / "config.yaml", "w") as f:
            yaml.dump(config, f, default_flow_style=False)


def _count_input_dim(cfg):
    input_vars = cfg.get("shared", {}).get("input_vars") or cfg.get("input_vars", [])
    count = sum(len(vs) for _, vs in input_vars)
    use_gait_phase = cfg.get("02_train", {}).get("data", {}).get("use_gait_phase") or cfg.get("use_gait_phase")
    if use_gait_phase:
        count += 2
    return count


def _feature_names_from_cfg(cfg):
    input_vars = cfg.get("shared", {}).get("input_vars") or cfg.get("input_vars", [])
    names = [f"{gp}/{v}" for gp, vs in input_vars for v in vs]
    use_gait_phase = cfg.get("02_train", {}).get("data", {}).get("use_gait_phase") or cfg.get("use_gait_phase")
    if use_gait_phase:
        names.extend(["derived/gait_phase_sin", "derived/gait_phase_cos"])
    return names


def _denorm_batch(x_norm, mean_t, std_t):
    if mean_t is None or std_t is None:
        return x_norm
    return x_norm * std_t.view(1, 1, -1) + mean_t.view(1, 1, -1)


def _norm_batch(x_raw, mean_t, std_t):
    if mean_t is None or std_t is None:
        return x_raw
    return (x_raw - mean_t.view(1, 1, -1)) / std_t.view(1, 1, -1)


def _align_pred_like(student_pred, teacher_pred):
    if student_pred.dim() == 2 and teacher_pred.dim() == 3 and teacher_pred.shape[1] == 1:
        teacher_pred = teacher_pred.squeeze(1)
    elif student_pred.dim() == 3 and student_pred.shape[1] == 1 and teacher_pred.dim() == 2:
        teacher_pred = teacher_pred.unsqueeze(1)
    return teacher_pred


def _load_teacher_bundle(train_cfg, device, scaler_mean, scaler_std, student_feature_names=None):
    distill_weight = float(train_cfg.get("distill_weight", 0.0))
    exp_path = train_cfg.get("distill_teacher_experiment")
    if distill_weight <= 0 or not exp_path:
        return None

    exp_dir = Path(exp_path)
    teacher_cfg_path = exp_dir / "config.yaml"
    teacher_ckpt_path = exp_dir / "model.pt"
    if not teacher_cfg_path.exists() or not teacher_ckpt_path.exists():
        raise FileNotFoundError(f"Teacher experiment is missing config/model: {exp_dir}")

    with open(teacher_cfg_path, "r") as f:
        teacher_cfg = yaml.safe_load(f)

    teacher_input_dim = _count_input_dim(teacher_cfg)
    teacher_output_vars = teacher_cfg.get("shared", {}).get("output_vars") or teacher_cfg.get("output_vars", [])
    teacher_output_dim = sum(len(vs) for _, vs in teacher_output_vars) or 1
    teacher_data_cfg = teacher_cfg.get("02_train", {}).get("data", {})
    teacher_est_tick_ranges = teacher_data_cfg.get("est_tick_ranges")
    teacher_horizon = len(teacher_est_tick_ranges) if teacher_est_tick_ranges else int(teacher_data_cfg.get("time_window_output", 1))
    teacher_model_cfg = teacher_cfg.get("02_train", {}).get("model") or teacher_cfg.get("shared", {}).get("model") or teacher_cfg.get("model", {})
    teacher_use_input_norm = teacher_data_cfg.get("normalize", True)

    teacher_model = build_model_from_cfg(
        teacher_model_cfg,
        input_dim=teacher_input_dim,
        output_dim=teacher_output_dim,
        horizon=teacher_horizon,
        use_input_norm=teacher_use_input_norm,
    )
    teacher_ckpt = torch.load(teacher_ckpt_path, map_location=device)
    teacher_model.load_state_dict(teacher_ckpt["state_dict"] if "state_dict" in teacher_ckpt else teacher_ckpt, strict=False)
    teacher_model.to(device)
    teacher_model.eval()
    for p in teacher_model.parameters():
        p.requires_grad = False

    teacher_feature_names = _feature_names_from_cfg(teacher_cfg)

    teacher_scaler_path = exp_dir / "scaler.npz"
    teacher_mean_t = teacher_std_t = None
    if teacher_scaler_path.exists():
        s = np.load(teacher_scaler_path)
        teacher_mean_t = torch.as_tensor(s["mean"], dtype=torch.float32, device=device)
        teacher_std_t = torch.as_tensor(s["scale"], dtype=torch.float32, device=device)

    student_mean_t = None if scaler_mean is None else torch.as_tensor(scaler_mean, dtype=torch.float32, device=device)
    student_std_t = None if scaler_std is None else torch.as_tensor(scaler_std, dtype=torch.float32, device=device)

    student_feature_names = student_feature_names or []
    student_to_teacher = []
    if student_feature_names and teacher_feature_names:
        teacher_name_to_idx = {name: i for i, name in enumerate(teacher_feature_names)}
        for s_idx, name in enumerate(student_feature_names):
            t_idx = teacher_name_to_idx.get(name)
            if t_idx is not None:
                student_to_teacher.append((s_idx, t_idx))
    elif student_mean_t is not None and teacher_mean_t is not None and len(student_mean_t) == len(teacher_mean_t):
        student_to_teacher = [(i, i) for i in range(len(student_mean_t))]

    if not student_to_teacher:
        raise ValueError("No overlapping teacher/student features found for distillation")

    print(
        f"[Distill] Teacher loaded from {exp_dir} (weight={distill_weight}, "
        f"matched_features={len(student_to_teacher)}/{len(teacher_feature_names)})"
    )
    return {
        "weight": distill_weight,
        "model": teacher_model,
        "student_mean": student_mean_t,
        "student_std": student_std_t,
        "teacher_mean": teacher_mean_t,
        "teacher_std": teacher_std_t,
        "teacher_input_dim": teacher_input_dim,
        "student_to_teacher": student_to_teacher,
    }


def _compute_distill_loss(bundle, xb_student_norm, student_pred):
    if bundle is None:
        return None
    xb_raw = _denorm_batch(xb_student_norm, bundle["student_mean"], bundle["student_std"])
    teacher_mean = bundle["teacher_mean"]
    teacher_std = bundle["teacher_std"]
    teacher_dim = int(bundle["teacher_input_dim"])
    batch_size, seq_len, _ = xb_raw.shape

    if teacher_mean is not None:
        xb_teacher_raw = teacher_mean.view(1, 1, -1).expand(batch_size, seq_len, -1).clone()
    else:
        xb_teacher_raw = torch.zeros(batch_size, seq_len, teacher_dim, dtype=xb_raw.dtype, device=xb_raw.device)

    for s_idx, t_idx in bundle["student_to_teacher"]:
        xb_teacher_raw[:, :, t_idx] = xb_raw[:, :, s_idx]

    xb_teacher = _norm_batch(xb_teacher_raw, teacher_mean, teacher_std)
    with torch.no_grad():
        teacher_pred = bundle["model"](xb_teacher)
    teacher_pred = _align_pred_like(student_pred, teacher_pred)
    return F.mse_loss(student_pred, teacher_pred)


def plot_model_summary(results):
    """
    results: list of (exp_name, seed, metrics_data, ckpt, ...)
    metrics_data: either float (old style loss) or dict (new style full metrics)
    """
    data = {}

    for row in results:
        name = row[0]
        metrics_val = row[2]

        if name not in data:
            data[name] = {'huber': [], 'mae': [], 'rmse': [], 'params': []}

        if isinstance(metrics_val, dict):
            data[name]['huber'].append(metrics_val.get('test_huber', 0.0))
            data[name]['mae'].append(metrics_val.get('test_mae', 0.0))
            data[name]['rmse'].append(metrics_val.get('test_rmse', 0.0))
            data[name]['params'].append(metrics_val.get('n_params', 0))
        else:
            data[name]['huber'].append(float(metrics_val))

    names = sorted(data.keys())

    means = {k: [] for k in ['huber', 'mae', 'rmse', 'params']}
    stds  = {k: [] for k in ['huber', 'mae', 'rmse', 'params']}

    print("\n=== Experiment Summary ===")
    for n in names:
        d = data[n]
        print(f"[{n}]")
        for k in means.keys():
            if len(d[k]) > 0:
                m = np.mean(d[k])
                s = np.std(d[k])
                means[k].append(m)
                stds[k].append(s)
                print(f"  {k}: {m:.6f} (+/- {s:.6f})")
            else:
                means[k].append(0)
                stds[k].append(0)

    fig, axes = plt.subplots(2, 2, figsize=(16, 12))

    ax = axes[0, 0]
    x_pos = np.arange(len(names))
    ax.bar(x_pos, means['huber'], yerr=stds['huber'], align='center', alpha=0.7, capsize=10, color='skyblue')
    ax.set_xticks(x_pos)
    ax.set_xticklabels(names, rotation=45, ha='right')
    ax.set_ylabel('Huber Loss')
    ax.set_title('Test Huber Loss')
    ax.grid(axis='y', linestyle='--', alpha=0.5)

    ax = axes[0, 1]
    ax.bar(x_pos, means['mae'], yerr=stds['mae'], align='center', alpha=0.7, capsize=10, color='lightgreen')
    ax.set_xticks(x_pos)
    ax.set_xticklabels(names, rotation=45, ha='right')
    ax.set_ylabel('MAE')
    ax.set_title('Test MAE')
    ax.grid(axis='y', linestyle='--', alpha=0.5)

    ax = axes[1, 0]
    ax.bar(x_pos, means['rmse'], yerr=stds['rmse'], align='center', alpha=0.7, capsize=10, color='salmon')
    ax.set_xticks(x_pos)
    ax.set_xticklabels(names, rotation=45, ha='right')
    ax.set_ylabel('RMSE')
    ax.set_title('Test RMSE')
    ax.grid(axis='y', linestyle='--', alpha=0.5)

    ax = axes[1, 1]
    colors = plt.cm.get_cmap('tab10', len(names))

    for i, n in enumerate(names):
        p_mean = means['params'][i]
        mae_mean = means['mae'][i]
        ax.scatter(p_mean, mae_mean, s=100, color=colors(i), label=n, edgecolors='k')

    ax.set_xlabel('# Parameters')
    ax.set_ylabel('Test MAE')
    ax.set_title('Parameters vs. Performance (MAE)')
    ax.grid(True, linestyle='--', alpha=0.5)
    ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')

    plt.tight_layout()
    return fig


def calculate_model_resources(model, input_dim, input_window, device):
    """
    Calculates parameters, model size, and estimates inference latency on the current device.
    """
    param_count = sum(p.numel() for p in model.parameters())
    param_size_mb = param_count * 4 / (1024 * 1024)

    dummy_input = torch.randn(1, input_window, input_dim).to(device)

    model.eval()

    with torch.no_grad():
        for _ in range(20):
            _ = model(dummy_input)

    t0 = time.time()
    n_loops = 100
    with torch.no_grad():
        for _ in range(n_loops):
            _ = model(dummy_input)
    t1 = time.time()

    avg_latency_ms = (t1 - t0) / n_loops * 1000.0
    max_freq_hz = 1000.0 / avg_latency_ms if avg_latency_ms > 0 else 0.0

    return {
        "n_params": param_count,
        "model_size_mb": param_size_mb,
        "host_latency_ms": avg_latency_ms,
        "host_max_freq_hz": max_freq_hz
    }


def calculate_permutation_importance(model, dataloader, feature_names, device):
    model.eval()
    N_feats = len(feature_names)
    total_samples = 0

    total_base_loss = 0.0
    feat_loss_sums = torch.zeros(N_feats, device=device)

    print(f"  Calculating Importances (Batch-wise, {N_feats} features)...")

    with torch.no_grad():
        for xb, yb in dataloader:
            xb, yb = xb.to(device), yb.to(device)
            B = xb.size(0)
            total_samples += B

            pred_base = model(xb)

            if pred_base.dim() == 2 and yb.dim() == 3 and yb.shape[1] == 1:
                yb_s = yb.squeeze(1)
            elif pred_base.dim() == 3 and yb.dim() == 2 and pred_base.shape[1] == 1:
                pred_base = pred_base.squeeze(1)
                yb_s = yb
            else:
                yb_s = yb

            base_loss = torch.sum(torch.abs(pred_base - yb_s)).item()
            total_base_loss += base_loss

            for i in range(N_feats):
                original_col = xb[:, :, i].clone()

                idx = torch.randperm(B, device=device)
                xb[:, :, i] = xb[idx, :, i]

                pred_p = model(xb)
                if pred_p.dim() == 3 and pred_p.shape[1] == 1: pred_p = pred_p.squeeze(1)

                p_loss = torch.sum(torch.abs(pred_p - yb_s))
                feat_loss_sums[i] += p_loss

                xb[:, :, i] = original_col

    base_mae = total_base_loss / (total_samples + 1e-9)
    print(f"  Baseline MAE: {base_mae:.6f}")

    feat_maes = feat_loss_sums.cpu().numpy() / (total_samples + 1e-9)
    importances = feat_maes - base_mae

    for i, imp in enumerate(importances):
        print(f"    Feat {i}: +{imp:.6f}", end='\r')
    print("")
    return importances


def train_experiment(
    X_train, Y_train,
    X_val, Y_val,
    X_test, Y_test,
    config,
    seed=42,
    save_dir="experiments/test",
    live_plotter=None,
    scaler_mean=None,
    scaler_std=None,
    feature_names=None
):
    save_path_dir = Path(save_dir)
    save_path_dir.mkdir(parents=True, exist_ok=True)

    logger = ExperimentLogger(save_path_dir)
    logger.save_env()
    logger.save_config(config)

    ckpt_path = save_path_dir / "model.pt"

    loader_cfg = config.get("02_train", {}).get("loader") or {}
    train_cfg  = config.get("02_train", {}).get("train") or {}

    model_cfg  = config.get("02_train", {}).get("model")
    if not model_cfg:
        data_cfg = config.get("02_train", {}).get("data")
        if data_cfg and "channels" in data_cfg:
            model_cfg = data_cfg

    if not model_cfg:
        model_cfg = config.get("model") or {}

    shared_model = config.get("shared", {}).get("model", {})
    if shared_model:
        def deep_merge(base, override):
            for k, v in override.items():
                if k in base and isinstance(base[k], dict) and isinstance(v, dict):
                    deep_merge(base[k], v)
                else:
                    base[k] = v
        deep_merge(shared_model, model_cfg)
        model_cfg = shared_model

    batch_size = loader_cfg.get("batch_size") or config.get("batch_size")
    val_batch_size = loader_cfg.get("val_batch_size") or config.get("val_batch_size") or batch_size

    epochs = int(train_cfg.get("epochs") or config.get("epochs"))
    patience = int(train_cfg.get("early_stop_patience") or config.get("patience"))
    lr = float(train_cfg.get("lr") or config.get("lr"))
    weight_decay = float(train_cfg.get("wd") or config.get("weight_decay"))

    dropout_p = model_cfg.get("dropout")
    if dropout_p is None:
        if "dropout" in config.get("data", {}): dropout_p = config["data"]["dropout"]
        else: dropout_p = config.get("dropout_p", 0.5)
    dropout_p = float(dropout_p)

    use_input_norm = config.get("data", {}).get("normalize", True)
    if use_input_norm is None:
        use_input_norm = config.get("use_input_norm", True)

    model_norm_type = model_cfg.get("model_norm", None)
    tcn_norm = model_norm_type
    mlp_norm = model_norm_type
    input_norm_type = model_cfg.get("input_norm_type", model_norm_type or "layer")

    tcn_channels = model_cfg.get("channels") or config.get("tcn_channels")
    kernel_size = model_cfg.get("kernel_size") or config.get("kernel_size")

    mlp_hidden  = model_cfg.get("head_hidden") or config.get("head_hidden") or config.get("mlp_hidden")
    head_dropout = model_cfg.get("head_dropout")

    cos_T0 = config.get("cos_T0", 10)
    cos_Tmult = config.get("cos_Tmult", 1)
    eta_min = config.get("eta_min", 1e-5)

    train_cfg_inner = config.get("02_train", {}).get("train", {})
    huber_delta = float(train_cfg_inner.get("huber_delta") or config.get("huber_delta", 0.5))

    torch.manual_seed(seed)
    np.random.seed(seed)
    random.seed(seed)

    target_mode = "sequence"
    train_section_cfg = config.get("02_train", {})
    data_cfg = train_section_cfg.get("data", {})
    if not data_cfg: data_cfg = config.get("data", {})

    shared_data = config.get("shared", {}).get("data", {})
    if shared_data:
        def deep_merge_data(base, override):
            for k, v in override.items():
                if k in base and isinstance(base[k], dict) and isinstance(v, dict):
                    deep_merge_data(base[k], v)
                else:
                    base[k] = v
        deep_merge_data(data_cfg, shared_data)

    print(f"[DEBUG-R] data_cfg keys: {list(data_cfg.keys())}")

    window_size = data_cfg.get("window_size") or config.get("window_size", 200)
    window_output = data_cfg.get("window_output") or data_cfg.get("time_window_output") or config.get("window_output", 1)
    stride = data_cfg.get("stride") or config.get("stride", 5)

    est_tick_ranges = data_cfg.get("est_tick_ranges") if data_cfg else config.get("est_tick_ranges")
    target_transform = data_cfg.get("target_transform")

    print(f"[DEBUG] win_in={window_size}, win_out={window_output}, ranges={est_tick_ranges}, stride={stride}")

    actual_train_params = train_section_cfg.get("train", {})
    aug_std = actual_train_params.get("augment_noise_std", 0.0)
    noise_std_per_dim = actual_train_params.get("noise_std_per_dim")
    if noise_std_per_dim is not None:
        print(f"[INFO] Per-dim noise augmentation: {len(noise_std_per_dim)} dims")
    elif aug_std > 0:
        print(f"[INFO] Training Data Augmentation Enabled: Gaussian Noise (std={aug_std})")

    time_warp_sigma = float(actual_train_params.get("time_warp_sigma", 0.0))
    if time_warp_sigma > 0:
        print(f"[INFO] Training Data Augmentation: Time Warp (sigma={time_warp_sigma})")

    mixup_alpha = float(actual_train_params.get("mixup_alpha", 0.0))
    if mixup_alpha > 0:
        print(f"[INFO] Training Data Augmentation: MixUp (alpha={mixup_alpha})")

    condition_weights = actual_train_params.get("condition_weights")
    overlap_consistency_weight = float(actual_train_params.get("overlap_consistency_weight", 0.0))
    if overlap_consistency_weight > 0:
        print(f"[INFO] Overlap consistency loss enabled: weight={overlap_consistency_weight}")

    _ar_cfg_pre = model_cfg.get("ar_feedback", {}) if model_cfg else {}
    _ar_k_ds = _ar_cfg_pre.get("k", 1) if _ar_cfg_pre.get("enable", False) else 0
    if _ar_k_ds > 0:
        print(f"[INFO] AR Feedback: dataset will return y_prev (ar_k={_ar_k_ds})")

    # Determine device early so datasets can be pre-loaded onto GPU (eliminates per-batch CPU→GPU copies)
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"[INFO] Pre-loading dataset tensors onto {device}")

    train_ds = LazyWindowDataset(X_train, Y_train, window_size, window_output, stride, target_mode, est_tick_ranges=est_tick_ranges, augment=True, noise_std=aug_std, noise_std_per_dim=noise_std_per_dim, ar_k=_ar_k_ds, device=device, time_warp_sigma=time_warp_sigma, target_transform=target_transform, input_mean=scaler_mean, input_std=scaler_std, return_index_info=(overlap_consistency_weight > 0))
    val_ds   = LazyWindowDataset(X_val,   Y_val,   window_size, window_output, stride, target_mode, est_tick_ranges=est_tick_ranges, augment=False, ar_k=_ar_k_ds, device=device, target_transform=target_transform, input_mean=scaler_mean, input_std=scaler_std)
    test_ds  = LazyWindowDataset(X_test,  Y_test,  window_size, window_output, stride, target_mode, est_tick_ranges=est_tick_ranges, augment=False, ar_k=_ar_k_ds, device=device, target_transform=target_transform, input_mean=scaler_mean, input_std=scaler_std)

    if platform.system() == 'Linux':
        num_workers = 0
    else:
        num_workers = 0

    train_meta = get_dataset_metadata(X_train)
    train_sampler = build_condition_sampler(train_ds, train_meta, condition_weights)
    train_shuffle = train_sampler is None and overlap_consistency_weight <= 0
    if overlap_consistency_weight > 0 and train_sampler is None:
        print("[INFO] Overlap consistency uses ordered training batches (shuffle disabled).")
    elif overlap_consistency_weight > 0 and train_sampler is not None:
        print("[WARN] Overlap consistency + weighted sampler: overlap pairs may be sparse.")

    train_loader = DataLoader(train_ds, batch_size=batch_size, shuffle=train_shuffle, sampler=train_sampler, num_workers=num_workers)
    val_loader   = DataLoader(val_ds, batch_size=val_batch_size, shuffle=False, num_workers=num_workers)
    test_loader  = DataLoader(test_ds, batch_size=val_batch_size, shuffle=False, num_workers=num_workers)

    input_dim = X_train[0].shape[1]

    ar_cfg = model_cfg.get("ar_feedback", {})
    ar_enable = ar_cfg.get("enable", False)
    if ar_enable:
        ar_k = ar_cfg.get("k", 1)
        print(f"[INFO] AR Feedback enabled: +{ar_k} channel(s) to input")
        input_dim += ar_k

    if est_tick_ranges:
        horizon = len(est_tick_ranges)
    else:
        horizon = window_output
    output_dim = Y_train[0].shape[1]

    model_type = model_cfg.get("type", "TCN")
    print(f"[INFO] Initializing Model: {model_type}")
    residual_skip = model_cfg.get("residual_skip", None)
    if residual_skip and residual_skip.get("enable", False):
        print(f"[INFO] Residual skip connection enabled: input_channel_idx={residual_skip['input_channel_idx']}")
    model = build_model_from_cfg(
        model_cfg,
        input_dim=input_dim,
        output_dim=output_dim,
        horizon=horizon,
        use_input_norm=use_input_norm,
    )

    model.to(device)

    optimizer = torch.optim.AdamW(model.parameters(), lr=lr, weight_decay=weight_decay)
    scheduler = None
    if cos_T0 > 0:
        scheduler = torch.optim.lr_scheduler.CosineAnnealingWarmRestarts(
            optimizer, T_0=cos_T0, T_mult=cos_Tmult, eta_min=eta_min
        )

    cost_fn_name = config.get("cost_function", config.get("loss", "huber")).lower()

    use_weighted_loss = config.get("use_weighted_loss", False)
    loss_decay_factor = config.get("loss_decay_factor", 0.8)

    reduction_val = 'none' if use_weighted_loss else 'mean'

    if cost_fn_name == "huber":
        criterion = nn.HuberLoss(delta=huber_delta, reduction=reduction_val)
    elif cost_fn_name == "mse":
        criterion = nn.MSELoss(reduction=reduction_val)
    elif cost_fn_name in ["mae", "l1"]:
        criterion = nn.L1Loss(reduction=reduction_val)
    else:
        print(f"[WARN] Unknown cost function '{cost_fn_name}', defaulting to Huber.")
        criterion = nn.HuberLoss(delta=huber_delta, reduction=reduction_val)

    loss_weights = None
    if use_weighted_loss:
        w_t = np.array([loss_decay_factor**t for t in range(horizon)])
        loss_weights = torch.from_numpy(w_t).float().to(device)
        loss_weights = loss_weights.view(1, horizon, 1)

    custom_losses = get_custom_losses(config, device, feature_names, scaler_mean, scaler_std)
    if custom_losses:
        print(f"[Loss] Active Custom Losses: {list(custom_losses.keys())}")
    teacher_bundle = _load_teacher_bundle(train_cfg, device, scaler_mean, scaler_std, feature_names)

    best_val = float("inf")
    val_loss = float("inf")
    patience_cnt = 0

    min_delta = float(train_cfg.get("early_stop_min_delta") or config.get("early_stop_min_delta", 1e-4))
    train_loss_target = float(train_cfg.get("train_loss_target") or config.get("train_loss_target", 0.0))

    # AMP Setup (TRAIN_AMP env var, set by train.sh)
    use_amp = os.getenv("TRAIN_AMP", "0") == "1"
    amp_scaler = torch.cuda.amp.GradScaler(enabled=use_amp)
    if use_amp:
        print("[AMP] FP16 Mixed Precision enabled (GradScaler active)")

    enable_progress = sys.stdout.isatty()

    for epoch in range(1, epochs + 1):
        t0 = time.time()
        model.train()
        running = 0.0
        n_batches = 0

        tf_ratio = 1.0
        if ar_enable:
            sched = ar_cfg.get("teacher_forcing_schedule", {})
            if sched:
                tf_start = sched.get("start", 1.0)
                tf_end = sched.get("end", 0.0)
                tf_ratio = tf_start + (tf_end - tf_start) * (epoch - 1) / max(1, epochs - 1)
            else:
                tf_ratio = ar_cfg.get("teacher_forcing", 1.0)

        train_iter = enumerate(train_loader, start=1)
        if enable_progress:
            train_iter = tqdm(
                train_iter,
                total=len(train_loader),
                desc=f"Epoch {epoch}",
                leave=False,
                file=sys.stdout,
            )
        for bi, batch in train_iter:
            batch_source_idx = None
            batch_start_idx = None
            if ar_enable:
                if overlap_consistency_weight > 0:
                    xb, yb, yb_prev, batch_source_idx, batch_start_idx = batch
                else:
                    xb, yb, yb_prev = batch
                xb, yb = xb.to(device), yb.to(device)
                yb_prev = yb_prev.to(device)
            else:
                if overlap_consistency_weight > 0:
                    xb, yb, batch_source_idx, batch_start_idx = batch
                else:
                    xb, yb = batch
                xb, yb = xb.to(device), yb.to(device)

            if ar_enable:
                B, T_in, D = xb.shape
                if tf_ratio >= 1.0 or random.random() < tf_ratio:
                    prev_channel = yb_prev.unsqueeze(1).expand(B, T_in, ar_k)
                else:
                    with torch.no_grad():
                        zero_ar = torch.zeros(B, T_in, ar_k, device=device)
                        xb_no_ar = torch.cat([xb, zero_ar], dim=-1)
                        student_pred = model(xb_no_ar)
                        if student_pred.dim() == 3:
                            student_ar = student_pred[:, 0, :ar_k]
                        elif student_pred.dim() == 2:
                            student_ar = student_pred[:, :ar_k]
                        else:
                            student_ar = student_pred.unsqueeze(-1)[:, :ar_k]
                    prev_channel = student_ar.unsqueeze(1).expand(B, T_in, ar_k)
                xb = torch.cat([xb, prev_channel], dim=-1)

            # MixUp augmentation (batch-level)
            if mixup_alpha > 0 and not ar_enable:
                lam = np.random.beta(mixup_alpha, mixup_alpha)
                batch_size = xb.shape[0]
                perm = torch.randperm(batch_size, device=xb.device)
                xb = lam * xb + (1 - lam) * xb[perm]
                yb = lam * yb + (1 - lam) * yb[perm]

            optimizer.zero_grad(set_to_none=True)

            try:
                with torch.cuda.amp.autocast(enabled=use_amp):
                    pred = model(xb)

                    if pred.dim() == 2 and yb.dim() == 3 and yb.shape[1] == 1:
                        yb = yb.squeeze(1)
                    elif pred.dim() == 3 and yb.dim() == 2 and pred.shape[1] == 1:
                        pred = pred.squeeze(1)

                    if use_weighted_loss:
                        raw_loss = criterion(pred, yb)
                        weighted_loss = raw_loss * loss_weights
                        loss = weighted_loss.mean()
                    else:
                        loss = criterion(pred, yb)

                    if custom_losses:
                        for name, loss_item in custom_losses.items():
                            fn = loss_item['fn']
                            weight = loss_item['weight']

                            if name == 'smoothness':
                                loss += weight * fn(pred)
                            elif name == 'gradient_penalty':
                                loss += weight * fn(pred)
                            elif name == 'kinematic':
                                loss += weight * fn(xb, pred)
                            elif name == 'prior_alignment':
                                loss += weight * fn(xb, pred)
                            elif name == 'freq_penalty':
                                loss += weight * fn(pred, yb)
                            elif name == 'derivative_matching':
                                loss += weight * fn(pred, yb)

                    if overlap_consistency_weight > 0:
                        overlap_loss = compute_overlap_consistency_loss(
                            pred,
                            batch_source_idx,
                            batch_start_idx,
                            input_window=window_size,
                            est_tick_ranges=est_tick_ranges,
                            window_output=window_output,
                        )
                        loss += overlap_consistency_weight * overlap_loss

                    if teacher_bundle is not None:
                        distill_loss = _compute_distill_loss(teacher_bundle, xb, pred)
                        loss += teacher_bundle["weight"] * distill_loss

                amp_scaler.scale(loss).backward()
                amp_scaler.step(optimizer)
                amp_scaler.update()

            except RuntimeError as e:
                if "out of memory" in str(e):
                    raise RuntimeError(
                        f"Memory Insufficient (OOM): Batch size {batch_size} is too large. "
                        "Please reduce batch_size in config."
                    ) from e
                raise e
            running += loss.item()
            n_batches += 1

            avg = running / max(1, n_batches)
            if enable_progress:
                train_iter.set_postfix({
                    'loss': f'{avg:.4f}',
                    'val': f'{val_loss:.4f}' if val_loss != float('inf') else 'N/A'
                })

        train_loss = running / max(1, n_batches)

        model.eval()
        val_running = 0.0
        val_iter = val_loader
        if enable_progress:
            val_iter = tqdm(val_loader, desc=f"  [VAL]", leave=False, file=sys.stdout)
        with torch.no_grad():
            for batch in val_iter:
                if ar_enable:
                    xb, yb, yb_prev = batch
                    xb, yb = xb.to(device), yb.to(device)
                    yb_prev = yb_prev.to(device)
                    B, T_in, _ = xb.shape
                    prev_channel = yb_prev.unsqueeze(1).expand(B, T_in, ar_k)
                    xb = torch.cat([xb, prev_channel], dim=-1)
                else:
                    xb, yb = batch
                    xb, yb = xb.to(device), yb.to(device)

                pred = model(xb)

                if use_weighted_loss:
                    raw_loss = criterion(pred, yb)
                    weighted_loss = raw_loss * loss_weights
                    loss = weighted_loss.mean()
                else:
                    loss = criterion(pred, yb)

                val_running += loss.item()
                if enable_progress:
                    val_iter.set_postfix({'v_loss': f'{val_running/max(1, len(val_loader)):.4f}'})
        val_loss = val_running / max(1, len(val_loader))

        if scheduler:
            scheduler.step()

        cur_lr = optimizer.param_groups[0]["lr"]
        epoch_time = time.time() - t0
        print(f"[Epoch {epoch:03d}/{epochs:03d}] train={train_loss:.6f} val={val_loss:.6f} lr={cur_lr:.2e} time={epoch_time:.1f}s")

        logger.log_epoch(epoch, train_loss, val_loss, cur_lr, epoch_time)

        if live_plotter:
            live_plotter.update_epoch(epoch, train_loss, val_loss)

        if val_loss < (best_val - min_delta):
            best_val = val_loss
            patience_cnt = 0
            ckpt_obj = {
                "state_dict": model.state_dict(),
                "config": config,
                "metrics": {"best_val": best_val},
                "epoch": epoch
            }
            torch.save(ckpt_obj, ckpt_path)
            print(f"  --> Saved Best: {best_val:.6f} (Imp > {min_delta})")
        else:
            patience_cnt += 1
            if val_loss < best_val:
                pass

            if patience_cnt >= patience:
                print("  --> Early Stopping (No improvement)")
                break

        gc.collect()
        if torch.cuda.is_available():
            torch.cuda.empty_cache()

        if train_loss_target > 0 and train_loss < train_loss_target:
            print(f"  --> Train Loss Target Reached ({train_loss:.6f} < {train_loss_target}). Stopping to prevent overfitting.")
            break
    logger.close()

    ckpt = torch.load(ckpt_path, map_location=device)
    model.load_state_dict(ckpt["state_dict"], strict=False)
    model.eval()

    test_running = 0.0
    with torch.no_grad():
        for batch in test_loader:
            if ar_enable:
                xb, yb, _ = batch
                xb, yb = xb.to(device), yb.to(device)
                yb_prev_t = batch[2].to(device)
                B, T_in, _ = xb.shape
                prev_channel = yb_prev_t.unsqueeze(1).expand(B, T_in, ar_k)
                xb = torch.cat([xb, prev_channel], dim=-1)
            else:
                xb, yb = batch
                xb, yb = xb.to(device), yb.to(device)

            pred = model(xb)

            if use_weighted_loss:
                raw_loss = criterion(pred, yb)
                weighted_loss = raw_loss * loss_weights
                l = weighted_loss.mean()
            else:
                l = criterion(pred, yb)

            test_running += l.item()
    test_loss = test_running / max(1, len(test_loader))

    preds_all, trues_all = [], []
    with torch.no_grad():
        for batch in test_loader:
            if ar_enable:
                xb, yb, yb_prev_m = batch
                xb = xb.to(device)
                yb_prev_m = yb_prev_m.to(device)
                B, T_in, _ = xb.shape
                prev_channel = yb_prev_m.unsqueeze(1).expand(B, T_in, ar_k)
                xb = torch.cat([xb, prev_channel], dim=-1)
            else:
                xb, yb = batch
                xb = xb.to(device)

            pred = model(xb).cpu().numpy()

            target = yb.cpu().numpy()
            if pred.ndim == 2 and target.ndim == 3 and target.shape[1] == 1:
                target = target.squeeze(1)
            elif pred.ndim == 3 and target.ndim == 2 and pred.shape[1] == 1:
                pred = np.squeeze(pred, axis=1)

            pred = inverse_target_transform(
                pred, xb.cpu().numpy(),
                target_transform=target_transform,
                input_mean=scaler_mean,
                input_std=scaler_std,
            )
            target = inverse_target_transform(
                target, xb.cpu().numpy(),
                target_transform=target_transform,
                input_mean=scaler_mean,
                input_std=scaler_std,
            )

            preds_all.append(pred)
            trues_all.append(target)

    P = np.concatenate(preds_all, axis=0)
    T = np.concatenate(trues_all, axis=0)
    err = P - T

    test_mae = np.mean(np.abs(err))
    test_rmse = np.sqrt(np.mean(err**2))

    input_window = window_size
    res_metrics = calculate_model_resources(model, input_dim, input_window, device)

    multistep_detail = {}
    if est_tick_ranges and len(est_tick_ranges) > 1 and P.ndim == 3:
        for h_idx, h_val in enumerate(est_tick_ranges):
            h_pred = P[:, h_idx, :]
            h_true = T[:, h_idx, :]
            h_err = h_pred - h_true
            multistep_detail[f"H{h_val}_mae"] = float(np.mean(np.abs(h_err)))
            multistep_detail[f"H{h_val}_rmse"] = float(np.sqrt(np.mean(h_err**2)))
        if 5 in est_tick_ranges:
            idx5 = est_tick_ranges.index(5)
            h5_err = P[:, idx5, :] - T[:, idx5, :]
            test_mae = float(np.mean(np.abs(h5_err)))
            test_rmse = float(np.sqrt(np.mean(h5_err**2)))

    metrics_result = {
        "test_huber": test_loss,
        "test_mae": float(test_mae),
        "test_rmse": float(test_rmse),
        "n_params": res_metrics["n_params"],
        "model_size_mb": float(f"{res_metrics['model_size_mb']:.4f}"),
        "host_latency_ms": float(f"{res_metrics['host_latency_ms']:.4f}"),
        "host_max_freq_hz": float(f"{res_metrics['host_max_freq_hz']:.2f}"),
        "config": config
    }

    if multistep_detail:
        metrics_result["multistep_detail"] = multistep_detail

    logger.save_metrics(metrics_result)

    return metrics_result, ckpt_path
