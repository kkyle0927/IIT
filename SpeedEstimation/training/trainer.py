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
import matplotlib.pyplot as plt
import yaml
from pathlib import Path
from torch.utils.data import DataLoader
from tqdm import tqdm

from core.models import TCN_MLP, StanceGatedTCN, AttentionTCN, TCN_GRU_Head
from core.losses import get_custom_losses
from core.datasets import LazyWindowDataset


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

    print(f"[DEBUG] win_in={window_size}, win_out={window_output}, ranges={est_tick_ranges}, stride={stride}")

    actual_train_params = train_section_cfg.get("train", {})
    aug_std = actual_train_params.get("augment_noise_std", 0.0)
    if aug_std > 0:
        print(f"[INFO] Training Data Augmentation Enabled: Gaussian Noise (std={aug_std})")

    _ar_cfg_pre = model_cfg.get("ar_feedback", {}) if model_cfg else {}
    _ar_k_ds = _ar_cfg_pre.get("k", 1) if _ar_cfg_pre.get("enable", False) else 0
    if _ar_k_ds > 0:
        print(f"[INFO] AR Feedback: dataset will return y_prev (ar_k={_ar_k_ds})")

    # Determine device early so datasets can be pre-loaded onto GPU (eliminates per-batch CPU→GPU copies)
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"[INFO] Pre-loading dataset tensors onto {device}")

    train_ds = LazyWindowDataset(X_train, Y_train, window_size, window_output, stride, target_mode, est_tick_ranges=est_tick_ranges, augment=True, noise_std=aug_std, ar_k=_ar_k_ds, device=device)
    val_ds   = LazyWindowDataset(X_val,   Y_val,   window_size, window_output, stride, target_mode, est_tick_ranges=est_tick_ranges, augment=False, ar_k=_ar_k_ds, device=device)
    test_ds  = LazyWindowDataset(X_test,  Y_test,  window_size, window_output, stride, target_mode, est_tick_ranges=est_tick_ranges, augment=False, ar_k=_ar_k_ds, device=device)

    if platform.system() == 'Linux':
        num_workers = 0
    else:
        num_workers = 0

    train_loader = DataLoader(train_ds, batch_size=batch_size, shuffle=True, num_workers=num_workers)
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

    common_args = dict(
        input_dim=input_dim,
        output_dim=output_dim,
        horizon=horizon,
        channels=tcn_channels,
        kernel_size=kernel_size,
        dropout=dropout_p,
        head_dropout=head_dropout,
        mlp_hidden=mlp_hidden,
        use_input_norm=use_input_norm,
        input_norm_type=input_norm_type,
        tcn_norm=tcn_norm,
        mlp_norm=mlp_norm
    )

    if model_type == "StanceGatedTCN":
        gating_dim = model_cfg.get("gating_dim", 1)
        model = StanceGatedTCN(**common_args, gating_dim=gating_dim)

    elif model_type == "AttentionTCN":
        attn_type = model_cfg.get("attention_type", "temporal")
        heads = model_cfg.get("attention_heads", 4)
        model = AttentionTCN(**common_args, attention_type=attn_type, attention_heads=heads)

    elif model_type == "TCN_GRU":
        gru_h = model_cfg.get("gru_hidden", 32)
        print(f"[INFO] TCN_GRU Head: hidden={gru_h}")
        model = TCN_GRU_Head(**common_args, gru_hidden=gru_h)

    else:
        model = TCN_MLP(**common_args)

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

        pbar = tqdm(enumerate(train_loader, start=1), total=len(train_loader), desc=f"Epoch {epoch}", leave=False, file=sys.stdout)
        for bi, batch in pbar:
            if ar_enable:
                xb, yb, yb_prev = batch
                xb, yb = xb.to(device), yb.to(device)
                yb_prev = yb_prev.to(device)
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
                            elif name == 'kinematic':
                                loss += weight * fn(xb, pred)
                            elif name == 'freq_penalty':
                                loss += weight * fn(pred, yb)

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
            pbar.set_postfix({
                'loss': f'{avg:.4f}',
                'val': f'{val_loss:.4f}' if val_loss != float('inf') else 'N/A'
            })

        train_loss = running / max(1, n_batches)

        model.eval()
        val_running = 0.0
        val_pbar = tqdm(val_loader, desc=f"  [VAL]", leave=False, file=sys.stdout)
        with torch.no_grad():
            for batch in val_pbar:
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
                val_pbar.set_postfix({'v_loss': f'{val_running/max(1, len(val_loader)):.4f}'})
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
