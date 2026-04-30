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

from core.models import (
    TCN_MLP,
    StanceGatedTCN,
    AttentionTCN,
    TCN_GRU_Head,
    CausalSmoothTCN_GRU,
    MultiScaleTCN_GRU,
    DualBranchTCN_GRU,
    MultiBranchTCN_GRU,
    FiLMConditionedTCN_GRU,
    BoundedResidualTCN_GRU,
    AffineCalibTCN_GRU,
    PositiveResidualTCN_GRU,
    AssistExpertTCN_GRU,
    SlowFastCalibTCN_GRU,
    StructuredSlowFastTCN_GRU,
    AdaptiveComplementaryTCN_GRU,
    CadenceStepLengthTCN_GRU,
    PersistentBiasTCN_GRU,
    SpeedModeResidualTCN_GRU,
    ObserverStateTCN_GRU,
    FactorizedAnchorTCN_GRU,
    MemoryCalibratorTCN_GRU,
    PrivilegedKinMemoryCalibratorTCN_GRU,
    AssistSwitchHeadTCN_GRU,
    AssistOnBiasCorrectorTCN_GRU,
    SpeedScaleSwitchTCN_GRU,
    RegimeExpertTCN_GRU,
    AssistSplitTCN_GRU,
    StrideProgressionTCN_GRU,
    PDRHybridTCN_GRU,
    PredictedKinCascadeTCN_GRU,
    ProgressionDistillTCN_GRU,
    AssistDeconfoundTCN_GRU,
    MultiAnchorFusionTCN_GRU,
    AdaptiveBiasStateTCN_GRU,
    MonotoneProgressionTCN_GRU,
    TCN_GRU_MDN,
    MDNLoss,
    mdn_expected_value,
    mdn_predictive_std,
)
from core.losses import get_custom_losses
from core.datasets import LazyWindowDataset, extract_target_base, inverse_target_transform


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


def _extract_model_outputs(model_out):
    if isinstance(model_out, dict):
        pred = model_out.get("pred")
        aux = {k: v for k, v in model_out.items() if k != "pred"}
        return pred, aux
    if isinstance(model_out, (list, tuple)):
        pred = model_out[0]
        aux = {f"aux_{i}": v for i, v in enumerate(model_out[1:], start=1)}
        return pred, aux
    return model_out, {}


def _move_extra_targets_to_device(extra_targets, device):
    if extra_targets is None:
        return None
    return {
        k: (v.to(device) if torch.is_tensor(v) else torch.as_tensor(v, dtype=torch.float32, device=device))
        for k, v in extra_targets.items()
    }


def _split_primary_target(yb, primary_output_dim):
    if yb is None:
        return None
    if primary_output_dim is None:
        return yb
    return yb[..., :int(primary_output_dim)]


def _denorm_last_input_channels(xb, indices, scaler_mean, scaler_std):
    vals = []
    for idx in indices:
        ch = xb[:, -1, idx]
        if scaler_mean is not None and scaler_std is not None:
            ch = ch * scaler_std[idx] + scaler_mean[idx]
        vals.append(ch)
    return vals


def _compute_regime_sample_weights(
    X_list,
    Y_list,
    input_window,
    stride,
    est_tick_ranges,
    assist_channel_idx,
    steady_dy_threshold,
    speed_bins,
    assist_quantiles,
    hard_case_target_threshold=None,
    hard_case_assist_threshold=None,
    hard_case_multiplier=1.0,
):
    if assist_channel_idx is None:
        return None
    assist_channel_idx = int(assist_channel_idx)
    max_lookahead = max(est_tick_ranges) if est_tick_ranges else 1
    assists = []
    rows = []

    for s_idx, (x_arr, y_arr) in enumerate(zip(X_list, Y_list)):
        length = x_arr.shape[0]
        limit = length - input_window - max_lookahead + 1
        if limit <= 0:
            continue
        for t in range(0, limit, stride):
            end_idx = t + input_window - 1
            target_idx = t + input_window + (est_tick_ranges[0] - 1 if est_tick_ranges else 0)
            target_val = float(y_arr[target_idx, 0])
            prev_idx = max(target_idx - 1, 0)
            abs_dy = float(abs(y_arr[target_idx, 0] - y_arr[prev_idx, 0]))
            assist_val = float(abs(x_arr[end_idx, assist_channel_idx]))
            assists.append(assist_val)
            rows.append((target_val, abs_dy, assist_val))

    if not rows:
        return None

    q_edges = np.quantile(np.asarray(assists, dtype=np.float32), assist_quantiles).tolist()
    counts = {}
    bucket_keys = []
    for target_val, abs_dy, assist_val in rows:
        speed_bucket = int(np.digitize(target_val, speed_bins, right=False))
        assist_bucket = int(np.digitize(assist_val, q_edges, right=False))
        steady_bucket = 0 if abs_dy <= steady_dy_threshold else 1
        key = (speed_bucket, assist_bucket, steady_bucket)
        counts[key] = counts.get(key, 0) + 1
        bucket_keys.append(key)

    weights = np.asarray([1.0 / counts[key] for key in bucket_keys], dtype=np.float64)
    if (
        hard_case_target_threshold is not None
        and hard_case_assist_threshold is not None
        and hard_case_multiplier is not None
        and float(hard_case_multiplier) > 1.0
    ):
        boosted = []
        for (target_val, abs_dy, assist_val), base_w in zip(rows, weights):
            if (
                target_val >= float(hard_case_target_threshold)
                and abs_dy <= float(steady_dy_threshold)
                and assist_val >= float(hard_case_assist_threshold)
            ):
                boosted.append(base_w * float(hard_case_multiplier))
            else:
                boosted.append(base_w)
        weights = np.asarray(boosted, dtype=np.float64)
    weights = weights / np.mean(weights)
    return weights


def _compute_auxiliary_losses(aux_outputs, xb, yb, extra_targets, train_cfg, scaler_mean, scaler_std, device):
    total = torch.tensor(0.0, device=device)

    aux_cfg = train_cfg.get("assist_auxiliary") or {}
    if aux_cfg.get("enable", False) and "assist_aux" in aux_outputs:
        idxs = [int(i) for i in aux_cfg.get("channel_indices", [])]
        if idxs:
            weights = aux_cfg.get("weights", [1.0] * len(idxs))
            target = 0.0
            for w, idx in zip(weights, idxs):
                ch = xb[:, -1, idx]
                if scaler_mean is not None and scaler_std is not None:
                    ch = ch * scaler_std[idx] + scaler_mean[idx]
                target = target + float(w) * ch
            target = target.unsqueeze(-1)
            total = total + float(aux_cfg.get("weight", 0.0)) * torch.mean((aux_outputs["assist_aux"] - target) ** 2)

    kin_cfg = train_cfg.get("kinematics_auxiliary") or {}
    kin_key = str(kin_cfg.get("output_key", "aux_kinematics"))
    if kin_cfg.get("enable", False) and kin_key in aux_outputs and extra_targets is not None:
        target = extra_targets.get("aux_tail")
        if target is not None:
            pred = aux_outputs[kin_key]
            pred, target = align_pred_target_torch(pred, target)
            weight = float(kin_cfg.get("weight", 0.0))
            if weight > 0.0:
                total = total + weight * torch.mean((pred - target) ** 2)

    prog_cfg = train_cfg.get("progression_distill") or {}
    if prog_cfg.get("enable", False) and "distill_progression" in aux_outputs and extra_targets is not None:
        kin = extra_targets.get("aux_tail")
        if kin is not None:
            hip_diff = torch.abs(kin[..., 0] - kin[..., 1])
            knee_diff = torch.abs(kin[..., 2] - kin[..., 3])
            ankle_diff = torch.abs(kin[..., 4] - kin[..., 5])
            geom = (
                float(prog_cfg.get("hip_weight", 0.50)) * hip_diff
                + float(prog_cfg.get("knee_weight", 0.30)) * knee_diff
                + float(prog_cfg.get("ankle_weight", 0.20)) * ankle_diff
            )
            cad_idx = prog_cfg.get("cadence_channel_idx")
            if cad_idx is not None:
                cad_idx = int(cad_idx)
                cad = xb[:, -1, cad_idx]
                if scaler_mean is not None and scaler_std is not None:
                    cad = cad * scaler_std[cad_idx] + scaler_mean[cad_idx]
                while cad.dim() < geom.dim():
                    cad = cad.unsqueeze(-1)
            else:
                cad = torch.ones_like(geom)
            teacher = F.softplus(float(prog_cfg.get("geom_scale", 1.0)) * geom) * F.softplus(
                float(prog_cfg.get("cadence_scale", 1.0)) * cad
            )
            pred = aux_outputs["distill_progression"]
            pred, teacher = align_pred_target_torch(pred, teacher)
            weight = float(prog_cfg.get("weight", 0.0))
            if weight > 0.0:
                total = total + weight * torch.mean((pred - teacher) ** 2)

    decomp_cfg = train_cfg.get("decomposition_loss") or {}
    if decomp_cfg.get("enable", False) and extra_targets is not None:
        target_lf = extra_targets.get("target_lf")
        if target_lf is not None:
            nominal = aux_outputs.get("nominal_component")
            slow_bias = aux_outputs.get("slow_bias")
            fast_residual = aux_outputs.get("fast_residual")
            if nominal is not None and slow_bias is not None and fast_residual is not None:
                target_hf = extra_targets.get("target_hf", yb - target_lf)
                slow_target = target_lf - nominal.detach()
                fast_target = target_hf
                w_lf = float(decomp_cfg.get("lf_weight", 0.0))
                w_hf = float(decomp_cfg.get("hf_weight", 0.0))
                w_slow = float(decomp_cfg.get("slow_weight", 0.0))
                if w_lf > 0:
                    total = total + w_lf * torch.mean(((nominal + slow_bias) - target_lf) ** 2)
                if w_hf > 0:
                    total = total + w_hf * torch.mean((fast_residual - fast_target) ** 2)
                if w_slow > 0:
                    total = total + w_slow * torch.mean((slow_bias - slow_target) ** 2)

    steady_cfg = train_cfg.get("steady_bias_loss") or {}
    if steady_cfg.get("enable", False) and extra_targets is not None:
        abs_dy = extra_targets.get("target_abs_dy")
        if abs_dy is not None:
            pred = aux_outputs.get("pred_for_loss")
            if pred is not None:
                thr = float(steady_cfg.get("dy_threshold", 0.003))
                margin = float(steady_cfg.get("margin", 0.0))
                under_only = bool(steady_cfg.get("under_only", True))
                mask = (abs_dy <= thr).to(pred.dtype)
                assist_idx = steady_cfg.get("assist_channel_idx")
                if assist_idx is not None:
                    assist_idx = int(assist_idx)
                    ctx = xb[:, -1, assist_idx]
                    if scaler_mean is not None and scaler_std is not None:
                        ctx = ctx * scaler_std[assist_idx] + scaler_mean[assist_idx]
                    ctx_thr = float(steady_cfg.get("assist_threshold", 0.0))
                    ctx_scale = float(steady_cfg.get("assist_scale", 1.0))
                    ctx_gamma = float(steady_cfg.get("assist_gamma", 1.0))
                    ctx_w = torch.clamp((ctx.abs() - ctx_thr) / max(ctx_scale, 1e-6), min=0.0)
                    while ctx_w.dim() < mask.dim():
                        ctx_w = ctx_w.unsqueeze(-1)
                    mask = mask * (1.0 + ctx_gamma * ctx_w)
                denom = torch.clamp(mask.sum(), min=1.0)
                mean_err = torch.sum((pred - yb) * mask) / denom
                if under_only:
                    penalty = torch.relu(-(mean_err + margin)) ** 2
                else:
                    penalty = mean_err ** 2
                total = total + float(steady_cfg.get("weight", 0.0)) * penalty

    plateau_cfg = train_cfg.get("plateau_loss") or {}
    if plateau_cfg.get("enable", False) and extra_targets is not None:
        abs_dy = extra_targets.get("target_abs_dy")
        pred = aux_outputs.get("pred_for_loss")
        if abs_dy is not None and pred is not None:
            mask = (abs_dy <= float(plateau_cfg.get("dy_threshold", 0.003))).to(pred.dtype)
            speed_thr = plateau_cfg.get("speed_threshold")
            if speed_thr is not None:
                mask = mask * (yb >= float(speed_thr)).to(pred.dtype)
            assist_idx = plateau_cfg.get("assist_channel_idx")
            if assist_idx is not None:
                ctx = xb[:, -1, int(assist_idx)]
                if scaler_mean is not None and scaler_std is not None:
                    ctx = ctx * scaler_std[int(assist_idx)] + scaler_mean[int(assist_idx)]
                ctx = ctx.unsqueeze(-1)
                assist_thr = float(plateau_cfg.get("assist_threshold", 0.0))
                mask = mask * (ctx >= assist_thr).to(pred.dtype)
            denom = torch.clamp(mask.sum(), min=1.0)
            err = pred - yb
            mean_err = torch.sum(err * mask) / denom
            centered_pred = (pred - torch.sum(pred * mask) / denom) * mask
            centered_true = (yb - torch.sum(yb * mask) / denom) * mask
            std_pred = torch.sqrt(torch.sum(centered_pred ** 2) / denom + 1e-8)
            std_true = torch.sqrt(torch.sum(centered_true ** 2) / denom + 1e-8)
            signed_w = float(plateau_cfg.get("signed_bias_weight", 0.0))
            mae_w = float(plateau_cfg.get("mae_weight", 0.0))
            gain_w = float(plateau_cfg.get("gain_weight", 0.0))
            if signed_w > 0.0:
                if bool(plateau_cfg.get("under_only", True)):
                    total = total + signed_w * torch.relu(-(mean_err + float(plateau_cfg.get("margin", 0.0)))) ** 2
                else:
                    total = total + signed_w * (mean_err ** 2)
            if mae_w > 0.0:
                total = total + mae_w * (torch.sum(torch.abs(err) * mask) / denom)
            if gain_w > 0.0:
                total = total + gain_w * ((std_pred - std_true) ** 2)

    anchor_cfg = train_cfg.get("anchor_supervision") or {}
    if anchor_cfg.get("enable", False):
        anchor = aux_outputs.get("anchor_component")
        if anchor is not None:
            if extra_targets is not None and extra_targets.get("target_abs_dy") is not None:
                mask = (extra_targets["target_abs_dy"] <= float(anchor_cfg.get("dy_threshold", 0.004))).to(anchor.dtype)
            else:
                mask = torch.ones_like(anchor)
            speed_thr = anchor_cfg.get("speed_threshold")
            if speed_thr is not None:
                mask = mask * (yb >= float(speed_thr)).to(anchor.dtype)
            denom = torch.clamp(mask.sum(), min=1.0)
            total = total + float(anchor_cfg.get("weight", 0.0)) * (torch.sum(((anchor - yb) ** 2) * mask) / denom)

    contrast_cfg = train_cfg.get("same_speed_contrastive") or {}
    if contrast_cfg.get("enable", False):
        feature_key = contrast_cfg.get("feature_key", "nominal_component")
        feat = aux_outputs.get(feature_key)
        assist_indices = [int(i) for i in contrast_cfg.get("channel_indices", [])]
        if feat is not None and assist_indices:
            if feat.dim() > 2:
                feat = feat.reshape(feat.shape[0], -1)
            elif feat.dim() == 1:
                feat = feat.unsqueeze(-1)
            y_scalar = yb.reshape(yb.shape[0], -1).mean(dim=-1)
            assist_terms = _denorm_last_input_channels(xb, assist_indices, scaler_mean, scaler_std)
            assist_proxy = torch.stack(assist_terms, dim=0).mean(dim=0)
            speed_tol = float(contrast_cfg.get("speed_tolerance", 0.03))
            assist_margin = float(contrast_cfg.get("assist_margin", 0.01))
            max_pairs = int(contrast_cfg.get("max_pairs", 64))
            pair_penalties = []
            B = feat.shape[0]
            for i in range(B):
                for j in range(i + 1, B):
                    if abs(float(y_scalar[i] - y_scalar[j])) > speed_tol:
                        continue
                    if abs(float(assist_proxy[i] - assist_proxy[j])) < assist_margin:
                        continue
                    pair_penalties.append(torch.mean((feat[i] - feat[j]) ** 2))
                    if len(pair_penalties) >= max_pairs:
                        break
                if len(pair_penalties) >= max_pairs:
                    break
            if pair_penalties:
                total = total + float(contrast_cfg.get("weight", 0.0)) * torch.stack(pair_penalties).mean()

    mode_cfg = train_cfg.get("speed_mode_auxiliary") or {}
    if mode_cfg.get("enable", False) and "speed_mode_logits" in aux_outputs:
        logits = aux_outputs["speed_mode_logits"]
        centers = torch.as_tensor(
            mode_cfg.get("centers", [0.75, 1.00, 1.25]),
            dtype=yb.dtype,
            device=device,
        ).view(1, -1)
        y_scalar = yb.reshape(yb.shape[0], -1).mean(dim=-1, keepdim=True)
        labels = torch.argmin(torch.abs(y_scalar - centers), dim=-1)
        total = total + float(mode_cfg.get("weight", 0.0)) * F.cross_entropy(logits, labels)

    group_cfg = train_cfg.get("group_dro") or {}
    if group_cfg.get("enable", False):
        pred = aux_outputs.get("pred_for_loss")
        assist_indices = [int(i) for i in group_cfg.get("channel_indices", [])]
        if pred is not None and assist_indices:
            pred_flat = pred.reshape(pred.shape[0], -1).mean(dim=-1)
            target_flat = yb.reshape(yb.shape[0], -1).mean(dim=-1)
            point_loss = torch.abs(pred_flat - target_flat)
            assist_terms = _denorm_last_input_channels(xb, assist_indices, scaler_mean, scaler_std)
            assist_proxy = torch.stack(assist_terms, dim=0).mean(dim=0)
            speed_bins = group_cfg.get("speed_bins", [0.9, 1.15])
            assist_bins = group_cfg.get("assist_bins", [0.01, 0.03])
            if extra_targets is not None and "target_abs_dy" in extra_targets:
                abs_dy = extra_targets["target_abs_dy"].reshape(extra_targets["target_abs_dy"].shape[0], -1).mean(dim=-1)
                steady_flag = (abs_dy > float(group_cfg.get("steady_dy_threshold", 0.003))).long()
            else:
                steady_flag = torch.zeros_like(point_loss, dtype=torch.long)
            speed_bucket = torch.bucketize(target_flat.detach(), torch.as_tensor(speed_bins, device=device, dtype=target_flat.dtype))
            assist_bucket = torch.bucketize(assist_proxy.detach(), torch.as_tensor(assist_bins, device=device, dtype=assist_proxy.dtype))
            group_id = speed_bucket * 16 + assist_bucket * 2 + steady_flag
            group_losses = []
            for gid in torch.unique(group_id):
                mask = group_id == gid
                if torch.any(mask):
                    group_losses.append(point_loss[mask].mean())
            if group_losses:
                if bool(group_cfg.get("softmax_weighting", True)):
                    temp = float(group_cfg.get("temperature", 5.0))
                    gl = torch.stack(group_losses)
                    weights = torch.softmax(temp * gl.detach(), dim=0)
                    dro_loss = torch.sum(weights * gl)
                else:
                    dro_loss = torch.stack(group_losses).max()
                total = total + float(group_cfg.get("weight", 0.0)) * dro_loss

    return total


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


def align_pred_target_torch(pred: torch.Tensor, target: torch.Tensor):
    """Align common [B,1] vs [B,1,1] shape mismatches before loss computation."""
    while pred.dim() > target.dim() and pred.dim() > 0 and pred.shape[-2] == 1:
        pred = pred.squeeze(-2)
    while target.dim() > pred.dim() and target.dim() > 0 and target.shape[-2] == 1:
        target = target.squeeze(-2)

    if pred.shape != target.shape:
        if pred.dim() == target.dim() == 3 and pred.shape[1] == 1 and target.shape[1] == 1:
            pred = pred.squeeze(1)
            target = target.squeeze(1)
        elif pred.dim() == 2 and target.dim() == 3 and target.shape[1] == 1:
            target = target.squeeze(1)
        elif pred.dim() == 3 and pred.shape[1] == 1 and target.dim() == 2:
            pred = pred.squeeze(1)
    return pred, target


def align_pred_target_numpy(pred: np.ndarray, target: np.ndarray):
    """Numpy counterpart for metric/inverse-transform paths."""
    while pred.ndim > target.ndim and pred.ndim > 0 and pred.shape[-2] == 1:
        pred = np.squeeze(pred, axis=-2)
    while target.ndim > pred.ndim and target.ndim > 0 and target.shape[-2] == 1:
        target = np.squeeze(target, axis=-2)

    if pred.shape != target.shape:
        if pred.ndim == target.ndim == 3 and pred.shape[1] == 1 and target.shape[1] == 1:
            pred = np.squeeze(pred, axis=1)
            target = np.squeeze(target, axis=1)
        elif pred.ndim == 2 and target.ndim == 3 and target.shape[1] == 1:
            target = np.squeeze(target, axis=1)
        elif pred.ndim == 3 and pred.shape[1] == 1 and target.ndim == 2:
            pred = np.squeeze(pred, axis=1)
    return pred, target


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

            pred_base, yb_s = align_pred_target_torch(pred_base, yb)

            base_loss = torch.sum(torch.abs(pred_base - yb_s)).item()
            total_base_loss += base_loss

            for i in range(N_feats):
                original_col = xb[:, :, i].clone()

                idx = torch.randperm(B, device=device)
                xb[:, :, i] = xb[idx, :, i]

                pred_p = model(xb)
                pred_p, _ = align_pred_target_torch(pred_p, yb_s)

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
    extra_targets_cfg = data_cfg.get("extra_targets")

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

    _ar_cfg_pre = model_cfg.get("ar_feedback", {}) if model_cfg else {}
    _ar_k_ds = _ar_cfg_pre.get("k", 1) if _ar_cfg_pre.get("enable", False) else 0
    if _ar_k_ds > 0:
        print(f"[INFO] AR Feedback: dataset will return y_prev (ar_k={_ar_k_ds})")

    # Determine device early so datasets can be pre-loaded onto GPU (eliminates per-batch CPU→GPU copies)
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    allow_tf32 = bool(actual_train_params.get("allow_tf32", True))
    cudnn_benchmark = bool(actual_train_params.get("cudnn_benchmark", True))
    clear_cuda_cache_each_epoch = bool(actual_train_params.get("clear_cuda_cache_each_epoch", False))

    if device.type == "cuda":
        torch.backends.cuda.matmul.allow_tf32 = allow_tf32
        torch.backends.cudnn.allow_tf32 = allow_tf32
        torch.backends.cudnn.benchmark = cudnn_benchmark
        print(
            f"[INFO] Runtime speed flags: allow_tf32={allow_tf32}, "
            f"cudnn_benchmark={cudnn_benchmark}, clear_cuda_cache_each_epoch={clear_cuda_cache_each_epoch}"
        )

    print(f"[INFO] Pre-loading dataset tensors onto {device}")

    train_ds = LazyWindowDataset(X_train, Y_train, window_size, window_output, stride, target_mode, est_tick_ranges=est_tick_ranges, augment=True, noise_std=aug_std, noise_std_per_dim=noise_std_per_dim, ar_k=_ar_k_ds, device=device, time_warp_sigma=time_warp_sigma, target_transform=target_transform, input_mean=scaler_mean, input_std=scaler_std, extra_targets=extra_targets_cfg)
    val_ds   = LazyWindowDataset(X_val,   Y_val,   window_size, window_output, stride, target_mode, est_tick_ranges=est_tick_ranges, augment=False, ar_k=_ar_k_ds, device=device, target_transform=target_transform, input_mean=scaler_mean, input_std=scaler_std, extra_targets=extra_targets_cfg)
    test_ds  = LazyWindowDataset(X_test,  Y_test,  window_size, window_output, stride, target_mode, est_tick_ranges=est_tick_ranges, augment=False, ar_k=_ar_k_ds, device=device, target_transform=target_transform, input_mean=scaler_mean, input_std=scaler_std, extra_targets=extra_targets_cfg)

    if platform.system() == 'Linux':
        num_workers = 0
    else:
        num_workers = 0

    sampler_cfg = train_cfg.get("regime_balanced_sampling") or {}
    train_sampler = None
    if sampler_cfg.get("enable", False) and _ar_k_ds == 0:
        weights = _compute_regime_sample_weights(
            X_train, Y_train,
            input_window=window_size,
            stride=stride,
            est_tick_ranges=est_tick_ranges or [1],
            assist_channel_idx=sampler_cfg.get("assist_channel_idx"),
            steady_dy_threshold=float(sampler_cfg.get("steady_dy_threshold", 0.003)),
            speed_bins=sampler_cfg.get("speed_bins", [0.9, 1.15]),
            assist_quantiles=sampler_cfg.get("assist_quantiles", [0.33, 0.66]),
            hard_case_target_threshold=sampler_cfg.get("hard_case_target_threshold"),
            hard_case_assist_threshold=sampler_cfg.get("hard_case_assist_threshold"),
            hard_case_multiplier=sampler_cfg.get("hard_case_multiplier", 1.0),
        )
        if weights is not None and len(weights) == len(train_ds):
            train_sampler = WeightedRandomSampler(
                weights=torch.as_tensor(weights, dtype=torch.double),
                num_samples=len(weights),
                replacement=True,
            )
            print(f"[INFO] Regime-balanced sampler enabled (n={len(weights)})")

    train_loader = DataLoader(train_ds, batch_size=batch_size, shuffle=(train_sampler is None), sampler=train_sampler, num_workers=num_workers)
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
    full_output_dim = Y_train[0].shape[1]
    primary_output_dim = int(data_cfg.get("primary_output_dim", full_output_dim))
    if primary_output_dim <= 0 or primary_output_dim > full_output_dim:
        raise ValueError(f"Invalid primary_output_dim={primary_output_dim} for full_output_dim={full_output_dim}")
    output_dim = primary_output_dim

    model_type = model_cfg.get("type", "TCN")
    print(f"[INFO] Initializing Model: {model_type}")

    residual_skip = model_cfg.get("residual_skip", None)
    if residual_skip and residual_skip.get("enable", False):
        print(f"[INFO] Residual skip connection enabled: input_channel_idx={residual_skip['input_channel_idx']}")

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
        mlp_norm=mlp_norm,
        residual_skip=residual_skip
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

    elif model_type == "CausalSmoothTCN_GRU":
        gru_h = model_cfg.get("gru_hidden", 32)
        smooth_w = model_cfg.get("smooth_window", 11)
        print(f"[INFO] CausalSmoothTCN_GRU: gru_hidden={gru_h}, smooth_window={smooth_w}")
        model = CausalSmoothTCN_GRU(**common_args, gru_hidden=gru_h, smooth_window=smooth_w)

    elif model_type == "MultiScaleTCN_GRU":
        gru_h = model_cfg.get("gru_hidden", 32)
        kernel_sizes = tuple(model_cfg.get("kernel_sizes", [3, 5, 7]))
        print(f"[INFO] MultiScaleTCN_GRU: gru_hidden={gru_h}, kernel_sizes={kernel_sizes}")
        model = MultiScaleTCN_GRU(**common_args, gru_hidden=gru_h, kernel_sizes=kernel_sizes)

    elif model_type == "DualBranchTCN_GRU":
        gru_h = model_cfg.get("gru_hidden", 32)
        branch_input_dims = model_cfg.get("branch_input_dims")
        branch_channels = model_cfg.get("branch_channels")
        print(f"[INFO] DualBranchTCN_GRU: gru_hidden={gru_h}, branch_input_dims={branch_input_dims}")
        model = DualBranchTCN_GRU(
            **common_args,
            gru_hidden=gru_h,
            branch_input_dims=branch_input_dims,
            branch_channels=branch_channels,
        )

    elif model_type == "MultiBranchTCN_GRU":
        gru_h = model_cfg.get("gru_hidden", 32)
        branch_input_dims = model_cfg.get("branch_input_dims")
        branch_channels = model_cfg.get("branch_channels")
        print(f"[INFO] MultiBranchTCN_GRU: gru_hidden={gru_h}, branch_input_dims={branch_input_dims}")
        model = MultiBranchTCN_GRU(
            **common_args,
            gru_hidden=gru_h,
            branch_input_dims=branch_input_dims,
            branch_channels=branch_channels,
        )

    elif model_type == "FiLMConditionedTCN_GRU":
        gru_h = model_cfg.get("gru_hidden", 32)
        branch_input_dims = model_cfg.get("branch_input_dims")
        branch_channels = model_cfg.get("branch_channels")
        film_scale = model_cfg.get("film_scale", 0.5)
        print(f"[INFO] FiLMConditionedTCN_GRU: gru_hidden={gru_h}, branch_input_dims={branch_input_dims}, film_scale={film_scale}")
        model = FiLMConditionedTCN_GRU(
            **common_args,
            gru_hidden=gru_h,
            branch_input_dims=branch_input_dims,
            branch_channels=branch_channels,
            film_scale=film_scale,
        )

    elif model_type == "BoundedResidualTCN_GRU":
        gru_h = model_cfg.get("gru_hidden", 32)
        branch_input_dims = model_cfg.get("branch_input_dims")
        branch_channels = model_cfg.get("branch_channels")
        residual_scale = model_cfg.get("residual_scale", 0.30)
        print(f"[INFO] BoundedResidualTCN_GRU: gru_hidden={gru_h}, branch_input_dims={branch_input_dims}, residual_scale={residual_scale}")
        model = BoundedResidualTCN_GRU(
            **common_args,
            gru_hidden=gru_h,
            branch_input_dims=branch_input_dims,
            branch_channels=branch_channels,
            residual_scale=residual_scale,
        )

    elif model_type == "AffineCalibTCN_GRU":
        gru_h = model_cfg.get("gru_hidden", 32)
        branch_input_dims = model_cfg.get("branch_input_dims")
        branch_channels = model_cfg.get("branch_channels")
        alpha_scale = model_cfg.get("alpha_scale", 0.30)
        beta_scale = model_cfg.get("beta_scale", 0.25)
        print(f"[INFO] AffineCalibTCN_GRU: gru_hidden={gru_h}, branch_input_dims={branch_input_dims}, alpha_scale={alpha_scale}, beta_scale={beta_scale}")
        model = AffineCalibTCN_GRU(
            **common_args,
            gru_hidden=gru_h,
            branch_input_dims=branch_input_dims,
            branch_channels=branch_channels,
            alpha_scale=alpha_scale,
            beta_scale=beta_scale,
        )

    elif model_type == "PositiveResidualTCN_GRU":
        gru_h = model_cfg.get("gru_hidden", 32)
        branch_input_dims = model_cfg.get("branch_input_dims")
        branch_channels = model_cfg.get("branch_channels")
        residual_scale = model_cfg.get("residual_scale", 0.25)
        gate_scale = model_cfg.get("gate_scale", 1.0)
        print(f"[INFO] PositiveResidualTCN_GRU: gru_hidden={gru_h}, branch_input_dims={branch_input_dims}, residual_scale={residual_scale}, gate_scale={gate_scale}")
        model = PositiveResidualTCN_GRU(
            **common_args,
            gru_hidden=gru_h,
            branch_input_dims=branch_input_dims,
            branch_channels=branch_channels,
            residual_scale=residual_scale,
            gate_scale=gate_scale,
        )

    elif model_type == "AssistExpertTCN_GRU":
        gru_h = model_cfg.get("gru_hidden", 32)
        branch_input_dims = model_cfg.get("branch_input_dims")
        branch_channels = model_cfg.get("branch_channels")
        gate_max = model_cfg.get("gate_max", 0.6)
        print(f"[INFO] AssistExpertTCN_GRU: gru_hidden={gru_h}, branch_input_dims={branch_input_dims}, gate_max={gate_max}")
        model = AssistExpertTCN_GRU(
            **common_args,
            gru_hidden=gru_h,
            branch_input_dims=branch_input_dims,
            branch_channels=branch_channels,
            gate_max=gate_max,
        )

    elif model_type == "SlowFastCalibTCN_GRU":
        gru_h = model_cfg.get("gru_hidden", 32)
        branch_input_dims = model_cfg.get("branch_input_dims")
        branch_channels = model_cfg.get("branch_channels")
        bias_scale = model_cfg.get("bias_scale", 0.20)
        residual_scale = model_cfg.get("residual_scale", 0.12)
        print(
            f"[INFO] SlowFastCalibTCN_GRU: gru_hidden={gru_h}, "
            f"branch_input_dims={branch_input_dims}, bias_scale={bias_scale}, residual_scale={residual_scale}"
        )
        model = SlowFastCalibTCN_GRU(
            **common_args,
            gru_hidden=gru_h,
            branch_input_dims=branch_input_dims,
            branch_channels=branch_channels,
            bias_scale=bias_scale,
            residual_scale=residual_scale,
        )
    elif model_type == "StructuredSlowFastTCN_GRU":
        gru_h = model_cfg.get("gru_hidden", 32)
        branch_input_dims = model_cfg.get("branch_input_dims")
        branch_channels = model_cfg.get("branch_channels")
        model = StructuredSlowFastTCN_GRU(
            **common_args,
            gru_hidden=gru_h,
            branch_input_dims=branch_input_dims,
            branch_channels=branch_channels,
            bias_scale=model_cfg.get("bias_scale", 0.18),
            residual_scale=model_cfg.get("residual_scale", 0.10),
            use_aux_head=model_cfg.get("use_aux_head", False),
            aux_dim=model_cfg.get("aux_dim", 1),
            use_subject_calibration=model_cfg.get("use_subject_calibration", False),
            subject_ctx_dims=model_cfg.get("subject_ctx_dims", 0),
            alpha_scale=model_cfg.get("alpha_scale", 0.08),
            beta_scale=model_cfg.get("beta_scale", 0.05),
        )
    elif model_type == "AdaptiveComplementaryTCN_GRU":
        model = AdaptiveComplementaryTCN_GRU(
            **common_args,
            gru_hidden=model_cfg.get("gru_hidden", 32),
            branch_input_dims=model_cfg.get("branch_input_dims"),
            branch_channels=model_cfg.get("branch_channels"),
            bias_scale=model_cfg.get("bias_scale", 0.18),
            residual_scale=model_cfg.get("residual_scale", 0.08),
            gate_temperature=model_cfg.get("gate_temperature", 1.0),
        )
    elif model_type == "CadenceStepLengthTCN_GRU":
        model = CadenceStepLengthTCN_GRU(
            **common_args,
            gru_hidden=model_cfg.get("gru_hidden", 32),
            branch_input_dims=model_cfg.get("branch_input_dims"),
            branch_channels=model_cfg.get("branch_channels"),
            residual_scale=model_cfg.get("residual_scale", 0.05),
        )
    elif model_type == "PersistentBiasTCN_GRU":
        model = PersistentBiasTCN_GRU(
            **common_args,
            gru_hidden=model_cfg.get("gru_hidden", 32),
            branch_input_dims=model_cfg.get("branch_input_dims"),
            branch_channels=model_cfg.get("branch_channels"),
            bias_scale=model_cfg.get("bias_scale", 0.18),
            residual_scale=model_cfg.get("residual_scale", 0.08),
            gate_temperature=model_cfg.get("gate_temperature", 1.0),
            state_leak=model_cfg.get("state_leak", 0.92),
        )
    elif model_type == "SpeedModeResidualTCN_GRU":
        model = SpeedModeResidualTCN_GRU(
            **common_args,
            gru_hidden=model_cfg.get("gru_hidden", 32),
            branch_input_dims=model_cfg.get("branch_input_dims"),
            branch_channels=model_cfg.get("branch_channels"),
            bias_scale=model_cfg.get("bias_scale", 0.18),
            mode_centers=model_cfg.get("mode_centers", [0.75, 1.0, 1.25]),
            mode_residual_scale=model_cfg.get("mode_residual_scale", 0.12),
        )
    elif model_type == "ObserverStateTCN_GRU":
        model = ObserverStateTCN_GRU(
            **common_args,
            gru_hidden=model_cfg.get("gru_hidden", 32),
            branch_input_dims=model_cfg.get("branch_input_dims"),
            branch_channels=model_cfg.get("branch_channels"),
            bias_scale=model_cfg.get("bias_scale", 0.12),
            residual_scale=model_cfg.get("residual_scale", 0.05),
            drift_scale=model_cfg.get("drift_scale", 0.06),
            update_stride=model_cfg.get("update_stride", 8),
        )
    elif model_type == "FactorizedAnchorTCN_GRU":
        model = FactorizedAnchorTCN_GRU(
            **common_args,
            gru_hidden=model_cfg.get("gru_hidden", 32),
            branch_input_dims=model_cfg.get("branch_input_dims"),
            branch_channels=model_cfg.get("branch_channels"),
            residual_scale=model_cfg.get("residual_scale", 0.06),
        )
    elif model_type == "MemoryCalibratorTCN_GRU":
        model = MemoryCalibratorTCN_GRU(
            **common_args,
            gru_hidden=model_cfg.get("gru_hidden", 32),
            branch_input_dims=model_cfg.get("branch_input_dims"),
            branch_channels=model_cfg.get("branch_channels"),
            bias_scale=model_cfg.get("bias_scale", 0.14),
            residual_scale=model_cfg.get("residual_scale", 0.04),
            calibrator_stride=model_cfg.get("calibrator_stride", 10),
        )
    elif model_type == "PrivilegedKinMemoryCalibratorTCN_GRU":
        model = PrivilegedKinMemoryCalibratorTCN_GRU(
            **common_args,
            gru_hidden=model_cfg.get("gru_hidden", 32),
            branch_input_dims=model_cfg.get("branch_input_dims"),
            branch_channels=model_cfg.get("branch_channels"),
            bias_scale=model_cfg.get("bias_scale", 0.14),
            residual_scale=model_cfg.get("residual_scale", 0.04),
            calibrator_stride=model_cfg.get("calibrator_stride", 10),
            aux_dim=model_cfg.get("aux_dim", 6),
            kin_scale=model_cfg.get("kin_scale", 0.04),
        )
    elif model_type == "AssistSwitchHeadTCN_GRU":
        model = AssistSwitchHeadTCN_GRU(
            **common_args,
            gru_hidden=model_cfg.get("gru_hidden", 32),
            branch_input_dims=model_cfg.get("branch_input_dims"),
            branch_channels=model_cfg.get("branch_channels"),
            bias_scale=model_cfg.get("bias_scale", 0.16),
            residual_scale=model_cfg.get("residual_scale", 0.08),
            gate_temperature=model_cfg.get("gate_temperature", 1.0),
            on_scale=model_cfg.get("on_scale", 0.12),
            off_scale=model_cfg.get("off_scale", 0.05),
            hard_switch=model_cfg.get("hard_switch", False),
            assist_state_index=model_cfg.get("assist_state_index", -3),
        )
    elif model_type == "StrideProgressionTCN_GRU":
        model = StrideProgressionTCN_GRU(
            **common_args,
            gru_hidden=model_cfg.get("gru_hidden", 32),
            branch_input_dims=model_cfg.get("branch_input_dims"),
            branch_channels=model_cfg.get("branch_channels"),
            phase_bins=model_cfg.get("phase_bins", 4),
            bias_scale=model_cfg.get("bias_scale", 0.10),
            residual_scale=model_cfg.get("residual_scale", 0.04),
        )
    elif model_type == "PDRHybridTCN_GRU":
        model = PDRHybridTCN_GRU(
            **common_args,
            gru_hidden=model_cfg.get("gru_hidden", 32),
            branch_input_dims=model_cfg.get("branch_input_dims"),
            branch_channels=model_cfg.get("branch_channels"),
            pdr_scale=model_cfg.get("pdr_scale", 0.10),
            bias_scale=model_cfg.get("bias_scale", 0.10),
            residual_scale=model_cfg.get("residual_scale", 0.04),
        )
    elif model_type == "PredictedKinCascadeTCN_GRU":
        model = PredictedKinCascadeTCN_GRU(
            **common_args,
            gru_hidden=model_cfg.get("gru_hidden", 32),
            branch_input_dims=model_cfg.get("branch_input_dims"),
            branch_channels=model_cfg.get("branch_channels"),
            bias_scale=model_cfg.get("bias_scale", 0.14),
            residual_scale=model_cfg.get("residual_scale", 0.04),
            calibrator_stride=model_cfg.get("calibrator_stride", 10),
            aux_dim=model_cfg.get("aux_dim", 6),
            kin_scale=model_cfg.get("kin_scale", 0.04),
            kin_mix_scale=model_cfg.get("kin_mix_scale", 0.20),
        )
    elif model_type == "ProgressionDistillTCN_GRU":
        model = ProgressionDistillTCN_GRU(
            **common_args,
            gru_hidden=model_cfg.get("gru_hidden", 32),
            branch_input_dims=model_cfg.get("branch_input_dims"),
            branch_channels=model_cfg.get("branch_channels"),
            bias_scale=model_cfg.get("bias_scale", 0.14),
            residual_scale=model_cfg.get("residual_scale", 0.04),
            calibrator_stride=model_cfg.get("calibrator_stride", 10),
            aux_dim=model_cfg.get("aux_dim", 6),
            kin_scale=model_cfg.get("kin_scale", 0.04),
            kin_mix_scale=model_cfg.get("kin_mix_scale", 0.20),
            progression_scale=model_cfg.get("progression_scale", 0.16),
        )
    elif model_type == "AssistDeconfoundTCN_GRU":
        model = AssistDeconfoundTCN_GRU(
            **common_args,
            gru_hidden=model_cfg.get("gru_hidden", 32),
            branch_input_dims=model_cfg.get("branch_input_dims"),
            branch_channels=model_cfg.get("branch_channels"),
            deconfound_scale=model_cfg.get("deconfound_scale", 0.25),
            bias_scale=model_cfg.get("bias_scale", 0.10),
            residual_scale=model_cfg.get("residual_scale", 0.04),
        )
    elif model_type == "MultiAnchorFusionTCN_GRU":
        model = MultiAnchorFusionTCN_GRU(
            **common_args,
            gru_hidden=model_cfg.get("gru_hidden", 32),
            branch_input_dims=model_cfg.get("branch_input_dims"),
            branch_channels=model_cfg.get("branch_channels"),
            bias_scale=model_cfg.get("bias_scale", 0.10),
            residual_scale=model_cfg.get("residual_scale", 0.04),
        )
    elif model_type == "AdaptiveBiasStateTCN_GRU":
        model = AdaptiveBiasStateTCN_GRU(
            **common_args,
            gru_hidden=model_cfg.get("gru_hidden", 32),
            branch_input_dims=model_cfg.get("branch_input_dims"),
            branch_channels=model_cfg.get("branch_channels"),
            bias_scale=model_cfg.get("bias_scale", 0.14),
            residual_scale=model_cfg.get("residual_scale", 0.04),
            update_stride=model_cfg.get("update_stride", 8),
        )
    elif model_type == "MonotoneProgressionTCN_GRU":
        model = MonotoneProgressionTCN_GRU(
            **common_args,
            gru_hidden=model_cfg.get("gru_hidden", 32),
            branch_input_dims=model_cfg.get("branch_input_dims"),
            branch_channels=model_cfg.get("branch_channels"),
            bias_scale=model_cfg.get("bias_scale", 0.08),
            residual_scale=model_cfg.get("residual_scale", 0.03),
        )
    elif model_type == "AssistOnBiasCorrectorTCN_GRU":
        model = AssistOnBiasCorrectorTCN_GRU(
            **common_args,
            gru_hidden=model_cfg.get("gru_hidden", 32),
            branch_input_dims=model_cfg.get("branch_input_dims"),
            branch_channels=model_cfg.get("branch_channels"),
            bias_scale=model_cfg.get("bias_scale", 0.16),
            residual_scale=model_cfg.get("residual_scale", 0.06),
            gate_temperature=model_cfg.get("gate_temperature", 1.0),
            on_bias_scale=model_cfg.get("on_bias_scale", 0.18),
            assist_state_index=model_cfg.get("assist_state_index", -3),
        )
    elif model_type == "SpeedScaleSwitchTCN_GRU":
        model = SpeedScaleSwitchTCN_GRU(
            **common_args,
            gru_hidden=model_cfg.get("gru_hidden", 32),
            branch_input_dims=model_cfg.get("branch_input_dims"),
            branch_channels=model_cfg.get("branch_channels"),
            bias_scale=model_cfg.get("bias_scale", 0.16),
            residual_scale=model_cfg.get("residual_scale", 0.06),
            gate_temperature=model_cfg.get("gate_temperature", 1.0),
            low_scale=model_cfg.get("low_scale", 0.05),
            high_scale=model_cfg.get("high_scale", 0.14),
            speed_temperature=model_cfg.get("speed_temperature", 1.0),
            hard_switch=model_cfg.get("hard_switch", False),
        )
    elif model_type == "RegimeExpertTCN_GRU":
        model = RegimeExpertTCN_GRU(
            **common_args,
            gru_hidden=model_cfg.get("gru_hidden", 32),
            branch_input_dims=model_cfg.get("branch_input_dims"),
            branch_channels=model_cfg.get("branch_channels"),
            bias_scale=model_cfg.get("bias_scale", 0.16),
            residual_scale=model_cfg.get("residual_scale", 0.06),
            gate_temperature=model_cfg.get("gate_temperature", 1.0),
            expert_scale=model_cfg.get("expert_scale", 0.14),
            assist_state_index=model_cfg.get("assist_state_index", -3),
            assist_high_index=model_cfg.get("assist_high_index", -2),
        )
    elif model_type == "AssistSplitTCN_GRU":
        model = AssistSplitTCN_GRU(
            **common_args,
            gru_hidden=model_cfg.get("gru_hidden", 32),
            branch_input_dims=model_cfg.get("branch_input_dims"),
            branch_channels=model_cfg.get("branch_channels"),
            bias_scale=model_cfg.get("bias_scale", 0.16),
            residual_scale=model_cfg.get("residual_scale", 0.06),
        )

    elif model_type == "TCN_GRU_MDN":
        gru_h = model_cfg.get("gru_hidden", 32)
        num_mixtures = int(model_cfg.get("num_mixtures", 5))
        print(f"[INFO] TCN_GRU_MDN: hidden={gru_h}, num_mixtures={num_mixtures}")
        model = TCN_GRU_MDN(**common_args, gru_hidden=gru_h, num_mixtures=num_mixtures)

    else:
        model = TCN_MLP(**common_args)

    pretrained_ckpt = train_cfg.get("pretrained_checkpoint") or model_cfg.get("pretrained_checkpoint")
    if pretrained_ckpt:
        print(f"[INFO] Loading pretrained checkpoint: {pretrained_ckpt}")
        ckpt = torch.load(pretrained_ckpt, map_location=device)
        state_dict = ckpt["state_dict"] if isinstance(ckpt, dict) and "state_dict" in ckpt else ckpt
        model_state = model.state_dict()
        filtered_state = {}
        skipped_shape = []
        for k, v in state_dict.items():
            if k not in model_state:
                continue
            if model_state[k].shape != v.shape:
                skipped_shape.append((k, tuple(v.shape), tuple(model_state[k].shape)))
                continue
            filtered_state[k] = v
        missing, unexpected = model.load_state_dict(filtered_state, strict=False)
        if skipped_shape:
            print(f"[INFO] Pretrained skipped shape-mismatched keys: {len(skipped_shape)}")
            for name, src_shape, dst_shape in skipped_shape[:10]:
                print(f"  - {name}: ckpt{src_shape} -> model{dst_shape}")
            if len(skipped_shape) > 10:
                print(f"  ... and {len(skipped_shape) - 10} more")
        if missing:
            print(f"[INFO] Pretrained missing keys: {len(missing)}")
        if unexpected:
            print(f"[INFO] Pretrained unexpected keys: {len(unexpected)}")

    freeze_all_except = train_cfg.get("freeze_all_except") or model_cfg.get("freeze_all_except") or []
    freeze_patterns = train_cfg.get("freeze_patterns") or model_cfg.get("freeze_patterns") or []
    if freeze_all_except:
        keep_patterns = [str(p) for p in freeze_all_except]
        for name, param in model.named_parameters():
            param.requires_grad = any(pat in name for pat in keep_patterns)
        print(f"[INFO] Freeze-all-except active: {keep_patterns}")
    elif freeze_patterns:
        drop_patterns = [str(p) for p in freeze_patterns]
        for name, param in model.named_parameters():
            if any(pat in name for pat in drop_patterns):
                param.requires_grad = False
        print(f"[INFO] Freeze patterns active: {drop_patterns}")

    model.to(device)
    is_mdn_model = model_type == "TCN_GRU_MDN"
    mdn_num_mixtures = int(model_cfg.get("num_mixtures", 5)) if is_mdn_model else 0

    trainable_params = [p for p in model.parameters() if p.requires_grad]
    if not trainable_params:
        raise RuntimeError("No trainable parameters remain after applying freeze settings.")
    optimizer = torch.optim.AdamW(trainable_params, lr=lr, weight_decay=weight_decay)
    scheduler = None
    if cos_T0 > 0:
        scheduler = torch.optim.lr_scheduler.CosineAnnealingWarmRestarts(
            optimizer, T_0=cos_T0, T_mult=cos_Tmult, eta_min=eta_min
        )

    cost_fn_name = config.get("cost_function", config.get("loss", "huber")).lower()

    use_weighted_loss = config.get("use_weighted_loss", False)
    loss_decay_factor = config.get("loss_decay_factor", 0.8)
    dyn_cfg = train_cfg.get("dynamic_loss") or {}
    dyn_enable = bool(dyn_cfg.get("enable", False))

    reduction_val = 'none' if (use_weighted_loss or dyn_enable) else 'mean'

    if is_mdn_model:
        criterion = MDNLoss(output_dim=output_dim, num_mixtures=mdn_num_mixtures, reduction='mean')
    elif cost_fn_name == "huber":
        criterion = nn.HuberLoss(delta=huber_delta, reduction=reduction_val)
    elif cost_fn_name == "mse":
        criterion = nn.MSELoss(reduction=reduction_val)
    elif cost_fn_name in ["mae", "l1"]:
        criterion = nn.L1Loss(reduction=reduction_val)
    else:
        print(f"[WARN] Unknown cost function '{cost_fn_name}', defaulting to Huber.")
        criterion = nn.HuberLoss(delta=huber_delta, reduction=reduction_val)

    if is_mdn_model and use_weighted_loss:
        print("[WARN] Weighted loss is disabled for TCN_GRU_MDN.")
        use_weighted_loss = False

    loss_weights = None
    if use_weighted_loss:
        w_t = np.array([loss_decay_factor**t for t in range(horizon)])
        loss_weights = torch.from_numpy(w_t).float().to(device)
        loss_weights = loss_weights.view(1, horizon, 1)

    custom_losses = get_custom_losses(config, device, feature_names, scaler_mean, scaler_std)
    if custom_losses:
        print(f"[Loss] Active Custom Losses: {list(custom_losses.keys())}")

    dyn_alpha = float(dyn_cfg.get("alpha", 2.0))
    dyn_ref = float(dyn_cfg.get("reference_delta", 0.05))
    dyn_max = float(dyn_cfg.get("max_scale", 4.0))
    dyn_idx = dyn_cfg.get("input_channel_idx")
    if dyn_enable and dyn_idx is not None:
        dyn_idx = int(dyn_idx)
        dyn_tt_mean = None if scaler_mean is None else torch.as_tensor(scaler_mean, dtype=torch.float32, device=device)
        dyn_tt_std = None if scaler_std is None else torch.as_tensor(scaler_std, dtype=torch.float32, device=device)
        dyn_prior_cfg = {"mode": "delta_from_input", "input_channel_idx": dyn_idx}
        print(f"[Loss] Dynamic weighting: input_channel_idx={dyn_idx}, alpha={dyn_alpha}, ref={dyn_ref}, max={dyn_max}")
    else:
        dyn_enable = False

    def apply_dynamic_weighting(raw_loss, xb, target):
        if not dyn_enable:
            return raw_loss.mean()
        target_abs = inverse_target_transform(
            target, xb,
            target_transform=target_transform,
            input_mean=dyn_tt_mean,
            input_std=dyn_tt_std,
        )
        base = extract_target_base(
            xb, dyn_prior_cfg,
            input_mean=dyn_tt_mean,
            input_std=dyn_tt_std,
        )
        if base is None:
            return raw_loss.mean()
        delta = torch.abs(target_abs - base.unsqueeze(-1))
        while delta.dim() < raw_loss.dim():
            delta = delta.unsqueeze(-1)
        if delta.dim() > raw_loss.dim():
            delta = delta.squeeze(-1)
        weights = 1.0 + dyn_alpha * torch.clamp(delta / max(dyn_ref, 1e-6), max=dyn_max)
        return (raw_loss * weights).mean()

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
            if ar_enable:
                xb, yb, yb_prev = batch
                xb, yb = xb.to(device), yb.to(device)
                yb = _split_primary_target(yb, primary_output_dim)
                yb_prev = yb_prev.to(device)
                extra_targets = None
            else:
                if isinstance(batch, (list, tuple)) and len(batch) == 3 and isinstance(batch[2], dict):
                    xb, yb, extra_targets = batch
                    extra_targets = _move_extra_targets_to_device(extra_targets, device)
                else:
                    xb, yb = batch
                    extra_targets = None
                xb, yb = xb.to(device), yb.to(device)
                yb = _split_primary_target(yb, primary_output_dim)

            if ar_enable:
                B, T_in, D = xb.shape
                if tf_ratio >= 1.0 or random.random() < tf_ratio:
                    prev_channel = yb_prev.unsqueeze(1).expand(B, T_in, ar_k)
                else:
                    with torch.no_grad():
                        zero_ar = torch.zeros(B, T_in, ar_k, device=device)
                        xb_no_ar = torch.cat([xb, zero_ar], dim=-1)
                        student_pred, _ = _extract_model_outputs(model(xb_no_ar))
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
                    if is_mdn_model:
                        pred_raw = model(xb, return_params=True)
                        pred = mdn_expected_value(pred_raw, output_dim, mdn_num_mixtures)
                        aux_outputs = {}
                    else:
                        pred_raw = None
                        model_out = model(xb)
                        pred, aux_outputs = _extract_model_outputs(model_out)
                        aux_outputs["pred_for_loss"] = pred

                    pred, yb = align_pred_target_torch(pred, yb)

                    if is_mdn_model:
                        loss = criterion(pred_raw, yb)
                    elif use_weighted_loss:
                        raw_loss = criterion(pred, yb)
                        weighted_loss = raw_loss * loss_weights
                        loss = weighted_loss.mean()
                    elif dyn_enable:
                        raw_loss = criterion(pred, yb)
                        loss = apply_dynamic_weighting(raw_loss, xb, yb)
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
                            elif name == 'underprediction':
                                loss += weight * fn(xb, pred, yb)
                            elif name == 'batch_bias':
                                loss += weight * fn(pred, yb)
                            elif name == 'assist_bias':
                                loss += weight * fn(xb, pred, yb)
                            elif name == 'freq_penalty':
                                loss += weight * fn(pred, yb)
                    loss += _compute_auxiliary_losses(
                        aux_outputs if not is_mdn_model else {},
                        xb, yb, extra_targets,
                        train_cfg, scaler_mean, scaler_std, device
                    )

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
                extra_targets = None
                if ar_enable:
                    xb, yb, yb_prev = batch
                    xb, yb = xb.to(device), yb.to(device)
                    yb = _split_primary_target(yb, primary_output_dim)
                    yb_prev = yb_prev.to(device)
                    B, T_in, _ = xb.shape
                    prev_channel = yb_prev.unsqueeze(1).expand(B, T_in, ar_k)
                    xb = torch.cat([xb, prev_channel], dim=-1)
                else:
                    extra_targets = None
                    if isinstance(batch, (list, tuple)) and len(batch) == 3 and isinstance(batch[2], dict):
                        xb, yb, extra_targets = batch
                        extra_targets = _move_extra_targets_to_device(extra_targets, device)
                    else:
                        xb, yb = batch
                    xb, yb = xb.to(device), yb.to(device)
                    yb = _split_primary_target(yb, primary_output_dim)

                if is_mdn_model:
                    pred_raw = model(xb, return_params=True)
                    pred = mdn_expected_value(pred_raw, output_dim, mdn_num_mixtures)
                    aux_outputs = {}
                else:
                    pred_raw = None
                    model_out = model(xb)
                    pred, aux_outputs = _extract_model_outputs(model_out)
                    aux_outputs["pred_for_loss"] = pred

                if not is_mdn_model:
                    pred, yb = align_pred_target_torch(pred, yb)

                if is_mdn_model:
                    loss = criterion(pred_raw, yb)
                elif use_weighted_loss:
                    raw_loss = criterion(pred, yb)
                    weighted_loss = raw_loss * loss_weights
                    loss = weighted_loss.mean()
                elif dyn_enable:
                    raw_loss = criterion(pred, yb)
                    loss = apply_dynamic_weighting(raw_loss, xb, yb)
                else:
                    loss = criterion(pred, yb)

                loss += _compute_auxiliary_losses(
                    aux_outputs if not is_mdn_model else {},
                    xb, yb, extra_targets,
                    train_cfg, scaler_mean, scaler_std, device
                )

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
        if clear_cuda_cache_each_epoch and torch.cuda.is_available():
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
            extra_targets = None
            if ar_enable:
                xb, yb, _ = batch
                xb, yb = xb.to(device), yb.to(device)
                yb = _split_primary_target(yb, primary_output_dim)
                yb_prev_t = batch[2].to(device)
                B, T_in, _ = xb.shape
                prev_channel = yb_prev_t.unsqueeze(1).expand(B, T_in, ar_k)
                xb = torch.cat([xb, prev_channel], dim=-1)
            else:
                extra_targets = None
                if isinstance(batch, (list, tuple)) and len(batch) == 3 and isinstance(batch[2], dict):
                    xb, yb, extra_targets = batch
                    extra_targets = _move_extra_targets_to_device(extra_targets, device)
                else:
                    xb, yb = batch
                xb, yb = xb.to(device), yb.to(device)
                yb = _split_primary_target(yb, primary_output_dim)

            if is_mdn_model:
                pred_raw = model(xb, return_params=True)
                pred = mdn_expected_value(pred_raw, output_dim, mdn_num_mixtures)
                aux_outputs = {}
            else:
                pred_raw = None
                model_out = model(xb)
                pred, aux_outputs = _extract_model_outputs(model_out)
                aux_outputs["pred_for_loss"] = pred

            if not is_mdn_model:
                pred, yb = align_pred_target_torch(pred, yb)

            if is_mdn_model:
                l = criterion(pred_raw, yb)
            elif use_weighted_loss:
                raw_loss = criterion(pred, yb)
                weighted_loss = raw_loss * loss_weights
                l = weighted_loss.mean()
            elif dyn_enable:
                raw_loss = criterion(pred, yb)
                l = apply_dynamic_weighting(raw_loss, xb, yb)
            else:
                l = criterion(pred, yb)

            l += _compute_auxiliary_losses(
                aux_outputs if not is_mdn_model else {},
                xb, yb, extra_targets,
                train_cfg, scaler_mean, scaler_std, device
            )

            test_running += l.item()
    test_loss = test_running / max(1, len(test_loader))

    preds_all, trues_all, stds_all = [], [], []
    with torch.no_grad():
        for batch in test_loader:
            if ar_enable:
                xb, yb, yb_prev_m = batch
                xb = xb.to(device)
                yb = _split_primary_target(yb, primary_output_dim)
                yb_prev_m = yb_prev_m.to(device)
                B, T_in, _ = xb.shape
                prev_channel = yb_prev_m.unsqueeze(1).expand(B, T_in, ar_k)
                xb = torch.cat([xb, prev_channel], dim=-1)
            else:
                if isinstance(batch, (list, tuple)) and len(batch) == 3 and isinstance(batch[2], dict):
                    xb, yb, _ = batch
                else:
                    xb, yb = batch
                xb = xb.to(device)
                yb = _split_primary_target(yb, primary_output_dim)

            if is_mdn_model:
                pred_t, pred_std_t = model(xb, return_std=True)
                pred = pred_t.cpu().numpy()
                stds_all.append(pred_std_t.cpu().numpy())
            else:
                model_out = model(xb)
                pred, _ = _extract_model_outputs(model_out)
                pred = pred.cpu().numpy()

            target = yb.cpu().numpy()
            pred, target = align_pred_target_numpy(pred, target)

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
        "test_loss": test_loss,
        "test_huber": test_loss,
        "test_mae": float(test_mae),
        "test_rmse": float(test_rmse),
        "n_params": res_metrics["n_params"],
        "model_size_mb": float(f"{res_metrics['model_size_mb']:.4f}"),
        "host_latency_ms": float(f"{res_metrics['host_latency_ms']:.4f}"),
        "host_max_freq_hz": float(f"{res_metrics['host_max_freq_hz']:.2f}"),
        "config": config
    }

    if is_mdn_model and stds_all:
        S = np.concatenate(stds_all, axis=0)
        metrics_result["test_nll"] = test_loss
        metrics_result["test_mean_pred_std"] = float(np.mean(S))
        metrics_result["test_median_pred_std"] = float(np.median(S))

    if multistep_detail:
        metrics_result["multistep_detail"] = multistep_detail

    logger.save_metrics(metrics_result)

    return metrics_result, ckpt_path
