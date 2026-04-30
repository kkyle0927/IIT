
import os
import sys
import glob
import json
import resource

# --- Resource Constraints (train.sh 기준과 동일: 1 GPU / 15 GB RAM / 16 CPU) ---
# 1. GPU: allow override for parallel shard workers submitted to the same GPU
_VISIBLE_GPU = os.environ.get(
    "COMPARE_RESULTS_CUDA_VISIBLE_DEVICES",
    os.environ.get("CUDA_VISIBLE_DEVICES", "0"),
)
os.environ["CUDA_VISIBLE_DEVICES"]     = _VISIBLE_GPU
# 2. Thread limits (OpenMP / MKL / NumExpr)
_CPU_THREADS = max(1, int(os.environ.get("COMPARE_RESULTS_THREADS", os.environ.get("SLURM_CPUS_PER_TASK", "16"))))
os.environ["OMP_NUM_THREADS"]          = str(_CPU_THREADS)
os.environ["MKL_NUM_THREADS"]          = str(_CPU_THREADS)
os.environ["NUMEXPR_NUM_THREADS"]      = str(_CPU_THREADS)
os.environ["KMP_DUPLICATE_LIB_OK"]     = "TRUE"
_EVAL_BATCH_SIZE = max(1, int(os.environ.get("COMPARE_RESULTS_EVAL_BATCH_SIZE", "1024")))
_DETAIL_BATCH_SIZE = max(1, int(os.environ.get("COMPARE_RESULTS_DETAIL_BATCH_SIZE", "512")))
_FI_BATCH_SIZE = max(1, int(os.environ.get("COMPARE_RESULTS_FI_BATCH_SIZE", "1024")))
# 3. RAM hard limit: 15 GB (same as train.sh get_limits())
_RAM_LIMIT_GB = float(os.environ.get("COMPARE_RESULTS_RAM_GB", "15"))
_RAM_LIMIT = int(_RAM_LIMIT_GB * 1024 * 1024 * 1024)
resource.setrlimit(resource.RLIMIT_AS, (_RAM_LIMIT, _RAM_LIMIT))
# ---------------------------------------------------------------------------------

import torch
import numpy as np
import pandas as pd
import matplotlib

# Check for display, otherwise use Agg backend (headless)
if os.environ.get('DISPLAY', '') == '':
    print('No display found. Using non-interactive Agg backend.')
    matplotlib.use('Agg')

# PyTorch thread limit
torch.set_num_threads(_CPU_THREADS)
print(f"[INFO] Resources optimized: GPU {_VISIBLE_GPU}, {_RAM_LIMIT_GB:g} GB RAM, {_CPU_THREADS} CPU threads max.")
print(f"[INFO] Eval batch sizes: eval={_EVAL_BATCH_SIZE}, detail={_DETAIL_BATCH_SIZE}, fi={_FI_BATCH_SIZE}")

import argparse
import matplotlib.pyplot as plt
import seaborn as sns
from pathlib import Path
from torch.utils.data import DataLoader
from scipy import signal

# -----------------------------------------------------------------------------
# Argument Parsing
# -----------------------------------------------------------------------------
def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--auto', action='store_true', help='Run in automated mode (no interaction)')
    parser.add_argument('--filter', type=str, default=None, help='Filter pattern for directories (e.g. "baseline_*")')
    parser.add_argument('--ablation', action='store_true', help='Generate ablation study comparison report')
    parser.add_argument('--fast-eval', action='store_true', help='Evaluate all experiments first and save metrics summary before rendering only top-k full plots')
    parser.add_argument('--full-plot-topk', type=int, default=0, help='After fast eval, render full plots only for top-k experiments by MAE')
    parser.add_argument('--fast-eval-stride', type=int, default=3, help='Evaluation stride to use during fast eval metric pass')
    parser.add_argument('--render-topk-only', action='store_true', help='Render full plots only for top-k experiments using an existing fast-eval summary')
    parser.add_argument('--num-shards', type=int, default=1, help='Split automated evaluation into N shards')
    parser.add_argument('--shard-index', type=int, default=0, help='Shard index to run when --num-shards > 1')
    parser.add_argument('--skip-ablation', action='store_true', help='Skip final ablation report after --auto')
    parser.add_argument('--skip-feature-importance', action='store_true', help='Skip expensive permutation importance plots')
    parser.add_argument('target_dir', nargs='?', help='Target experiment directory to analyze (optional)')
    args = parser.parse_args()
    if args.num_shards < 1:
        parser.error("--num-shards must be >= 1")
    if args.shard_index < 0 or args.shard_index >= args.num_shards:
        parser.error("--shard-index must satisfy 0 <= shard-index < num-shards")
    return args

# -----------------------------------------------------------------------------
# Settings
# -----------------------------------------------------------------------------

# Import Model and Dataset from main script
# Ensure the directory is in path
sys.path.append(str(Path(__file__).parent))

# -----------------------------------------------------------------------------
# Helper Functions (Replacing import dependencies)
# -----------------------------------------------------------------------------
def parse_subject_prefix(s):
    if s.startswith("m1_"): return "m1_", s[3:]
    if s.startswith("m2_"): return "m2_", s[3:]
    if s.startswith("m3_"): return "m3_", s[3:]
    return None, s

def get_data_sources_from_config(cfg):
    sources = {}
    raw_sources = cfg.get("shared", {}).get("data_sources", {}) or cfg.get("data_sources", {})
    if not isinstance(raw_sources, dict): return {}
    for key, val in raw_sources.items():
        if isinstance(val, dict) and 'prefix' in val:
            sources[val['prefix']] = val
    return sources
from SpeedEstimator_TCN_MLP_experiments import (
    TCN_MLP, StanceGatedTCN, AttentionTCN, TCN_GRU_Head, LazyWindowDataset, build_nn_dataset, build_nn_dataset_multi,
    make_subject_selection, CONDITION_SELECTION, extract_condition_data_v2
)
from core.models import CausalSmoothTCN_GRU, MultiScaleTCN_GRU, DualBranchTCN_GRU, MultiBranchTCN_GRU, FiLMConditionedTCN_GRU, BoundedResidualTCN_GRU, AffineCalibTCN_GRU, PositiveResidualTCN_GRU, AssistExpertTCN_GRU, SlowFastCalibTCN_GRU, StructuredSlowFastTCN_GRU, TCN_GRU_MDN
from core.datasets import inverse_target_transform
import SpeedEstimator_TCN_MLP_experiments as main_script

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
_CONFIG_DIR_CACHE = {}


def extract_primary_prediction(model_out):
    if isinstance(model_out, dict):
        return model_out.get("pred")
    if isinstance(model_out, (list, tuple)):
        return model_out[0]
    return model_out

def get_experiment_group_name(exp_path):
    fold_name = exp_path.name
    return exp_path.parent.name if fold_name.startswith("fold_") else fold_name

def get_base_experiment_name(exp_path):
    return get_experiment_group_name(exp_path).split("_Test-")[0]

def infer_config_relative_dir_from_base_name(base_name):
    cached = _CONFIG_DIR_CACHE.get(base_name)
    if cached is not None:
        return cached

    config_root = Path("configs")
    matches = sorted(config_root.glob(f"**/{base_name}.yaml"))
    if not matches:
        _CONFIG_DIR_CACHE[base_name] = None
        return None

    if len(matches) > 1:
        print(f"[WARN] Multiple config matches for {base_name}. Using {matches[0]}")

    rel_dir = matches[0].parent.relative_to(config_root)
    _CONFIG_DIR_CACHE[base_name] = rel_dir
    return rel_dir

def infer_output_root_from_target_dir(target_dir):
    target_path = Path(target_dir)
    parts = target_path.parts
    if not parts:
        return None
    if parts[0] in {"experiments", "configs"} and len(parts) > 1:
        return Path("compare_result").joinpath(*parts[1:])
    return None

def infer_output_root_from_exp_path(exp_path):
    rel_dir = infer_config_relative_dir_from_base_name(get_base_experiment_name(exp_path))
    if rel_dir is None:
        return None
    return Path("compare_result") / rel_dir

def infer_common_output_root(target_dir, exp_dirs):
    target_root = infer_output_root_from_target_dir(target_dir)
    if target_root is not None:
        return target_root

    inferred_roots = {
        root for root in (infer_output_root_from_exp_path(exp_dir) for exp_dir in exp_dirs)
        if root is not None
    }
    if len(inferred_roots) == 1:
        return inferred_roots.pop()
    if len(inferred_roots) > 1:
        print("[WARN] Mixed archive sources detected. Falling back to per-experiment output roots.")
    return None

def get_fast_eval_summary_path(output_root, num_shards=1, shard_index=0):
    if num_shards > 1:
        return output_root / f"fast_eval_metrics_shard_{shard_index + 1}_of_{num_shards}.csv"
    return output_root / "fast_eval_metrics.csv"

def save_fast_eval_summary(results, output_root, num_shards=1, shard_index=0):
    summary_path = get_fast_eval_summary_path(output_root, num_shards=num_shards, shard_index=shard_index)
    df = pd.DataFrame(results)
    if not df.empty and "path" in df.columns:
        df["path"] = df["path"].astype(str)
    df.to_csv(summary_path, index=False)
    print(f"[AUTO] Saved fast-eval summary to: {summary_path}")
    return summary_path

def load_merged_fast_eval_summary(output_root):
    merged_path = output_root / "fast_eval_metrics.csv"
    if merged_path.exists():
        return pd.read_csv(merged_path), merged_path

    shard_paths = sorted(output_root.glob("fast_eval_metrics_shard_*_of_*.csv"))
    if not shard_paths:
        return None, merged_path

    frames = []
    for shard_path in shard_paths:
        try:
            frames.append(pd.read_csv(shard_path))
        except Exception as e:
            print(f"[WARN] Failed to read {shard_path}: {e}")

    if not frames:
        return None, merged_path

    merged_df = pd.concat(frames, ignore_index=True)
    if "name" in merged_df.columns:
        merged_df = merged_df.drop_duplicates(subset=["name"], keep="first")
    merged_df = merged_df.sort_values(["mae", "name"], ascending=[True, True]).reset_index(drop=True)
    merged_df.to_csv(merged_path, index=False)
    print(f"[AUTO] Merged shard summaries to: {merged_path}")
    return merged_df, merged_path

def collect_experiment_dirs(root_path, filter_pattern=None):
    if filter_pattern:
        print(f"[AUTO] Searching for folders matching '{filter_pattern}' in {root_path}")
        if ',' in filter_pattern:
            patterns = filter_pattern.split(',')
            exp_dirs = []
            for p in patterns:
                exp_dirs.extend(list(root_path.glob(p.strip())))
            return list(set(exp_dirs))
        return list(root_path.glob(filter_pattern))

    if (root_path / "config_dump.yaml").exists() or (root_path / "baseline.yaml").exists():
        return [root_path]

    print(f"[AUTO] Searching for all subfolders in {root_path}")
    exp_dirs = []
    for item in root_path.iterdir():
        if not item.is_dir():
            continue
        if (item / "model.pt").exists():
            exp_dirs.append(item)
            continue
        for sub_item in item.iterdir():
            if sub_item.is_dir() and (sub_item / "model.pt").exists():
                exp_dirs.append(sub_item)
    return exp_dirs

def list_experiments(exp_root="experiments"):
    root = Path(exp_root)
    if not root.exists():
        print(f"Directory {root} does not exist.")
        return []
    
    # [NEW] Support hierarchical structure: experiments/{exp_name}/fold_{subject}/
    # Search for model.pt in both:
    # 1. Direct children (old flat structure)
    # 2. Grandchildren (new nested structure)
    exp_dirs = []
    
    for item in root.iterdir():
        if not item.is_dir():
            continue
        
        # Check if this is a fold directory (has model.pt)
        if (item / "model.pt").exists():
            exp_dirs.append(item)
        else:
            # Check if this is an experiment group (has fold_* subdirectories)
            for fold_dir in item.iterdir():
                if fold_dir.is_dir() and (fold_dir / "model.pt").exists():
                    exp_dirs.append(fold_dir)
    
    exp_dirs.sort(key=lambda x: x.name)
    return exp_dirs

def shard_experiment_dirs(exp_dirs, num_shards=1, shard_index=0):
    ordered_dirs = sorted(set(exp_dirs), key=lambda p: (p.parent.name, p.name))
    if num_shards == 1:
        return ordered_dirs
    return ordered_dirs[shard_index::num_shards]

def select_experiments(exp_dirs):
    print("\n=== Available Experiments ===")
    for i, d in enumerate(exp_dirs):
        print(f"{i:3d}: {d.name}")
    
    print("\nEnter IDs to compare (e.g. '0, 3, 5' or 'all') [Default: all]: ")
    try:
        choice = input("Selection: ").strip()
    except EOFError:
        choice = ""  # Default to empty/all on non-interactive
    
    if choice.lower() == 'all' or choice == '':
        return exp_dirs
    
    try:
        indices = [int(x.strip()) for x in choice.split(',')]
        selected = [exp_dirs[i] for i in indices if 0 <= i < len(exp_dirs)]
        return selected
    except ValueError:
        print("Invalid input.")
        return []

def calculate_lag(y_true, y_pred, fs=100, series_offsets=None):
    """
    Calculate temporal lag (phase shift) using Cross-Correlation.
    Positive lag => Prediction is DELAYED (behind truth).
    """
    if series_offsets is not None:
        lags_list = []
        # series_offsets contains start indices of each trial in the concatenated array.
        # The last element of offsets isn't the end, it's the start of the last block? 
        # No, usually logical to have start of each. We need end of each.
        # Check load_and_evaluate logic: offsets.append(curr). curr starts at 0.
        # So offsets = [0, len_1, len_1+len_2, ...]. 
        # Oh, wait.
        # line 549: offsets.append(curr) -> offsets[0]=0.
        # curr += num_samples.
        # So offsets are [0, N1, N1+N2, ...].
        # The lengths are diffs.
        
        # We need to iterate up to len(offsets). 
        # The total length of y_true should match the sum of segments.
        # If offsets is [0, 100], and total len is 100. Then segment is 0:100.
        # If offsets is [0, 100, 200]. segments: 0:100, 100:200.
        
        num_segments = len(series_offsets)
        total_len = len(y_true)
        
        for i in range(num_segments):
            start = series_offsets[i]
            end = series_offsets[i+1] if i < num_segments - 1 else total_len
            
            if start >= end: continue
            
            yt = y_true[start:end]
            yp = y_pred[start:end]
            
            if len(yt) < 10: continue # Skip too short
            
            # Recurse without offsets
            l = calculate_lag(yt, yp, fs, series_offsets=None)
            lags_list.append(l)
            
        if not lags_list: return 0.0
        return np.mean(lags_list)

    correlation = signal.correlate(y_true - np.mean(y_true), y_pred - np.mean(y_pred), mode="full")
    lags = signal.correlation_lags(len(y_true), len(y_pred), mode="full")
    lag_idx = np.argmax(correlation)
    lag_samples = lags[lag_idx]
    
    # [FIX] Positive lag => Prediction is DELAYED (behind truth).
    # correlation_lags returns index shift. Positive lag means y_pred is shifted RIGHT (delayed) vs y_true?
    # No, correlate(a, b). Argmax is shift of b relative to a.
    # If y_pred is delayed (t+k), it matches y_true at lag k.
    # So positive lag matches delay.
    # Original code inverted it (-lag_samples).
    lag_ms = lag_samples * (1000.0 / fs)
    return lag_ms

def calculate_smoothness(y_pred, series_offsets=None):
    """
    Smoothness metric: Std Dev of the first derivative (Jitter).
    Lower is smoother.
    """
    if series_offsets is not None:
        jitters = []
        total_len = len(y_pred)
        num_segments = len(series_offsets)
        for i in range(num_segments):
            start = series_offsets[i]
            end = series_offsets[i+1] if i < num_segments - 1 else total_len
            if start >= end: continue
            yp = y_pred[start:end]
            if len(yp) < 2: continue
            jitters.append(calculate_smoothness(yp, series_offsets=None))
        if not jitters: return 0.0
        return np.mean(jitters)

    diff = np.diff(y_pred, axis=0)
    jitter = np.std(diff)
    return jitter

def apply_post_processing(y_pred_raw, y_true_raw, X_list, Y_list, cfg, 
                          window_size, window_output, stride_eval, est_tick_ranges):
    """
    Applies Overlap-Averaging and Post-LPF based on config.
    Returns (y_pred, y_true, offsets).
    """
    output_dim = y_pred_raw.shape[-1]
    
    # 1. Sequence Overlap Averaging
    if y_pred_raw.ndim == 3 and est_tick_ranges is None and window_output > 1:
        print(f"[EVAL] Applying Sequence Overlap Averaging (H={window_output})...")
        y_pred_list = []
        y_true_list = []
        curr_offset = 0
        for i, trial_x in enumerate(X_list):
            length = len(trial_x)
            limit = length - window_size - window_output + 1
            if limit <= 0: continue
            num_samples = len(range(0, limit, stride_eval))
            if num_samples == 0: continue
            
            trial_pred_win = y_pred_raw[curr_offset : curr_offset + num_samples]
            recon_len = num_samples + window_output - 1
            sum_pred = np.zeros((recon_len, output_dim))
            count_pred = np.zeros((recon_len, 1))
            
            trial_true_full = Y_list[i][window_size : window_size + recon_len]
            
            for t_idx in range(num_samples):
                sum_pred[t_idx : t_idx + window_output] += trial_pred_win[t_idx]
                count_pred[t_idx : t_idx + window_output] += 1
            
            y_pred_trial = sum_pred / count_pred
            y_pred_list.append(y_pred_trial)
            y_true_list.append(trial_true_full[:len(y_pred_trial)])
            curr_offset += num_samples
            
        if y_pred_list:
            y_pred = np.concatenate(y_pred_list, axis=0)
            y_true = np.concatenate(y_true_list, axis=0)
            offsets = [0]
            for i in range(len(y_pred_list)-1):
                offsets.append(offsets[-1] + len(y_pred_list[i]))
        else:
            y_pred = y_pred_raw
            y_true = y_true_raw
            offsets = [0]
    else:
        # Single point mode
        y_pred = y_pred_raw
        y_true = y_true_raw
        offsets = []
        curr = 0
        for i in range(len(X_list)):
            offsets.append(curr)
            length = len(X_list[i])
            max_lookahead = window_output if est_tick_ranges is None else max(est_tick_ranges)
            limit = length - window_size - max_lookahead + 1
            num_samples = len(range(0, limit, stride_eval)) if limit > 0 else 0
            curr += num_samples

    # 2. Post-Processing LPF
    post_cfg = cfg.get("03_eval", {}).get("post_process", {})
    if post_cfg.get("enable_lpf", False):
        cutoff = post_cfg.get("lpf_cutoff", 1.5)
        order = post_cfg.get("lpf_order", 4)
        print(f"[EVAL] Applying Post-Process LPF (fc={cutoff}Hz)...")
        from scipy.signal import butter, filtfilt
        nyq = 0.5 * 100
        b, a = butter(order, cutoff/nyq, btype='low')
        
        trial_preds = []
        for i in range(len(offsets)):
            start = offsets[i]
            end = offsets[i+1] if i < len(offsets)-1 else len(y_pred)
            segment = y_pred[start:end]
            if len(segment) > 3 * order:
                filtered = filtfilt(b, a, segment, axis=0) if segment.ndim == 2 else filtfilt(b, a, segment)
                trial_preds.append(filtered)
            else:
                trial_preds.append(segment)
        if trial_preds:
            y_pred = np.concatenate(trial_preds, axis=0)

    # 3. Overlap Ensemble (Multi-Step predictions only)
    overlap_cfg = cfg.get("03_eval", {}).get("overlap_ensemble", {})
    if overlap_cfg.get("enable", False) and est_tick_ranges and len(est_tick_ranges) > 1:
        method = overlap_cfg.get("method", "mean")
        print(f"[EVAL] Applying Overlap Ensemble (method={method}, H={len(est_tick_ranges)})...")
        # y_pred at this point: (N, H, D) where each sample predicts H future steps
        # For absolute time t, predictions come from multiple run-steps
        H = len(est_tick_ranges)
        new_preds = []
        new_trues = []
        curr = 0
        for trial_idx, trial_x in enumerate(X_list):
            length = len(trial_x)
            max_la = max(est_tick_ranges)
            limit = length - window_size - max_la + 1
            if limit <= 0: continue
            num_samples = len(range(0, limit, stride_eval))
            if num_samples == 0: continue
            
            trial_pred = y_pred[curr:curr+num_samples]  # (num_samples, H, D)
            out_dim = trial_pred.shape[-1] if trial_pred.ndim == 3 else 1
            
            # Reconstruct: for each absolute time step, collect all predictions
            recon_len = num_samples + max_la - 1
            if method == "median":
                # Collect all predictions per time step
                buckets = [[] for _ in range(recon_len)]
                for s_idx in range(num_samples):
                    for h_idx, h_val in enumerate(est_tick_ranges):
                        t_abs = s_idx * stride_eval + h_val - 1
                        if t_abs < recon_len:
                            val = trial_pred[s_idx, h_idx] if trial_pred.ndim == 3 else trial_pred[s_idx]
                            buckets[t_abs].append(val)
                fused = np.array([np.median(b, axis=0) if b else 0.0 for b in buckets])
            else:
                # Mean or Weighted Mean
                sum_pred = np.zeros((recon_len, out_dim)) if out_dim > 1 else np.zeros(recon_len)
                weight_sum = np.zeros(recon_len)
                
                for s_idx in range(num_samples):
                    for h_idx, h_val in enumerate(est_tick_ranges):
                        t_abs = s_idx * stride_eval + h_val - 1
                        if t_abs < recon_len:
                            if method == "weighted_mean":
                                w = overlap_cfg.get("weights_decay", 0.8) ** (h_val - 1)
                            else:
                                w = 1.0
                            val = trial_pred[s_idx, h_idx] if trial_pred.ndim == 3 else trial_pred[s_idx]
                            sum_pred[t_abs] += val * w
                            weight_sum[t_abs] += w
                
                weight_sum[weight_sum == 0] = 1.0
                if out_dim > 1:
                    fused = sum_pred / weight_sum[:, None]
                else:
                    fused = sum_pred / weight_sum
            
            # Get corresponding true values
            trial_true = Y_list[trial_idx][window_size:window_size + len(fused)]
            if len(trial_true) > len(fused):
                trial_true = trial_true[:len(fused)]
            elif len(fused) > len(trial_true):
                fused = fused[:len(trial_true)]
            
            new_preds.append(fused if fused.ndim >= 1 else fused.reshape(-1, 1))
            new_trues.append(trial_true)
            curr += num_samples
        
        if new_preds:
            y_pred = np.concatenate(new_preds, axis=0)
            y_true = np.concatenate(new_trues, axis=0)
            offsets = [0]
            for i in range(len(new_preds) - 1):
                offsets.append(offsets[-1] + len(new_preds[i]))

    # 4. Biomechanical Clamp (limit speed change rate)
    biomech_max_acc = cfg.get("03_eval", {}).get("biomech_clamp_max_acc", None)
    if biomech_max_acc is not None and biomech_max_acc > 0:
        fs_eval = 100.0  # Hz
        max_dv = float(biomech_max_acc) / fs_eval  # m/s per sample
        print(f"[EVAL] Applying Biomechanical Clamp (max_acc={biomech_max_acc} m/s², max_dv={max_dv:.4f} m/s/sample)...")
        if offsets:
            trial_preds_clamped = []
            for i in range(len(offsets)):
                start = offsets[i]
                end = offsets[i+1] if i < len(offsets)-1 else len(y_pred)
                seg = y_pred[start:end].copy()
                for t in range(1, len(seg)):
                    dv = seg[t] - seg[t-1]
                    if np.any(np.abs(dv) > max_dv):
                        seg[t] = seg[t-1] + np.clip(dv, -max_dv, max_dv)
                trial_preds_clamped.append(seg)
            y_pred = np.concatenate(trial_preds_clamped, axis=0)
        else:
            seg = y_pred.copy()
            for t in range(1, len(seg)):
                dv = seg[t] - seg[t-1]
                if np.any(np.abs(dv) > max_dv):
                    seg[t] = seg[t-1] + np.clip(dv, -max_dv, max_dv)
            y_pred = seg

    # 5. EMA Post-Filter
    ema_alpha = cfg.get("03_eval", {}).get("ema_alpha", None)
    if ema_alpha is not None and ema_alpha > 0:
        alpha = float(ema_alpha)
        print(f"[EVAL] Applying EMA Post-Filter (alpha={alpha})...")
        # Apply per-trial to avoid bleeding across trials
        if offsets:
            trial_preds_ema = []
            for i in range(len(offsets)):
                start = offsets[i]
                end = offsets[i+1] if i < len(offsets)-1 else len(y_pred)
                seg = y_pred[start:end].copy()
                if len(seg) > 1:
                    for t in range(1, len(seg)):
                        seg[t] = alpha * seg[t] + (1 - alpha) * seg[t-1]
                trial_preds_ema.append(seg)
            y_pred = np.concatenate(trial_preds_ema, axis=0)
        else:
            seg = y_pred.copy()
            for t in range(1, len(seg)):
                seg[t] = alpha * seg[t] + (1 - alpha) * seg[t-1]
            y_pred = seg

    return y_pred, y_true, offsets

def _sequential_ar_inference(model, X_list, Y_list, window_size, window_output,
                              stride_eval, est_tick_ranges, ar_k, device):
    """
    AR 모델 전용 순차 추론.
    각 윈도우에서 이전 모델 예측값을 AR 채널로 사용 (cold start = zeros).
    반환: (y_pred_raw, y_true_raw) — apply_post_processing 입력과 동일한 형식.
    """
    import numpy as np
    max_la = max(est_tick_ranges) if est_tick_ranges else window_output
    all_preds, all_trues = [], []

    for X_s, Y_s in zip(X_list, Y_list):
        if isinstance(X_s, torch.Tensor):
            X_s = X_s.numpy()
        if isinstance(Y_s, torch.Tensor):
            Y_s = Y_s.numpy()
        T = X_s.shape[0]
        valid_end = T - window_size - max_la + 1
        if valid_end <= 0:
            continue

        prev_ar = torch.zeros(1, ar_k, device=device)  # cold start

        for t in range(0, valid_end, stride_eval):
            x_win = torch.from_numpy(X_s[t:t + window_size]).float().unsqueeze(0).to(device)  # (1, T, D)
            ar_ch = prev_ar.unsqueeze(1).expand(1, window_size, ar_k)   # (1, T, ar_k)
            x_ar = torch.cat([x_win, ar_ch], dim=-1)

            with torch.no_grad():
                out = extract_primary_prediction(model(x_ar))  # (1, H, D_out) or (1, D_out)

            # Update AR channel: use first-horizon prediction
            if out.dim() == 3:
                prev_ar = out[0:1, 0, :ar_k].detach()   # (1, ar_k)
            else:
                prev_ar = out[0:1, :ar_k].detach()       # (1, ar_k)

            all_preds.append(out.cpu().numpy())

            # Ground truth at first prediction target
            if est_tick_ranges:
                tgt_idx = t + window_size + est_tick_ranges[0] - 1
            else:
                tgt_idx = t + window_size
            if tgt_idx < T:
                all_trues.append(Y_s[tgt_idx:tgt_idx + 1])  # (1, D_out)

    if not all_preds:
        return np.zeros((0,)), np.zeros((0,))
    return np.concatenate(all_preds, axis=0), np.concatenate(all_trues, axis=0)


def run_tta_inference(model, loader, device, n_passes=5, noise_std=0.02, aggregation="mean"):
    """
    Test-Time Augmentation: run N forward passes (1 clean + N-1 noisy),
    aggregate predictions via mean or median.
    """
    all_pass_preds = []

    for p in range(n_passes):
        y_preds = []
        with torch.no_grad():
            for xb, yb in loader:
                xb = xb.to(device)
                if p > 0:  # passes 1..N-1 add noise
                    xb = xb + torch.randn_like(xb) * noise_std
                out = extract_primary_prediction(model(xb))
                y_preds.append(out.cpu().numpy())
        all_pass_preds.append(np.concatenate(y_preds, axis=0))

    stacked = np.stack(all_pass_preds, axis=0)  # (N, samples, ...)
    if aggregation == "median":
        y_pred_tta = np.median(stacked, axis=0)
    else:
        y_pred_tta = np.mean(stacked, axis=0)

    # Collect y_true from final pass (same across all passes)
    y_trues = []
    with torch.no_grad():
        for xb, yb in loader:
            y_trues.append(yb.cpu().numpy())
    y_true_tta = np.concatenate(y_trues, axis=0)

    return y_pred_tta, y_true_tta


def load_and_evaluate(exp_dir, device, return_seqs=False, return_extras=False, stride_eval_override=None):
    """
    Loads model and config, rebuilds test dataset, runs inference.

    [REFACTORED] Training 스크립트(SpeedEstimator_TCN_MLP_experiments.py)의
    config 해석, 데이터 로딩, normalization, 모델 초기화 경로를 정확히 재사용합니다.

    Args:
      return_extras: if True, also return 'model', 'cfg', 'c_in_vars', 'c_out_vars',
                     'scaler' (mean, scale), 'test_sub' for downstream reuse.

    참조:
      - Config 해석: train_experiment() line 1335-1349 + CV loop line 1833-1905
      - 모델 초기화: train_experiment() line 1275-1410
      - Normalization: CV loop line 2019-2042
    """
    import re
    import yaml
    
    exp_path = Path(exp_dir)
    model_path = exp_path / "model.pt"
    
    if not model_path.exists():
        print(f"Skipping {exp_dir.name}: missing model.")
        return None

    folder_name = exp_path.name
    model_norm_type = "layer" # Default
    
    # =========================================================================
    # Step 1: YAML Config 로드 (Training CV loop line 1833과 동일)
    # =========================================================================
    # [NEW] Handle both flat and hierarchical structures
    if folder_name.startswith("fold_"):
        exp_name = exp_path.parent.name
        match = re.search(r"fold_((?:m\d+_)?S\d+)", folder_name)
        test_sub = match.group(1) if match else folder_name.replace("fold_", "")
    else:
        exp_name_match = re.match(r"^(.+?)_Test-", folder_name)
        if not exp_name_match:
            print(f"Could not parse experiment name from {folder_name}. Skipping.")
            return None
        exp_name = exp_name_match.group(1)
        
        match = re.search(r"Test-((?:m\d+_)?S\d+)", folder_name)
        if match:
            test_sub = match.group(1)
        else:
            print(f"Could not infer test subject from {folder_name}. Skipping.")
            return None
    
    original_config_path = Path(f"configs/{exp_name}.yaml")
    
    config_to_load = None
    if original_config_path.exists():
        config_to_load = original_config_path
    else:
         # Fallback to local config.yaml
         local_config = exp_path / "config.yaml"
         if local_config.exists():
             config_to_load = local_config
             print(f"[WARN] Global config {original_config_path} not found. Using local: {local_config}")
         else:
             print(f"Config file not found: {original_config_path} AND {local_config}. Skipping.")
             return None
    
    # [FIX-CORRECTION] Training script DOES load baseline.yaml implicitly (lines 2595-2632).
    # So we MUST load it here too for partial configs like 'exp_single_ema_a080'.
    base_config_path = Path("configs/baseline.yaml")
    if not base_config_path.exists():
        base_config_path = Path("config_backups/baseline.yaml")
    cfg = {}
    
    def deep_merge(base, override):
        for k, v in override.items():
            if k in base and isinstance(base[k], dict) and isinstance(v, dict):
                deep_merge(base[k], v)
            else:
                base[k] = v

    if base_config_path.exists():
        with open(base_config_path, 'r') as f:
            cfg = yaml.safe_load(f)
            print(f"[EVAL] Loaded base defaults from {base_config_path}")
    
    with open(config_to_load, 'r') as f:
        exp_cfg = yaml.safe_load(f)
        # Use deep_merge for inheritance
        deep_merge(cfg, exp_cfg)
    
    print(f"\n{'='*60}")
    print(f"[EVAL] {folder_name}")
    print(f"[EVAL] Config: {original_config_path}")
    
    # test_sub is already extracted above
    
    # =========================================================================
    # Step 3: Config 파싱 - Training CV loop (line 1833-1905)과 동일한 경로
    # =========================================================================
    # input_vars / output_vars (Training line 1833-1840)
    curr_input_vars = cfg.get("input_vars")
    curr_output_vars = cfg.get("output_vars")
    if not curr_input_vars and 'shared' in cfg:
        curr_input_vars = cfg['shared'].get('input_vars')
    if not curr_output_vars and 'shared' in cfg:
        curr_output_vars = cfg['shared'].get('output_vars')
    if not curr_input_vars and '01_construction' in cfg:
        curr_input_vars = cfg['01_construction'].get('inputs')
    if not curr_output_vars and '01_construction' in cfg:
        curr_output_vars = cfg['01_construction'].get('outputs')
    
    if not curr_input_vars or not curr_output_vars:
        print(f"Skipping {folder_name}: Missing input/output vars in config.")
        return None
    
    # Tuple conversion (Training line 1932-1940)
    def parse_vars(var_list_from_yaml):
        if not var_list_from_yaml: return []
        parsed = []
        for item in var_list_from_yaml:
            gpath = item[0]
            vars = item[1]
            parsed.append((gpath, vars))
        return parsed
    
    c_in_vars = parse_vars(curr_input_vars)
    c_out_vars = parse_vars(curr_output_vars)
    
    # subjects / conditions (Training line 1842-1846)
    if not cfg.get('subjects') and 'shared' in cfg:
        cfg['subjects'] = cfg['shared'].get('subjects', [])
    if not cfg.get('conditions') and 'shared' in cfg:
        cfg['conditions'] = cfg['shared'].get('conditions', [])
    
    curr_cond_names = cfg.get("conditions", [])
    
    # =========================================================================
    # Step 4: Data 파라미터 - Training과 동일 (line 1848-1860)
    # =========================================================================
    # window_size (Training line 1849-1851)
    curr_window = cfg.get("time_window_input")
    if not curr_window and 'shared' in cfg and 'data' in cfg['shared']:
        curr_window = cfg['shared']['data'].get('window_size')
    if not curr_window:
        curr_window = 200  # Default
    
    # est_tick_ranges (Training line 1853-1856, 1858-1860)
    curr_est_tick_ranges = cfg.get("est_tick_ranges")
    if not curr_est_tick_ranges and 'shared' in cfg and 'data' in cfg['shared']:
        val = cfg['shared']['data'].get('est_tick_ranges')
        if val is not None:
            curr_est_tick_ranges = val
        else:
            y_delay = cfg['shared']['data'].get('y_delay', 5)
            curr_est_tick_ranges = [y_delay] if isinstance(y_delay, int) else y_delay
    if curr_est_tick_ranges:
        cfg["est_tick_ranges"] = curr_est_tick_ranges
    
    # LPF (Training line 1868-1879)
    curr_lpf_cutoff = cfg.get("lpf_cutoff")
    if curr_lpf_cutoff is None and '01_construction' in cfg:
        curr_lpf_cutoff = cfg['01_construction'].get('lpf_cutoff')
    if curr_lpf_cutoff is None and 'shared' in cfg:
        curr_lpf_cutoff = cfg['shared'].get('lpf_cutoff')
    if curr_lpf_cutoff is None:
        curr_lpf_cutoff = 0.5
        
    curr_lpf_order = cfg.get("lpf_order")
    if curr_lpf_order is None and '01_construction' in cfg:
        curr_lpf_order = cfg['01_construction'].get('lpf_order')
    if curr_lpf_order is None and 'shared' in cfg:
        curr_lpf_order = cfg['shared'].get('lpf_order')
    if curr_lpf_order is None:
        curr_lpf_order = 4

    curr_target_lpf_mode = cfg.get("target_lpf_mode")
    if curr_target_lpf_mode is None and '01_construction' in cfg:
        curr_target_lpf_mode = cfg['01_construction'].get('target_lpf_mode')
    if curr_target_lpf_mode is None and 'shared' in cfg:
        curr_target_lpf_mode = cfg['shared'].get('target_lpf_mode')
    if curr_target_lpf_mode is None and '02_train' in cfg and 'data' in cfg['02_train']:
        curr_target_lpf_mode = cfg['02_train']['data'].get('target_lpf_mode')
    if curr_target_lpf_mode is None:
        curr_target_lpf_mode = "zero_phase"
    
    # Data flags (Training line 1968-1971)
    data_cfg_now = cfg.get("02_train", {}).get("data", {})
    use_phys_vel = data_cfg_now.get("use_physical_velocity_model", False)
    use_gait_ph  = data_cfg_now.get("use_gait_phase", False)

    # Input LPF (must match training preprocessing)
    curr_input_lpf_cutoff = cfg.get("input_lpf_cutoff_hz")
    if curr_input_lpf_cutoff is None and 'shared' in cfg:
        curr_input_lpf_cutoff = cfg['shared'].get('input_lpf_cutoff_hz')
    if curr_input_lpf_cutoff is None and '02_train' in cfg and 'data' in cfg['02_train']:
        curr_input_lpf_cutoff = cfg['02_train']['data'].get('input_lpf_cutoff_hz')
    curr_input_lpf_order = 4
    if 'shared' in cfg and cfg['shared'].get('input_lpf_order'):
        curr_input_lpf_order = cfg['shared']['input_lpf_order']
    elif '02_train' in cfg and 'data' in cfg['02_train'] and cfg['02_train']['data'].get('input_lpf_order'):
        curr_input_lpf_order = cfg['02_train']['data']['input_lpf_order']
    # Per-group input LPF
    curr_input_lpf_per_group = None
    if '02_train' in cfg and 'data' in cfg['02_train']:
        curr_input_lpf_per_group = cfg['02_train']['data'].get('input_lpf_per_group')
    if curr_input_lpf_per_group is None and 'shared' in cfg:
        curr_input_lpf_per_group = cfg['shared'].get('input_lpf_per_group')
    
    # =========================================================================
    # Step 5: train_experiment 내부와 동일한 data_cfg 파싱 (line 1335-1349)
    # =========================================================================
    train_cfg_section = cfg.get("02_train", {})
    data_cfg = train_cfg_section.get("data", {})
    if not data_cfg: data_cfg = cfg.get("data", {})
    
    # Training line 1341-1344: 정확한 windowing 파라미터
    window_size = data_cfg.get("window_size") or cfg.get("window_size", 200)
    window_output = data_cfg.get("window_output") or data_cfg.get("time_window_output") or cfg.get("window_output", 1)
    data_stride = data_cfg.get("stride") or cfg.get("stride", 5)
    
    # Training line 1346-1347: est_tick_ranges
    est_tick_ranges = data_cfg.get("est_tick_ranges") if data_cfg else cfg.get("est_tick_ranges")
    target_transform = data_cfg.get("target_transform")
    
    # Evaluation stride = 1 by default. Fast-eval can override this to a coarser stride.
    stride_eval = stride_eval_override if stride_eval_override is not None else 1
    
    print(f"[EVAL] Data: win_in={window_size}, win_out={window_output}, stride_train={data_stride}, stride_eval={stride_eval}")
    print(
        f"[EVAL] est_tick_ranges={est_tick_ranges}, lpf_cutoff={curr_lpf_cutoff}, "
        f"lpf_order={curr_lpf_order}, target_lpf_mode={curr_target_lpf_mode}"
    )
    
    # =========================================================================
    # Step 6: 데이터 로딩 (Training CV loop line 1994-2002과 동일)
    # =========================================================================
    print(f"[EVAL] Loading Test Data for Subject {test_sub}...")
    
    X_list, Y_list = main_script.build_nn_dataset_multi(
        cfg,
        [test_sub],
        curr_cond_names,
        c_in_vars, c_out_vars,
        window_size,       # Training과 동일한 window_size (YAML에서)
        window_output,     # Training과 동일한 window_output (YAML에서)
        stride_eval,       # Evaluation은 stride=1
        subject_selection=None,
        condition_selection=CONDITION_SELECTION,
        lpf_cutoff=curr_lpf_cutoff,
        lpf_order=curr_lpf_order,
        target_lpf_mode=curr_target_lpf_mode,
        est_tick_ranges=est_tick_ranges,
        use_physical_velocity_model=use_phys_vel,
        use_gait_phase=use_gait_ph,
        input_lpf_cutoff=curr_input_lpf_cutoff,
        input_lpf_order=curr_input_lpf_order,
        input_lpf_per_group=curr_input_lpf_per_group
    )
    
    if not X_list:
        print(f"[ERROR] No test data found for {test_sub}. Check prefixes and data_sources.")
        return None
    
    # =========================================================================
    # Step 7: Normalization - Training과 동일 (line 2019-2042)
    # scaler.npz을 로드하여 Training에서 저장한 mean/scale 적용
    # =========================================================================
    scaler_path = exp_path / "scaler.npz"
    if scaler_path.exists():
        scaler = np.load(scaler_path)
        mean = scaler['mean']
        scale = scaler['scale']
        print(f"[EVAL] Applying scaler from {scaler_path} (mean shape={mean.shape})")
        for i in range(len(X_list)):
            X_list[i] = (X_list[i] - mean) / scale
    else:
        print(f"[WARN] No scaler.npz found in {exp_path}! Model expects normalized input.")
    
    # =========================================================================
    # Step 8: Dataset 생성 - Training line 1357-1359와 동일
    # =========================================================================
    ds = LazyWindowDataset(
        X_list, Y_list, 
        window_size, window_output, stride_eval,
        target_mode="sequence", 
        est_tick_ranges=est_tick_ranges,
        target_transform=target_transform,
        input_mean=mean if scaler_path.exists() else None,
        input_std=scale if scaler_path.exists() else None,
    )
    loader = DataLoader(ds, batch_size=_EVAL_BATCH_SIZE, shuffle=False, num_workers=0)
    
    # =========================================================================
    # Step 9: 모델 초기화 - train_experiment (line 1275-1410)과 동일
    # =========================================================================
    # [NEW] Merge overrides from shared.model/data (Deep Merge)
    shared_model = cfg.get("shared", {}).get("model", {})
    shared_data  = cfg.get("shared", {}).get("data", {})
    
    def deep_merge(base, override):
        for k, v in override.items():
            if k in base and isinstance(base[k], dict) and isinstance(v, dict):
                deep_merge(base[k], v)
            else:
                base[k] = v

    model_cfg = train_cfg_section.get("model", {})
    if not model_cfg:
        data_cfg_m = train_cfg_section.get("data", {})
        if data_cfg_m and "channels" in data_cfg_m: model_cfg = data_cfg_m
    if not model_cfg: model_cfg = cfg.get("model") or {}
    
    deep_merge(model_cfg, shared_model)
    
    # AR Feedback check
    ar_cfg = model_cfg.get("ar_feedback", {})
    ar_enable = ar_cfg.get("enable", False)
    ar_k = ar_cfg.get("k", 1)

    input_dim = X_list[0].shape[1]
    if ar_enable:
        print(f"[EVAL] AR Feedback detected. Augmenting input_dim {input_dim} -> {input_dim + ar_k}")
        input_dim += ar_k

    output_dim = Y_list[0].shape[1]
    
    if est_tick_ranges:
        horizon = len(est_tick_ranges)
    else:
        horizon = window_output
    
    model_norm_type = model_cfg.get("model_norm", None)
    input_norm_type = model_cfg.get("input_norm_type", model_norm_type or "layer")
    
    # TCN 파라미터 (Training line 1286-1310)
    tcn_channels = model_cfg.get("channels") or cfg.get("tcn_channels", (64, 64, 128))
    kernel_size = model_cfg.get("kernel_size") or cfg.get("kernel_size", 3)
    
    dropout_p = model_cfg.get("dropout")
    if dropout_p is None:
        if "dropout" in cfg.get("data", {}): dropout_p = cfg["data"]["dropout"]
        else: dropout_p = cfg.get("dropout_p", 0.5)
    dropout_p = float(dropout_p)
    
    head_dropout = model_cfg.get("head_dropout")
    mlp_hidden = model_cfg.get("head_hidden") or cfg.get("mlp_hidden", 128)
    
    # Normalization flags (Training line 1302-1309)
    use_input_norm = cfg.get("data", {}).get("normalize", True)
    if use_input_norm is None:
        use_input_norm = cfg.get("use_input_norm", True)
    
    tcn_norm = model_norm_type
    mlp_norm = model_norm_type
    
    print(f"[EVAL] Model: type={model_cfg.get('type', 'TCN')}, ch={tcn_channels}, k={kernel_size}")
    print(f"[EVAL] Dropout={dropout_p}, head_dropout={head_dropout}, hidden={mlp_hidden}")
    print(f"[EVAL] use_input_norm={use_input_norm}, norm_type={model_norm_type}")
    print(f"[EVAL] input_dim={input_dim}, output_dim={output_dim}, horizon={horizon}")
    
    # Model Instantiation (Training line 1374-1410)
    model_type = model_cfg.get("type", "TCN")
    
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
        model = TCN_GRU_Head(**common_args, gru_hidden=gru_h)
    elif model_type == "CausalSmoothTCN_GRU":
        gru_h = model_cfg.get("gru_hidden", 32)
        smooth_w = model_cfg.get("smooth_window", 11)
        model = CausalSmoothTCN_GRU(**common_args, gru_hidden=gru_h, smooth_window=smooth_w)
    elif model_type == "MultiScaleTCN_GRU":
        gru_h = model_cfg.get("gru_hidden", 32)
        kernel_sizes = tuple(model_cfg.get("kernel_sizes", [3, 5, 7]))
        model = MultiScaleTCN_GRU(**common_args, gru_hidden=gru_h, kernel_sizes=kernel_sizes)
    elif model_type == "DualBranchTCN_GRU":
        gru_h = model_cfg.get("gru_hidden", 32)
        branch_input_dims = model_cfg.get("branch_input_dims")
        branch_channels = model_cfg.get("branch_channels")
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
        model = MultiBranchTCN_GRU(
            **common_args,
            gru_hidden=gru_h,
            branch_input_dims=branch_input_dims,
            branch_channels=branch_channels,
        )
    elif model_type == "FiLMConditionedTCN_GRU":
        model = FiLMConditionedTCN_GRU(
            **common_args,
            gru_hidden=model_cfg.get("gru_hidden", 32),
            branch_input_dims=model_cfg.get("branch_input_dims"),
            branch_channels=model_cfg.get("branch_channels"),
            film_scale=model_cfg.get("film_scale", 0.5),
        )
    elif model_type == "BoundedResidualTCN_GRU":
        model = BoundedResidualTCN_GRU(
            **common_args,
            gru_hidden=model_cfg.get("gru_hidden", 32),
            branch_input_dims=model_cfg.get("branch_input_dims"),
            branch_channels=model_cfg.get("branch_channels"),
            residual_scale=model_cfg.get("residual_scale", 0.30),
        )
    elif model_type == "AffineCalibTCN_GRU":
        model = AffineCalibTCN_GRU(
            **common_args,
            gru_hidden=model_cfg.get("gru_hidden", 32),
            branch_input_dims=model_cfg.get("branch_input_dims"),
            branch_channels=model_cfg.get("branch_channels"),
            alpha_scale=model_cfg.get("alpha_scale", 0.30),
            beta_scale=model_cfg.get("beta_scale", 0.25),
        )
    elif model_type == "PositiveResidualTCN_GRU":
        model = PositiveResidualTCN_GRU(
            **common_args,
            gru_hidden=model_cfg.get("gru_hidden", 32),
            branch_input_dims=model_cfg.get("branch_input_dims"),
            branch_channels=model_cfg.get("branch_channels"),
            residual_scale=model_cfg.get("residual_scale", 0.25),
            gate_scale=model_cfg.get("gate_scale", 1.0),
        )
    elif model_type == "AssistExpertTCN_GRU":
        model = AssistExpertTCN_GRU(
            **common_args,
            gru_hidden=model_cfg.get("gru_hidden", 32),
            branch_input_dims=model_cfg.get("branch_input_dims"),
            branch_channels=model_cfg.get("branch_channels"),
            gate_max=model_cfg.get("gate_max", 0.6),
        )
    elif model_type == "SlowFastCalibTCN_GRU":
        model = SlowFastCalibTCN_GRU(
            **common_args,
            gru_hidden=model_cfg.get("gru_hidden", 32),
            branch_input_dims=model_cfg.get("branch_input_dims"),
            branch_channels=model_cfg.get("branch_channels"),
            bias_scale=model_cfg.get("bias_scale", 0.20),
            residual_scale=model_cfg.get("residual_scale", 0.12),
        )
    elif model_type == "StructuredSlowFastTCN_GRU":
        model = StructuredSlowFastTCN_GRU(
            **common_args,
            gru_hidden=model_cfg.get("gru_hidden", 32),
            branch_input_dims=model_cfg.get("branch_input_dims"),
            branch_channels=model_cfg.get("branch_channels"),
            bias_scale=model_cfg.get("bias_scale", 0.18),
            residual_scale=model_cfg.get("residual_scale", 0.10),
            use_aux_head=model_cfg.get("use_aux_head", False),
            aux_dim=model_cfg.get("aux_dim", 1),
            use_subject_calibration=model_cfg.get("use_subject_calibration", False),
            subject_ctx_dims=model_cfg.get("subject_ctx_dims", 0),
            alpha_scale=model_cfg.get("alpha_scale", 0.08),
            beta_scale=model_cfg.get("beta_scale", 0.05),
        )
    elif model_type == "TCN_GRU_MDN":
        gru_h = model_cfg.get("gru_hidden", 32)
        num_mixtures = int(model_cfg.get("num_mixtures", 5))
        model = TCN_GRU_MDN(**common_args, gru_hidden=gru_h, num_mixtures=num_mixtures)
    else:
        model = TCN_MLP(**common_args)
    
    model.to(device)
    
    # =========================================================================
    # Step 10: Weights 로드 및 Inference
    # =========================================================================
    ckpt = torch.load(model_path, map_location=device)
    model.load_state_dict(ckpt["state_dict"] if "state_dict" in ckpt else ckpt, strict=False)
    model.eval()
    
    if ar_enable:
        # [AR Feedback] Sequential inference: each window uses previous model prediction as AR channel
        print(f"[EVAL] AR Feedback: running sequential inference (ar_k={ar_k})")
        y_pred_raw, y_true_raw = _sequential_ar_inference(
            model, X_list, Y_list, window_size, window_output,
            stride_eval, est_tick_ranges, ar_k, device
        )
    else:
        # Check for TTA config
        tta_cfg = cfg.get("03_eval", {}).get("tta", {})
        if not tta_cfg:
            tta_cfg = cfg.get("shared", {}).get("tta", {})
        tta_enable = tta_cfg.get("enable", False)

        if tta_enable:
            tta_n = tta_cfg.get("n_passes", 5)
            tta_noise = tta_cfg.get("noise_std", 0.02)
            tta_agg = tta_cfg.get("aggregation", "mean")
            print(f"[EVAL] TTA enabled: {tta_n} passes, noise={tta_noise}, agg={tta_agg}")
            y_pred_raw, y_true_raw = run_tta_inference(
                model, loader, device,
                n_passes=tta_n, noise_std=tta_noise, aggregation=tta_agg
            )
        else:
            # Standard batch inference
            y_preds = []
            y_trues = []
            y_stds = []
            with torch.no_grad():
                for xb, yb in loader:
                    xb = xb.to(device)
                    if model_type == "TCN_GRU_MDN":
                        out, out_std = model(xb, return_std=True)
                        y_stds.append(out_std.cpu().numpy())
                    else:
                        out = extract_primary_prediction(model(xb))
                    y_preds.append(out.cpu().numpy())
                    y_trues.append(yb.cpu().numpy())
            y_pred_raw = np.concatenate(y_preds, axis=0)
            y_true_raw = np.concatenate(y_trues, axis=0)
    
    y_pred, y_true, offsets = apply_post_processing(
        y_pred_raw, y_true_raw, X_list, Y_list, cfg,
        window_size, window_output, stride_eval, est_tick_ranges
    )

    if target_transform:
        flat_x = []
        for xb, _ in loader:
            flat_x.append(xb.cpu().numpy())
        if flat_x:
            x_ref = np.concatenate(flat_x, axis=0)
            y_pred = inverse_target_transform(
                y_pred, x_ref,
                target_transform=target_transform,
                input_mean=mean if scaler_path.exists() else None,
                input_std=scale if scaler_path.exists() else None,
            )
            y_true = inverse_target_transform(
                y_true, x_ref,
                target_transform=target_transform,
                input_mean=mean if scaler_path.exists() else None,
                input_std=scale if scaler_path.exists() else None,
            )

    print(f"[EVAL] Final y_pred shape: {y_pred.shape}")
    
    # =========================================================================
    # Step 11: Metric 계산 (Re-calculation after averaging/filtering)
    # =========================================================================
    vp = y_pred.squeeze()
    vt = y_true.squeeze()
    
    # Ensure they are 1D for metric functions
    if vp.ndim > 1: vp = vp[:, 0]
    if vt.ndim > 1: vt = vt[:, 0]
    
    mae = np.mean(np.abs(vp - vt))
    rmse = np.sqrt(np.mean((vp - vt)**2))
    
    ss_res = np.sum((vt - vp)**2)
    ss_tot = np.sum((vt - np.mean(vt))**2)
    r2 = 1 - (ss_res / ss_tot) if ss_tot > 0 else 0.0
    
    lag_ms = calculate_lag(vt, vp, fs=100, series_offsets=offsets)
    smoothness = calculate_smoothness(vp, series_offsets=offsets)
    
    print(f"[EVAL] Results: MAE={mae:.4f}, RMSE={rmse:.4f}, R²={r2:.4f}, Lag={lag_ms:.1f}ms")
    
    ret = {
        "name": folder_name,
        "mae": mae,
        "rmse": rmse,
        "r2": r2,
        "lag_ms": lag_ms,
        "smoothness": smoothness,
        "folder_path": exp_dir,
        "est_tick_ranges": est_tick_ranges,
        "series_offsets": offsets
    }
    metrics_path = exp_path / "metrics.json"
    if metrics_path.exists():
        try:
            with open(metrics_path, "r") as f_metrics:
                train_metrics = json.load(f_metrics)
            for key in ["n_params", "host_latency_ms", "test_mae", "test_rmse", "test_r2"]:
                if key in train_metrics:
                    ret[key] = train_metrics[key]
        except Exception as e:
            print(f"[WARN] Failed to read metrics.json for {folder_name}: {e}")

    if model_type == "TCN_GRU_MDN" and 'y_stds' in locals() and y_stds:
        std_raw = np.concatenate(y_stds, axis=0)
        ret["mean_pred_std_raw"] = float(np.mean(std_raw))
        ret["median_pred_std_raw"] = float(np.median(std_raw))
    
    # [MOVED UP] offsets calculation was here

    if return_seqs:
        ret["y_true_seq"] = y_true
        ret["y_pred_seq"] = y_pred

    if return_extras:
        ret["model"] = model
        ret["cfg"] = cfg
        ret["c_in_vars"] = c_in_vars
        ret["c_out_vars"] = c_out_vars
        ret["test_sub"] = test_sub
        # scaler info
        if scaler_path.exists():
            scaler = np.load(scaler_path)
            ret["scaler_mean"] = scaler['mean']
            ret["scaler_scale"] = scaler['scale']

    return ret

def parse_vars(var_list_from_yaml):
    """Parse nested variable list from YAML config."""
    if not var_list_from_yaml: return []
    parsed = []
    for item in var_list_from_yaml:
        if isinstance(item, (list, tuple)) and len(item) == 2:
            gpath = item[0]
            vars = item[1]
            parsed.append((gpath, vars))
    return parsed

# Feature groups for importance plot labeling (new experiment groups A-E)
_EXTRA_FEATURE_GROUPS = {
    'global_acc': ['global_acc_x', 'global_acc_y', 'global_acc_z'],
    'global_gyr': ['global_gyr_x', 'global_gyr_y', 'global_gyr_z'],
    'global_vel': ['global_vel_y', 'global_vel_treadmill'],
    'gait':       ['gait_phase_l', 'gait_phase_r', 'contact_l', 'contact_r', 'stride_freq_l', 'stride_freq_r'],
    'sub_info':   ['height', 'weight'],
    'torque':     ['torque'],
    'thigh':      ['thigh_angle'],
    'hip':        ['hip_angle'],
}


def get_feature_names(cfg, input_vars):
    """Reconstruct feature names based on config and input variables."""
    feature_names = []
    for gpath, vars in input_vars:
        gpath_lower = gpath.lower()

        # sub_info: anthropometric constants
        if gpath == 'sub_info':
            for v in vars:
                feature_names.append(f"SubInfo_{v}")
            continue

        if 'back_imu' in gpath_lower:
            prefix = "IMU"
        elif 'robot' in gpath_lower:
            if 'left' in gpath_lower: prefix = "Robot_L"
            elif 'right' in gpath_lower: prefix = "Robot_R"
            else: prefix = "Robot"
        elif 'grf' in gpath_lower:
            side = 'L' if 'left' in gpath_lower else 'R'
            prefix = f"GRF_{side}"
        elif 'kin_qdot' in gpath_lower:
            prefix = "J_Vel"
        elif 'kin_q' in gpath_lower:
            prefix = "Deg"
        else:
            prefix = gpath.split('/')[-1]

        for v in vars:
            # Global IMU vars
            if v.startswith('global_'):
                feature_names.append(f"IMU_Global_{v[7:]}")
                continue
            # Gait phase / contact
            if v in ('gait_phase_l', 'gait_phase_r', 'contact_l', 'contact_r'):
                tag = 'Phase' if 'gait_phase' in v else 'Contact'
                side = 'L' if v.endswith('_l') else 'R'
                feature_names.append(f"Gait_{tag}_{side}")
                continue
            v_clean = v.replace('ankle_angle', 'Ankle').replace('knee_angle', 'Knee').replace('hip_', 'Hip_')
            v_clean = v_clean.replace('flexion', 'Flex').replace('adduction', 'Add').replace('rotation', 'Rot')
            v_clean = v_clean.replace('_l', '_L').replace('_r', '_R')
            feature_names.append(f"{prefix}_{v_clean}")

    # Add gait phase if enabled in config
    if cfg.get("02_train", {}).get("data", {}).get("use_gait_phase") or cfg.get("use_gait_phase"):
        feature_names.append("GaitPhase_L")
        feature_names.append("GaitPhase_R")

    return feature_names

def plot_feature_importance(importances, feature_names, save_path, title="Feature Importance"):
    """Plot horizontal bar chart of feature importance."""
    if len(importances) == 0: return
    
    # Sort for plotting
    sorted_idx = np.argsort(importances)
    
    # Dynamic height to ensure ALL features are visible (0.3 inch per feat for better spacing)
    fig_h = max(8, len(importances) * 0.3)
    plt.figure(figsize=(12, fig_h)) # Slightly wider for better labels
    
    plt.barh(range(len(importances)), importances[sorted_idx], color='lightgreen', edgecolor='black', alpha=0.8)
    plt.yticks(range(len(importances)), [feature_names[i] for i in sorted_idx], fontsize=10)
    plt.xlabel("MAE Increase (Importance)", fontsize=12)
    plt.title(title, fontsize=14, fontweight='bold')
    plt.grid(axis='x', linestyle='--', alpha=0.5)
    plt.tight_layout()
    plt.savefig(save_path, dpi=150) # Higher DPI for clarity
    plt.close()
    print(f"  [SAVED] Feature Importance plot: {save_path}")

def main():
    exp_dirs = list_experiments()
    if not exp_dirs:
        print("No experiments found.")
        return

    selected = select_experiments(exp_dirs)
    if not selected:
        print("None selected.")
        return
        
    print(f"\nAnalyzing {len(selected)} experiments...")
    
    # [NEW] Single Model Mode
    if len(selected) == 1:
        print("[MODE] Single Model Detailed Analysis")
        analyze_detailed_single(selected[0])
        return
    
    # Create output directory
    output_dir = Path("compare_result")
    output_dir.mkdir(exist_ok=True, parents=True)
    
    results = []
    
    for d in selected:
        res = load_and_evaluate(d, device)
        if res:
            results.append(res)
            print(f"Done: {d.name} | MAE={res['mae']:.4f} | Lag={res['lag_ms']:.1f}ms")
            
    if not results:
        return

    # Create Summary DataFrame
    df = pd.DataFrame(results)
    
    # Sort by MAE for better visualization
    df = df.sort_values("mae", ascending=True)
    
    print("\n=== Summary Metrics ===")
    print(df[["name", "mae", "rmse", "lag_ms", "smoothness"]])
    
    # Dynamic Figure Height based on number of bars
    num_bars = len(df)
    fig_h = max(6, num_bars * 0.3)
    
    # Plotting
    # 1. Bar Chart: MAE Comparison
    plt.figure(figsize=(12, fig_h))
    sns.barplot(data=df, x="mae", y="name", palette="viridis")
    plt.title("Model Comparison - MAE (Lower is Better)")
    plt.xlabel("MAE (m/s)")
    plt.tight_layout()
    plt.savefig(output_dir / "comparison_mae.png")
    print(f"\nSaved {output_dir / 'comparison_mae.png'}")
    plt.close() # Close to free memory
    
    # 2. Bar Chart: Lag Comparison
    plt.figure(figsize=(12, fig_h))
    # Sort by Lag for this plot? Or keep MAE order? User said "Sort by performance". 
    # Usually consistent order is nice, but specific sorting is better.
    # Let's sort by Lag for the Lag plot.
    df_lag = df.sort_values("lag_ms", ascending=True)
    sns.barplot(data=df_lag, x="lag_ms", y="name", palette="coolwarm")
    plt.title("Model Comparison - Lag (Closer to 0 is Better)")
    plt.xlabel("Lag (ms) [Positive = Prediction Delayed]")
    plt.tight_layout()
    plt.savefig(output_dir / "comparison_lag.png")
    print(f"Saved {output_dir / 'comparison_lag.png'}")
    plt.close()

    # -------------------------------------------------------------------------
    # [NEW] Multi-Model Trajectory Comparison Plot
    # -------------------------------------------------------------------------
    print("\n[INFO] Generating Multi-Model Trajectory Comparison...")
    
    # Store predictions for all models
    model_predictions = {} 
    gt_signal = None
    
    # Select color palette
    colors = sns.color_palette("bright", len(df))
    # Map names to colors
    model_colors = {}
    for i, name in enumerate(df['name'].unique()):
        model_colors[name] = colors[i]
    
    # Iterate all rows to gather predictions
    for idx, row in df.iterrows():
        name = row['name']
        path = row['folder_path']
        print(f"  Loading {name}...")
        
        try:
            # Re-run with return_seqs=True
            # Note: load_and_evaluate is defined outside main scope but available
            res = load_and_evaluate(path, device, return_seqs=True)
            
            y_true = res['y_true_seq'] 
            y_pred = res['y_pred_seq'] # (N, H, 1)

            if y_pred.ndim == 2:
                y_pred = y_pred[:, None, :]
            elif y_pred.ndim == 1:
                y_pred = y_pred[:, None, None]

            # Alignment check: GT should be identical. Take first one.
            if gt_signal is None:
                gt_signal = y_true[:, 0, 0] # Continuous GT
                
            # Use Horizon=1 (index 0) for the main comparison line
            preds = y_pred[:, 0, 0]
            
            model_predictions[name] = preds
            
        except Exception as e:
            print(f"  Failed to load {name}: {e}")

    # Plotting Loop
    if gt_signal is not None:
        durations = [60, 10]
        
        for dur in durations:
            samples = dur * 100
            if samples > len(gt_signal): samples = len(gt_signal)
            
            plt.figure(figsize=(15, 6))
            time_axis = np.arange(samples) * 0.01
            
            # Plot GT (Black)
            plt.plot(time_axis, gt_signal[:samples], 'k-', linewidth=2.5, label="Ground Truth", alpha=0.7)
            
            # Plot Models
            for name, preds in model_predictions.items():
                val_len = min(len(preds), samples)
                if val_len <= 0: continue

                p_seq = preds[:val_len]
                t_seq = time_axis[:val_len]
                
                c = model_colors.get(name, 'r')
                width = 2.5 if "full" in name and "reference" not in name else 1.5
                style = '-' 
                
                plt.plot(t_seq, p_seq, style, color=c, linewidth=width, label=name, alpha=0.9)
                
            plt.title(f"Multi-Model Speed Estimation Comparison ({dur}s)")
            plt.xlabel("Time (s)")
            plt.ylabel("Speed (m/s)")
            plt.legend()
            plt.grid(True, alpha=0.3)
            plt.tight_layout()
            
            fname = output_dir / f"compare_models_{dur}s.png"
            plt.savefig(fname)
            print(f"Saved {fname}")
            plt.close()

    print("Multi-model comparison complete.")

def plot_training_curves(exp_dir, save_dir):
    """
    Plots Train vs Val loss from train_log.csv to detect overfitting.
    """
    log_path = exp_dir / "train_log.csv"
    if not log_path.exists():
        print(f"[WARN] No train_log.csv found in {exp_dir}")
        return

    try:
        df = pd.read_csv(log_path)
        plt.figure(figsize=(10, 5))
        plt.plot(df['epoch'], df['train_loss'], label='Train Loss', linewidth=2)
        plt.plot(df['epoch'], df['val_loss'], label='Val Loss', linewidth=2, linestyle='--')
        plt.title(f"Training Dynamics (Overfitting Check) - {exp_dir.name}")
        plt.xlabel("Epoch")
        plt.ylabel("Loss")
        plt.legend()
        plt.grid(True, alpha=0.3)
        plt.yscale('log') # Log scale often helps see divergence
        
        save_path = save_dir / "training_curves.png"
        plt.savefig(save_path)
        plt.close()
        print(f"Saved {save_path}")
    except Exception as e:
        print(f"[ERROR] Failed to plot training curves: {e}")

def plot_trajectory(y_true, y_pred, title, save_path, series_offsets=None):
    """
    Plots trajectory comparison with Full and Zoomed views.
    y_true, y_pred: (T, 1) or (T,) or (T)
    series_offsets: List of start indices for each trial (to insert breaks)
    """
    # Helper to insert NaNs at boundaries for plotting
    def insert_breaks(data, offsets):
        if offsets is None or len(offsets) < 2: return data, np.arange(len(data))/100.0
        
        # We need to construct a new array with NaNs inserted between segments
        # offsets: [0, 100, 200...] (Start indices of segments in original data)
        # We want to insert NaN at 100, 200...
        
        segments = []
        total_len = len(data)
        
        for i in range(len(offsets)):
            start = offsets[i]
            end = offsets[i+1] if i < len(offsets)-1 else total_len
            if start >= end: continue
            segments.append(data[start:end])
            if i < len(offsets)-1:
                segments.append(np.array([np.nan]))
                
        if not segments: return data, np.arange(len(data))/100.0
        
        new_data = np.concatenate(segments)
        new_t = np.arange(len(new_data)) / 100.0 # Time is now artifactual/expanded but shows gaps ?? 
        # Actually time should probably just be broken too? 
        # Or we just use a single time axis and let data have NaNs.
        # If we insert NaNs, the index shifts.
        # Simpler: Plot each segment separately?
        # NO, single plot call is better for performance.
        # Just use the NaN approach.
        return new_data, new_t

    # Wait, if we change the time axis length, it might be confusing.
    # But usually "concatenated trials" is an artificial timeline anyway.
    # So inserting a small gap (NaN) is fine.
    
    # Ensure 1D
    y_true = np.squeeze(y_true)
    y_pred = np.squeeze(y_pred)
    # If still not 1D (e.g. sequence output), take the first index
    if y_true.ndim > 1: y_true = y_true[:, 0]
    if y_pred.ndim > 1: y_pred = y_pred[:, 0]
    
    # Insert Breaks
    y_true_plot, t_plot = insert_breaks(y_true, series_offsets)
    y_pred_plot, _ = insert_breaks(y_pred, series_offsets) # t_plot should be same
    
    fig, axes = plt.subplots(2, 1, figsize=(12, 10))
    
    # 1. Full Duration
    axes[0].plot(t_plot, y_true_plot, label='Ground Truth (Target)', alpha=0.7, color='black')
    axes[0].plot(t_plot, y_pred_plot, label='Prediction', alpha=0.9, color='red', linestyle='--')
    
    # Add trial boundaries
    fs = 100.0
    if series_offsets is not None:
        for offset in series_offsets:
            if offset == 0: continue
            boundary_t = offset / fs
            axes[0].axvline(x=boundary_t, color='blue', linestyle=':', alpha=0.5)
            if offset == series_offsets[1] if len(series_offsets) > 1 else -1: # Only label first boundary to avoid clutter
                axes[0].text(boundary_t, axes[0].get_ylim()[1], 'Trial Boundary', rotation=90, color='blue', alpha=0.5, va='top')

    axes[0].set_title(f"{title} - Full Duration")
    axes[0].set_xlabel("Time (s)")
    axes[0].set_ylabel("Speed (m/s)")
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)
    
    # 2. Zoomed (15s Window)
    start_t, end_t = 10.0, 25.0
    # Adjust if total duration is short
    if t_plot[-1] < 15.0:
        start_t, end_t = 0, t_plot[-1]
    elif t_plot[-1] < 25.0:
        start_t = 0
        end_t = 15.0
        
    mask = (t_plot >= start_t) & (t_plot <= end_t)
    if np.sum(mask) > 0:
        axes[1].plot(t_plot[mask], y_true_plot[mask], label='Ground Truth', color='black', alpha=0.7)
        axes[1].plot(t_plot[mask], y_pred_plot[mask], label='Prediction', color='red', linestyle='--', alpha=0.9)
        axes[1].set_title(f"Zoomed ({start_t}s - {end_t}s)")
        axes[1].set_xlabel("Time (s)")
        axes[1].set_ylabel("Speed (m/s)")
        axes[1].legend()
        axes[1].grid(True, alpha=0.3)
    else:
        axes[1].text(0.5, 0.5, "Data too short for zoom", ha='center')
        
    plt.tight_layout()
    plt.savefig(save_path)
    plt.close()


def plot_detailed_condition_trajectories(exp_path, save_dir, device,
                                         preloaded_model=None, preloaded_cfg=None,
                                         preloaded_c_in_vars=None, preloaded_c_out_vars=None,
                                         preloaded_scaler=None):
    """
    Generates 2x3 grid plots (Levels x Zooms) for each condition.
    Copied/Refactored from analyze_detailed_single to be reusable.
    If preloaded_model/cfg are provided, skips redundant config parsing and model loading.
    """
    print(f"[INFO] Generating Detailed Grid Plots for {exp_path.name}...")
    import yaml, re, h5py
    from torch.utils.data import DataLoader

    folder_name = exp_path.name

    # ---- Use preloaded data if available, otherwise load from scratch ----
    if preloaded_cfg is not None and preloaded_model is not None:
        cfg = preloaded_cfg
        c_in_vars = preloaded_c_in_vars
        c_out_vars = preloaded_c_out_vars
        model = preloaded_model
        mean, scale = (preloaded_scaler if preloaded_scaler else (None, None))
        _skip_model_load = True
    else:
        _skip_model_load = False

        # 1. Load Config
        exp_name_match = re.match(r"^(.+?)_Test-", folder_name)
        exp_name = exp_name_match.group(1) if exp_name_match else "baseline"
        original_config_path = Path(f"configs/{exp_name}.yaml")
        cfg = None
        if original_config_path.exists():
            with open(original_config_path, 'r') as f_cfg: cfg = yaml.safe_load(f_cfg)
        else:
            config_path = exp_path / "config.yaml"
            if not config_path.exists():
                print("  -> Config not found, skipping detailed plots.")
                return
            with open(config_path) as f_cfg: cfg = yaml.safe_load(f_cfg)

        # 2. Parse Variables
        def _parse_vars_local(var_list_from_yaml):
            if not var_list_from_yaml: return []
            parsed = []
            for item in var_list_from_yaml:
                if isinstance(item, (list, tuple)) and len(item) == 2:
                    parsed.append((item[0], item[1]))
            return parsed

        curr_input_vars = cfg.get("input_vars")
        curr_output_vars = cfg.get("output_vars")
        if not curr_input_vars and 'shared' in cfg: curr_input_vars = cfg['shared'].get('input_vars')
        if not curr_output_vars and 'shared' in cfg: curr_output_vars = cfg['shared'].get('output_vars')
        if not curr_input_vars and '01_construction' in cfg: curr_input_vars = cfg['01_construction'].get('inputs')
        if not curr_output_vars and '01_construction' in cfg: curr_output_vars = cfg['01_construction'].get('outputs')

        c_in_vars = _parse_vars_local(curr_input_vars)
        c_out_vars = _parse_vars_local(curr_output_vars)

        mean, scale = None, None

    # 3. Data Parameters (always needed for per-condition loading)
    train_cfg_section = cfg.get("02_train", {})
    data_cfg = train_cfg_section.get("data", {})
    if not data_cfg: data_cfg = cfg.get("data", {})

    window_size = data_cfg.get("window_size") or cfg.get("window_size", 200)
    window_output = data_cfg.get("window_output") or data_cfg.get("time_window_output") or cfg.get("window_output", 1)
    est_tick_ranges = data_cfg.get("est_tick_ranges") if data_cfg else cfg.get("est_tick_ranges")
    target_transform = data_cfg.get("target_transform")

    # LPF
    curr_lpf_cutoff = cfg.get("lpf_cutoff")
    if curr_lpf_cutoff is None and 'shared' in cfg: curr_lpf_cutoff = cfg['shared'].get('lpf_cutoff')
    eval_lpf = curr_lpf_cutoff or 0.5

    curr_lpf_order = cfg.get("lpf_order")
    if curr_lpf_order is None and 'shared' in cfg: curr_lpf_order = cfg['shared'].get('lpf_order')
    curr_lpf_order = curr_lpf_order or 4

    # 4. Identify Subject and Data Path
    match = re.search(r"Test-((?:m\d+_)?S\d+)", folder_name)
    test_sub = match.group(1) if match else "S001"

    prefix_found, real_sub = parse_subject_prefix(test_sub)
    data_sources = get_data_sources_from_config(cfg)
    if prefix_found and prefix_found in data_sources:
        data_path = data_sources[prefix_found]['path']
    else:
        data_path = cfg.get("data_path", "combined_data_S008.h5")
        if not os.path.exists(data_path): data_path = "combined_data_S008.h5"

    if not os.path.exists(data_path):
        print(f"  -> Data path {data_path} not found. Skipping.")
        return

    if not _skip_model_load:
        # 5. Load Model
        model_path = exp_path / "model.pt"
        if not model_path.exists(): return

        input_dim = 0
        count = 0
        for g, vs in c_in_vars: count += len(vs)
        use_gait_ph = cfg.get("02_train", {}).get("data", {}).get("use_gait_phase") or cfg.get("use_gait_phase")
        if use_gait_ph: count += 2
        input_dim = count

        out_dim = 0
        for g, vs in c_out_vars: out_dim += len(vs)
        if out_dim == 0: out_dim = 1

        model_cfg_d = train_cfg_section.get("model")
        if not model_cfg_d: model_cfg_d = data_cfg if "channels" in data_cfg else cfg.get("model", {})

        tcn_channels = model_cfg_d.get("channels") or cfg.get("tcn_channels", (64, 64, 128))
        kernel_size = model_cfg_d.get("kernel_size") or cfg.get("kernel_size", 3)
        dropout_p = float(model_cfg_d.get("dropout") or cfg.get("dropout_p", 0.5))
        head_dropout = model_cfg_d.get("head_dropout", dropout_p)
        mlp_hidden = model_cfg_d.get("head_hidden") or cfg.get("mlp_hidden", 128)
        use_input_norm = cfg.get("data", {}).get("normalize", True)
        model_norm_type = model_cfg_d.get("model_norm", None)

        horizon = len(est_tick_ranges) if est_tick_ranges else window_output

        model_type = model_cfg_d.get("type", "TCN")
        common_args = dict(input_dim=input_dim, output_dim=out_dim, horizon=horizon, channels=tcn_channels, kernel_size=kernel_size, dropout=dropout_p, head_dropout=head_dropout, mlp_hidden=mlp_hidden, use_input_norm=use_input_norm, tcn_norm=model_norm_type, mlp_norm=model_norm_type)

        if model_type == "StanceGatedTCN":
            model = StanceGatedTCN(**common_args, gating_dim=model_cfg_d.get("gating_dim", 1))
        elif model_type == "AttentionTCN":
            model = AttentionTCN(**common_args, attention_type=model_cfg_d.get("attention_type", "temporal"), attention_heads=model_cfg_d.get("attention_heads", 4))
        elif model_type == "TCN_GRU":
            model = TCN_GRU_Head(**common_args, gru_hidden=model_cfg_d.get("gru_hidden", 32))
        elif model_type == "DualBranchTCN_GRU":
            model = DualBranchTCN_GRU(
                **common_args,
                gru_hidden=model_cfg_d.get("gru_hidden", 32),
                branch_input_dims=model_cfg_d.get("branch_input_dims"),
                branch_channels=model_cfg_d.get("branch_channels"),
            )
        elif model_type == "MultiBranchTCN_GRU":
            model = MultiBranchTCN_GRU(
                **common_args,
                gru_hidden=model_cfg_d.get("gru_hidden", 32),
                branch_input_dims=model_cfg_d.get("branch_input_dims"),
                branch_channels=model_cfg_d.get("branch_channels"),
            )
        elif model_type == "FiLMConditionedTCN_GRU":
            model = FiLMConditionedTCN_GRU(
                **common_args,
                gru_hidden=model_cfg_d.get("gru_hidden", 32),
                branch_input_dims=model_cfg_d.get("branch_input_dims"),
                branch_channels=model_cfg_d.get("branch_channels"),
                film_scale=model_cfg_d.get("film_scale", 0.5),
            )
        elif model_type == "BoundedResidualTCN_GRU":
            model = BoundedResidualTCN_GRU(
                **common_args,
                gru_hidden=model_cfg_d.get("gru_hidden", 32),
                branch_input_dims=model_cfg_d.get("branch_input_dims"),
                branch_channels=model_cfg_d.get("branch_channels"),
                residual_scale=model_cfg_d.get("residual_scale", 0.30),
            )
        elif model_type == "AffineCalibTCN_GRU":
            model = AffineCalibTCN_GRU(
                **common_args,
                gru_hidden=model_cfg_d.get("gru_hidden", 32),
                branch_input_dims=model_cfg_d.get("branch_input_dims"),
                branch_channels=model_cfg_d.get("branch_channels"),
                alpha_scale=model_cfg_d.get("alpha_scale", 0.30),
                beta_scale=model_cfg_d.get("beta_scale", 0.25),
            )
        elif model_type == "PositiveResidualTCN_GRU":
            model = PositiveResidualTCN_GRU(
                **common_args,
                gru_hidden=model_cfg_d.get("gru_hidden", 32),
                branch_input_dims=model_cfg_d.get("branch_input_dims"),
                branch_channels=model_cfg_d.get("branch_channels"),
                residual_scale=model_cfg_d.get("residual_scale", 0.25),
                gate_scale=model_cfg_d.get("gate_scale", 1.0),
            )
        elif model_type == "AssistExpertTCN_GRU":
            model = AssistExpertTCN_GRU(
                **common_args,
                gru_hidden=model_cfg_d.get("gru_hidden", 32),
                branch_input_dims=model_cfg_d.get("branch_input_dims"),
                branch_channels=model_cfg_d.get("branch_channels"),
                gate_max=model_cfg_d.get("gate_max", 0.6),
            )
        elif model_type == "SlowFastCalibTCN_GRU":
            model = SlowFastCalibTCN_GRU(
                **common_args,
                gru_hidden=model_cfg_d.get("gru_hidden", 32),
                branch_input_dims=model_cfg_d.get("branch_input_dims"),
                branch_channels=model_cfg_d.get("branch_channels"),
                bias_scale=model_cfg_d.get("bias_scale", 0.20),
                residual_scale=model_cfg_d.get("residual_scale", 0.12),
            )
        elif model_type == "StructuredSlowFastTCN_GRU":
            model = StructuredSlowFastTCN_GRU(
                **common_args,
                gru_hidden=model_cfg_d.get("gru_hidden", 32),
                branch_input_dims=model_cfg_d.get("branch_input_dims"),
                branch_channels=model_cfg_d.get("branch_channels"),
                bias_scale=model_cfg_d.get("bias_scale", 0.18),
                residual_scale=model_cfg_d.get("residual_scale", 0.10),
                use_aux_head=model_cfg_d.get("use_aux_head", False),
                aux_dim=model_cfg_d.get("aux_dim", 1),
                use_subject_calibration=model_cfg_d.get("use_subject_calibration", False),
                subject_ctx_dims=model_cfg_d.get("subject_ctx_dims", 0),
                alpha_scale=model_cfg_d.get("alpha_scale", 0.08),
                beta_scale=model_cfg_d.get("beta_scale", 0.05),
            )
        elif model_type == "TCN_GRU_MDN":
            model = TCN_GRU_MDN(**common_args, gru_hidden=model_cfg_d.get("gru_hidden", 32), num_mixtures=int(model_cfg_d.get("num_mixtures", 5)))
        else:
            model = TCN_MLP(**common_args)

        ckpt = torch.load(model_path, map_location=device)
        model.load_state_dict(ckpt['state_dict'] if 'state_dict' in ckpt else ckpt, strict=False)
        model.to(device)
        model.eval()

        # Scaler
        scaler_path = exp_path / "scaler.npz"
        if scaler_path.exists():
            s_data = np.load(scaler_path)
            mean = s_data['mean']
            scale = s_data['scale']

    # 6. Plot Loop
    from SpeedEstimator_TCN_MLP_experiments import extract_condition_data_v2
    
    # Conditions
    cond_cfg = cfg.get("conditions")
    if not cond_cfg and 'shared' in cfg: cond_cfg = cfg['shared'].get("conditions")
    if not cond_cfg: cond_cfg = []
    
    # Expand conditions using H5
    actual_conds = []
    with h5py.File(data_path, 'r') as f_check:
        if real_sub not in f_check: return
        avail = list(f_check[real_sub].keys())
        for c in cond_cfg:
            if c in avail: actual_conds.append(c)
            elif c == 'level': actual_conds.extend([ac for ac in avail if ac.startswith('level_')])
            else:
                matches = [ac for ac in avail if ac.startswith(c)]
                if matches: actual_conds.extend(matches)
    actual_conds = list(dict.fromkeys(actual_conds))

    with h5py.File(data_path, 'r') as f:
        for cond in actual_conds:
            if cond not in f[real_sub]: continue
            
            # Target Levels: lv0, lv4, lv7 (First 3 found)
            available_lv = sorted(list(f[real_sub][cond].keys()))
            target_lvs = available_lv[:3] # Max 3 columns
            
            if not target_lvs: continue
            
            # Figure width depends on columns. Keep Full + 2 zoom rows only.
            fig, axes = plt.subplots(3, len(target_lvs), figsize=(6*len(target_lvs), 12), squeeze=False)
            has_data = False
            
            for col_idx, lv_name in enumerate(target_lvs):
                try:
                    # Force Single Trial extract
                    X_list, Y_list = extract_condition_data_v2(
                        f, real_sub, cond, c_in_vars, c_out_vars, 
                        lpf_cutoff=eval_lpf, lpf_order=curr_lpf_order, fs=100,
                        include_levels=[lv_name], include_trials=['trial_01'] # First trial only
                    )
                except Exception as e:
                    X_list = []
                    
                if not X_list:
                    for r in range(3): axes[r, col_idx].axis('off')
                    continue
                
                has_data = True
                X_arr = X_list[0]
                Y_arr = Y_list[0]
                
                if mean is not None:
                    try:
                        X_arr = (X_arr - mean) / (scale + 1e-8)
                    except ValueError: pass
                
                # Inference
                ds_vis = LazyWindowDataset(
                    [X_arr], [Y_arr], window_size, window_output, 1,
                    target_mode="sequence",
                    est_tick_ranges=est_tick_ranges,
                    target_transform=target_transform,
                    input_mean=mean,
                    input_std=scale,
                )
                loader_vis = DataLoader(ds_vis, batch_size=_DETAIL_BATCH_SIZE, shuffle=False)
                
                preds, targets = [], []
                with torch.no_grad():
                     for xb, yb in loader_vis:
                         xb = xb.to(device)
                         out = extract_primary_prediction(model(xb))
                         preds.append(out.cpu().numpy())
                         targets.append(yb.cpu().numpy())
                
                if not preds: continue
                P_raw = np.concatenate(preds, axis=0)
                T_raw = np.concatenate(targets, axis=0)
                
                # Apply same post-processing logic (Averaging / LPF)
                p_seq_full, t_seq_full, _ = apply_post_processing(
                    P_raw, T_raw, [X_arr], [Y_arr], cfg,
                    window_size, window_output, 1, est_tick_ranges
                )

                if target_transform:
                    x_windows = []
                    for xb, _ in loader_vis:
                        x_windows.append(xb.cpu().numpy())
                    if x_windows:
                        x_ref = np.concatenate(x_windows, axis=0)
                        p_seq_full = inverse_target_transform(
                            p_seq_full, x_ref,
                            target_transform=target_transform,
                            input_mean=mean,
                            input_std=scale,
                        )
                        t_seq_full = inverse_target_transform(
                            t_seq_full, x_ref,
                            target_transform=target_transform,
                            input_mean=mean,
                            input_std=scale,
                        )
                
                p_seq = p_seq_full.squeeze()
                t_seq = t_seq_full.squeeze()
                if p_seq.ndim > 1: p_seq = p_seq[:, 0]
                if t_seq.ndim > 1: t_seq = t_seq[:, 0]
                
                fs_Hz = 100.0
                t = np.arange(len(t_seq)) / fs_Hz
                
                # Row 0: Full
                ax0 = axes[0, col_idx]
                ax0.plot(t, t_seq, color='black', alpha=0.7, label='GT')
                ax0.plot(t, p_seq, color='red', alpha=0.9, linestyle='--', label='Pred')
                ax0.set_title(f"{lv_name} (Full)")
                ax0.grid(True, alpha=0.3)
                if col_idx == 0: ax0.legend()
                
                # Rows 1-2: Zooms
                windows = [(0, 15), (15, 30)]
                for r, (start_t, end_t) in enumerate(windows, start=1):
                    ax = axes[r, col_idx]
                    real_end = min(end_t, t[-1])
                    if start_t >= t[-1]:
                        ax.text(0.5, 0.5, "Out of Range", ha='center')
                        continue
                    
                    mask = (t >= start_t) & (t <= real_end)
                    if np.sum(mask) > 10:
                        ax.plot(t[mask], t_seq[mask], color='black', alpha=0.7)
                        ax.plot(t[mask], p_seq[mask], color='red', alpha=0.9, linestyle='--')
                        ax.set_title(f"Zoom ({start_t}-{end_t}s)")
                        ax.grid(True, alpha=0.3)
                    else:
                        ax.text(0.5, 0.5, "Too Short", ha='center')

            if has_data:
                plt.tight_layout()
                safe_cond = cond.replace('/', '_')
                fname = save_dir / f"{test_sub}_{safe_cond}.png"
                plt.savefig(fname)
                print(f"  [SAVED] Detail Plot: {fname}")
            plt.close()


def _run_trial_prediction(model, X_arr, Y_arr, cfg, window_size, window_output,
                          est_tick_ranges, mean, scale, target_transform):
    X_eval = X_arr
    if mean is not None:
        try:
            X_eval = (X_eval - mean) / (scale + 1e-8)
        except ValueError:
            pass

    ds_vis = LazyWindowDataset(
        [X_eval], [Y_arr], window_size, window_output, 1,
        target_mode="sequence",
        est_tick_ranges=est_tick_ranges,
        target_transform=target_transform,
        input_mean=mean,
        input_std=scale,
    )
    loader_vis = DataLoader(ds_vis, batch_size=_DETAIL_BATCH_SIZE, shuffle=False)

    preds, targets = [], []
    with torch.no_grad():
        for xb, yb in loader_vis:
            xb = xb.to(device)
            out = extract_primary_prediction(model(xb))
            preds.append(out.cpu().numpy())
            targets.append(yb.cpu().numpy())

    if not preds:
        return None, None

    p_raw = np.concatenate(preds, axis=0)
    t_raw = np.concatenate(targets, axis=0)
    p_seq_full, t_seq_full, _ = apply_post_processing(
        p_raw, t_raw, [X_eval], [Y_arr], cfg,
        window_size, window_output, 1, est_tick_ranges
    )

    if target_transform:
        x_windows = []
        for xb, _ in loader_vis:
            x_windows.append(xb.cpu().numpy())
        if x_windows:
            x_ref = np.concatenate(x_windows, axis=0)
            p_seq_full = inverse_target_transform(
                p_seq_full, x_ref,
                target_transform=target_transform,
                input_mean=mean,
                input_std=scale,
            )
            t_seq_full = inverse_target_transform(
                t_seq_full, x_ref,
                target_transform=target_transform,
                input_mean=mean,
                input_std=scale,
            )

    p_seq = p_seq_full.squeeze()
    t_seq = t_seq_full.squeeze()
    if p_seq.ndim > 1:
        p_seq = p_seq[:, 0]
    if t_seq.ndim > 1:
        t_seq = t_seq[:, 0]
    return t_seq, p_seq


def _compute_regime_stats(y_true, y_pred, dy_threshold=0.003):
    err = y_pred - y_true
    dy = np.zeros_like(y_true)
    if len(y_true) > 1:
        dy[1:] = np.abs(np.diff(y_true))
    onset = np.zeros_like(y_true, dtype=bool)
    decay = np.zeros_like(y_true, dtype=bool)
    if len(y_true) > 1:
        diff = np.diff(y_true)
        onset[1:] = diff > dy_threshold
        decay[1:] = diff < -dy_threshold
    steady = dy <= dy_threshold

    def _masked_mean(mask):
        if not np.any(mask):
            return np.nan
        return float(np.mean(err[mask]))

    def _masked_mae(mask):
        if not np.any(mask):
            return np.nan
        return float(np.mean(np.abs(err[mask])))

    return {
        "mae": float(np.mean(np.abs(err))),
        "rmse": float(np.sqrt(np.mean(err ** 2))),
        "signed_bias": float(np.mean(err)),
        "std_error": float(np.std(err)),
        "steady_plateau_bias": _masked_mean(steady),
        "steady_plateau_mae": _masked_mae(steady),
        "onset_error": _masked_mean(onset),
        "decay_error": _masked_mean(decay),
        "trough_error_p05": float(np.percentile(err, 5)),
    }


def save_regime_metrics_for_experiment(exp_path, output_dir, device,
                                       preloaded_model=None, preloaded_cfg=None,
                                       preloaded_c_in_vars=None, preloaded_c_out_vars=None,
                                       preloaded_scaler=None):
    import h5py
    import re

    if preloaded_model is None or preloaded_cfg is None:
        return

    folder_name = exp_path.name
    cfg = preloaded_cfg
    c_in_vars = preloaded_c_in_vars
    c_out_vars = preloaded_c_out_vars
    model = preloaded_model
    mean, scale = (preloaded_scaler if preloaded_scaler else (None, None))

    train_cfg_section = cfg.get("02_train", {})
    data_cfg = train_cfg_section.get("data", {}) or cfg.get("data", {})
    window_size = data_cfg.get("window_size") or cfg.get("window_size", 200)
    window_output = data_cfg.get("window_output") or data_cfg.get("time_window_output") or cfg.get("window_output", 1)
    est_tick_ranges = data_cfg.get("est_tick_ranges") if data_cfg else cfg.get("est_tick_ranges")
    target_transform = data_cfg.get("target_transform")
    curr_lpf_cutoff = cfg.get("lpf_cutoff")
    if curr_lpf_cutoff is None and 'shared' in cfg:
        curr_lpf_cutoff = cfg['shared'].get('lpf_cutoff')
    eval_lpf = curr_lpf_cutoff or 0.5
    curr_lpf_order = cfg.get("lpf_order")
    if curr_lpf_order is None and 'shared' in cfg:
        curr_lpf_order = cfg['shared'].get('lpf_order')
    curr_lpf_order = curr_lpf_order or 4

    match = re.search(r"Test-((?:m\\d+_)?S\\d+)", folder_name)
    test_sub = match.group(1) if match else "S001"
    prefix_found, real_sub = parse_subject_prefix(test_sub)
    data_sources = get_data_sources_from_config(cfg)
    if prefix_found and prefix_found in data_sources:
        data_path = data_sources[prefix_found]['path']
    else:
        data_path = cfg.get("data_path", "combined_data_S008.h5")
        if not os.path.exists(data_path):
            data_path = "combined_data_S008.h5"

    cond_cfg = cfg.get("conditions")
    if not cond_cfg and 'shared' in cfg:
        cond_cfg = cfg['shared'].get("conditions")
    cond_cfg = cond_cfg or []

    rows = []
    with h5py.File(data_path, 'r') as f:
        if real_sub not in f:
            return
        avail = list(f[real_sub].keys())
        actual_conds = []
        for c in cond_cfg:
            if c in avail:
                actual_conds.append(c)
            elif c == 'level':
                actual_conds.extend([ac for ac in avail if ac.startswith('level_')])
            else:
                matches = [ac for ac in avail if ac.startswith(c)]
                if matches:
                    actual_conds.extend(matches)
        actual_conds = list(dict.fromkeys(actual_conds))

        for cond in actual_conds:
            for lv_name in sorted(list(f[real_sub][cond].keys())):
                X_list, Y_list, meta_list = extract_condition_data_v2(
                    f, real_sub, cond, c_in_vars, c_out_vars,
                    lpf_cutoff=eval_lpf, lpf_order=curr_lpf_order, fs=100,
                    include_levels=[lv_name], include_trials=None,
                    return_metadata=True,
                )
                for X_arr, Y_arr, meta in zip(X_list, Y_list, meta_list):
                    t_seq, p_seq = _run_trial_prediction(
                        model, X_arr, Y_arr, cfg, window_size, window_output,
                        est_tick_ranges, mean, scale, target_transform
                    )
                    if t_seq is None:
                        continue
                    rows.append({
                        "model": folder_name,
                        "subject": test_sub,
                        "condition": cond,
                        "level": lv_name,
                        "trial": meta.get("trial"),
                        **_compute_regime_stats(t_seq, p_seq),
                    })

    if not rows:
        return

    detail_df = pd.DataFrame(rows)
    detail_df.to_csv(output_dir / "regime_metrics_detailed.csv", index=False)

    summary_df = detail_df.groupby(["condition", "level"], as_index=False).agg({
        "mae": "mean",
        "rmse": "mean",
        "signed_bias": "mean",
        "std_error": "mean",
        "steady_plateau_bias": "mean",
        "steady_plateau_mae": "mean",
        "onset_error": "mean",
        "decay_error": "mean",
        "trough_error_p05": "mean",
    })
    summary_df.to_csv(output_dir / "condition_level_summary.csv", index=False)

    assist_df = detail_df.groupby(["level"], as_index=False).agg({
        "signed_bias": "mean",
        "steady_plateau_bias": "mean",
        "mae": "mean",
        "rmse": "mean",
    })
    assist_df.to_csv(output_dir / "assist_bucket_summary.csv", index=False)

def render_full_outputs_for_result(exp_path, fold_output_dir, res, skip_feature_importance=False):
    fold_name = exp_path.name
    exp_id = get_experiment_group_name(exp_path)

    plot_training_curves(exp_path, fold_output_dir)

    if "y_true_seq" in res and "y_pred_seq" in res:
        traj_plot_path = fold_output_dir / "trajectory.png"
        plot_trajectory(
            res["y_true_seq"],
            res["y_pred_seq"],
            f"Trajectory: {exp_id} ({fold_name})",
            traj_plot_path,
            series_offsets=res.get("series_offsets")
        )
        print(f"  [SAVED] Trajectory plot: {traj_plot_path}")

    scaler = None
    if "scaler_mean" in res:
        scaler = (res["scaler_mean"], res["scaler_scale"])
    plot_detailed_condition_trajectories(
        exp_path, fold_output_dir, device,
        preloaded_model=res.get("model"),
        preloaded_cfg=res.get("cfg"),
        preloaded_c_in_vars=res.get("c_in_vars"),
        preloaded_c_out_vars=res.get("c_out_vars"),
        preloaded_scaler=scaler
    )
    save_regime_metrics_for_experiment(
        exp_path, fold_output_dir, device,
        preloaded_model=res.get("model"),
        preloaded_cfg=res.get("cfg"),
        preloaded_c_in_vars=res.get("c_in_vars"),
        preloaded_c_out_vars=res.get("c_out_vars"),
        preloaded_scaler=scaler,
    )

    if skip_feature_importance:
        print(f"  -> Skipping Feature Importance for {fold_name} (--skip-feature-importance).")
        return
    if (fold_output_dir / "feature_importance.png").exists():
        print(f"  -> Feature Importance already exists for {fold_name}. Skipping.")
        return

    print(f"  -> Calculating Feature Importance for {fold_name}...")
    _fi_model = res.get("model")
    _fi_cfg = res.get("cfg")
    _fi_c_in_vars = res.get("c_in_vars")

    if not (_fi_model and _fi_cfg and _fi_c_in_vars):
        return

    raw_input_vars = _fi_cfg.get("01_construction", {}).get("inputs") or _fi_cfg.get("shared", {}).get("input_vars")
    if not raw_input_vars:
        raw_input_vars = []
    input_vars = parse_vars(raw_input_vars)
    f_names = get_feature_names(_fi_cfg, input_vars)

    sub_name = res.get("test_sub", "")
    prefix_found, real_sub = parse_subject_prefix(sub_name)
    data_sources = get_data_sources_from_config(_fi_cfg)
    if prefix_found and prefix_found in data_sources:
        data_path = data_sources[prefix_found]['path']
    else:
        data_path = _fi_cfg.get("data_path", "combined_data_S008.h5")
        if not os.path.exists(data_path):
            data_path = "combined_data_S008.h5"

    import h5py
    actual_conds = []
    with h5py.File(data_path, 'r') as f_check:
        if real_sub in f_check:
            available_conds = list(f_check[real_sub].keys())
        else:
            available_conds = []
        cond_cfg = _fi_cfg.get("conditions")
        if not cond_cfg and "shared" in _fi_cfg:
            cond_cfg = _fi_cfg["shared"].get("conditions")
        if not cond_cfg:
            cond_cfg = []
        for c in cond_cfg:
            if c in available_conds:
                actual_conds.append(c)
            elif c == 'level':
                actual_conds.extend([ac for ac in available_conds if ac.startswith('level_')])
            else:
                actual_conds.extend([ac for ac in available_conds if ac.startswith(c)])
    actual_conds = list(dict.fromkeys(actual_conds))

    eval_lpf = _fi_cfg.get("shared", {}).get("lpf_cutoff", 0.5)
    curr_lpf_order = _fi_cfg.get("shared", {}).get("lpf_order", 4)

    from SpeedEstimator_TCN_MLP_experiments import extract_condition_data_v2
    X_imp_list, Y_imp_list = [], []
    with h5py.File(data_path, 'r') as f_imp:
        for cond in actual_conds:
            if cond not in f_imp[real_sub]:
                continue
            try:
                x_list, y_list = extract_condition_data_v2(
                    f_imp, real_sub, cond,
                    [(item[0], item[1]) for item in input_vars],
                    [(item[0], item[1]) for item in parse_vars(_fi_cfg.get("shared", {}).get("output_vars"))],
                    lpf_cutoff=eval_lpf, lpf_order=curr_lpf_order, fs=100
                )
                X_imp_list.extend(x_list)
                Y_imp_list.extend(y_list)
            except Exception as e:
                print(f"  [WARN] Skipping {cond}: {e}")

    if not X_imp_list:
        return

    X_test = np.concatenate(X_imp_list, axis=0)
    Y_test = np.concatenate(Y_imp_list, axis=0)

    if "scaler_mean" in res:
        X_test = (X_test - res["scaler_mean"]) / (res["scaler_scale"] + 1e-8)

    _fi_data_cfg = _fi_cfg.get("02_train", {}).get("data", {})
    imp_ds = LazyWindowDataset(
        [X_test], [Y_test],
        _fi_data_cfg.get("window_size", 200),
        len(_fi_data_cfg.get("est_tick_ranges", [5])),
        _fi_data_cfg.get("stride_inf", 1),
        target_mode="sequence",
        est_tick_ranges=_fi_data_cfg.get("est_tick_ranges", [5])
    )
    imp_loader = DataLoader(imp_ds, batch_size=_FI_BATCH_SIZE, shuffle=False, num_workers=0)

    train_cfg = _fi_cfg.get("02_train", {}).get("train", {})
    loss_name = train_cfg.get("loss", "mse").lower()
    huber_delta = float(train_cfg.get("huber_delta", 1.0))
    if loss_name == "huber":
        criterion = torch.nn.HuberLoss(delta=huber_delta, reduction='sum')
    elif loss_name in ["mae", "l1"]:
        criterion = torch.nn.L1Loss(reduction='sum')
    else:
        criterion = torch.nn.MSELoss(reduction='sum')

    imps = calculate_permutation_importance(_fi_model, imp_loader, device=device, criterion=criterion, batch_size=_FI_BATCH_SIZE)
    plot_feature_importance(imps, f_names, fold_output_dir / "feature_importance.png", title=f"Importance ({loss_name}): {fold_name}")

def render_topk_full_plots(target_dir, filter_pattern=None, top_k=3, skip_feature_importance=False):
    if top_k < 1:
        print("[AUTO] full_plot_topk < 1. Skipping top-k rerender.")
        return

    root_path = Path(target_dir if target_dir else "experiments")
    exp_dirs = collect_experiment_dirs(root_path, filter_pattern=filter_pattern)
    if not exp_dirs:
        print("[AUTO] No experiments available for top-k rerender.")
        return

    default_output_root = infer_common_output_root(target_dir, exp_dirs)
    if default_output_root is None:
        default_output_root = Path("compare_result") / root_path.name
    default_output_root.mkdir(exist_ok=True, parents=True)

    summary_df, summary_path = load_merged_fast_eval_summary(default_output_root)
    if summary_df is None or summary_df.empty:
        print(f"[AUTO] No fast-eval summary found under {default_output_root}. Cannot rerender top-k plots.")
        return

    exp_map = {str(exp_path): exp_path for exp_path in exp_dirs}
    top_df = summary_df.sort_values(["mae", "name"], ascending=[True, True]).head(top_k).copy()
    top_selection_path = default_output_root / f"top_{top_k}_full_plot_selection.csv"
    top_df.to_csv(top_selection_path, index=False)
    print(f"[AUTO] Rendering full plots for top {len(top_df)} experiments from {summary_path}")
    print(f"[AUTO] Saved top-k selection to: {top_selection_path}")

    for idx, row in enumerate(top_df.itertuples(index=False), 1):
        exp_path = exp_map.get(str(row.path))
        if exp_path is None:
            candidate = Path(str(row.path))
            if candidate.exists():
                exp_path = candidate
        if exp_path is None:
            print(f"  [WARN] Missing experiment path for {row.name}. Skipping.")
            continue

        output_root = infer_output_root_from_exp_path(exp_path) or default_output_root
        fold_output_dir = output_root / get_experiment_group_name(exp_path) / exp_path.name
        fold_output_dir.mkdir(exist_ok=True, parents=True)
        print(f"\n[TOPK {idx}/{len(top_df)}] Rendering full plots for {exp_path.name}")
        res = load_and_evaluate(exp_path, device, return_seqs=True, return_extras=True)
        if not res:
            continue
        render_full_outputs_for_result(exp_path, fold_output_dir, res, skip_feature_importance=skip_feature_importance)

def analyze_automated(target_dir, filter_pattern=None, num_shards=1, shard_index=0, skip_feature_importance=False, fast_eval=False, full_plot_topk=0, fast_eval_stride=3):

    if not target_dir: target_dir = "experiments"
    root_path = Path(target_dir)
    exp_dirs = collect_experiment_dirs(root_path, filter_pattern=filter_pattern)

    valid_dirs = shard_experiment_dirs(exp_dirs, num_shards=num_shards, shard_index=shard_index)
    
    if not valid_dirs:
        if num_shards > 1:
            print(f"No valid experiments found for shard {shard_index + 1}/{num_shards}.")
        else:
            print("No valid experiments found.")
        return

    if num_shards > 1:
        print(f"[AUTO] Shard {shard_index + 1}/{num_shards}: {len(valid_dirs)} experiments assigned.")
    else:
        print(f"[AUTO] Found {len(valid_dirs)} experiments to analyze.")

    # ==========================================================================
    # SINGLE PASS: Evaluate + generate all plots in one loop (no duplicate calls)
    # ==========================================================================
    import time as _time
    _t0_total = _time.time()

    default_output_root = infer_common_output_root(target_dir, valid_dirs)
    if default_output_root is None:
        default_output_root = Path("compare_result") / root_path.name
    default_output_root.mkdir(exist_ok=True, parents=True)

    results = []
    eval_cache = {}  # name -> full result dict (model, cfg, seqs, etc.)

    for idx, exp_path in enumerate(valid_dirs, 1):
        _t0_item = _time.time()
        fold_name = exp_path.name
        exp_id = get_experiment_group_name(exp_path)
        output_root = infer_output_root_from_exp_path(exp_path) or default_output_root
        fold_output_dir = output_root / exp_id / fold_name
        fold_output_dir.mkdir(exist_ok=True, parents=True)

        print(f"\n[{idx}/{len(valid_dirs)}] Processing: {exp_id} -> {fold_name}")

        try:
            # --- ONE call to load_and_evaluate (replaces two separate calls) ---
            stride_override = fast_eval_stride if fast_eval else None
            res = load_and_evaluate(
                exp_path,
                device,
                return_seqs=True,
                return_extras=True,
                stride_eval_override=stride_override,
            )
            if not res:
                continue

            results.append({
                "name": fold_name,
                "path": exp_path,
                "mae": res["mae"],
                "rmse": res["rmse"],
                "r2": res.get("r2", 0.0),
                "lag_ms": res.get("lag_ms", 0.0),
                "smoothness": res.get("smoothness", 0.0),
            })
            eval_cache[fold_name] = res

            if fast_eval:
                print(f"  -> Fast eval enabled. Saved metrics only for {fold_name}.")
            else:
                render_full_outputs_for_result(
                    exp_path,
                    fold_output_dir,
                    res,
                    skip_feature_importance=skip_feature_importance,
                )

            _elapsed = _time.time() - _t0_item
            print(f"  [DONE] {fold_name} in {_elapsed:.1f}s")

        except Exception as e:
            print(f"  [ERROR] Failed to process {fold_name}: {e}")
            import traceback
            traceback.print_exc()

    _total_elapsed = _time.time() - _t0_total
    print(f"\n[AUTO] All {len(valid_dirs)} experiments processed in {_total_elapsed:.0f}s ({_total_elapsed/60:.1f}min)")

    save_fast_eval_summary(results, default_output_root, num_shards=num_shards, shard_index=shard_index)

    if fast_eval and full_plot_topk > 0:
        if num_shards == 1:
            ranked_results = sorted(results, key=lambda x: (x["mae"], x["name"]))[:full_plot_topk]
            print(f"[AUTO] Fast eval complete. Rendering full plots for top {len(ranked_results)} experiments.")
            for idx, row in enumerate(ranked_results, 1):
                exp_path = Path(row["path"])
                output_root = infer_output_root_from_exp_path(exp_path) or default_output_root
                fold_output_dir = output_root / get_experiment_group_name(exp_path) / exp_path.name
                fold_output_dir.mkdir(exist_ok=True, parents=True)
                print(f"\n[TOPK {idx}/{len(ranked_results)}] Rendering full plots for {exp_path.name}")
                full_res = load_and_evaluate(
                    exp_path,
                    device,
                    return_seqs=True,
                    return_extras=True,
                    stride_eval_override=None,
                )
                if not full_res:
                    continue
                render_full_outputs_for_result(
                    exp_path,
                    fold_output_dir,
                    full_res,
                    skip_feature_importance=skip_feature_importance,
                )
        else:
            print(f"[AUTO] Fast eval shard {shard_index + 1}/{num_shards} finished. Parent process should run --render-topk-only --full-plot-topk {full_plot_topk}.")
    
# -----------------------------------------------------------------------------
# Detailed Single Model Analysis (Feature Importance included)
# -----------------------------------------------------------------------------
def analyze_detailed_single(exp_path):
    # This function incorporates Feature Importance and Detailed Plotting
    # Reuses load_and_evaluate but needs model object for permutation importance
    
    print(f"Running Detailed Analysis for {exp_path.name}...")
    
    
    import yaml
    import torch
    import matplotlib.pyplot as plt
    import numpy as np
    import os
    import h5py
    from SpeedEstimator_TCN_MLP_experiments import extract_condition_data_v2
    
    # 1. Load Model & Data
    # load_and_evaluate already has the correct Training-consistent logic
    res = load_and_evaluate(exp_path, device, return_seqs=True)
    if not res: return
    
    # [REFACTORED] Training과 동일한 config 해석 (load_and_evaluate와 동일한 경로)
    import re
    folder_name = exp_path.name
    exp_name_match = re.match(r"^(.+?)_Test-", folder_name)
    exp_name = exp_name_match.group(1) if exp_name_match else "baseline" # Default if no match
    
    # Load base config (baseline.yaml) first, just like training
    base_config_path = Path("configs/baseline.yaml")
    if not base_config_path.exists():
        base_config_path = Path("config_backups/baseline.yaml")
    cfg = {}
    
    def deep_merge(base, override):
        for k, v in override.items():
            if k in base and isinstance(base[k], dict) and isinstance(v, dict):
                deep_merge(base[k], v)
            else:
                base[k] = v

    if base_config_path.exists():
        with open(base_config_path, 'r') as f:
            cfg = yaml.safe_load(f)
            print(f"[EVAL] Loaded base defaults from {base_config_path}")
    
    # Determine the experiment-specific config path
    original_config_path = Path(f"configs/{exp_name}.yaml")
    if original_config_path.exists():
        config_to_load = original_config_path
    else:
        # Fallback to per-experiment config.yaml
        config_to_load = exp_path / "config.yaml"

    with open(config_to_load, 'r') as f:
        exp_cfg = yaml.safe_load(f)
        # Use deep_merge for inheritance
        deep_merge(cfg, exp_cfg)
    
    # Config 파싱 - Training과 동일 (load_and_evaluate line 참조)
    curr_input_vars = cfg.get("input_vars")
    curr_output_vars = cfg.get("output_vars")
    if not curr_input_vars and 'shared' in cfg:
        curr_input_vars = cfg['shared'].get('input_vars')
    if not curr_output_vars and 'shared' in cfg:
        curr_output_vars = cfg['shared'].get('output_vars')
    if not curr_input_vars and '01_construction' in cfg:
        curr_input_vars = cfg['01_construction'].get('inputs')
    if not curr_output_vars and '01_construction' in cfg:
        curr_output_vars = cfg['01_construction'].get('outputs')
    
    def _parse_vars_local(var_list_from_yaml):
        if not var_list_from_yaml: return []
        return [(item[0], item[1]) for item in var_list_from_yaml]
    
    c_in_vars = _parse_vars_local(curr_input_vars)
    c_out_vars = _parse_vars_local(curr_output_vars)
    
    if not cfg.get('subjects') and 'shared' in cfg:
        cfg['subjects'] = cfg['shared'].get('subjects', [])
    if not cfg.get('conditions') and 'shared' in cfg:
        cfg['conditions'] = cfg['shared'].get('conditions', [])
    
    # est_tick_ranges (Training line 1853-1860)
    curr_est_tick_ranges = cfg.get("est_tick_ranges")
    if not curr_est_tick_ranges and 'shared' in cfg and 'data' in cfg['shared']:
        val = cfg['shared']['data'].get('est_tick_ranges')
        if val is not None:
            curr_est_tick_ranges = val
        else:
            y_delay = cfg['shared']['data'].get('y_delay', 5)
            curr_est_tick_ranges = [y_delay] if isinstance(y_delay, int) else y_delay
    if curr_est_tick_ranges:
        cfg["est_tick_ranges"] = curr_est_tick_ranges
    
    # Windowing (Training line 1335-1349과 동일)
    train_cfg_section = cfg.get("02_train", {})
    data_cfg = train_cfg_section.get("data", {})
    if not data_cfg: data_cfg = cfg.get("data", {})
    
    window_size = data_cfg.get("window_size") or cfg.get("window_size", 200)
    window_output = data_cfg.get("window_output") or data_cfg.get("time_window_output") or cfg.get("window_output", 1)
    data_stride = data_cfg.get("stride") or cfg.get("stride", 5)
    est_tick_ranges = data_cfg.get("est_tick_ranges") if data_cfg else cfg.get("est_tick_ranges")
    
    # Test subject
    match = re.search(r"Test-((?:m\d+_)?S\d+)", folder_name)
    test_sub = match.group(1) if match else "S001"
    
    # LPF (Training line 1868-1879)
    curr_lpf_cutoff = cfg.get("lpf_cutoff")
    if curr_lpf_cutoff is None and 'shared' in cfg:
        curr_lpf_cutoff = cfg['shared'].get('lpf_cutoff')
    if curr_lpf_cutoff is None:
        curr_lpf_cutoff = 0.5
    eval_lpf = curr_lpf_cutoff
    
    curr_lpf_order = cfg.get("lpf_order")
    if curr_lpf_order is None and 'shared' in cfg:
        curr_lpf_order = cfg['shared'].get('lpf_order')
    if curr_lpf_order is None:
        curr_lpf_order = 4

    curr_target_lpf_mode = cfg.get("target_lpf_mode")
    if curr_target_lpf_mode is None and 'shared' in cfg:
        curr_target_lpf_mode = cfg['shared'].get('target_lpf_mode')
    if curr_target_lpf_mode is None and '02_train' in cfg and 'data' in cfg['02_train']:
        curr_target_lpf_mode = cfg['02_train']['data'].get('target_lpf_mode')
    if curr_target_lpf_mode is None:
        curr_target_lpf_mode = "zero_phase"
    
    # [NEW] Multi-dataset path handling for condition check


    prefix_found, real_sub = parse_subject_prefix(test_sub)
    data_sources = get_data_sources_from_config(cfg)
    
    if prefix_found and prefix_found in data_sources:
        data_path = data_sources[prefix_found]['path']
    else:
        data_path = cfg.get("data_path", "combined_data_S008.h5")
        if not os.path.exists(data_path): data_path = "combined_data_S008.h5"
    
    # 2. Build Dataset (per-condition)
    actual_conds = []
    with h5py.File(data_path, 'r') as f_check:
        if real_sub in f_check:
            available_conds = list(f_check[real_sub].keys())
        else:
            print(f"[WARN] Subject {real_sub} not found in {data_path}.")
            available_conds = []

        for c in cfg["conditions"]:
            if c in available_conds:
                actual_conds.append(c)
            elif c == 'level':
                actual_conds.extend([ac for ac in available_conds if ac.startswith('level_')])
            else:
                matches = [ac for ac in available_conds if ac.startswith(c)]
                if matches: actual_conds.extend(matches)
    
    actual_conds = list(dict.fromkeys(actual_conds))
    
    print(f"[EVAL-Detail] win_in={window_size}, win_out={window_output}, stride={data_stride}, est_tick_ranges={est_tick_ranges}")
    X, Y = build_nn_dataset_multi(
        cfg, [test_sub], actual_conds, c_in_vars, c_out_vars, 
        window_size, window_output, data_stride,
        condition_selection=CONDITION_SELECTION, lpf_cutoff=eval_lpf, 
        lpf_order=curr_lpf_order,
        target_lpf_mode=curr_target_lpf_mode,
        est_tick_ranges=est_tick_ranges
    )
    
    if not X or len(X) == 0:
        print(f"[ERROR] build_nn_dataset returned empty X for {test_sub}. Check data availability.")
        return
    
    # 3. Apply Scaler (Training line 2040-2042과 동일)
    scaler_path = exp_path / "scaler.npz"
    if not scaler_path.exists():
        if "scaler_path" in cfg:
            p = Path(cfg["scaler_path"])
            if p.exists(): scaler_path = p
    
    if scaler_path.exists():
        print(f"[INFO] Loading scaler from {scaler_path}")
        scaler = np.load(scaler_path)
        mean = scaler['mean']
        scale = scaler['scale']
        print("[INFO] Applying scaler to test data...")
        for i in range(len(X)):
            X[i] = (X[i] - mean) / scale
    else:
        print("[WARN] No scaler found! Inference might be garbage if model expects scaled data.")

    input_dim = X[0].shape[1]
    
    # Model 초기화 (Training line 1277-1410과 동일)
    model_cfg = train_cfg_section.get("model")
    if not model_cfg:
        data_cfg_m = train_cfg_section.get("data")
        if data_cfg_m and "channels" in data_cfg_m: 
            model_cfg = data_cfg_m
    if not model_cfg: 
        model_cfg = cfg.get("model", {})
    
    tcn_channels = model_cfg.get("channels") or cfg.get("tcn_channels", (64, 64, 128))
    kernel_size = model_cfg.get("kernel_size") or cfg.get("kernel_size", 3)
    
    dropout_p = model_cfg.get("dropout")
    if dropout_p is None:
        dropout_p = cfg.get("dropout_p", 0.5)
    dropout_p = float(dropout_p)
    
    head_dropout = model_cfg.get("head_dropout")
    mlp_hidden = model_cfg.get("head_hidden") or cfg.get("mlp_hidden", 128)
    
    use_input_norm = cfg.get("data", {}).get("normalize", True)
    if use_input_norm is None:
        use_input_norm = cfg.get("use_input_norm", True)
    
    model_norm_type = model_cfg.get("model_norm", None)
    
    if est_tick_ranges:
        horizon = len(est_tick_ranges)
    else:
        horizon = window_output

    model_type = model_cfg.get("type", "TCN")
    common_args = dict(
        input_dim=input_dim,
        output_dim=Y[0].shape[1],
        horizon=horizon,
        channels=tcn_channels,
        kernel_size=kernel_size,
        dropout=dropout_p,
        head_dropout=head_dropout,
        mlp_hidden=mlp_hidden,
        use_input_norm=use_input_norm,
        tcn_norm=model_norm_type,
        mlp_norm=model_norm_type
    )
    if model_type == "StanceGatedTCN":
        model = StanceGatedTCN(**common_args, gating_dim=model_cfg.get("gating_dim", 1))
    elif model_type == "AttentionTCN":
        model = AttentionTCN(**common_args, attention_type=model_cfg.get("attention_type", "temporal"), attention_heads=model_cfg.get("attention_heads", 4))
    elif model_type == "TCN_GRU":
        model = TCN_GRU_Head(**common_args, gru_hidden=model_cfg.get("gru_hidden", 32))
    elif model_type == "DualBranchTCN_GRU":
        model = DualBranchTCN_GRU(
            **common_args,
            gru_hidden=model_cfg.get("gru_hidden", 32),
            branch_input_dims=model_cfg.get("branch_input_dims"),
            branch_channels=model_cfg.get("branch_channels"),
        )
    elif model_type == "MultiBranchTCN_GRU":
        model = MultiBranchTCN_GRU(
            **common_args,
            gru_hidden=model_cfg.get("gru_hidden", 32),
            branch_input_dims=model_cfg.get("branch_input_dims"),
            branch_channels=model_cfg.get("branch_channels"),
        )
    elif model_type == "FiLMConditionedTCN_GRU":
        model = FiLMConditionedTCN_GRU(
            **common_args,
            gru_hidden=model_cfg.get("gru_hidden", 32),
            branch_input_dims=model_cfg.get("branch_input_dims"),
            branch_channels=model_cfg.get("branch_channels"),
            film_scale=model_cfg.get("film_scale", 0.5),
        )
    elif model_type == "BoundedResidualTCN_GRU":
        model = BoundedResidualTCN_GRU(
            **common_args,
            gru_hidden=model_cfg.get("gru_hidden", 32),
            branch_input_dims=model_cfg.get("branch_input_dims"),
            branch_channels=model_cfg.get("branch_channels"),
            residual_scale=model_cfg.get("residual_scale", 0.30),
        )
    elif model_type == "AffineCalibTCN_GRU":
        model = AffineCalibTCN_GRU(
            **common_args,
            gru_hidden=model_cfg.get("gru_hidden", 32),
            branch_input_dims=model_cfg.get("branch_input_dims"),
            branch_channels=model_cfg.get("branch_channels"),
            alpha_scale=model_cfg.get("alpha_scale", 0.30),
            beta_scale=model_cfg.get("beta_scale", 0.25),
        )
    elif model_type == "PositiveResidualTCN_GRU":
        model = PositiveResidualTCN_GRU(
            **common_args,
            gru_hidden=model_cfg.get("gru_hidden", 32),
            branch_input_dims=model_cfg.get("branch_input_dims"),
            branch_channels=model_cfg.get("branch_channels"),
            residual_scale=model_cfg.get("residual_scale", 0.25),
            gate_scale=model_cfg.get("gate_scale", 1.0),
        )
    elif model_type == "AssistExpertTCN_GRU":
        model = AssistExpertTCN_GRU(
            **common_args,
            gru_hidden=model_cfg.get("gru_hidden", 32),
            branch_input_dims=model_cfg.get("branch_input_dims"),
            branch_channels=model_cfg.get("branch_channels"),
            gate_max=model_cfg.get("gate_max", 0.6),
        )
    elif model_type == "SlowFastCalibTCN_GRU":
        model = SlowFastCalibTCN_GRU(
            **common_args,
            gru_hidden=model_cfg.get("gru_hidden", 32),
            branch_input_dims=model_cfg.get("branch_input_dims"),
            branch_channels=model_cfg.get("branch_channels"),
            bias_scale=model_cfg.get("bias_scale", 0.20),
            residual_scale=model_cfg.get("residual_scale", 0.12),
        )
    elif model_type == "StructuredSlowFastTCN_GRU":
        model = StructuredSlowFastTCN_GRU(
            **common_args,
            gru_hidden=model_cfg.get("gru_hidden", 32),
            branch_input_dims=model_cfg.get("branch_input_dims"),
            branch_channels=model_cfg.get("branch_channels"),
            bias_scale=model_cfg.get("bias_scale", 0.18),
            residual_scale=model_cfg.get("residual_scale", 0.10),
            use_aux_head=model_cfg.get("use_aux_head", False),
            aux_dim=model_cfg.get("aux_dim", 1),
            use_subject_calibration=model_cfg.get("use_subject_calibration", False),
            subject_ctx_dims=model_cfg.get("subject_ctx_dims", 0),
            alpha_scale=model_cfg.get("alpha_scale", 0.08),
            beta_scale=model_cfg.get("beta_scale", 0.05),
        )
    elif model_type == "TCN_GRU_MDN":
        model = TCN_GRU_MDN(**common_args, gru_hidden=model_cfg.get("gru_hidden", 32), num_mixtures=int(model_cfg.get("num_mixtures", 5)))
    else:
        model = TCN_MLP(**common_args)
    model.to(device)

    model_path = exp_path / "model.pt"
    ckpt = torch.load(model_path, map_location=device)
    model.load_state_dict(ckpt['state_dict'] if 'state_dict' in ckpt else ckpt, strict=False)
    model.eval()
    
    # ---------------------------------------------------------
    # Trajectory Visualization Loop (Targeting 2x3 Subplots)
    # ---------------------------------------------------------
    # plot_dir is now determined by where it's called from. 
    # analyze_detailed_single is called mostly manually, but we can just use exp_path.parent.name
    output_root = infer_output_root_from_exp_path(exp_path) or Path("compare_result") / exp_path.parent.name
    plot_dir = output_root / folder_name / folder_name
    plot_dir.mkdir(parents=True, exist_ok=True)

    # [NEW] Plot Training Curves (Overfitting Check)
    exp_dir_path = Path("experiments") / folder_name
    print(f"Checking for train logs in {exp_dir_path}...")
    plot_training_curves(exp_dir_path, plot_dir)

    # [REFACTORED] Use shared function
    plot_detailed_condition_trajectories(exp_path, plot_dir, device)
    
    # ---------------------------------------------------------
    # Feature Importance Plot
    # ---------------------------------------------------------
    print("Preparing Feature Importance Data...")
    
    feature_names = generate_feature_names(c_in_vars)
    if len(feature_names) != input_dim:
        feature_names = [f"F{i}" for i in range(input_dim)]

    # Load ALL data for importance
    print(f"Loading data for Permutation Importance (Subject: {real_sub})...")
    # Load ALL data for importance using manual extraction (same as trajectory loop)
    # This avoids issues with build_nn_dataset_multi config parsing
    print(f"Loading data for Permutation Importance (Subject: {real_sub})...")
    X_imp_list = []
    Y_imp_list = []
    
    with h5py.File(data_path, 'r') as f_imp:
        for cond in actual_conds:
             if cond not in f_imp[real_sub]: continue
             
             # Extract ALL trials/levels for this condition
             # We use default arguments for include_levels/trials to get everything
             try:
                 x_list, y_list = extract_condition_data_v2(
                    f_imp, real_sub, cond, c_in_vars, c_out_vars,
                    lpf_cutoff=eval_lpf,
                    lpf_order=curr_lpf_order,
                    fs=100
                 )
                 # extract_condition_data_v2 returns list of trials
                 X_imp_list.extend(x_list)
                 Y_imp_list.extend(y_list)
             except Exception as e:
                 print(f"Skipping {cond} due to error: {e}")
                 
    if not X_imp_list:
        print("[WARN] No data loaded for feature importance. Skipping.")
        return 

    X_imp = np.concatenate(X_imp_list, axis=0) # (N, T, D)
    Y_imp = np.concatenate(Y_imp_list, axis=0)

    # Normalize
    if scaler_path.exists():
         X_imp = (X_imp - mean) / scale
         
    # Run Importance
    criterion = torch.nn.MSELoss()
    print("Running Permutation Importance...")
    try:
        ds = LazyWindowDataset([X_imp], [Y_imp], cfg.get("time_window_input", 100), 10, 20, target_mode="sequence", est_tick_ranges=curr_est_tick_ranges)
        loader = DataLoader(ds, batch_size=_FI_BATCH_SIZE, shuffle=False)
        importances = calculate_permutation_importance(model, loader, device, criterion, batch_size=_FI_BATCH_SIZE)
    except Exception as e:
        print(f"Importance calc failed: {e}")
        return
    
    # Plot Importance - Show ALL features
    indices = np.argsort(importances)[::-1]  # All features, sorted by importance
    
    # Dynamic figure height based on number of features
    num_features = len(indices)
    fig_height = max(8, num_features * 0.3)  # At least 8 inches, grow with features
    
    plt.figure(figsize=(14, fig_height))
    plt.barh(range(len(indices)), importances[indices])
    plt.yticks(range(len(indices)), [feature_names[i] for i in indices])
    plt.xlabel("Importance Score (MSE Increase)")
    # folder_name is already defined at top of function
    # folder_name = exp_path.name 
    plt.title(f"Feature Importance - All {num_features} Features ({folder_name})")
    plt.tight_layout()
    
    # Ensure compare_result/folder_name exists
    save_dir = plot_dir # plot_dir is compare_result/exp_name/exp_name
    # actually plot_dir is created at start of function: plot_dir = Path("compare_result") / exp_dir.name / exp_dir.name
    
    plt.savefig(plot_dir / "feature_importance.png", dpi=150)
    print(f"Saved {plot_dir}/feature_importance.png ({num_features} features)")
    print(f"Saved compare_result/{folder_name}_importance.png ({num_features} features)")

def calculate_permutation_importance(model, loader, device, criterion, batch_size=None):
    """
    Calculate Permutation Feature Importance with Batched Inference to Prevent OOM.
    Returns: np.array of importance scores (Loss increase)
    """
    model.eval()
    if batch_size is None:
        batch_size = _FI_BATCH_SIZE
    
    # 1. Calculate Baseline Loss
    # criterion is passed from outside
    total_loss = 0.0
    total_samples = 0
    
    # Store data on CPU to avoid VRAM overload, only move batch to GPU
    # Actually, if dataset is huge, we can't even store X/Y in RAM easily?
    # BUT, loader usually yields from RAM.
    # To shuffle a feature column, we need the full column?
    # YES. So we must collect all X. 
    # If X is too large for RAM, we have a problem. 
    # Assuming X fits in System RAM (64GB), but maybe not GPU VRAM (12GB).
    
    # Collect all data to CPU Tensor
    # Collect all data to CPU Tensor
    X_list, Y_list = [], []
    try:
        for i, (xb, yb) in enumerate(loader):
            X_list.append(xb)
            Y_list.append(yb)
    except Exception as e:
        print(f"    [ERROR] Data loading failed: {e}")
        return np.array([])
    
    if not X_list: 
        print("    [WARN] No data in loader!")
        return np.array([])
    
    X_cpu = torch.cat(X_list, dim=0) # (N, T, D)
    Y_cpu = torch.cat(Y_list, dim=0) # (N, T, D) or (N, D)

    
    del X_list, Y_list
    
    N, T, D = X_cpu.shape
    
    # Baseline Loop
    model.eval()
    with torch.no_grad():
        for i in range(0, N, batch_size):
            xb = X_cpu[i:i+batch_size].to(device)
            yb = Y_cpu[i:i+batch_size].to(device)
            pred = extract_primary_prediction(model(xb))
            
            # Align shapes
            if pred.ndim == 2 and yb.ndim == 3 and yb.shape[1] == 1: yb = yb.squeeze(1)
            elif pred.ndim == 3 and yb.ndim == 2 and pred.shape[1] == 1: pred = pred.squeeze(1)
            
            total_loss += criterion(pred, yb).item()
            
    base_loss = total_loss / N
    
    # Permutation Loop
    importances = []
    print(f"  Calcuating Importance for {D} features...")
    
    for d in range(D):
        print(f"    Feature {d+1}/{D}: ", end="", flush=True)
        # Permute in-place to avoid 9GB clone
        perm_idx = torch.randperm(N)
        original_col = X_cpu[:, :, d].clone()
        
        perm_total_loss = 0.0
        try:
            X_cpu[:, :, d] = X_cpu[perm_idx, :, d]
            
            with torch.no_grad():
                for i in range(0, N, batch_size):
                    xb = X_cpu[i:i+batch_size].to(device)
                    yb = Y_cpu[i:i+batch_size].to(device)
                    pred = extract_primary_prediction(model(xb))
                    
                    if pred.ndim == 2 and yb.ndim == 3 and yb.shape[1] == 1: yb = yb.squeeze(1)
                    elif pred.ndim == 3 and yb.ndim == 2 and pred.shape[1] == 1: pred = pred.squeeze(1)
                    
                    perm_total_loss += criterion(pred, yb).item()
        finally:
            # Restore original data
            X_cpu[:, :, d] = original_col
        
        perm_loss = perm_total_loss / N
        importance = perm_loss - base_loss
        importances.append(importance)
        print(f"Importance = {importance:.6f}")
        
    return np.array(importances)

def get_category(name):
    name = name.lower()
    if "baseline" in name:
        if "noangular" in name: return "Ablation (Input)"
        if "noimu" in name: return "Ablation (Input)"
        if "modelsame" in name: return "Architecture"
        return "Baseline"
    if "ablation" in name or "no_" in name: return "Ablation (Input)"
    if "stride" in name or "lpf" in name: return "Hyperparameters"
    if "instance_norm" in name: return "Architecture"
    if "grf" in name or "imu" in name or "pos" in name: return "Ablation (Input)"
    return "Other"

def analyze_ablation_report(target_dir="experiments", filter_pattern=None):
    print(f"Starting Ablation Analysis Report for {target_dir}...")
    # device is global in this script

    root_path = Path(target_dir)
    exp_dirs = collect_experiment_dirs(root_path, filter_pattern=filter_pattern)
    if not exp_dirs:
        print(f"No experiments found in '{target_dir}' directory.")
        return

    output_dir = infer_common_output_root(target_dir, exp_dirs)
    if output_dir is None:
        output_dir = Path("compare_result") / Path(target_dir).name
    output_dir.mkdir(exist_ok=True, parents=True)

    print(f"Found {len(exp_dirs)} potential experiment folders.")
    
    results = []
    
    # 2. Evaluate Loop
    for i, exp_dir in enumerate(exp_dirs):
        print(f"[{i+1}/{len(exp_dirs)}] Evaluating {exp_dir.name}...")
        try:
            # return_seqs=False to save memory/time
            res = load_and_evaluate(exp_dir, device, return_seqs=False)
            if res:
                results.append(res)
        except Exception as e:
            print(f"  [ERROR] Failed to evaluate {exp_dir.name}: {e}")
            
    if not results:
        print("No valid results collected.")
        return

    df = pd.DataFrame(results)
    
    # 3. Processing and Aggregation
    def get_group_name(folder_name):
        return folder_name.split("_Test-")[0]

    df['group'] = df['name'].apply(get_group_name)
    df['category'] = df['group'].apply(get_category)
    
    # Aggregate by Group (Mean metrics)
    group_df = df.groupby('group')[['mae', 'rmse', 'r2', 'lag_ms']].mean().reset_index()
    
    # Identify Baseline for Delta calculation
    baseline_rows = group_df[group_df['group'] == 'baseline']
    if len(baseline_rows) > 0:
        base_mae = baseline_rows.iloc[0]['mae']
        print(f"\nBaseline (baseline) found: MAE={base_mae:.4f}")
    else:
        # Try finding closest match
        candidates = group_df[group_df['group'].str.startswith('baseline')]
        if len(candidates) > 0:
            base_row = candidates.sort_values('mae').iloc[0]
            base_mae = base_row['mae']
            print(f"\nUsing '{base_row['group']}' as baseline (MAE={base_mae:.4f})")
        else:
            print("\nNo baseline found. Delta will be 0.")
            base_mae = 0.0
            
    group_df['mae_delta'] = group_df['mae'] - base_mae
    group_df['mae_pct_change'] = (group_df['mae_delta'] / base_mae) * 100 if base_mae > 0 else 0.0
    
    # Sort for better visibility (Ascending MAE)
    group_df = group_df.sort_values('mae', ascending=True)
    
    # Print Table
    print("\n" + "="*100)
    print(f"{'Experiment Group':<40} | {'MAE':<8} | {'Delta':<8} | {'% Chg':<8} | {'Lag(ms)':<8}")
    print("-" * 100)
    for _, row in group_df.iterrows():
        print(f"{row['group']:<40} | {row['mae']:.4f}   | {row['mae_delta']:+.4f}   | {row['mae_pct_change']:+.1f}%    | {row['lag_ms']:.1f}")
    print("="*100 + "\n")
    
    # Save CSV
    out_csv = output_dir / "final_ablation_summary.csv"
    group_df.to_csv(out_csv, index=False)
    print(f"Saved Summary CSV to: {out_csv}")
    
    # 4. Visualization
    # Plot 1: Absolute MAE Bar Chart
    plt.figure(figsize=(12, max(6, len(group_df)*0.4)))
    sns.barplot(data=group_df, x="mae", y="group", hue="group", dodge=False, palette="viridis")
    plt.axvline(base_mae, color='red', linestyle='--', label=f'Baseline ({base_mae:.4f})')
    plt.title("Model Performance Comparison (MAE)")
    plt.xlabel("MAE (m/s) [Lower is Better]")
    plt.legend()
    plt.tight_layout()
    plt.savefig(output_dir / "final_comparison_mae.png")
    
    # Plot 2: Delta MAE (Diverging Bar Chart)
    plt.figure(figsize=(12, max(6, len(group_df)*0.4)))
    
    colors = ['green' if x < 0 else 'red' for x in group_df['mae_delta']]
    
    sns.barplot(data=group_df, x="mae_delta", y="group", hue="group", dodge=False, palette=colors, legend=False)
    plt.axvline(0, color='black', linewidth=1)
    plt.title(f"Ablation Impact (Relative to Baseline MAE: {base_mae:.4f})")
    plt.xlabel("Change in MAE (m/s) [Positive = Worse, Negative = Better]")
    plt.tight_layout()
    plt.savefig(output_dir / "final_ablation_delta.png")
    
    print("Saved plots to compare_result/")

if __name__ == "__main__":
    args = get_args()
    target_dir = args.target_dir if args.target_dir else "experiments"
    if args.ablation:
        analyze_ablation_report(target_dir, filter_pattern=args.filter)
    elif args.render_topk_only:
        render_topk_full_plots(
            target_dir,
            filter_pattern=args.filter,
            top_k=args.full_plot_topk,
            skip_feature_importance=args.skip_feature_importance,
        )
    elif args.auto:
        analyze_automated(
            target_dir,
            args.filter,
            num_shards=args.num_shards,
            shard_index=args.shard_index,
            skip_feature_importance=args.skip_feature_importance,
            fast_eval=args.fast_eval,
            full_plot_topk=args.full_plot_topk,
        )
        # [NEW] Automatically run ablation report after auto-analysis
        if not args.skip_ablation:
            analyze_ablation_report(target_dir, filter_pattern=args.filter)
    else:
        main()
