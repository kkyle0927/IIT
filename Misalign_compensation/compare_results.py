
import os
import sys
import glob
import json
import torch
import numpy as np
import pandas as pd
import matplotlib
import os

# --- Resource Constraints (User Request: 1/4 System) ---
# Limit to 1 GPU (Index 0)
os.environ["CUDA_VISIBLE_DEVICES"] = "0"

# Check for display, otherwise use Agg backend (headless)
if os.environ.get('DISPLAY', '') == '':
    print('No display found. Using non-interactive Agg backend.')
    matplotlib.use('Agg')

# Limit CPU Threads to 16 (approx 1/4 of 64 cores)
torch.set_num_threads(16)
print(f"[INFO] Resources optimized: 1 GPU, 16 CPU Threads max.")
# --------------------------------------------------------

import argparse
import matplotlib.pyplot as plt
import seaborn as sns
from pathlib import Path
from torch.utils.data import DataLoader
from scipy import signal
from config_utils import (
    get_config_conditions,
    get_config_input_vars,
    get_config_output_vars,
    get_data_sources,
    get_est_tick_ranges,
    get_primary_data_path,
    get_train_data_config,
    get_window_size,
    normalize_experiment_config,
)

# -----------------------------------------------------------------------------
# Argument Parsing
# -----------------------------------------------------------------------------
def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--auto', action='store_true', help='Run in automated mode (no interaction)')
    parser.add_argument('--filter', type=str, default=None, help='Filter pattern for directories (e.g. "baseline_*")')
    parser.add_argument('--ablation', action='store_true', help='Generate ablation study comparison report')
    parser.add_argument('target_dir', nargs='?', help='Target experiment directory to analyze (optional)')
    return parser.parse_args()

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

from model_training import (
    TCN_MLP, StanceGatedTCN, AttentionTCN, TCN_GRU, LazyWindowDataset, build_nn_dataset, build_nn_dataset_multi,
    make_subject_selection, CONDITION_SELECTION
)
import model_training as main_script

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")


def find_config_by_exp_name(exp_name, config_root="configs"):
    matches = sorted(Path(config_root).rglob(f"{exp_name}.yaml"))
    return matches[0] if matches else None

def list_experiments(exp_root="experiments"):
    root = Path(exp_root)
    if not root.exists():
        print(f"Directory {root} does not exist.")
        return []

    exp_dirs = sorted({p.parent for p in root.rglob("model.pt")})
    return sorted(exp_dirs, key=lambda x: str(x))

def select_experiments(exp_dirs, auto=False):
    if auto:
        return exp_dirs
        
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
    
    lag_ms = -lag_samples * (1000.0 / fs)
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

def resolve_cfg(cfg):
    return normalize_experiment_config(cfg)

def apply_post_processing(y_pred_raw, y_true_raw, X_list, Y_list, cfg, 
                          window_size, window_output, stride_eval, est_tick_ranges):
    """
    Applies Overlap-Averaging and Post-LPF based on config.
    Returns (y_pred, y_true, offsets).
    """
    output_dim = y_pred_raw.shape[-1]
    
    print(f"[DEBUG] apply_post_processing start: X_list={len(X_list)} trials")
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
        print(f"[DEBUG] Single point mode branch started.")
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
        print(f"[DEBUG] Single point mode branch finished. len(offsets)={len(offsets)}")

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
            
    print(f"[DEBUG] apply_post_processing finished.")
        
    return y_pred, y_true, offsets

def load_and_evaluate(exp_dir, device, return_seqs=False, return_extras=False):
    """
    Loads model and config, rebuilds test dataset, runs inference.
    
    [REFACTORED] Training 스크립트(model_training.py)의
    config 해석, 데이터 로딩, normalization, 모델 초기화 경로를 정확히 재사용합니다.
    
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
    
    local_config = exp_path / "config.yaml"
    original_config_path = find_config_by_exp_name(exp_name)
    
    config_to_load = None
    if local_config.exists():
        config_to_load = local_config
        print(f"[INFO] Using local config: {local_config}")
    elif original_config_path and original_config_path.exists():
        config_to_load = original_config_path
        print(f"[WARN] Local config not found. Using global: {original_config_path}")
    else:
        print(f"[ERROR] Config file not found in {local_config} or {original_config_path}. Skipping.")
        return None
    
    with open(config_to_load, 'r', encoding='utf-8') as f:
        cfg = normalize_experiment_config(yaml.safe_load(f))
    
    print(f"\n{'='*60}", flush=True)
    print(f"[EVAL] Starting evaluation for {folder_name}", flush=True)
    print(f"[EVAL] {folder_name}")
    print(f"[EVAL] Valid Config: {config_to_load}")
    
    # test_sub is already extracted above
    
    # =========================================================================
    # Step 3: Config 파싱 - Training CV loop (line 1833-1905)과 동일한 경로
    # =========================================================================
    # input_vars / output_vars (Training line 1833-1840)
    curr_input_vars = get_config_input_vars(cfg)
    curr_output_vars = get_config_output_vars(cfg)
    
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
    curr_cond_names = get_config_conditions(cfg)
    
    # =========================================================================
    # Step 4: Data 파라미터 - Training과 동일 (line 1848-1860)
    # =========================================================================
    # window_size (Training line 1849-1851)
    curr_window = get_window_size(cfg, default=200)
    
    # est_tick_ranges (Training line 1853-1856, 1858-1860)
    curr_est_tick_ranges = get_est_tick_ranges(cfg)
    if curr_est_tick_ranges:
        cfg["est_tick_ranges"] = curr_est_tick_ranges
    
    # LPF (Training line 1868-1879)
    # Dataset logic (eval uses stride=1)
    
    # Data flags (Training line 1968-1971)
    data_cfg_now = get_train_data_config(cfg)
    use_phys_vel = data_cfg_now.get("use_physical_velocity_model", False)
    use_gait_ph  = data_cfg_now.get("use_gait_phase", False)
    use_cf = data_cfg_now.get("use_complementary_filter_euler", False)
    if not use_cf:
        use_cf = cfg.get("data", {}).get("use_complementary_filter_euler", False)
    
    # =========================================================================
    # Step 5: train_experiment 내부와 동일한 data_cfg 파싱 (line 1335-1349)
    # =========================================================================
    train_cfg_section = cfg.get("02_train", {})
    data_cfg = get_train_data_config(cfg)
    if not data_cfg: data_cfg = cfg.get("data", {})
    
    # Training line 1341-1344: 정확한 windowing 파라미터
    window_size = data_cfg.get("window_size") or cfg.get("window_size", 200)
    window_output = data_cfg.get("window_output") or data_cfg.get("time_window_output") or cfg.get("window_output", 1)
    data_stride = data_cfg.get("stride") or cfg.get("stride", 5)
    
    # Training line 1346-1347: est_tick_ranges
    est_tick_ranges = data_cfg.get("est_tick_ranges") if data_cfg else cfg.get("est_tick_ranges")
    
    # Evaluation stride = 1 (denser than training for smooth trajectory)
    stride_eval = 1
    
    print(f"[EVAL] Data: win_in={window_size}, win_out={window_output}, stride_train={data_stride}, stride_eval={stride_eval}")
    print(f"[EVAL] est_tick_ranges={est_tick_ranges}")
    
    # Explicitly inject data_sources so build_nn_dataset_multi can find them
    srcs = get_data_sources(cfg)
    if srcs:
        if 'shared' not in cfg: cfg['shared'] = {}
        cfg['shared']['data_sources'] = srcs

    # [NEW] Residual Learning Mode Detection
    use_residual = data_cfg_now.get("residual_target", False)
    use_calib_thigh = data_cfg_now.get("calibrate_robot_thigh", False)

    print(f"[EVAL] Loading Test Data for Subject {test_sub}...")
    if use_residual:
        print(f"[EVAL] Residual target mode detected — will restore to absolute values after inference.")

    common_build_args = dict(
        config=cfg,
        sub_names=[test_sub],
        cond_names=curr_cond_names,
        input_vars=c_in_vars, output_vars=c_out_vars,
        time_window_input=window_size,
        time_window_output=window_output,
        stride=stride_eval,
        subject_selection=None,
        condition_selection=CONDITION_SELECTION,
        est_tick_ranges=est_tick_ranges,
        use_physical_velocity_model=use_phys_vel,
        use_gait_phase=use_gait_ph,
        use_complementary_filter_euler=use_cf,
        calibrate_robot_thigh=use_calib_thigh
    )

    X_list, Y_list = main_script.build_nn_dataset_multi(
        **common_build_args, residual_target=use_residual
    )

    # If residual mode, also load absolute targets for restoration
    Y_abs_list = None
    if use_residual:
        _, Y_abs_list = main_script.build_nn_dataset_multi(
            **common_build_args, residual_target=False
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
    # DataLoader 대신 간단한 Numpy Batch Generator 사용 (고착 방지)
    # =========================================================================
    ds = LazyWindowDataset(
        X_list, Y_list, 
        window_size, window_output, stride_eval,
        target_mode="sequence", 
        est_tick_ranges=est_tick_ranges
    )
    
    # Simple DataLoader replacement
    def memory_safe_loader(dataset, batch_size=1024):
        total = len(dataset)
        for i in range(0, total, batch_size):
            end = min(i + batch_size, total)
            batch_x = []
            batch_y = []
            for j in range(i, end):
                x, y = dataset[j]
                batch_x.append(x)
                batch_y.append(y)
            yield torch.stack(batch_x), torch.stack(batch_y)
            
    loader = memory_safe_loader(ds, batch_size=1024)
    
    # =========================================================================
    # Step 9: 모델 초기화 - train_experiment (line 1275-1410)과 동일
    # =========================================================================
    input_dim = X_list[0].shape[1]
    output_dim = Y_list[0].shape[1]
    
    if est_tick_ranges:
        horizon = len(est_tick_ranges)
    else:
        horizon = window_output
    
    # Model Config (Training line 1277-1285)
    model_cfg = train_cfg_section.get("model")
    if not model_cfg:
        data_cfg_m = train_cfg_section.get("data")
        if data_cfg_m and "channels" in data_cfg_m: model_cfg = data_cfg_m
    if not model_cfg: model_cfg = cfg.get("model") or {}
    
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
    
    model_norm_type = model_cfg.get("model_norm", None)
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
    else:
        model = TCN_MLP(**common_args)
    
    model.to(device)
    
    # Step 10: State Dict 로드 - Training line 2085-2086과 동일
    import warnings
    print(f"[DEBUG] Loading {model_path} onto {device}...")
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", category=FutureWarning)
        ckpt = torch.load(model_path, map_location=device, weights_only=False)
    print(f"[DEBUG] Load complete.")
    model.load_state_dict(ckpt["state_dict"] if "state_dict" in ckpt else ckpt, strict=False)
    model.eval()
    
    y_preds = []
    y_trues = []
    
    print(f"[DEBUG] Starting evaluate DataLoader loop...")
    with torch.no_grad():
        for b_idx, (xb, yb) in enumerate(loader):
            xb = xb.to(device)
            out = model(xb)
            y_preds.append(out.cpu().numpy())
            y_trues.append(yb.cpu().numpy())
    print(f"[DEBUG] Finished evaluate DataLoader loop.")
            
    y_pred_raw = np.concatenate(y_preds, axis=0)
    y_true_raw = np.concatenate(y_trues, axis=0)
    
    y_pred, y_true, offsets = apply_post_processing(
        y_pred_raw, y_true_raw, X_list, Y_list, cfg,
        window_size, window_output, stride_eval, est_tick_ranges
    )

    # [NEW] Residual → Absolute Restoration
    if use_residual and Y_abs_list:
        print(f"[EVAL] Restoring residual predictions to absolute thigh angles...")
        # Compute robot_ref = Y_abs - Y_res per trial (= calibrated robot thigh)
        robot_ref_list = [Y_abs_list[i] - Y_list[i] for i in range(len(Y_list))]
        # Extract robot_ref at same window positions using LazyWindowDataset
        ds_ref = LazyWindowDataset(
            X_list, robot_ref_list,
            window_size, window_output, stride_eval,
            target_mode="sequence", est_tick_ranges=est_tick_ranges
        )
        ref_values = []
        for j in range(len(ds_ref)):
            _, ref_y = ds_ref[j]
            ref_values.append(ref_y.numpy() if hasattr(ref_y, 'numpy') else ref_y)
        robot_ref_raw = np.stack(ref_values)
        # Apply same post-processing to robot_ref
        robot_ref, _, _ = apply_post_processing(
            robot_ref_raw, robot_ref_raw, X_list, robot_ref_list, cfg,
            window_size, window_output, stride_eval, est_tick_ranges
        )
        # Squeeze robot_ref to match y_pred shape (prevent broadcasting OOM)
        if robot_ref.ndim == 3 and robot_ref.shape[1] == 1:
            robot_ref = robot_ref[:, 0, :]
        if y_pred.ndim == 3 and y_pred.shape[1] == 1:
            y_pred = y_pred[:, 0, :]
        if y_true.ndim == 3 and y_true.shape[1] == 1:
            y_true = y_true[:, 0, :]
        # Restore to absolute values
        y_pred = y_pred + robot_ref
        y_true = y_true + robot_ref
        print(f"[EVAL] Residual restoration complete. y_pred range: [{y_pred.min():.2f}, {y_pred.max():.2f}]")

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
    
    # [MOVED UP] offsets calculation was here

    if return_seqs:
        ret["y_true_seq"] = y_true
        ret["y_pred_seq"] = y_pred

    if return_extras:
        ret["model"] = model
        ret["cfg"] = cfg
        ret["c_in_vars"] = c_in_vars
        ret["c_out_vars"] = c_out_vars
        ret["X_list"] = X_list
        ret["Y_list"] = Y_list
        ret["scaler_mean"] = mean if scaler_path.exists() else None
        ret["scaler_scale"] = scale if scaler_path.exists() else None
        ret["window_size"] = window_size
        ret["est_tick_ranges"] = est_tick_ranges
        ret["test_sub"] = test_sub
        
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

def get_feature_names(cfg, input_vars):
    """Reconstruct feature names based on config and input variables."""
    feature_names = []
    for gpath, vars in input_vars:
        # Determine prefix based on group path (Mirroring main script logic)
        gpath_lower = gpath.lower()
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
            # Clean up individual variable names
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
    print("[DEBUG] Entered main()", flush=True)
    args = get_args()
    print(f"[DEBUG] Args parsed: {args}", flush=True)
    
    exp_dirs = list_experiments()
    if not exp_dirs:
        print("No experiments found.")
        return

    # Filter with --filter if provided
    if args.filter:
        import fnmatch
        exp_dirs = [d for d in exp_dirs if fnmatch.fnmatch(d.name, args.filter)]
        if not exp_dirs:
             print(f"No experiments found matching filter {args.filter}")
             return

    selected = select_experiments(exp_dirs, auto=args.auto)
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
    plt.xlabel("MAE (deg)")
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
                
            plt.title(f"Multi-Model Hip Angle Estimation Comparison ({dur}s)")
            plt.xlabel("Time (s)")
            plt.ylabel("Hip Angle (deg)")
            plt.legend()
            plt.grid(True, alpha=0.3)
            plt.tight_layout()
            
            fname = output_dir / f"compare_models_{dur}s.png"
            plt.savefig(fname)
            print(f"Saved {fname}")
            plt.close()

    print("Multi-model comparison complete.")

def analyze_single_folder(folder_path):
    print(f"\n[Auto Analysis] Processing: {folder_path}")
    
    config_path = os.path.join(folder_path, "config_dump.yaml")
    if not os.path.exists(config_path):
        yamls = glob.glob(os.path.join(folder_path, "*.yaml"))
        if yamls: config_path = yamls[0]
        else:
            print("  -> No config found. Skipping.")
            return

    import yaml
    with open(config_path, 'r') as f:
        cfg = normalize_experiment_config(yaml.safe_load(f))

    input_vars = get_config_input_vars(cfg)
    out_groups = get_config_output_vars(cfg)
    output_vars_flat = []
    for item in out_groups:
        if isinstance(item, list) and len(item) == 2:
            gpath, vars = item
            output_vars_flat.extend([f"{gpath}/{v}" for v in vars])
        else:
            output_vars_flat.append(item)
    data_path = get_primary_data_path(cfg)
    window = get_window_size(cfg, default=100)
    stride = get_train_data_config(cfg).get("stride", cfg.get("stride", 1))
    est_tick_ranges = get_est_tick_ranges(cfg)

    model_path = os.path.join(folder_path, "model.pt")
    if not os.path.exists(model_path):
        print("  -> No model found. Skipping.")
        return
        
    device = "cuda" if torch.cuda.is_available() else "cpu"
    flat_in = []
    for item in input_vars:
        if isinstance(item, list): flat_in.extend([f"{item[0]}/{v}" for v in item[1]])
        else: flat_in.append(item)
    in_dim = len(flat_in)
    out_dim = len(output_vars_flat) # Use flat
    model_cfg = cfg.get("02_train", {}).get("model") or cfg.get("model", {})
    tcn_ch = model_cfg.get("channels") or cfg.get("tcn_channels", [32, 32, 64, 64])
    k_size = model_cfg.get("kernel_size") or cfg.get("kernel_size", 4)
    print(f"[DEBUG] Model CFG keys: {list(model_cfg.keys())}")
    print(f"[DEBUG] Extracted Kernel Size: {k_size}")
    dropout = float(model_cfg.get("dropout") or cfg.get("dropout_p", 0.3))
    head_dropout = model_cfg.get("head_dropout")
    
    hidden = model_cfg.get("head_hidden") or cfg.get("mlp_hidden", [64, 32])
    
    norm_type = model_cfg.get("model_norm", "layer")

    try:
        model = TCN_MLP(
            in_dim, out_dim, 
            horizon=1, 
            channels=tcn_ch, 
            kernel_size=k_size, 
            dropout=dropout,
            head_dropout=head_dropout,
            tcn_norm=norm_type,
            mlp_norm=norm_type,
            mlp_hidden=hidden, 
            use_input_norm=cfg.get("shared", {}).get("data", {}).get("normalize", True)
        ).to(device)
        model.load_state_dict(torch.load(model_path, map_location=device), strict=False)
        model.eval()
    except Exception as e:
        print(f"  -> Model load failed: {e}")
        return

    if not os.path.exists(data_path): 
        if os.path.exists("combined_data.h5"): data_path = "combined_data.h5"
    
    conditions = ['accel_sine', 'decline_5deg', 'incline_10deg', 'level_075mps', 'level_100mps', 'level_125mps', 'stopandgo']
    import re
    match = re.search(r"Test-(S\d+)", folder_path)
    test_subjs = [match.group(1)] if match else ["S008"]

    save_dir = os.path.join("compare_result", os.path.basename(folder_path))
    os.makedirs(save_dir, exist_ok=True)
    print(f"  -> Saving plots to: {save_dir}")
    
    # [NEW] Check Overfitting
    plot_training_curves(Path(folder_path), Path(save_dir))
    
    results = []
    for subj in test_subjs:
        for cond in conditions:
            try:
                X_list, Y_list = build_nn_dataset(
                    data_path, [subj], [cond], 
                    input_vars, out_groups, # Use GROUPED OUTPUTS
                    window, 1, stride, # 1 dummy output window
                    est_tick_ranges=est_tick_ranges
                )
                
                # [FIX] Apply Scaler Logic (Same as analyze_detailed_single)
                scaler_path = folder_path / "scaler.npz"
                if not scaler_path.exists():
                     if "scaler_path" in cfg and cfg["scaler_path"]:
                          p = Path(cfg["scaler_path"])
                          if p.exists(): scaler_path = p
                          
                if scaler_path.exists():
                    try:
                        scaler = np.load(scaler_path)
                        mean = scaler['mean']
                        scale = scaler['scale']
                        for i in range(len(X_list)):
                            X_list[i] = (X_list[i] - mean) / (scale + 1e-8)
                        # print(f"    -> Scaler applied for {subj}/{cond}")
                    except: pass
                
                if len(X_list) == 0: continue

                ds = LazyWindowDataset(X_list, Y_list, window, 1, stride, est_tick_ranges=est_tick_ranges)
                if len(ds) == 0: continue
                
                dl = DataLoader(ds, batch_size=1024, shuffle=False, num_workers=0)
                
                preds, targets = [], []
                with torch.no_grad():
                    for bx, by in dl:
                        bx = bx.to(device)
                        p = model(bx)
                        if isinstance(p, list): p = p[0]
                        preds.append(p.cpu().numpy())
                        targets.append(by.numpy())
                preds = np.concatenate(preds)
                targets = np.concatenate(targets)
                mae = np.mean(np.abs(preds - targets))
                rmse = np.sqrt(np.mean((preds - targets)**2))
                
                results.append({'subj': subj, 'cond': cond, 'mae': mae, 'rmse': rmse})

                # [FIX] Separate Left/Right Plots
                t_plot = targets[:1000] # (T, 2)
                p_plot = preds[:1000]   # (T, 2)

                fig, axes = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
                
                # Left Hip (Index 0)
                axes[0].plot(t_plot[:, 0], label='True Hip Angle (L)', color='black', alpha=0.6)
                axes[0].plot(p_plot[:, 0], label='Compensated (L)', color='red', alpha=0.7)
                axes[0].set_title(f"[{subj}] {cond} - Left Hip | MAE={np.mean(np.abs(p_plot[:, 0] - t_plot[:, 0])):.4f}")
                axes[0].legend()
                axes[0].grid(True, alpha=0.3)
                axes[0].set_ylabel("Angle (deg)")
                
                # Right Hip (Index 1) - If exists
                if t_plot.shape[1] > 1:
                    axes[1].plot(t_plot[:, 1], label='True Hip Angle (R)', color='black', alpha=0.6)
                    axes[1].plot(p_plot[:, 1], label='Compensated (R)', color='red', alpha=0.7)
                    axes[1].set_title(f"Right Hip | MAE={np.mean(np.abs(p_plot[:, 1] - t_plot[:, 1])):.4f}")
                    axes[1].legend()
                    axes[1].grid(True, alpha=0.3)
                    axes[1].set_ylabel("Angle (deg)")
                else:
                    axes[1].text(0.5, 0.5, "Single Channel Output", ha='center')
                    
                plt.xlabel("Time (ticks)")
                plt.tight_layout()
                plt.savefig(os.path.join(save_dir, f"{subj}_{cond}.png"))
                plt.close()
                print(f"    -> {subj}/{cond}: Overall MAE={mae:.4f}")
            except Exception as e:
                print(f"   -> Error on {subj}/{cond}: {e}")
    
    # Save Summary CSV
    import pandas as pd
    if results:
        df = pd.DataFrame(results)
        df.to_csv(os.path.join(save_dir, "summary.csv"), index=False)
        print(f"  -> Summary saved to {save_dir}/summary.csv")

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
    axes[0].plot(t_plot, y_true_plot, label='True Thigh Angle', alpha=0.7, color='black')
    axes[0].plot(t_plot, y_pred_plot, label='Compensated', alpha=0.9, color='red', linestyle='--')
    
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
    axes[0].set_ylabel("Thigh Angle (deg)")
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
        axes[1].set_ylabel("Thigh Angle (deg)")
        axes[1].legend()
        axes[1].grid(True, alpha=0.3)
    else:
        axes[1].text(0.5, 0.5, "Data too short for zoom", ha='center')
        
    plt.tight_layout()
    plt.savefig(save_path)
    plt.close()


def plot_detailed_condition_trajectories(exp_path, save_dir, device):
    """
    Generates 2x3 grid plots (Levels x Zooms) for each condition.
    Copied/Refactored from analyze_detailed_single to be reusable.
    """
    print(f"[INFO] Generating Detailed Grid Plots for {exp_path.name}...")
    import yaml, re, h5py
    from torch.utils.data import DataLoader
    
    folder_name = exp_path.name
    
    # 1. Load Config
    exp_name_match = re.match(r"^(.+?)_Test-", folder_name)
    exp_name = exp_name_match.group(1) if exp_name_match else "baseline"
    
    local_config = exp_path / "config.yaml"
    global_config = find_config_by_exp_name(exp_name)
    
    cfg = None
    if local_config.exists():
        print(f"  -> Using local config: {local_config}")
        with open(local_config, 'r', encoding='utf-8') as f_cfg: cfg = normalize_experiment_config(yaml.safe_load(f_cfg))
    elif global_config and global_config.exists():
        print(f"  -> Local config missing. Using global: {global_config}")
        with open(global_config, 'r') as f_cfg: cfg = normalize_experiment_config(yaml.safe_load(f_cfg))
    else:
        print("  -> Config not found anywhere, skipping detailed plots.")
        return

    # 2. Parse Variables
    def _parse_vars_local(var_list_from_yaml):
        if not var_list_from_yaml: return []
        parsed = []
        for item in var_list_from_yaml:
            if isinstance(item, (list, tuple)) and len(item) == 2:
                parsed.append((item[0], item[1]))
        return parsed

    # input_vars / output_vars
    curr_input_vars = get_config_input_vars(cfg)
    curr_output_vars = get_config_output_vars(cfg)
    
    c_in_vars = _parse_vars_local(curr_input_vars)
    c_out_vars = _parse_vars_local(curr_output_vars)
    
    # 3. Data Parameters
    train_cfg_section = cfg.get("02_train", {})
    data_cfg = train_cfg_section.get("data", {})
    if not data_cfg: data_cfg = cfg.get("data", {})
    
    window_size = data_cfg.get("window_size") or cfg.get("window_size", 200)
    window_output = data_cfg.get("window_output") or data_cfg.get("time_window_output") or cfg.get("window_output", 1)
    est_tick_ranges = data_cfg.get("est_tick_ranges") if data_cfg else cfg.get("est_tick_ranges")
    
    # LPF logic removed

    # 4. Identify Subject and Data Path
    match = re.search(r"Test-((?:m\d+_)?S\d+)", folder_name)
    test_sub = match.group(1) if match else "S001"
    
    prefix_found, real_sub = parse_subject_prefix(test_sub)
    data_sources = get_data_sources_from_config(cfg)
    if prefix_found and prefix_found in data_sources:
        data_path = data_sources[prefix_found]['path']
    else:
        data_path = cfg.get("data_path", "combined_data.h5")
        if not os.path.exists(data_path): data_path = "combined_data.h5"

    if not os.path.exists(data_path):
        print(f"  -> Data path {data_path} not found. Skipping.")
        return

    # 5. Load Model
    model_path = exp_path / "model.pt"
    if not model_path.exists(): return
    
    # Reconstruct Model (Partial duplication of load_and_evaluate logic)
    input_dim = 0
    # Count vars:
    count = 0
    for g, vs in c_in_vars: count += len(vs)
    # Add Gait Phase?
    use_gait_ph = cfg.get("02_train", {}).get("data", {}).get("use_gait_phase") or cfg.get("use_gait_phase")
    if use_gait_ph: count += 2 # L/R

    input_dim = count
    
    # Output dim
    out_dim = 0
    for g, vs in c_out_vars: out_dim += len(vs)
    if out_dim == 0: out_dim = 1 # Fallback
    
    model_cfg = train_cfg_section.get("model")
    if not model_cfg: model_cfg = data_cfg if "channels" in data_cfg else cfg.get("model", {})
    
    # Debug
    print(f"[DEBUG] Model CFG keys: {model_cfg.keys() if model_cfg else 'None'}")
    
    tcn_channels = model_cfg.get("channels") or cfg.get("tcn_channels", (64, 64, 128))
    kernel_size = model_cfg.get("kernel_size") or cfg.get("kernel_size", 3)
    dropout_p = float(model_cfg.get("dropout") or cfg.get("dropout_p", 0.5))
    head_dropout = model_cfg.get("head_dropout", dropout_p)
    
    # head_hidden can be a list [128, 64] or int 64 — pass as-is since TCN_MLP supports both
    raw_hidden = model_cfg.get("head_hidden") or cfg.get("mlp_hidden")
    mlp_hidden = raw_hidden if raw_hidden is not None else 128

    print(f"[DEBUG] mlp_hidden resolved to: {mlp_hidden}")

    use_input_norm = cfg.get("data", {}).get("normalize", True)
    model_norm_type = model_cfg.get("model_norm", None)
    
    horizon = len(est_tick_ranges) if est_tick_ranges else window_output
    
    model_type = model_cfg.get("type", "TCN")
    common_args = dict(input_dim=input_dim, output_dim=out_dim, horizon=horizon, channels=tcn_channels, kernel_size=kernel_size, dropout=dropout_p, head_dropout=head_dropout, mlp_hidden=mlp_hidden, use_input_norm=use_input_norm, tcn_norm=model_norm_type, mlp_norm=model_norm_type)

    if model_type == "StanceGatedTCN":
        model = StanceGatedTCN(**common_args, gating_dim=model_cfg.get("gating_dim", 1))
    elif model_type == "AttentionTCN":
        model = AttentionTCN(**common_args, attention_type=model_cfg.get("attention_type", "temporal"), attention_heads=model_cfg.get("attention_heads", 4))
    elif model_type == "TCN_GRU":
        gru_hidden = model_cfg.get("gru_hidden", 64)
        gru_layers = model_cfg.get("gru_layers", 1)
        bidirectional = model_cfg.get("bidirectional", False)
        model = TCN_GRU(**common_args, gru_hidden=gru_hidden, gru_layers=gru_layers, bidirectional=bidirectional)
    else:
        model = TCN_MLP(**common_args)
        
    ckpt = torch.load(model_path, map_location=device)
    model.load_state_dict(ckpt['state_dict'] if 'state_dict' in ckpt else ckpt, strict=False)
    model.to(device)
    model.eval()

    # Scaler
    scaler_path = exp_path / "scaler.npz"
    mean, scale = None, None
    if scaler_path.exists():
        s_data = np.load(scaler_path)
        mean = s_data['mean']
        scale = s_data['scale']

    # 6. Plot Loop
    from model_training import extract_condition_data_v2
    
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
            
            # Figure width depends on columns
            fig, axes = plt.subplots(4, len(target_lvs), figsize=(6*len(target_lvs), 16), squeeze=False)
            has_data = False
            
            # Retrieve CF flag (check 02_train.data first, then shared.data, then root data)
            use_cf = cfg.get("02_train", {}).get("data", {}).get("use_complementary_filter_euler", False)
            if not use_cf:
                use_cf = cfg.get("shared", {}).get("data", {}).get("use_complementary_filter_euler", False)
            if not use_cf:
                use_cf = cfg.get("data", {}).get("use_complementary_filter_euler", False)

            # [NEW] Residual mode for detailed plots
            detail_residual = data_cfg.get("residual_target", False)
            detail_calib = data_cfg.get("calibrate_robot_thigh", False)

            for col_idx, lv_name in enumerate(target_lvs):
                try:
                    # Force Single Trial extract
                    X_list, Y_list = extract_condition_data_v2(
                        f, real_sub, cond, c_in_vars, c_out_vars,
                        include_levels=[lv_name],
                        fs=100, use_complementary_filter_euler=use_cf,
                        residual_target=detail_residual,
                        calibrate_robot_thigh=detail_calib
                    )
                except Exception as e:
                    X_list = []
                    
                if not X_list:
                    for r in range(4): axes[r, col_idx].axis('off')
                    continue
                
                has_data = True
                X_arr = X_list[0]
                Y_arr = Y_list[0]
                # [NEW] Robust Robot Angle Extraction for L/R
                robot_seq_L = None
                robot_seq_R = None
                
                # Find indices for 'robot' -> 'hip_angle' (or fallback)
                flat_input_map = []
                idx_counter = 0
                for g_path, v_list in c_in_vars:
                    for v_name in v_list:
                        flat_input_map.append({
                            'idx': idx_counter,
                            'group': g_path,
                            'name': v_name,
                            'full': f"{g_path}/{v_name}"
                        })
                        idx_counter += 1
                
                # Search for Left and Right Hip Angles
                for item in flat_input_map:
                    # Check for explicit "hip_angle" or fallback "motor_angle"
                    is_angle = 'hip_angle' in item['name'] or 'motor_angle' in item['name']
                    if not is_angle: continue
                    
                    # Check side
                    path_lower = item['full'].lower()
                    if 'left' in path_lower:
                        if robot_seq_L is None: robot_seq_L = X_arr[:, item['idx']]
                    elif 'right' in path_lower:
                        if robot_seq_R is None: robot_seq_R = X_arr[:, item['idx']]
                
                if mean is not None:
                    try:
                        X_arr = (X_arr - mean) / (scale + 1e-8)
                    except ValueError: pass
                
                # Inference
                ds_vis = LazyWindowDataset([X_arr], [Y_arr], window_size, window_output, 1, target_mode="sequence", est_tick_ranges=est_tick_ranges)
                loader_vis = DataLoader(ds_vis, batch_size=512, shuffle=False, num_workers=0)
                
                preds, targets = [], []
                with torch.no_grad():
                     for xb, yb in loader_vis:
                         xb = xb.to(device)
                         out = model(xb)
                         if isinstance(out, list): out = out[0]
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
                
                p_seq = p_seq_full.squeeze()
                t_seq = t_seq_full.squeeze()
                
                # Determine channels to plot (Max 2: Left, Right)
                # If 1D, treat as Left (or single). If 2D, 0=Left, 1=Right (usually).
                n_channels = 1
                if p_seq.ndim > 1: n_channels = p_seq.shape[1]
                
                fs_Hz = 100.0
                t = np.arange(len(t_seq)) / fs_Hz
                
                # Plot Configuration: 
                # We have 4 rows reserved: [Full, Zoom1, Zoom2, Zoom3].
                # If we have 2 channels, we can either:
                # A) Overlay them (cluttered)
                # B) Split rows? (Current layout is fixed 4 rows)
                # C) Plot Channel 0 (Left) and Channel 1 (Right) in the SAME subplot but different styles?
                # Decision: Plot Channel 0 (Left) primarily or overlay both with clear colors.
                # Requirement: Show Robot, True, Compensated.
                # Let's start with Channel 0 (Left) as primary, and Channel 1 (Right) if requested.
                # Actually, user wants "Pre-calibration Robot", "Compensated", "True".
                # Let's plot BOTH if available, using subplots?
                # No, the grid is fixed. Let's overlay but make Right lighter?
                
                # Better: Allow checking L/R.
                # Let's just plot Channel 0 (Left) for clarity if we can't change grid.
                # BUT, Hip angles are usually L/R. 
                # Let's stick to Channel 0 (Left) for now to match the "single column" design, 
                # OR plot Channel 0 in top half and Channel 1 in bottom half of lines?
                # Let's go with: Channel 0 (Left).
                
                # Channel 0
                p_0 = p_seq[:, 0] if n_channels > 1 else p_seq
                t_0 = t_seq[:, 0] if n_channels > 1 else t_seq
                r_0 = robot_seq_L # Left default

                # If only Right exists for some reason?
                if r_0 is None and robot_seq_R is not None: r_0 = robot_seq_R

                # [NEW] Residual → Absolute restoration for detailed plots
                # First prediction corresponds to Y index (window_size - 1)
                # because est_tick_ranges=[0] → y_index = t + window_size + 0 - 1
                if detail_residual and r_0 is not None:
                    start_idx = int(window_size) - 1
                    robot_ref = r_0[start_idx : start_idx + len(p_0)]
                    min_len = min(len(robot_ref), len(p_0))
                    if min_len > 0:
                        p_0[:min_len] = p_0[:min_len] + robot_ref[:min_len]
                        t_0[:min_len] = t_0[:min_len] + robot_ref[:min_len]

                # Prepare Robot Plot Data (Align length)
                r_plot = None
                if r_0 is not None:
                     # Align: t=0 corresponds to (window_size - 1) in X
                     match_len = len(t)
                     start_idx = int(window_size) - 1
                     available = len(r_0) - start_idx
                     if available > 0:
                         plot_len = min(match_len, available)
                         r_plot = r_0[start_idx : start_idx + plot_len]

                # Row 0: Full Duration
                ax0 = axes[0, col_idx]
                ax0.plot(t, t_0, color='black', alpha=0.7, label='MoCap Thigh (Left)')
                if r_plot is not None:
                     ax0.plot(t[:len(r_plot)], r_plot, color='green', alpha=0.5, label='Robot Thigh (Calibrated)')

                ax0.plot(t, p_0, color='red', alpha=0.9, linestyle='--', label='Estimated Thigh')
                ax0.set_title(f"{lv_name} (Left Thigh)")
                ax0.grid(True, alpha=0.3)
                if col_idx == 0: ax0.legend(loc='upper right', fontsize='small')
                
                # Rows 1-3: Zooms
                windows = [(0, 15), (15, 30), (30, 45)]
                for r, (start_t, end_t) in enumerate(windows, start=1):
                    ax = axes[r, col_idx]
                    real_end = min(end_t, t[-1])
                    if start_t >= t[-1]:
                        ax.text(0.5, 0.5, "Out of Range", ha='center')
                        continue
                    
                    mask = (t >= start_t) & (t <= real_end)
                    if np.sum(mask) > 10:
                        ax.plot(t[mask], t_0[mask], color='black', alpha=0.7)
                        if r_plot is not None:
                             # Slice mask to match r_plot length
                             r_len = len(r_plot)
                             r_mask = mask[:r_len]
                             if np.sum(r_mask) > 0:
                                 # We need t[:r_len] specifically? 
                                 # Actually t[mask] might be longer than r_plot[mask] if mask extends beyond r_len
                                 # So we must use t[:r_len][r_mask] and r_plot[r_mask]
                                 ax.plot(t[:r_len][r_mask], r_plot[r_mask], color='green', alpha=0.5)
                        
                        ax.plot(t[mask], p_0[mask], color='red', alpha=0.9, linestyle='--')
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

def analyze_automated(target_dir, filter_pattern=None):

    if not target_dir: target_dir = "experiments"
    root_path = Path(target_dir)
    
    if filter_pattern:
        print(f"[AUTO] Searching for folders matching '{filter_pattern}' in {root_path}")
        if ',' in filter_pattern:
            patterns = filter_pattern.split(',')
            exp_dirs = []
            for p in patterns:
                exp_dirs.extend(list(root_path.glob(p.strip())))
            # Deduplicate just in case
            exp_dirs = list(set(exp_dirs))
        else:
            exp_dirs = list(root_path.glob(filter_pattern))
    else:
        if (root_path / "config_dump.yaml").exists() or (root_path / "baseline.yaml").exists():
            exp_dirs = [root_path]
        else:
            print(f"[AUTO] Searching for all subfolders in {root_path}")
            exp_dirs = []
            for item in root_path.iterdir():
                if item.is_dir():
                    # 1. Check if direct child has model.pt
                    if (item / "model.pt").exists():
                        exp_dirs.append(item)
                    else:
                        # 2. Check if grandchildren have model.pt
                        for sub_item in item.iterdir():
                            if sub_item.is_dir() and (sub_item / "model.pt").exists():
                                exp_dirs.append(sub_item)

    valid_dirs = list(set(exp_dirs)) # Deduplicate
    
    if not valid_dirs:
        print("No valid experiments found.")
        return

    print(f"[AUTO] Found {len(valid_dirs)} experiments to analyze.")

    import time as _time
    _t0_total = _time.time()

    base_output_dir = Path("compare_result") / root_path.name
    base_output_dir.mkdir(exist_ok=True, parents=True)

    results = []

    for idx, exp_path in enumerate(valid_dirs, 1):
        _t0 = _time.time()
        fold_name = exp_path.name
        exp_id = exp_path.parent.name if fold_name.startswith("fold_") else fold_name
        fold_output_dir = base_output_dir / exp_id / fold_name
        fold_output_dir.mkdir(exist_ok=True, parents=True)

        print(f"\n[{idx}/{len(valid_dirs)}] Processing: {fold_name}")

        try:
            res = load_and_evaluate(exp_path, device, return_seqs=True, return_extras=True)
            if not res:
                continue

            results.append({
                "name": fold_name,
                "path": exp_path,
                "mae": res["mae"],
                "rmse": res["rmse"],
                "r2": res.get("r2", 0.0)
            })

            # 1. Training curves
            plot_training_curves(exp_path, fold_output_dir)

            # 2. Trajectory plot
            if "y_true_seq" in res and "y_pred_seq" in res:
                traj_path = fold_output_dir / "trajectory.png"
                plot_trajectory(
                    res["y_true_seq"], res["y_pred_seq"],
                    f"Trajectory: {exp_id} ({fold_name})",
                    traj_path, series_offsets=res.get("series_offsets")
                )
                print(f"  [SAVED] Trajectory: {traj_path}")

            # 3. Detailed 2x3 grid plots
            plot_detailed_condition_trajectories(exp_path, fold_output_dir, device)

            # 4. Feature Importance (reuse already-loaded model & data)
            fi_path = fold_output_dir / "feature_importance.png"
            if fi_path.exists():
                print(f"  -> Feature Importance already exists for {fold_name}. Skipping.")
            else:
                print(f"  -> Calculating Feature Importance for {fold_name}...")
                try:
                    model_fi = res.get("model")
                    cfg_fi = res.get("cfg", {})
                    c_in_vars_fi = res.get("c_in_vars", [])
                    X_list_fi = res.get("X_list")
                    Y_list_fi = res.get("Y_list")
                    window_size_fi = res.get("window_size", 200)
                    est_tick_fi = res.get("est_tick_ranges", [5])
                    scaler_mean_fi = res.get("scaler_mean")
                    scaler_scale_fi = res.get("scaler_scale")

                    if model_fi and X_list_fi:
                        # Apply scaler if not already applied (load_and_evaluate already scales X_list)
                        # But X_list is modified in place there, so it's already scaled.

                        f_names = get_feature_names(cfg_fi, c_in_vars_fi)

                        imp_ds = LazyWindowDataset(
                            X_list_fi, Y_list_fi,
                            window_size_fi, 1, 5,  # stride=5 for importance speed
                            target_mode="sequence", est_tick_ranges=est_tick_fi
                        )
                        imp_loader = DataLoader(imp_ds, batch_size=1024, shuffle=False, num_workers=0)

                        train_cfg = cfg_fi.get("02_train", {}).get("train", {})
                        loss_name = train_cfg.get("loss", "mse").lower()
                        huber_delta = float(train_cfg.get("huber_delta", 1.0))
                        if loss_name == "huber":
                            criterion = torch.nn.HuberLoss(delta=huber_delta, reduction='sum')
                        elif loss_name in ["mae", "l1"]:
                            criterion = torch.nn.L1Loss(reduction='sum')
                        else:
                            criterion = torch.nn.MSELoss(reduction='sum')

                        imps = calculate_permutation_importance(
                            model_fi, imp_loader, device, criterion
                        )
                        plot_feature_importance(
                            imps, f_names, fi_path,
                            title=f"Feature Importance ({loss_name}): {fold_name}"
                        )
                        print(f"  [SAVED] Feature Importance: {fi_path}")
                    else:
                        print(f"  [WARN] Missing model or data for importance. Skipping.")

                except Exception as e_imp:
                    print(f"  [WARN] Importance failed for {fold_name}: {e_imp}")
                    import traceback; traceback.print_exc()

            print(f"  [DONE] {fold_name} in {_time.time()-_t0:.1f}s")

        except Exception as e:
            print(f"  [ERROR] Failed to process {fold_name}: {e}")
            import traceback
            traceback.print_exc()

    _total = _time.time() - _t0_total
    print(f"\n[AUTO] {len(valid_dirs)} experiments processed in {_total:.0f}s ({_total/60:.1f}min)")



def plot_model_comparison(results):
    names = [r["name"] for r in results]
    maes = [r["mae"] for r in results]
    
    idx = np.argsort(maes)
    names = [names[i] for i in idx]
    maes = [maes[i] for i in idx]
    
    plt.figure(figsize=(10, 6 + len(results)*0.2))
    plt.barh(range(len(results)), maes, color='skyblue')
    plt.yticks(range(len(results)), names)
    plt.xlabel("Test MAE (deg) - Lower is Better")
    plt.title(f"Model Comparison (N={len(results)})")
    
    for i, v in enumerate(maes):
        plt.text(v, i, f" {v:.4f}", va='center')
        
    plt.tight_layout()
    os.makedirs("compare_result", exist_ok=True)
    plt.savefig("compare_result/model_comparison.png")
    print("Saved compare_result/model_comparison.png")




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
    import h5py
    from model_training import extract_condition_data_v2
    
    
    # 1. Load Model & Data
    # load_and_evaluate already has the correct Training-consistent logic
    res = load_and_evaluate(exp_path, device, return_seqs=True)
    if not res: return
    
    # [REFACTORED] Training과 동일한 config 해석 (load_and_evaluate와 동일한 경로)
    import re
    folder_name = exp_path.name
    exp_name_match = re.match(r"^(.+?)_Test-", folder_name)
    exp_name = exp_name_match.group(1) if exp_name_match else "baseline"
    
    original_config_path = find_config_by_exp_name(exp_name)
    if original_config_path and original_config_path.exists():
        with open(original_config_path, 'r') as f_cfg:
            cfg = normalize_experiment_config(yaml.safe_load(f_cfg))
    else:
        # Fallback to per-experiment config.yaml
        config_path = exp_path / "config.yaml"
        with open(config_path) as f_cfg: cfg = normalize_experiment_config(yaml.safe_load(f_cfg))
    
    # Config 파싱 - Training과 동일 (load_and_evaluate line 참조)
    curr_input_vars = get_config_input_vars(cfg)
    curr_output_vars = get_config_output_vars(cfg)
    
    def _parse_vars_local(var_list_from_yaml):
        if not var_list_from_yaml: return []
        return [(item[0], item[1]) for item in var_list_from_yaml]
    
    c_in_vars = _parse_vars_local(curr_input_vars)
    c_out_vars = _parse_vars_local(curr_output_vars)
    
    cfg['conditions'] = get_config_conditions(cfg)
    
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
    # lpf_order removed
    
    # [NEW] Multi-dataset path handling for condition check


    prefix_found, real_sub = parse_subject_prefix(test_sub)
    data_sources = get_data_sources_from_config(cfg)
    
    if prefix_found and prefix_found in data_sources:
        data_path = data_sources[prefix_found]['path']
    else:
        data_path = cfg.get("data_path", "combined_data.h5")
        if not os.path.exists(data_path): data_path = "combined_data.h5"
    
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
        condition_selection=CONDITION_SELECTION, 
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

    model = TCN_MLP(
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
    ).to(device)
    
    model_path = exp_path / "model.pt"
    ckpt = torch.load(model_path, map_location=device)
    model.load_state_dict(ckpt['state_dict'] if 'state_dict' in ckpt else ckpt, strict=False)
    model.eval()
    
    # ---------------------------------------------------------
    # Trajectory Visualization Loop (Targeting 2x3 Subplots)
    # ---------------------------------------------------------
    print("Running Trajectory Analysis (2x3 Grid per Condition)...")
    plot_dir = Path(f"compare_result/{folder_name}")
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
        ds = LazyWindowDataset([X_imp], [Y_imp], get_window_size(cfg, default=100), 10, 20, target_mode="sequence", est_tick_ranges=curr_est_tick_ranges)
        loader = DataLoader(ds, batch_size=2048, shuffle=False)
        importances = calculate_permutation_importance(model, loader, device, criterion)
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

def generate_feature_names(c_in_vars):
    names = []
    for gpath, vars in c_in_vars:
        if 'Back_imu' in gpath or 'back_imu' in gpath: prefix = "IMU"
        elif 'robot/left' in gpath: prefix = "Robot_L"
        elif 'robot/right' in gpath: prefix = "Robot_R"
        elif 'forceplate' in gpath: prefix = f"GRF_{'L' if 'left' in gpath else 'R'}"
        elif 'kin_q' in gpath: prefix = "Kin"
        else: prefix = gpath.split('/')[-1]
        for v in vars: names.append(f"{prefix}_{v}")
    return names

def calculate_permutation_importance(model, loader, device, criterion, batch_size=2048):
    """
    Calculate Permutation Feature Importance with Batched Inference to Prevent OOM.
    Returns: np.array of importance scores (Loss increase)
    """
    model.eval()
    
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
            pred = model(xb)
            
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
                    pred = model(xb)
                    
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

def parse_vars(var_list):
    # Helper to ensure [ (grp, [vars]) ] structure
    # If list of [grp, [vars]], return as is
    if not var_list: return []
    if isinstance(var_list[0], (list, tuple)) and len(var_list[0]) == 2 and isinstance(var_list[0][1], list):
        return var_list
    # Handle flat list if necessary? Default to assume correct structure from config
    return var_list


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

def analyze_ablation_report():
    print("Starting Ablation Analysis Report...")
    # device is global in this script
    
    # Check if compare_result exists
    output_dir = Path("compare_result")
    output_dir.mkdir(exist_ok=True)
        
    # 1. Find all experiments
    exp_dirs = list_experiments("experiments")
    if not exp_dirs:
        print("No experiments found in 'experiments/' directory.")
        return

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
    plt.xlabel("MAE (deg) [Lower is Better]")
    plt.legend()
    plt.tight_layout()
    plt.savefig(output_dir / "final_comparison_mae.png")
    
    # Plot 2: Delta MAE (Diverging Bar Chart)
    plt.figure(figsize=(12, max(6, len(group_df)*0.4)))
    
    colors = ['green' if x < 0 else 'red' for x in group_df['mae_delta']]
    
    sns.barplot(data=group_df, x="mae_delta", y="group", hue="group", dodge=False, palette=colors, legend=False)
    plt.axvline(0, color='black', linewidth=1)
    plt.title(f"Ablation Impact (Relative to Baseline MAE: {base_mae:.4f})")
    plt.xlabel("Change in MAE (deg) [Positive = Worse, Negative = Better]")
    plt.tight_layout()
    plt.savefig(output_dir / "final_ablation_delta.png")
    
    print("Saved plots to compare_result/")

if __name__ == "__main__":
    args = get_args()
    if args.ablation:
        analyze_ablation_report()
    elif args.auto:
        analyze_automated(args.target_dir, args.filter)
        # [NEW] Automatically run ablation report after auto-analysis
        analyze_ablation_report()
    else:
        main()
