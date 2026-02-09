
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

# -----------------------------------------------------------------------------
# Argument Parsing
# -----------------------------------------------------------------------------
def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--auto', action='store_true', help='Run in automated mode (no interaction)')
    parser.add_argument('--filter', type=str, default=None, help='Filter pattern for directories (e.g. "baseline_*")')
    parser.add_argument('target_dir', nargs='?', help='Target experiment directory to analyze (optional)')
    return parser.parse_args()

# -----------------------------------------------------------------------------
# Settings
# -----------------------------------------------------------------------------

# Import Model and Dataset from main script
# Ensure the directory is in path
sys.path.append(str(Path(__file__).parent))
from SpeedEstimator_TCN_MLP_experiments import (
    TCN_MLP, StanceGatedTCN, AttentionTCN, LazyWindowDataset, build_nn_dataset,
    make_subject_selection, CONDITION_SELECTION
)
import SpeedEstimator_TCN_MLP_experiments as main_script

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

def list_experiments(exp_root="experiments"):
    root = Path(exp_root)
    if not root.exists():
        print(f"Directory {root} does not exist.")
        return []
    
    # List all subdirectories containing model.pt
    exp_dirs = [d for d in root.iterdir() if d.is_dir() and (d / "model.pt").exists()]
    exp_dirs.sort(key=lambda x: x.name)
    return exp_dirs

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

def calculate_lag(y_true, y_pred, fs=100):
    """
    Calculate temporal lag (phase shift) using Cross-Correlation.
    Positive lag => Prediction is DELAYED (behind truth).
    """
    correlation = signal.correlate(y_true - np.mean(y_true), y_pred - np.mean(y_pred), mode="full")
    lags = signal.correlation_lags(len(y_true), len(y_pred), mode="full")
    lag_idx = np.argmax(correlation)
    lag_samples = lags[lag_idx]
    
    # lag_samples < 0 means y_pred is ahead?
    # correct: if y_pred is shifted right (delayed), peak is at negative lag? checking...
    # signal.correlate(in1, in2): 
    # If in2 is shifted right by k relative to in1, the peak is at index ...
    # Easier: Time difference. 
    # Let's say y_true is at t=0. y_pred is at t=10 (delayed).
    # correlate(y_true, y_pred) peak will be at negative lag.
    # So Lag (ms) = -lag_samples * (1000/fs)
    
    lag_ms = -lag_samples * (1000.0 / fs)
    return lag_ms

def calculate_smoothness(y_pred):
    """
    Smoothness metric: Std Dev of the first derivative (Jitter).
    Lower is smoother.
    """
    diff = np.diff(y_pred, axis=0)
    jitter = np.std(diff)
    return jitter

def load_and_evaluate(exp_dir, device, return_seqs=False):
    """
    Loads model and config, rebuilds test dataset, runs inference.
    Returns metrics and time-series.
    """
    exp_path = Path(exp_dir)
    config_path = exp_path / "config.yaml"
    model_path = exp_path / "model.pt"
    
    if not config_path.exists() or not model_path.exists():
        print(f"Skipping {exp_dir.name}: missing config or model.")
        return None

    # Load Config
    with open(config_path, 'r') as f:
        import yaml
        config = yaml.safe_load(f)
        
    cfg = config
    
    # [FIX] Enforce y_delay -> est_tick_ranges logic if missing
    # [FIX] Enforce y_delay -> est_tick_ranges logic if missing
    # Commented out because we want to allow Sequence Mode fallback (time_window_output)
    if "est_tick_ranges" not in cfg:
         pass
         # # Check defaults
         # base_y_delay = 5 # Default
         # if "shared" in cfg and "data" in cfg["shared"]:
         #     base_y_delay = cfg["shared"]["data"].get("y_delay", 5)
         # elif "y_delay" in cfg:
         #     base_y_delay = cfg["y_delay"]
         # elif "01_construction" in cfg: # Check deeply
         #     # Try to infer...
         #     pass
         #     
         # # If still missing, default to [5]
         # cfg["est_tick_ranges"] = [base_y_delay]
         # print(f"[INFO] Derived est_tick_ranges: {cfg['est_tick_ranges']}")
    
    # [FIX] Handle Nested Config Mapping (Pre-computation)
    if '01_construction' in cfg and 'input_vars' not in cfg:
        cfg['input_vars'] = cfg['01_construction']['inputs']
        cfg['output_vars'] = cfg['01_construction']['outputs']

    # Parse Vars (Helper from main script logic needed, but simpler to repeat or assume correct)
    # The config has 'input_vars' and 'output_vars' as lists.
    input_vars = cfg.get("input_vars", [])
    output_vars = cfg.get("output_vars", [])
    # In config, it is already stored as list of lists.
    
    # Rebuild Test Dataset
    # We need to know WHICH subject was the test subject for this specific fold.
    # The folder name usually contains "Test-SXXX".
    folder_name = exp_path.name
    import re
    match = re.search(r"Test-(S\d+)", folder_name)
    if match:
        test_sub = match.group(1)
    else:
        print(f"Could not infer test subject from {folder_name}. Skipping.")
        return None
        
    # Data params
    # Load base config to get default data path
    base_config_path = Path("configs/base_config.yaml") 
    if base_config_path.exists():
        with open(base_config_path, 'r') as f:
            import yaml
            base_config = yaml.safe_load(f)
            data_path = Path(base_config.get("data_path", "combined_data.h5")).resolve()
    else:
        # Fallback
        data_path = Path("combined_data.h5").resolve()
        
    if not data_path.exists():
         # Try relative to main script location
         data_path = Path("combined_data.h5")
         
    # We only need the TEST data
    # 1. Build Dataset (Test Only) -> fast
    print(f"Loading Test Data for {folder_name} (Subject {test_sub})...")
    
    # Must use same windowing params as training
    # Smart Extraction
    train_cfg = cfg.get("02_train", {})
    data_cfg = train_cfg.get("data", {})
    if not data_cfg and "shared" in cfg and "data" in cfg["shared"]: # Fallback to anchor
        data_cfg = cfg["shared"]["data"]
    if not data_cfg: data_cfg = cfg.get("data", {}) # Fallback to global data

    win_in = data_cfg.get("time_window_input") or cfg.get("time_window_input", 100)
    win_out = data_cfg.get("time_window_output") or cfg.get("time_window_output", 10)
    stride = data_cfg.get("stride") or cfg.get("stride", 20)
    
    stride_eval = 1 
    
    # Custom Horizon
    est_tick_ranges = data_cfg.get("est_tick_ranges") or cfg.get("est_tick_ranges", None)
    
    # [FIX] Heuristic: If win_out > 1, we assume Sequence Mode even if ranges is polluted
    if win_out > 1:
        print(f"[AUTO-FIX] win_out={win_out} > 1, forcing est_tick_ranges=None (Sequence Mode)")
        est_tick_ranges = None
    
    input_vars = cfg["input_vars"]
    output_vars = cfg["output_vars"]
    
    # We use build_nn_dataset directly
    lpf_c = cfg.get("lpf_cutoff")
    if lpf_c is None and "01_construction" in cfg:
        lpf_c = cfg["01_construction"].get("lpf_cutoff")
    lpf_c = lpf_c if lpf_c is not None else 0.5
    
    lpf_o = cfg.get("lpf_order")
    if lpf_o is None and "01_construction" in cfg:
        lpf_o = cfg["01_construction"].get("lpf_order")
    lpf_o = lpf_o if lpf_o is not None else 5
    
    # [NEW] Extract Data Flags
    data_cfg_now = cfg.get("02_train", {}).get("data", {})
    use_phys_vel = data_cfg_now.get("use_physical_velocity_model", False)
    use_gait_ph  = data_cfg_now.get("use_gait_phase", False)
    
    X_list, Y_list = build_nn_dataset(
        str(data_path), 
        [test_sub], # Only test subject
        cfg["conditions"], # Use all conditions trained on?
        input_vars, output_vars,
        win_in, win_out, stride_eval, # Correct signature: win_in, win_out, stride
        condition_selection=CONDITION_SELECTION,
        lpf_cutoff=lpf_c,
        lpf_order=lpf_o,
        est_tick_ranges=est_tick_ranges,
        use_physical_velocity_model=use_phys_vel,
        use_gait_phase=use_gait_ph
    )
    
    if not X_list:
        print("No test data found.")
        return None
        
    ds = LazyWindowDataset(X_list, Y_list, win_in, win_out, stride_eval, target_mode="sequence", est_tick_ranges=est_tick_ranges)
    loader = DataLoader(ds, batch_size=1024, shuffle=False, num_workers=0)
    
    # Load Model
    # Need to infer input_dim/output_dim from data
    input_dim = X_list[0].shape[1]
    output_dim = Y_list[0].shape[1]
    
    if est_tick_ranges:
        horizon = len(est_tick_ranges)
    else:
        horizon = win_out
    
    print(f"[DEBUG-EVAL] CFG Keys: {list(cfg.keys())}")
    if "02_train" in cfg:
        print(f"[DEBUG-EVAL] 02_train keys: {list(cfg['02_train'].keys())}")
        if cfg["02_train"].get("model"):
             print(f"[DEBUG-EVAL] 02_train.model keys: {list(cfg['02_train']['model'].keys())}")

    model_cfg = cfg.get("02_train", {}).get("model")
    if not model_cfg:
        # Fallback to data if present
        data_cfg = cfg.get("02_train", {}).get("data")
        if data_cfg and "channels" in data_cfg: model_cfg = data_cfg
    if not model_cfg: model_cfg = cfg.get("model") or {}
    
    tcn_ch = model_cfg.get("channels") or cfg.get("tcn_channels", (64, 64, 128))
    k_size = model_cfg.get("kernel_size") or cfg.get("kernel_size", 3)
    dropout = float(model_cfg.get("dropout") or cfg.get("dropout_p", 0.1))
    head_dropout = model_cfg.get("head_dropout")
    norm_type = model_cfg.get("model_norm")
    
    hidden = model_cfg.get("head_hidden") or cfg.get("mlp_hidden", 128)
    
    print(f"[DEBUG-VALS] k_size={k_size}, hidden={hidden}, tcn_ch={tcn_ch}")
    print(f"[DEBUG-VALS] model_cfg kernel: {model_cfg.get('kernel_size')}")

    # Model Instantiation
    model_type = model_cfg.get("type", "TCN")
    print(f"[INFO] Initializing Model: {model_type}")
    
    common_args = dict(
        input_dim=input_dim, 
        output_dim=output_dim, 
        horizon=horizon,
        channels=tcn_ch,
        kernel_size=k_size,
        dropout=dropout,
        head_dropout=head_dropout,
        mlp_hidden=hidden, 
        use_input_norm=cfg.get("use_input_norm", True),
        tcn_norm=norm_type,
        mlp_norm=norm_type
    )

    if model_type == "StanceGatedTCN":
        gating_dim = model_cfg.get("gating_dim", 1) 
        gating_signal = model_cfg.get("gating_signal", "contact")
        str_mapping = {'contact': 1} # Placeholder
        model = StanceGatedTCN(**common_args, gating_dim=gating_dim)
        
    elif model_type == "AttentionTCN":
        attn_type = model_cfg.get("attention_type", "temporal")
        heads = model_cfg.get("attention_heads", 4)
        model = AttentionTCN(**common_args, attention_type=attn_type, attention_heads=heads)
        
    else:
        model = TCN_MLP(**common_args)
    
    model.to(device)
    
    # Load Weights
    ckpt = torch.load(model_path, map_location=device)
    # Handle strictly=False logic same as training
    model.load_state_dict(ckpt["state_dict"] if "state_dict" in ckpt else ckpt, strict=False)
    model.eval()
    
    # Inference
    y_preds = []
    y_trues = []
    
    with torch.no_grad():
        for xb, yb in loader:
            xb = xb.to(device)
            out = model(xb) # (B, T, D)
            y_preds.append(out.cpu().numpy())
            y_trues.append(yb.cpu().numpy())
            
    y_pred = np.concatenate(y_preds, axis=0) # (N, T, D)
    y_true = np.concatenate(y_trues, axis=0)
    
    print(f"[DEBUG] y_pred shape: {y_pred.shape}, y_true shape: {y_true.shape}")
    
    
    # Flatten for overall metrics
    # CRITICAL FIX: Align evaluation to 50ms (Tick 5, Index 4) for fair comparison.
    # Baseline targets 50ms (est_tick_ranges=[5]).
    # Sequence models (Horizon 5, 10, 20) predict [1..H] steps ahead.
    # We want index 4 (5th step) from Sequence models to compare with Baseline.
    # If using [-1], Horizon 20 is evaluated at 200ms, which is unfair.
    
    if y_pred.ndim == 3: # (N, H, D)
        H = y_pred.shape[1]
        
        # Determine index
        # If est_tick_ranges exists, we usually respect it.
        # But here we want to standardize.
        
        # If Sequence Mode (ranges is None)
        if est_tick_ranges is None:
            # We want 5th step (Index 4)
            # If H < 5, use last.
            target_idx = min(4, H-1)
            print(f"[DEBUG] Using Sequence Index {target_idx} (Step {target_idx+1}) for Evaluation")
            vp = y_pred[:, target_idx, 0]
            vt = y_true[:, target_idx, 0]
        else:
            # Range Mode (Baseline uses [5])
            # It only has 1 output usually, so index 0.
            # Assuming est_tick_ranges=[5], then y_pred is (N, 1, 1) or (N, 1) if reshaped?
            # TCN_MLP reshaped to (N, H, D). So (N, 1, 1).
            vp = y_pred[:, 0, 0]
            vt = y_true[:, 0, 0]
    else:
        # Assume (N, 1) or (N,)
        vp = y_pred.squeeze()
        vt = y_true.squeeze()
        
        
    metrics_horizon = 50 # ms (Label purpose)
    
    # Calculate Metrics
    mae = np.mean(np.abs(vp - vt))
    rmse = np.sqrt(np.mean((vp - vt)**2))
    
    # 10-Tick Metrics
    # Lag (ms) - assuming 100Hz -> 10ms per tick
    lag_ms = calculate_lag(vt, vp, fs=100)
    
    # Smoothness (Jitter)
    smoothness = calculate_smoothness(vp)
    
    # Return compact dictionary to save memory
    ret = {
        "name": folder_name,
        "mae": mae,
        "rmse": rmse,
        "lag_ms": lag_ms,
        "smoothness": smoothness,
        "rmse": rmse,
        "lag_ms": lag_ms,
        "smoothness": smoothness,
        "folder_path": exp_dir, # Keep path to reload later
        "est_tick_ranges": est_tick_ranges
    }
    
    if return_seqs:
        ret["y_true_seq"] = y_true # (N, Win_Out, 1)
        ret["y_pred_seq"] = y_pred # (N, Win_Out, 1)
        
    return ret

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
        cfg = yaml.safe_load(f)

    if "01_construction" in cfg:
        input_vars = cfg.get("inputs") or cfg["01_construction"]["inputs"]
        out_groups = cfg.get("outputs") or cfg["01_construction"]["outputs"]
        output_vars_flat = []
        for item in out_groups:
            if isinstance(item, list) and len(item) == 2:
                gpath, vars = item
                output_vars_flat.extend([f"{gpath}/{v}" for v in vars])
            else:
                output_vars_flat.append(item)
        data_path = cfg.get("data_path") or cfg["01_construction"].get("src_h5") or cfg["shared"].get("src_h5")
        window = cfg.get("time_window_input") or cfg.get("shared", {}).get("data", {}).get("window_size", 100)
        stride = cfg.get("stride") or cfg.get("shared", {}).get("data", {}).get("stride", 10)
        est_tick_ranges = cfg.get("est_tick_ranges")
    else:
        input_vars = cfg["input_vars"]
        out_groups = cfg["output_vars"]
        output_vars_flat = out_groups # Assume flat if old? Or grouped? Old format was flat strings usually.
        # But extract_v2 expects grouped. Old code used extract_v1?
        # Let's assume input_vars/output_vars are correct structure for the extractor used.
        # If old format uses extract_v2, it must be grouped.
        data_path = cfg["data_path"]
        window = cfg["time_window_input"]
        stride = cfg.get("stride", 1)
        est_tick_ranges = cfg.get("est_tick_ranges", None)

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
                
                dl = DataLoader(ds, batch_size=1024, shuffle=False)
                
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

                plt.figure(figsize=(10, 4))
                plt.plot(targets[:1000].squeeze(), label='GT', color='black', alpha=0.6)
                plt.plot(preds[:1000].squeeze(), label='Pred', color='red', alpha=0.7)
                plt.title(f"[{subj}] {cond} | MAE={mae:.4f}")
                plt.legend()
                plt.tight_layout()
                plt.savefig(os.path.join(save_dir, f"{subj}_{cond}.png"))
                plt.close()
                print(f"    -> {subj}/{cond}: MAE={mae:.4f}")
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

def plot_trajectory(y_true, y_pred, title, save_path):
    """
    Plots trajectory comparison with Full and Zoomed views.
    y_true, y_pred: (T, 1) or (T,) or (T)
    """
    # Ensure 1D
    y_true = np.squeeze(y_true)
    y_pred = np.squeeze(y_pred)
    
    fs = 100.0
    t = np.arange(len(y_true)) / fs
    
    fig, axes = plt.subplots(2, 1, figsize=(12, 10))
    
    # 1. Full Duration
    axes[0].plot(t, y_true, label='Ground Truth (Target)', alpha=0.7, color='black')
    axes[0].plot(t, y_pred, label='Prediction', alpha=0.9, color='red', linestyle='--')
    axes[0].set_title(f"{title} - Full Duration")
    axes[0].set_xlabel("Time (s)")
    axes[0].set_ylabel("Speed (m/s)")
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)
    
    # 2. Zoomed (15s Window, e.g. 10s-25s)
    start_t, end_t = 10.0, 25.0
    # Adjust if total duration is short
    if t[-1] < 15.0:
        start_t, end_t = 0, t[-1]
    elif t[-1] < 25.0:
        start_t = 0
        end_t = 15.0
        
    mask = (t >= start_t) & (t <= end_t)
    if np.sum(mask) > 0:
        axes[1].plot(t[mask], y_true[mask], label='Ground Truth', color='black', alpha=0.7)
        axes[1].plot(t[mask], y_pred[mask], label='Prediction', color='red', linestyle='--', alpha=0.9)
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
            exp_dirs = [d for d in root_path.iterdir() if d.is_dir()]

    valid_dirs = []
    for d in exp_dirs:
        if (d / "model.pt").exists():
            valid_dirs.append(d)
    
    if not valid_dirs:
        print("No valid experiments found.")
        return

    print(f"[AUTO] Found {len(valid_dirs)} experiments to analyze.")
    
    # 1. Collect Metrics for ALL
    results = []
    for exp_path in valid_dirs:
        try:
            res = load_and_evaluate(exp_path, device, return_seqs=False)
            if res:
                results.append({
                    "name": exp_path.name,
                    "path": exp_path,
                    "mae": res["mae"],
                    "rmse": res["rmse"],
                    "r2": res.get("r2", 0.0)
                })
        except Exception as e:
            print(f"[ERROR] Failed to evaluate {exp_path.name}: {e}")
            import traceback
            traceback.print_exc()
            
    if not results: return
    
    # 2. Compare Plot (ALL)
    plot_model_comparison(results)
    
    # 3. Best Model Detailed Analysis
    best = sorted(results, key=lambda x: x["mae"])[0]
    print(f"\n[RESULT] Best Model: {best['name']} (MAE={best['mae']:.4f})")
    
    analyze_detailed_single(best["path"])
    
def plot_model_comparison(results):
    names = [r["name"] for r in results]
    maes = [r["mae"] for r in results]
    
    idx = np.argsort(maes)
    names = [names[i] for i in idx]
    maes = [maes[i] for i in idx]
    
    plt.figure(figsize=(10, 6 + len(results)*0.2))
    plt.barh(range(len(results)), maes, color='skyblue')
    plt.yticks(range(len(results)), names)
    plt.xlabel("Test MAE (m/s) - Lower is Better")
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
    
    # 1. Load Model & Data
    # We need access to the model object, which load_and_evaluate doesn't return (it returns metrics).
    # So we copy the loading logic or modify load_and_evaluate.
    # Let's verify load_and_evaluate returns. It returns dict.
    
    # Quick fix: Load everything manually here to be safe and clean.
    res = load_and_evaluate(exp_path, device, return_seqs=True)
    if not res: return
    
    # Load Model (Again, sorry for duplication but safer)
    model_path = exp_path / "model.pt"
    config_path = exp_path / "config.yaml"
    with open(config_path) as f: cfg = yaml.safe_load(f)
    
    # [FIX] Enforce y_delay
    # Commented out to allow Sequence Mode fallback
    if "est_tick_ranges" not in cfg:
        pass
        # cfg["est_tick_ranges"] = [cfg.get("y_delay", 5)]
        
    # Build Dataset
    # We need the TEST loader
    # Parse Names
    inputs = cfg.get("input_vars") or cfg["01_construction"]["inputs"]
    outputs = cfg.get("output_vars") or cfg["01_construction"]["outputs"] # Fixed key
    c_in_vars = parse_vars(inputs)
    c_out_vars = parse_vars(outputs)
    
    folder_name = exp_path.name
    import re
    match = re.search(r"Test-(S\d+)", folder_name)
    test_sub = match.group(1) if match else "S001"
    
    # Data Path
    data_path = cfg.get("data_path", "combined_data.h5")
    if not os.path.exists(data_path): data_path = "combined_data.h5"
    
    # [LOGIC] Input LPF is disabled in code, Output LPF is active.
    # We must pass the config's lpf_cutoff to filter the targets during evaluation.
    eval_lpf = cfg.get("lpf_cutoff")
    if eval_lpf is None and "01_construction" in cfg:
        eval_lpf = cfg["01_construction"].get("lpf_cutoff")
    if eval_lpf is None: eval_lpf = 0.5
    
    # 2. Build Dataset (Raw first)
    X, Y = build_nn_dataset(
        data_path, [test_sub], cfg["conditions"], c_in_vars, c_out_vars, 
        cfg.get("time_window_input", 100), 10, cfg.get("stride", 20),
        condition_selection=CONDITION_SELECTION, lpf_cutoff=eval_lpf, est_tick_ranges=cfg.get("est_tick_ranges", None)
    )
    
    # 3. Apply Scaler (Crucial for correct inference)
    # Check for scaler.npz in likely locations
    scaler_path = exp_path / "scaler.npz"
    if not scaler_path.exists():
        # Check if config points to one
        if "scaler_path" in cfg:
            p = Path(cfg["scaler_path"])
            if p.exists(): scaler_path = p
        # Check inside experiment "runs_tcn" or similar if needed, but usually strictly saved in exp dir now?
        # If not found, check parent?
    
    if scaler_path.exists():
        print(f"[INFO] Loading scaler from {scaler_path}")
        scaler = np.load(scaler_path)
        mean = scaler['mean']
        scale = scaler['scale']
        
        # Apply normalization: (X - mean) / scale
        # X is list of arrays (T, D)
        # mean/scale are (D,)
        print("[INFO] Applying scaler to test data...")
        for i in range(len(X)):
            X[i] = (X[i] - mean) / (scale + 1e-8) # Avoid div/0
    else:
        print("[WARN] No scaler found! Inference might be garbage if model expects scaled data.")

    input_dim = X[0].shape[1]
    
    train_sec = cfg.get("02_train", {})
    model_cfg = train_sec.get("model")
    if not model_cfg:
        # Fallback to data if present (common in baseline.yaml)
        data_cfg = train_sec.get("data")
        if data_cfg and "channels" in data_cfg: 
            model_cfg = data_cfg
    if not model_cfg: 
        model_cfg = cfg.get("model", {})
    tcn_ch = model_cfg.get("channels") or cfg.get("tcn_channels", (64, 64, 128))
    k_size = model_cfg.get("kernel_size") or cfg.get("kernel_size", 3)
    dropout = float(model_cfg.get("dropout") or cfg.get("dropout_p", 0.1))
    head_dropout = model_cfg.get("head_dropout")
    norm_type = model_cfg.get("model_norm")
    hidden = model_cfg.get("head_hidden") or cfg.get("mlp_hidden", 128)

    model = TCN_MLP(
        input_dim=input_dim,
        output_dim=Y[0].shape[1],
        horizon=len(cfg["est_tick_ranges"]) if cfg.get("est_tick_ranges") else 10,
        channels=tcn_ch,
        kernel_size=k_size,
        dropout=dropout,
        head_dropout=head_dropout,
        mlp_hidden=hidden,
        use_input_norm=cfg.get("use_input_norm", True),
        tcn_norm="layer", mlp_norm="layer"
    ).to(device)
    
    ckpt = torch.load(model_path, map_location=device)
    model.load_state_dict(ckpt['state_dict'], strict=False)
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
    
    test_conditions = cfg["conditions"]
    
    # Target Levels for Columns 0, 1, 2
    target_levels_map = {'lv0': 0, 'lv4': 1, 'lv7': 2}
    
    # We need manual file handle since build_nn_dataset doesn't expose `extract` function's file handle
    # But extract_condition_data_v2 usually takes an open H5 handle 'f'
    # We can perform the extraction loop here.
    
    import h5py
    from SpeedEstimator_TCN_MLP_experiments import extract_condition_data_v2
    
    with h5py.File(data_path, 'r') as f:
        for cond in test_conditions:
            # Prepare Figure: 4 Rows (Full, Zoom 0-15, Zoom 15-30, Zoom 30-45), 3 Cols (lv0, lv4, lv7)
            fig, axes = plt.subplots(4, 3, figsize=(18, 16))
            
            # Check availability
            if cond not in f[test_sub]:
                plt.close()
                continue
                
            has_data = False
            
            for lv_name, col_idx in target_levels_map.items():
                # Extract Single Trial (trial_01)
                try:
                    X_list, Y_list = extract_condition_data_v2(
                        f, test_sub, cond, c_in_vars, c_out_vars, 
                        lpf_cutoff=eval_lpf, lpf_order=5, fs=100,
                        include_levels=[lv_name], include_trials=['trial_01'] # Force Single Trial
                    )
                except Exception as e:
                    # Level might not exist
                    X_list = []
                    
                if not X_list:
                    # Hide axes if no data
                    for r in range(4):
                        axes[r, col_idx].axis('off')
                    continue
                
                has_data = True
                X_arr = X_list[0]
                Y_arr = Y_list[0] # Target (Filtered per extract config)
                
                # Normalize X
                if scaler_path.exists():
                     X_arr = (X_arr - mean) / (scale + 1e-8)
                
                # Run sliding window / batch
                ds_vis = LazyWindowDataset([X_arr], [Y_arr], cfg.get("time_window_input", 100), 10, 1, target_mode="sequence", est_tick_ranges=cfg.get("est_tick_ranges", None))
                loader_vis = DataLoader(ds_vis, batch_size=512, shuffle=False)
                
                preds, targets = [], []
                with torch.no_grad():
                     for xb, yb in loader_vis:
                         xb = xb.to(device)
                         out = model(xb)
                         preds.append(out.cpu().numpy())
                         targets.append(yb.cpu().numpy())
                
                if not preds: continue
                
                P_all = np.concatenate(preds, axis=0)
                T_all = np.concatenate(targets, axis=0)
                
                if P_all.ndim == 3:
                     p_seq = P_all[:, 0, 0]
                     t_seq = T_all[:, 0, 0]
                else:
                     p_seq = P_all.squeeze()
                     t_seq = T_all.squeeze()
                
                # Plot
                fs = 100.0
                t = np.arange(len(t_seq)) / fs
                
                # Row 0: Full
                ax0 = axes[0, col_idx]
                ax0.plot(t, t_seq, color='black', alpha=0.7, label='GT')
                ax0.plot(t, p_seq, color='red', alpha=0.9, linestyle='--', label='Pred')
                ax0.set_title(f"{lv_name} (Full)")
                ax0.set_xlabel("Time (s)")
                ax0.set_ylabel("Speed (m/s)")
                ax0.grid(True, alpha=0.3)
                if col_idx == 0: ax0.legend()
                
                # Rows 1-3: Zooms
                windows = [(0, 15), (15, 30), (30, 45)]
                for r, (start_t, end_t) in enumerate(windows, start=1):
                    ax = axes[r, col_idx]
                    
                    # Adjust end if too short
                    real_end = min(end_t, t[-1])
                    if start_t >= t[-1]:
                        ax.text(0.5, 0.5, "Out of Range", ha='center')
                        continue

                    mask = (t >= start_t) & (t <= real_end)
                    if np.sum(mask) > 10: # Min samples
                        ax.plot(t[mask], t_seq[mask], color='black', alpha=0.7)
                        ax.plot(t[mask], p_seq[mask], color='red', alpha=0.9, linestyle='--')
                        ax.set_title(f"{lv_name} Zoom ({start_t}-{end_t}s)")
                        ax.set_xlabel("Time (s)")
                        ax.set_ylabel("Speed (m/s)")
                        ax.grid(True, alpha=0.3)
                    else:
                        ax.text(0.5, 0.5, "Too Short / No Data", ha='center')
                    
            if has_data:
                plt.tight_layout()
                safe_cond = cond.replace('/', '_')
                fname = plot_dir / f"{test_sub}_{safe_cond}.png"
                plt.savefig(fname)
                print(f"Saved {fname}")
            plt.close()
    
    # ---------------------------------------------------------
    feature_names = generate_feature_names(c_in_vars)
    if len(feature_names) != input_dim:
        feature_names = [f"F{i}" for i in range(input_dim)]
        
    # Run Importance
    print("Running Permutation Importance...")
    ds = LazyWindowDataset(X, Y, cfg.get("time_window_input", 100), 10, 20, target_mode="sequence", est_tick_ranges=cfg["est_tick_ranges"])
    loader = DataLoader(ds, batch_size=2048, shuffle=False)
    
    importances = calculate_permutation_importance(model, loader, device)
    
    # Plot Importance
    indices = np.argsort(importances)[::-1][:20]
    
    plt.figure(figsize=(12, 6))
    plt.bar(range(len(indices)), importances[indices])
    plt.xticks(range(len(indices)), [feature_names[i] for i in indices], rotation=45, ha='right')
    plt.title(f"Feature Importance ({folder_name})")
    plt.tight_layout()
    plt.savefig(f"compare_result/{folder_name}_importance.png")
    print(f"Saved compare_result/{folder_name}_importance.png")

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

def calculate_permutation_importance(model, loader, device):
    X_batches, Y_batches = [], []
    for xb, yb in loader:
        X_batches.append(xb)
        Y_batches.append(yb)
    X = torch.cat(X_batches).to(device)
    Y = torch.cat(Y_batches).to(device)
    
    criterion = torch.nn.MSELoss()
    with torch.no_grad():
        base = criterion(model(X), Y).item()
    
    imps = []
    for i in range(X.shape[2]):
        orig = X[:, :, i].clone()
        idx = torch.randperm(X.size(0), device=device)
        X[:, :, i] = X[idx, :, i]
        with torch.no_grad():
            new_loss = criterion(model(X), Y).item()
        imps.append(new_loss - base)
        X[:, :, i] = orig # Restore
        print(f"Feat {i}: {imps[-1]:.6f}", end='\r')
    print("")
    return np.array(imps)

def parse_vars(var_list):
    # Helper to ensure [ (grp, [vars]) ] structure
    # If list of [grp, [vars]], return as is
    if not var_list: return []
    if isinstance(var_list[0], (list, tuple)) and len(var_list[0]) == 2 and isinstance(var_list[0][1], list):
        return var_list
    # Handle flat list if necessary? Default to assume correct structure from config
    return var_list

if __name__ == "__main__":
    args = get_args()
    if args.auto:
        analyze_automated(args.target_dir, filter_pattern=args.filter)
    else:
        main()
