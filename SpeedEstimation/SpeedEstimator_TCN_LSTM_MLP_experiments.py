
import os
os.environ["OMP_NUM_THREADS"] = "1"
os.environ["MKL_NUM_THREADS"] = "1"
os.environ["NUMEXPR_NUM_THREADS"] = "1"
os.environ["MKL_THREADING_LAYER"] = "GNU"
os.environ["KMP_DUPLICATE_LIB_OK"] = "TRUE"

import torch
torch.set_num_threads(1)
torch.set_num_interop_threads(1)

import torch.nn as nn
from torch.utils.data import Dataset, DataLoader
import numpy as np
import random
import h5py
import matplotlib.pyplot as plt
import time
from scipy.signal import butter, filtfilt
import yaml
import json
import shutil
import glob
from pathlib import Path
import sys
import platform
import datetime

# data_length removed as requested - using full dataset length dynamically

# Preprocessing Config
fs = 100
# Global config defaults (Constants)
time_window_output = 10
stride = 10

# input_vars, output_vars defined in YAML/base_config now.
# TRAIN_SUBJECTS, VAL_SUBJECTS, TEST_SUBJECTS defined in YAML/base_config now.

def make_subject_selection(include_list):
    return {'include': include_list, 'exclude': []}

CONDITION_SELECTION = {'include': None, 'exclude': []}
# data_length = 12000 # Removed as requested

# -------------------------------------------------------------------------------------------------
# Helper Functions (Dataset, Plotting, etc) -> Copy from original
# -------------------------------------------------------------------------------------------------

def plot_filtering_check(data_path, sub, cond, output_vars, lpf_cutoff, fs=100):
    """
    Visualizes the effect of LPF on the reference velocity for a single trial.
    Blocks execution until closed.
    """
    print(f"\n[INFO] Visualizing Filtering Effect for {sub}/{cond}...")
    with h5py.File(data_path, 'r') as f:
        # Just grab the first trial found
        if sub not in f or cond not in f[sub]:
            print("  -> Sample not found.")
            return

def plot_filtering_check(data_path, sub, cond, output_vars, lpf_cutoff, fs=100):
    """
    Visualizes the effect of LPF on the reference velocity for a single trial.
    Blocks execution until closed.
    Includes Button to Scale Up (5s window) and Slider to Scroll.
    """
    from matplotlib.widgets import Slider, Button
    
    print(f"\n[INFO] Visualizing Filtering Effect for {sub}/{cond}...")
    with h5py.File(data_path, 'r') as f:
        # Just grab the first trial found
        if sub not in f or cond not in f[sub]:
            print("  -> Sample not found.")
            return

        cond_grp = f[sub][cond]
        lv_key = list(cond_grp.keys())[0]
        trial_key = list(cond_grp[lv_key].keys())[0]
        trial_grp = cond_grp[lv_key][trial_key]
        
        raw_vals = {}
        filtered_vals = {}
        
        for gpath, vars in output_vars:
            # Navigate
            g = trial_grp
            parts = gpath.split('/')
            valid = True
            for p in parts:
                if p in g: g = g[p]
                else: valid=False; break
            if not valid: continue
            
            for v in vars:
                if v in g:
                    raw = g[v][:]
                    raw = np.nan_to_num(raw)
                    raw_vals[v] = raw
                    
                    if lpf_cutoff is not None:
                        # Use same filter logic
                        from scipy.signal import butter, filtfilt
                        nyq = 0.5 * fs
                        normal_cutoff = lpf_cutoff / nyq
                        b, a = butter(4, normal_cutoff, btype='low', analog=False) # Order 4 matched
                        filt = filtfilt(b, a, raw)
                        filtered_vals[v] = filt
                        
        if not raw_vals:
            print("  -> No output vars found to plot.")
            return
            
        # Plot Setup
        num_plots = len(raw_vals)
        fig, axes = plt.subplots(num_plots, 1, figsize=(10, 4*num_plots), sharex=True)
        if num_plots == 1: axes = [axes]
        
        # Adjust layout to make room for widgets
        plt.subplots_adjust(bottom=0.25)
        
        lines_raw = []
        lines_filt = []
        
        max_idx = 0
        for i, (vname, rdata) in enumerate(raw_vals.items()):
            max_idx = max(max_idx, len(rdata))
            ax = axes[i]
            l1, = ax.plot(rdata, label='Raw', alpha=0.5, color='gray')
            lines_raw.append(l1)
            
            if vname in filtered_vals:
                l2, = ax.plot(filtered_vals[vname], label=f'Filtered ({lpf_cutoff}Hz)', color='red', linewidth=1.5)
                lines_filt.append(l2)
                
            ax.set_title(f"Reference Signal: {vname}")
            ax.legend()
            ax.grid(True)
            
        # Widgets
        axcolor = 'lightgoldenrodyellow'
        
        # Slider
        ax_scroll = plt.axes([0.2, 0.1, 0.65, 0.03], facecolor=axcolor)
        s_scroll = Slider(ax_scroll, 'Scroll', 0, max_idx, valinit=0)
        
        # Button
        ax_button = plt.axes([0.8, 0.025, 0.1, 0.04])
        btn_zoom = Button(ax_button, 'Zoom 5s', color=axcolor, hovercolor='0.975')
        
        window_size_samples = 5 * fs # 5 seconds
        is_zoomed = [False] # Mutable container
        
        def update(val):
            if not is_zoomed[0]:
                return # Do nothing if not zoomed
                
            start = int(s_scroll.val)
            end = start + window_size_samples
            
            for ax in axes:
                ax.set_xlim(start, end)
            fig.canvas.draw_idle()
            
        def toggle_zoom(event):
            is_zoomed[0] = not is_zoomed[0]
            
            if is_zoomed[0]:
                btn_zoom.label.set_text("Reset View")
                # Trigger initial zoom at current slider pos
                update(s_scroll.val)
            else:
                btn_zoom.label.set_text("Zoom 5s")
                # Reset X limits
                for ax in axes:
                    ax.set_xlim(auto=True)
                    ax.relim()
                    ax.autoscale_view()
                fig.canvas.draw_idle()
                
        s_scroll.on_changed(update)
        btn_zoom.on_clicked(toggle_zoom)
        
        print("  -> Interactively check filtering. Close to continue...")
        plt.show(block=True)

def get_ground_contact(grf_fz, threshold=20.0):
    return (np.nan_to_num(grf_fz) > threshold).astype(float)

def butter_lowpass_filter(data, cutoff, fs, order=4):
    from scipy.signal import butter, filtfilt
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    if data.ndim == 1:
        y = filtfilt(b, a, data)
    else:
        y = filtfilt(b, a, data, axis=0)
    return y

class LiveTrainingPlotter:
    def __init__(self):
        self.fig, self.ax = plt.subplots(figsize=(8, 5))
        self.ax.set_xlabel('Epoch')
        self.ax.set_ylabel('Loss')
        self.ax.grid(True)
        plt.ion()
        plt.show(block=False)
        self.train_line = None
        self.val_line = None
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
            # Could add pip freeze here if needed
            
    def save_metrics(self, metrics_dict):
        # Convert numpy types to native python for JSON serialization
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

class WindowDataset(Dataset):
    def __init__(self, X, Y, target_mode="mean"):
        self.X = torch.from_numpy(X).float()
        Y = torch.from_numpy(Y).float()
        if target_mode == "mean":
            self.Y = Y.mean(dim=1)
        elif target_mode == "last":
            self.Y = Y[:, -1, :]
        elif target_mode == "sequence":
            self.Y = Y
        else:
            raise ValueError("target_mode must be 'mean', 'last', or 'sequence'")

    def __len__(self):
        return self.X.shape[0]

    def __getitem__(self, idx):
        return self.X[idx], self.Y[idx]

def extract_condition_data_v2(
    f, sub, cond, 
    input_vars, output_vars,
    lpf_cutoff=None, lpf_order=4, fs=100
):
    """
    input_vars, output_vars: list of (group_path, [var_names...])
    Return: (T, InDim), (T, OutDim)
    """
    
    # -------------------------------------------------------------------------
    # 1. Locate All Trials (Iterate Levels)
    # -------------------------------------------------------------------------
    if sub not in f or cond not in f[sub]:
        # print(f"[DEBUG] {sub}/{cond}: Missing group")
        return None, None

    # New Hierarchy: Subject -> Condition -> Level (lv0, lv4...) -> Trial (trial_01...)
    # We aggregate all trials found under all levels for this condition? 
    # Or just concatenate them? For now, let's concatenate all valid trials found.
    
    cond_group = f[sub][cond]
    level_keys = list(cond_group.keys()) # e.g. ['lv0', 'lv4']
    
    valid_trials_data = [] # List of (in_arr, out_arr) to be stacked or returned?
    # Actually, the original code returned a single array for the condition. 
    # If a condition has multiple trials (or levels), we should probably concatenate them along time 
    # OR return a list of trials. The caller `build_nn_dataset` logic:
    # `X_arr, Y_arr = extract...` then it creates windows.
    # If we concatenate unrelated trials, we create a jump discontinuity.
    # Ideally, we should yield trials or return a list.
    # BUT to keep changes minimal to `build_nn_dataset`, let's concatenate but insert a delimiter/gap if possible,
    # OR just accept the small discontinuity if it's treated as one long stream.
    # Better approach: Iterate and concat. 
    # Note: `build_nn_dataset` creates windows. If we concat, we risk a window spanning across trials.
    # However, standard practice often ignores this or uses a large gap.
    # Let's just concat for now to fit the signature.

    in_list_all = []
    out_list_all = []

    for lv in level_keys:
        lv_group = cond_group[lv]
        trial_keys = list(lv_group.keys()) # e.g. ['trial_01']
        
        for trial_name in trial_keys:
            trial_group = lv_group[trial_name]
            
            # --- Per Trial Extraction ---
            
            # buffers for augmentation
            kin_q_arrays = []  # list of (data_array, var_name)
            left_fz = None
            right_fz = None
            
            curr_trial_in = []
            
            # (A) Standard Load Inputs (Explicit)
            valid_trial = True
            
            # Helper to get base data
            def get_data_from_group(g, path):
                parts = path.split('/')
                curr = g
                for p in parts:
                    if p in curr: curr = curr[p]
                    else: return None
                return curr[:]

            for gpath, vars in input_vars:
                # gpath is e.g. 'robot/back_imu' or 'mocap/kin_q'
                
                # Navigate to group
                grp = trial_group
                parts = gpath.split('/')
                missing_grp = False
                for p in parts:
                    if p in grp: grp = grp[p]
                    else: missing_grp=True; break
                
                if missing_grp:
                    print(f"[DEBUG] {sub}/{cond}/{lv}/{trial_name}: Missing group {gpath}")
                    valid_trial = False
                    break
                
                for v in vars:
                    # Handle Virtual Variables
                    target_data = None
                    
                    # 1. Contact (forceplate logic)
                    if v == 'contact':
                        # Expect 'z' to exist in the same group to derive contact
                        if 'z' in grp:
                            z_raw = grp['z'][:]
                            target_data = get_ground_contact(z_raw, threshold=20.0)
                        else:
                            print(f"[DEBUG] Cannot derive 'contact', 'z' missing in {gpath}")
                            valid_trial = False
                            break
                            
                    # 2. Acceleration (_ddot)
                    elif v.endswith('_ddot'):
                        base_name = v[:-5] # remove _ddot
                        if base_name in grp:
                            raw = grp[base_name][:]
                            # First deriv
                            vel = np.gradient(raw, 1.0/fs)
                            vel = butter_lowpass_filter(vel, 60.0, fs, order=4) # Higher cutoff for intermediate? Or fixed? Using 60 as per old code
                            # Second deriv
                            acc = np.gradient(vel, 1.0/fs)
                            target_data = acc # Will be filtered again by LPF main loop if set
                        else:
                            # Maybe base name isn't in group? Error?
                            print(f"[DEBUG] Base var {base_name} missing for {v}")
                            valid_trial = False; break
                            
                    # 3. Velocity (_dot)
                    elif v.endswith('_dot'):
                        base_name = v[:-4] # remove _dot
                        if base_name in grp:
                            raw = grp[base_name][:]
                            vel = np.gradient(raw, 1.0/fs)
                            target_data = vel
                        else:
                            print(f"[DEBUG] Base var {base_name} missing for {v}")
                            valid_trial = False; break
                            
                    # 4. Standard Variable
                    else:
                        if v in grp:
                            target_data = grp[v][:]
                        else:
                            print(f"[DEBUG] {sub}/{cond}/{lv}/{trial_name}: Missing var {v} in {gpath}")
                            valid_trial = False
                            break
                    
                    if target_data is None:
                        valid_trial = False
                        break
                        
                    # Apply Main LPF
                    data = np.nan_to_num(target_data)
                    if lpf_cutoff is not None:
                         data = butter_lowpass_filter(data, lpf_cutoff, fs, lpf_order)
                    
                    curr_trial_in.append(data.reshape(-1, 1))

            if not valid_trial: continue

            # (B) Augment: Removed (Automatic Joint Velocity/Acc)
            # (C) Augment: Removed (Automatic Contact)
            
            # Now we trust curr_trial_in fits exactly the config list


            if not curr_trial_in:
                 continue
            in_arr_trial = np.hstack(curr_trial_in)

            # -------------------------------------------------------------------------
            # 2. Output
            # -------------------------------------------------------------------------
            curr_trial_out = []
            
            for gpath, vars in output_vars:
                # Navigate Group
                grp = trial_group
                parts = gpath.split('/')
                valid_out = True
                for p in parts:
                    if p in grp: grp = grp[p]
                    else:
                        print(f"[DEBUG] {sub}/{cond}/{lv}/{trial_name}: Missing output group {gpath}")
                        valid_out = False
                        break
                if not valid_out: break
                
                for v in vars:
                    if v not in grp:
                        print(f"[DEBUG] {sub}/{cond}/{lv}/{trial_name}: Missing output var {v}")
                        valid_out = False
                        break
                    d = grp[v][:]
                    d = np.nan_to_num(d)
                    if lpf_cutoff is not None:
                        d = butter_lowpass_filter(d, lpf_cutoff, fs, lpf_order)
                    curr_trial_out.append(d.reshape(-1, 1))
            
            if not valid_out: continue
            if not curr_trial_out: continue
            
            out_arr_trial = np.hstack(curr_trial_out)
            
            # Length check
            min_len = min(in_arr_trial.shape[0], out_arr_trial.shape[0])
            in_list_all.append(in_arr_trial[:min_len])
            out_list_all.append(out_arr_trial[:min_len])
            
    if not in_list_all:
        return None, None
        
    return np.concatenate(in_list_all, axis=0), np.concatenate(out_list_all, axis=0)

def build_nn_dataset(
    data_path, 
    sub_names, cond_names,
    input_vars, output_vars,
    time_window_input, time_window_output, stride,
    subject_selection=None,
    condition_selection=None,
    debug_plot=False,
    lpf_cutoff=None,
    lpf_order=4
):
    X_list = []
    Y_list = []

    print(f"[DEBUG] build_nn_dataset called with subjects={sub_names} conditions={cond_names}")
    with h5py.File(data_path, 'r') as f:
        print(f"[DEBUG] H5 file opened. Keys: {list(f.keys())}")
        for sub in sub_names:
            if subject_selection:
                if sub not in subject_selection.get('include', sub_names):
                    # print(f"[DEBUG] Skipping sub {sub}")
                    continue
                if sub in subject_selection.get('exclude', []):
                    continue
            
            print(f"[DEBUG] Processing sub {sub}")
            
            if sub not in f:
                print(f"[DEBUG] Sub {sub} not in H5 file!")
                continue

            for cond in cond_names:
                if condition_selection:
                    if condition_selection.get('include') and cond not in condition_selection['include']:
                        continue
                    if cond in condition_selection.get('exclude', []):
                        continue
                        
                if cond not in f[sub]:
                    print(f"[DEBUG] Cond {cond} not in f[{sub}]! Keys: {list(f[sub].keys())}")
                    continue

                # Data load
                X_arr, Y_arr = extract_condition_data_v2(
                    f, sub, cond, input_vars, output_vars,
                    lpf_cutoff=lpf_cutoff, lpf_order=lpf_order
                )
                if X_arr is None:
                    print(f"[DEBUG] Extract returned None for {sub}/{cond}")
                    continue
                
                # Sliding Window
                prev_len = len(X_list)
                for i in range(0, len(X_arr) - time_window_input - time_window_output + 1, stride):
                    limit = i + time_window_input
                    end   = limit + time_window_output
                    
                    x_win = X_arr[i : limit, :]     # (Tw_in, Din)
                    y_win = Y_arr[limit : end, :]   # (Tw_out, Dout)
                    
                    X_list.append(x_win)
                    Y_list.append(y_win)
                
                if len(X_list) == prev_len:
                    print(f"[DEBUG] No windows created for {sub}/{cond}. X_arr len={len(X_arr)}")

    if len(X_list) == 0:
        print("[DEBUG] X_list is empty at end of build_nn_dataset")
        return np.array([]), np.array([])
    
    return np.array(X_list), np.array(Y_list)

def plot_model_summary(results):
    """
    results: list of (exp_name, seed, metrics_data, ckpt, ...)
    metrics_data: either float (old style loss) or dict (new style full metrics)
    """
    import matplotlib.pyplot as plt
    
    # Organize data
    # Structure: names -> list of metrics
    data = {}
    
    for row in results:
        name = row[0]
        # seed = row[1] 
        metrics_val = row[2]
        
        if name not in data:
            data[name] = {'huber': [], 'mae': [], 'rmse': [], 'params': []}
            
        if isinstance(metrics_val, dict):
            data[name]['huber'].append(metrics_val.get('test_huber', 0.0))
            data[name]['mae'].append(metrics_val.get('test_mae', 0.0))
            data[name]['rmse'].append(metrics_val.get('test_rmse', 0.0))
            data[name]['params'].append(metrics_val.get('n_params', 0))
        else:
            # Fallback if only float provided
            data[name]['huber'].append(float(metrics_val))
            
    names = sorted(data.keys())
    
    # Prepare Stats
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

    # Plotting 2x2
    fig, axes = plt.subplots(2, 2, figsize=(16, 12))
    
    # 1. Huber Loss
    ax = axes[0, 0]
    x_pos = np.arange(len(names))
    ax.bar(x_pos, means['huber'], yerr=stds['huber'], align='center', alpha=0.7, capsize=10, color='skyblue')
    ax.set_xticks(x_pos)
    ax.set_xticklabels(names, rotation=45, ha='right')
    ax.set_ylabel('Huber Loss')
    ax.set_title('Test Huber Loss')
    ax.grid(axis='y', linestyle='--', alpha=0.5)

    # 2. MAE
    ax = axes[0, 1]
    ax.bar(x_pos, means['mae'], yerr=stds['mae'], align='center', alpha=0.7, capsize=10, color='lightgreen')
    ax.set_xticks(x_pos)
    ax.set_xticklabels(names, rotation=45, ha='right')
    ax.set_ylabel('MAE')
    ax.set_title('Test MAE')
    ax.grid(axis='y', linestyle='--', alpha=0.5)

    # 3. RMSE
    ax = axes[1, 0]
    ax.bar(x_pos, means['rmse'], yerr=stds['rmse'], align='center', alpha=0.7, capsize=10, color='salmon')
    ax.set_xticks(x_pos)
    ax.set_xticklabels(names, rotation=45, ha='right')
    ax.set_ylabel('RMSE')
    ax.set_title('Test RMSE')
    ax.grid(axis='y', linestyle='--', alpha=0.5)
    
    # 4. Params vs Performance (Scatter)
    ax = axes[1, 1]
    # x-axis: params, y-axis: MAE (or Huber)
    # Scatter plot of means
    
    # Color map for names
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

# -------------------------------------------------------------------------------------------------
# Model Definition: TCN_LSTM_MLP Only
# -------------------------------------------------------------------------------------------------

class TransposedBatchNorm1d(nn.Module):
    """
    BatchNorm1d wrapper for inputs of shape (B, L, C).
    Transposes to (B, C, L), applies BN, then transposes back to (B, L, C).
    """
    def __init__(self, num_features):
        super().__init__()
        self.bn = nn.BatchNorm1d(num_features)

    def forward(self, x):
        # x: (B, L, C)
        x = x.transpose(1, 2)  # (B, C, L)
        x = self.bn(x)
        x = x.transpose(1, 2)  # (B, L, C)
        return x

class TCNEncoder(nn.Module):
    def __init__(self, input_dim, channels=(64, 64, 128), kernel_size=3, dropout=0.1, norm_type=None):
        super().__init__()
        layers = []
        in_ch = input_dim
        for i, ch in enumerate(channels):
            dilation = 2 ** i
            
            block_layers = [
                nn.Conv1d(in_ch, ch, kernel_size, padding=0, dilation=dilation),
            ]
            
            if norm_type == 'batch':
                block_layers.append(nn.BatchNorm1d(ch))
            elif norm_type == 'layer':
                # LayerNorm for (B, C, L) -> GroupNorm(1, ch)
                block_layers.append(nn.GroupNorm(1, ch))
            
            block_layers.append(nn.ReLU())
            block_layers.append(nn.Dropout(dropout))
            
            layers += block_layers
            in_ch = ch
            
        self.network = nn.Sequential(*layers)
        self.out_ch = channels[-1]

    def forward(self, x):
        # x: (B, T, D) -> (B, D, T)
        x = x.transpose(1, 2)
        import torch.nn.functional as F
        y = x
        for layer in self.network:
            if isinstance(layer, nn.Conv1d):
                k = layer.kernel_size[0]
                d = layer.dilation[0]
                pad_left = (k - 1) * d
                y = F.pad(y, (pad_left, 0))
            y = layer(y)
        return y[:, :, -1]

class TCN_LSTM_MLP(nn.Module):
    def __init__(self, input_dim, output_dim, horizon, channels=(64,64,128), kernel_size=3, 
                 dropout=0.1, hidden_size=128, num_layers=1, mlp_hidden=128,
                 use_input_norm=False, 
                 tcn_norm=None, 
                 lstm_use_ln=False, 
                 mlp_norm=None):
        super().__init__()
        
        self.use_input_norm = use_input_norm
        if use_input_norm:
            self.input_norm = nn.LayerNorm(input_dim)
            
        # TCN Encoder
        self.enc = TCNEncoder(input_dim, channels, kernel_size, dropout, norm_type=tcn_norm)
        
        self.horizon = horizon
        self.output_dim = output_dim
        
        # LSTM
        self.lstm = nn.LSTM(
            input_size=self.enc.out_ch, hidden_size=hidden_size,
            num_layers=num_layers, batch_first=True,
            dropout=dropout if num_layers > 1 else 0.0
        )
        self.lstm_use_ln = lstm_use_ln
        if lstm_use_ln:
            self.lstm_ln = nn.LayerNorm(hidden_size)

        # MLP Head construction
        head_layers = [nn.Linear(hidden_size, mlp_hidden)]
        
        if mlp_norm == 'batch':
            head_layers.append(TransposedBatchNorm1d(mlp_hidden))
        elif mlp_norm == 'layer':
            head_layers.append(nn.LayerNorm(mlp_hidden))
            
        head_layers.append(nn.ReLU())
        head_layers.append(nn.Dropout(dropout))
        head_layers.append(nn.Linear(mlp_hidden, output_dim))
        
        self.head = nn.Sequential(*head_layers)

    def forward(self, x):
        # x: (B, T, D)
        if self.use_input_norm:
            x = self.input_norm(x)
            
        ctx = self.enc(x)  # (B, C)
        dec_in = ctx.unsqueeze(1).repeat(1, self.horizon, 1)  # (B, H, C)
        yseq, _ = self.lstm(dec_in)  # (B, H, hidden)
        
        if self.lstm_use_ln:
            yseq = self.lstm_ln(yseq)
            
        return self.head(yseq)       # (B, H, D)

# -------------------------------------------------------------------------------------------------
# Training Function (Accepts Config)
# -------------------------------------------------------------------------------------------------

# -------------------------------------------------------------------------------------------------
# Resource Calculation Helper
# -------------------------------------------------------------------------------------------------
def calculate_model_resources(model, input_dim, input_window, device):
    """
    Calculates parameters, model size, and estimates inference latency on the current device.
    """
    # 1. Parameter Count & Size
    param_count = sum(p.numel() for p in model.parameters())
    param_size_mb = param_count * 4 / (1024 * 1024) # Assuming FP32 (4 bytes)
    
    # 2. Inference Timing (Host)
    # Create dummy input based on model input shape (B=1, T, D)
    dummy_input = torch.randn(1, input_window, input_dim).to(device)
    
    model.eval()
    
    # Warmup
    with torch.no_grad():
        for _ in range(20):
            _ = model(dummy_input)
            
    # Timing Loop
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

def train_experiment(
    X_train, Y_train,
    X_val, Y_val,
    X_test, Y_test,
    config,
    seed=42,
    save_dir="experiments/test",
    live_plotter=None
):
    # Setup Logger
    save_path_dir = Path(save_dir)
    save_path_dir.mkdir(parents=True, exist_ok=True)
    
    logger = ExperimentLogger(save_path_dir)
    logger.save_env()
    logger.save_config(config)
    
    ckpt_path = save_path_dir / "model.pt"

    # Extract Hyperparams
    batch_size = config.get("batch_size", 256)
    val_batch_size = config.get("val_batch_size", batch_size)
    epochs = config.get("epochs", 50)
    patience = config.get("patience", 10)
    lr = config.get("lr", 1e-3)
    weight_decay = config.get("weight_decay", 1e-4)
    dropout_p = config.get("dropout_p", 0.1)
    
    dropout_p = config.get("dropout_p", 0.1)
    
    # Normalization Flags
    use_input_norm = config.get("use_input_norm", False)
    tcn_norm = config.get("tcn_norm", None)
    lstm_use_ln = config.get("lstm_use_ln", False)
    mlp_norm = config.get("mlp_norm", None)
    
    # Model Architecture Params
    tcn_channels = config.get("tcn_channels", (64, 64, 128))
    kernel_size = config.get("kernel_size", 3)
    lstm_hidden = config.get("lstm_hidden", 128)
    lstm_layers = config.get("lstm_layers", 1)
    mlp_hidden  = config.get("mlp_hidden", 128)
    
    # Scheduler
    cos_T0 = config.get("cos_T0", 10)
    cos_Tmult = config.get("cos_Tmult", 1)
    eta_min = config.get("eta_min", 1e-5)
    
    # Loss
    huber_delta = config.get("huber_delta", 0.5)
    
    # Reproducibility
    torch.manual_seed(seed)
    np.random.seed(seed)
    random.seed(seed)
    
    # Dataset
    target_mode = "sequence"
    train_ds = WindowDataset(X_train, Y_train, target_mode)
    val_ds   = WindowDataset(X_val,   Y_val,   target_mode)
    test_ds  = WindowDataset(X_test,  Y_test,  target_mode)
    
    train_loader = DataLoader(train_ds, batch_size=batch_size, shuffle=True)
    val_loader   = DataLoader(val_ds, batch_size=val_batch_size, shuffle=False)
    test_loader  = DataLoader(test_ds, batch_size=val_batch_size, shuffle=False)
    
    input_dim = X_train.shape[2]
    horizon = Y_train.shape[1]
    output_dim = Y_train.shape[2]
    
    # Model Instantiation
    model = TCN_LSTM_MLP(
        input_dim, output_dim, horizon,
        channels=tcn_channels,
        kernel_size=kernel_size,
        dropout=dropout_p,
        hidden_size=lstm_hidden,
        num_layers=lstm_layers,
        mlp_hidden=mlp_hidden,
        use_input_norm=use_input_norm,
        tcn_norm=tcn_norm,
        lstm_use_ln=lstm_use_ln,
        mlp_norm=mlp_norm
    )
    
    # Check if we should resume (logic can be added here, for now overwrite)
    # If resuming, load ckpt first.
    
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model.to(device)
    
    optimizer = torch.optim.AdamW(model.parameters(), lr=lr, weight_decay=weight_decay)
    scheduler = None
    if cos_T0 > 0:
        scheduler = torch.optim.lr_scheduler.CosineAnnealingWarmRestarts(
            optimizer, T_0=cos_T0, T_mult=cos_Tmult, eta_min=eta_min
        )
        
    # Cost Function
    # Cost Function
    cost_fn_name = config.get("cost_function", "huber").lower()
    
    # Weighted Loss Config
    use_weighted_loss = config.get("use_weighted_loss", False)
    loss_decay_factor = config.get("loss_decay_factor", 0.8)
    
    # If weighting is ON, we need 'none' to apply weights manually.
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
        
    # Pre-calculate Weights if needed
    loss_weights = None
    if use_weighted_loss:
        # W shape: (1, Horizon, 1) or (1, Horizon, OutputDim)
        # We want to weight by Time Step.
        # w_t = decay^t for t=0..Horizon-1
        w_t = np.array([loss_decay_factor**t for t in range(horizon)])
        
        # Normalize weights? Or keep them <= 1.
        # Let's keep them as decay factors.
        loss_weights = torch.from_numpy(w_t).float().to(device)
        loss_weights = loss_weights.view(1, horizon, 1) # Broadcast across batch and out_dim
        
    best_val = float("inf")
    patience_cnt = 0
    
    print_every = 50
    
    for epoch in range(1, epochs + 1):
        t0 = time.time()
        model.train()
        running = 0.0
        n_batches = 0
        
        for bi, (xb, yb) in enumerate(train_loader, start=1):
            xb, yb = xb.to(device), yb.to(device)
            optimizer.zero_grad(set_to_none=True)
            pred = model(xb)
            
            # Loss Calculation
            if use_weighted_loss:
                # Loss is (B, H, D) because reduction='none'
                raw_loss = criterion(pred, yb)
                # Apply weights
                weighted_loss = raw_loss * loss_weights
                loss = weighted_loss.mean()
            else:
                loss = criterion(pred, yb)
                
            loss.backward()
            optimizer.step()
            running += loss.item()
            n_batches += 1
            
            if (bi % print_every) == 0:
                 avg = running / max(1, n_batches)
                 print(f"[Epoch {epoch:03d} - interim] loss={avg:.6f}", end='\r')
        
        train_loss = running / max(1, n_batches)
        
        model.eval()
        val_running = 0.0
        with torch.no_grad():
            for xb, yb in val_loader:
                xb, yb = xb.to(device), yb.to(device)
                pred = model(xb)
                
                if use_weighted_loss:
                    raw_loss = criterion(pred, yb)
                    weighted_loss = raw_loss * loss_weights
                    loss = weighted_loss.mean()
                else:
                    loss = criterion(pred, yb)
                    
                val_running += loss.item()
        val_loss = val_running / max(1, len(val_loader))
        
        if scheduler:
            scheduler.step(epoch - 1)
            
        cur_lr = optimizer.param_groups[0]["lr"]
        epoch_time = time.time() - t0
        print(f"[Epoch {epoch:03d}/{epochs:03d}] train={train_loss:.6f} val={val_loss:.6f} lr={cur_lr:.2e} time={epoch_time:.1f}s")
        
        # Log to CSV
        logger.log_epoch(epoch, train_loss, val_loss, cur_lr, epoch_time)
        
        if live_plotter:
            live_plotter.update_epoch(epoch, train_loss, val_loss)
            
        if val_loss < best_val:
            best_val = val_loss
            patience_cnt = 0
            ckpt_obj = {
                "state_dict": model.state_dict(),
                "config": config,
                "metrics": {"best_val": best_val},
                "epoch": epoch
            }
            torch.save(ckpt_obj, ckpt_path)
            print(f"  --> Saved Best: {best_val:.6f}")
        else:
            patience_cnt += 1
            if patience_cnt >= patience:
                print("  --> Early Stopping")
                break
                
    # Evaluate
    logger.close()
    
    ckpt = torch.load(ckpt_path, map_location=device)
    model.load_state_dict(ckpt["state_dict"])
    model.eval()
    
    test_running = 0.0
    with torch.no_grad():
        for xb, yb in test_loader:
            xb, yb = xb.to(device), yb.to(device)
            pred = model(xb)
            
            if use_weighted_loss:
                # For reporting consistency, report weighted loss
                raw_loss = criterion(pred, yb)
                weighted_loss = raw_loss * loss_weights
                l = weighted_loss.mean()
            else:
                l = criterion(pred, yb)
                
            test_running += l.item()
    test_loss = test_running / max(1, len(test_loader))
    
    # Calculate MAE & RMSE
    preds_all, trues_all = [], []
    with torch.no_grad():
        for xb, yb in test_loader:
            xb = xb.to(device)
            pred = model(xb).cpu().numpy()
            preds_all.append(pred)
            trues_all.append(yb.numpy())
            
    P = np.concatenate(preds_all, axis=0) # (N, H, D)
    T = np.concatenate(trues_all, axis=0) # (N, H, D)
    err = P - T
    
    test_mae = np.mean(np.abs(err))
    test_rmse = np.sqrt(np.mean(err**2))
    
    test_mae = np.mean(np.abs(err))
    test_rmse = np.sqrt(np.mean(err**2))
    
    # Calculate Resources
    input_window = X_train.shape[1]
    res_metrics = calculate_model_resources(model, input_dim, input_window, device)
    
    # Metrics Dictionary
    metrics_result = {
        "test_huber": test_loss,
        "test_mae": float(test_mae),
        "test_rmse": float(test_rmse),
        
        # Resource Metrics
        "n_params": res_metrics["n_params"],
        "model_size_mb": float(f"{res_metrics['model_size_mb']:.4f}"),
        "host_latency_ms": float(f"{res_metrics['host_latency_ms']:.4f}"),
        "host_max_freq_hz": float(f"{res_metrics['host_max_freq_hz']:.2f}"),
        
        # Original Config
        "config": config
    }
    
    logger.save_metrics(metrics_result)
    
    # Save Confusion Matrix if needed (but this is regression)
    # np.save(save_path_dir / "predictions.npy", P)
    # np.save(save_path_dir / "truths.npy", T)
    
    return metrics_result, ckpt_path



# -------------------------------------------------------------------------------------------------
# Main Execution
# -------------------------------------------------------------------------------------------------

if __name__ == "__main__":
    
    # Load Configs
    config_dir = Path("configs")
    yaml_files = list(config_dir.glob("*.yaml"))
    
    # 1. Load Base Config (Must exist)
    base_config_path = config_dir / "base_config.yaml"
    if not base_config_path.exists():
        print(f"[ERROR] {base_config_path} not found. Cannot proceed without base defaults.")
        sys.exit(1)
        
    with open(base_config_path, 'r') as f:
        base_config = yaml.safe_load(f)
    print(f"[INFO] Loaded base defaults from {base_config_path}")
    
    # 2. Prepare Experiments List
    experiments_to_run = []
    
    if not yaml_files:
        pass # Should not happen if base_config exists
    else:
        print(f"Found {len(yaml_files)} configs: {[f.name for f in yaml_files]}")
        
        for yf in yaml_files:
            # Skip base_config.yaml if we want to run it only once
            # But the logic below merges cfg onto base. 
            # If yf IS base_config.yaml, merged will be identical to base_config.
            # This is fine, it means we run the base experiment.
            
            with open(yf, 'r') as f:
                cfg = yaml.safe_load(f)
                
            # Merge with defaults
            merged = base_config.copy()
            merged.update(cfg)
            
            exp_name = merged.get("exp_name", yf.stem)
            experiments_to_run.append((exp_name, merged))
            
    # Remove Global Dataset Load - Move to Experiment Loop
    # We need to rebuild dataset for EACH experiment because input_vars/output_vars might change!
    
    live_plotter = LiveTrainingPlotter()
    
    results = [] # Reset results list
    seeds = [42]

    for exp_name, cfg in experiments_to_run:
        for sd in seeds:
            full_name = f"{exp_name}_seed{sd}"
            print(f"\n>>> Start Experiment: {full_name}")
            live_plotter.start_session(exp_name, sd)
            
            # Directory Setup
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            save_dir = f"experiments/{full_name}"
            
            # --- Dynamic Data Loading ---
            # Extract vars from config or use default global if not present (backward compatibility)
            # But we moved global vars to base_config, so cfg SHOULD have them.
            
            current_data_path = cfg.get("data_path", base_config["data_path"])
            
            curr_input_vars = cfg.get("input_vars", base_config["input_vars"])
            curr_output_vars = cfg.get("output_vars", base_config["output_vars"])
            
            # Subj / Window
            curr_window = cfg.get("time_window_input", base_config["time_window_input"])
            
            cv_mode = cfg.get("cv_mode", base_config["cv_mode"])
            
            # LPF
            curr_lpf_cutoff = cfg.get("lpf_cutoff", base_config["lpf_cutoff"])
            curr_lpf_order  = cfg.get("lpf_order", base_config["lpf_order"])
            
            # Dataset Universe (Full list of subs/conds available to read)
            curr_sub_names = cfg.get("subjects", base_config["subjects"])
            curr_cond_names = cfg.get("conditions", base_config["conditions"])
            
            # --- TCN Dynamic Config ---
            # Construct channels list from layers/hidden
            if "tcn_channels" in cfg:
                curr_tcn_channels = cfg["tcn_channels"]
            else:
                n_layers = cfg.get("tcn_layers", base_config.get("tcn_layers", 3))
                n_hidden = cfg.get("tcn_hidden", base_config.get("tcn_hidden", 64))
                curr_tcn_channels = [n_hidden] * n_layers
            
            # Inject into config for model init
            cfg["tcn_channels"] = curr_tcn_channels
            print(f"[INFO] Configured TCN: layers={len(curr_tcn_channels)}, hidden={curr_tcn_channels[0]}")
            
            # --- CV Logic: Forced LOSO ---
            if cv_mode != "loso":
                print(f"[WARN] cv_mode='{cv_mode}' detected, but this script now enforces 'loso'. Proceeding with LOSO.")
            
            sub_runs = []
            all_subs = sorted(list(set(curr_sub_names)))
            print(f"[CV-MODE] LOSO selected. Total subjects: {len(all_subs)}")
            
            for i in range(len(all_subs)):
                test_sub = all_subs[i]
                # Val sub: use previous subject (cyclic)
                val_sub = all_subs[i-1] 
                train_subs = [s for s in all_subs if s != test_sub and s != val_sub]
                
                run_name = f"{exp_name}_Test-{test_sub}"
                sub_runs.append({
                    "name": run_name,
                    "train": train_subs,
                    "val": [val_sub],
                    "test": [test_sub]
                })

            # Tuple conversion
            def parse_vars(var_list_from_yaml):
                parsed = []
                for item in var_list_from_yaml:
                    # item is [gpath, [list_of_vars]]
                    gpath = item[0]
                    vars = item[1]
                    parsed.append((gpath, vars))
                return parsed

            c_in_vars = parse_vars(curr_input_vars)
            c_out_vars = parse_vars(curr_output_vars)
            
            # --- Visualization Check (Once per Config) ---
            # We explicitly check filtering on the first subject of the first run
            # to verify reference quality interactively.
            if sub_runs:
                check_sub = sub_runs[0]['train'][0]
                check_cond = curr_cond_names[0] # Pick first condition from universe
                plot_filtering_check(current_data_path, check_sub, check_cond, c_out_vars, curr_lpf_cutoff)

            # --- Execute Sub Runs ---
            for run_meta in sub_runs:
                final_exp_name = run_meta["name"]
                c_train_subs = run_meta["train"]
                c_val_subs = run_meta["val"]
                c_test_subs = run_meta["test"]
                
                full_name_seed = f"{final_exp_name}_seed{sd}" # e.g. Base_Test-S001_seed42
                
                print(f"Building Dataset for {final_exp_name} using {current_data_path}...")
                print(f"  Train: {c_train_subs}")
                print(f"  Val:   {c_val_subs}")
                print(f"  Test:  {c_test_subs}")
                
                # Rebuild per run because subjects change
                X_train, Y_train = build_nn_dataset(
                    current_data_path, curr_sub_names, curr_cond_names, c_in_vars, c_out_vars,
                    curr_window, time_window_output, stride,
                    subject_selection=make_subject_selection(c_train_subs),
                    condition_selection=CONDITION_SELECTION, lpf_cutoff=curr_lpf_cutoff, lpf_order=curr_lpf_order
                )
                X_val, Y_val = build_nn_dataset(
                    current_data_path, curr_sub_names, curr_cond_names, c_in_vars, c_out_vars,
                    curr_window, time_window_output, stride,
                    subject_selection=make_subject_selection(c_val_subs),
                    condition_selection=CONDITION_SELECTION, lpf_cutoff=curr_lpf_cutoff, lpf_order=curr_lpf_order
                )
                X_test, Y_test = build_nn_dataset(
                    current_data_path, curr_sub_names, curr_cond_names, c_in_vars, c_out_vars,
                    curr_window, time_window_output, stride,
                    subject_selection=make_subject_selection(c_test_subs),
                    condition_selection=CONDITION_SELECTION, lpf_cutoff=curr_lpf_cutoff, lpf_order=curr_lpf_order
                )
            
            if len(X_train) == 0:
                print(f"[ERROR] No training data for {exp_name}. Check input_vars/output_vars paths.")
                continue

            print(f"Data Shapes: X_train={X_train.shape}, Y_train={Y_train.shape}")
            
            print(f"\n>>> Start Experiment: {full_name_seed}")
            live_plotter.start_session(final_exp_name, sd)
            save_dir = f"experiments/{full_name_seed}" # Update name
            
            metrics, ckpt_path = train_experiment(
                X_train, Y_train, X_val, Y_val, X_test, Y_test,
                cfg, seed=sd, save_dir=save_dir, live_plotter=live_plotter
            )
            
            print(f"[RESULT] {full_name_seed} | test_loss={metrics['test_huber']:.6f} | params={metrics['n_params']}")
            
            # Store with final name
            results.append((final_exp_name, sd, metrics, ckpt_path, cfg, c_in_vars))
            
    # Summary Plot
    if results:
        summary_fig = plot_model_summary(results)
        summary_fig.savefig("experiments/summary_plot.png")
    
    # -------------------------------------------------------------------------
    # Feature Importance (Best Model)
    # -------------------------------------------------------------------------
    if not results:
        print("No results to analyze.")
        exit()
        
    print("\n>>> Calculating Feature Importance for Best Model...")
    
    # 1. Find Best Model (lowest test_mae)
    best_res = sorted(results, key=lambda x: x[2]['test_mae'])[0]
    best_name, best_seed, best_metrics, best_ckpt, best_cfg, best_in_vars = best_res
    print(f"Best Model: {best_name} (MAE={best_metrics['test_mae']:.6f})")
    
    # 2. Re-load Best Model
    # Re-instantiate
    # Need to reload dataset if we want to re-calc importance on TEST set safely?
    # We have X_test in memory BUT it might be from the LAST iteration.
    # CRITICAL: If experiments use DIFFERENT inputs, X_test currently holds the LAST one.
    # So we MUST re-load the dataset for the best model to ensure X_test matches best_cfg.
    
    print("Re-loading Test Data for Best Model Feature Importance...")
    c_in_vars = best_in_vars
    c_out_vars = parse_vars(best_cfg.get("output_vars", base_config["output_vars"]))
    best_data_path = best_cfg.get("data_path", base_config["data_path"])
    
    # Needs universe from config too
    best_sub_names = best_cfg.get("subjects", base_config["subjects"])
    best_cond_names = best_cfg.get("conditions", base_config["conditions"])
    best_lpf_cutoff = best_cfg.get("lpf_cutoff", base_config["lpf_cutoff"])
    best_lpf_order = best_cfg.get("lpf_order", base_config["lpf_order"])
    
    X_test_best, Y_test_best = build_nn_dataset(
        best_data_path, best_sub_names, best_cond_names, c_in_vars, c_out_vars,
        time_window_input, time_window_output, stride,
        subject_selection=make_subject_selection(TEST_SUBJECTS), # Note: This might need to track best model's specific test subjects if LOSO
        condition_selection=CONDITION_SELECTION, lpf_cutoff=best_lpf_cutoff, lpf_order=best_lpf_order
    )
    
    best_model = TCN_LSTM_MLP(
        input_dim=X_test_best.shape[2],
        output_dim=Y_test_best.shape[2],
        horizon=Y_test_best.shape[1],
        channels=best_cfg.get("tcn_channels", (64, 64, 128)),
        kernel_size=best_cfg.get("kernel_size", 3),
        dropout=best_cfg.get("dropout_p", 0.1),
        hidden_size=best_cfg.get("lstm_hidden", 128),
        num_layers=best_cfg.get("lstm_layers", 1),
        mlp_hidden=best_cfg.get("mlp_hidden", 128),
        use_input_norm=best_cfg.get("use_input_norm", False),
        tcn_norm=best_cfg.get("tcn_norm", None),
        lstm_use_ln=best_cfg.get("lstm_use_ln", False),
        mlp_norm=best_cfg.get("mlp_norm", None)
    )
    
    # Load Weights
    ckpt = torch.load(best_ckpt, map_location='cpu') # Plotting on CPU is fine
    best_model.load_state_dict(ckpt['state_dict'])
    
    device = "cuda" if torch.cuda.is_available() else "cpu"
    best_model.to(device)
    best_model.eval()
    
    # 3. Construct Feature Names
    # Must match extract_condition_data_v2 logic
    feature_names = []
    
    # (A) Input Vars - Use best_in_vars
    for gpath, vars in c_in_vars:
        # Custom Naming Logic
        if 'Back_imu' in gpath or 'back_imu' in gpath:
            prefix = "IMU"
        elif 'Robot' in gpath or 'robot' in gpath:
            if 'left' in gpath: prefix = "Robot_L"
            elif 'right' in gpath: prefix = "Robot_R"
            else: prefix = "Robot"
        elif 'grf' in gpath:
            # forceplate/grf/left
            side = 'L' if 'left' in gpath else 'R'
            prefix = f"GRF_{side}"
        elif 'kin_qdot' in gpath:
            prefix = "J_Vel_Meas"
            pass 
        elif 'kin_q' in gpath:
            prefix = "Deg"
        else:
            prefix = gpath.split('/')[-1]
            
        for v in vars:
            # Clean var names
            v_clean = v.replace('ankle_angle', 'Ankle').replace('knee_angle', 'Knee').replace('hip_', 'Hip_')
            v_clean = v_clean.replace('flexion', 'Flex').replace('adduction', 'Add').replace('rotation', 'Rot')
            v_clean = v_clean.replace('_l', '_L').replace('_r', '_R')
            
            if 'grf' in gpath.lower():
                name = f"{prefix}_{v}"
            elif 'Back_imu' in gpath or 'back_imu' in gpath:
                v_clean = v.replace('Accel_', 'A').replace('Gyro_', 'G')
                v_clean = v_clean.replace('accel_', 'A').replace('gyro_', 'G') # Handle lowercase
                name = f"{prefix}_{v_clean}"
            else:
                name = f"{prefix}_{v_clean}"
                
            feature_names.append(name)
            
    # (B) Augment: Joint Veolcity/Accel
    kin_q_vars = []
    for gpath, vars in c_in_vars:
        if 'kin_q' in gpath and 'dot' not in gpath:
             kin_q_vars = vars
             break
             
    for v in kin_q_vars:
        v_clean = v.replace('ankle_angle', 'Ankle').replace('knee_angle', 'Knee').replace('hip_', 'Hip_')
        v_clean = v_clean.replace('flexion', 'Flex').replace('adduction', 'Add').replace('rotation', 'Rot')
        v_clean = v_clean.replace('_l', '_L').replace('_r', '_R')
        feature_names.append(f"AngVel_{v_clean}")
        
    for v in kin_q_vars:
        v_clean = v.replace('ankle_angle', 'Ankle').replace('knee_angle', 'Knee').replace('hip_', 'Hip_')
        v_clean = v_clean.replace('flexion', 'Flex').replace('adduction', 'Add').replace('rotation', 'Rot')
        v_clean = v_clean.replace('_l', '_L').replace('_r', '_R')
        feature_names.append(f"AngAcc_{v_clean}")
        
    # (C) Contact
    feature_names.append("Contact_L")
    feature_names.append("Contact_R")
    
    if len(feature_names) != X_test_best.shape[2]:
        print(f"[WARN] Feature name count ({len(feature_names)}) != Input dim ({X_test_best.shape[2]}). Using indices.")
        feature_names = [f"Feat_{i}" for i in range(X_test_best.shape[2])]
        
    # 4. Calculate Permutation Importance
    def calculate_permutation_importance(model, X, Y, feature_names, device='cpu'):
        # Baseline MAE
        test_ds = WindowDataset(X, Y, target_mode="sequence")
        loader = DataLoader(test_ds, batch_size=1024, shuffle=False)
        criterion = nn.L1Loss() # MAE
        
        model.eval()
        
        def get_loss():
            running = 0.0
            with torch.no_grad():
                for xb, yb in loader:
                    xb, yb = xb.to(device), yb.to(device)
                    pred = model(xb)
                    running += criterion(pred, yb).item() * xb.size(0)
            return running / len(test_ds)
            
        base_mae = get_loss()
        print(f"  Baseline MAE: {base_mae:.6f}")
        
        importances = []
        
        # Iterate features
        X_orig = X.copy()
        
        for i in range(X.shape[2]):
            # Shuffle column i
            saved_col = X_orig[:, :, i].copy()
            np.random.shuffle(X_orig[:, :, i]) # Shuffle along batch axis
            
            test_ds.X[:, :, i] = torch.from_numpy(X_orig[:, :, i])
            
            # Measure
            perm_mae = get_loss()
            imp = perm_mae - base_mae
            importances.append(imp)
            
            # Restore
            X_orig[:, :, i] = saved_col
            test_ds.X[:, :, i] = torch.from_numpy(saved_col)
            
            print(f"    Feat {i} ({feature_names[i]}): +{imp:.6f}", end='\r')
            
        print("")
        return np.array(importances)

    importances = calculate_permutation_importance(best_model, X_test_best, Y_test_best, feature_names, device)
    
    # 5. Plot All Features
    n_feats = len(importances)
    sorted_idx = np.argsort(importances)[::-1] # All sorted
    
    # Dynamic Figure Height: 0.25 inch per feature
    fig_h = max(8, n_feats * 0.25)
    plt.figure(figsize=(10, fig_h))
    
    plt.barh(range(n_feats), importances[sorted_idx], align='center')
    plt.yticks(range(n_feats), [feature_names[i] for i in sorted_idx])
    plt.xlabel('MAE Increase (Importance)')
    plt.title(f'Feature Importance (All Features) - {best_name}')
    plt.gca().invert_yaxis() # Top importance at top
    plt.tight_layout()
    plt.savefig("experiments/feature_importance.png")
    plt.show(block=True)

