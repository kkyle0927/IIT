# =========================================================================================
# [CRITICAL NOTICE - 수정 금지 / 변경 시 확인 필수]
# * 이 파일은 [GOLDEN BASELINE] 학습 스크립트입니다.
# * 어떤 이유로든 이 파일(SpeedEstimator_TCN_MLP_experiments.py)의 내용을 수정해야 한다면, 
#   반드시 수정하기 전에 사용자에게 한국어(Korean)로 변경하려는 내용과 이유를 설명하고 명시적인 허락을 받아야 합니다.
# * 사용자가 "수정해"라고 말하더라도, 이 주석을 상기하며 "정말로 수정하시겠습니까? 
#   이 주석에 따르면 변경 전 재확인이 필요합니다."라고 한 번 더 되물어 확인받으십시오.
# * 안전 장치를 무시하고 임의로 변경하지 마십시오.
# =========================================================================================


import os
from typing import Optional, List
os.environ["OMP_NUM_THREADS"] = "1"
os.environ["MKL_NUM_THREADS"] = "1"
os.environ["NUMEXPR_NUM_THREADS"] = "1"
os.environ["MKL_THREADING_LAYER"] = "GNU"
os.environ["KMP_DUPLICATE_LIB_OK"] = "TRUE"

import torch
import torch.nn.functional as F
import torch.fft
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
import argparse
from tqdm import tqdm

# from losses import get_custom_losses  <-- Removed external dependency
import torch.fft as fft

# =========================================================================================




def get_custom_losses(config, device, feature_names=None, scaler_mean=None, scaler_std=None):
    """
    Returns a dictionary of custom loss functions and their weights based on config.
    Structure: {'name': {'fn': func, 'weight': float}}
    """
    losses = {}
    train_cfg = config.get("02_train", {}).get("train", {})

    # 1. Smoothness Loss
    # Checks 'smoothness_loss_weight' in baseline.yaml
    w_smooth = float(train_cfg.get("smoothness_loss_weight", 0.0))
    if w_smooth > 0:
        def smoothness_fn(pred):
            # Total Variation: mean((x_t - x_{t-1})^2)
            diff = pred[:, :, 1:] - pred[:, :, :-1]
            return torch.mean(diff ** 2)
        
        losses["smoothness"] = {"fn": smoothness_fn, "weight": w_smooth}

    # 2. Kinematic/Physics Loss
    # Checks 'kinematic_loss_weight_*' in baseline.yaml
    # Currently a placeholder/stub as full kinematic chain requires body model info.
    w_kin_pos = float(train_cfg.get("kinematic_loss_weight_pos", 0.0))
    w_kin_limit = float(train_cfg.get("kinematic_loss_weight_limit", 0.0))
    
    # If weights are non-zero, we add the function. 
    # For now, if 0, we can skip or add a dummy.
    # The training loop expects fn(xb, pred).
    if w_kin_pos > 0 or w_kin_limit > 0:
        def kinematic_fn(x, pred):
            # Placeholder for kinematic constraints
            # x: Inputs (e.g. joint angles)
            # pred: Outputs (e.g. speeds)
            # Implement physics constraints here if needed.
            return torch.tensor(0.0, device=pred.device)
            
        # We sum the weights for the single 'kinematic' entry expect by the loop
        losses["kinematic"] = {"fn": kinematic_fn, "weight": w_kin_pos + w_kin_limit}

    # 3. Frequency Penalty Loss
    penalty_type = train_cfg.get("loss_penalty")
    if penalty_type == "frequency_cutoff_penalty":
        w_freq = float(train_cfg.get("loss_penalty_weight", 0.0))
        
        def freq_loss_fn(pred, target):
            # pred, target: (B, H, D)
            # FFT along time dimension? But pred is (B, H, D). H is small?
            # Or is it (B, T, D)?
            # TCN_MLP returns (B, H, D). If H=1, it's just point prediction.
            # Frequency analysis requires TIME SEQUENCE.
            # If we predict only 1 point (or 5 points), FFT is useless.
            # WAIT. If we train with Window Output 1, we can't do Frequency Loss on Prediction.
            # Unless we gather a batch? No, batch is shuffled.
            
            # Constraint: Frequency Loss ONLY works if we predict a SEQUENCE (e.g. 50-100 ticks).
            # The current baseline predicts 1 tick (or 5 sparse ticks).
            # The User Request implies "Train to verify frequency".
            # If the model output is too short, we typically penalize the smoothness of the mapping?
            # OR we assume the model output is a sequence (Seq2Seq).
            
            # IF output is Scalar, Freq Loss is impossible per sample.
            # We will perform a dummy check: if seq_len < 10, skip.
            if pred.shape[1] < 10:
                return torch.tensor(0.0, device=pred.device)
                
            # Perform FFT
            freq_pred = torch.fft.rfft(pred, dim=1)
            freq_target = torch.fft.rfft(target, dim=1)
            
            # Penalize high freq magnitude difference?
            # User said: "If speed changes at freq higher than reference > loss increases"
            # Reference (target) implies the "valid" bandwidth.
            # If Pred has power in freq bands where Target has NO power, penalize.
            
            mag_pred = torch.abs(freq_pred)
            mag_target = torch.abs(freq_target)
            
            # Mask: where target power is low (noise floor), penalize pred power
            threshold = 1e-4 # Epsilon
            mask = mag_target < threshold 
            
            penalty = mag_pred * mask.float()
            return torch.mean(penalty)

        if w_freq > 0:
            losses["freq_penalty"] = {"fn": freq_loss_fn, "weight": w_freq}



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
            
            for v in g:
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

class LazyWindowDataset(Dataset):
    def __init__(self, X_list, Y_list, input_window, output_window, stride, target_mode="sequence", est_tick_ranges=None, augment=False, noise_std=0.0):
        """
        X_list: list of np.array (T, D_in) - Raw time series
        Y_list: list of np.array (T, D_out)
        target_mode: 'sequence', 'mean', 'last' (Used only if est_tick_ranges is None)
        est_tick_ranges: list of ints, e.g. [1, 2, 3] or [10]. If set, Y is (D_out, len(ranges)) or similar
        augment: bool, if True apply random noise
        noise_std: float, standard deviation of Gaussian noise
        """
        self.X_list = [torch.from_numpy(x).float() for x in X_list]
        self.Y_list = [torch.from_numpy(y).float() for y in Y_list]
        self.input_window = input_window
        self.output_window = output_window
        self.stride = stride
        self.target_mode = target_mode
        self.est_tick_ranges = est_tick_ranges
        self.augment = augment
        self.noise_std = noise_std
        
        # Determine max lookahead for validity check
        if self.est_tick_ranges:
            self.max_lookahead = max(self.est_tick_ranges)
        else:
            self.max_lookahead = output_window
            
        # Pre-calculate valid indices
        # self.indices = [ (series_idx, start_time_idx), ... ]
        self.indices = []
        for s_idx, x_data in enumerate(self.X_list):
            length = x_data.shape[0]
            # Valid start range: [0, length - win_in - max_lookahead]
            # range is exclusive at the end, so +1
            limit = length - input_window - self.max_lookahead + 1
            if limit > 0:
                for t in range(0, limit, stride):
                    self.indices.append((s_idx, t))
                    
    def __len__(self):
        return len(self.indices)

    def __getitem__(self, idx):
        s_idx, t = self.indices[idx]
        
        x_data = self.X_list[s_idx]
        y_data = self.Y_list[s_idx]
        
        limit = t + self.input_window
        
        x_win = x_data[t : limit].clone() # Clone to avoid modifying original or shared memory issues if augmenting
        
        if self.augment and self.noise_std > 0:
            x_win += torch.randn_like(x_win) * self.noise_std
        
        if self.est_tick_ranges:
            # Multi-Horizon Prediction Mode
            # Construct Y by picking specific indices
            # If t is start, x ends at limit(exclusive). limit corresponds to t+win_in.
            # Prediction at +k means data point at limit + k - 1?
            # Example: win_in=100. t=0. x=[0...99]. 
            # Predict +1 tick => x[100] (which is the next point).
            # So index = limit + k - 1. (Assuming k>=1)
            
            y_indices = [(limit + k - 1) for k in self.est_tick_ranges]
            y_win = y_data[y_indices] # (Len(ranges), D_out)
            
            # Ensure shape is (Len(ranges), D_out) or (D_out, Len) ?
            # Pytorch model usually expects (B, Horizon, D_out).
            # This is correct.
        else:
            # Continuous Sequence Mode
            end = limit + self.output_window
            y_win = y_data[limit : end] # Sequence target (Win_Out, D_out)
        
        if self.target_mode == "mean":
            y_win = y_win.mean(dim=0)
        elif self.target_mode == "last":
            y_win = y_win[-1, :]
            
        return x_win, y_win

def extract_condition_data_v2(
    f, sub, cond, 
    input_vars, output_vars,
    lpf_cutoff=None, lpf_order=4, fs=100,
    include_levels=None, include_trials=None,
    use_physical_velocity_model=False,
    use_gait_phase=False
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
        return [], []

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
        if include_levels and lv not in include_levels:
             continue # Filter Level
             
        lv_group = cond_group[lv]
        trial_keys = list(lv_group.keys()) # e.g. ['trial_01']
        
        for trial_name in trial_keys:
            if include_trials and trial_name not in include_trials:
                 continue # Filter Trial
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
                
                # Load Data Columns
                block_cols = []
                for v_name in vars:
                    # 1. Try Direct Load
                    if v_name in grp:
                        col_data = grp[v_name][:]
                    
                    # 2. Try Derived Logic (Velocity/Acceleration/Contact)
                    elif v_name.endswith('_dot') or v_name.endswith('_ddot'):
                        is_accel = v_name.endswith('_ddot')
                        base_name = v_name[:-5] if is_accel else v_name[:-4]
                        
                        if base_name in grp:
                            # Load Base Position
                            base_data = grp[base_name][:]
                            
                            # Differentiation (fs given in arg, default 100)
                            # 1st Deriv
                            d1 = np.gradient(base_data, axis=0) * fs
                            
                            if is_accel:
                                # 2nd Deriv
                                d2 = np.gradient(d1, axis=0) * fs
                                target_data = d2
                            else:
                                target_data = d1
                                
                            # Apply LPF (User requested 30Hz for derivatives)
                            # Using 30Hz cutoff for these derivatives specifically
                            target_data = butter_lowpass_filter(target_data, cutoff=30.0, fs=fs, order=4)
                            col_data = target_data
                            
                            # [NEW] Physical Velocity Mod
                            if use_physical_velocity_model and 'hip' in v_name and 'flexion' in v_name and not is_accel:
                                # V = L * theta_dot * cos(theta)
                                # Load Leg Length
                                L = 0.9
                                if sub in f:
                                     # Try various paths
                                     if "leg_length" in f[sub].attrs: L = f[sub].attrs["leg_length"]
                                     elif "sub_info" in f[sub] and "leg_length" in f[sub]["sub_info"].attrs:
                                          L = f[sub]["sub_info"].attrs["leg_length"]
                                          
                                theta_rad = base_data # base_data is cleaned/valid
                                # Ensure radians? Mocap might be deg? 
                                # Usually defaults to Rad. Check magnitude.
                                # If > 10, likely Deg. If < 2, likely Rad.
                                if np.mean(np.abs(theta_rad)) > 10:
                                     theta_rad = np.deg2rad(theta_rad)
                                     
                                theta_dot = col_data
                                if np.mean(np.abs(theta_dot)) > 50: # Heuristic for Deg/s
                                     theta_dot = np.deg2rad(theta_dot) # Convert for Physics formula?
                                     # But if we output feature, NN handles scale.
                                     # Formula V = L * w * cos(th). V is m/s. 
                                     # w must be rad/s.
                                     pass
                                     
                                # Apply formula
                                v_phys = L * theta_dot * np.cos(theta_rad)
                                col_data = v_phys
                                # print(f"[DEBUG] Applied Physical Model (L={L:.2f})")
                                
                        else:
                             print(f"[DEBUG] {sub}/{cond}/{lv}/{trial_name}: Base {base_name} not found for {v_name}")
                             valid_trial = False
                             break

                    elif v_name == 'contact':
                         # [NEW] Derive contact from GRF Vertical Force (z)
                         # Logic: If 'z' exists in this group, use threshold
                         if 'z' in grp:
                             fz = grp['z'][:]
                             col_data = get_ground_contact(fz, threshold=20.0).reshape(-1, 1)
                         else:
                             print(f"[DEBUG] {sub}/{cond}/{lv}/{trial_name}: GRF Z not found for contact derivation")
                             valid_trial = False
                             break
                             
                    elif gpath == "derived" and "gait_phase" in v_name:
                         # [NEW] Gait Phase Logic
                         # Load contact relation
                         side = 'left' if '_L' in v_name else 'right'
                         z_path = f"forceplate/grf/{side}/z"
                         z_data = get_data_from_group(trial_group, z_path)
                         if z_data is not None:
                             cont = get_ground_contact(z_data)
                             # Calculate Phase
                             # Simple 1-period ramp
                             pad = np.pad(cont.flatten(), (1,0), constant_values=0)
                             diff = np.diff(pad)
                             hs = np.where(diff == 1)[0]
                             ph = np.zeros_like(cont)
                             for k in range(len(hs)-1):
                                 s, e = hs[k], hs[k+1]
                                 ph[s:e] = np.linspace(0, 1, e-s)
                             if len(hs)>0: ph[hs[-1]:] = 1.0 # Tail
                             col_data = ph[:, None]
                         else:
                             # Fallback 0
                             print(f"[WARN] No Z force for Gait Phase {v_name}")
                             col_data = np.zeros((get_data_from_group(trial_group, "robot/left/hip_angle").shape[0], 1))

                    elif use_physical_velocity_model and (v_name.endswith('_dot') or v_name.endswith('_ddot')) and 'hip' in v_name and 'flexion' in v_name:
                        # [NEW] Physical Velocity Model
                        # V = L * theta_dot * cos(theta)
                        # 1. We have theta_dot (col_data)
                        # 2. Need theta (base_name) loaded above
                        # 3. Need Leg Length
                        
                        # Load Leg Length
                        if sub in f and "leg_length" in f[sub].attrs:
                            L = f[sub].attrs["leg_length"]
                        elif sub in f and "sub_info" in f[sub] and "leg_length" in f[sub]["sub_info"].attrs: # Generic path check
                            L = f[sub]["sub_info"].attrs["leg_length"]
                        else:
                            L = 0.9 # Default 0.9m
                        
                        # Load Theta
                        is_accel = v_name.endswith('_ddot')
                        base_name = v_name[:-5] if is_accel else v_name[:-4]
                        
                        # We calculated d1 or d2.
                        # theta is base_data (loaded in previous elif block 596)
                        # BUT python scoping... 'base_data' is updated inside that elif.
                        # We are in a generic 'elif'. 
                        # We should MERGE logic.
                        # Actually, lines 590-617 handle _dot.
                        # We need to INTERCEPT that logic.
                        # I will NOT add a new elif, but modify the existing one via replacement.
                        pass # Handled in separate chunk?
                        
                    else:
                         print(f"[DEBUG] {sub}/{cond}/{lv}/{trial_name}: Dataset {v_name} not found in {gpath}")
                         valid_trial = False
                         break

                    if col_data.ndim == 1: col_data = col_data[:, None]
                    block_cols.append(col_data)
                
                if not valid_trial: break
                arr = np.concatenate(block_cols, axis=1)
                    
                # [FIX] GRF Normalization (Requested by User)
                # "forceplate" AND "grf" in path -> divide by body weight
                # Need mass.
                if "forceplate" in gpath.lower() and "grf" in gpath.lower():
                     # Get Mass
                     mass = 70.0 # Default
                     # Try to read mass from subject level attributes
                     if sub in f and "mass" in f[sub].attrs:
                         mass = f[sub].attrs["mass"]
                     bw = mass * 9.81
                     arr = arr / bw
                     # print(f"[DEBUG] Applied GRF Norm (mass={mass}) to {gpath}")

                kin_q_arrays.append(arr)
                curr_trial_in.append(arr)
                # Loop (524-589) Removed: Redundant data loading.
                # 'arr' already contains all variables for this group.

            if not valid_trial: continue

            # (B) Augment: Gait Phase
            if use_gait_phase:
                 # Logic: Find contact in curr_trial_in? 
                 # curr_trial_in is list of arrays corresponding to input_vars.
                 # We need to find which one is contact.
                 # But input_vars might not include contact if we are adding Gait Phase.
                 # We need to load contact explicitly.
                 
                 # Load Left Contact
                 contact_L = None
                 contact_R = None
                 
                 # Try to find GRF groups
                 # Using helper get_data_from_group
                 g_L = get_data_from_group(trial_group, "forceplate/grf/left/z")
                 g_R = get_data_from_group(trial_group, "forceplate/grf/right/z")
                 
                 if g_L is not None: contact_L = get_ground_contact(g_L)
                 if g_R is not None: contact_R = get_ground_contact(g_R)
                 
                 if contact_L is not None and contact_R is not None:
                     # Calculate Phase
                     def calc_phase(contact):
                         # Contact 0->1 transition is HS.
                         # Reset to 0 at HS, then ramp to 1 until next HS.
                         # Simple ramp: (t - t_hs) / (t_next_hs - t_hs)
                         # This requires future info (offline). We are offline here.
                         
                         phase = np.zeros(len(contact))  # 1D array
                         # Find HS indices
                         pad = np.pad(contact.flatten(), (1,0), constant_values=0)
                         diff = np.diff(pad)
                         hs_indices = np.where(diff == 1)[0]
                         
                         # Fill phase
                         for i in range(len(hs_indices)-1):
                             start = hs_indices[i]
                             end = hs_indices[i+1]
                             dur = end - start
                             ramp = np.linspace(0, 1, dur, endpoint=False)
                             phase[start:end] = ramp
                             
                         # Tail
                         if len(hs_indices) > 0:
                             last = hs_indices[-1]
                             # Just continue linear trend or hold? 
                             phase[last:] = 1.0 
                             
                         return phase[:, None]  # Return as 2D column
                         
                     ph_L = calc_phase(contact_L)
                     ph_R = calc_phase(contact_R)
                     
                     # Append to inputs ?
                     # If the USER requested 'derived/gait_phase_L' in input_vars, 
                     # we should have handled it in the loop.
                     # But 'derived' logic was not in the loop.
                     # We append it here as "Extra Features".
                     # However, build_nn_dataset expects columns to match input_vars count.
                     # If input_vars has 'derived', we handled it? No.
                     # We need to handle 'derived' in the main loop OR
                     # Assume input_vars INCLUDES 'gait_phase' logic.
                     pass 

            # (C) Physical Velocity Model Processing
            if use_physical_velocity_model:
                # Iterate columns found in this trial
                # We need column indices matching 'hip_flexion' and 'hip_flexion_dot'.
                # This is tricky because we flattened everything.
                # Better: Check 'input_vars' loop again.
                pass
                
            # [FIX] Physical Velocity Model should be applied DURING loading (Line 600ish)
            # Retrying the logic in the main loop is hard.
            # We will iterate the loaded buffer if we can map usage.
            # But the buffer is just values.
            # We implemented specific handling below.
            pass

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
                        # print(f"[DEBUG] Applying LPF {lpf_cutoff}Hz to output {v}")
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
        return [], []
        
    return in_list_all, out_list_all

def build_nn_dataset(
    data_path, 
    sub_names, cond_names,
    input_vars, output_vars,
    time_window_input, time_window_output, stride,
    subject_selection=None,
    condition_selection=None,
    debug_plot=False,
    lpf_cutoff=None,
    lpf_order=4,
    est_tick_ranges=None,
    use_physical_velocity_model=False,
    use_gait_phase=False
):
    X_list = []
    Y_list = []

    # print(f"[DEBUG] build_nn_dataset called with subjects={sub_names} conditions={cond_names}")
    
    if not os.path.exists(data_path):
         # Try global path if relative fails? 
         # Assuming data_path is correct.
         pass
         
    with h5py.File(data_path, 'r') as f:
        # print(f"[DEBUG] H5 file opened. Keys: {list(f.keys())}")
        for sub in sub_names:
            if subject_selection:
                if sub not in subject_selection.get('include', sub_names):
                    continue
                if sub in subject_selection.get('exclude', []):
                    continue
            
            # print(f"[DEBUG] Processing sub {sub}")
            
            if sub not in f:
                continue

            for cond in cond_names:
                if condition_selection:
                    if condition_selection.get('include') and cond not in condition_selection['include']:
                        continue
                    if cond in condition_selection.get('exclude', []):
                        continue
                        
                if cond not in f[sub]:
                    # print(f"[DEBUG] Cond {cond} not in f[{sub}]! Keys: {list(f[sub].keys())}")
                    continue

                # Data load
                X_trials, Y_trials = extract_condition_data_v2(
                    f, sub, cond, input_vars, output_vars,
                    lpf_cutoff=lpf_cutoff, lpf_order=lpf_order,
                    use_physical_velocity_model=use_physical_velocity_model,
                    use_gait_phase=use_gait_phase
                )
                
                if not X_trials:
                    # print(f"[DEBUG] Extract returned empty for {sub}/{cond}")
                    continue
                
                # Iterate over executed trials
                for X_arr, Y_arr in zip(X_trials, Y_trials):
                    # Check min length
                    if est_tick_ranges:
                        req_out_len = max(est_tick_ranges)
                    else:
                        req_out_len = time_window_output
                        
                    min_len = time_window_input + req_out_len
                    
                    if len(X_arr) < min_len:
                        continue

                    # Store RAW series (Lazy Loading)
                    X_list.append(X_arr)
                    Y_list.append(Y_arr)

    if len(X_list) == 0:
        print("[DEBUG] X_list is empty at end of build_nn_dataset")
        return [], []
    
    # Return LIST of arrays, not stacked array
    return X_list, Y_list

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
# Model Definition: TCN_MLP Only
# -------------------------------------------------------------------------------------------------

class Chomp1d(nn.Module):
    """Remove the last `chomp_size` elements to keep causality after padding."""
    def __init__(self, chomp_size: int):
        super().__init__()
        self.chomp_size = chomp_size

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        if self.chomp_size == 0:
            return x
        return x[:, :, :-self.chomp_size]


class TemporalBlock(nn.Module):
    def __init__(
        self,
        in_ch: int,
        out_ch: int,
        kernel_size: int,
        stride: int,
        dilation: int,
        dropout: float,
        norm_type: Optional[str] = None,
    ):
        super().__init__()
        from torch.nn.utils import weight_norm
        padding = (kernel_size - 1) * dilation  # causal padding

        self.conv1 = nn.Conv1d(in_ch, out_ch, kernel_size, stride=stride, padding=padding, dilation=dilation)
        self.chomp1 = Chomp1d(padding)
        self.relu1 = nn.ReLU(inplace=True)
        self.drop1 = nn.Dropout(dropout)

        self.conv2 = nn.Conv1d(out_ch, out_ch, kernel_size, stride=stride, padding=padding, dilation=dilation)
        self.chomp2 = Chomp1d(padding)
        self.relu2 = nn.ReLU(inplace=True)
        self.drop2 = nn.Dropout(dropout)

        if norm_type == 'weight':
            self.conv1 = weight_norm(self.conv1)
            self.conv2 = weight_norm(self.conv2)
            self.bn1 = nn.Identity()
            self.bn2 = nn.Identity()
        elif norm_type == 'batch':
            self.bn1 = nn.BatchNorm1d(out_ch)
            self.bn2 = nn.BatchNorm1d(out_ch)
        elif norm_type == 'layer':
            self.bn1 = nn.GroupNorm(1, out_ch)
            self.bn2 = nn.GroupNorm(1, out_ch)
        else:
            self.bn1 = nn.Identity()
            self.bn2 = nn.Identity()

        self.net = nn.Sequential(
            self.conv1, self.chomp1, self.bn1, self.relu1, self.drop1,
            self.conv2, self.chomp2, self.bn2, self.relu2, self.drop2
        )

        self.downsample = nn.Conv1d(in_ch, out_ch, 1) if in_ch != out_ch else nn.Identity()
        self.relu = nn.ReLU(inplace=True)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        out = self.net(x)
        res = self.downsample(x)
        return self.relu(out + res)

class TCNEncoder(nn.Module):
    def __init__(self, input_dim, channels=(64, 64, 128), kernel_size=3, dropout=0.1, norm_type=None):
        super().__init__()
        
        layers = []
        in_ch = input_dim
        for i, ch in enumerate(channels):
            dilation = 2 ** i
            layers.append(TemporalBlock(in_ch, ch, kernel_size, stride=1, dilation=dilation, dropout=dropout, norm_type=norm_type))
            in_ch = ch
            
        self.network = nn.Sequential(*layers)
        self.out_ch = channels[-1]

    def forward(self, x):
        # x: (B, T, D) -> (B, D, T)
        # Handle 2D input (B, D) -> (B, 1, D)
        if x.dim() == 2:
            x = x.unsqueeze(1)
        x = x.transpose(1, 2)
        y = self.network(x)
        return y[:, :, -1]

# KinematicChainLoss moved to losses.py

class TCN_MLP(nn.Module):
    def __init__(self, input_dim, output_dim, horizon, channels=(64,64,128), kernel_size=3, 
                 dropout=0.1, head_dropout=None, mlp_hidden=128,
                 use_input_norm=False, 
                 tcn_norm=None, 
                 mlp_norm=None):
        super().__init__()
        
        self.use_input_norm = use_input_norm
        if use_input_norm:
            self.input_norm = nn.LayerNorm(input_dim)
            
        # TCN Encoder
        self.enc = TCNEncoder(input_dim, channels, kernel_size, dropout, norm_type=tcn_norm)
        
        self.horizon = horizon
        self.output_dim = output_dim
        
        # Calculate Head Input Size from TCN Output
        head_in = self.enc.out_ch

        # MLP Head construction
        head_layers = []
        
        # Support for multi-layer hidden
        if isinstance(mlp_hidden, (list, tuple)):
            h_list = mlp_hidden
        else:
            h_list = [mlp_hidden]
            
        if head_dropout is None: head_dropout = dropout
            
        curr_in = head_in
        for h_size in h_list:
            head_layers.append(nn.Linear(curr_in, h_size))
            if mlp_norm == 'batch':
                head_layers.append(nn.LayerNorm(h_size))
            elif mlp_norm == 'layer':
                head_layers.append(nn.LayerNorm(h_size))
            
            head_layers.append(nn.ReLU())
            if head_dropout > 0:
                head_layers.append(nn.Dropout(head_dropout))
            
            curr_in = h_size
            
        self.head_base = nn.Sequential(*head_layers)
        
        # Final Output Layer (No LSTM, so just Linear: hidden -> horizon*out)
        self.head_out = nn.Linear(curr_in, horizon * output_dim)

    def forward(self, x):
        # x: (B, T, D)
        # Handle 2D input (B, D) -> (B, 1, D)
        if x.dim() == 2:
            x = x.unsqueeze(1)
        
        if self.use_input_norm:
            x = self.input_norm(x)
            
        # TCN Encoder
        ctx = self.enc(x) # (B, C)
        
        # MLP Head
        feat = self.head_base(ctx) # (B, Hidden)
        
        out = self.head_out(feat) # (B, H*out)
        
        # Reshape to (B, H, out)
        if self.horizon > 1:
            out = out.view(-1, self.horizon, self.output_dim)
            
        return out

# -------------------------------------------------------------------------------------------------
# [NEW] Advanced Model Architectures (Moved to be after TCN_MLP defined)
# =========================================================================================

class StanceGatedTCN(TCN_MLP):
    def __init__(self, *args, **kwargs):
        # Extract subclass-specific args
        gating_dim = kwargs.pop('gating_dim', 1) 
        # But TCN_MLP doesn't accept 'gating_dim'. 
        # We need to handle args carefully.
        # Assuming args are passed positionally matching TCN_MLP, or kwargs.
        super().__init__(*args, **kwargs)
        
        input_dim = kwargs.get('input_dim') if 'input_dim' in kwargs else args[0]
        
        # Gating Network: (B, T, In) -> (B, T, In)
        # Simple MLP per-step or shared weights? Shared weights (Conv1d(1)) or Linear.
        # We use a small MLP applied to each time step.
        self.gating_net = nn.Sequential(
            nn.Linear(input_dim, 16),
            nn.ReLU(),
            nn.Linear(16, input_dim),
            nn.Sigmoid()
        )
        
    def forward(self, x):
        # x: (B, T, D)
        # Handle 2D input (B, D) -> (B, 1, D)
        if x.dim() == 2:
            x = x.unsqueeze(1)
        
        # 1. Calculate Gate
        # We use the whole input to determine relevance
        # We use the whole input to determine relevance
        gate = self.gating_net(x) # (B, T, D)
        
        # 2. Apply Gate
        x_gated = x * gate
        
        # 3. Standard TCN Forward (Manual implementation since TCN_MLP.forward is rigid)
        # We can call super().forward(x_gated)
        return super().forward(x_gated)

class AttentionTCN(TCN_MLP):
    def __init__(self, *args, **kwargs):
        atten_type = kwargs.pop('attention_type', 'temporal')
        heads = kwargs.pop('attention_heads', 4)
        super().__init__(*args, **kwargs)
        
        # Attention Layer - placed AFTER Encoder, BEFORE Head
        # Encoder out: (B, enc_out_ch) is pooled? No.
        # TCNEncoder returns (B, C_out) picking last step?
        # Wait, TCNEncoder code: return y[:, :, -1] (Last step).
        # This kills temporal dimension!
        # If we want Temporal Attention, we need the full sequence from Encoder.
        
        # We need to hack TCNEncoder or override it.
        # TCNEncoder is defined in this file.
        # We can wrap self.enc with a version that returns full sequence, 
        # OR we just access self.enc.network directly?
        pass

    def forward(self, x):
        # Handle 2D input (B, D) -> (B, 1, D)
        if x.dim() == 2:
            x = x.unsqueeze(1)
            
        if self.use_input_norm:
            x = self.input_norm(x)
            
        # Access internal network of encoder to get Full Sequence
        # TCNEncoder.forward: x.transpose(1, 2) -> network -> y
        x_t = x.transpose(1, 2) # (B, C, T)
        enc_seq = self.enc.network(x_t) # (B, C_out, T)
        
        # Apply Temporal Attention?
        # Query: Last step? Or learned parameter?
        # "Temporal Attention": Weighted sum of all time steps.
        # Key/Value: enc_seq (T steps). Query: Last step or global context.
        
        # Simple Attention: Alpha = Softmax(Linear(seq))
        # Context = Sum(Alpha * seq)
        
        # But wait, we haven't defined the attention layer in __init__ because we needed 'out_ch'
        # which is available in self.enc.out_ch
        
        # Lazy Init check? No, do it in init?
        # In init, we can access self.enc.out_ch
        if not hasattr(self, 'atten_layer'):
             dim = self.enc.out_ch
             self.atten_w = nn.Linear(dim, 1) # Score
             self.atten_layer = True
             self.atten_w.to(x.device) # Ensure device
        
        # Calculate Scores
        # enc_seq: (B, C, T) -> (B, T, C)
        seq = enc_seq.transpose(1, 2)
        scores = self.atten_w(seq) # (B, T, 1)
        weights = F.softmax(scores, dim=1) # (B, T, 1)
        
        # Context Vector
        ctx = torch.sum(seq * weights, dim=1) # (B, C)
        
        # MLP Head
        feat = self.head_base(ctx)
        out = self.head_out(feat)
        
        if self.horizon > 1:
            out = out.view(-1, self.horizon, self.output_dim)
            
        return out

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
    live_plotter=None,
    scaler_mean=None,
    scaler_std=None,
    feature_names=None
):
    # Setup Logger
    save_path_dir = Path(save_dir)
    save_path_dir.mkdir(parents=True, exist_ok=True)
    
    logger = ExperimentLogger(save_path_dir)
    logger.save_env()
    logger.save_config(config)
    
    ckpt_path = save_path_dir / "model.pt"

    # Extract Hyperparams
    # Extract Hyperparams
    # [FIX] Load from config (YAML priority)
    # Check 02_train -> loader first, then flat config
    loader_cfg = config.get("02_train", {}).get("loader") or {}
    train_cfg  = config.get("02_train", {}).get("train") or {}
    
    # Model Config: Handle 'model: null' or missing
    model_cfg  = config.get("02_train", {}).get("model")
    if not model_cfg:
        # Check if model params are in 'data' (anchor case)
        data_cfg = config.get("02_train", {}).get("data")
        if data_cfg and "channels" in data_cfg:
             model_cfg = data_cfg
             
    if not model_cfg:
        model_cfg = config.get("model") or {}

    batch_size = loader_cfg.get("batch_size") or config.get("batch_size")
    val_batch_size = loader_cfg.get("val_batch_size") or config.get("val_batch_size") or batch_size
    
    epochs = int(train_cfg.get("epochs") or config.get("epochs"))
    patience = int(train_cfg.get("early_stop_patience") or config.get("patience"))
    lr = float(train_cfg.get("lr") or config.get("lr"))
    weight_decay = float(train_cfg.get("wd") or config.get("weight_decay"))
    
    # Dropout & Norm
    dropout_p = model_cfg.get("dropout")
    if dropout_p is None: 
        if "dropout" in config.get("data", {}): dropout_p = config["data"]["dropout"]
        else: dropout_p = config.get("dropout_p", 0.5)
    dropout_p = float(dropout_p)
    
    # Normalization Flags
    use_input_norm = config.get("data", {}).get("normalize", True) # YAML data.normalize
    if use_input_norm is None: 
        use_input_norm = config.get("use_input_norm", True)
        
    model_norm_type = model_cfg.get("model_norm", None)
    tcn_norm = model_norm_type
    mlp_norm = model_norm_type
    
    # Model Architecture Params
    # Model Architecture Params
    tcn_channels = model_cfg.get("channels") or config.get("tcn_channels")
    kernel_size = model_cfg.get("kernel_size") or config.get("kernel_size")
    
    mlp_hidden  = model_cfg.get("head_hidden") or config.get("head_hidden") or config.get("mlp_hidden")
    head_dropout = model_cfg.get("head_dropout") # Optional, defaults to dropout_p if None in next step
    
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
    # Lazy Windowing (Stride & Windows passed here)
    train_cfg = config.get("02_train", {})
    data_cfg = train_cfg.get("data", {})
    if not data_cfg: data_cfg = config.get("data", {}) # Fallback to root data if legacy
    
    print(f"[DEBUG-R] data_cfg keys: {list(data_cfg.keys())}")
    
    # Data Parameters (Standardize on YAML keys: window_size, window_output, stride, y_delay)
    window_size = data_cfg.get("window_size") or config.get("window_size", 200)
    window_output = data_cfg.get("window_output") or data_cfg.get("time_window_output") or config.get("window_output", 1)
    stride = data_cfg.get("stride") or config.get("stride", 5)
    
    # Priority: data_cfg["est_tick_ranges"] > config["est_tick_ranges"] > None
    est_tick_ranges = data_cfg.get("est_tick_ranges") if data_cfg else config.get("est_tick_ranges")
    
    print(f"[DEBUG] win_in={window_size}, win_out={window_output}, ranges={est_tick_ranges}, stride={stride}")
    
    # Augmentation Params
    actual_train_params = train_cfg.get("train", {})
    aug_std = actual_train_params.get("augment_noise_std", 0.0)
    if aug_std > 0:
        print(f"[INFO] Training Data Augmentation Enabled: Gaussian Noise (std={aug_std})")
    
    train_ds = LazyWindowDataset(X_train, Y_train, window_size, window_output, stride, target_mode, est_tick_ranges=est_tick_ranges, augment=True, noise_std=aug_std)
    val_ds   = LazyWindowDataset(X_val,   Y_val,   window_size, window_output, stride, target_mode, est_tick_ranges=est_tick_ranges, augment=False)
    test_ds  = LazyWindowDataset(X_test,  Y_test,  window_size, window_output, stride, target_mode, est_tick_ranges=est_tick_ranges, augment=False)
    
    train_loader = DataLoader(train_ds, batch_size=batch_size, shuffle=True, num_workers=0)
    val_loader   = DataLoader(val_ds, batch_size=val_batch_size, shuffle=False, num_workers=0)
    test_loader  = DataLoader(test_ds, batch_size=val_batch_size, shuffle=False, num_workers=0)
    
    # input_dim from first series
    input_dim = X_train[0].shape[1]
    
    if est_tick_ranges:
        horizon = len(est_tick_ranges)
    else:
        horizon = window_output
    output_dim = Y_train[0].shape[1]
    
    # Model Instantiation
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
        tcn_norm=tcn_norm,
        mlp_norm=mlp_norm
    )
    
    if model_type == "StanceGatedTCN":
        # Extract specific args
        gating_dim = model_cfg.get("gating_dim", 1) # Not used?
        gating_signal = model_cfg.get("gating_signal", "contact")
        # We pass extra args via kwargs if supported, or explicit
        model = StanceGatedTCN(**common_args, gating_dim=gating_dim)
        
    elif model_type == "AttentionTCN":
        attn_type = model_cfg.get("attention_type", "temporal")
        heads = model_cfg.get("attention_heads", 4)
        model = AttentionTCN(**common_args, attention_type=attn_type, attention_heads=heads)
        
    else:
        # Default TCN_MLP
        model = TCN_MLP(**common_args)
    
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
    # [FIX] Remote uses "loss", Local uses "cost_function"
    cost_fn_name = config.get("cost_function", config.get("loss", "huber")).lower()
    
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
        
    # Init Generic Losses from Config
    custom_losses = get_custom_losses(config, device, feature_names, scaler_mean, scaler_std)
    if custom_losses:
        print(f"[Loss] Active Custom Losses: {list(custom_losses.keys())}")

    best_val = float("inf")
    patience_cnt = 0
    
    # [FIX] Better Early Stopping Logic
    min_delta = float(train_cfg.get("early_stop_min_delta") or config.get("early_stop_min_delta", 1e-4))
    train_loss_target = float(train_cfg.get("train_loss_target") or config.get("train_loss_target", 0.0))
    
    for epoch in range(1, epochs + 1):
        t0 = time.time()
        model.train()
        running = 0.0
        n_batches = 0
        
        # Inner Loop: Batches
        # leave=False keeps the log clean by clearing completed epochs
        pbar = tqdm(enumerate(train_loader, start=1), total=len(train_loader), desc=f"Epoch {epoch}", leave=False, file=sys.stdout)
        for bi, (xb, yb) in pbar:
            xb, yb = xb.to(device), yb.to(device)
            optimizer.zero_grad(set_to_none=True)
            pred = model(xb)
            
            # [FIX] Robust Shape Alignment
            # yb might be (B, 1, D) if horizon=1, pred is (B, D)
            if pred.dim() == 2 and yb.dim() == 3 and yb.shape[1] == 1:
                yb = yb.squeeze(1)
            elif pred.dim() == 3 and yb.dim() == 2 and pred.shape[1] == 1:
                pred = pred.squeeze(1)
            
            # Loss Calculation
            if use_weighted_loss:
                # Loss is (B, H, D) because reduction='none'
                raw_loss = criterion(pred, yb)
                # Apply weights
                weighted_loss = raw_loss * loss_weights
                loss = weighted_loss.mean()
            else:
                loss = criterion(pred, yb)
                
            # Apply Custom Losses
            if custom_losses:
                for name, loss_item in custom_losses.items():
                    fn = loss_item['fn']
                    weight = loss_item['weight']
                    
                    # Check signature needed
                    # Smoothness needs 'pred'
                    # Kinematic needs 'xb', 'pred'
                    # Simple dispatch based on name or args?
                    # Simplify: pass raw args, loss fn decides or we wrap?
                    # Losses.py specific wrapper is cleaner, but for now:
                    if name == 'smoothness':
                        loss += weight * fn(pred)
                    elif name == 'kinematic':
                        loss += weight * fn(xb, pred)
                    elif name == 'freq_penalty':
                        # Freq loss needs (pred, target)
                        loss += weight * fn(pred, yb)
            
            loss.backward()
            optimizer.step()
            running += loss.item()
            n_batches += 1
            
            # Update pbar description with current loss
            avg = running / max(1, n_batches)
            pbar.set_postfix({'loss': f'{avg:.4f}'})
        
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
            
        # [FIX] Early Stopping with min_delta
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
            # Check if strictly better (even if small) to save "latest best" anyway? 
            # Usually we only save meaningful improvements.
            if val_loss < best_val:
                 # It improved, but not enough to reset patience. 
                 # We still might want to save it? No, keep semantics consistent.
                 # Actually, usually getting *any* better is worth saving, 
                 # but patience should only reset on *significant* improvement.
                 pass
                 
            if patience_cnt >= patience:
                print("  --> Early Stopping (No improvement)")
                break
        
        # [NEW] Train Loss Target Stop (Prevent Overfitting)
        if train_loss_target > 0 and train_loss < train_loss_target:
             print(f"  --> Train Loss Target Reached ({train_loss:.6f} < {train_loss_target}). Stopping to prevent overfitting.")
             break
    logger.close()
    
    ckpt = torch.load(ckpt_path, map_location=device)
    model.load_state_dict(ckpt["state_dict"], strict=False) # strict=False to avoid minor mismatch crashes
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
            
            # Align shapes before appending
            target = yb.numpy()
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
    
    test_mae = np.mean(np.abs(err))
    test_rmse = np.sqrt(np.mean(err**2))
    
    # Calculate Resources
    input_window = config.get("time_window_input", 100) # Fixed: X_train is list
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
    
    # Arg Parse
    parser = argparse.ArgumentParser()
    parser.add_argument("--rank", type=int, default=0, help="Rank of the current process")
    parser.add_argument("--total_nodes", type=int, default=1, help="Total number of nodes/GPUs")
    parser.add_argument("--config", type=str, default=None, help="Specific config file to run")
    args = parser.parse_args()

    # Load Configs
    config_dir = Path("configs")
    
    if args.config:
        yaml_files = [Path(args.config)]
    else:
        yaml_files = list(config_dir.rglob("*.yaml")) # rglob for recursive search
    
    # 1. Load Base Config (Must exist)
    # 1.5 Load BASE CONFIG if 'base_config.yaml' exists and merge
    # (Typically base_config is default, args.config overrides it)
    # Check if 'exp_name' is missing, implying it might be a partial config
    base_config_path_str = "configs/base_config.yaml"
    base_config = {} # Initialize an empty base_config
    
    # Check if base_config.yaml exists and if the current config is not base_config itself
    if os.path.exists(base_config_path_str) and (args.config is None or Path(args.config).name != "base_config.yaml"):
        print(f"Loading base config from {base_config_path_str}...")
        with open(base_config_path_str, 'r') as bf:
            base_config = yaml.safe_load(bf)
        print(f"[INFO] Loaded base defaults from {base_config_path_str}")
    else:
        print("No base_config.yaml found or explicitly provided as the main config. Proceeding with provided config only.")
        
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
            
            # Fix: Do not inherit exp_name from base_config blindly.
            # If cfg doesn't have it, use filename.
            exp_name = cfg.get("exp_name", yf.stem)
            merged["exp_name"] = exp_name
            experiments_to_run.append((exp_name, merged))
            
    # --- Sharding for Parallel Execution ---
    # Sort to ensure consistent order across processes
    experiments_to_run.sort(key=lambda x: x[0])
    
    # Filter based on rank
    total_exps = len(experiments_to_run)
    experiments_to_run = [e for i, e in enumerate(experiments_to_run) if i % args.total_nodes == args.rank]
    
    print(f"\n[PARALLEL] Rank {args.rank}/{args.total_nodes} - Process {len(experiments_to_run)}/{total_exps} experiments.")
            
    # Remove Global Dataset Load - Move to Experiment Loop
    # We need to rebuild dataset for EACH experiment because input_vars/output_vars might change!
    
    live_plotter = LiveTrainingPlotter()
    
    results = [] # Reset results list
    seeds = [42]

    # Progress bar for Experiments (Total Progress)
    # Use position=0 for the outer bar.
    for exp_name, cfg in tqdm(experiments_to_run, desc=f"GPU-{args.rank} Total", position=0):
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
            
            
    # [FIX] Handle Nested Config Mapping (New Format -> Root Keys for compatibility)
    if '01_construction' in cfg:
        # Data Path
        if 'data_path' not in cfg:
            cfg['data_path'] = cfg['01_construction'].get('src_h5') or cfg['shared'].get('src_h5')
            
        # Subjects & Conditions
        if 'subjects' not in cfg:
            cfg['subjects'] = cfg['01_construction'].get('include_subjects', [])
            
        if 'conditions' not in cfg:
            cfg['conditions'] = cfg['01_construction'].get('include_conditions', [])
            
    # CV Mode Default
    if 'cv_mode' not in cfg:
        # Check 02_train
        if '02_train' in cfg:
            cfg['cv_mode'] = cfg['02_train'].get('split', {}).get('mode', 'loso')
        else:
            cfg['cv_mode'] = 'loso'

    # Retrieve data_path safely
    # [FIX] Handle Nested Config Mapping (Pre-computation)
    if 'data_path' not in cfg:
        src = None
        if '01_construction' in cfg: src = cfg['01_construction'].get('src_h5')
        if not src and 'shared' in cfg: src = cfg['shared'].get('src_h5')
        if not src: src = "./combined_data.h5" # Default fallback
        cfg['data_path'] = src

    if '01_construction' in cfg:
        # Subjects & Conditions
        if 'subjects' not in cfg:
            cfg['subjects'] = cfg['01_construction'].get('include_subjects', [])
        if 'conditions' not in cfg:
            cfg['conditions'] = cfg['01_construction'].get('include_conditions', [])
            
    if 'subjects' not in cfg: cfg['subjects'] = [] # Or handle dynamic loading
    if 'conditions' not in cfg: cfg['conditions'] = []
            
    if 'cv_mode' not in cfg and '02_train' in cfg:
        cfg['cv_mode'] = cfg['02_train'].get('split', {}).get('mode', 'loso')
    elif 'cv_mode' not in cfg:
        cfg['cv_mode'] = 'loso'

    # Retrieve data_path safely
    current_data_path = cfg.get("data_path")
    if not current_data_path and base_config:
        current_data_path = base_config.get("data_path")
        
    if not current_data_path:
        raise ValueError("data_path not found in config or base_config")
        
    print(f"Data Path: {current_data_path}")
    
    curr_input_vars = cfg.get("input_vars") or (base_config.get("input_vars") if base_config else None)
    curr_output_vars = cfg.get("output_vars") or (base_config.get("output_vars") if base_config else None)
    
    # [FIX] Fallback to shared section
    if not curr_input_vars and 'shared' in cfg:
        curr_input_vars = cfg['shared'].get('input_vars')
    if not curr_output_vars and 'shared' in cfg:
        curr_output_vars = cfg['shared'].get('output_vars')
    
    # [FIX] Ensure subjects and conditions are populated from shared
    if not cfg.get('subjects') and 'shared' in cfg:
        cfg['subjects'] = cfg['shared'].get('subjects', [])
    if not cfg.get('conditions') and 'shared' in cfg:
        cfg['conditions'] = cfg['shared'].get('conditions', [])

    # Subj / Window
    curr_window = cfg.get("time_window_input") or (base_config.get("time_window_input") if base_config else None)
    if not curr_window and 'shared' in cfg and 'data' in cfg['shared']:
        curr_window = cfg['shared']['data'].get('window_size')
        
    curr_est_tick_ranges = cfg.get("est_tick_ranges") or (base_config.get("est_tick_ranges") if base_config else None)
    if not curr_est_tick_ranges and 'shared' in cfg and 'data' in cfg['shared']:
        val = cfg['shared']['data'].get('y_delay', 5) # Default 5 if missing?
        curr_est_tick_ranges = [val] if isinstance(val, int) else val
        
    # [FIX] Inject back into cfg so train_experiment sees it
    if curr_est_tick_ranges:
        cfg["est_tick_ranges"] = curr_est_tick_ranges
    # If missing in root, check 01_construction
    if not curr_input_vars and '01_construction' in cfg:
        curr_input_vars = cfg['01_construction'].get('inputs')
    if not curr_output_vars and '01_construction' in cfg:
        curr_output_vars = cfg['01_construction'].get('outputs')
            
    # LPF
    # LPF
    curr_lpf_cutoff = cfg.get("lpf_cutoff")
    if curr_lpf_cutoff is None and '01_construction' in cfg:
        curr_lpf_cutoff = cfg['01_construction'].get('lpf_cutoff')
    if curr_lpf_cutoff is None:
        curr_lpf_cutoff = base_config.get("lpf_cutoff", 0.5) if base_config else 0.5
        
    curr_lpf_order = cfg.get("lpf_order")
    if curr_lpf_order is None and '01_construction' in cfg:
        curr_lpf_order = cfg['01_construction'].get('lpf_order')
    if curr_lpf_order is None:
        curr_lpf_order = base_config.get("lpf_order", 5) if base_config else 5
            
    # Dataset Universe (Full list of subs/conds available to read)
    curr_sub_names = cfg.get("subjects") or (base_config.get("subjects") if base_config else None)
    curr_cond_names = cfg.get("conditions") or (base_config.get("conditions") if base_config else None)
            
    # --- TCN Dynamic Config ---
    # Construct channels list from layers/hidden
    if "tcn_channels" in cfg:
        curr_tcn_channels = cfg["tcn_channels"]
    # [FIX] Check for nested model config in 02_train or shared
    # [FIX] Check for nested model config in 02_train or shared
    elif "02_train" in cfg and cfg["02_train"].get("model") and "channels" in cfg["02_train"]["model"]:
        curr_tcn_channels = cfg["02_train"]["model"]["channels"]
    # [NEW] Check if it's in 02_train['data'] (Anchor case)
    elif "02_train" in cfg and "data" in cfg["02_train"] and "channels" in cfg["02_train"]["data"]:
         curr_tcn_channels = cfg["02_train"]["data"]["channels"]
    elif "shared" in cfg and cfg["shared"].get("model") and "channels" in cfg["shared"]["model"]:
        curr_tcn_channels = cfg["shared"]["model"]["channels"]
    else:
        n_layers = cfg.get("tcn_layers", base_config.get("tcn_layers", 3) if base_config else 3)
        n_hidden = cfg.get("tcn_hidden", base_config.get("tcn_hidden", 64) if base_config else 64)
        curr_tcn_channels = [n_hidden] * n_layers
    
    # Inject into config for model init
    cfg["tcn_channels"] = curr_tcn_channels
    print(f"[INFO] Configured TCN: layers={len(curr_tcn_channels)}, hidden={curr_tcn_channels[0]}")
    
    # --- CV Logic: Forced LOSO ---
    cv_mode = cfg.get('cv_mode', 'loso')
    if cv_mode != "loso":
        print(f"[WARN] cv_mode='{cv_mode}' detected, but this script now enforces 'loso'. Proceeding with LOSO.")


    sub_runs = []
    all_subs = sorted(list(set(curr_sub_names))) if curr_sub_names else []
    print(f"[CV-MODE] LOSO selected. Total subjects: {len(all_subs)}")
            
    for i in range(len(all_subs)):
        test_sub = all_subs[i]
        if test_sub != 'S004': continue # [TEMP] Force S004 only for debugging
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
    visualization_enabled = False # Default to False to prevent error
    if visualization_enabled:
        viz_save_path = f"visualizations/{exp_name}_structure.png"
        pass
        
    print(f"\n[INFO] Starting Cross-Validation Loops: {len(sub_runs)} folds")
    
    for run_meta in sub_runs:
        final_exp_name = run_meta["name"]
        c_train_subs = run_meta["train"]
        c_val_subs = run_meta["val"]
        c_test_subs = run_meta["test"]
        
        full_name_seed = f"{final_exp_name}_seed{sd}"
        print(f"\n>>> Start Experiment: {full_name_seed}")
        print(f"  Train: {c_train_subs}")
        print(f"  Val:   {c_val_subs}")
        print(f"  Test:  {c_test_subs}")
        
        # Load Datasets
        c_est_tick_ranges = cfg.get("est_tick_ranges", None)
        
        # [NEW] Extract Data Flags
        data_cfg_now = cfg.get("02_train", {}).get("data", {})
        use_phys_vel = data_cfg_now.get("use_physical_velocity_model", False)
        use_gait_ph  = data_cfg_now.get("use_gait_phase", False)
        
        if use_phys_vel: print(f"  -> Physical Velocity Model Enabled")
        if use_gait_ph:  print(f"  -> Gait Phase Input Enabled")

        X_train, Y_train = build_nn_dataset(
            current_data_path, curr_sub_names, curr_cond_names, c_in_vars, c_out_vars,
            curr_window, time_window_output, stride,
            subject_selection=make_subject_selection(c_train_subs),
            condition_selection=CONDITION_SELECTION, lpf_cutoff=curr_lpf_cutoff, lpf_order=curr_lpf_order,
            est_tick_ranges=c_est_tick_ranges,
            use_physical_velocity_model=use_phys_vel,
            use_gait_phase=use_gait_ph
        )
        X_val, Y_val = build_nn_dataset(
            current_data_path, curr_sub_names, curr_cond_names, c_in_vars, c_out_vars,
            curr_window, time_window_output, stride,
            subject_selection=make_subject_selection(c_val_subs),
            condition_selection=CONDITION_SELECTION, lpf_cutoff=curr_lpf_cutoff, lpf_order=curr_lpf_order,
            est_tick_ranges=c_est_tick_ranges,
            use_physical_velocity_model=use_phys_vel,
            use_gait_phase=use_gait_ph
        )
        X_test, Y_test = build_nn_dataset(
            current_data_path, curr_sub_names, curr_cond_names, c_in_vars, c_out_vars,
            curr_window, time_window_output, stride,
            subject_selection=make_subject_selection(c_test_subs),
            condition_selection=CONDITION_SELECTION, lpf_cutoff=curr_lpf_cutoff, lpf_order=curr_lpf_order,
            est_tick_ranges=c_est_tick_ranges,
            use_physical_velocity_model=use_phys_vel,
            use_gait_phase=use_gait_ph
        )

        if len(X_train) == 0:
            print(f"[ERROR] No training data for {full_name_seed}. Check selections.")
            continue
            
        save_dir = f"experiments/{full_name_seed}"
        os.makedirs(save_dir, exist_ok=True)
            
        # ---------------------------------------------------------
        # [FIX] Normalization Logic
        # ---------------------------------------------------------
        # Check config
        use_norm = cfg.get("use_input_norm", True) # Default true if not specified
        if not use_norm and '02_train' in cfg:
             use_norm = cfg['02_train'].get('model',{}).get('use_norm', True) # Legacy check
             
        if use_norm:
            print("[INFO] Computing GLOBAL mean/std for Input Normalization...")
            # Pytorch style: (C, T)? No, our data is (T, D).
            # We want mean/std per Feature (D).
            
            # Concatenate all training trials to compute stats
            all_train = np.concatenate(X_train, axis=0) # (Total_T, D)
            mean = np.mean(all_train, axis=0)
            std = np.std(all_train, axis=0)
            scale = std
            
            # Avoid div/0
            scale[scale < 1e-8] = 1.0
            
            # Save scaler
            scaler_path = os.path.join(save_dir, "scaler.npz")
            np.savez(scaler_path, mean=mean, scale=scale)
            print(f"  -> Scaler saved to {scaler_path}")
            
            # Apply to ALL splits (In-place)
            print("  -> Applying normalization to Train/Val/Test...")
            for i in range(len(X_train)): X_train[i] = (X_train[i] - mean) / scale
            for i in range(len(X_val)): X_val[i] = (X_val[i] - mean) / scale
            for i in range(len(X_test)): X_test[i] = (X_test[i] - mean) / scale
            
        else:
            print("[INFO] Skipping Input Normalization (use_input_norm=False)")
        print(f"Data Series Count: Train={len(X_train)}, Val={len(X_val)}, Test={len(X_test)}")
        
        save_dir = f"experiments/{full_name_seed}"
        live_plotter.start_session(final_exp_name, sd)
        
        # [NEW] Construct simple feature names list for Kinematic Loss Mapping
        # We flatten the c_in_vars list which is [(path, [vars]), ...]
        feat_names_simple = []
        for gpath, vars in c_in_vars:
            for v in vars:
                feat_names_simple.append(f"{gpath}/{v}")
        
        metrics, ckpt_path = train_experiment(
            X_train, Y_train, X_val, Y_val, X_test, Y_test,
            cfg, seed=sd, save_dir=save_dir, live_plotter=live_plotter,
            scaler_mean=mean if use_norm else None,
            scaler_std=scale if use_norm else None,
            feature_names=feat_names_simple
        )
        
        print(f"[RESULT] {full_name_seed} | test_loss={metrics.get('test_mse', 0):.6f}")
        results.append((final_exp_name, sd, metrics, ckpt_path, cfg, c_in_vars))
    
    # Summary Plot
    if results:
        summary_fig = plot_model_summary(results)
        summary_fig.savefig("experiments/summary_plot.png")
    
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
    
    # Retrieve Output Vars
    if "output_vars" in best_cfg:
        raw_out = best_cfg["output_vars"]
    elif "01_construction" in best_cfg:
        raw_out = best_cfg["01_construction"].get("outputs")
    else:
        raw_out = base_config.get("output_vars") if base_config else None
        
    c_out_vars = parse_vars(raw_out)
    
    # Retrieve Data Path
    best_data_path = best_cfg.get("data_path")
    if not best_data_path:
        best_data_path = best_cfg.get("01_construction", {}).get("src_h5") or \
                         best_cfg.get("shared", {}).get("src_h5") or \
                         (base_config.get("data_path") if base_config else None)

    # Retrieve Other params
    best_sub_names = best_cfg.get("subjects") or best_cfg.get("01_construction", {}).get("include_subjects")
    best_cond_names = best_cfg.get("conditions") or best_cfg.get("01_construction", {}).get("include_conditions")
    best_lpf_cutoff = best_cfg.get("lpf_cutoff") or best_cfg.get("01_construction", {}).get("lpf_cutoff", 0.5)
    best_lpf_order = best_cfg.get("lpf_order") or best_cfg.get("01_construction", {}).get("lpf_order", 5)
    
    # Window
    best_window = best_cfg.get("time_window_input") or best_cfg.get("shared", {}).get("data", {}).get("window_size")

    X_test_best, Y_test_best = build_nn_dataset(
        best_data_path, best_sub_names, best_cond_names, c_in_vars, c_out_vars,
        best_window, best_cfg.get("time_window_output", 10), best_cfg.get("stride", 20),
        subject_selection=make_subject_selection(best_sub_names), # Use all relevant subjects for global check
        condition_selection=CONDITION_SELECTION, lpf_cutoff=best_lpf_cutoff, lpf_order=best_lpf_order,
        est_tick_ranges=best_cfg.get("est_tick_ranges", None)
    )
    
    best_horizon = len(best_cfg.get("est_tick_ranges")) if best_cfg.get("est_tick_ranges") else best_cfg.get("time_window_output", 10)
    
    # [FIX] Extract Model Config correctly (Nested priority)
    model_cfg = best_cfg.get("02_train", {}).get("model", {})
    if not model_cfg:
        model_cfg = best_cfg.get("model", {})

    tcn_channels = model_cfg.get("channels") or best_cfg.get("tcn_channels")
    kernel_size = model_cfg.get("kernel_size") or best_cfg.get("kernel_size")
    mlp_hidden = model_cfg.get("head_hidden") or best_cfg.get("head_hidden") or best_cfg.get("mlp_hidden")
    head_dropout = model_cfg.get("head_dropout")
    dropout_p = float(model_cfg.get("dropout") or best_cfg.get("dropout_p", 0.1))
    
    # Norm types
    model_norm = model_cfg.get("model_norm")
    
    best_model = TCN_MLP(
        input_dim=X_test_best[0].shape[1],
        output_dim=Y_test_best[0].shape[1],
        horizon=best_horizon, 
        channels=tcn_channels,
        kernel_size=kernel_size,
        dropout=dropout_p,
        head_dropout=head_dropout,
        mlp_hidden=mlp_hidden,
        use_input_norm=best_cfg.get("use_input_norm", True), # Default to True to match training logic if missing
        tcn_norm=model_norm,
        mlp_norm=model_norm
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
    
    if len(feature_names) != X_test_best[0].shape[1]:
        print(f"[WARN] Feature name count ({len(feature_names)}) != Input dim ({X_test_best[0].shape[1]}). Using indices.")
        feature_names = [f"Feat_{i}" for i in range(X_test_best[0].shape[1])]
        
    # 4. Calculate Permutation Importance
    def calculate_permutation_importance(model, test_loader, feature_names, device='cpu'):
        print("Collecting test data into single tensor for importance analysis...")
        X_batches = []
        Y_batches = []
        
        # Collect all batches to a single tensor for easy shuffling
        for xb, yb in tqdm(test_loader, desc="Loading Test Data"):
            X_batches.append(xb)
            Y_batches.append(yb)
            
        if not X_batches:
             print("No test data found.")
             return np.array([])
             
        X_tensor = torch.cat(X_batches, dim=0).to(device)
        Y_tensor = torch.cat(Y_batches, dim=0).to(device)

        criterion = nn.L1Loss() # MAE
        model.eval()
        
        def get_loss(x_in, y_in):
            with torch.no_grad():
                batch_size = 1024
                N = x_in.size(0)
                running_loss = 0.0
                
                for i in range(0, N, batch_size):
                    xb = x_in[i:i+batch_size]
                    yb = y_in[i:i+batch_size]
                    pred = model(xb)
                    running_loss += criterion(pred, yb).item() * xb.size(0)
                    
                return running_loss / N
            
        base_mae = get_loss(X_tensor, Y_tensor)
        print(f"  Baseline MAE: {base_mae:.6f}")
        
        importances = []
        N_feats = X_tensor.shape[2]
        
        for i in range(N_feats):
            # Backup
            original_col = X_tensor[:, :, i].clone()
            
            # Shuffle
            perm_idx = torch.randperm(X_tensor.size(0), device=device)
            X_tensor[:, :, i] = X_tensor[perm_idx, :, i]
            
            # Measure
            perm_mae = get_loss(X_tensor, Y_tensor)
            imp = perm_mae - base_mae
            importances.append(imp)
            
            # Restore
            X_tensor[:, :, i] = original_col
            print(f"    Feat {i} ({feature_names[i]}): +{imp:.6f}", end='\r')
            
        print("")
        return np.array(importances)

    # Create Loader for Importance Calculation
    # Need to reuse parameters from best_cfg
    imp_test_ds = LazyWindowDataset(
        X_test_best, Y_test_best, 
        best_window, best_cfg.get("time_window_output", 60), best_cfg.get("stride", 20), 
        target_mode="sequence", est_tick_ranges=best_cfg.get("est_tick_ranges", None)
    )
    imp_loader = DataLoader(imp_test_ds, batch_size=1024, shuffle=False, num_workers=0)
    
    importances = calculate_permutation_importance(best_model, imp_loader, feature_names, device)
    
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

