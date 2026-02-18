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
import gc
import copy
import pandas as pd
import seaborn as sns
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
            # Multi-step: diff along time/horizon axis (dim=1)
            if pred.dim() == 3 and pred.shape[1] > 1:
                diff = pred[:, 1:, :] - pred[:, :-1, :]
                return torch.mean(diff ** 2)
            return torch.tensor(0.0, device=pred.device)
        
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

def parse_vars(var_list_from_yaml):
    """
    Converts list-based YAML vars [[path, [vars]], ...] to tuple-based list.
    """
    if not var_list_from_yaml:
        return []
    parsed = []
    for item in var_list_from_yaml:
        # item is [gpath, [list_of_vars]]
        if isinstance(item, (list, tuple)) and len(item) >= 2:
            gpath = item[0]
            vars = item[1]
            parsed.append((gpath, vars))
    return parsed

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

def butter_highpass_filter(data, cutoff, fs, order=4):
    from scipy.signal import butter, filtfilt
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='high', analog=False)
    if data.ndim == 1:
        y = filtfilt(b, a, data)
    else:
        y = filtfilt(b, a, data, axis=0)
    return y

def get_data_from_group(g, path):
    parts = path.split('/')
    curr = g
    for p in parts:
        if p in curr: curr = curr[p]
        else: return None
    return curr[:]

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
    use_gait_phase=False,
    input_lpf_cutoff=None, input_lpf_order=4
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
                # [NEW] Group-level Derived Cache
                derived_cache = {}
                
                # [NEW] Load Treadmill Acceleration (for frame of reference correction)
                a_belt_global = 0.0
                v_belt_data = get_data_from_group(trial_group, "treadmill/left/speed_leftbelt")
                if v_belt_data is not None:
                    # v_belt_data is usually (T, 1) or (T,)
                    v_belt_flat = v_belt_data.flatten()
                    a_belt_global = np.gradient(v_belt_flat, axis=0) * fs
                
                for v_name in vars:
                    col_data = None
                    base_name = None
                    is_accel = False
                    
                    # 1. Direct Load
                    if v_name in grp:
                        col_data = grp[v_name][:]
                        if input_lpf_cutoff is not None:
                            col_data = butter_lowpass_filter(col_data, input_lpf_cutoff, fs, input_lpf_order)
                    
                    # 2. Derived Logic (Velocity/Acceleration/Contact)
                    elif v_name.endswith('_dot') or v_name.endswith('_ddot'):
                        if v_name.endswith('_ddot'):
                            base_name = v_name[:-5]
                            is_accel = True
                        else:
                            base_name = v_name[:-4]
                            is_accel = False
                        
                        if base_name in grp:
                            # Load Base Position
                            base_data = grp[base_name][:]
                            d1 = np.gradient(base_data, axis=0) * fs
                            if is_accel:
                                target_data = np.gradient(d1, axis=0) * fs
                            else:
                                target_data = d1
                                
                            target_data = butter_lowpass_filter(target_data, cutoff=30.0, fs=fs, order=4)
                            col_data = target_data
                            
                            if use_physical_velocity_model and not is_accel and 'hip' in v_name and 'flexion' in v_name:
                                L = 0.9 # Default
                                if sub in f and "leg_length" in f[sub].attrs: L = f[sub].attrs["leg_length"]
                                elif sub in f and "sub_info" in f[sub] and "leg_length" in f[sub]["sub_info"].attrs: L = f[sub]["sub_info"].attrs["leg_length"]
                                col_data = L * target_data * np.cos(base_data)
                        else:
                             print(f"[WARN] Base data {base_name} not found for {v_name}")
                             valid_trial = False; break

                    # [NEW] Fallback for Milestone 1: hip_angle -> motor_angle
                    elif 'hip_angle' in v_name:
                        alt_name = v_name.replace('hip_angle', 'motor_angle')
                        if alt_name in grp:
                            col_data = grp[alt_name][:]
                        else:
                            print(f"[DEBUG-FALLBACK] {alt_name} NOT found in grp.")

                    # 3. Derive Contact from GRF Z
                    elif v_name == 'contact':
                         if 'z' in grp:
                             col_data = get_ground_contact(grp['z'][:], threshold=20.0).reshape(-1, 1)
                         else:
                             print(f"[DEBUG] GRF Z not found for contact")
                             valid_trial = False; break
                             
                    # 4. Gait Phase (Derived)
                    elif gpath == "derived" and "gait_phase" in v_name:
                         side = 'left' if '_L' in v_name else 'right'
                         z_data = get_data_from_group(trial_group, f"forceplate/grf/{side}/z")
                         if z_data is not None:
                             cont = get_ground_contact(z_data)
                             pad = np.pad(cont.flatten(), (1,0), constant_values=0)
                             hs = np.where(np.diff(pad) == 1)[0]
                             ph = np.zeros_like(cont)
                             for k in range(len(hs)-1):
                                 s, e = hs[k], hs[k+1]
                                 ph[s:e] = np.linspace(0, 1, e-s)
                             if len(hs)>0: ph[hs[-1]:] = 1.0
                             col_data = ph[:, None]
                         else:
                             col_data = np.zeros((get_data_from_group(trial_group, "robot/left/hip_angle").shape[0], 1))
                    
                    # 5. Advanced IMU Features (Orientation & Integrated Velocity)
                    elif gpath == "robot/back_imu" and v_name in ["roll", "pitch", "yaw", "vel_forward"]:
                        try:
                            if v_name in derived_cache:
                                col_data = derived_cache[v_name]
                            else:
                                raw_ax = grp['accel_x'][:]
                                raw_ay = grp['accel_y'][:]
                                raw_az = grp['accel_z'][:]
                                raw_gx = grp['gyro_x'][:]
                                raw_gy = grp['gyro_y'][:]
                                raw_gz = grp['gyro_z'][:]
                                
                                if sub.startswith("m1_"): raw_ax = -raw_ax
                                elif sub.startswith("m2_"): raw_az = -raw_az

                                n = len(raw_ax)
                                dt = 1.0 / fs
                                alpha = 0.98
                                rs, ps, ys = np.zeros(n), np.zeros(n), np.zeros(n)
                                curr_r, curr_p, curr_y = 0.0, 0.0, 0.0
                                for i in range(n):
                                    r_a = np.arctan2(raw_ay[i], raw_az[i])
                                    p_a = np.arctan2(-raw_ax[i], np.sqrt(raw_ay[i]**2 + raw_az[i]**2))
                                    curr_r = alpha*(curr_r + raw_gx[i]*dt) + (1-alpha)*r_a
                                    curr_p = alpha*(curr_p + raw_gy[i]*dt) + (1-alpha)*p_a
                                    curr_y += raw_gz[i] * dt
                                    rs[i], ps[i], ys[i] = curr_r, curr_p, curr_y
                                
                                # Frame of Reference Correction: a_total = a_body_inertial + a_belt
                                # This transforms the feature into the treadmill surface frame.
                                a_lin = raw_ax + 9.81 * np.sin(ps)
                                if isinstance(a_belt_global, np.ndarray) and len(a_belt_global) == n:
                                    a_lin = a_lin + a_belt_global
                                
                                vel_f = butter_highpass_filter(np.cumsum(a_lin)*dt, 0.1, fs, 2)
                                
                                derived_cache["roll"], derived_cache["pitch"], derived_cache["yaw"], derived_cache["vel_forward"] = rs, ps, ys, vel_f
                                col_data = derived_cache[v_name]
                        except Exception as e:
                            print(f"[ERROR] IMU Feature Error: {e}")
                            valid_trial = False; break

                    else:
                        print(f"[DEBUG] {sub}/{cond}/{lv}/{trial_name}: Dataset {v_name} not found in {gpath}")
                        valid_trial = False; break

                    # -------------------------------------------------------------------------
                    # [NEW] Data Correction: Hardcoded Sign Flipping based on User Request
                    # -------------------------------------------------------------------------
                    if col_data is not None:
                         # 1. Milestone 1 Corrections (prefix "m1_")
                        if sub.startswith("m1_"):
                            # GRF Left X
                            if "grf" in gpath and "left" in gpath and v_name == "x":
                                col_data = -col_data
                            # GRF Right X
                            elif "grf" in gpath and "right" in gpath and v_name == "x":
                                col_data = -col_data
                            # Knee Angle L & L_dot
                            elif "knee_angle_l" in v_name:
                                col_data = -col_data
                            # Knee Angle R & R_dot
                            elif "knee_angle_r" in v_name:
                                col_data = -col_data

                        # 2. Milestone 2 Corrections (prefix "m2_")
                        elif sub.startswith("m2_"):
                            # GRF Left Z
                            if "grf" in gpath and "left" in gpath and v_name == "z":
                                col_data = -col_data
                            # GRF Right Z
                            elif "grf" in gpath and "right" in gpath and v_name == "z":
                                col_data = -col_data

                    # Append to block columns
                    if col_data is not None:
                        if col_data.ndim == 1: col_data = col_data[:, None]
                        block_cols.append(col_data)
                    else:
                        valid_trial = False
                        break
                
                if not valid_trial: break
                arr = np.concatenate(block_cols, axis=1).astype(np.float32)
                    
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

            # --- Synchronization (Handle slight length mismatches between sensor groups) ---
            # 1. Collect all potential arrays (inputs and outputs) to find global min length
            all_objs = curr_trial_in.copy()
            
            # Identify output arrays first
            extracted_outs = []
            valid_out = True
            for gpath, vars in output_vars:
                grp = trial_group
                parts = gpath.split('/')
                found_grp = True
                for p in parts:
                    if p in grp: grp = grp[p]
                    else: found_grp = False; break
                if not found_grp: valid_out = False; break
                
                for v in vars:
                    if v not in grp: valid_out = False; break
                    d = grp[v][:]
                    d = np.nan_to_num(d).reshape(-1, 1)
                    if lpf_cutoff is not None:
                        d = butter_lowpass_filter(d, lpf_cutoff, fs, lpf_order)
                    extracted_outs.append(d)
            
            if not valid_out or not extracted_outs:
                continue
                
            # 2. Find common minimum length
            all_objs.extend(extracted_outs)
            min_len_trial = min([a.shape[0] for a in all_objs])
            
            # 3. Trim and Stack
            in_arr_trial = np.hstack([a[:min_len_trial] for a in curr_trial_in])
            out_arr_trial = np.hstack([a[:min_len_trial] for a in extracted_outs])
            
            # Valid trial found
            in_list_all.append(in_arr_trial)
            out_list_all.append(out_arr_trial)

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
    use_gait_phase=False,
    input_lpf_cutoff=None,
    input_lpf_order=4
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
                    use_gait_phase=use_gait_phase,
                    input_lpf_cutoff=input_lpf_cutoff,
                    input_lpf_order=input_lpf_order
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

def build_nn_dataset_multi(
    config,
    sub_names, cond_names,
    input_vars, output_vars,
    time_window_input, time_window_output, stride,
    subject_selection=None,
    condition_selection=None,
    lpf_cutoff=None,
    lpf_order=None,
    est_tick_ranges=None,
    use_physical_velocity_model=False,
    use_gait_phase=False,
    input_lpf_cutoff=None,
    input_lpf_order=4
):
    # Determine data sources
    data_sources = {}

    if 'shared' in config and 'data_sources' in config['shared']:
        for ds_name, ds_config in config['shared']['data_sources'].items():
            prefix = ds_config.get('prefix', '')
            path = ds_config.get('path', '')
            exclude = ds_config.get('exclude_subjects', [])
            if path:
                 data_sources[prefix] = {'path': path, 'exclude_subjects': exclude}
    
    # Fallback to single path in config
    if not data_sources:
        data_path = config.get('data_path')
        if not data_path:
             # Try nested
             if '01_construction' in config: data_path = config['01_construction'].get('src_h5')
             if not data_path and 'shared' in config: data_path = config['shared'].get('src_h5')
             if not data_path: data_path = './combined_data.h5'
             
        data_sources = {'': {'path': data_path, 'exclude_subjects': []}} # Empty prefix for default
        
    X_all = []
    Y_all = []
    
    print(f"[DATA] Loading Multi-Source: keys={list(data_sources.keys())}")
    
    for prefix, src_cfg in data_sources.items():
        path = src_cfg['path']
        src_exclude = src_cfg.get('exclude_subjects', [])
        
        # 1. Filter & Strip 'sub_names'
        # sub_names are like ['m1_S011', 'm2_S001', ...]
        # We only keep those matching the current prefix.
        current_source_subs = []
        if sub_names:
            for s in sub_names:
                if prefix and s.startswith(prefix):
                    # Match prefix -> Strip and Keep
                    stripped = s[len(prefix):]
                    current_source_subs.append(stripped)
                elif not prefix:
                    # No prefix source (Legacy or Single H5)
                    # We try to keep all, provided they don't look like they belong to another prefix?
                    # Or simpler: Just keep all. If they aren't in H5, build_nn_dataset skips them.
                    current_source_subs.append(s)
        
        if not current_source_subs:
            continue
            
        # 2. Filter & Strip 'subject_selection'
        # subject_selection has 'include' and 'exclude' lists of full names.
        curr_sub_select = {}
        if subject_selection:
            # Handle Include
            if 'include' in subject_selection:
                inc_list = []
                for s in subject_selection['include']:
                    if prefix and s.startswith(prefix): inc_list.append(s[len(prefix):])
                    elif not prefix: inc_list.append(s)
                curr_sub_select['include'] = inc_list
            
            # Handle Exclude
            # Combine Global Exclude (from subject_selection) AND Source-Specific Exclude
            ex_list = []
            if 'exclude' in subject_selection:
                for s in subject_selection['exclude']:
                    if prefix and s.startswith(prefix): ex_list.append(s[len(prefix):])
                    elif not prefix: ex_list.append(s)
            
            # Add Source Excludes (assuming they match keys in H5 directly? or full names?)
            # Config usually has exclude_subjects: ['S004'] for that source.
            if src_exclude:
                ex_list.extend(src_exclude)
                
            curr_sub_select['exclude'] = ex_list

        
        print(f"  -> Source '{prefix}': {path} | Subs: {len(current_source_subs)}")
            
        # Call build_nn_dataset
        X_src, Y_src = build_nn_dataset(
            path, current_source_subs, cond_names,
            input_vars, output_vars,
            time_window_input, time_window_output, stride,
            subject_selection=curr_sub_select,
            condition_selection=condition_selection,
            lpf_cutoff=lpf_cutoff,
            lpf_order=lpf_order,
            est_tick_ranges=est_tick_ranges,
            use_physical_velocity_model=use_physical_velocity_model,
            use_gait_phase=use_gait_phase,
            input_lpf_cutoff=input_lpf_cutoff,
            input_lpf_order=input_lpf_order
        )
        
        if X_src:
            print(f"     Loaded {len(X_src)} trials")
            X_all.extend(X_src)
            Y_all.extend(Y_src)
            
    return X_all, Y_all

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
        elif norm_type == 'instance':
            self.bn1 = nn.InstanceNorm1d(out_ch)
            self.bn2 = nn.InstanceNorm1d(out_ch)
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
                 mlp_norm=None, **kwargs):
        super().__init__()
        
        self.use_input_norm = use_input_norm
        if use_input_norm:
            norm_type = kwargs.get('input_norm_type', 'layer')
            if norm_type == 'instance':
                # InstanceNorm1d expects (B, C, T). 
                # Our input is (B, T, D). We will transpose in forward or use LayerNorm if 1D.
                self.input_norm = nn.InstanceNorm1d(input_dim)
            else:
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
            if isinstance(self.input_norm, nn.InstanceNorm1d):
                # x: (B, T, D) -> (B, D, T) for InstanceNorm1d
                x = x.transpose(1, 2)
                x = self.input_norm(x)
                x = x.transpose(1, 2)
            else:
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
# [NEW] TCN + GRU Head (Stateful Head for Recursive Experiments)
# -------------------------------------------------------------------------------------------------
class TCN_GRU_Head(TCN_MLP):
    """TCN encoder + GRU head (replaces MLP head)."""
    def __init__(self, *args, gru_hidden=32, **kwargs):
        super().__init__(*args, **kwargs)
        enc_out = self.enc.out_ch
        self.gru = nn.GRU(enc_out, gru_hidden, batch_first=True)
        self.head_out = nn.Linear(gru_hidden, self.horizon * self.output_dim)
        # Remove MLP head layers (replaced by GRU)
        self.head_base = nn.Identity()
        self._gru_hidden = gru_hidden

    def forward(self, x):
        if x.dim() == 2:
            x = x.unsqueeze(1)
        if self.use_input_norm:
            if isinstance(self.input_norm, nn.InstanceNorm1d):
                x = x.transpose(1, 2)
                x = self.input_norm(x)
                x = x.transpose(1, 2)
            else:
                x = self.input_norm(x)
        # Get full TCN sequence output (not just last step)
        x_t = x.transpose(1, 2)  # (B, D, T)
        enc_seq = self.enc.network(x_t)  # (B, C, T)
        enc_seq = enc_seq.transpose(1, 2)  # (B, T, C)
        gru_out, _ = self.gru(enc_seq)  # (B, T, gru_hidden)
        ctx = gru_out[:, -1, :]  # Last step: (B, gru_hidden)
        out = self.head_out(ctx)  # (B, H*out)
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
    
    # [NEW] Merge overrides from shared.model (experiment YAMLs override shared.model
    # but YAML anchors create separate copies in 02_train.model, so we must merge)
    shared_model = config.get("shared", {}).get("model", {})
    if shared_model:
        def deep_merge(base, override):
            """Deep merge override dict into base dict"""
            for k, v in override.items():
                if k in base and isinstance(base[k], dict) and isinstance(v, dict):
                    deep_merge(base[k], v)
                else:
                    base[k] = v
        deep_merge(model_cfg, shared_model)

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
    input_norm_type = model_cfg.get("input_norm_type", model_norm_type or "layer")
    
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
    train_cfg_inner = config.get("02_train", {}).get("train", {})
    huber_delta = float(train_cfg_inner.get("huber_delta") or config.get("huber_delta", 0.5))
    
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
    
    # [NEW] Merge overrides from shared.data (same anchor duplication issue as model)
    shared_data = config.get("shared", {}).get("data", {})
    if shared_data:
        def deep_merge_data(base, override):
            """Deep merge override dict into base dict"""
            for k, v in override.items():
                if k in base and isinstance(base[k], dict) and isinstance(v, dict):
                    deep_merge_data(base[k], v)
                else:
                    base[k] = v
        deep_merge_data(data_cfg, shared_data)
    
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
    
    if platform.system() == 'Linux':
        num_workers = 0 # Revert to 0 to avoid Virtual Memory Overhead (15GB limit)
    else:
        num_workers = 0
            
    train_loader = DataLoader(train_ds, batch_size=batch_size, shuffle=True, num_workers=num_workers)
    val_loader   = DataLoader(val_ds, batch_size=val_batch_size, shuffle=False, num_workers=num_workers)
    test_loader  = DataLoader(test_ds, batch_size=val_batch_size, shuffle=False, num_workers=num_workers)
    
    # input_dim from first series
    input_dim = X_train[0].shape[1]
    
    # [NEW] AR Feedback: augment input with previous prediction channel
    ar_cfg = model_cfg.get("ar_feedback", {})
    ar_enable = ar_cfg.get("enable", False)
    if ar_enable:
        ar_k = ar_cfg.get("k", 1)
        print(f"[INFO] AR Feedback enabled: +{ar_k} channel(s) to input")
        input_dim += ar_k  # Extra channel for prev_pred
    
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
        input_norm_type=input_norm_type,
        tcn_norm=tcn_norm,
        mlp_norm=mlp_norm
    )
    
    if model_type == "StanceGatedTCN":
        gating_dim = model_cfg.get("gating_dim", 1)
        gating_signal = model_cfg.get("gating_signal", "contact")
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
    val_loss = float("inf")
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
        # [NEW] AR Feedback: teacher forcing ratio for this epoch
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
        for bi, (xb, yb) in pbar:
            xb, yb = xb.to(device), yb.to(device)
            
            # [NEW] AR Feedback: append prev_pred channel
            if ar_enable:
                B, T_in, D = xb.shape
                if random.random() < tf_ratio:
                    # Teacher forcing: use ground truth (last output value = y at t-1)
                    # Approximate: use the output LPF target value at the last input timestep
                    prev_val = yb[:, 0:1, :] if yb.dim() == 3 else yb[:, :1].unsqueeze(1)
                    prev_channel = prev_val.expand(B, T_in, 1)
                else:
                    prev_channel = torch.zeros(B, T_in, 1, device=device)
                xb = torch.cat([xb, prev_channel], dim=-1)
            
            optimizer.zero_grad(set_to_none=True)
            
            # [FIX] OOM Handling
            try:
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
                        
                        if name == 'smoothness':
                            loss += weight * fn(pred)
                        elif name == 'kinematic':
                            loss += weight * fn(xb, pred)
                        elif name == 'freq_penalty':
                            loss += weight * fn(pred, yb)
                
                loss.backward()
                optimizer.step()
                
            except RuntimeError as e:
                if "out of memory" in str(e):
                    raise RuntimeError(
                        f"Memory Insufficient (OOM): Batch size {batch_size} is too large. "
                        "Please reduce batch_size in config."
                    ) from e
                raise e
            running += loss.item()
            n_batches += 1
            
            # Update pbar description with current loss
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
            for xb, yb in val_pbar:
                xb, yb = xb.to(device), yb.to(device)
                
                # [NEW] AR Feedback Validation: append zero channel (inference mode)
                if ar_enable:
                    B, T_in, _ = xb.shape
                    prev_channel = torch.zeros(B, T_in, ar_k, device=device)
                    xb = torch.cat([xb, prev_channel], dim=-1)
                
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
        
        # [NEW] Memory Cleanup after Epoch
        gc.collect()
        if torch.cuda.is_available():
            torch.cuda.empty_cache()
        
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
            
            # [NEW] AR Feedback Test: append zero channel (inference mode)
            if ar_enable:
                B, T_in, _ = xb.shape
                prev_channel = torch.zeros(B, T_in, ar_k, device=device)
                xb = torch.cat([xb, prev_channel], dim=-1)
                
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
    
    # [NEW] Multi-Step per-horizon metrics
    multistep_detail = {}
    if est_tick_ranges and len(est_tick_ranges) > 1 and P.ndim == 3:
        for h_idx, h_val in enumerate(est_tick_ranges):
            h_pred = P[:, h_idx, :]
            h_true = T[:, h_idx, :]
            h_err = h_pred - h_true
            multistep_detail[f"H{h_val}_mae"] = float(np.mean(np.abs(h_err)))
            multistep_detail[f"H{h_val}_rmse"] = float(np.sqrt(np.mean(h_err**2)))
        # Override primary metrics with t+5 index for baseline compatibility
        if 5 in est_tick_ranges:
            idx5 = est_tick_ranges.index(5)
            h5_err = P[:, idx5, :] - T[:, idx5, :]
            test_mae = float(np.mean(np.abs(h5_err)))
            test_rmse = float(np.sqrt(np.mean(h5_err**2)))
    
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
    
    # Nest multi-step detail inside metrics (safe extension)
    if multistep_detail:
        metrics_result["multistep_detail"] = multistep_detail
    
    logger.save_metrics(metrics_result)
    
    # Save Confusion Matrix if needed (but this is regression)
    # np.save(save_path_dir / "predictions.npy", P)
    # np.save(save_path_dir / "truths.npy", T)
    
    return metrics_result, ckpt_path



# -------------------------------------------------------------------------------------------------
# Distribution Analysis (User Request)
# -------------------------------------------------------------------------------------------------


# -------------------------------------------------------------------------------------------------
# H5 Inspection Tool (Integrated)
# -------------------------------------------------------------------------------------------------
def inspect_h5_structure(file_path):
    if not os.path.exists(file_path):
        print(f"[ERROR] File not found: {file_path}")
        return

    print(f"\n[INFO] Inspecting file: {file_path}")
    with h5py.File(file_path, 'r') as f:
        subjects = list(f.keys())
        if not subjects:
            print("  No subjects found.")
            return
        
        # Take the first subject
        sub_key = subjects[0]
        sub_group = f[sub_key]
        print(f"  Subject: {sub_key}")
        
        conditions = list(sub_group.keys())
        if not conditions:
            print("    No conditions found.")
            return
            
        cond_key = conditions[0]
        cond_item = sub_group[cond_key]
        print(f"    Condition: {cond_key} (Type: {type(cond_item)})")
        
        if not isinstance(cond_item, h5py.Group):
            print(f"    Item {cond_key} is not a group.")
            return

        levels = list(cond_item.keys())
        if not levels:
            print("      No levels found.")
            return
            
        lv_key = levels[0]
        lv_item = cond_item[lv_key]
        print(f"      Level: {lv_key} (Type: {type(lv_item)})")
        
        if not isinstance(lv_item, h5py.Group):
            print(f"      Item {lv_key} is not a group.")
            return
            
        trials = list(lv_item.keys())
        if not trials:
            print("        No trials found.")
            return
            
        trial_key = trials[0]
        trial_group = lv_item[trial_key]
        print(f"        Trial: {trial_key}")
        
        def print_tree(name, obj):
            if isinstance(obj, h5py.Group):
                print("  " * (name.count('/') + 4) + f"Group: {name.split('/')[-1]}")
            else:
                print("  " * (name.count('/') + 4) + f"Dataset: {name.split('/')[-1]} (shape: {obj.shape})")

        print("        [Sample Trial Structure]")
        trial_group.visititems(print_tree)

# -------------------------------------------------------------------------------------------------
# Distribution Analysis (User Request)
# -------------------------------------------------------------------------------------------------

def load_all_subject_data(config):
    """
    Load data for ALL subjects across ALL data sources defined in config.
    Returns a dictionary: { subject_name: { feature_name: data_array } }
    """
    # ... (rest of the function remains same, just inserting above) ...
    data_sources = {}
    if 'shared' in config and 'data_sources' in config['shared']:
        for ds_name, ds_config in config['shared']['data_sources'].items():
            prefix = ds_config.get('prefix', '')
            path = ds_config.get('path', '')
            exclude = ds_config.get('exclude_subjects', [])
            if path:
                 data_sources[prefix] = {'path': path, 'exclude_subjects': exclude}
    
    if not data_sources:
        data_path = config.get('data_path', './combined_data.h5')
        data_sources = {'': {'path': data_path, 'exclude_subjects': []}}

    # ...

    input_vars = []
    if 'shared' in config and 'input_vars' in config['shared']:
        input_vars = config['shared']['input_vars']
    elif '01_construction' in config and 'inputs' in config['01_construction']:
        input_vars = config['01_construction']['inputs']
    
    # Also include output vars for check? (Optional, user asked for inputs)
    # output_vars = config['shared']['output_vars']
    
    # Parse input vars into a list of (group, var) tuples for easier checking
    # But wait, we need to handle _dot and _ddot logic. 
    # Let's map: group -> list of var_names
    target_map = {}
    for grp, vars in input_vars:
        if grp not in target_map: target_map[grp] = []
        target_map[grp].extend(vars)
        
    all_data = {} 
    
    total_feats = sum(len(v) for v in target_map.values())
    print(f"[DIST-ANALYSIS] Loading data from {len(data_sources)} sources for {total_feats} features across {len(target_map)} groups...")
    
    for prefix, src_cfg in data_sources.items():
        h5_path = src_cfg.get('path')
        exclude_subs = src_cfg.get('exclude_subjects', [])
        
        if not os.path.exists(h5_path):
            print(f"[WARN] Data file not found: {h5_path}")
            continue
            
        with h5py.File(h5_path, 'r') as f:
            for sub_key in f.keys():
                if sub_key in exclude_subs: continue
                
                full_sub_name = f"{prefix}{sub_key}"
                
                conditions = [k for k in f[sub_key].keys() if isinstance(f[sub_key][k], h5py.Group)]
                sub_feat_data = {} # key: "group/var" -> list of arrays
                
                for cond in conditions:
                    if cond in ['subject_info', 'derived']: continue
                    cond_group = f[sub_key][cond]
                    
                    # Iterate Levels
                    for lv_name in cond_group.keys():
                        lv_group = cond_group[lv_name]
                        if not isinstance(lv_group, h5py.Group): continue
                        
                        # Iterate Trials
                        for trial_name in lv_group.keys():
                            trial_group = lv_group[trial_name]
                            if not isinstance(trial_group, h5py.Group): continue
                            
                            for grp_name, target_vars in target_map.items():
                                # Navigate to group inside trial
                                curr = trial_group
                                for p in grp_name.split('/'):
                                    if p in curr: curr = curr[p]
                                    else: curr = None; break
                                    
                                if curr:
                                    data_grp = curr
                                    
                                    # [NEW] Load Treadmill Acceleration for Analysis
                                    a_belt_analysis = 0.0
                                    v_belt_a = get_data_from_group(trial_group, "treadmill/left/speed_leftbelt")
                                    if v_belt_a is not None:
                                        a_belt_analysis = np.gradient(v_belt_a.flatten(), axis=0) * 100
                                    
                                    for feat in target_vars:
                                        val = None
                                        
                                        # 1. Direct Load
                                        if feat in data_grp:
                                            val = data_grp[feat][:]
                                        
                                        # [NEW] Fallback for Milestone 1: hip_angle -> motor_angle
                                        elif 'hip_angle' in feat:
                                            alt_name = feat.replace('hip_angle', 'motor_angle')
                                            if alt_name in data_grp:
                                                val = data_grp[alt_name][:]
                                            
                                        # 2. Advanced IMU Features
                                        elif grp_name == "robot/back_imu" and feat in ["roll", "pitch", "yaw", "vel_forward"]:
                                            try:
                                                raw_ax = data_grp['accel_x'][:]
                                                raw_ay = data_grp['accel_y'][:]
                                                raw_az = data_grp['accel_z'][:]
                                                raw_gx = data_grp['gyro_x'][:]
                                                raw_gy = data_grp['gyro_y'][:]
                                                raw_gz = data_grp['gyro_z'][:]
                                                if full_sub_name.startswith("m1_"): raw_ax = -raw_ax
                                                elif full_sub_name.startswith("m2_"): raw_az = -raw_az
                                                n = len(raw_ax); dt = 0.01; alpha = 0.98
                                                rs, ps, ys = np.zeros(n), np.zeros(n), np.zeros(n)
                                                cr, cp, cy = 0.0, 0.0, 0.0
                                                for i in range(n):
                                                    ra = np.arctan2(raw_ay[i], raw_az[i])
                                                    pa = np.arctan2(-raw_ax[i], np.sqrt(raw_ay[i]**2+raw_az[i]**2))
                                                    cr = alpha*(cr+raw_gx[i]*dt)+(1-alpha)*ra
                                                    cp = alpha*(cp+raw_gy[i]*dt)+(1-alpha)*pa
                                                    cy += raw_gz[i] * dt
                                                    rs[i], ps[i], ys[i] = cr, cp, cy
                                                
                                                if feat == "vel_forward":
                                                    alin = raw_ax + 9.81 * np.sin(ps)
                                                    if isinstance(a_belt_analysis, np.ndarray) and len(a_belt_analysis) == n:
                                                        alin = alin + a_belt_analysis
                                                    val = butter_highpass_filter(np.cumsum(alin)*dt, 0.1, 100, 2)
                                                elif feat == "roll": val = rs
                                                elif feat == "pitch": val = ps
                                                elif feat == "yaw": val = ys
                                            except: val = None

                                        # 3. Derived (_dot / _ddot)
                                        if val is None and (feat.endswith('_dot') or feat.endswith('_ddot')):
                                            if feat.endswith('_ddot'):
                                                base = feat[:-5]
                                                is_accel = True
                                            else:
                                                base = feat[:-4]
                                                is_accel = False
                                            
                                            if base in data_grp:
                                                base_val = data_grp[base][:]
                                                # 1st deriv
                                                val = np.gradient(base_val, axis=0) * 100 # fs=100 assume
                                                if feat.endswith('_ddot'):
                                                    val = np.gradient(val, axis=0) * 100
                                                    
                                        # -------------------------------------------------------------------------
                                        # [FIX] Apply Data Correction (Sign Flipping & GRF Norm) to Analysis Data
                                        # To ensure distribution analysis matches training preprocessing.
                                        # -------------------------------------------------------------------------
                                        if val is not None:
                                            # 1. Sign Flipping
                                            if full_sub_name.startswith("m1_"):
                                                if "forceplate/grf" in grp_name and feat == "x": val = -val
                                                elif "knee_angle_l" in feat or "knee_angle_r" in feat: val = -val
                                            elif full_sub_name.startswith("m2_"):
                                                if "forceplate/grf" in grp_name and feat == "z": val = -val

                                            # 2. GRF Normalization (Divide by Body Weight)
                                            if "forceplate" in grp_name.lower() and "grf" in grp_name.lower():
                                                mass = 70.0 # Default
                                                if sub_key in f and "mass" in f[sub_key].attrs:
                                                    mass = f[sub_key].attrs["mass"]
                                                bw = mass * 9.81
                                                val = val / bw

                                        if val is not None:
                                            full_key = f"{grp_name}/{feat}"
                                            if full_key not in sub_feat_data: sub_feat_data[full_key] = []
                                            sub_feat_data[full_key].append(val)
                
                # Concatenate
                if sub_feat_data:
                    all_data[full_sub_name] = {}
                    for key, val_list in sub_feat_data.items():
                        if val_list:
                            all_data[full_sub_name][key] = np.concatenate(val_list)

    return all_data

def analyze_data_distribution(config, output_dir):
    print("\n" + "="*60)
    print("  Running Data Distribution Analysis (Outlier Detection)")
    print("="*60)
    
    all_data = load_all_subject_data(config)
    if not all_data:
        print("[WARN] No data loaded for distribution analysis.")
        return

    os.makedirs(output_dir, exist_ok=True)
    features = sorted(list(set(k for sub_data in all_data.values() for k in sub_data.keys())))
    stats_summary = []
    
    for feature in features:
        plot_data = []
        for sub, data in all_data.items():
            if feature in data:
                vals = data[feature]
                # Downsample for plotting
                if len(vals) > 5000:
                    vals_plot = np.random.choice(vals, 5000, replace=False)
                else:
                    vals_plot = vals
                
                for v in vals_plot:
                    plot_data.append({'Subject': sub, 'Value': v})
                
                stats_summary.append({
                    'Feature': feature, 'Subject': sub,
                    'Mean': np.mean(vals), 'Std': np.std(vals)
                })
        
        if not plot_data: continue
            
        df_feat = pd.DataFrame(plot_data)
        
        # Violin Plot
        plt.figure(figsize=(12, 6))
        sns.violinplot(data=df_feat, x='Subject', y='Value', hue='Subject', legend=False)
        plt.xticks(rotation=45, ha='right')
        plt.title(f'Distribution: {feature}')
        plt.tight_layout()
        safe_fname = feature.replace('/', '_')
        plt.savefig(os.path.join(output_dir, f'dist_violin_{safe_fname}.png'))
        plt.close()

    # Save Stats & Check Outliers
    df_stats = pd.DataFrame(stats_summary)
    df_stats.to_csv(os.path.join(output_dir, 'subject_stats.csv'), index=False)
    
    print(f"[INFO] Distribution plots saved to {output_dir}")
    print("[INFO] Checking for outliers (Mean Shift > 2-Sigma)...")
    
    for feature in features:
        feat_stats = df_stats[df_stats['Feature'] == feature]
        overall_mean = feat_stats['Mean'].mean()
        overall_std = feat_stats['Mean'].std()
        
        threshold = 2 * overall_std
        outliers = feat_stats[np.abs(feat_stats['Mean'] - overall_mean) > threshold]
        
        if not outliers.empty:
            print(f"  [ALARM] {feature}: Detect {len(outliers)} outliers")
            for _, row in outliers.iterrows():
                z = (row['Mean'] - overall_mean) / (overall_std + 1e-9)
                print(f"    - {row['Subject']}: Z={z:.2f} (Mean={row['Mean']:.4f})")
                
    # [MEMORY] Explicit Cleanup
    del all_data
    del df_stats
    gc.collect()
    print("="*60 + "\n")

# -------------------------------------------------------------------------------------------------
# Main Execution
# -------------------------------------------------------------------------------------------------

if __name__ == "__main__":
    
    # Arg Parse
    parser = argparse.ArgumentParser()
    parser.add_argument("--rank", type=int, default=0, help="Rank of the current process")
    parser.add_argument("--total_nodes", type=int, default=1, help="Total number of nodes/GPUs")
    parser.add_argument("--config", type=str, default=None, help="Specific config file to run")
    parser.add_argument("--inspect", type=str, default=None, help="Path to H5 file to inspect structure")
    args = parser.parse_args()

    # [NEW] Inspect Mode
    if args.inspect:
        inspect_h5_structure(args.inspect)
        sys.exit(0)

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
    base_config_path_str = "configs/baseline.yaml"
    base_config = {} # Initialize an empty base_config
    
    # Check if base_config.yaml exists and if the current config is not base_config itself
    if os.path.exists(base_config_path_str) and (args.config is None or Path(args.config).name != "baseline.yaml"):
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
                
            # Merge with defaults (RECURSIVELY)
            def recursive_update(target, update):
                for k, v in update.items():
                    if isinstance(v, dict) and k in target and isinstance(target[k], dict):
                        recursive_update(target[k], v)
                    else:
                        target[k] = v
            
            merged = copy.deepcopy(base_config)
            recursive_update(merged, cfg)

            
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
    
    # [USER REQUEST] Run Distribution Analysis at the very beginning
    # Use the first valid config (or base_config) to find data sources
    if args.rank == 0:
        analysis_config = base_config
        if not analysis_config and experiments_to_run:
            analysis_config = experiments_to_run[0][1]
            
        if analysis_config:
            # We assume output dir based on first config or generic
            dist_output_dir = "compare_result/distribution_analysis"
            analyze_data_distribution(analysis_config, dist_output_dir)
            
    # Remove Global Dataset Load - Move to Experiment Loop
    # We need to rebuild dataset for EACH experiment because input_vars/output_vars might change!
    
    # [FIX] Disable Live Plotter in Headless/Cluster Env to save memory/avoid backend issues
    # live_plotter = LiveTrainingPlotter()
    live_plotter = None
    
    results = [] # Reset results list
    seeds = [42]

    # Progress bar for Experiments (Total Progress)
    # Use position=0 for the outer bar.
    for exp_name, cfg in tqdm(experiments_to_run, desc=f"GPU-{args.rank} Total", position=0):
        for sd in seeds:
            full_name = f"{exp_name}_seed{sd}"
            print(f"\n>>> Start Experiment: {full_name}")
            # live_plotter.start_session(exp_name, sd)
            
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
                
            # print(f"Data Path: {current_data_path}")
            
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
            
            # [NEW] Input LPF
            curr_input_lpf_cutoff = cfg.get("input_lpf_cutoff_hz")
            if curr_input_lpf_cutoff is None and 'shared' in cfg:
                curr_input_lpf_cutoff = cfg['shared'].get('input_lpf_cutoff_hz')
            if curr_input_lpf_cutoff is None and '02_train' in cfg and 'data' in cfg['02_train']:
                curr_input_lpf_cutoff = cfg['02_train']['data'].get('input_lpf_cutoff_hz')
            
            curr_input_lpf_order = cfg.get("input_lpf_order", 4)
            if 'shared' in cfg and cfg['shared'].get('input_lpf_order'):
                curr_input_lpf_order = cfg['shared'].get('input_lpf_order')
            elif '02_train' in cfg and 'data' in cfg['02_train'] and cfg['02_train']['data'].get('input_lpf_order'):
                curr_input_lpf_order = cfg['02_train']['data'].get('input_lpf_order')
                    
            # Dataset Universe (Full list of subs/conds available to read)
            curr_sub_names = cfg.get("subjects") or (base_config.get("subjects") if base_config else None)
            curr_cond_names = cfg.get("conditions") or (base_config.get("conditions") if base_config else None)
                    
            # --- TCN Dynamic Config ---
            if "tcn_channels" in cfg:
                curr_tcn_channels = cfg["tcn_channels"]
            elif "02_train" in cfg and cfg["02_train"].get("model") and "channels" in cfg["02_train"]["model"]:
                curr_tcn_channels = cfg["02_train"]["model"]["channels"]
            elif "02_train" in cfg and "data" in cfg["02_train"] and "channels" in cfg["02_train"]["data"]:
                 curr_tcn_channels = cfg["02_train"]["data"]["channels"]
            elif "shared" in cfg and cfg["shared"].get("model") and "channels" in cfg["shared"]["model"]:
                curr_tcn_channels = cfg["shared"]["model"]["channels"]
            else:
                n_layers = cfg.get("tcn_layers", base_config.get("tcn_layers", 3) if base_config else 3)
                n_hidden = cfg.get("tcn_hidden", base_config.get("tcn_hidden", 64) if base_config else 64)
                curr_tcn_channels = [n_hidden] * n_layers
            
            cfg["tcn_channels"] = curr_tcn_channels
            
            # CV Logic: Forced LOSO
            sub_runs = []
            all_subs = sorted(list(set(curr_sub_names))) if curr_sub_names else []
            loso_filter = cfg.get("03_eval", {}).get("split", {}).get("loso_subjects")
            if not loso_filter: loso_filter = cfg.get("loso_subjects")
                
            for i in range(len(all_subs)):
                test_sub = all_subs[i]
                if loso_filter and test_sub not in loso_filter: continue
                val_sub = all_subs[i-1] 
                train_subs = [s for s in all_subs if s != test_sub and s != val_sub]
                sub_runs.append({
                    "name": f"{exp_name}_Test-{test_sub}",
                    "train": train_subs, "val": [val_sub], "test": [test_sub]
                })

            c_in_vars = parse_vars(curr_input_vars)
            c_out_vars = parse_vars(curr_output_vars)
            
            print(f"\n[INFO] Starting Cross-Validation Loops: {len(sub_runs)} folds (Config: {exp_name})")
            
            for run_meta in sub_runs:
                final_exp_name = run_meta["name"]
                full_name_seed = f"{final_exp_name}_seed{sd}"
                print(f"\n>>> Start Experiment: {full_name_seed}")
                
                # Load Datasets
                data_cfg_now = cfg.get("02_train", {}).get("data", {})
                use_phys_vel = data_cfg_now.get("use_physical_velocity_model", False)
                use_gait_ph  = data_cfg_now.get("use_gait_phase", False)
                
                X_train, Y_train = build_nn_dataset_multi(
                    cfg, curr_sub_names, curr_cond_names, c_in_vars, c_out_vars,
                    curr_window, time_window_output, stride,
                    subject_selection=make_subject_selection(run_meta["train"]),
                    condition_selection=CONDITION_SELECTION, lpf_cutoff=curr_lpf_cutoff, lpf_order=curr_lpf_order,
                    est_tick_ranges=cfg.get("est_tick_ranges"),
                    use_physical_velocity_model=use_phys_vel, use_gait_phase=use_gait_ph,
                    input_lpf_cutoff=curr_input_lpf_cutoff, input_lpf_order=curr_input_lpf_order
                )
                X_val, Y_val = build_nn_dataset_multi(
                    cfg, curr_sub_names, curr_cond_names, c_in_vars, c_out_vars,
                    curr_window, time_window_output, stride,
                    subject_selection=make_subject_selection(run_meta["val"]),
                    condition_selection=CONDITION_SELECTION, lpf_cutoff=curr_lpf_cutoff, lpf_order=curr_lpf_order,
                    est_tick_ranges=cfg.get("est_tick_ranges"),
                    use_physical_velocity_model=use_phys_vel, use_gait_phase=use_gait_ph,
                    input_lpf_cutoff=curr_input_lpf_cutoff, input_lpf_order=curr_input_lpf_order
                )
                X_test, Y_test = build_nn_dataset_multi(
                    cfg, curr_sub_names, curr_cond_names, c_in_vars, c_out_vars,
                    curr_window, time_window_output, stride,
                    subject_selection=make_subject_selection(run_meta["test"]),
                    condition_selection=CONDITION_SELECTION, lpf_cutoff=curr_lpf_cutoff, lpf_order=curr_lpf_order,
                    est_tick_ranges=cfg.get("est_tick_ranges"),
                    use_physical_velocity_model=use_phys_vel, use_gait_phase=use_gait_ph,
                    input_lpf_cutoff=curr_input_lpf_cutoff, input_lpf_order=curr_input_lpf_order
                )

                if len(X_train) == 0:
                    print(f"[ERROR] No training data for {full_name_seed}.")
                    continue
                    
                local_save_dir = f"experiments/{full_name_seed}"
                os.makedirs(local_save_dir, exist_ok=True)
                    
                # Normalization
                use_norm = cfg.get("use_input_norm", True)
                if not use_norm and '02_train' in cfg:
                     use_norm = cfg['02_train'].get('model',{}).get('use_norm', True)
                     
                if use_norm:
                    print("[INFO] Computing GLOBAL mean/std for Input Normalization...")
                    all_train = np.concatenate(X_train, axis=0)
                    mean = np.mean(all_train, axis=0)
                    std = np.std(all_train, axis=0)
                    scale = std
                    scale[scale < 1e-8] = 1.0
                    np.savez(os.path.join(local_save_dir, "scaler.npz"), mean=mean, scale=scale)
                    
                    # [MEMORY] Clear all_train immediately
                    del all_train
                    gc.collect()

                    for i in range(len(X_train)): X_train[i] = (X_train[i] - mean) / scale
                    for i in range(len(X_val)): X_val[i] = (X_val[i] - mean) / scale
                    for i in range(len(X_test)): X_test[i] = (X_test[i] - mean) / scale
                else:
                    normalize_type = cfg.get("02_train", {}).get("data", {}).get("normalize_type", "global")
                    if normalize_type == "subject":
                        print("[INFO] Applying SUBJECT-WISE Normalization...")
                        for data_list in [X_train, X_val, X_test]:
                            for i in range(len(data_list)):
                                m, s = np.mean(data_list[i], axis=0), np.std(data_list[i], axis=0) + 1e-8
                                data_list[i] = (data_list[i] - m) / s

                feat_names_simple = [f"{gp}/{v}" for gp, vs in c_in_vars for v in vs]
                
                metrics, ckpt_path = train_experiment(
                    X_train, Y_train, X_val, Y_val, X_test, Y_test,
                    cfg, seed=sd, save_dir=local_save_dir, live_plotter=None,
                    scaler_mean=mean if use_norm else None, scaler_std=scale if use_norm else None,
                    feature_names=feat_names_simple
                )
                
                # [MEMORY] Clear big lists and collect garbage after fold
                del X_train, Y_train, X_val, Y_val, X_test, Y_test
                gc.collect()
                if torch.cuda.is_available(): torch.cuda.empty_cache()

                print(f"[RESULT] {full_name_seed} | test_mae={metrics.get('test_mae', 0):.6f}")
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
    elif "shared" in best_cfg and "output_vars" in best_cfg["shared"]:
        raw_out = best_cfg["shared"]["output_vars"]
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

    X_test_best, Y_test_best = build_nn_dataset_multi(
        best_cfg, best_sub_names, best_cond_names, c_in_vars, c_out_vars,
        best_window, best_cfg.get("time_window_output", 1), best_cfg.get("stride", 5),
        subject_selection=make_subject_selection(best_sub_names), # Use all relevant subjects for global check
        condition_selection=CONDITION_SELECTION, lpf_cutoff=best_lpf_cutoff, lpf_order=best_lpf_order,
        est_tick_ranges=best_cfg.get("est_tick_ranges", None)
    )
    
    best_horizon = len(best_cfg.get("est_tick_ranges")) if best_cfg.get("est_tick_ranges") else best_cfg.get("time_window_output", 10)
    
    # [FIX] Extract Model Config correctly (Nested priority)
    # Priority: 02_train.model > shared.model > root
    model_cfg = best_cfg.get("02_train", {}).get("model", {})
    if not model_cfg:
        model_cfg = best_cfg.get("shared", {}).get("model", {})
    if not model_cfg:
        model_cfg = best_cfg.get("model", {})

    tcn_channels = model_cfg.get("channels") or best_cfg.get("tcn_channels") or best_cfg.get("channels")
    kernel_size = model_cfg.get("kernel_size") or best_cfg.get("kernel_size")
    mlp_hidden = model_cfg.get("head_hidden") or best_cfg.get("head_hidden") or best_cfg.get("mlp_hidden")
    head_dropout = model_cfg.get("head_dropout") or best_cfg.get("head_dropout")
    dropout_p = float(model_cfg.get("dropout") or best_cfg.get("dropout", 0.1))
    
    # Norm types
    model_norm = model_cfg.get("model_norm")
    
    if not X_test_best:
        print("[WARN] No data found for Feature Importance Re-loading. Skipping.")
    else:
        best_model = TCN_MLP(
            input_dim=X_test_best[0].shape[1],
            output_dim=Y_test_best[0].shape[1],
            horizon=best_horizon, 
            channels=tcn_channels,
            kernel_size=kernel_size,
            dropout=dropout_p,
            head_dropout=head_dropout,
            mlp_hidden=mlp_hidden,
            use_input_norm=best_cfg.get("use_input_norm", True), 
            tcn_norm=model_norm,
            mlp_norm=model_norm
        )
        
        # Load Weights
        ckpt = torch.load(best_ckpt, map_location='cpu')
        best_model.load_state_dict(ckpt['state_dict'])
        
        device = "cuda" if torch.cuda.is_available() else "cpu"
        best_model.to(device)
        best_model.eval()
        
        # 3. Construct Feature Names
        feature_names = []
        for gpath, vars in c_in_vars:
            if 'Back_imu' in gpath or 'back_imu' in gpath:
                prefix = "IMU"
            elif 'Robot' in gpath or 'robot' in gpath:
                prefix = "Robot_L" if 'left' in gpath else ("Robot_R" if 'right' in gpath else "Robot")
            elif 'grf' in gpath:
                prefix = f"GRF_{'L' if 'left' in gpath else 'R'}"
            elif 'kin_q' in gpath:
                prefix = "Deg"
            else:
                prefix = gpath.split('/')[-1]
                
            for v in vars:
                v_clean = v.replace('ankle_angle', 'Ankle').replace('knee_angle', 'Knee').replace('hip_', 'Hip_')
                v_clean = v_clean.replace('flexion', 'Flex').replace('adduction', 'Add').replace('rotation', 'Rot')
                v_clean = v_clean.replace('_l', '_L').replace('_r', '_R')
                feature_names.append(f"{prefix}_{v_clean}")
                
        # 4. Compute Importance (Permutation)
        print(f"  Calculating Importances for {len(feature_names)} features...")
        
        def calculate_permutation_importance(model, dataloader, feature_names, device):
            model.eval()
            N_feats = len(feature_names)
            total_samples = 0
            
            # Accumulators
            total_base_loss = 0.0
            feat_loss_sums = torch.zeros(N_feats, device=device)
            
            print(f"  Calculating Importances (Batch-wise, {N_feats} features)...")
            
            with torch.no_grad():
                for xb, yb in dataloader:
                    xb, yb = xb.to(device), yb.to(device)
                    B = xb.size(0)
                    total_samples += B
                    
                    # 1. Base Prediction
                    pred_base = model(xb)
                    
                    # Shape Alignment
                    if pred_base.dim() == 2 and yb.dim() == 3 and yb.shape[1] == 1: 
                        yb_s = yb.squeeze(1)
                    elif pred_base.dim() == 3 and yb.dim() == 2 and pred_base.shape[1] == 1: 
                        pred_base = pred_base.squeeze(1)
                        yb_s = yb
                    else:
                        yb_s = yb

                    # Base Loss (Sum)
                    base_loss = torch.sum(torch.abs(pred_base - yb_s)).item()
                    total_base_loss += base_loss
                    
                    # 2. Permutation (Intra-batch)
                    for i in range(N_feats):
                        original_col = xb[:, :, i].clone()
                        
                        # Shuffle indices within batch
                        idx = torch.randperm(B, device=device)
                        xb[:, :, i] = xb[idx, :, i]
                        
                        pred_p = model(xb)
                        # Shape Alignment for Permuted (assume same as base)
                        if pred_p.dim() == 3 and pred_p.shape[1] == 1: pred_p = pred_p.squeeze(1)
                        
                        # Permuted Loss sum
                        p_loss = torch.sum(torch.abs(pred_p - yb_s))
                        feat_loss_sums[i] += p_loss
                        
                        # Restore
                        xb[:, :, i] = original_col
            
            # Compute Averages
            base_mae = total_base_loss / (total_samples + 1e-9)
            print(f"  Baseline MAE: {base_mae:.6f}")
            
            feat_maes = feat_loss_sums.cpu().numpy() / (total_samples + 1e-9)
            importances = feat_maes - base_mae
            
            for i, imp in enumerate(importances):
                print(f"    Feat {i}: +{imp:.6f}", end='\r')
            print("")
            return importances

        # [MEMORY-FIX] Restore original stride but use Batch Processing for safety
        fi_stride = best_cfg.get("stride", 5)
        
        imp_test_ds = LazyWindowDataset(
            X_test_best, Y_test_best, 
            best_window, best_cfg.get("time_window_output", 1), fi_stride, 
            target_mode="sequence", est_tick_ranges=best_cfg.get("est_tick_ranges", None)
        )
        
        # [MEMORY-FIX] Explicit GC & Shuffle=True for intra-batch permutation
        import gc
        gc.collect()
        torch.cuda.empty_cache()
        
        imp_loader = DataLoader(imp_test_ds, batch_size=512, shuffle=True, num_workers=0)
        importances = calculate_permutation_importance(best_model, imp_loader, feature_names, device)
        
        # 5. Plot
        n_feats = len(importances)
        sorted_idx = np.argsort(importances)[::-1]
        fig_h = max(8, n_feats * 0.25)
        plt.figure(figsize=(10, fig_h))
        plt.barh(np.arange(n_feats), importances[sorted_idx][::-1], color='skyblue')
        plt.yticks(np.arange(n_feats), np.array(feature_names)[sorted_idx][::-1] if len(feature_names)==n_feats else np.arange(n_feats))
        plt.xlabel("Permutation Importance (MAE Increase)")
        plt.title(f"Feature Importance: {best_name}")
        plt.tight_layout()
        plt.savefig("experiments/feature_importance_best.png")
        print(">>> Feature Importance Saved: experiments/feature_importance_best.png")
