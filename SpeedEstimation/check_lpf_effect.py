
import h5py
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt
import os

def butter_lowpass_filter(data, cutoff, fs, order=4):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    if data.ndim == 1:
        y = filtfilt(b, a, data)
    else:
        y = filtfilt(b, a, data, axis=0)
    return y



import yaml
import sys

def check_lpf_effect_all_conditions(data_path, config_path, subject="S004"):
    """
    Iterates through all conditions for a given subject and plots LPF effect using Config settings.
    """
    if not os.path.exists(data_path):
        print(f"File not found: {data_path}")
        return
        
    # Load Config
    with open(config_path, 'r') as f:
        cfg = yaml.safe_load(f)
        
    cutoff = cfg.get("lpf_cutoff")
    if cutoff is None and "01_construction" in cfg:
        cutoff = cfg["01_construction"].get("lpf_cutoff")
    if cutoff is None: cutoff = 0.5
    
    order = cfg.get("lpf_order")
    if order is None and "01_construction" in cfg:
        order = cfg["01_construction"].get("lpf_order")
    if order is None: order = 4
    fs = 100.0
    
    print(f"[INFO] Using LPF settings from {config_path}: Cutoff={cutoff}Hz, Order={order}")
    print(f"[INFO] Analyzing subject: {subject} from {data_path}")

    with h5py.File(data_path, 'r') as f:
        if subject not in f:
            print(f"Subject {subject} not found in H5.")
            return
            
        conditions = list(f[subject].keys())
        
        output_dir = "filtered_velocity"
        os.makedirs(output_dir, exist_ok=True)
        
        for cond in conditions:
            if cond == 'sub_info': continue
            
            # Find Trial Group
            lv_keys = list(f[subject][cond].keys())
            trial_grp = None
            for lv in lv_keys:
                if isinstance(f[subject][cond][lv], h5py.Group):
                    for tk in f[subject][cond][lv].keys():
                        if 'trial' in tk:
                            trial_grp = f[subject][cond][lv][tk]
                            break
                if trial_grp: break
            
            if trial_grp is None: continue

            # Load Velocity
            target_path = 'common/v_Y_true'
            curr = trial_grp
            parts = target_path.split('/')
            valid = True
            for p in parts:
                if p in curr: curr = curr[p]
                else: valid=False; break
            
            if valid:
                raw_vel = np.nan_to_num(curr[:])
            else:
                continue
                
            # Filter
            filtered_vel = butter_lowpass_filter(raw_vel, cutoff, fs, order=order)
                
            # Plot
            N = len(raw_vel)
            time = np.arange(N) / fs
            
            fig, axes = plt.subplots(2, 1, figsize=(12, 10))
            
            axes[0].plot(time, raw_vel, label='Raw', color='gray', alpha=0.5)
            axes[0].plot(time, filtered_vel, label=f'LPF {cutoff}Hz', color='red', linewidth=2)
            axes[0].set_title(f"Full Duration: {subject} - {cond} (Cutoff: {cutoff}Hz)")
            axes[0].legend()
            
            # Zoom
            start_t, end_t = 10.0, 25.0
            if time[-1] < 25.0: start_t, end_t = 0, min(15.0, time[-1])
            
            mask = (time >= start_t) & (time <= end_t)
            if np.sum(mask) > 0:
                axes[1].plot(time[mask], raw_vel[mask], label='Raw', color='gray', alpha=0.5)
                axes[1].plot(time[mask], filtered_vel[mask], label=f'LPF {cutoff}Hz', color='red', linewidth=2)
                axes[1].set_title(f"Zoomed ({start_t}s - {end_t}s)")
                axes[1].legend()
            
            plt.tight_layout()
            safe_cond = cond.replace('/', '_')
            save_path = os.path.join(output_dir, f"{subject}_{safe_cond}_lpf_{cutoff}Hz.png")
            plt.savefig(save_path)
            plt.close()
            print(f"Saved {save_path}")

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=str, default="configs/reference_baseline/baseline.yaml")
    parser.add_argument("--data", type=str, default="combined_data.h5")
    args = parser.parse_args()
    
    check_lpf_effect_all_conditions(args.data, args.config, subject="S004")
