
import os
import h5py
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt

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

def visualize_features(config):
    """
    Visualizes Input Features for M1 vs M2 consistency check.
    Target: S004
    Condition: M1 (level_08mps) vs M2 (level_075mps)
    """
    print(f"\n[VISUALIZATION] Starting Input Feature Comparison (M1 vs M2)...")
    
    # 1. Setup Targets
    sub_target = "S004"
    cond_m1 = "level_08mps"
    cond_m2 = "level_075mps"
    
    ds_m1 = config.get("shared", {}).get("data_sources", {}).get("milestone1", {})
    ds_m2 = config.get("shared", {}).get("data_sources", {}).get("milestone2", {})
    
    path_m1 = ds_m1.get("path", "./combined_data_milestone1.h5")
    path_m2 = ds_m2.get("path", "./combined_data.h5")
    
    save_dir = "compare_result/feature_plot"
    os.makedirs(save_dir, exist_ok=True)
    
    # 2. Get Input Vars
    raw_vars = config.get("input_vars")
    if not raw_vars and 'shared' in config:
        raw_vars = config['shared'].get("input_vars")
    if not raw_vars and '01_construction' in config:
        raw_vars = config['01_construction'].get("inputs")
        
    if not raw_vars:
        print("[WARN] No input_vars found for visualization.")
        # return # Continue to check output_vars
        
    var_list = parse_vars(raw_vars)
    
    # [NEW] Add Output Vars
    raw_outs = config.get("output_vars")
    if not raw_outs and 'shared' in config:
        raw_outs = config['shared'].get("output_vars")
    if not raw_outs and '01_construction' in config:
        raw_outs = config['01_construction'].get("outputs")
        
    if raw_outs:
        out_list = parse_vars(raw_outs)
        var_list.extend(out_list)
        print(f"[INFO] Added {len(out_list)} output variable groups for visualization.")
    
    if not var_list:
        print("[WARN] No variables (input or output) found to visualize.")
        return
    fs = 100
    
    # Helper to load a specific variable from trial group
    def load_var(grp, v_name, g_path_context):
        col_data = None
        
        # 1. Direct Load
        if v_name in grp:
            col_data = grp[v_name][:]
            
        # 2. Derived Logic (Simple Derivative)
        elif v_name.endswith('_dot') or v_name.endswith('_ddot'):
            if v_name.endswith('_ddot'):
                base_name = v_name[:-5]
                is_accel = True
            else:
                base_name = v_name[:-4]
                is_accel = False
            
            if base_name in grp:
                base_data = grp[base_name][:]
                d1 = np.gradient(base_data, axis=0) * fs
                if is_accel:
                    target_data = np.gradient(d1, axis=0) * fs
                else:
                    target_data = d1
                
                # Apply 30Hz LPF for derivatives (same as training)
                col_data = butter_lowpass_filter(target_data, cutoff=30.0, fs=fs, order=4)
                
        # 3. Fallback: hip_angle -> motor_angle (M1 specific often)
        elif 'hip_angle' in v_name:
            alt_name = v_name.replace('hip_angle', 'motor_angle')
            if alt_name in grp:
                col_data = grp[alt_name][:]
                
        return col_data

    # Data Containers
    data_m1 = {} # "gpath/var" -> array
    data_m2 = {}
    
    # 3. Extract M1
    if os.path.exists(path_m1):
        with h5py.File(path_m1, 'r') as f:
            if sub_target in f and cond_m1 in f[sub_target]:
                # Take first level/trial
                c_grp = f[sub_target][cond_m1]
                lk = list(c_grp.keys())[0] # lv0
                tk = list(c_grp[lk].keys())[0] # trial_01
                trial_grp = c_grp[lk][tk]
                
                for gpath, vars in var_list:
                    # Navigate
                    curr = trial_grp
                    valid = True
                    for p in gpath.split('/'):
                        if p in curr: curr = curr[p]
                        else: valid=False; break
                    
                    if valid:
                        for v in vars:
                            val = load_var(curr, v, gpath)
                            if val is not None:
                                key = f"{gpath}/{v}"
                                data_m1[key] = val
    else:
        print(f"[WARN] M1 file not found: {path_m1}")

    # 4. Extract M2
    if os.path.exists(path_m2):
        with h5py.File(path_m2, 'r') as f:
            if sub_target in f and cond_m2 in f[sub_target]:
                c_grp = f[sub_target][cond_m2]
                lk = list(c_grp.keys())[0]
                tk = list(c_grp[lk].keys())[0]
                trial_grp = c_grp[lk][tk]
                
                for gpath, vars in var_list:
                    curr = trial_grp
                    valid = True
                    for p in gpath.split('/'):
                        if p in curr: curr = curr[p]
                        else: valid=False; break
                    if valid:
                        for v in vars:
                            val = load_var(curr, v, gpath)
                            if val is not None:
                                key = f"{gpath}/{v}"
                                data_m2[key] = val
    else:
        print(f"[WARN] M2 file not found: {path_m2}")

    # 5. Plot Comparison
    # Iterate all requested keys
    all_keys = sorted(list(set(list(data_m1.keys()) + list(data_m2.keys()))))
    
    for key in all_keys:
        d1 = data_m1.get(key)
        d2 = data_m2.get(key)
        
        # Determine shared Y-limits
        vis_min, vis_max = None, None
        
        # Collect all valid data points to find min/max
        all_vals = []
        if d1 is not None: all_vals.append(d1)
        if d2 is not None: all_vals.append(d2)
        
        if all_vals:
            concat_vals = np.concatenate(all_vals)
            v_min, v_max = np.min(concat_vals), np.max(concat_vals)
            margin = (v_max - v_min) * 0.1 if v_max != v_min else 1.0
            vis_min = v_min - margin
            vis_max = v_max + margin
        
        plt.figure(figsize=(10, 6))
        
        # Subplot 1: M1
        ax1 = plt.subplot(2, 1, 1)
        if d1 is not None:
            plt.plot(d1, label=f"M1: {sub_target}/{cond_m1}", color='blue', alpha=0.7)
            plt.title(f"{key} - Milestone 1")
            plt.legend()
            plt.grid(True)
            if vis_min is not None: plt.ylim(vis_min, vis_max)
        else:
            plt.text(0.5, 0.5, "Data Not Found in M1", ha='center')
            
        # Subplot 2: M2
        ax2 = plt.subplot(2, 1, 2)
        if d2 is not None:
            plt.plot(d2, label=f"M2: {sub_target}/{cond_m2}", color='orange', alpha=0.7)
            plt.title(f"{key} - Milestone 2")
            plt.legend()
            plt.grid(True)
            if vis_min is not None: plt.ylim(vis_min, vis_max)
        else:
            plt.text(0.5, 0.5, "Data Not Found in M2", ha='center')
            
        plt.tight_layout()
        safe_name = key.replace('/', '_').replace(" ", "")
        out_path = os.path.join(save_dir, f"{safe_name}.png")
        plt.savefig(out_path)
        plt.close()
        # print(f"[saved] {out_path}")
    
    print(f"[VISUALIZATION] Saved {len(all_keys)} feature distribution plots to {save_dir}")
    
    # ---------------------------------------------------------
    # [NEW] Joint Angle Comparison (Mocap vs Robot)
    # ---------------------------------------------------------
    print("\n[VISUALIZATION] Starting Mocap vs Robot Joint Angle Comparison...")
    
    # Define pairs to plot
    # (Title, Robot_Key, Mocap_Key)
    # Note: Keys are relative to trial group, so we need full path or partial path search
    plot_pairs = [
        ("Left Hip Angle", "robot/left/hip_angle", "mocap/kin_q/hip_flexion_l"),
        ("Right Hip Angle", "robot/right/hip_angle", "mocap/kin_q/hip_flexion_r")
    ]
    
    save_dir_joint = "compare_result/feature_plot" # As requested
    os.makedirs(save_dir_joint, exist_ok=True)
    
    def extract_and_plot_joint(f_path, label_prefix):
        if not os.path.exists(f_path): return
        
        with h5py.File(f_path, 'r') as f:
            if sub_target in f and cond_m1 in f[sub_target]: # reuse cond_m1/m2 logic?
                 # Actually, let's just use the first available condition/trial to show the alignment
                 # The user didn't specify condition, just "make a png". 
                 # We will use the same target subject/condition as above for consistency.
                 
                 # Determine Condition based on label_prefix (M1 or M2)
                 target_cond = cond_m1 if "Milestone 1" in label_prefix else cond_m2
                 
                 if target_cond not in f[sub_target]:
                     print(f"  -> Condition {target_cond} not found in {label_prefix}")
                     return

                 c_grp = f[sub_target][target_cond]
                 lk = list(c_grp.keys())[0]
                 tk = list(c_grp[lk].keys())[0]
                 trial_grp = c_grp[lk][tk]
                 
                 # Plot Pairs
                 for title, r_key, m_key in plot_pairs:
                     # Load Data
                     # We need to find the keys. define helper to find key in group hierarchy
                     def find_dset(g, partial_key):
                         # 1. Exact match attempt
                         if partial_key in g: return g[partial_key][:]
                         
                         # 2. Split and traverse
                         curr = g
                         valid = True
                         for p in partial_key.split('/'):
                             if p in curr: curr = curr[p]
                             else: valid = False; break
                         if valid and isinstance(curr, h5py.Dataset): return curr[:]
                         
                         # 3. Fallback: hip_angle -> motor_angle (Common in M1)
                         if 'hip_angle' in partial_key:
                             alt_key = partial_key.replace('hip_angle', 'motor_angle')
                             # Try traversing with alt_key
                             curr = g
                             valid = True
                             for p in alt_key.split('/'):
                                 if p in curr: curr = curr[p]
                                 else: valid = False; break
                             if valid and isinstance(curr, h5py.Dataset):
                                 print(f"      [INFO] Used fallback '{alt_key}' for '{partial_key}'")
                                 return curr[:]
                         
                         return None

                     r_data = find_dset(trial_grp, r_key)
                     m_data = find_dset(trial_grp, m_key)
                     
                     if r_data is not None and m_data is not None:
                         fig, axes = plt.subplots(2, 1, figsize=(10, 10))
                         
                         # 1. Full Duration
                         axes[0].plot(m_data, label='Mocap (Ground Truth)', color='black', alpha=0.7)
                         axes[0].plot(r_data, label='Robot (Input)', color='red', linestyle='--', alpha=0.8)
                         axes[0].set_title(f"{label_prefix}: {title} Alignment Check (Full)\n({sub_target}/{target_cond})")
                         axes[0].legend()
                         axes[0].grid(True, alpha=0.3)
                         axes[0].set_ylabel("Angle (deg)")
                         
                         # 2. Zoomed (Middle 10s)
                         # fs = 100 assumed
                         mid_idx = len(m_data) // 2
                         half_window = 500 # 5 seconds * 100 Hz
                         start_idx = max(0, mid_idx - half_window)
                         end_idx = min(len(m_data), mid_idx + half_window)
                         
                         if end_idx > start_idx:
                             t_zoom = np.arange(start_idx, end_idx)
                             axes[1].plot(t_zoom, m_data[start_idx:end_idx], label='Mocap', color='black', alpha=0.7)
                             axes[1].plot(t_zoom, r_data[start_idx:end_idx], label='Robot', color='red', linestyle='--', alpha=0.8)
                             axes[1].set_title(f"Zoomed (Middle 10s: {start_idx/100:.1f}s - {end_idx/100:.1f}s)")
                             axes[1].legend()
                             axes[1].grid(True, alpha=0.3)
                             axes[1].set_ylabel("Angle (deg)")
                             axes[1].set_xlabel("Time (ticks)")
                         else:
                             axes[1].text(0.5, 0.5, "Data to short for zoom", ha='center')
                         
                         plt.tight_layout()
                         
                         # Save
                         safe_title = title.replace(" ", "_").lower()
                         safe_prefix = label_prefix.split()[0].lower() + label_prefix.split()[1] # M1 or M2
                         fname = f"{safe_prefix}_{safe_title}_alignment.png"
                         out_p = os.path.join(save_dir_joint, fname)
                         plt.savefig(out_p)
                         plt.close()
                         print(f"  -> Saved {out_p}")
                     else:
                         print(f"  -> Data missing for {title} in {label_prefix} (R={r_data is not None}, M={m_data is not None})")

    # Run for M1
    extract_and_plot_joint(path_m1, "Milestone 1")
    # Run for M2
    extract_and_plot_joint(path_m2, "Milestone 2")
