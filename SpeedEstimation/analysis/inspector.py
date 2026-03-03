import os
import gc
import numpy as np
import h5py
import matplotlib.pyplot as plt
import pandas as pd
import seaborn as sns

from data.utils import get_data_from_group, get_ground_contact, butter_highpass_filter


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


def load_all_subject_data(config):
    """
    Load data for ALL subjects across ALL data sources defined in config.
    Returns a dictionary: { subject_name: { feature_name: data_array } }
    """
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

    input_vars = []
    if 'shared' in config and 'input_vars' in config['shared']:
        input_vars = config['shared']['input_vars']
    elif '01_construction' in config and 'inputs' in config['01_construction']:
        input_vars = config['01_construction']['inputs']

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
                sub_feat_data = {}

                for cond in conditions:
                    if cond in ['subject_info', 'derived']: continue
                    cond_group = f[sub_key][cond]

                    for lv_name in cond_group.keys():
                        lv_group = cond_group[lv_name]
                        if not isinstance(lv_group, h5py.Group): continue

                        for trial_name in lv_group.keys():
                            trial_group = lv_group[trial_name]
                            if not isinstance(trial_group, h5py.Group): continue

                            for grp_name, target_vars in target_map.items():
                                curr = trial_group
                                for p in grp_name.split('/'):
                                    if p in curr: curr = curr[p]
                                    else: curr = None; break

                                if curr:
                                    data_grp = curr

                                    a_belt_analysis = 0.0
                                    v_belt_a = get_data_from_group(trial_group, "treadmill/left/speed_leftbelt")
                                    if v_belt_a is not None:
                                        a_belt_analysis = np.gradient(v_belt_a.flatten(), axis=0) * 100

                                    for feat in target_vars:
                                        val = None

                                        if feat in data_grp:
                                            val = data_grp[feat][:]

                                        elif 'hip_angle' in feat:
                                            alt_name = feat.replace('hip_angle', 'motor_angle')
                                            if alt_name in data_grp:
                                                val = data_grp[alt_name][:]

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

                                        if val is None and (feat.endswith('_dot') or feat.endswith('_ddot')):
                                            if feat.endswith('_ddot'):
                                                base = feat[:-5]
                                                is_accel = True
                                            else:
                                                base = feat[:-4]
                                                is_accel = False

                                            if base in data_grp:
                                                base_val = data_grp[base][:]
                                                val = np.gradient(base_val, axis=0) * 100
                                                if feat.endswith('_ddot'):
                                                    val = np.gradient(val, axis=0) * 100

                                        if val is not None:
                                            if full_sub_name.startswith("m1_"):
                                                if "forceplate/grf" in grp_name and feat == "x": val = -val
                                                elif "knee_angle_l" in feat or "knee_angle_r" in feat: val = -val
                                            elif full_sub_name.startswith("m2_"):
                                                if "forceplate/grf" in grp_name and feat == "z": val = -val

                                            if "forceplate" in grp_name.lower() and "grf" in grp_name.lower():
                                                mass = 70.0
                                                if sub_key in f and "mass" in f[sub_key].attrs:
                                                    mass = f[sub_key].attrs["mass"]
                                                bw = mass * 9.81
                                                val = val / bw

                                        if val is not None:
                                            full_key = f"{grp_name}/{feat}"
                                            if full_key not in sub_feat_data: sub_feat_data[full_key] = []
                                            sub_feat_data[full_key].append(val)

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

        plt.figure(figsize=(12, 6))
        sns.violinplot(data=df_feat, x='Subject', y='Value', hue='Subject', legend=False)
        plt.xticks(rotation=45, ha='right')
        plt.title(f'Distribution: {feature}')
        plt.tight_layout()
        safe_fname = feature.replace('/', '_')
        plt.savefig(os.path.join(output_dir, f'dist_violin_{safe_fname}.png'))
        plt.close()

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

    del all_data
    del df_stats
    gc.collect()
    print("="*60 + "\n")
