import os
import numpy as np
import h5py

from .utils import (
    fs, butter_lowpass_filter, butter_highpass_filter,
    get_ground_contact, get_data_from_group,
)

# ---------------------------------------------------------------------------
# Global IMU Transformation helpers
# ---------------------------------------------------------------------------
_GLOBAL_IMU_VARS = frozenset([
    'global_acc_x', 'global_acc_y', 'global_acc_z',
    'global_gyr_x', 'global_gyr_y', 'global_gyr_z',
    'global_vel_y', 'global_vel_treadmill',
])


def _quat_to_rotmat(q_wxyz: np.ndarray) -> np.ndarray:
    """(4,) wxyz → (3,3) rotation matrix"""
    w, x, y, z = q_wxyz
    return np.array([
        [1 - 2*(y*y + z*z),   2*(x*y - z*w),   2*(x*z + y*w)],
        [  2*(x*y + z*w), 1 - 2*(x*x + z*z),   2*(y*z - x*w)],
        [  2*(x*z - y*w),   2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ])


def compute_global_imu(accel: np.ndarray, gyro: np.ndarray,
                       quat_wxyz: np.ndarray, calib_n: int = 100):
    """Quaternion-based global frame (X=right, Y=forward, Z=up). Gravity removed from acc.
    Fully vectorized — no Python loops over samples."""
    q_ref = np.mean(quat_wxyz[:calib_n], axis=0)
    q_ref /= (np.linalg.norm(q_ref) + 1e-12)
    R_ref_inv = _quat_to_rotmat(q_ref).T          # (3, 3)
    g_global = np.array([0.0, 0.0, 9.81])

    # Normalize all quaternions at once: (N, 4)
    norms = np.linalg.norm(quat_wxyz, axis=1, keepdims=True) + 1e-12
    q = quat_wxyz / norms
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]

    # Build (N, 3, 3) rotation matrices via broadcasting
    R = np.stack([
        np.stack([1 - 2*(y*y + z*z),   2*(x*y - z*w),   2*(x*z + y*w)], axis=1),
        np.stack([  2*(x*y + z*w), 1 - 2*(x*x + z*z),   2*(y*z - x*w)], axis=1),
        np.stack([  2*(x*z - y*w),   2*(y*z + x*w), 1 - 2*(x*x + y*y)], axis=1),
    ], axis=1)  # (N, 3, 3)

    # R_rel = R @ R_ref_inv  →  (N, 3, 3) @ (3, 3)
    R_rel = R @ R_ref_inv   # (N, 3, 3)

    # Rotate vectors: (N, 3, 3) @ (N, 3, 1) → squeeze to (N, 3)
    acc_g = (R_rel @ accel[:, :, None])[:, :, 0] - g_global
    gyr_g = (R_rel @ gyro[:, :, None])[:, :, 0]
    return acc_g, gyr_g


def compute_gait_phase(grf_fz: np.ndarray, threshold: float = 20.0) -> np.ndarray:
    """GRF Fz → Gait Phase [0, 1). Heel strike = rising edge of contact signal."""
    contact = (np.nan_to_num(grf_fz.flatten()) > threshold).astype(float)
    pad = np.pad(contact, (1, 0), constant_values=0)
    strikes = np.where(np.diff(pad) > 0)[0]
    N = len(contact)
    phase = np.zeros(N)
    for i in range(len(strikes) - 1):
        s0, s1 = strikes[i], strikes[i + 1]
        phase[s0:s1] = np.linspace(0.0, 1.0, s1 - s0, endpoint=False)
    return phase


def compute_stride_frequency(grf_fz: np.ndarray, fs: float = 100.0,
                             threshold: float = 20.0) -> np.ndarray:
    """GRF Fz → real-time stride frequency (Hz). Inverse of heel-strike interval."""
    contact = (np.nan_to_num(grf_fz.flatten()) > threshold).astype(float)
    pad = np.pad(contact, (1, 0), constant_values=0)
    strikes = np.where(np.diff(pad) > 0)[0]
    N = len(grf_fz.flatten())
    freq = np.zeros(N)
    for i in range(len(strikes) - 1):
        s0, s1 = strikes[i], strikes[i + 1]
        f = fs / (s1 - s0)
        freq[s0:s1] = f
    if len(strikes) > 0:
        freq[strikes[-1]:] = freq[max(0, strikes[-1] - 1)]
    return freq


def extract_condition_data_v2(
    f, sub, cond,
    input_vars, output_vars,
    lpf_cutoff=None, lpf_order=4, fs=fs,
    include_levels=None, include_trials=None,
    use_physical_velocity_model=False,
    use_gait_phase=False,
    input_lpf_cutoff=None, input_lpf_order=4
):
    """
    input_vars, output_vars: list of (group_path, [var_names...])
    Return: list of (T, InDim) arrays, list of (T, OutDim) arrays
    """
    if sub not in f or cond not in f[sub]:
        return [], []

    cond_group = f[sub][cond]
    level_keys = list(cond_group.keys())

    in_list_all = []
    out_list_all = []

    for lv in level_keys:
        if include_levels and lv not in include_levels:
            continue

        lv_group = cond_group[lv]
        trial_keys = list(lv_group.keys())

        for trial_name in trial_keys:
            if include_trials and trial_name not in include_trials:
                continue
            trial_group = lv_group[trial_name]

            kin_q_arrays = []
            curr_trial_in = []
            valid_trial = True
            trial_cache = {}

            for gpath, vars in input_vars:
                # sub_info: subject-level scalar constants (height, weight, etc.)
                if gpath == 'sub_info':
                    ref = get_data_from_group(trial_group, "treadmill/left/speed_leftbelt")
                    if ref is None:
                        ref = get_data_from_group(trial_group, "robot/left/torque")
                    if ref is None:
                        valid_trial = False; break
                    n_si = len(ref.flatten())
                    si = f[sub].get('sub_info') if sub in f else None
                    si_cols = []
                    for v_name in vars:
                        default = {'height': 1.70, 'weight': 70.0}.get(v_name, 0.0)
                        val = float(si.attrs[v_name]) if (si is not None and v_name in si.attrs) else default
                        si_cols.append(np.full((n_si, 1), val, dtype=np.float32))
                    si_arr = np.concatenate(si_cols, axis=1)
                    kin_q_arrays.append(si_arr)
                    curr_trial_in.append(si_arr)
                    continue

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

                block_cols = []
                derived_cache = {}

                a_belt_global = 0.0
                v_belt_data = get_data_from_group(trial_group, "treadmill/left/speed_leftbelt")
                if v_belt_data is not None:
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

                    # 2. Derived Logic (Velocity/Acceleration)
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

                            target_data = butter_lowpass_filter(target_data, cutoff=30.0, fs=fs, order=4)
                            col_data = target_data

                            if use_physical_velocity_model and not is_accel and 'hip' in v_name and 'flexion' in v_name:
                                L = 0.9
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

                                a_lin = raw_ax + 9.81 * np.sin(ps)
                                if isinstance(a_belt_global, np.ndarray) and len(a_belt_global) == n:
                                    a_lin = a_lin + a_belt_global

                                vel_f = butter_highpass_filter(np.cumsum(a_lin)*dt, 0.1, fs, 2)

                                derived_cache["roll"], derived_cache["pitch"], derived_cache["yaw"], derived_cache["vel_forward"] = rs, ps, ys, vel_f
                                col_data = derived_cache[v_name]
                        except Exception as e:
                            print(f"[ERROR] IMU Feature Error: {e}")
                            valid_trial = False; break

                    # 6. Global IMU (quaternion-based frame transform)
                    elif gpath == "robot/back_imu" and v_name in _GLOBAL_IMU_VARS:
                        try:
                            if 'global_imu_cache' not in derived_cache:
                                _bg = trial_group
                                for _p in ['robot', 'back_imu']:
                                    _bg = _bg[_p] if _p in _bg else None
                                    if _bg is None: break
                                back_imu_grp = _bg
                                if back_imu_grp is not None and 'quat_w' in back_imu_grp:
                                    quat = np.stack([
                                        back_imu_grp['quat_w'][:], back_imu_grp['quat_x'][:],
                                        back_imu_grp['quat_y'][:], back_imu_grp['quat_z'][:]
                                    ], axis=1)
                                    _ax = back_imu_grp['accel_x'][:]
                                    _ay = back_imu_grp['accel_y'][:]
                                    _az = back_imu_grp['accel_z'][:]
                                    _gx = back_imu_grp['gyro_x'][:]
                                    _gy = back_imu_grp['gyro_y'][:]
                                    _gz = back_imu_grp['gyro_z'][:]
                                    if sub.startswith('m2_'): _az = -_az
                                    elif sub.startswith('m1_'): _ax = -_ax
                                    ag, gg = compute_global_imu(
                                        np.stack([_ax, _ay, _az], axis=1),
                                        np.stack([_gx, _gy, _gz], axis=1),
                                        quat, calib_n=100)
                                    dt_g = 1.0 / fs
                                    vel_y = butter_highpass_filter(np.cumsum(ag[:, 1]) * dt_g, 0.1, fs, 2)
                                    v_belt = get_data_from_group(trial_group, 'treadmill/left/speed_leftbelt')
                                    if v_belt is not None:
                                        n_g = len(ag)
                                        a_b = np.gradient(v_belt.flatten()) * fs
                                        a_b = a_b[:n_g] if len(a_b) >= n_g else np.pad(a_b, (0, n_g - len(a_b)))
                                        vel_tread = butter_highpass_filter(
                                            np.cumsum((ag[:, 1] + a_b)) * dt_g, 0.1, fs, 2)
                                    else:
                                        vel_tread = vel_y
                                else:
                                    n_g = len(grp['accel_x'][:]) if 'accel_x' in grp else 1
                                    ag = np.zeros((n_g, 3))
                                    gg = np.zeros((n_g, 3))
                                    vel_y = vel_tread = np.zeros(n_g)
                                derived_cache['global_imu_cache'] = {
                                    'global_acc_x': ag[:, 0], 'global_acc_y': ag[:, 1], 'global_acc_z': ag[:, 2],
                                    'global_gyr_x': gg[:, 0], 'global_gyr_y': gg[:, 1], 'global_gyr_z': gg[:, 2],
                                    'global_vel_y': vel_y, 'global_vel_treadmill': vel_tread,
                                }
                            col_data = derived_cache['global_imu_cache'].get(v_name, np.zeros(1))
                        except Exception as e:
                            print(f"[ERROR] Global IMU Error: {e}")
                            valid_trial = False; break

                    # 7. Gait Phase / Foot Contact (from forceplate/grf)
                    elif "forceplate/grf" in gpath and v_name in ('gait_phase_l', 'gait_phase_r', 'contact_l', 'contact_r', 'stride_freq_l', 'stride_freq_r'):
                        side = 'left' if v_name.endswith('_l') else 'right'
                        fz = get_data_from_group(trial_group, f'forceplate/grf/{side}/z')
                        if fz is not None:
                            if 'gait_phase' in v_name:
                                col_data = compute_gait_phase(fz.flatten(), threshold=20.0).reshape(-1, 1)
                            elif 'stride_freq' in v_name:
                                col_data = compute_stride_frequency(fz.flatten(), fs=fs, threshold=20.0).reshape(-1, 1)
                            else:
                                col_data = get_ground_contact(fz.flatten(), threshold=20.0).reshape(-1, 1)
                        else:
                            valid_trial = False; break

                    else:
                        print(f"[DEBUG] {sub}/{cond}/{lv}/{trial_name}: Dataset {v_name} not found in {gpath}")
                        valid_trial = False; break

                    # Data Correction: Sign Flipping
                    if col_data is not None:
                        if sub.startswith("m1_"):
                            if "grf" in gpath and "left" in gpath and v_name == "x":
                                col_data = -col_data
                            elif "grf" in gpath and "right" in gpath and v_name == "x":
                                col_data = -col_data
                            elif "knee_angle_l" in v_name:
                                col_data = -col_data
                            elif "knee_angle_r" in v_name:
                                col_data = -col_data
                        elif sub.startswith("m2_"):
                            if "grf" in gpath and "left" in gpath and v_name == "z":
                                col_data = -col_data
                            elif "grf" in gpath and "right" in gpath and v_name == "z":
                                col_data = -col_data

                    if col_data is not None:
                        if col_data.ndim == 1: col_data = col_data[:, None]
                        block_cols.append(col_data)
                    else:
                        valid_trial = False
                        break

                if not valid_trial: break
                arr = np.concatenate(block_cols, axis=1).astype(np.float32)

                # GRF Normalization
                if "forceplate" in gpath.lower() and "grf" in gpath.lower():
                    mass = 70.0
                    if sub in f and "mass" in f[sub].attrs:
                        mass = f[sub].attrs["mass"]
                    bw = mass * 9.81
                    arr = arr / bw

                kin_q_arrays.append(arr)
                curr_trial_in.append(arr)

            if not valid_trial: continue

            # Gait Phase augmentation (placeholder - logic in main loop if needed)
            if use_gait_phase:
                pass

            if use_physical_velocity_model:
                pass

            if not curr_trial_in:
                continue

            # Synchronization
            all_objs = curr_trial_in.copy()
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

            all_objs.extend(extracted_outs)
            min_len_trial = min([a.shape[0] for a in all_objs])

            in_arr_trial = np.hstack([a[:min_len_trial] for a in curr_trial_in])
            out_arr_trial = np.hstack([a[:min_len_trial] for a in extracted_outs])

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

    if not os.path.exists(data_path):
        pass

    with h5py.File(data_path, 'r') as f:
        for sub in sub_names:
            if subject_selection:
                if sub not in subject_selection.get('include', sub_names):
                    continue
                if sub in subject_selection.get('exclude', []):
                    continue

            if sub not in f:
                continue

            for cond in cond_names:
                if condition_selection:
                    if condition_selection.get('include') and cond not in condition_selection['include']:
                        continue
                    if cond in condition_selection.get('exclude', []):
                        continue

                if cond not in f[sub]:
                    continue

                X_trials, Y_trials = extract_condition_data_v2(
                    f, sub, cond, input_vars, output_vars,
                    lpf_cutoff=lpf_cutoff, lpf_order=lpf_order,
                    use_physical_velocity_model=use_physical_velocity_model,
                    use_gait_phase=use_gait_phase,
                    input_lpf_cutoff=input_lpf_cutoff,
                    input_lpf_order=input_lpf_order
                )

                if not X_trials:
                    continue

                for X_arr, Y_arr in zip(X_trials, Y_trials):
                    if est_tick_ranges:
                        req_out_len = max(est_tick_ranges)
                    else:
                        req_out_len = time_window_output

                    min_len = time_window_input + req_out_len

                    if len(X_arr) < min_len:
                        continue

                    X_list.append(X_arr)
                    Y_list.append(Y_arr)

    if len(X_list) == 0:
        print("[DEBUG] X_list is empty at end of build_nn_dataset")
        return [], []

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
    data_sources = {}

    if 'shared' in config and 'data_sources' in config['shared']:
        for ds_name, ds_config in config['shared']['data_sources'].items():
            prefix = ds_config.get('prefix', '')
            path = ds_config.get('path', '')
            exclude = ds_config.get('exclude_subjects', [])
            if path:
                data_sources[prefix] = {'path': path, 'exclude_subjects': exclude}

    if not data_sources:
        data_path = config.get('data_path')
        if not data_path:
            if '01_construction' in config: data_path = config['01_construction'].get('src_h5')
            if not data_path and 'shared' in config: data_path = config['shared'].get('src_h5')
            if not data_path: data_path = './combined_data.h5'

        data_sources = {'': {'path': data_path, 'exclude_subjects': []}}

    X_all = []
    Y_all = []

    print(f"[DATA] Loading Multi-Source: keys={list(data_sources.keys())}")

    for prefix, src_cfg in data_sources.items():
        path = src_cfg['path']
        src_exclude = src_cfg.get('exclude_subjects', [])

        current_source_subs = []
        if sub_names:
            for s in sub_names:
                if prefix and s.startswith(prefix):
                    stripped = s[len(prefix):]
                    current_source_subs.append(stripped)
                elif not prefix:
                    current_source_subs.append(s)

        if not current_source_subs:
            continue

        curr_sub_select = {}
        if subject_selection:
            if 'include' in subject_selection:
                inc_list = []
                for s in subject_selection['include']:
                    if prefix and s.startswith(prefix): inc_list.append(s[len(prefix):])
                    elif not prefix: inc_list.append(s)
                curr_sub_select['include'] = inc_list

            ex_list = []
            if 'exclude' in subject_selection:
                for s in subject_selection['exclude']:
                    if prefix and s.startswith(prefix): ex_list.append(s[len(prefix):])
                    elif not prefix: ex_list.append(s)

            if src_exclude:
                ex_list.extend(src_exclude)

            curr_sub_select['exclude'] = ex_list

        print(f"  -> Source '{prefix}': {path} | Subs: {len(current_source_subs)}")

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
