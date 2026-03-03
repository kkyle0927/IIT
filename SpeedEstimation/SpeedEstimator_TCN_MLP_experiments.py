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
torch.set_num_threads(8)

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

# =========================================================================================
# [Re-exports for backward compatibility]
# compare_results.py imports:
#   from SpeedEstimator_TCN_MLP_experiments import (
#       TCN_MLP, StanceGatedTCN, AttentionTCN, TCN_GRU_Head, LazyWindowDataset,
#       build_nn_dataset, build_nn_dataset_multi, make_subject_selection, CONDITION_SELECTION
#   )
#   import SpeedEstimator_TCN_MLP_experiments as main_script
# All of these continue to work unchanged via these re-exports.
# =========================================================================================

from core.models import Chomp1d, TemporalBlock, TCNEncoder, TCN_MLP, StanceGatedTCN, AttentionTCN, TCN_GRU_Head
from core.losses import get_custom_losses
from core.datasets import LazyWindowDataset
from data.utils import (
    fs, time_window_output, stride,
    parse_vars, make_subject_selection,
    CONDITION_SELECTION,
    plot_filtering_check,
    get_ground_contact,
    butter_lowpass_filter,
    butter_highpass_filter,
    get_data_from_group,
)
from data.loader import extract_condition_data_v2, build_nn_dataset, build_nn_dataset_multi
from training.trainer import (
    LiveTrainingPlotter,
    ExperimentLogger,
    plot_model_summary,
    calculate_model_resources,
    train_experiment,
    calculate_permutation_importance,
)
from analysis.inspector import inspect_h5_structure, load_all_subject_data, analyze_data_distribution

# =========================================================================================
# Main Execution
# =========================================================================================

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
        yaml_files = list(config_dir.rglob("*.yaml"))

    base_config_path_str = "configs/baseline.yaml"
    if not os.path.exists(base_config_path_str):
        base_config_path_str = "config_backups/baseline.yaml"
    base_config = {}

    if os.path.exists(base_config_path_str) and (args.config is None or Path(args.config).name != "baseline.yaml"):
        print(f"Loading base config from {base_config_path_str}...")
        with open(base_config_path_str, 'r') as bf:
            base_config = yaml.safe_load(bf)
        print(f"[INFO] Loaded base defaults from {base_config_path_str}")
    else:
        print("No base_config.yaml found or explicitly provided as the main config. Proceeding with provided config only.")

    # Prepare Experiments List
    experiments_to_run = []

    if not yaml_files:
        pass
    else:
        print(f"Found {len(yaml_files)} configs: {[f.name for f in yaml_files]}")

        for yf in yaml_files:
            with open(yf, 'r') as f:
                cfg = yaml.safe_load(f)

            def recursive_update(target, update):
                for k, v in update.items():
                    if isinstance(v, dict) and k in target and isinstance(target[k], dict):
                        recursive_update(target[k], v)
                    else:
                        target[k] = v

            merged = copy.deepcopy(base_config)
            recursive_update(merged, cfg)

            exp_name = cfg.get("exp_name", yf.stem)
            merged["exp_name"] = exp_name
            experiments_to_run.append((exp_name, merged))

    # Sharding for Parallel Execution
    experiments_to_run.sort(key=lambda x: x[0])

    total_exps = len(experiments_to_run)
    experiments_to_run = [e for i, e in enumerate(experiments_to_run) if i % args.total_nodes == args.rank]

    print(f"\n[PARALLEL] Rank {args.rank}/{args.total_nodes} - Process {len(experiments_to_run)}/{total_exps} experiments.")

    # Run Distribution Analysis at the very beginning
    if args.rank == 0:
        analysis_config = base_config
        if not analysis_config and experiments_to_run:
            analysis_config = experiments_to_run[0][1]

        if analysis_config:
            dist_output_dir = "compare_result/distribution_analysis"
            analyze_data_distribution(analysis_config, dist_output_dir)

    live_plotter = None

    results = []
    seeds = [42]

    for exp_name, cfg in tqdm(experiments_to_run, desc=f"GPU-{args.rank} Total", position=0):
        for sd in seeds:
            full_name = f"{exp_name}_seed{sd}"
            print(f"\n>>> Start Experiment: {full_name}")

            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            save_dir = f"experiments/{full_name}"

            if '01_construction' in cfg:
                if 'data_path' not in cfg:
                    cfg['data_path'] = cfg['01_construction'].get('src_h5') or cfg['shared'].get('src_h5')

                if 'subjects' not in cfg:
                    cfg['subjects'] = cfg['01_construction'].get('include_subjects', [])

                if 'conditions' not in cfg:
                    cfg['conditions'] = cfg['01_construction'].get('include_conditions', [])

            if 'cv_mode' not in cfg:
                if '02_train' in cfg:
                    cfg['cv_mode'] = cfg['02_train'].get('split', {}).get('mode', 'loso')
                else:
                    cfg['cv_mode'] = 'loso'

            if 'data_path' not in cfg:
                src = None
                if '01_construction' in cfg: src = cfg['01_construction'].get('src_h5')
                if not src and 'shared' in cfg: src = cfg['shared'].get('src_h5')
                if not src: src = "./combined_data.h5"
                cfg['data_path'] = src

            if '01_construction' in cfg:
                if 'subjects' not in cfg:
                    cfg['subjects'] = cfg['01_construction'].get('include_subjects', [])
                if 'conditions' not in cfg:
                    cfg['conditions'] = cfg['01_construction'].get('include_conditions', [])

            if 'subjects' not in cfg: cfg['subjects'] = []
            if 'conditions' not in cfg: cfg['conditions'] = []

            if 'cv_mode' not in cfg and '02_train' in cfg:
                cfg['cv_mode'] = cfg['02_train'].get('split', {}).get('mode', 'loso')
            elif 'cv_mode' not in cfg:
                cfg['cv_mode'] = 'loso'

            current_data_path = cfg.get("data_path")
            if not current_data_path and base_config:
                current_data_path = base_config.get("data_path")

            if not current_data_path:
                raise ValueError("data_path not found in config or base_config")

            curr_input_vars = cfg.get("input_vars") or (base_config.get("input_vars") if base_config else None)
            curr_output_vars = cfg.get("output_vars") or (base_config.get("output_vars") if base_config else None)

            if not curr_input_vars and 'shared' in cfg:
                curr_input_vars = cfg['shared'].get('input_vars')
            if not curr_output_vars and 'shared' in cfg:
                curr_output_vars = cfg['shared'].get('output_vars')

            if not cfg.get('subjects') and 'shared' in cfg:
                cfg['subjects'] = cfg['shared'].get('subjects', [])
            if not cfg.get('conditions') and 'shared' in cfg:
                cfg['conditions'] = cfg['shared'].get('conditions', [])

            curr_window = cfg.get("time_window_input") or (base_config.get("time_window_input") if base_config else None)
            if not curr_window and 'shared' in cfg and 'data' in cfg['shared']:
                curr_window = cfg['shared']['data'].get('window_size')

            curr_est_tick_ranges = cfg.get("est_tick_ranges") or (base_config.get("est_tick_ranges") if base_config else None)
            if not curr_est_tick_ranges and 'shared' in cfg and 'data' in cfg['shared']:
                val = cfg['shared']['data'].get('y_delay', 5)
                curr_est_tick_ranges = [val] if isinstance(val, int) else val

            if curr_est_tick_ranges:
                cfg["est_tick_ranges"] = curr_est_tick_ranges
            if not curr_input_vars and '01_construction' in cfg:
                curr_input_vars = cfg['01_construction'].get('inputs')
            if not curr_output_vars and '01_construction' in cfg:
                curr_output_vars = cfg['01_construction'].get('outputs')

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

            curr_sub_names = cfg.get("subjects") or (base_config.get("subjects") if base_config else None)
            curr_cond_names = cfg.get("conditions") or (base_config.get("conditions") if base_config else None)

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
                mean, scale = None, None
                normalize_type = cfg.get("02_train", {}).get("data", {}).get("normalize_type", "standard")

                if normalize_type == "subject":
                    # Per-trial z-score (replaces global scaler for E2/E3 experiments)
                    print("[INFO] Applying SUBJECT-WISE (per-trial) Normalization...")
                    for data_list in [X_train, X_val, X_test]:
                        for i in range(len(data_list)):
                            m = np.mean(data_list[i], axis=0).astype(np.float32)
                            s = (np.std(data_list[i], axis=0) + 1e-8).astype(np.float32)
                            data_list[i] = (data_list[i] - m) / s
                    use_norm = False
                else:
                    use_norm = cfg.get("02_train", {}).get("model", {}).get("use_input_norm", True)
                    if not use_norm and '02_train' in cfg:
                        use_norm = cfg['02_train'].get('model', {}).get('use_norm', True)

                    if use_norm:
                        print("[INFO] Computing GLOBAL mean/std for Input Normalization...")
                        all_train = np.concatenate(X_train, axis=0)
                        mean = np.mean(all_train, axis=0).astype(np.float32)
                        std = np.std(all_train, axis=0).astype(np.float32)
                        scale = std
                        scale[scale < 1e-8] = 1.0
                        np.savez(os.path.join(local_save_dir, "scaler.npz"), mean=mean, scale=scale)

                        del all_train
                        gc.collect()

                        for i in range(len(X_train)): X_train[i] = (X_train[i] - mean) / scale
                        for i in range(len(X_val)): X_val[i] = (X_val[i] - mean) / scale
                        for i in range(len(X_test)): X_test[i] = (X_test[i] - mean) / scale

                feat_names_simple = [f"{gp}/{v}" for gp, vs in c_in_vars for v in vs]

                metrics, ckpt_path = train_experiment(
                    X_train, Y_train, X_val, Y_val, X_test, Y_test,
                    cfg, seed=sd, save_dir=local_save_dir, live_plotter=None,
                    scaler_mean=mean, scaler_std=scale,
                    feature_names=feat_names_simple
                )

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
        sys.exit(0)

    print("\n>>> Calculating Feature Importance for Best Model...")

    best_res = sorted(results, key=lambda x: x[2]['test_mae'])[0]
    best_name, best_seed, best_metrics, best_ckpt, best_cfg, best_in_vars = best_res
    print(f"Best Model: {best_name} (MAE={best_metrics['test_mae']:.6f})")

    print("Re-loading Test Data for Best Model Feature Importance...")
    c_in_vars = best_in_vars

    if "output_vars" in best_cfg:
        raw_out = best_cfg["output_vars"]
    elif "shared" in best_cfg and "output_vars" in best_cfg["shared"]:
        raw_out = best_cfg["shared"]["output_vars"]
    elif "01_construction" in best_cfg:
        raw_out = best_cfg["01_construction"].get("outputs")
    else:
        raw_out = base_config.get("output_vars") if base_config else None

    c_out_vars = parse_vars(raw_out)

    best_data_path = best_cfg.get("data_path")
    if not best_data_path:
        best_data_path = best_cfg.get("01_construction", {}).get("src_h5") or \
                         best_cfg.get("shared", {}).get("src_h5") or \
                         (base_config.get("data_path") if base_config else None)

    best_sub_names = best_cfg.get("subjects") or best_cfg.get("01_construction", {}).get("include_subjects")
    best_cond_names = best_cfg.get("conditions") or best_cfg.get("01_construction", {}).get("include_conditions")
    best_lpf_cutoff = best_cfg.get("lpf_cutoff") or best_cfg.get("01_construction", {}).get("lpf_cutoff", 0.5)
    best_lpf_order = best_cfg.get("lpf_order") or best_cfg.get("01_construction", {}).get("lpf_order", 5)

    best_window = best_cfg.get("time_window_input") or best_cfg.get("shared", {}).get("data", {}).get("window_size")

    X_test_best, Y_test_best = build_nn_dataset_multi(
        best_cfg, best_sub_names, best_cond_names, c_in_vars, c_out_vars,
        best_window, best_cfg.get("time_window_output", 1), best_cfg.get("stride", 5),
        subject_selection=make_subject_selection(best_sub_names),
        condition_selection=CONDITION_SELECTION, lpf_cutoff=best_lpf_cutoff, lpf_order=best_lpf_order,
        est_tick_ranges=best_cfg.get("est_tick_ranges", None)
    )

    best_horizon = len(best_cfg.get("est_tick_ranges")) if best_cfg.get("est_tick_ranges") else best_cfg.get("time_window_output", 10)

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

    model_norm = model_cfg.get("model_norm")

    if not X_test_best:
        print("[WARN] No data found for Feature Importance Re-loading. Skipping.")
    else:
        model_type = model_cfg.get("type", "TCN")
        input_norm_type = model_cfg.get("input_norm_type", model_norm or "layer")
        common_args = dict(
            input_dim=X_test_best[0].shape[1],
            output_dim=Y_test_best[0].shape[1],
            horizon=best_horizon,
            channels=tcn_channels,
            kernel_size=kernel_size,
            dropout=dropout_p,
            head_dropout=head_dropout,
            mlp_hidden=mlp_hidden,
            use_input_norm=best_cfg.get("use_input_norm", True),
            input_norm_type=input_norm_type,
            tcn_norm=model_norm,
            mlp_norm=model_norm
        )
        if model_type == "StanceGatedTCN":
            best_model = StanceGatedTCN(**common_args, gating_dim=model_cfg.get("gating_dim", 1))
        elif model_type == "AttentionTCN":
            best_model = AttentionTCN(**common_args, attention_type=model_cfg.get("attention_type", "temporal"), attention_heads=model_cfg.get("attention_heads", 4))
        elif model_type == "TCN_GRU":
            best_model = TCN_GRU_Head(**common_args, gru_hidden=model_cfg.get("gru_hidden", 32))
        else:
            best_model = TCN_MLP(**common_args)

        ckpt = torch.load(best_ckpt, map_location='cpu')
        best_model.load_state_dict(ckpt['state_dict'])

        device = "cuda" if torch.cuda.is_available() else "cpu"
        best_model.to(device)
        best_model.eval()

        # Construct Feature Names
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

        print(f"  Calculating Importances for {len(feature_names)} features...")

        fi_stride = best_cfg.get("stride", 5)

        imp_test_ds = LazyWindowDataset(
            X_test_best, Y_test_best,
            best_window, best_cfg.get("time_window_output", 1), fi_stride,
            target_mode="sequence", est_tick_ranges=best_cfg.get("est_tick_ranges", None)
        )

        gc.collect()
        torch.cuda.empty_cache()

        imp_loader = DataLoader(imp_test_ds, batch_size=512, shuffle=True, num_workers=0)
        importances = calculate_permutation_importance(best_model, imp_loader, feature_names, device)

        # Plot
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
