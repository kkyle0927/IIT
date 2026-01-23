
import os
import torch
import numpy as np
import matplotlib.pyplot as plt
import torch.nn as nn
from torch.utils.data import DataLoader

# Import everything from the main experiment script
from SpeedEstimator_TCN_LSTM_MLP_experiments import (
    build_nn_dataset, 
    WindowDataset,
    TCN_LSTM_MLP,
    plot_model_summary,
    TransposedBatchNorm1d,
    TCNEncoder,
)

# -----------------------------------------------------------------------------
# Redefine Constants and Helper Functions (Local to this script)
# -----------------------------------------------------------------------------

def make_subject_selection(include_list):
    return {'include': include_list, 'exclude': []}

data_path = '251030_combined_data.h5'

sub_names = ['S004','S005','S006','S007','S008','S009','S010','S011']
cond_names = [
    'accel_sine','asym_30deg','asym_60deg','cadence_120p','cadence_90p',
    'crouch','decline_5deg','incline_10deg','level_08mps','level_12mps','level_16mps'
]

TEST_SUBJECTS  = ['S011']
CONDITION_SELECTION = {'include': None, 'exclude': []}

fs = 100
time_window_input = 100
time_window_output = 10
stride = 10
LPF_CUTOFF = 6.0
LPF_ORDER = 5

input_vars = [
    ('Robot', ['incPosLH', 'incPosRH']),
    ('Back_imu', ['Accel_X', 'Accel_Y', 'Accel_Z', 'Gyro_X', 'Gyro_Y', 'Gyro_Z', 'Pitch', 'Roll', 'Yaw']),
    ('MoCap/grf_measured/left/force', ['Fx', 'Fy', 'Fz']),
    ('MoCap/grf_measured/right/force', ['Fx', 'Fy', 'Fz']),
    ('MoCap/kin_q', ['ankle_angle_l', 'ankle_angle_r', 'hip_adduction_l', 'hip_adduction_r', 'hip_flexion_l', 'hip_flexion_r', 'hip_rotation_l', 'hip_rotation_r', 'knee_angle_l', 'knee_angle_r']),
    ('MoCap/kin_qdot', ['ankle_angle_l', 'ankle_angle_r', 'hip_adduction_l', 'hip_adduction_r', 'hip_flexion_l', 'hip_flexion_r', 'hip_rotation_l', 'hip_rotation_r', 'knee_angle_l', 'knee_angle_r'])
]
output_vars = [('Common', ['v_Y_true', 'v_Z_true'])]

# -----------------------------------------------------------------------------
# Evaluation Function
# -----------------------------------------------------------------------------

def evaluate_checkpoint(model_cls, ckpt_path, X_test, Y_test, device='cpu'):
    if not os.path.exists(ckpt_path):
        print(f"[WARN] Checkpoint not found: {ckpt_path}")
        return None
        
    checkpoint = torch.load(ckpt_path, map_location=device)
    config = checkpoint['config']
    
    # Re-instantiate model
    model = model_cls(
        input_dim=X_test.shape[2],
        output_dim=Y_test.shape[2],
        horizon=Y_test.shape[1],
        channels=config.get("tcn_channels", (64, 64, 128)),
        kernel_size=config.get("kernel_size", 3),
        dropout=config.get("dropout_p", 0.1),
        hidden_size=config.get("lstm_hidden", 128),
        num_layers=config.get("lstm_layers", 1),
        mlp_hidden=config.get("mlp_hidden", 128),
        use_input_norm=config.get("use_input_norm", False),
        tcn_norm=config.get("tcn_norm", None),
        lstm_use_ln=config.get("lstm_use_ln", False),
        mlp_norm=config.get("mlp_norm", None)
    )
    
    model.load_state_dict(checkpoint['state_dict'])
    model.to(device)
    model.eval()
    
    # Evaluate
    test_ds = WindowDataset(X_test, Y_test, target_mode="sequence")
    test_loader = DataLoader(test_ds, batch_size=config.get("val_batch_size", 256), shuffle=False)
    criterion = nn.HuberLoss(delta=config.get("huber_delta", 0.5))
    
    test_running = 0.0
    preds_all, trues_all = [], []
    
    with torch.no_grad():
        for xb, yb in test_loader:
            xb, yb = xb.to(device), yb.to(device)
            pred = model(xb)
            test_running += criterion(pred, yb).item()
            preds_all.append(pred.cpu().numpy())
            trues_all.append(yb.cpu().numpy())
            
    test_loss = test_running / max(1, len(test_loader))
    
    P = np.concatenate(preds_all, axis=0)
    T = np.concatenate(trues_all, axis=0)
    err = P - T
    
    test_mae = np.mean(np.abs(err))
    test_rmse = np.sqrt(np.mean(err**2))
    
    n_params = sum(p.numel() for p in model.parameters() if p.requires_grad)
    
    return {
        "test_huber": test_loss,
        "test_mae": float(test_mae),
        "test_rmse": float(test_rmse),
        "n_params": n_params,
        "config": config
    }

if __name__ == "__main__":
    device = "cuda" if torch.cuda.is_available() else "cpu"
    print(f"Using device: {device}")
    
    print("Loading Test Data...")
    # Only need Test data for evaluation
    X_test, Y_test = build_nn_dataset(
        data_path, sub_names, cond_names, input_vars, output_vars,
        time_window_input, time_window_output, stride,
        subject_selection=make_subject_selection(TEST_SUBJECTS),
        condition_selection=CONDITION_SELECTION, lpf_cutoff=LPF_CUTOFF, lpf_order=LPF_ORDER
    )
    print(f"Test Data: {X_test.shape}")
    
    # Define Experiments (Must match main script to find files)
    experiments = [
        ("Base", {}),
        ("Deep_TCN", {}),
        ("Wide_LSTM", {}),
        ("Deep_LSTM", {}),
        ("Large_Kernel", {}),
        ("High_Dropout", {}),
        ("TCN_BN", {}),
        ("LSTM_LN", {}),
        ("MLP_BN", {}),
        ("Mix_TCN(BN)_LSTM(LN)_MLP(BN)", {}),
        ("Input_Norm", {})
    ]
    
    seeds = [42]
    results = []
    
    for exp_name, _ in experiments:
        for sd in seeds:
            ckpt_name = f"opt_tcn_lstm_mlp_{exp_name}_{sd}.pt"
            print(f"Loading {ckpt_name}...")
            
            metrics = evaluate_checkpoint(TCN_LSTM_MLP, ckpt_name, X_test, Y_test, device)
            
            if metrics:
                print(f"  [OK] {exp_name}: MAE={metrics['test_mae']:.4f}")
                results.append((exp_name, sd, metrics, ckpt_name, metrics['config']))
            else:
                print(f"  [FAIL] Checkpoint load failed or file missing.")

    if results:
        print("\nGenerating Summary Plot...")
        fig_summary = plot_model_summary(results)
        fig_summary.savefig('opt_results_summary.png')
        plt.close(fig_summary)
        print("Saved opt_results_summary.png")
        
        # -------------------------------------------------------------------------
        # Feature Importance (Best Model)
        # -------------------------------------------------------------------------
        print("\n>>> Calculating Feature Importance for Best Model...")
        
        # 1. Find Best Model (lowest test_mae)
        best_res = sorted(results, key=lambda x: x[2]['test_mae'])[0]
        best_name, best_seed, best_metrics, best_ckpt, best_cfg = best_res
        print(f"Best Model: {best_name} (MAE={best_metrics['test_mae']:.6f})")
        
        # 2. Re-load Best Model
        # Re-instantiate
        best_model = TCN_LSTM_MLP(
            input_dim=X_test.shape[2],
            output_dim=Y_test.shape[2],
            horizon=Y_test.shape[1],
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
        
        best_model.to(device)
        best_model.eval()
        
        # 3. Construct Feature Names
        # Must match extract_condition_data_v2 logic
        feature_names = []
        
        # (A) Input Vars
        for gpath, vars in input_vars:
            # Custom Naming Logic
            if 'Back_imu' in gpath:
                prefix = "IMU"
            elif 'Robot' in gpath:
                prefix = "Robot"
            elif 'grf_measured' in gpath:
                # MoCap/grf_measured/left/force
                side = 'L' if 'left' in gpath else 'R'
                prefix = f"GRF_{side}"
            elif 'kin_qdot' in gpath:
                prefix = "J_Vel_Meas"
                # Actually kin_qdot in input_vars usually means standard MoCap derived.
                pass 
            elif 'kin_q' in gpath:
                prefix = "J_Ang"
            else:
                prefix = gpath.split('/')[-1]
                
            for v in vars:
                # Clean var names
                v_clean = v.replace('ankle_angle', 'Ankle').replace('knee_angle', 'Knee').replace('hip_', 'Hip_')
                v_clean = v_clean.replace('flexion', 'Flex').replace('adduction', 'Add').replace('rotation', 'Rot')
                v_clean = v_clean.replace('_l', '_L').replace('_r', '_R')
                
                if 'grf' in gpath.lower():
                    # GRF vars are Fx, Fy, Fz
                    name = f"{prefix}_{v}"
                elif 'Back_imu' in gpath:
                    # Accel_X -> Ax
                    v_clean = v.replace('Accel_', 'A').replace('Gyro_', 'G')
                    name = f"{prefix}_{v_clean}"
                elif 'kin_q' in gpath:
                    name = f"{prefix}_{v_clean}"
                else:
                    name = f"{prefix}_{v_clean}"
                    
                feature_names.append(name)
                
        # (B) Augment: Joint Veolcity/Accel
        # kin_q vars are repeated for Vel and Accel
        kin_q_vars = []
        for gpath, vars in input_vars:
            if 'kin_q' in gpath and 'dot' not in gpath:
                 kin_q_vars = vars
                 break
                 
        for v in kin_q_vars:
            v_clean = v.replace('ankle_angle', 'Ankle').replace('knee_angle', 'Knee').replace('hip_', 'Hip_')
            v_clean = v_clean.replace('flexion', 'Flex').replace('adduction', 'Add').replace('rotation', 'Rot')
            v_clean = v_clean.replace('_l', '_L').replace('_r', '_R')
            feature_names.append(f"Calc_Vel_{v_clean}")
            
        for v in kin_q_vars:
            v_clean = v.replace('ankle_angle', 'Ankle').replace('knee_angle', 'Knee').replace('hip_', 'Hip_')
            v_clean = v_clean.replace('flexion', 'Flex').replace('adduction', 'Add').replace('rotation', 'Rot')
            v_clean = v_clean.replace('_l', '_L').replace('_r', '_R')
            feature_names.append(f"Calc_Acc_{v_clean}")
            
        # (C) Contact
        feature_names.append("Contact_L")
        feature_names.append("Contact_R")
        
        if len(feature_names) != X_test.shape[2]:
            print(f"[WARN] Feature name count ({len(feature_names)}) != Input dim ({X_test.shape[2]}). Using indices.")
            feature_names = [f"Feat_{i}" for i in range(X_test.shape[2])]
            
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

        importances = calculate_permutation_importance(best_model, X_test, Y_test, feature_names, device)
        
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
        plt.savefig('feature_importance_best_model.png')
        plt.close()
        print("Saved feature_importance_best_model.png")
    
        # -------------------------------------------------------------------------
        # Visualization: Prediction vs Ground Truth (Best Model)
        # -------------------------------------------------------------------------
        print("\n>>> Visualizing Predictions vs Ground Truth...")
        
        # Run Inference on Test Set
        test_ds = WindowDataset(X_test, Y_test, target_mode="sequence")
        loader = DataLoader(test_ds, batch_size=1024, shuffle=False)
        
        preds_all = []
        trues_all = []
        
        best_model.eval()
        with torch.no_grad():
            for xb, yb in loader:
                xb, yb = xb.to(device), yb.to(device)
                pred = best_model(xb)
                preds_all.append(pred.cpu().numpy())
                trues_all.append(yb.cpu().numpy())
                
        # (N_windows, Horizon, D)
        P = np.concatenate(preds_all, axis=0) 
        T = np.concatenate(trues_all, axis=0)
        
        # 1. Continuous Trajectory Plot (Stitching)
        # Since stride=10 and horizon=10, we can reshape to (Total_Time, D)
        # Note: This concatenates ALL conditions/subjects. Boundaries will be jumpy.
        # We'll plot a subset (e.g. first 2000 frames) which likely corresponds to S011/Condition1
        
        P_cont = P.reshape(-1, P.shape[2])
        T_cont = T.reshape(-1, T.shape[2])
        
        # Select a plotting window (e.g. 1000 frames from the middle or start)
        # Let's verify if 0-1000 has data (it should)
        plot_len = 1000
        start_idx = 0
        
        fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
        
        # Vy
        axes[0].plot(T_cont[start_idx:start_idx+plot_len, 0], 'k-', label='True', alpha=0.6)
        axes[0].plot(P_cont[start_idx:start_idx+plot_len, 0], 'r--', label='Pred', alpha=0.8)
        axes[0].set_title(f"Speed Estimation (Vy) - {best_name}")
        axes[0].set_ylabel("Speed (m/s)")
        axes[0].legend()
        axes[0].grid(True, linestyle='--', alpha=0.5)
        
        # Vz
        axes[1].plot(T_cont[start_idx:start_idx+plot_len, 1], 'k-', label='True', alpha=0.6)
        axes[1].plot(P_cont[start_idx:start_idx+plot_len, 1], 'b--', label='Pred', alpha=0.8)
        axes[1].set_title(f"Speed Estimation (Vz) - {best_name}")
        axes[1].set_ylabel("Speed (m/s)")
        axes[1].set_xlabel("Time Frames (Stitched)")
        axes[1].legend()
        axes[1].grid(True, linestyle='--', alpha=0.5)
        
        plt.tight_layout()
        plt.savefig('trajectory_prediction_best_model.png')
        plt.close()
        print("Saved trajectory_prediction_best_model.png")
        
        # 2. Error by Horizon Step
        # Calculate MAE for each horizon step (0 to 9)
        # P: (N, H, D), T: (N, H, D)
        abs_diff = np.abs(P - T) # (N, H, D)
        mae_per_step = np.mean(abs_diff, axis=0) # (H, D)
        
        horizon_steps = np.arange(1, P.shape[1] + 1)
        
        plt.figure(figsize=(8, 6))
        plt.plot(horizon_steps, mae_per_step[:, 0], 'rs-', label='Vy MAE')
        plt.plot(horizon_steps, mae_per_step[:, 1], 'bo-', label='Vz MAE')
        plt.xlabel("Horizon Step (Lookahead)")
        plt.ylabel("Mean Absolute Error (m/s)")
        plt.title(f"Prediction Error vs Horizon - {best_name}")
        plt.legend()
        plt.grid(True, linestyle='--', alpha=0.5)
        plt.xticks(horizon_steps)
        
        plt.tight_layout()
        plt.savefig('horizon_error_best_model.png')
        plt.close()
        print("Saved horizon_error_best_model.png")
    else:
        print("No results to plot.")
