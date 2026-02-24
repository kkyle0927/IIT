import yaml
import os
import copy

def load_yaml(path):
    with open(path, 'r') as f:
        return yaml.safe_load(f)

def save_yaml(path, data):
    # print(f"Generating {path}...")
    with open(path, 'w') as f:
        yaml.dump(data, f, default_flow_style=False, sort_keys=False)

def main():
    base_path = "configs/baseline.yaml"
    if not os.path.exists(base_path):
        print("Error: baseline.yaml not found at", os.path.abspath(base_path))
        return

    base = load_yaml(base_path)
    
    # Base Inputs (Euler + Hip)
    # Note: baseline.yaml already has these as default now
    inputs_base = base['input_vars'] 

    def make_exp(name, updates):
        cfg = copy.deepcopy(base)
        cfg['exp_name'] = name
        
        # Helper to update nested dicts
        for k, v in updates.items():
            if isinstance(v, dict) and k in cfg and isinstance(cfg[k], dict):
                cfg[k].update(v)
            else:
                cfg[k] = v
        
        save_yaml(f"configs/{name}.yaml", cfg)
        print(f"Generated {name}.yaml")

    print(f"Generating experiments from {base_path}...")
    
    # -------------------------------------------------------------------------
    # A. Feature Engineering
    # -------------------------------------------------------------------------
    
    # 1. Gait Phase (User Request)
    # Inputs: Euler + Hip + Gait Phase
    inputs_gait = copy.deepcopy(inputs_base)
    inputs_gait.insert(1, ["derived", ["gait_phase_L", "gait_phase_R"]])
    make_exp("exp_gait_phase", {"input_vars": inputs_gait})
    
    # 2. Euler + Gyro Fusion
    inputs_gyro = copy.deepcopy(inputs_base)
    inputs_gyro.insert(1, ["robot/back_imu", ["gyro_x", "gyro_y", "gyro_z"]])
    make_exp("exp_euler_gyro_fusion", {"input_vars": inputs_gyro})
    
    # 3. Euler + Accel Fusion
    inputs_accel = copy.deepcopy(inputs_base)
    inputs_accel.insert(1, ["robot/back_imu", ["accel_x", "accel_y", "accel_z"]])
    make_exp("exp_euler_accel_fusion", {"input_vars": inputs_accel})
    
    # 4. Full IMU + Euler
    inputs_full = copy.deepcopy(inputs_base)
    inputs_full.insert(1, ["robot/back_imu", ["accel_x", "accel_y", "accel_z", "gyro_x", "gyro_y", "gyro_z"]])
    make_exp("exp_full_imu_euler", {"input_vars": inputs_full})
    
    # 5. Yaw Only (Trunk Rotation)
    inputs_yaw = [
        ["derived/euler", ["yaw"]],
        ["robot/left", ["hip_angle"]],
        ["robot/right", ["hip_angle"]]
    ]
    make_exp("exp_yaw_only", {"input_vars": inputs_yaw})
    
    # 6. Roll/Pitch Only (No Yaw)
    inputs_rp = [
        ["derived/euler", ["roll", "pitch"]],
        ["robot/left", ["hip_angle"]],
        ["robot/right", ["hip_angle"]]
    ]
    make_exp("exp_roll_pitch_only", {"input_vars": inputs_rp})
    
    # 7. Hip Velocity (Hip Dot)
    inputs_hv = copy.deepcopy(inputs_base)
    # Append dot vars to existing robot group? Or separate? 
    # Robot group is at index 1 and 2. 
    # inputs_base[1] is ["robot/left", ["hip_angle"]]
    inputs_hv[1][1].append("hip_angle_dot")
    inputs_hv[2][1].append("hip_angle_dot")
    make_exp("exp_hip_velocity", {"input_vars": inputs_hv})
    
    # 8. Full Features (Euler + IMU + Hip + HipDot)
    inputs_all = copy.deepcopy(inputs_full)
    inputs_all[2][1].append("hip_angle_dot") # Index shifted by insert
    inputs_all[3][1].append("hip_angle_dot")
    make_exp("exp_full_features", {"input_vars": inputs_all})
    
    # -------------------------------------------------------------------------
    # B. Model Architecture
    # -------------------------------------------------------------------------
    
    # 9. Deeper TCN
    make_exp("exp_deeper_tcn", {"model": {"channels": [32, 64, 128, 256, 256]}})
    
    # 10. Wider TCN
    make_exp("exp_wider_tcn", {"model": {"channels": [64, 128, 128, 256]}})
    
    # 11. Large Kernel
    make_exp("exp_large_kernel", {"model": {"kernel_size": 9}})
    
    # 12. Small Kernel
    make_exp("exp_small_kernel", {"model": {"kernel_size": 3}}) # Baseline is 5? No, baseline check
    # Baseline line 144: kernel_size: 5. 
    # So small kernel = 3 is valid experiment.
    
    # 13. Multi Head MLP
    make_exp("exp_multi_head_mlp", {"model": {"head_hidden": [128, 64]}})
    
    # 14. Gated TCN
    make_exp("exp_gated_tcn", {"model": {"type": "StanceGatedTCN"}})
    
    # 15. Attention V2
    make_exp("exp_attention_v2", {"model": {"type": "AttentionTCN", "attention_heads": 4}})
    
    # 16. TCN GRU (Hybrid)
    make_exp("exp_tcn_gru", {"model": {"type": "TCN_GRU", "gru_hidden": 64, "gru_layers": 1}})
    
    # 17. TCN GRU Deep
    make_exp("exp_tcn_gru_deep", {"model": {"type": "TCN_GRU", "gru_hidden": 128, "gru_layers": 2}})
    
    # 18. TCN BiGRU
    make_exp("exp_tcn_bigru", {"model": {"type": "TCN_GRU", "gru_hidden": 64, "gru_layers": 1, "bidirectional": True}})
    
    # -------------------------------------------------------------------------
    # C. Window & Stride
    # -------------------------------------------------------------------------
    
    # 19. Win 100 (1s)
    make_exp("exp_win_100", {"data": {"window_size": 100}})
    
    # 20. Win 300 (3s)
    make_exp("exp_win_300", {"data": {"window_size": 300}})
    
    # 21. Win 400 (4s)
    make_exp("exp_win_400", {"data": {"window_size": 400}})
    
    # 22. Stride 1 (Dense)
    make_exp("exp_stride_01", {"data": {"stride": 1}})
    
    # -------------------------------------------------------------------------
    # D. Regularization
    # -------------------------------------------------------------------------
    
    # 23. Low Dropout
    make_exp("exp_low_dropout", {"model": {"dropout": 0.2, "head_dropout": 0.2}})
    
    # 24. High Dropout
    make_exp("exp_high_dropout", {"model": {"dropout": 0.7, "head_dropout": 0.7}})
    
    # 25. Low LR
    make_exp("exp_low_lr", {"train": {"learning_rate": 0.0003, "epochs": 50}})
    
    # 26. High WD
    make_exp("exp_high_wd", {"train": {"weight_decay": 0.05}})
    
    # -------------------------------------------------------------------------
    # E. Autoregressive / Sequence
    # -------------------------------------------------------------------------
    
    # 27. Seq 10
    make_exp("exp_seq_output_10", {"data": {"time_window_output": 10, "est_tick_ranges": [1,2,3,4,5,6,7,8,9,10]}})
    
    # 28. Seq 25
    make_exp("exp_seq_output_25", {"data": {"time_window_output": 25, "est_tick_ranges": [5, 10, 15, 20, 25]}})
    
    # 29. Multi Horizon
    make_exp("exp_multi_horizon", {"data": {"est_tick_ranges": [0, 5, 10, 20, 50], "time_window_output": 50}}) # Need output window to cover max range

if __name__ == "__main__":
    main()
