
import yaml
import os
import copy

def load_yaml(path):
    with open(path, 'r') as f:
        return yaml.safe_load(f)

def save_yaml(path, data):
    print(f"Generating {path}...")
    with open(path, 'w') as f:
        yaml.dump(data, f, default_flow_style=False, sort_keys=False)

def main():
    base_config_path = "configs/baseline.yaml"
    if not os.path.exists(base_config_path):
        print("Error: configs/baseline.yaml not found.")
        return

    # Load Baseline to copy common sections
    base_config = load_yaml(base_config_path)
    common_input = base_config.get('shared', {}).get('input_vars')
    common_output = base_config.get('shared', {}).get('output_vars')
    common_shared = base_config.get('shared')

    # Helper to inject common vars
    def inject_common(cfg):
        if 'shared' not in cfg: cfg['shared'] = common_shared
        # input/output vars are deeply nested in 'shared' usually, 
        # but the analysis script checks cfg['input_vars'] OR cfg['shared']['input_vars'].
        # Since we copied 'shared', it should be fine!
        return cfg

    # -------------------------------------------------------------------------
    # 1. Window Size Variations
    # -------------------------------------------------------------------------
    windows = [30, 50, 100, 200, 400] # ticks (0.3s, 0.5s, 1.0s, 2.0s, 4.0s)
    
    for w in windows:
        if w == 200: continue # Skip baseline
        
        cfg = {
            "exp_name": f"exp_win_{w:03d}",
            "data": {
                "window_size": w,
                "use_complementary_filter_euler": False
            }
        }
        cfg = inject_common(cfg)
        save_yaml(f"configs/exp_win_{w:03d}.yaml", cfg)

    # -------------------------------------------------------------------------
    # 2. Stride Variations
    # -------------------------------------------------------------------------
    strides = [2, 5, 10, 20] # ticks (20ms, 50ms, 100ms, 200ms)
    
    for s in strides:
        if s == 5: continue # Skip baseline if baseline is 5 (It is)
        
        cfg = {
            "exp_name": f"exp_stride_{s:02d}",
            "data": {
                "stride": s,
                "use_complementary_filter_euler": False
            }
        }
        cfg = inject_common(cfg)
        # Explicitly set window size to baseline (200) to isolate stride effect?
        # Typically changes merge, so window_size remains 200.
        save_yaml(f"configs/exp_stride_{s:02d}.yaml", cfg)
        
    # Explicit 50ms stride config for completeness/naming consistency if requested
    # But usually redundant if baseline is 5. User asked for it. 
    # Let's create it as exp_stride_05.yaml (same as baseline but explicit)
    cfg_s5 = {
        "exp_name": "exp_stride_05",
        "data": { "stride": 5, "use_complementary_filter_euler": False }
    }
    cfg_s5 = inject_common(cfg_s5)
    save_yaml("configs/exp_stride_05.yaml", cfg_s5)

    # -------------------------------------------------------------------------
    # 3. Feature Selection: No Gyro
    # -------------------------------------------------------------------------
    # Removing ["robot/back_imu", ["...gyro..."]]
    # We need to redefine input_vars completely.
    # Baseline has: 
    # - ["robot/back_imu", ["accel_x", "accel_y", "accel_z", "gyro_x", "gyro_y", "gyro_z"]]
    # - ["robot/left", ["hip_angle"]]
    # - ...
    
    input_vars_no_gyro = [
        ["robot/back_imu", ["accel_x", "accel_y", "accel_z"]],
        ["robot/left", ["hip_angle"]],
        ["robot/right", ["hip_angle"]]
    ]
    
    cfg_no_gyro = {
        "exp_name": "exp_no_gyro",
        "input_vars": input_vars_no_gyro,
        "data": { "use_complementary_filter_euler": False },
        "shared": copy.deepcopy(common_shared)
    }
    # Override shared input vars just in case, or ensure input_vars at root takes precedence
    # compare_results checks root first.
    save_yaml("configs/exp_no_gyro.yaml", cfg_no_gyro)

    # -------------------------------------------------------------------------
    # 4. Attention TCN
    # -------------------------------------------------------------------------
    cfg_attn = {
        "exp_name": "exp_attention",
        "model": {
            "type": "AttentionTCN",
            "attention_type": "temporal",
            "attention_heads": 4
        },
        "data": { "use_complementary_filter_euler": False }
    }
    cfg_attn = inject_common(cfg_attn)
    save_yaml("configs/exp_attention.yaml", cfg_attn)

    # -------------------------------------------------------------------------
    # 5. Euler Angles (Complementary Filter) - NEW
    # -------------------------------------------------------------------------
    # We need to change input_vars to use 'derived/euler' instead of raw IMU.
    # The 'derived/euler' group is intercepted in model_training.py.
    # Vars: ['roll', 'pitch', 'yaw']
    
    input_vars_euler = [
        ["derived/euler", ["roll", "pitch", "yaw"]],
        ["robot/left", ["hip_angle"]],
        ["robot/right", ["hip_angle"]]
    ]
    
    cfg_euler = {
        "exp_name": "exp_euler_angles",
        "input_vars": input_vars_euler, # Override inputs
        "data": {
            "use_complementary_filter_euler": True # Enable Logic
        },
        "shared": copy.deepcopy(common_shared)
    }
    save_yaml("configs/exp_euler_angles.yaml", cfg_euler)

    # -------------------------------------------------------------------------
    # 6. Shallow TCN (Lightweight)
    # -------------------------------------------------------------------------
    cfg_shallow = {
        "exp_name": "exp_shallow_tcn",
        "model": {
            "type": "TCN",
            "channels": [32, 64], # Only 2 layers
            "kernel_size": 3,
            "dropout": 0.1
        },
        "data": { "use_complementary_filter_euler": False }
    }
    cfg_shallow = inject_common(cfg_shallow)
    save_yaml("configs/exp_shallow_tcn.yaml", cfg_shallow)



if __name__ == "__main__":
    main()
