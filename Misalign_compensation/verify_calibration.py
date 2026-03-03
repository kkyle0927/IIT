import yaml
import h5py
import numpy as np
import matplotlib.pyplot as plt
import os
import sys

# Add current path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from model_training import extract_condition_data_v2

def main():
    config_path = "configs/baseline.yaml"
    with open(config_path, 'r', encoding='utf-8') as f:
        cfg = yaml.safe_load(f)

    # Variables from baseline
    input_vars = cfg['shared']['input_vars']
    output_vars = cfg['shared']['output_vars']
    use_comp = cfg['shared']['data'].get('use_complementary_filter_euler', False)
    
    data_path = cfg['shared']['data_sources']['milestone2']['path']
    
    # We will test on S001 (which is from m2), level_075mps
    sub = "S001"
    cond = "level_075mps"
    
    print(f"Loading data from {data_path} for {sub}/{cond}...")
    
    with h5py.File(data_path, 'r') as f:
        X_trials, Y_trials = extract_condition_data_v2(
            f, sub, cond, input_vars, output_vars,
            use_complementary_filter_euler=use_comp
        )
        
    if not X_trials:
        print("No data extracted.")
        return
        
    # Take the first trial
    X = X_trials[0]
    Y = Y_trials[0]
    
    print(f"Extracted X shape: {X.shape}, Y shape: {Y.shape}")
    
    # In baseline.yaml, input_vars are:
    # 0,1,2: derived/euler (roll, pitch, yaw)
    # 3: robot/left/hip_angle
    # 4: robot/left/torque
    # 5: robot/right/hip_angle
    # 6: robot/right/torque
    
    # output_vars:
    # 0: mocap/kin_q/hip_flexion_l (actually thigh angle now)
    # 1: mocap/kin_q/hip_flexion_r
    
    robot_hip_l = X[:, 3]
    mocap_thigh_l = Y[:, 0]
    
    # Plot
    plt.figure(figsize=(15, 6))
    plt.plot(robot_hip_l, label='Robot Left Hip (Calibrated, Input)', color='green', alpha=0.7)
    plt.plot(mocap_thigh_l, label='MoCap Left Thigh (Target, Output)', color='blue', alpha=0.7, linestyle='--')
    
    plt.title(f"Verification of Calibration - {sub} {cond} Trial 1")
    plt.xlabel("Time Steps (100Hz)")
    plt.ylabel("Angle (degrees)")
    plt.legend()
    plt.grid(True)
    
    out_file = "calibration_verification.png"
    plt.savefig(out_file)
    print(f"Successfully saved plot to {out_file}")

if __name__ == "__main__":
    main()
