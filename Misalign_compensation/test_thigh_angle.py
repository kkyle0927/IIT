import h5py
import numpy as np
import matplotlib.pyplot as plt
import os

def test_thigh_angle():
    data_path = r"c:\Github\IIT\SpeedEstimation\combined_data.h5"
    if not os.path.exists(data_path):
        print(f"Error: {data_path} not found.")
        return

    # M2 데이터 중 하나 선택
    sub = "m2_S001"
    cond = "level_075mps"
    
    with h5py.File(data_path, 'r') as f:
        # Check sub keys in case the prefix isn't there
        if sub not in f:
            if "S001" in f:
                sub = "S001"
            else:
                print(f"Data for {sub} not found. Keys: {list(f.keys())}")
                return
                
        if cond not in f[sub]:
            print(f"Data for {sub}/{cond} not found. Keys: {list(f[sub].keys())}")
            return
            
        lv = list(f[sub][cond].keys())[0] # e.g., 'lv0'
        trial = list(f[sub][cond][lv].keys())[0] # e.g., 'trial_01'
        print(f"Testing on trial: {sub}/{cond}/{lv}/{trial}")
        
        trial_grp = f[sub][cond][lv][trial]
        
        # 데이터 로드
        try:
            robot_l = trial_grp['robot']['left']['hip_angle'][:]
            mocap_hip_l = trial_grp['mocap']['kin_q']['hip_flexion_l'][:]
            mocap_pelvis_list = trial_grp['mocap']['kin_q']['pelvis_list'][:]
        except KeyError as e:
            print(f"Key Error: {e}")
            if 'mocap' in trial_grp and 'kin_q' in trial_grp['mocap']:
                print("Available mocap/kin_q keys:", list(trial_grp['mocap']['kin_q'].keys()))
            return

        # 1D 배열로 변환
        robot_l = np.array(robot_l).flatten()
        mocap_hip_l = np.array(mocap_hip_l).flatten()
        mocap_pelvis_list = np.array(mocap_pelvis_list).flatten()
        
        # 길이 맞추기
        min_len = min(len(robot_l), len(mocap_hip_l), len(mocap_pelvis_list))
        robot_l = robot_l[:min_len]
        mocap_hip_l = mocap_hip_l[:min_len]
        mocap_pelvis_list = mocap_pelvis_list[:min_len]

        # 두 가지 연산 케이스 
        # Case 1: hip_flexion + pelvis_list
        mocap_thigh_plus = mocap_hip_l + mocap_pelvis_list
        # Case 2: hip_flexion - pelvis_list
        mocap_thigh_minus = mocap_hip_l - mocap_pelvis_list
        
        # 1초 (100 샘플) 평균 
        calib_samples = min(100, min_len)
        robot_mean = np.mean(robot_l[:calib_samples])
        plus_mean = np.mean(mocap_thigh_plus[:calib_samples])
        minus_mean = np.mean(mocap_thigh_minus[:calib_samples])
        
        print(f"First 1s mean - Robot: {robot_mean:.2f}")
        print(f"First 1s mean - MoCap(+): {plus_mean:.2f}")
        print(f"First 1s mean - MoCap(-): {minus_mean:.2f}")
        
        # 시각화
        plt.figure(figsize=(15, 10))
        
        # Zoomed plot (첫 1000 샘플만)
        plot_len = min(1000, min_len)
        t = np.arange(plot_len)
        
        # 보정 안 한 원본 트렌드 비교
        plt.subplot(2, 1, 1)
        plt.plot(t, robot_l[:plot_len], label='Robot Hip Angle', color='red')
        plt.plot(t, mocap_hip_l[:plot_len], label='MoCap Hip Flexion Only', color='grey', linestyle=':')
        plt.plot(t, mocap_thigh_plus[:plot_len], label='MoCap (Hip + Pelvis List)', color='blue')
        plt.plot(t, mocap_thigh_minus[:plot_len], label='MoCap (Hip - Pelvis List)', color='green')
        plt.title(f"Raw Trend Comparison (First {plot_len} samples)")
        plt.legend()
        plt.grid(True)
        
        # 보정 후 비교 (초기 offset 적용)
        plt.subplot(2, 1, 2)
        offset_plus = plus_mean - robot_mean
        offset_minus = minus_mean - robot_mean
        
        robot_calib_plus = robot_l[:plot_len] + offset_plus
        robot_calib_minus = robot_l[:plot_len] + offset_minus
        
        plt.plot(t, mocap_thigh_plus[:plot_len], label='MoCap (+)', color='blue')
        plt.plot(t, robot_calib_plus, label='Robot (Calibrated for +)', color='red', linestyle='--')
        
        plt.plot(t, mocap_thigh_minus[:plot_len], label='MoCap (-)', color='green')
        plt.plot(t, robot_calib_minus, label='Robot (Calibrated for -)', color='orange', linestyle='-.')
        
        plt.title("Calibrated Comparison")
        plt.legend()
        plt.grid(True)
        
        plt.tight_layout()
        plt.savefig("thigh_angle_test.png")
        print("Saved plot to thigh_angle_test.png")

if __name__ == "__main__":
    test_thigh_angle()
