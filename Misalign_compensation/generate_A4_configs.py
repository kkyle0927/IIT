#!/usr/bin/env python3
"""Generate Archive_4 experiment YAML configs based on A3_quaternion baseline."""
import yaml
import copy
import os

with open("configs/archive_3/exp_A3_quaternion.yaml", "r") as f:
    base = yaml.safe_load(f)

def save_yaml(cfg, name, folder="configs/archive_4"):
    os.makedirs(folder, exist_ok=True)
    cfg["exp_name"] = name
    path = f"{folder}/{name}.yaml"
    with open(path, "w") as f:
        yaml.dump(cfg, f, default_flow_style=False, allow_unicode=True, sort_keys=False)
    print(f"  Created: {path}")

def get_model(cfg):
    return cfg["shared"]["model"]

def get_data(cfg):
    return cfg["shared"]["data"]

def get_train(cfg):
    return cfg["02_train"]["train"]

def get_inputs(cfg):
    return cfg["shared"]["input_vars"]

# === Update baseline.yaml ===
print("[0] Updating configs/baseline.yaml (quaternion-based)")
bl = copy.deepcopy(base)
bl["exp_name"] = "baseline"
with open("configs/baseline.yaml", "w") as f:
    yaml.dump(bl, f, default_flow_style=False, allow_unicode=True, sort_keys=False)
print("  Updated: configs/baseline.yaml")

# === A4 Experiments ===

# 1. quat_kinetics: +thigh_angle_dot, torque_dot
print("[1/10] exp_A4_quat_kinetics")
c = copy.deepcopy(base)
get_inputs(c).append(["robot/left", ["thigh_angle_dot", "torque_dot"]])
get_inputs(c).append(["robot/right", ["thigh_angle_dot", "torque_dot"]])
save_yaml(c, "exp_A4_quat_kinetics")

# 2. quat_motor: +motor_angle
print("[2/10] exp_A4_quat_motor")
c = copy.deepcopy(base)
get_inputs(c).append(["robot/left", ["motor_angle"]])
get_inputs(c).append(["robot/right", ["motor_angle"]])
save_yaml(c, "exp_A4_quat_motor")

# 3. quat_kin_motor: +kinetics +motor_angle (all)
print("[3/10] exp_A4_quat_kin_motor")
c = copy.deepcopy(base)
get_inputs(c).append(["robot/left", ["thigh_angle_dot", "torque_dot"]])
get_inputs(c).append(["robot/right", ["thigh_angle_dot", "torque_dot"]])
get_inputs(c).append(["robot/left", ["motor_angle"]])
get_inputs(c).append(["robot/right", ["motor_angle"]])
save_yaml(c, "exp_A4_quat_kin_motor")

# 4. quat_dropout_015
print("[4/10] exp_A4_quat_dropout_015")
c = copy.deepcopy(base)
get_model(c)["dropout"] = 0.15
get_model(c)["head_dropout"] = 0.15
save_yaml(c, "exp_A4_quat_dropout_015")

# 5. quat_dropout_01
print("[5/10] exp_A4_quat_dropout_01")
c = copy.deepcopy(base)
get_model(c)["dropout"] = 0.1
get_model(c)["head_dropout"] = 0.1
save_yaml(c, "exp_A4_quat_dropout_01")

# 6. quat_k7: kernel_size=7
print("[6/10] exp_A4_quat_k7")
c = copy.deepcopy(base)
get_model(c)["kernel_size"] = 7
save_yaml(c, "exp_A4_quat_k7")

# 7. quat_win300: window_size=300
print("[7/10] exp_A4_quat_win300")
c = copy.deepcopy(base)
get_data(c)["window_size"] = 300
c["02_train"]["data"]["window_size"] = 300
save_yaml(c, "exp_A4_quat_win300")

# 8. quat_noise_001: augment_noise_std=0.01
print("[8/10] exp_A4_quat_noise_001")
c = copy.deepcopy(base)
get_train(c)["augment_noise_std"] = 0.01
save_yaml(c, "exp_A4_quat_noise_001")

# 9. quat_noise_005: augment_noise_std=0.05
print("[9/10] exp_A4_quat_noise_005")
c = copy.deepcopy(base)
get_train(c)["augment_noise_std"] = 0.05
save_yaml(c, "exp_A4_quat_noise_005")

# 10. quat_kin_k7: +kinetics + kernel=7
print("[10/10] exp_A4_quat_kin_k7")
c = copy.deepcopy(base)
get_inputs(c).append(["robot/left", ["thigh_angle_dot", "torque_dot"]])
get_inputs(c).append(["robot/right", ["thigh_angle_dot", "torque_dot"]])
get_model(c)["kernel_size"] = 7
save_yaml(c, "exp_A4_quat_kin_k7")

print("\nDone! 10 A4 configs + baseline updated.")
