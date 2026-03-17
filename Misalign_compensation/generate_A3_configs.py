#!/usr/bin/env python3
"""Generate Archive_3 experiment YAML configs based on new baseline (thigh_lowdrop)."""
import yaml
import copy
import os

# Load thigh_lowdrop as the new baseline template
with open("configs/archive_2/exp_A2_thigh_lowdrop.yaml", "r") as f:
    base = yaml.safe_load(f)

def save_yaml(cfg, name):
    cfg["exp_name"] = name
    path = f"configs/{name}.yaml"
    with open(path, "w") as f:
        yaml.dump(cfg, f, default_flow_style=False, allow_unicode=True, sort_keys=False)
    print(f"  Created: {path}")

# =============================================================================
# Step 1: New baseline (= thigh_lowdrop as-is, just rename)
# =============================================================================
print("[1/11] Creating new baseline.yaml")
baseline = copy.deepcopy(base)
save_yaml(baseline, "baseline")

# =============================================================================
# Step 2: Archive_3 experiments
# =============================================================================

# Helper to get mutable refs
def get_model(cfg):
    return cfg["shared"]["model"]

def get_data(cfg):
    return cfg["shared"]["data"]

def get_train(cfg):
    return cfg["02_train"]["train"]

def get_inputs(cfg):
    return cfg["shared"]["input_vars"]

def set_inputs(cfg, inputs):
    cfg["shared"]["input_vars"] = inputs

# --- 1. exp_A3_win300_k7: window=300, kernel=7 ---
print("[2/11] exp_A3_win300_k7")
c = copy.deepcopy(base)
get_data(c)["window_size"] = 300
c["02_train"]["data"]["window_size"] = 300
get_model(c)["kernel_size"] = 7
save_yaml(c, "exp_A3_win300_k7")

# --- 2. exp_A3_kinetics: + thigh_angle_dot, torque_dot ---
print("[3/11] exp_A3_kinetics")
c = copy.deepcopy(base)
inputs = get_inputs(c)
inputs.append(["robot/left", ["thigh_angle_dot", "torque_dot"]])
inputs.append(["robot/right", ["thigh_angle_dot", "torque_dot"]])
save_yaml(c, "exp_A3_kinetics")

# --- 3. exp_A3_quaternion: + quaternion ---
print("[4/11] exp_A3_quaternion")
c = copy.deepcopy(base)
inputs = get_inputs(c)
inputs.append(["robot/back_imu", ["quat_w", "quat_x", "quat_y", "quat_z"]])
save_yaml(c, "exp_A3_quaternion")

# --- 4. exp_A3_motor_angle: + motor_angle ---
print("[5/11] exp_A3_motor_angle")
c = copy.deepcopy(base)
inputs = get_inputs(c)
inputs.append(["robot/left", ["motor_angle"]])
inputs.append(["robot/right", ["motor_angle"]])
save_yaml(c, "exp_A3_motor_angle")

# --- 5. exp_A3_full_combo: win300_k7 + kinetics ---
print("[6/11] exp_A3_full_combo")
c = copy.deepcopy(base)
get_data(c)["window_size"] = 300
c["02_train"]["data"]["window_size"] = 300
get_model(c)["kernel_size"] = 7
inputs = get_inputs(c)
inputs.append(["robot/left", ["thigh_angle_dot", "torque_dot"]])
inputs.append(["robot/right", ["thigh_angle_dot", "torque_dot"]])
save_yaml(c, "exp_A3_full_combo")

# --- 6. exp_A3_dropout_01: dropout=0.1 ---
print("[7/11] exp_A3_dropout_01")
c = copy.deepcopy(base)
get_model(c)["dropout"] = 0.1
get_model(c)["head_dropout"] = 0.1
save_yaml(c, "exp_A3_dropout_01")

# --- 7. exp_A3_dropout_03: dropout=0.3 ---
print("[8/11] exp_A3_dropout_03")
c = copy.deepcopy(base)
get_model(c)["dropout"] = 0.3
get_model(c)["head_dropout"] = 0.3
save_yaml(c, "exp_A3_dropout_03")

# --- 8. exp_A3_epochs_50: epochs=50 ---
print("[9/11] exp_A3_epochs_50")
c = copy.deepcopy(base)
get_train(c)["epochs"] = 50
save_yaml(c, "exp_A3_epochs_50")

# --- 9. exp_A3_lr_0005: lr=0.0005 ---
print("[10/11] exp_A3_lr_0005")
c = copy.deepcopy(base)
get_train(c)["lr"] = 0.0005
save_yaml(c, "exp_A3_lr_0005")

# --- 10. exp_A3_channels_wide: [64,128,128,256] ---
print("[11/11] exp_A3_channels_wide")
c = copy.deepcopy(base)
get_model(c)["channels"] = [64, 128, 128, 256]
save_yaml(c, "exp_A3_channels_wide")

print("\nDone! 11 configs created (1 baseline + 10 experiments)")
