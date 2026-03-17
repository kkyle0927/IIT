#!/usr/bin/env python3
"""Generate Archive_5 experiment YAML configs based on A4 best (quat_dropout_015)."""
import yaml
import copy
import os

with open("configs/archive_4/exp_A4_quat_dropout_015.yaml", "r") as f:
    base = yaml.safe_load(f)

def save_yaml(cfg, name, folder="configs/archive_5"):
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

def get_train_data(cfg):
    return cfg["02_train"]["data"]

def get_train(cfg):
    return cfg["02_train"]["train"]

def get_inputs(cfg):
    return cfg["shared"]["input_vars"]

def enable_residual(cfg):
    """Enable residual_target in both shared.data and 02_train.data."""
    get_data(cfg)["residual_target"] = True
    get_train_data(cfg)["residual_target"] = True

# === Update baseline.yaml ===
print("[0] Updating configs/baseline.yaml (quat_dropout_015-based)")
bl = copy.deepcopy(base)
bl["exp_name"] = "baseline"
with open("configs/baseline.yaml", "w") as f:
    yaml.dump(bl, f, default_flow_style=False, allow_unicode=True, sort_keys=False)
print("  Updated: configs/baseline.yaml")

# === A5 Experiments ===

# 1. exp_A5_residual: residual target learning
print("[1/10] exp_A5_residual")
c = copy.deepcopy(base)
enable_residual(c)
save_yaml(c, "exp_A5_residual")

# 2. exp_A5_res_mech_power: residual + mech_power input
print("[2/10] exp_A5_res_mech_power")
c = copy.deepcopy(base)
enable_residual(c)
get_inputs(c).append(["derived/mech_power", ["left", "right"]])
save_yaml(c, "exp_A5_res_mech_power")

# 3. exp_A5_res_deflection: residual + deflection input
print("[3/10] exp_A5_res_deflection")
c = copy.deepcopy(base)
enable_residual(c)
get_inputs(c).append(["derived/deflection", ["left", "right"]])
save_yaml(c, "exp_A5_res_deflection")

# 4. exp_A5_mech_power: normal target + mech_power input
print("[4/10] exp_A5_mech_power")
c = copy.deepcopy(base)
get_inputs(c).append(["derived/mech_power", ["left", "right"]])
save_yaml(c, "exp_A5_mech_power")

# 5. exp_A5_deflection: normal target + deflection input
print("[5/10] exp_A5_deflection")
c = copy.deepcopy(base)
get_inputs(c).append(["derived/deflection", ["left", "right"]])
save_yaml(c, "exp_A5_deflection")

# 6. exp_A5_res_dropout_01: residual + dropout=0.1 (lower than base 0.15)
print("[6/10] exp_A5_res_dropout_01")
c = copy.deepcopy(base)
enable_residual(c)
get_model(c)["dropout"] = 0.1
get_model(c)["head_dropout"] = 0.1
save_yaml(c, "exp_A5_res_dropout_01")

# 7. exp_A5_res_kin_power: residual + kinetics + mech_power
print("[7/10] exp_A5_res_kin_power")
c = copy.deepcopy(base)
enable_residual(c)
get_inputs(c).append(["robot/left", ["thigh_angle_dot", "torque_dot"]])
get_inputs(c).append(["robot/right", ["thigh_angle_dot", "torque_dot"]])
get_inputs(c).append(["derived/mech_power", ["left", "right"]])
save_yaml(c, "exp_A5_res_kin_power")

# 8. exp_A5_gated_tcn: StanceGatedTCN model
print("[8/10] exp_A5_gated_tcn")
c = copy.deepcopy(base)
get_model(c)["type"] = "StanceGatedTCN"
save_yaml(c, "exp_A5_gated_tcn")

# 9. exp_A5_res_noise_001: residual + noise augmentation
print("[9/10] exp_A5_res_noise_001")
c = copy.deepcopy(base)
enable_residual(c)
get_train(c)["augment_noise_std"] = 0.01
save_yaml(c, "exp_A5_res_noise_001")

# 10. exp_A5_res_k7: residual + kernel_size=7
print("[10/10] exp_A5_res_k7")
c = copy.deepcopy(base)
enable_residual(c)
get_model(c)["kernel_size"] = 7
save_yaml(c, "exp_A5_res_k7")

print("\nDone! 10 A5 configs + baseline updated.")
