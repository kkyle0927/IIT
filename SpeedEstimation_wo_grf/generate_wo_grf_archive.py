import copy
import os
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parent
OUT_DIR = ROOT / "configs" / "archive_season1" / "archive_1"


SUBJECTS = [f"m2_S00{i}" for i in range(1, 9)]
CONDITIONS = [
    "level_075mps", "level_100mps", "level_125mps",
    "accel_sine", "decline_5deg", "incline_10deg", "stopandgo",
]


SPEED_OUTPUT = [["common", ["v_Y_true"]]]
CONTACT_OUTPUT = [["forceplate/grf/left", ["contact_l"]], ["forceplate/grf/right", ["contact_r"]]]

BODYFRAME = ["derived/body_frame_imu", [
    "accel_right", "accel_forward", "accel_up",
    "gyro_right", "gyro_forward", "gyro_up",
    "forward_accel_overground",
]]
RAW_IMU = ["robot/back_imu", ["roll", "pitch", "yaw", "vel_forward"]]
MOCAP_KIN = ["mocap/kin_q", [
    "hip_flexion_l", "hip_flexion_r", "knee_angle_l", "knee_angle_r", "ankle_angle_l", "ankle_angle_r",
]]
MOCAP_KIN_DOT = ["mocap/kin_q", [
    "hip_flexion_l_dot", "hip_flexion_r_dot", "knee_angle_l_dot", "knee_angle_r_dot", "ankle_angle_l_dot", "ankle_angle_r_dot",
]]
MOCAP_KIN_DDOT = ["mocap/kin_q", [
    "hip_flexion_l_ddot", "hip_flexion_r_ddot", "knee_angle_l_ddot", "knee_angle_r_ddot", "ankle_angle_l_ddot", "ankle_angle_r_ddot",
]]
ROBOT_LEFT = ["robot/left", ["hip_angle", "thigh_angle", "torque"]]
ROBOT_RIGHT = ["robot/right", ["hip_angle", "thigh_angle", "torque"]]
ROBOT_TORQUES = [["robot/left", ["torque"]], ["robot/right", ["torque"]]]
SUB_INFO = ["sub_info", ["height", "weight"]]
IMUINT = ["derived/imu_integrated_speed", ["imu_integrated_speed"]]
IMU_CAUSAL = ["derived/imu_speed_causal", ["imu_speed_causal"]]
STRIDE = ["derived/stride_progression", ["stride_speed_robot"]]
INTERACTION = ["derived/interaction_power", [
    "interaction_power_l", "interaction_power_r", "interaction_work_pos", "interaction_work_neg",
]]
CONTACT_PHASE = ["derived/contact_estimate", ["contact_prob_l_phase", "contact_prob_r_phase"]]
CONTACT_KIN = ["derived/contact_estimate", ["contact_prob_l_kin", "contact_prob_r_kin"]]
CONTACT_FUSION = ["derived/contact_estimate", ["contact_prob_l_fusion", "contact_prob_r_fusion"]]
MODELCOM_PHASE = ["derived/model_com_speed_nogrf", ["model_com_speed_nogrf_phase"]]
MODELCOM_KIN = ["derived/model_com_speed_nogrf", ["model_com_speed_nogrf_kin"]]
MODELCOM_FUSION = ["derived/model_com_speed_nogrf", ["model_com_speed_nogrf_fusion"]]


def _base_cfg():
    return {
        "cv_mode": "loso",
        "shared": {
            "data_sources": {
                "milestone2": {"path": "./combined_data_S008.h5", "prefix": "m2_", "exclude_subjects": []}
            },
            "subjects": SUBJECTS,
            "conditions": CONDITIONS,
            "input_vars": [],
            "output_vars": SPEED_OUTPUT,
            "lpf_cutoff": 0.5,
            "lpf_order": 4,
            "data": {
                "window_size": 200,
                "stride": 5,
                "stride_inf": 1,
                "y_delay": 5,
                "est_tick_ranges": [5],
                "normalize": True,
                "time_window_output": 1,
            },
            "model": {},
        },
        "02_train": {
            "data": {
                "window_size": 200,
                "stride": 5,
                "stride_inf": 1,
                "y_delay": 5,
                "est_tick_ranges": [5],
                "normalize": True,
                "time_window_output": 1,
                "normalize_type": "standard",
                "use_physical_velocity_model": False,
                "use_gait_phase": False,
            },
            "model": {},
            "loader": {"batch_size": 128, "val_batch_size": 256},
            "train": {
                "task_type": "regression",
                "loss": "huber",
                "huber_delta": 0.1,
                "epochs": 30,
                "lr": 0.001,
                "wd": 0.001,
                "amp": True,
                "device": "cuda",
                "save_dir": "./runs_tcn",
                "early_stop_patience": 10,
                "early_stop_min_delta": 1e-4,
                "early_stop_warmup": 10,
                "k_fold": 4,
                "cos_T0": 10,
                "cos_Tmult": 1,
                "eta_min": 1e-5,
                "augment_noise_std": 0.03,
                "smoothness_loss_weight": 0.0,
                "kinematic_loss_weight_pos": 0.0,
                "kinematic_loss_weight_limit": 0,
                "kinematic_loss_degrees": False,
                "kinematic_loss_tolerance": 0.05,
                "train_loss_target": 0.0,
            },
        },
        "03_eval": {
            "split": {
                "mode": "loso",
                "level": "subject",
                "seed": 42,
                "ratio": [0.7, 0.15, 0.15],
                "manual": {"val": ["m2_S007"], "test": ["m2_S008"]},
                "save_to": None,
                "loso_subjects": ["m2_S008"],
            },
            "data": {
                "window_size": 200,
                "stride": 5,
                "stride_inf": 1,
                "y_delay": 5,
                "est_tick_ranges": [5],
                "normalize": True,
                "time_window_output": 1,
            },
            "train": {"device": "cuda"},
            "scaler_path": "./runs_tcn/scaler.npz",
        },
    }


def _size_model(model_type: str, size: str, prior_idxs=None):
    if size == "base":
        channels, gru_hidden, head_hidden = [32, 32, 64, 64, 128, 128], 32, [64]
    else:
        channels, gru_hidden, head_hidden = [24, 24, 48, 48, 64, 64], 24, [48]

    model = {
        "type": model_type,
        "gru_hidden": gru_hidden,
        "channels": channels,
        "kernel_size": 5,
        "dropout": 0.2,
        "head_hidden": head_hidden,
        "head_dropout": 0.4 if size == "compact" else 0.5,
        "model_norm": "batch",
        "use_input_norm": True,
        "input_norm_type": "instance",
        "ar_feedback": {"enable": False},
        "use_norm": True,
    }
    if model_type == "FusionTCN_GRU":
        model["fusion"] = {
            "prior_input_channel_indices": prior_idxs or [],
            "logit_bias": [0.0] + [1.0] * len(prior_idxs or []),
            "weight_temperature": 1.0,
        }
    return model


def _flatten_input_dim(input_vars):
    return sum(len(vs) for _, vs in input_vars)


def _find_prior_indices(input_vars, feature_names):
    indices = []
    offset = 0
    for _, vs in input_vars:
        for v in vs:
            if v in feature_names:
                indices.append(offset)
            offset += 1
    return indices


def _pglpf_cfg():
    return {"accelerations": 8.0, "imu": 8.0, "velocities": 4.0}


def _make_speed_cfg(exp_name, input_vars, model_type, size, window, lpf_variant, prior_vars=None):
    cfg = _base_cfg()
    cfg["exp_name"] = exp_name
    cfg["shared"]["input_vars"] = copy.deepcopy(input_vars)
    cfg["shared"]["data"]["window_size"] = window
    cfg["02_train"]["data"]["window_size"] = window
    cfg["03_eval"]["data"]["window_size"] = window
    if lpf_variant == "pglpf":
        cfg["shared"]["input_lpf_per_group"] = _pglpf_cfg()
        cfg["02_train"]["data"]["input_lpf_per_group"] = _pglpf_cfg()

    prior_idxs = None
    if model_type == "FusionTCN_GRU":
        prior_idxs = _find_prior_indices(input_vars, prior_vars or [])
    model_cfg = _size_model(model_type, size, prior_idxs=prior_idxs)
    cfg["shared"]["model"] = copy.deepcopy(model_cfg)
    cfg["02_train"]["model"] = copy.deepcopy(model_cfg)
    return cfg


def _make_contact_cfg(exp_name, input_vars, size, window, lpf_variant):
    cfg = _base_cfg()
    cfg["exp_name"] = exp_name
    cfg["shared"]["input_vars"] = copy.deepcopy(input_vars)
    cfg["shared"]["output_vars"] = copy.deepcopy(CONTACT_OUTPUT)
    cfg["shared"]["lpf_cutoff"] = None
    cfg["shared"]["lpf_order"] = None
    cfg["shared"]["data"]["window_size"] = window
    cfg["shared"]["data"]["y_delay"] = 0
    cfg["shared"]["data"]["est_tick_ranges"] = [0]
    cfg["02_train"]["data"]["window_size"] = window
    cfg["02_train"]["data"]["y_delay"] = 0
    cfg["02_train"]["data"]["est_tick_ranges"] = [0]
    cfg["03_eval"]["data"]["window_size"] = window
    cfg["03_eval"]["data"]["y_delay"] = 0
    cfg["03_eval"]["data"]["est_tick_ranges"] = [0]
    if lpf_variant == "pglpf":
        cfg["shared"]["input_lpf_per_group"] = _pglpf_cfg()
        cfg["02_train"]["data"]["input_lpf_per_group"] = _pglpf_cfg()
    model_cfg = _size_model("TCN_GRU", size)
    cfg["shared"]["model"] = copy.deepcopy(model_cfg)
    cfg["02_train"]["model"] = copy.deepcopy(model_cfg)
    cfg["02_train"]["train"]["huber_delta"] = 0.05
    return cfg


SPEED_FAMILIES = {
    "raw_full": [MOCAP_KIN, MOCAP_KIN_DOT, MOCAP_KIN_DDOT, RAW_IMU, *ROBOT_TORQUES, SUB_INFO],
    "raw_lite": [MOCAP_KIN, MOCAP_KIN_DOT, MOCAP_KIN_DDOT, RAW_IMU, SUB_INFO],
    "bf_imuint": [MOCAP_KIN, MOCAP_KIN_DOT, MOCAP_KIN_DDOT, BODYFRAME, IMUINT, *ROBOT_TORQUES, SUB_INFO],
    "bf_imucausal": [MOCAP_KIN, MOCAP_KIN_DOT, MOCAP_KIN_DDOT, BODYFRAME, IMU_CAUSAL, *ROBOT_TORQUES, SUB_INFO],
    "bf_interaction": [BODYFRAME, ROBOT_LEFT, ROBOT_RIGHT, INTERACTION, SUB_INFO],
    "bf_modelcom_phase": [BODYFRAME, ROBOT_LEFT, ROBOT_RIGHT, MODELCOM_PHASE, SUB_INFO],
    "bf_modelcom_kin": [BODYFRAME, ROBOT_LEFT, ROBOT_RIGHT, MODELCOM_KIN, SUB_INFO],
    "bf_modelcom_fusion": [BODYFRAME, ROBOT_LEFT, ROBOT_RIGHT, MODELCOM_FUSION, SUB_INFO],
    "fusion_contact_kin": [BODYFRAME, ROBOT_LEFT, ROBOT_RIGHT, IMU_CAUSAL, STRIDE, CONTACT_KIN, SUB_INFO],
    "fusion_allpriors": [BODYFRAME, ROBOT_LEFT, ROBOT_RIGHT, IMU_CAUSAL, STRIDE, INTERACTION, MODELCOM_FUSION, CONTACT_FUSION, SUB_INFO],
}

CONTACT_FAMILIES = {
    "contact_bodyframe": [BODYFRAME, SUB_INFO],
    "contact_robotkin": [ROBOT_LEFT, ROBOT_RIGHT, SUB_INFO],
    "contact_bodyframe_robot": [BODYFRAME, ROBOT_LEFT, ROBOT_RIGHT, SUB_INFO],
    "contact_rich": [MOCAP_KIN, MOCAP_KIN_DOT, BODYFRAME, ROBOT_LEFT, ROBOT_RIGHT, SUB_INFO],
}


def save_yaml(cfg, path):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as f:
        yaml.safe_dump(cfg, f, sort_keys=False, indent=2)


def main():
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    windows = [300, 200, 150]
    sizes = ["base", "compact"]
    lpf_variants = ["none", "pglpf"]
    count = 0

    for fam_name, inputs in SPEED_FAMILIES.items():
        model_type = "FusionTCN_GRU" if fam_name.startswith("fusion_") else "TCN_GRU"
        prior_vars = []
        if fam_name == "fusion_contact_kin":
            prior_vars = ["imu_speed_causal", "stride_speed_robot"]
        elif fam_name == "fusion_allpriors":
            prior_vars = ["imu_speed_causal", "stride_speed_robot", "model_com_speed_nogrf_fusion"]

        for window in windows:
            for size in sizes:
                for lpf in lpf_variants:
                    exp_name = f"exp_WG_S1A1_{fam_name}_w{window}_{size}_{lpf}"
                    cfg = _make_speed_cfg(exp_name, inputs, model_type, size, window, lpf, prior_vars=prior_vars)
                    save_yaml(cfg, OUT_DIR / f"{exp_name}.yaml")
                    count += 1

    for fam_name, inputs in CONTACT_FAMILIES.items():
        for window in windows:
            for size in sizes:
                for lpf in lpf_variants:
                    exp_name = f"exp_WG_S1A1_{fam_name}_w{window}_{size}_{lpf}"
                    cfg = _make_contact_cfg(exp_name, inputs, size, window, lpf)
                    save_yaml(cfg, OUT_DIR / f"{exp_name}.yaml")
                    count += 1

    print(f"[DONE] generated {count} configs under {OUT_DIR}")


if __name__ == "__main__":
    main()
