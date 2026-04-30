import yaml
import copy
import os

def load_baseline(path="configs/baseline.yaml"):
    with open(path, "r") as f:
        # yaml.safe_load(f) results in a dictionary.
        # Note: & anchors and * aliases are resolved by safe_load.
        return yaml.safe_load(f)

def save_yaml(cfg, path):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as f:
        yaml.safe_dump(cfg, f, sort_keys=False, indent=2)

def make_exp_smoothness(base):
    """실험 1: Smoothness Loss 강화"""
    cfg = copy.deepcopy(base)
    cfg["02_train"]["train"]["smoothness_loss_weight"] = 2.0
    cfg["exp_name"] = "Exp_smoothness"
    return cfg

def make_exp_leglen_norm(base):
    """실험 2: Leg Length 기반 수평 속도 정규화"""
    cfg = copy.deepcopy(base)
    # shared.input_vars에 추가 (이미 resolved 됨)
    cfg["shared"]["input_vars"].append(["derived/leg_velocity", ["derived/leg_vel_l", "derived/leg_vel_r"]])
    cfg["exp_name"] = "Exp_leglen_norm"
    return cfg

def make_exp_gait_phase(base):
    """실험 3: Gait Phase 입력 추가"""
    cfg = copy.deepcopy(base)
    cfg["shared"]["input_vars"].append(["derived", ["derived/gait_phase_left", "derived/gait_phase_right"]])
    cfg["exp_name"] = "Exp_gait_phase"
    return cfg

def make_exp_stance_gate(base):
    """실험 4: Stance Gating"""
    cfg = copy.deepcopy(base)
    cfg["02_train"]["model"]["type"] = "StanceGatedTCN"
    cfg["shared"]["input_vars"].append(["derived", ["derived/contact_left", "derived/contact_right"]])
    cfg["exp_name"] = "Exp_stance_gate"
    return cfg

def make_exp_attention(base):
    """실험 5: Temporal Attention"""
    cfg = copy.deepcopy(base)
    cfg["02_train"]["model"]["type"] = "AttentionTCN"
    cfg["02_train"]["model"]["attention_type"] = "temporal"
    cfg["exp_name"] = "Exp_attention"
    return cfg

def make_exp_instance_norm(base):
    """실험 6: Instance Normalization (Subject-wise)"""
    cfg = copy.deepcopy(base)
    cfg["use_input_norm"] = False # Global norm 비활성화
    cfg["02_train"]["data"]["normalize_type"] = "subject" # 피험자별 정규화 활성화
    cfg["02_train"]["model"]["model_norm"] = "instance" # 모델 내부 InstanceNorm1d 사용
    cfg["exp_name"] = "Exp_instance_norm"
    return cfg

if __name__ == "__main__":
    base = load_baseline()
    exps = [
        # make_exp_smoothness(base),
        # make_exp_leglen_norm(base),
        # make_exp_gait_phase(base),
        # make_exp_stance_gate(base),
        # make_exp_attention(base),
        make_exp_instance_norm(base)
    ]
    
    for cfg in exps:
        # [NEW] Ensure LOSO filter and Batch Size are consistent based on baseline or explicit
        # baseline.yaml already has these, so copy.deepcopy(base) should inherit them.
        # But we double check and can explicitly set them if needed.
        
        path = f"configs/{cfg['exp_name']}.yaml"
        save_yaml(cfg, path)
        print(f"[SAVED] {path}")
