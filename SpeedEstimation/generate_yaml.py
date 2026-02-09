
import os
import yaml
import copy
import sys

def load_yaml(path):
    with open(path, 'r') as f:
        return yaml.safe_load(f)

def save_yaml(path, data):
    with open(path, 'w') as f:
        yaml.dump(data, f, sort_keys=False, default_flow_style=False)

def ensure_model_dict(cfg):
    # If 02_train.model is None, initialize it as empty dict
    if cfg['02_train'].get('model') is None:
        cfg['02_train']['model'] = {}
    return cfg

def main():
    base_config_path = "configs/baseline.yaml"
    if not os.path.exists(base_config_path):
        print(f"Error: {base_config_path} not found.")
        return

    base_config = load_yaml(base_config_path)
    output_dir = "configs/advanced"
    os.makedirs(output_dir, exist_ok=True)
    
    print(f"[Generator] Base config loaded from {base_config_path}")

    # =================================================================
    # 1. Frequency Penalty Loss
    # =================================================================
    # Goal: penalize high freq
    cfg = copy.deepcopy(base_config)
    cfg = ensure_model_dict(cfg)
    exp_name = "Exp_Freq_Penalty_Loss"
    cfg['02_train']['experiment_name'] = exp_name
    
    # Toggle existing keys
    cfg['02_train']['train']['loss_penalty'] = "frequency_cutoff_penalty"
    cfg['02_train']['train']['loss_penalty_weight'] = 0.5
    cfg['02_train']['train']['loss_penalty_cutoff'] = "reference"
    
    save_yaml(f"{output_dir}/{exp_name}.yaml", cfg)
    print(f"[Generated] {exp_name}")

    # =================================================================
    # 2. Leg Length Normalization
    # =================================================================
    cfg = copy.deepcopy(base_config)
    cfg = ensure_model_dict(cfg)
    exp_name = "Exp_Leg_Length_Norm"
    cfg['02_train']['experiment_name'] = exp_name
    
    # Toggle data keys
    if 'data' in cfg['02_train']:
        cfg['02_train']['data']['normalize_type'] = "leg_length_scaled"
        cfg['02_train']['data']['use_physical_velocity_model'] = True
        
    save_yaml(f"{output_dir}/{exp_name}.yaml", cfg)
    print(f"[Generated] {exp_name}")

    # =================================================================
    # 3. Gait Phase Input
    # =================================================================
    cfg = copy.deepcopy(base_config)
    cfg = ensure_model_dict(cfg)
    exp_name = "Exp_Gait_Phase_Input"
    cfg['02_train']['experiment_name'] = exp_name
    
    # Add input manually (still needed as baseline inputs are fixed list)
    new_input = ["derived", ["gait_phase_L", "gait_phase_R"]]
    if '01_construction' in cfg and 'inputs' in cfg['01_construction']:
        cfg['01_construction']['inputs'].append(new_input)
        
    cfg['02_train']['data']['use_gait_phase'] = True
    
    save_yaml(f"{output_dir}/{exp_name}.yaml", cfg)
    print(f"[Generated] {exp_name}")

    # =================================================================
    # 4. Stance Gating
    # =================================================================
    cfg = copy.deepcopy(base_config)
    cfg = ensure_model_dict(cfg)
    exp_name = "Exp_Stance_Gating"
    cfg['02_train']['experiment_name'] = exp_name
    
    # Toggle model keys
    cfg['02_train']['model']['type'] = "StanceGatedTCN"
    cfg['02_train']['model']['gating_signal'] = "contact"
        
    save_yaml(f"{output_dir}/{exp_name}.yaml", cfg)
    print(f"[Generated] {exp_name}")

    # =================================================================
    # 5. Attention Mechanism
    # =================================================================
    cfg = copy.deepcopy(base_config)
    cfg = ensure_model_dict(cfg)
    exp_name = "Exp_Attention_Model"
    cfg['02_train']['experiment_name'] = exp_name
    
    # Toggle model keys
    cfg['02_train']['model']['type'] = "AttentionTCN"
    cfg['02_train']['model']['attention_type'] = "temporal"
    cfg['02_train']['model']['attention_heads'] = 4
        
    save_yaml(f"{output_dir}/{exp_name}.yaml", cfg)
    print(f"[Generated] {exp_name}")

if __name__ == "__main__":
    main()
