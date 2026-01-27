
import yaml
import os
import itertools
from copy import deepcopy

# Base Configuration (Default Template)
base_config = {
    # Data / Path
    "data_path": "C:/Github/IIT/SpeedEstimation/data/preprocessed", 
    # "subjects": ... Removed to inherit from base_config.yaml (S001...S008)
    
    # Training
    "epochs": 50,
    "batch_size": 256,
    "lr": 0.001,
    "patience": 10,
    
    # Architecture Defaults
    "kernel_size": 3,
    "dropout_p": 0.1,
}

# Grid Search Space
search_space = {
    "Architecture": [
        {"name": "Tiny",   "tcn_channels": [16, 32, 64],  "lstm_hidden": 32,  "mlp_hidden": 32},
        {"name": "Small",  "tcn_channels": [32, 64, 128], "lstm_hidden": 64,  "mlp_hidden": 64},
        {"name": "Medium", "tcn_channels": [64, 128, 256],"lstm_hidden": 128, "mlp_hidden": 128},
    ],
    # Independent Normalization Options
    "InputNorm": [False, True],
    "TCN_Norm":  [None, "batch"],
    "LSTM_Norm": [False, True], # LayerNorm
    
    "TimeWindow": [40, 80, 100] 
}

def generate():
    save_dir = "configs/grid_search"
    # Clean up old configs if needed, or just overwrite
    os.makedirs(save_dir, exist_ok=True)
    
    # Generate Combinations
    keys = search_space.keys()
    values = search_space.values()
    combinations = list(itertools.product(*values))
    
    print(f"Generating {len(combinations)} configurations...")
    
    for i, combo in enumerate(combinations):
        # combo is a tuple (Arch, InputNorm, TCN_Norm, LSTM_Norm, TimeWindow)
        arch, in_norm, tcn_norm, lstm_ln, win = combo
        
        # Short string codes for filename
        in_str = "InNorm" if in_norm else "NoIn"
        tcn_str = "TcnBN" if tcn_norm == "batch" else "TcnNo"
        lstm_str = "LstmLN" if lstm_ln else "LstmNo"
        
        # Create Config Name
        config_name = f"GS_{i+1:03d}_{arch['name']}_{in_str}_{tcn_str}_{lstm_str}_W{win}"
        
        # Build Config Dictionary
        new_config = {
            "experiment_name": config_name,
            "time_window_input": win,
            "epochs": 25,
            
            # Architecture
            "tcn_channels": arch["tcn_channels"],
            "lstm_hidden": arch["lstm_hidden"],
            "mlp_hidden": arch["mlp_hidden"],
            
            # Normalization
            "use_input_norm": in_norm,
            "tcn_norm": tcn_norm,
            "lstm_use_ln": lstm_ln,
            "mlp_norm": None, # Defaulting to None as not specified
            
            # Preprocessing
            "lpf_cutoff": 0.5
        }
        
        # Write to YAML
        filename = f"{save_dir}/{config_name}.yaml"
        with open(filename, "w") as f:
            yaml.dump(new_config, f, default_flow_style=False)
            
        print(f"  -> Created {filename}")

if __name__ == "__main__":
    generate()
