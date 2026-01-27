# python compare_experiments.py 으로 실행

import os
import json
import tkinter as tk
from tkinter import filedialog
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np

def select_files():
    root = tk.Tk()
    root.withdraw() # Hide main window
    file_paths = filedialog.askopenfilenames(
        title="Select metrics.json files",
        filetypes=[("JSON files", "*.json")],
        initialdir="./experiments"
    )
    return file_paths

def load_metrics(file_paths):
    data = []
    for path in file_paths:
        try:
            with open(path, 'r') as f:
                content = json.load(f)
            
            # Helper to safely get config values
            cfg = content.get('config', {})
            
            # Determine a readable name (folder name or config name)
            exp_name = os.path.basename(os.path.dirname(path))
            
            entry = {
                'Name': exp_name,
                
                # Performance
                'MAE': content.get('test_mae', np.nan),
                'RMSE': content.get('test_rmse', np.nan),
                'Loss': content.get('test_huber', np.nan),
                
                # Resources
                'Params': content.get('n_params', 0),
                'Size(MB)': content.get('model_size_mb', 0),
                'Latency(ms)': content.get('host_latency_ms', 0),
                
                # Config/Conditions (Key Hyperparams)
                'TCN_Channels': str(cfg.get('tcn_channels', '?')),
                'Kernel': cfg.get('kernel_size', '?'),
                'LSTM_Nodes': cfg.get('lstm_hidden', '?'),
                'LSTM_Layers': cfg.get('lstm_layers', '?'),
                'MLP_Nodes': cfg.get('mlp_hidden', '?'),
                'Input_Dim': cfg.get('input_dim', '?'), # Might not be directly in config if inferred
                'LPF': cfg.get('lpf_cutoff', 'None'),
                'Features': len(cfg.get('input_vars', [])) if isinstance(cfg.get('input_vars'), list) else '?'
            }
            data.append(entry)
        except Exception as e:
            print(f"Error loading {path}: {e}")
            
    return pd.DataFrame(data)

def visualize_comparison(df):
    if df.empty:
        print("No data to visualize.")
        return

    # Set style
    sns.set_style("whitegrid")
    
    # 1. Performance Comparison (Bar Chart)
    # Pivot for melting if needed, or just plot directly
    
    fig = plt.figure(figsize=(18, 10))
    gs = fig.add_gridspec(2, 3)
    
    # Plot 1: MAE Comparison
    ax1 = fig.add_subplot(gs[0, 0])
    sns.barplot(data=df, x='Name', y='MAE', ax=ax1, palette='viridis')
    ax1.set_title('Test MAE (Lower is Better)')
    ax1.set_xticklabels(ax1.get_xticklabels(), rotation=45, ha='right')
    
    # Plot 2: Latency vs MAE (Trade-off)
    ax2 = fig.add_subplot(gs[0, 1])
    sns.scatterplot(data=df, x='Latency(ms)', y='MAE', hue='Name', s=150, ax=ax2, style='Name')
    ax2.set_title('Latency vs Accuracy (Trade-off)')
    # Add labels
    for i, row in df.iterrows():
        ax2.text(row['Latency(ms)'], row['MAE'], row['Name'], fontsize=8, alpha=0.7)
        
    # Plot 3: Params Comparison
    ax3 = fig.add_subplot(gs[0, 2])
    sns.barplot(data=df, x='Name', y='Params', ax=ax3, palette='magma')
    ax3.set_title('Parameter Count (Model Complexity)')
    ax3.set_xticklabels(ax3.get_xticklabels(), rotation=45, ha='right')
    
    # Plot 4: Detailed Table in Plot (Optional, but pandas printing is better)
    # Instead, let's do RMSE
    ax4 = fig.add_subplot(gs[1, 0])
    sns.barplot(data=df, x='Name', y='RMSE', ax=ax4, palette='rocket')
    ax4.set_title('Test RMSE')
    ax4.set_xticklabels(ax4.get_xticklabels(), rotation=45, ha='right')
    
    # Plot 5: Model Size vs MAE
    ax5 = fig.add_subplot(gs[1, 1])
    sns.scatterplot(data=df, x='Size(MB)', y='MAE', hue='Name', s=150, ax=ax5)
    ax5.set_title('Model Size (MB) vs MAE')
    
    # Plot 6: Hyperparam Summary (Text)
    ax6 = fig.add_subplot(gs[1, 2])
    ax6.axis('off')
    ax6.set_title('Key Configuration Differences')
    
    # Create a text summary of what varies
    # Find columns that vary
    constant_cols = [c for c in df.columns if df[c].nunique() <= 1]
    varying_cols = [c for c in df.columns if c not in constant_cols and c not in ['Name', 'MAE', 'RMSE', 'Loss', 'Params', 'Size(MB)', 'Latency(ms)']]
    
    summary_txt = "Varying Configs:\n"
    if not varying_cols:
        summary_txt += "All selected models have identical configs\n(except result metrics)"
    else:
        # Show table of varying configs
        sub_df = df[['Name'] + varying_cols]
        # Format as string
        txt_table = sub_df.to_string(index=False, justify='left', max_colwidth=20)
        summary_txt += txt_table
        
    ax6.text(0, 1, summary_txt, fontsize=10, verticalalignment='top', fontfamily='monospace')

    plt.tight_layout()
    plt.show()

def main():
    print("Please select 'metrics.json' files from your experiment folders...")
    files = select_files()
    
    if not files:
        print("No files selected.")
        return
        
    df = load_metrics(files)
    
    if df.empty:
        print("Could not load metrics.")
        return
        
    # Sort by MAE
    df = df.sort_values(by='MAE')
    
    # Reorder columns for display
    display_cols = ['Name', 'MAE', 'RMSE', 'Latency(ms)', 'Params', 'Size(MB)'] 
    # Add other cols that might be interesting
    extra_cols = [c for c in df.columns if c not in display_cols]
    
    print("\n" + "="*80)
    print(" EXPERIMENT COMPARISON SUMMARY")
    print("="*80)
    print(df[display_cols].to_string(index=False))
    print("\n" + "-"*80)
    print(" Full Details:")
    print(df[display_cols + extra_cols].to_string(index=False))
    print("="*80)
    
    visualize_comparison(df)

if __name__ == "__main__":
    main()
