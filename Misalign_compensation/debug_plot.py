
import sys
import torch
from pathlib import Path
from compare_results import plot_detailed_condition_trajectories

def main():
    if len(sys.argv) < 2:
        print("Usage: python debug_plot.py <exp_dir>")
        return

    exp_path = Path(sys.argv[1])
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    
    print(f"Running plot_detailed_condition_trajectories on {exp_path}...")
    try:
        plot_detailed_condition_trajectories(exp_path, exp_path, device)
        print("Plotting function finished.")
    except Exception as e:
        print(f"Plotting failed: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()
