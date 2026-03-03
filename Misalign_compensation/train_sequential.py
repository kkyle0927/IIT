import os
import glob
import subprocess
import time
import sys

def main():
    # 1. GPU Setup
    # The user requested GPU 3, but nvidia-smi -L only shows GPU 0.
    # We will set it to what the user requested, but print a warning.
    gpu_id = os.environ.get("CUDA_VISIBLE_DEVICES", "3")
    print(f"[INFO] Using CUDA_VISIBLE_DEVICES={gpu_id}")
    
    # 2. Find all configs
    configs = sorted(glob.glob("configs/**/*.yaml", recursive=True))
    # Filter out baseline if needed? train.sh ran all.
    valid_configs = [c for c in configs if os.path.exists(c) and "baseline.yaml" not in c]
    # Add baseline first? Or just follow train.sh
    # train.sh line 153: configs = sorted(glob.glob("configs/**/*.yaml", recursive=True))
    # It would include baseline.yaml.
    all_configs = sorted(glob.glob("configs/**/*.yaml", recursive=True))
    
    if not all_configs:
        print("[ERROR] No YAML configs found in configs/")
        return

    print(f"[INFO] Found {len(all_configs)} experiments. Starting sequential training...")
    
    os.makedirs("logs", exist_ok=True)
    
    for i, cfg in enumerate(all_configs):
        cfg_name = os.path.basename(cfg).replace(".yaml", "")
        log_path = f"logs/{cfg_name}.log"
        
        print(f"\n[{i+1}/{len(all_configs)}] Launching {cfg_name}...")
        print(f"      Log: {log_path}")
        
        # Build command
        cmd = ["python", "model_training.py", "--config", cfg]
        
        # Set environment
        env = os.environ.copy()
        env["CUDA_VISIBLE_DEVICES"] = gpu_id
        env["PYTHONUNBUFFERED"] = "1"
        
        start_time = time.time()
        
        try:
            with open(log_path, "w", encoding='utf-8') as f:
                # Start process
                process = subprocess.Popen(
                    cmd,
                    env=env,
                    stdout=f,
                    stderr=subprocess.STDOUT,
                    text=True
                )
                
                # Wait for completion
                process.wait()
                
                if process.returncode == 0:
                    print(f"      Completed successfully in {time.time() - start_time:.1f}s")
                else:
                    print(f"      [ERROR] Process failed with return code {process.returncode}")
                    
        except KeyboardInterrupt:
            print("\n[STOP] Training interrupted by user.")
            process.terminate()
            break
        except Exception as e:
            print(f"      [ERROR] Unexpected error: {e}")

    print("\n[INFO] All sequential training jobs finished.")
    print("Starting comparison...")
    subprocess.run(["python", "compare_results.py", "--auto"])

if __name__ == "__main__":
    main()
