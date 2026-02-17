
import h5py
import sys

def inspect(path):
    print(f"Inspecting {path}...")
    try:
        with h5py.File(path, 'r') as f:
            print("Top level keys:", list(f.keys()))
            # Check S004
            if 'S004' in f:
                print("S004 found.")
                grp = f['S004']
                print("Conditions:", list(grp.keys()))
                # Check level_08mps
                if 'level_08mps' in grp:
                    cond = grp['level_08mps']
                    # Check lv0
                    if 'lv0' in cond:
                         lv0 = cond['lv0']
                         # Check trial_01
                         if 'trial_01' in lv0:
                             tr = lv0['trial_01']
                             print("trial_01 keys:", list(tr.keys()))
                             if 'derived' in tr:
                                 print("derived keys:", list(tr['derived'].keys()))
                             else:
                                 print("derived group NOT found in trial_01")
            else:
                print("S004 NOT found.")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    inspect("combined_data_milestone1.h5")
    inspect("combined_data.h5")
