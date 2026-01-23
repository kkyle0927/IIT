
import h5py

data_path = '251030_combined_data.h5'

with h5py.File(data_path, 'r') as f:
    if 'S004' in f:
        print("\n--- S004 / accel_sine / trial_01 Structure ---")
        try:
            # Try to go into trial_01
            g = f['S004']['accel_sine']['trial_01']
            print("Keys inside trial_01:", list(g.keys()))
            
            # Check if Robot is there
            if 'Robot' in g:
                print(">>> FOUND 'Robot' group!")
                print("Keys inside Robot:", list(g['Robot'].keys()))
            else:
                print(">>> 'Robot' group NOT found in trial_01")
        except KeyError:
            print("Could not access S004/accel_sine/trial_01")
            
        print("\n--- Structure of f['S004']['accel_sine'] itself ---")
        print(f['S004']['accel_sine'].keys())
