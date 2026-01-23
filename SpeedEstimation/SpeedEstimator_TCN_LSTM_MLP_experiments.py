
import os
os.environ["OMP_NUM_THREADS"] = "1"
os.environ["MKL_NUM_THREADS"] = "1"
os.environ["NUMEXPR_NUM_THREADS"] = "1"
os.environ["MKL_THREADING_LAYER"] = "GNU"
os.environ["KMP_DUPLICATE_LIB_OK"] = "TRUE"

import torch
torch.set_num_threads(1)
torch.set_num_interop_threads(1)

import torch.nn as nn
from torch.utils.data import Dataset, DataLoader
import numpy as np
import random
import h5py
import matplotlib.pyplot as plt
import time
from scipy.signal import butter, filtfilt

# 데이터셋 로드
data_path = '251030_combined_data.h5'

# subject, condition 정보
sub_names = ['S004','S005','S006','S007','S008','S009','S010','S011']
cond_names = [
    'accel_sine','asym_30deg','asym_60deg','cadence_120p','cadence_90p',
    'crouch','decline_5deg','incline_10deg','level_08mps','level_12mps','level_16mps'
]
data_length = 12000

# -------------------------------------------------------------------------------------------------
# Helper Functions (Dataset, Plotting, etc) -> Copy from original
# -------------------------------------------------------------------------------------------------

def get_ground_contact(grf_fz, threshold=20.0):
    return (np.nan_to_num(grf_fz) > threshold).astype(float)

def butter_lowpass_filter(data, cutoff, fs, order=4):
    from scipy.signal import butter, filtfilt
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    if data.ndim == 1:
        y = filtfilt(b, a, data)
    else:
        y = filtfilt(b, a, data, axis=0)
    return y

class LiveTrainingPlotter:
    def __init__(self):
        self.fig, self.ax = plt.subplots(figsize=(8, 5))
        self.ax.set_xlabel('Epoch')
        self.ax.set_ylabel('Loss')
        self.ax.grid(True)
        plt.ion()
        plt.show(block=False)
        self.train_line = None
        self.val_line = None
        self.epochs = []
        self.train_losses = []
        self.val_losses = []

    def start_session(self, model_name, seed):
        self.ax.clear()
        self.ax.set_xlabel('Epoch')
        self.ax.set_ylabel('Loss')
        self.ax.set_title(f"Training {model_name} (seed={seed})")
        self.ax.grid(True)
        self.train_line, = self.ax.plot([], [], 'b-', label='Train Loss')
        self.val_line,   = self.ax.plot([], [], 'r--', label='Val Loss')
        self.ax.legend()
        self.epochs = []
        self.train_losses = []
        self.val_losses = []
        plt.pause(0.1)

    def update_epoch(self, epoch, train_loss, val_loss):
        self.epochs.append(epoch)
        self.train_losses.append(train_loss)
        self.val_losses.append(val_loss)
        self.train_line.set_data(self.epochs, self.train_losses)
        self.val_line.set_data(self.epochs, self.val_losses)
        self.ax.relim()
        self.ax.autoscale_view()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

class WindowDataset(Dataset):
    def __init__(self, X, Y, target_mode="mean"):
        self.X = torch.from_numpy(X).float()
        Y = torch.from_numpy(Y).float()
        if target_mode == "mean":
            self.Y = Y.mean(dim=1)
        elif target_mode == "last":
            self.Y = Y[:, -1, :]
        elif target_mode == "sequence":
            self.Y = Y
        else:
            raise ValueError("target_mode must be 'mean', 'last', or 'sequence'")

    def __len__(self):
        return self.X.shape[0]

    def __getitem__(self, idx):
        return self.X[idx], self.Y[idx]

def extract_condition_data_v2(
    f, sub, cond, 
    input_vars, output_vars,
    lpf_cutoff=None, lpf_order=4, fs=100
):
    """
    input_vars, output_vars: list of (group_path, [var_names...])
    Return: (T, InDim), (T, OutDim)
    """
    
    # -------------------------------------------------------------------------
    # 1. Load Inputs & Augment
    # -------------------------------------------------------------------------
    in_list = []
    
    # Locate Trial Group (e.g. 'trial_01')
    if sub not in f or cond not in f[sub]:
        print(f"[DEBUG] {sub}/{cond}: Missing group")
        return None, None
        
    cond_group = f[sub][cond]
    trial_keys = list(cond_group.keys())
    if len(trial_keys) == 0:
        print(f"[DEBUG] {sub}/{cond}: No trial keys found")
        return None, None
        
    # Assuming the first key is the trial
    trial_name = trial_keys[0]
    trial_group = cond_group[trial_name]
    
    # buffers for augmentation
    kin_q_arrays = []  # list of (data_array, var_name)
    left_fz = None
    right_fz = None
    
    # (A) Standard Load
    for gpath, vars in input_vars:
        # Check in trial_group
        if gpath not in trial_group:
            # Fallback check: maybe gpath implies trial? No, H5 structure is S/C/T/G
            print(f"[DEBUG] {sub}/{cond}/{trial_name}: Missing group {gpath}")
            return None, None
        
        grp = trial_group[gpath]
        for v in vars:
            if v not in grp:
                print(f"[DEBUG] {sub}/{cond}/{trial_name}: Missing var {v} in {gpath}")
                return None, None
            
            raw_data = grp[v][:]
            
            # Apply LPF
            data = np.nan_to_num(raw_data)
            if lpf_cutoff is not None:
                data = butter_lowpass_filter(data, lpf_cutoff, fs, lpf_order)
            
            in_list.append(data.reshape(-1, 1))
            
            # Identify specific vars for augmentation
            if 'kin_q' in gpath and 'dot' not in gpath:
                kin_q_arrays.append(data)
                
            # Forces (for potential contact detection)
            # path is typically MoCap/grf_measured/left/force/Fz
            if 'grf_measured/left/force' in gpath and v == 'Fz':
                left_fz = raw_data
            if 'grf_measured/right/force' in gpath and v == 'Fz':
                right_fz = raw_data

    # (B) Augment: Joint Velocity & Acceleration
    aug_chem_cutoff = 60.0
    if aug_chem_cutoff >= fs / 2:
        aug_chem_cutoff = fs / 2 - 1.0  # safe clamp (e.g. 49Hz)
    
    for q_data in kin_q_arrays:
        # Velocity
        vel = np.gradient(q_data, 1.0/fs)
        vel = butter_lowpass_filter(vel, aug_chem_cutoff, fs, order=4)
        in_list.append(vel.reshape(-1, 1))
        
        # Acceleration
        acc = np.gradient(vel, 1.0/fs)
        acc = butter_lowpass_filter(acc, aug_chem_cutoff, fs, order=4)
        in_list.append(acc.reshape(-1, 1))

    # (C) Augment: Contact Features
    # If Fz was not in input_vars, fetch it from trial_group
    def get_fz_from_trial(side):
        # path: MoCap/grf_measured/{side}/force/Fz
        try:
            return trial_group[f'MoCap/grf_measured/{side}/force']['Fz'][:]
        except:
            return None

    if left_fz is None: left_fz = get_fz_from_trial('left')
    if right_fz is None: right_fz = get_fz_from_trial('right')
        
    if left_fz is not None:
        c_l = get_ground_contact(left_fz, threshold=20.0)
        in_list.append(c_l.reshape(-1, 1))
        
    if right_fz is not None:
        c_r = get_ground_contact(right_fz, threshold=20.0)
        in_list.append(c_r.reshape(-1, 1))

    if not in_list:
        print(f"[DEBUG] {sub}/{cond}: in_list is empty")
        return None, None
    in_arr = np.hstack(in_list)

    # -------------------------------------------------------------------------
    # 2. Output
    # -------------------------------------------------------------------------
    out_list = []
    for gpath, vars in output_vars:
        if gpath not in trial_group:
            print(f"[DEBUG] {sub}/{cond}: Missing output group {gpath}")
            return None, None
        grp = trial_group[gpath]
        for v in vars:
            if v not in grp:
                print(f"[DEBUG] {sub}/{cond}: Missing output var {v}")
                return None, None
            d = grp[v][:]
            d = np.nan_to_num(d)
            if lpf_cutoff is not None:
                d = butter_lowpass_filter(d, lpf_cutoff, fs, lpf_order)
            out_list.append(d.reshape(-1, 1))
            
    if not out_list:
        print(f"[DEBUG] {sub}/{cond}: out_list is empty")
        return None, None
    out_arr = np.hstack(out_list)

    # Length check
    min_len = min(in_arr.shape[0], out_arr.shape[0])
    return in_arr[:min_len], out_arr[:min_len]

def build_nn_dataset(
    data_path, 
    sub_names, cond_names,
    input_vars, output_vars,
    time_window_input, time_window_output, stride,
    subject_selection=None,
    condition_selection=None,
    debug_plot=False,
    lpf_cutoff=None,
    lpf_order=4
):
    X_list = []
    Y_list = []

    print(f"[DEBUG] build_nn_dataset called with subjects={sub_names} conditions={cond_names}")
    with h5py.File(data_path, 'r') as f:
        print(f"[DEBUG] H5 file opened. Keys: {list(f.keys())}")
        for sub in sub_names:
            if subject_selection:
                if sub not in subject_selection.get('include', sub_names):
                    # print(f"[DEBUG] Skipping sub {sub}")
                    continue
                if sub in subject_selection.get('exclude', []):
                    continue
            
            print(f"[DEBUG] Processing sub {sub}")
            
            if sub not in f:
                print(f"[DEBUG] Sub {sub} not in H5 file!")
                continue

            for cond in cond_names:
                if condition_selection:
                    if condition_selection.get('include') and cond not in condition_selection['include']:
                        continue
                    if cond in condition_selection.get('exclude', []):
                        continue
                        
                if cond not in f[sub]:
                    print(f"[DEBUG] Cond {cond} not in f[{sub}]! Keys: {list(f[sub].keys())}")
                    continue

                # Data load
                X_arr, Y_arr = extract_condition_data_v2(
                    f, sub, cond, input_vars, output_vars,
                    lpf_cutoff=lpf_cutoff, lpf_order=lpf_order
                )
                if X_arr is None:
                    print(f"[DEBUG] Extract returned None for {sub}/{cond}")
                    continue
                
                # Sliding Window
                prev_len = len(X_list)
                for i in range(0, len(X_arr) - time_window_input - time_window_output + 1, stride):
                    limit = i + time_window_input
                    end   = limit + time_window_output
                    
                    x_win = X_arr[i : limit, :]     # (Tw_in, Din)
                    y_win = Y_arr[limit : end, :]   # (Tw_out, Dout)
                    
                    X_list.append(x_win)
                    Y_list.append(y_win)
                
                if len(X_list) == prev_len:
                    print(f"[DEBUG] No windows created for {sub}/{cond}. X_arr len={len(X_arr)}")

    if len(X_list) == 0:
        print("[DEBUG] X_list is empty at end of build_nn_dataset")
        return np.array([]), np.array([])
    
    return np.array(X_list), np.array(Y_list)

def plot_model_summary(results):
    """
    results: list of (exp_name, seed, metrics_data, ckpt, ...)
    metrics_data: either float (old style loss) or dict (new style full metrics)
    """
    import matplotlib.pyplot as plt
    
    # Organize data
    # Structure: names -> list of metrics
    data = {}
    
    for row in results:
        name = row[0]
        # seed = row[1] 
        metrics_val = row[2]
        
        if name not in data:
            data[name] = {'huber': [], 'mae': [], 'rmse': [], 'params': []}
            
        if isinstance(metrics_val, dict):
            data[name]['huber'].append(metrics_val.get('test_huber', 0.0))
            data[name]['mae'].append(metrics_val.get('test_mae', 0.0))
            data[name]['rmse'].append(metrics_val.get('test_rmse', 0.0))
            data[name]['params'].append(metrics_val.get('n_params', 0))
        else:
            # Fallback if only float provided
            data[name]['huber'].append(float(metrics_val))
            
    names = sorted(data.keys())
    
    # Prepare Stats
    means = {k: [] for k in ['huber', 'mae', 'rmse', 'params']}
    stds  = {k: [] for k in ['huber', 'mae', 'rmse', 'params']}
    
    print("\n=== Experiment Summary ===")
    for n in names:
        d = data[n]
        print(f"[{n}]")
        for k in means.keys():
            if len(d[k]) > 0:
                m = np.mean(d[k])
                s = np.std(d[k])
                means[k].append(m)
                stds[k].append(s)
                print(f"  {k}: {m:.6f} (+/- {s:.6f})")
            else:
                means[k].append(0)
                stds[k].append(0)

    # Plotting 2x2
    fig, axes = plt.subplots(2, 2, figsize=(16, 12))
    
    # 1. Huber Loss
    ax = axes[0, 0]
    x_pos = np.arange(len(names))
    ax.bar(x_pos, means['huber'], yerr=stds['huber'], align='center', alpha=0.7, capsize=10, color='skyblue')
    ax.set_xticks(x_pos)
    ax.set_xticklabels(names, rotation=45, ha='right')
    ax.set_ylabel('Huber Loss')
    ax.set_title('Test Huber Loss')
    ax.grid(axis='y', linestyle='--', alpha=0.5)

    # 2. MAE
    ax = axes[0, 1]
    ax.bar(x_pos, means['mae'], yerr=stds['mae'], align='center', alpha=0.7, capsize=10, color='lightgreen')
    ax.set_xticks(x_pos)
    ax.set_xticklabels(names, rotation=45, ha='right')
    ax.set_ylabel('MAE')
    ax.set_title('Test MAE')
    ax.grid(axis='y', linestyle='--', alpha=0.5)

    # 3. RMSE
    ax = axes[1, 0]
    ax.bar(x_pos, means['rmse'], yerr=stds['rmse'], align='center', alpha=0.7, capsize=10, color='salmon')
    ax.set_xticks(x_pos)
    ax.set_xticklabels(names, rotation=45, ha='right')
    ax.set_ylabel('RMSE')
    ax.set_title('Test RMSE')
    ax.grid(axis='y', linestyle='--', alpha=0.5)
    
    # 4. Params vs Performance (Scatter)
    ax = axes[1, 1]
    # x-axis: params, y-axis: MAE (or Huber)
    # Scatter plot of means
    
    # Color map for names
    colors = plt.cm.get_cmap('tab10', len(names))
    
    for i, n in enumerate(names):
        p_mean = means['params'][i]
        mae_mean = means['mae'][i]
        
        ax.scatter(p_mean, mae_mean, s=100, color=colors(i), label=n, edgecolors='k')
        
    ax.set_xlabel('# Parameters')
    ax.set_ylabel('Test MAE')
    ax.set_title('Parameters vs. Performance (MAE)')
    ax.grid(True, linestyle='--', alpha=0.5)
    ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    
    plt.tight_layout()
    return fig

# -------------------------------------------------------------------------------------------------
# Model Definition: TCN_LSTM_MLP Only
# -------------------------------------------------------------------------------------------------

class TransposedBatchNorm1d(nn.Module):
    """
    BatchNorm1d wrapper for inputs of shape (B, L, C).
    Transposes to (B, C, L), applies BN, then transposes back to (B, L, C).
    """
    def __init__(self, num_features):
        super().__init__()
        self.bn = nn.BatchNorm1d(num_features)

    def forward(self, x):
        # x: (B, L, C)
        x = x.transpose(1, 2)  # (B, C, L)
        x = self.bn(x)
        x = x.transpose(1, 2)  # (B, L, C)
        return x

class TCNEncoder(nn.Module):
    def __init__(self, input_dim, channels=(64, 64, 128), kernel_size=3, dropout=0.1, norm_type=None):
        super().__init__()
        layers = []
        in_ch = input_dim
        for i, ch in enumerate(channels):
            dilation = 2 ** i
            
            block_layers = [
                nn.Conv1d(in_ch, ch, kernel_size, padding=0, dilation=dilation),
            ]
            
            if norm_type == 'batch':
                block_layers.append(nn.BatchNorm1d(ch))
            elif norm_type == 'layer':
                # LayerNorm for (B, C, L) -> GroupNorm(1, ch)
                block_layers.append(nn.GroupNorm(1, ch))
            
            block_layers.append(nn.ReLU())
            block_layers.append(nn.Dropout(dropout))
            
            layers += block_layers
            in_ch = ch
            
        self.network = nn.Sequential(*layers)
        self.out_ch = channels[-1]

    def forward(self, x):
        # x: (B, T, D) -> (B, D, T)
        x = x.transpose(1, 2)
        import torch.nn.functional as F
        y = x
        for layer in self.network:
            if isinstance(layer, nn.Conv1d):
                k = layer.kernel_size[0]
                d = layer.dilation[0]
                pad_left = (k - 1) * d
                y = F.pad(y, (pad_left, 0))
            y = layer(y)
        return y[:, :, -1]

class TCN_LSTM_MLP(nn.Module):
    def __init__(self, input_dim, output_dim, horizon, channels=(64,64,128), kernel_size=3, 
                 dropout=0.1, hidden_size=128, num_layers=1, mlp_hidden=128,
                 use_input_norm=False, 
                 tcn_norm=None, 
                 lstm_use_ln=False, 
                 mlp_norm=None):
        super().__init__()
        
        self.use_input_norm = use_input_norm
        if use_input_norm:
            self.input_norm = nn.LayerNorm(input_dim)
            
        # TCN Encoder
        self.enc = TCNEncoder(input_dim, channels, kernel_size, dropout, norm_type=tcn_norm)
        
        self.horizon = horizon
        self.output_dim = output_dim
        
        # LSTM
        self.lstm = nn.LSTM(
            input_size=self.enc.out_ch, hidden_size=hidden_size,
            num_layers=num_layers, batch_first=True,
            dropout=dropout if num_layers > 1 else 0.0
        )
        self.lstm_use_ln = lstm_use_ln
        if lstm_use_ln:
            self.lstm_ln = nn.LayerNorm(hidden_size)

        # MLP Head construction
        head_layers = [nn.Linear(hidden_size, mlp_hidden)]
        
        if mlp_norm == 'batch':
            head_layers.append(TransposedBatchNorm1d(mlp_hidden))
        elif mlp_norm == 'layer':
            head_layers.append(nn.LayerNorm(mlp_hidden))
            
        head_layers.append(nn.ReLU())
        head_layers.append(nn.Dropout(dropout))
        head_layers.append(nn.Linear(mlp_hidden, output_dim))
        
        self.head = nn.Sequential(*head_layers)

    def forward(self, x):
        # x: (B, T, D)
        if self.use_input_norm:
            x = self.input_norm(x)
            
        ctx = self.enc(x)  # (B, C)
        dec_in = ctx.unsqueeze(1).repeat(1, self.horizon, 1)  # (B, H, C)
        yseq, _ = self.lstm(dec_in)  # (B, H, hidden)
        
        if self.lstm_use_ln:
            yseq = self.lstm_ln(yseq)
            
        return self.head(yseq)       # (B, H, D)

# -------------------------------------------------------------------------------------------------
# Training Function (Accepts Config)
# -------------------------------------------------------------------------------------------------

def train_experiment(
    X_train, Y_train,
    X_val, Y_val,
    X_test, Y_test,
    config,
    seed=42,
    save_path="model_ckpt.pt",
    live_plotter=None
):
    # Extract Hyperparams
    batch_size = config.get("batch_size", 256)
    val_batch_size = config.get("val_batch_size", batch_size)
    epochs = config.get("epochs", 50)
    patience = config.get("patience", 10)
    lr = config.get("lr", 1e-3)
    weight_decay = config.get("weight_decay", 1e-4)
    dropout_p = config.get("dropout_p", 0.1)
    
    dropout_p = config.get("dropout_p", 0.1)
    
    # Normalization Flags
    use_input_norm = config.get("use_input_norm", False)
    tcn_norm = config.get("tcn_norm", None)
    lstm_use_ln = config.get("lstm_use_ln", False)
    mlp_norm = config.get("mlp_norm", None)
    
    # Model Architecture Params
    tcn_channels = config.get("tcn_channels", (64, 64, 128))
    kernel_size = config.get("kernel_size", 3)
    lstm_hidden = config.get("lstm_hidden", 128)
    lstm_layers = config.get("lstm_layers", 1)
    mlp_hidden  = config.get("mlp_hidden", 128)
    
    # Scheduler
    cos_T0 = config.get("cos_T0", 10)
    cos_Tmult = config.get("cos_Tmult", 1)
    eta_min = config.get("eta_min", 1e-5)
    
    # Loss
    huber_delta = config.get("huber_delta", 0.5)
    
    # Reproducibility
    torch.manual_seed(seed)
    np.random.seed(seed)
    random.seed(seed)
    
    # Dataset
    target_mode = "sequence"
    train_ds = WindowDataset(X_train, Y_train, target_mode)
    val_ds   = WindowDataset(X_val,   Y_val,   target_mode)
    test_ds  = WindowDataset(X_test,  Y_test,  target_mode)
    
    train_loader = DataLoader(train_ds, batch_size=batch_size, shuffle=True)
    val_loader   = DataLoader(val_ds, batch_size=val_batch_size, shuffle=False)
    test_loader  = DataLoader(test_ds, batch_size=val_batch_size, shuffle=False)
    
    input_dim = X_train.shape[2]
    horizon = Y_train.shape[1]
    output_dim = Y_train.shape[2]
    
    # Model Instantiation
    model = TCN_LSTM_MLP(
        input_dim, output_dim, horizon,
        channels=tcn_channels,
        kernel_size=kernel_size,
        dropout=dropout_p,
        hidden_size=lstm_hidden,
        num_layers=lstm_layers,
        mlp_hidden=mlp_hidden,
        use_input_norm=use_input_norm,
        tcn_norm=tcn_norm,
        lstm_use_ln=lstm_use_ln,
        mlp_norm=mlp_norm
    )
    
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model.to(device)
    
    optimizer = torch.optim.AdamW(model.parameters(), lr=lr, weight_decay=weight_decay)
    criterion = nn.HuberLoss(delta=huber_delta)
    
    scheduler = None
    if cos_T0 > 0:
        scheduler = torch.optim.lr_scheduler.CosineAnnealingWarmRestarts(
            optimizer, T_0=cos_T0, T_mult=cos_Tmult, eta_min=eta_min
        )
        
    best_val = float("inf")
    patience_cnt = 0
    
    print_every = 50
    
    for epoch in range(1, epochs + 1):
        t0 = time.time()
        model.train()
        running = 0.0
        n_batches = 0
        
        for bi, (xb, yb) in enumerate(train_loader, start=1):
            xb, yb = xb.to(device), yb.to(device)
            optimizer.zero_grad(set_to_none=True)
            pred = model(xb)
            loss = criterion(pred, yb)
            loss.backward()
            optimizer.step()
            running += loss.item()
            n_batches += 1
            
            if (bi % print_every) == 0:
                 avg = running / max(1, n_batches)
                 print(f"[Epoch {epoch:03d} - interim] loss={avg:.6f}", end='\r')
        
        train_loss = running / max(1, n_batches)
        
        model.eval()
        val_running = 0.0
        with torch.no_grad():
            for xb, yb in val_loader:
                xb, yb = xb.to(device), yb.to(device)
                val_running += criterion(model(xb), yb).item()
        val_loss = val_running / max(1, len(val_loader))
        
        if scheduler:
            scheduler.step(epoch - 1)
            
        cur_lr = optimizer.param_groups[0]["lr"]
        print(f"[Epoch {epoch:03d}/{epochs:03d}] train={train_loss:.6f} val={val_loss:.6f} lr={cur_lr:.2e} time={time.time()-t0:.1f}s")
        
        if live_plotter:
            live_plotter.update_epoch(epoch, train_loss, val_loss)
            
        if val_loss < best_val:
            best_val = val_loss
            patience_cnt = 0
            ckpt_obj = {
                "state_dict": model.state_dict(),
                "config": config,
                "metrics": {"best_val": best_val}
            }
            torch.save(ckpt_obj, save_path)
            print(f"  --> Saved Best: {best_val:.6f}")
        else:
            patience_cnt += 1
            if patience_cnt >= patience:
                print("  --> Early Stopping")
                break
                
    # Evaluate
    ckpt = torch.load(save_path, map_location=device)
    model.load_state_dict(ckpt["state_dict"])
    model.eval()
    
    test_running = 0.0
    with torch.no_grad():
        for xb, yb in test_loader:
            xb, yb = xb.to(device), yb.to(device)
            test_running += criterion(model(xb), yb).item()
    test_loss = test_running / max(1, len(test_loader))
    
    # Calculate MAE & RMSE
    preds_all, trues_all = [], []
    with torch.no_grad():
        for xb, yb in test_loader:
            xb = xb.to(device)
            pred = model(xb).cpu().numpy()
            preds_all.append(pred)
            trues_all.append(yb.numpy())
            
    P = np.concatenate(preds_all, axis=0) # (N, H, D)
    T = np.concatenate(trues_all, axis=0) # (N, H, D)
    err = P - T
    
    test_mae = np.mean(np.abs(err))
    test_rmse = np.sqrt(np.mean(err**2))
    
    n_params = sum(p.numel() for p in model.parameters() if p.requires_grad)
    
    return {
        "test_huber": test_loss,
        "test_mae": float(test_mae),
        "test_rmse": float(test_rmse),
        "n_params": n_params,
        "config": config
    }



# -------------------------------------------------------------------------------------------------
# Main Execution
# -------------------------------------------------------------------------------------------------

if __name__ == "__main__":
    # Preprocessing Config
    fs = 100
    time_window_input = 100
    time_window_output = 10
    stride = 10
    LPF_CUTOFF = 6.0
    LPF_ORDER = 5
    
    input_vars = [
        ('Robot', ['incPosLH', 'incPosRH']),
        ('Back_imu', ['Accel_X', 'Accel_Y', 'Accel_Z', 'Gyro_X', 'Gyro_Y', 'Gyro_Z', 'Pitch', 'Roll', 'Yaw']),
        ('MoCap/grf_measured/left/force', ['Fx', 'Fy', 'Fz']),
        ('MoCap/grf_measured/right/force', ['Fx', 'Fy', 'Fz']),
        ('MoCap/kin_q', ['ankle_angle_l', 'ankle_angle_r', 'hip_adduction_l', 'hip_adduction_r', 'hip_flexion_l', 'hip_flexion_r', 'hip_rotation_l', 'hip_rotation_r', 'knee_angle_l', 'knee_angle_r']),
        ('MoCap/kin_qdot', ['ankle_angle_l', 'ankle_angle_r', 'hip_adduction_l', 'hip_adduction_r', 'hip_flexion_l', 'hip_flexion_r', 'hip_rotation_l', 'hip_rotation_r', 'knee_angle_l', 'knee_angle_r'])
    ]
    output_vars = [('Common', ['v_Y_true', 'v_Z_true'])]

    TRAIN_SUBJECTS = ['S004', 'S005', 'S006', 'S007', 'S008', 'S009']
    VAL_SUBJECTS   = ['S010']
    TEST_SUBJECTS  = ['S011']

    def make_subject_selection(include_list):
        return {'include': include_list, 'exclude': []}

    CONDITION_SELECTION = {'include': None, 'exclude': []}
    
    print("Loading Data...")
    X_train, Y_train = build_nn_dataset(
        data_path, sub_names, cond_names, input_vars, output_vars,
        time_window_input, time_window_output, stride,
        subject_selection=make_subject_selection(TRAIN_SUBJECTS),
        condition_selection=CONDITION_SELECTION, lpf_cutoff=LPF_CUTOFF, lpf_order=LPF_ORDER
    )
    X_val, Y_val = build_nn_dataset(
        data_path, sub_names, cond_names, input_vars, output_vars,
        time_window_input, time_window_output, stride,
        subject_selection=make_subject_selection(VAL_SUBJECTS),
        condition_selection=CONDITION_SELECTION, lpf_cutoff=LPF_CUTOFF, lpf_order=LPF_ORDER
    )
    X_test, Y_test = build_nn_dataset(
        data_path, sub_names, cond_names, input_vars, output_vars,
        time_window_input, time_window_output, stride,
        subject_selection=make_subject_selection(TEST_SUBJECTS),
        condition_selection=CONDITION_SELECTION, lpf_cutoff=LPF_CUTOFF, lpf_order=LPF_ORDER
    )
    
    print(f"Train: {X_train.shape}, Val: {X_val.shape}, Test: {X_test.shape}")
    
    # -------------------------------------------------------------------------
    # Define Experiments (Modifications)
    # -------------------------------------------------------------------------
    
    base_config = {
        "epochs": 20, "patience": 5, "batch_size": 1024,
        "lr": 3e-4, "weight_decay": 1e-2, "huber_delta": 0.5,
        "tcn_channels": (64, 64, 128), "kernel_size": 3,
        "lstm_hidden": 128, "lstm_layers": 1, "mlp_hidden": 128,
        "dropout_p": 0.3,
        "use_input_norm": False,
        "tcn_norm": None,
        "lstm_use_ln": False,
        "mlp_norm": None
    }
    
    experiments = [
        ("Base", {}),
        ("Deep_TCN", {"tcn_channels": (64, 64, 128, 256)}),
        ("Wide_LSTM", {"lstm_hidden": 256, "mlp_hidden": 256}),
        ("Deep_LSTM", {"lstm_layers": 2}),
        ("Large_Kernel", {"kernel_size": 5}),
        ("High_Dropout", {"dropout_p": 0.5}),
        ("TCN_BN", {"tcn_norm": "batch"}),
        ("LSTM_LN", {"lstm_use_ln": True}),
        ("MLP_BN", {"mlp_norm": "batch"}),
        ("Mix_TCN(BN)_LSTM(LN)_MLP(BN)", {"tcn_norm": "batch", "lstm_use_ln": True, "mlp_norm": "batch"}),
        ("Input_Norm", {"use_input_norm": True})
    ]
    
    seeds = [42] # 시간 관계상 1개씩만, 필요하면 [42, 43, 44]
    results = []
    
    live_plotter = LiveTrainingPlotter()
    
    for exp_name, override in experiments:
        cfg = base_config.copy()
        cfg.update(override)
        
        for sd in seeds:
            full_name = f"{exp_name}_seed{sd}"
            print(f"\n>>> Start Experiment: {full_name}")
            live_plotter.start_session(exp_name, sd)
            
            ckpt_name = f"opt_tcn_lstm_mlp_{exp_name}_{sd}.pt"
            
            metrics = train_experiment(
                X_train, Y_train, X_val, Y_val, X_test, Y_test,
                cfg, seed=sd, save_path=ckpt_name, live_plotter=live_plotter
            )
            
            print(f"[RESULT] {full_name} | test_loss={metrics['test_huber']:.6f} | params={metrics['n_params']}")
            results.append((exp_name, sd, metrics, ckpt_name, metrics['config']))
            
    # Summary Plot
    summary_fig = plot_model_summary(results)
    
    # -------------------------------------------------------------------------
    # Feature Importance (Best Model)
    # -------------------------------------------------------------------------
    print("\n>>> Calculating Feature Importance for Best Model...")
    
    # 1. Find Best Model (lowest test_mae)
    best_res = sorted(results, key=lambda x: x[2]['test_mae'])[0]
    best_name, best_seed, best_metrics, best_ckpt, best_cfg = best_res
    print(f"Best Model: {best_name} (MAE={best_metrics['test_mae']:.6f})")
    
    # 2. Re-load Best Model
    # Re-instantiate
    best_model = TCN_LSTM_MLP(
        input_dim=X_test.shape[2],
        output_dim=Y_test.shape[2],
        horizon=Y_test.shape[1],
        channels=best_cfg.get("tcn_channels", (64, 64, 128)),
        kernel_size=best_cfg.get("kernel_size", 3),
        dropout=best_cfg.get("dropout_p", 0.1),
        hidden_size=best_cfg.get("lstm_hidden", 128),
        num_layers=best_cfg.get("lstm_layers", 1),
        mlp_hidden=best_cfg.get("mlp_hidden", 128),
        use_input_norm=best_cfg.get("use_input_norm", False),
        tcn_norm=best_cfg.get("tcn_norm", None),
        lstm_use_ln=best_cfg.get("lstm_use_ln", False),
        mlp_norm=best_cfg.get("mlp_norm", None)
    )
    
    # Load Weights
    ckpt = torch.load(best_ckpt, map_location='cpu') # Plotting on CPU is fine
    best_model.load_state_dict(ckpt['state_dict'])
    
    device = "cuda" if torch.cuda.is_available() else "cpu"
    best_model.to(device)
    best_model.eval()
    
    # 3. Construct Feature Names
    # Must match extract_condition_data_v2 logic
    feature_names = []
    
    # (A) Input Vars
    for gpath, vars in input_vars:
        # Custom Naming Logic
        if 'Back_imu' in gpath:
            prefix = "IMU"
        elif 'Robot' in gpath:
            prefix = "Robot"
        elif 'grf_measured' in gpath:
            # MoCap/grf_measured/left/force
            side = 'L' if 'left' in gpath else 'R'
            prefix = f"GRF_{side}"
        elif 'kin_qdot' in gpath:
            prefix = "J_Vel_Meas"
            pass 
        elif 'kin_q' in gpath:
            prefix = "J_Ang"
        else:
            prefix = gpath.split('/')[-1]
            
        for v in vars:
            # Clean var names
            v_clean = v.replace('ankle_angle', 'Ankle').replace('knee_angle', 'Knee').replace('hip_', 'Hip_')
            v_clean = v_clean.replace('flexion', 'Flex').replace('adduction', 'Add').replace('rotation', 'Rot')
            v_clean = v_clean.replace('_l', '_L').replace('_r', '_R')
            
            if 'grf' in gpath.lower():
                name = f"{prefix}_{v}"
            elif 'Back_imu' in gpath:
                v_clean = v.replace('Accel_', 'A').replace('Gyro_', 'G')
                name = f"{prefix}_{v_clean}"
            else:
                name = f"{prefix}_{v_clean}"
                
            feature_names.append(name)
            
    # (B) Augment: Joint Veolcity/Accel
    kin_q_vars = []
    for gpath, vars in input_vars:
        if 'kin_q' in gpath and 'dot' not in gpath:
             kin_q_vars = vars
             break
             
    for v in kin_q_vars:
        v_clean = v.replace('ankle_angle', 'Ankle').replace('knee_angle', 'Knee').replace('hip_', 'Hip_')
        v_clean = v_clean.replace('flexion', 'Flex').replace('adduction', 'Add').replace('rotation', 'Rot')
        v_clean = v_clean.replace('_l', '_L').replace('_r', '_R')
        feature_names.append(f"Calc_Vel_{v_clean}")
        
    for v in kin_q_vars:
        v_clean = v.replace('ankle_angle', 'Ankle').replace('knee_angle', 'Knee').replace('hip_', 'Hip_')
        v_clean = v_clean.replace('flexion', 'Flex').replace('adduction', 'Add').replace('rotation', 'Rot')
        v_clean = v_clean.replace('_l', '_L').replace('_r', '_R')
        feature_names.append(f"Calc_Acc_{v_clean}")
        
    # (C) Contact
    feature_names.append("Contact_L")
    feature_names.append("Contact_R")
    
    if len(feature_names) != X_test.shape[2]:
        print(f"[WARN] Feature name count ({len(feature_names)}) != Input dim ({X_test.shape[2]}). Using indices.")
        feature_names = [f"Feat_{i}" for i in range(X_test.shape[2])]
        
    # 4. Calculate Permutation Importance
    def calculate_permutation_importance(model, X, Y, feature_names, device='cpu'):
        # Baseline MAE
        test_ds = WindowDataset(X, Y, target_mode="sequence")
        loader = DataLoader(test_ds, batch_size=1024, shuffle=False)
        criterion = nn.L1Loss() # MAE
        
        model.eval()
        
        def get_loss():
            running = 0.0
            with torch.no_grad():
                for xb, yb in loader:
                    xb, yb = xb.to(device), yb.to(device)
                    pred = model(xb)
                    running += criterion(pred, yb).item() * xb.size(0)
            return running / len(test_ds)
            
        base_mae = get_loss()
        print(f"  Baseline MAE: {base_mae:.6f}")
        
        importances = []
        
        # Iterate features
        X_orig = X.copy()
        
        for i in range(X.shape[2]):
            # Shuffle column i
            saved_col = X_orig[:, :, i].copy()
            np.random.shuffle(X_orig[:, :, i]) # Shuffle along batch axis
            
            test_ds.X[:, :, i] = torch.from_numpy(X_orig[:, :, i])
            
            # Measure
            perm_mae = get_loss()
            imp = perm_mae - base_mae
            importances.append(imp)
            
            # Restore
            X_orig[:, :, i] = saved_col
            test_ds.X[:, :, i] = torch.from_numpy(saved_col)
            
            print(f"    Feat {i} ({feature_names[i]}): +{imp:.6f}", end='\r')
            
        print("")
        return np.array(importances)

    importances = calculate_permutation_importance(best_model, X_test, Y_test, feature_names, device)
    
    # 5. Plot All Features
    n_feats = len(importances)
    sorted_idx = np.argsort(importances)[::-1] # All sorted
    
    # Dynamic Figure Height: 0.25 inch per feature
    fig_h = max(8, n_feats * 0.25)
    plt.figure(figsize=(10, fig_h))
    
    plt.barh(range(n_feats), importances[sorted_idx], align='center')
    plt.yticks(range(n_feats), [feature_names[i] for i in sorted_idx])
    plt.xlabel('MAE Increase (Importance)')
    plt.title(f'Feature Importance (All Features) - {best_name}')
    plt.gca().invert_yaxis() # Top importance at top
    plt.tight_layout()
    
    plt.show(block=True)

