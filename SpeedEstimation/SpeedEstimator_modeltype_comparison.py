import os

# from SpeedEstimator_train_v4 import interp_nan_2d_linear
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

from matplotlib.widgets import Button
from scipy.signal import butter, filtfilt
from scipy.interpolate import CubicSpline

# 데이터셋 로드
data_path = '251030_combined_data.h5'

# subject, condition 정보
sub_names = ['S004','S005','S006','S007','S008','S009','S010','S011']
cond_names = [
    'accel_sine','asym_30deg','asym_60deg','cadence_120p','cadence_90p',
    'crouch','decline_5deg','incline_10deg','level_08mps','level_12mps','level_16mps'
]
data_length = 12000

def get_ground_contact(grf_fz, threshold=20.0):
    """[절차 1] GRF Fz 데이터를 이용한 접촉 판별 (0 또는 1)"""
    return (np.nan_to_num(grf_fz) > threshold).astype(float)

def centered_moving_average(x, win, pad_mode="reflect"):
    """
    Offline centered moving average (zero-phase 성격)
    x: (T,) 또는 (T, D)
    win: samples (권장: 홀수)
    """
    x = np.asarray(x, dtype=float)
    if win <= 1:
        return x

    # centered 정렬을 위해 홀수 권장
    if (win % 2) == 0:
        win += 1

    pad = win // 2
    kernel = np.ones(win, dtype=float) / win

    if x.ndim == 1:
        xp = np.pad(x, (pad, pad), mode=pad_mode)
        return np.convolve(xp, kernel, mode="valid")

    out = np.empty_like(x, dtype=float)
    for j in range(x.shape[1]):
        xp = np.pad(x[:, j], (pad, pad), mode=pad_mode)
        out[:, j] = np.convolve(xp, kernel, mode="valid")
    return out


def butter_lowpass_filter(data, cutoff, fs, order=4):
    """
    Zero-phase Butterworth Low-Pass Filter.
    data: (T,) or (T, D)
    cutoff: Cutoff frequency (Hz)
    fs: Sampling frequency (Hz)
    order: Filter order
    """
    from scipy.signal import butter, filtfilt
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    
    if data.ndim == 1:
        y = filtfilt(b, a, data)
    else:
        # data is (T, D), apply along time axis 0
        y = filtfilt(b, a, data, axis=0)
    return y


def plot_velocity_comparison_filter(raw_v, filtered_v, fs=100, lpf_cutoff=None):
    import numpy as np
    import matplotlib.pyplot as plt

    t = np.arange(raw_v.shape[0]) / fs

    fig, axes = plt.subplots(2, 1, figsize=(14, 6), sharex=True)

    # Y
    axes[0].plot(t, raw_v[:, 0], color='silver', label='Raw v_Y')
    axes[0].plot(t, filtered_v[:, 0], 'r', linewidth=1.5, label='Filtered v_Y')
    axes[0].set_ylabel('v_Y [m/s]')
    axes[0].grid(True)
    axes[0].legend()

    # Z
    axes[1].plot(t, raw_v[:, 1], color='silver', label='Raw v_Z')
    axes[1].plot(t, filtered_v[:, 1], 'r', linewidth=1.5, label='Filtered v_Z')
    axes[1].set_ylabel('v_Z [m/s]')
    axes[1].set_xlabel('Time [s]')
    axes[1].grid(True)
    axes[1].legend()

    title = "Speed reference filtering (Butterworth LPF)"
    if lpf_cutoff is not None:
        title += f" | cutoff={lpf_cutoff} Hz"
    fig.suptitle(title)

    plt.tight_layout()


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
        Y = torch.from_numpy(Y).float()   # Y: (N, Tw, D)

        if target_mode == "mean":
            self.Y = Y.mean(dim=1)        # (N, D)
        elif target_mode == "last":
            self.Y = Y[:, -1, :]          # (N, D)
        elif target_mode == "sequence":
            self.Y = Y                    # (N, Tw, D)
        else:
            raise ValueError("target_mode must be 'mean', 'last', or 'sequence'")

    def __len__(self):
        return self.X.shape[0]

    def __getitem__(self, idx):
        return self.X[idx], self.Y[idx]

class TCNEncoder(nn.Module):
    def __init__(self, input_dim, channels=(64, 64, 128), kernel_size=3, dropout=0.1):
        super().__init__()

        layers = []
        in_ch = input_dim
        for i, ch in enumerate(channels):
            dilation = 2 ** i
            layers += [
                nn.Conv1d(in_ch, ch, kernel_size, padding=0, dilation=dilation),
                nn.ReLU(),
                nn.Dropout(dropout),
            ]
            in_ch = ch

        self.network = nn.Sequential(*layers)
        self.out_ch = channels[-1]

    def forward(self, x):
        x = x.transpose(1, 2)  # (B, D, T)

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


class TCN_FC(nn.Module):
    def __init__(self, input_dim, output_dim, horizon, channels=(64,64,128), kernel_size=3, dropout=0.1):
        super().__init__()
        self.enc = TCNEncoder(input_dim, channels, kernel_size, dropout)
        self.horizon = horizon
        self.output_dim = output_dim
        self.head = nn.Linear(self.enc.out_ch, horizon * output_dim)

    def forward(self, x):
        z = self.enc(x)  # (B, C)
        y = self.head(z) # (B, H*D)
        return y.view(-1, self.horizon, self.output_dim)


class TCN_MLP(nn.Module):
    def __init__(self, input_dim, output_dim, horizon, channels=(64,64,128), kernel_size=3, dropout=0.1, mlp_hidden=256):
        super().__init__()
        self.enc = TCNEncoder(input_dim, channels, kernel_size, dropout)
        self.horizon = horizon
        self.output_dim = output_dim
        self.head = nn.Sequential(
            nn.Linear(self.enc.out_ch, mlp_hidden),
            nn.ReLU(),
            nn.Dropout(dropout),
            nn.Linear(mlp_hidden, horizon * output_dim)
        )

    def forward(self, x):
        z = self.enc(x)  # (B, C)
        y = self.head(z) # (B, H*D)
        return y.view(-1, self.horizon, self.output_dim)


class TCN_GRU_FC(nn.Module):
    def __init__(self, input_dim, output_dim, horizon, channels=(64,64,128), kernel_size=3, dropout=0.1, hidden_size=128, num_layers=1):
        super().__init__()
        self.enc = TCNEncoder(input_dim, channels, kernel_size, dropout)
        self.horizon = horizon
        self.output_dim = output_dim
        self.gru = nn.GRU(
            input_size=self.enc.out_ch, hidden_size=hidden_size,
            num_layers=num_layers, batch_first=True,
            dropout=dropout if num_layers > 1 else 0.0
        )
        self.fc = nn.Linear(hidden_size, output_dim)

    def forward(self, x):
        ctx = self.enc(x)  # (B, C)
        dec_in = ctx.unsqueeze(1).repeat(1, self.horizon, 1)  # (B, H, C)
        yseq, _ = self.gru(dec_in)  # (B, H, hidden)
        return self.fc(yseq)        # (B, H, D)


class TCN_GRU_MLP(nn.Module):
    def __init__(self, input_dim, output_dim, horizon, channels=(64,64,128), kernel_size=3, dropout=0.1, hidden_size=128, num_layers=1):
        super().__init__()
        self.enc = TCNEncoder(input_dim, channels, kernel_size, dropout)
        self.horizon = horizon
        self.output_dim = output_dim
        self.gru = nn.GRU(
            input_size=self.enc.out_ch, hidden_size=hidden_size,
            num_layers=num_layers, batch_first=True,
            dropout=dropout if num_layers > 1 else 0.0
        )
        # MLP Head: hidden_size -> hidden_size -> output_dim
        self.head = nn.Sequential(
            nn.Linear(hidden_size, hidden_size),
            nn.ReLU(),
            nn.Dropout(dropout),
            nn.Linear(hidden_size, output_dim)
        )

    def forward(self, x):
        ctx = self.enc(x)  # (B, C)
        dec_in = ctx.unsqueeze(1).repeat(1, self.horizon, 1)  # (B, H, C)
        yseq, _ = self.gru(dec_in)  # (B, H, hidden)
        return self.head(yseq)      # (B, H, D)


class TCN_LSTM_FC(nn.Module):
    def __init__(self, input_dim, output_dim, horizon, channels=(64,64,128), kernel_size=3, dropout=0.1, hidden_size=128, num_layers=1):
        super().__init__()
        self.enc = TCNEncoder(input_dim, channels, kernel_size, dropout)
        self.horizon = horizon
        self.output_dim = output_dim
        self.lstm = nn.LSTM(
            input_size=self.enc.out_ch, hidden_size=hidden_size,
            num_layers=num_layers, batch_first=True,
            dropout=dropout if num_layers > 1 else 0.0
        )
        self.fc = nn.Linear(hidden_size, output_dim)

    def forward(self, x):
        ctx = self.enc(x)  # (B, C)
        dec_in = ctx.unsqueeze(1).repeat(1, self.horizon, 1)  # (B, H, C)
        yseq, _ = self.lstm(dec_in)  # (B, H, hidden)
        return self.fc(yseq)         # (B, H, D)


class TCN_LSTM_MLP(nn.Module):
    def __init__(self, input_dim, output_dim, horizon, channels=(64,64,128), kernel_size=3, dropout=0.1, hidden_size=128, num_layers=1):
        super().__init__()
        self.enc = TCNEncoder(input_dim, channels, kernel_size, dropout)
        self.horizon = horizon
        self.output_dim = output_dim
        self.lstm = nn.LSTM(
            input_size=self.enc.out_ch, hidden_size=hidden_size,
            num_layers=num_layers, batch_first=True,
            dropout=dropout if num_layers > 1 else 0.0
        )
        self.head = nn.Sequential(
            nn.Linear(hidden_size, hidden_size),
            nn.ReLU(),
            nn.Dropout(dropout),
            nn.Linear(hidden_size, output_dim)
        )

    def forward(self, x):
        ctx = self.enc(x)  # (B, C)
        dec_in = ctx.unsqueeze(1).repeat(1, self.horizon, 1)  # (B, H, C)
        yseq, _ = self.lstm(dec_in)  # (B, H, hidden)
        return self.head(yseq)       # (B, H, D)


def train_speed_estimator(
    X_train, Y_train,
    X_val, Y_val,
    X_test, Y_test,
    model_type="TCN_MLP",
    target_mode="sequence",
    cfg=None,
    seed=42,
    save_path="speed_estimator_best.pt",
    live_plotter=None
):
    import time
    """
    cfg 예시:
    cfg = {
        "batch_size": 1024,
        "val_batch_size": 1024,
        "epochs": 50,
        "patience": 5,
        "dropout_p": 0.3,
        "attn_heads": 1,
        "lr": 3e-4,
        "weight_decay": 1e-2,
        "cos_T0": 10,
        "cos_Tmult": 1,
        "eta_min": 1e-5,
        "huber_delta": 0.5,
        "print_every": 50
    }
    """
    if cfg is None:
        cfg = {}

    batch_size = int(cfg.get("batch_size", 256))
    val_batch_size = int(cfg.get("val_batch_size", batch_size))
    epochs = int(cfg.get("epochs", cfg.get("max_epochs", 80)))
    patience = int(cfg.get("patience", 10))

    dropout_p = float(cfg.get("dropout_p", 0.1))
    lr = float(cfg.get("lr", 1e-3))
    weight_decay = float(cfg.get("weight_decay", 1e-4))

    cos_T0 = int(cfg.get("cos_T0", 0))
    cos_Tmult = int(cfg.get("cos_Tmult", 1))
    eta_min = float(cfg.get("eta_min", 0.0))

    huber_delta = float(cfg.get("huber_delta", 0.5))
    print_every = int(cfg.get("print_every", 50))

    torch.manual_seed(seed)
    np.random.seed(seed)
    random.seed(seed)

    train_ds = WindowDataset(X_train, Y_train, target_mode)
    val_ds   = WindowDataset(X_val,   Y_val,   target_mode)
    test_ds  = WindowDataset(X_test,  Y_test,  target_mode)

    train_loader = DataLoader(train_ds, batch_size=batch_size, shuffle=True)
    val_loader   = DataLoader(val_ds, batch_size=val_batch_size, shuffle=False)
    test_loader  = DataLoader(test_ds, batch_size=val_batch_size, shuffle=False)

    input_dim = X_train.shape[2]

    # target_mode에 따라 output shape 처리
    if target_mode == "sequence":
        horizon = train_ds.Y.shape[1]      # H
        output_dim = train_ds.Y.shape[2]   # D
    else:
        horizon = 1
        output_dim = train_ds.Y.shape[1]

    # backbone 공통 하이퍼 (필요시 cfg로 노출)
    if "tcn_channels" in cfg:
        tcn_channels = tuple(cfg["tcn_channels"])
    else:
        n_layers = int(cfg.get("tcn_layers", 3))
        n_hidden = int(cfg.get("tcn_hidden", 64))
        tcn_channels = tuple([n_hidden] * n_layers)
        
    kernel_size = int(cfg.get("kernel_size", 3))

    param_budget = cfg.get("param_budget", None)     # int 또는 None
    param_tol = float(cfg.get("param_tol", 0.03))    # 허용 오차 비율
    max_hidden = int(cfg.get("max_hidden", 1024))

    # ckpt 기록용(모든 분기에서 안전하게)
    mlp_hidden = None
    rnn_hidden = None
    rnn_layers = None

    def _count_params(m):
        return sum(p.numel() for p in m.parameters() if p.requires_grad)

    def _tune_hidden(build_fn, default_hidden):
        # budget이 없으면 cfg의 기본값 그대로 사용
        if param_budget is None:
            return default_hidden

        lo, hi = 8, max_hidden
        best_h = default_hidden
        best_diff = float("inf")

        # 단조 증가(대체로 hidden 커질수록 param 증가)를 가정하고 탐색
        for _ in range(20):
            mid = (lo + hi) // 2
            m = build_fn(mid)
            n = _count_params(m)
            diff = abs(n - param_budget) / max(1, param_budget)

            if diff < best_diff:
                best_diff = diff
                best_h = mid

            if diff <= param_tol:
                return mid

            if n < param_budget:
                lo = mid + 1
            else:
                hi = mid - 1

            if lo > hi:
                break

        return best_h

    if model_type == "TCN_FC":
        model = TCN_FC(input_dim, output_dim, horizon,
                       channels=tcn_channels, kernel_size=kernel_size,
                       dropout=dropout_p)
        model_config = {}

    elif model_type == "TCN_MLP":
        mlp_hidden0 = int(cfg.get("mlp_hidden", 256))
        mlp_hidden = _tune_hidden(
            lambda h: TCN_MLP(input_dim, output_dim, horizon,
                            channels=tcn_channels, kernel_size=kernel_size,
                            dropout=dropout_p, mlp_hidden=h),
            mlp_hidden0
        )
        model = TCN_MLP(input_dim, output_dim, horizon,
                        channels=tcn_channels, kernel_size=kernel_size,
                        dropout=dropout_p, mlp_hidden=mlp_hidden)
        model_config = {"mlp_hidden": mlp_hidden}

    elif model_type == "TCN_GRU_FC":
        hidden0 = int(cfg.get("rnn_hidden", 128))
        num_layers  = int(cfg.get("rnn_layers", 1))
        hidden_size = _tune_hidden(
            lambda h: TCN_GRU_FC(input_dim, output_dim, horizon,
                                    channels=tcn_channels, kernel_size=kernel_size,
                                    dropout=dropout_p, hidden_size=h, num_layers=num_layers),
            hidden0
        )
        model = TCN_GRU_FC(input_dim, output_dim, horizon,
                            channels=tcn_channels, kernel_size=kernel_size,
                            dropout=dropout_p, hidden_size=hidden_size, num_layers=num_layers)
        rnn_hidden = hidden_size
        rnn_layers = num_layers
        model_config = {"hidden_size": hidden_size, "num_layers": num_layers}

    elif model_type == "TCN_GRU_MLP":
        hidden0 = int(cfg.get("rnn_hidden", 128))
        num_layers  = int(cfg.get("rnn_layers", 1))
        hidden_size = _tune_hidden(
            lambda h: TCN_GRU_MLP(input_dim, output_dim, horizon,
                                    channels=tcn_channels, kernel_size=kernel_size,
                                    dropout=dropout_p, hidden_size=h, num_layers=num_layers),
            hidden0
        )
        model = TCN_GRU_MLP(input_dim, output_dim, horizon,
                            channels=tcn_channels, kernel_size=kernel_size,
                            dropout=dropout_p, hidden_size=hidden_size, num_layers=num_layers)
        rnn_hidden = hidden_size
        rnn_layers = num_layers
        model_config = {"hidden_size": hidden_size, "num_layers": num_layers}

    elif model_type == "TCN_LSTM_FC":
        hidden0 = int(cfg.get("rnn_hidden", 128))
        num_layers  = int(cfg.get("rnn_layers", 1))
        hidden_size = _tune_hidden(
            lambda h: TCN_LSTM_FC(input_dim, output_dim, horizon,
                                    channels=tcn_channels, kernel_size=kernel_size,
                                    dropout=dropout_p, hidden_size=h, num_layers=num_layers),
            hidden0
        )
        model = TCN_LSTM_FC(input_dim, output_dim, horizon,
                                channels=tcn_channels, kernel_size=kernel_size,
                                dropout=dropout_p, hidden_size=hidden_size, num_layers=num_layers)
        rnn_hidden = hidden_size
        rnn_layers = num_layers
        model_config = {"hidden_size": hidden_size, "num_layers": num_layers}

    elif model_type == "TCN_LSTM_MLP":
        hidden0 = int(cfg.get("rnn_hidden", 128))
        num_layers  = int(cfg.get("rnn_layers", 1))
        hidden_size = _tune_hidden(
            lambda h: TCN_LSTM_MLP(input_dim, output_dim, horizon,
                                    channels=tcn_channels, kernel_size=kernel_size,
                                    dropout=dropout_p, hidden_size=h, num_layers=num_layers),
            hidden0
        )
        model = TCN_LSTM_MLP(input_dim, output_dim, horizon,
                                channels=tcn_channels, kernel_size=kernel_size,
                                dropout=dropout_p, hidden_size=hidden_size, num_layers=num_layers)
        rnn_hidden = hidden_size
        rnn_layers = num_layers
        model_config = {"hidden_size": hidden_size, "num_layers": num_layers}

    else:
        raise ValueError(f"Unknown model_type: {model_type}")

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

            if (bi % print_every) == 0 or (bi == len(train_loader)):
                avg = running / max(1, n_batches)
                cur_lr = optimizer.param_groups[0]["lr"]
                print(f"[Epoch {epoch:03d}/{epochs:03d}] "
                      f"batch {bi:05d}/{len(train_loader):05d} "
                      f"train_loss={avg:.6f} lr={cur_lr:.2e}")

        train_loss = running / max(1, n_batches)

        model.eval()
        val_running = 0.0
        with torch.no_grad():
            for xb, yb in val_loader:
                xb, yb = xb.to(device), yb.to(device)
                val_running += criterion(model(xb), yb).item()
        val_loss = val_running / max(1, len(val_loader))

        if scheduler is not None:
            scheduler.step(epoch - 1)

        dt = time.time() - t0
        cur_lr = optimizer.param_groups[0]["lr"]
        print(f"[Epoch {epoch:03d}/{epochs:03d}] "
              f"train_loss={train_loss:.6f} val_loss={val_loss:.6f} "
              f"lr={cur_lr:.2e} time={dt:.1f}s")
              
        if live_plotter is not None:
            live_plotter.update_epoch(epoch, train_loss, val_loss)

        if val_loss < best_val:
            best_val = val_loss
            patience_cnt = 0
            ckpt_obj = {
                "state_dict": model.state_dict(),
                "model_type": model_type,
                "target_mode": target_mode,
                "cfg_used": {
                    "tcn_channels": tuple(tcn_channels),
                    "kernel_size": int(kernel_size),
                    "dropout_p": float(dropout_p),
                    "rnn_hidden": int(rnn_hidden) if rnn_hidden is not None else None,
                    "mlp_hidden": int(mlp_hidden) if mlp_hidden is not None else None,
                    "rnn_layers": int(rnn_layers) if rnn_layers is not None else None,
                }
            }
            torch.save(ckpt_obj, save_path)
            print(f"  saved: {save_path} (best val={best_val:.6f})")

        else:
            patience_cnt += 1
            print(f"  no improve ({patience_cnt}/{patience})")
            if patience_cnt >= patience:
                print("  early stopping")
                break

    obj = torch.load(save_path, map_location=device, weights_only=False)
    state = obj["state_dict"] if isinstance(obj, dict) and "state_dict" in obj else obj
    model.load_state_dict(state)
    model.eval()

    test_running = 0.0
    with torch.no_grad():
        for xb, yb in test_loader:
            xb, yb = xb.to(device), yb.to(device)
            test_running += criterion(model(xb), yb).item()
    test_loss = test_running / max(1, len(test_loader))

    # step별 MAE/RMSE 계산 (target_mode=="sequence"일 때만)
    step_mae, step_rmse = None, None

    n_params = sum(p.numel() for p in model.parameters() if p.requires_grad)

    if target_mode == "sequence":
        preds_all = []
        trues_all = []
        with torch.no_grad():
            for xb, yb in test_loader:
                xb = xb.to(device)
                pred = model(xb).cpu().numpy()   # (B,H,D)
                preds_all.append(pred)
                trues_all.append(yb.numpy())     # (B,H,D)

        P = np.concatenate(preds_all, axis=0)    # (N,H,D)
        T = np.concatenate(trues_all, axis=0)    # (N,H,D)

        err = P - T
        step_mae = np.mean(np.abs(err), axis=0)                 # (H,D)
        step_rmse = np.sqrt(np.mean(err**2, axis=0) + 1e-12)    # (H,D)

    metrics = {
        "test_huber": float(test_loss),
        "step_mae": step_mae,
        "step_rmse": step_rmse,
        "n_params": int(n_params),
        "model_config": model_config
    }
    return metrics

def plot_pred_true_all_conditions(
    data_path,
    sub_names,
    cond_names,
    input_vars,
    output_vars,
    time_window_input,
    time_window_output,
    stride,
    test_subjects,
    model_type="tcn",
    target_mode="mean",
    ckpt_path="speed_estimator_best.pt",
    batch_size=512,
    fs=100,
    window_seconds=20.0,
    max_points=5000,
    model_cfg=None,
    lpf_cutoff=None,
    lpf_order=4
):
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    # 모델을 만들기 위한 dim 확보: 첫 cond로 한번 뽑아서 input_dim/output_dim 결정
    first_cond = cond_names[0]
    X0, Y0 = build_nn_dataset(
        data_path, sub_names, cond_names,
        input_vars, output_vars,
        time_window_input, time_window_output, stride,
        subject_selection={'include': test_subjects, 'exclude': []},
        condition_selection={'include': [first_cond], 'exclude': []}, lpf_cutoff=lpf_cutoff, lpf_order=lpf_order
    )
    if X0 is None or Y0 is None or X0.size == 0:
        print("[WARN] No samples for first condition. Check test_subjects/cond_names.")
        return

    ds0 = WindowDataset(X0, Y0, target_mode=target_mode)
    input_dim = X0.shape[2]

    if target_mode == "sequence":
        horizon = ds0.Y.shape[1]      # H
        output_dim = ds0.Y.shape[2]   # D
    else:
        horizon = 1
        output_dim = ds0.Y.shape[1]

    model_cfg = model_cfg or {}

    obj = torch.load(ckpt_path, map_location="cpu")
    if isinstance(obj, dict) and "cfg_used" in obj:
        cu = obj["cfg_used"]
        model_cfg = {**model_cfg, **{k: v for k, v in cu.items() if v is not None}}

    tcn_channels = tuple(model_cfg.get("tcn_channels", (64, 64, 128)))
    kernel_size  = int(model_cfg.get("kernel_size", 3))
    dropout_p    = float(model_cfg.get("dropout_p", 0.1))

    if model_type == "TCN_FC":
        model = TCN_FC(input_dim, output_dim, horizon, channels=tcn_channels, kernel_size=kernel_size, dropout=dropout_p)
    elif model_type == "TCN_MLP":
        model = TCN_MLP(input_dim, output_dim, horizon, channels=tcn_channels, kernel_size=kernel_size, dropout=dropout_p, mlp_hidden=int(model_cfg.get("mlp_hidden", 256)))
    elif model_type == "TCN_GRU_FC":
        model = TCN_GRU_FC(input_dim, output_dim, horizon, channels=tcn_channels, kernel_size=kernel_size, dropout=dropout_p, hidden_size=int(model_cfg.get("rnn_hidden", 128)), num_layers=int(model_cfg.get("rnn_layers", 1)))
    elif model_type == "TCN_GRU_MLP":
        model = TCN_GRU_MLP(input_dim, output_dim, horizon, channels=tcn_channels, kernel_size=kernel_size, dropout=dropout_p, hidden_size=int(model_cfg.get("rnn_hidden", 128)), num_layers=int(model_cfg.get("rnn_layers", 1)))
    elif model_type == "TCN_LSTM_FC":
        model = TCN_LSTM_FC(input_dim, output_dim, horizon, channels=tcn_channels, kernel_size=kernel_size, dropout=dropout_p, hidden_size=int(model_cfg.get("rnn_hidden", 128)), num_layers=int(model_cfg.get("rnn_layers", 1)))
    elif model_type == "TCN_LSTM_MLP":
        model = TCN_LSTM_MLP(input_dim, output_dim, horizon, channels=tcn_channels, kernel_size=kernel_size, dropout=dropout_p, hidden_size=int(model_cfg.get("rnn_hidden", 128)), num_layers=int(model_cfg.get("rnn_layers", 1)))
    else:
        raise ValueError(f"unsupported model_type: {model_type}")

    obj = torch.load(ckpt_path, map_location=device)
    state = obj["state_dict"] if isinstance(obj, dict) and "state_dict" in obj else obj
    model.load_state_dict(state)
    model.to(device)
    model.eval()

    # output label names for plot
    out_names = []
    for _, var_list in (output_vars or []):
        out_names.extend(var_list)

    for cond in cond_names:
        Xc, Yc = build_nn_dataset(
            data_path, sub_names, cond_names,
            input_vars, output_vars,
            time_window_input, time_window_output, stride,
            subject_selection={'include': test_subjects, 'exclude': []},



            condition_selection={'include': [cond], 'exclude': []}, lpf_cutoff=lpf_cutoff, lpf_order=lpf_order
        )

        if Xc is None or Yc is None or Xc.size == 0: 
            print(f"[INFO] Skip {cond}: no samples.")
            continue

        dsc = WindowDataset(Xc, Yc, target_mode=target_mode)
        loader = DataLoader(dsc, batch_size=batch_size, shuffle=False)

        preds, trues = [], []
        with torch.no_grad():
            for xb, yb in loader:
                xb = xb.to(device)
                pred = model(xb).cpu().numpy()
                preds.append(pred)
                trues.append(yb.numpy())

        pred_all = np.concatenate(preds, axis=0)
        true_all = np.concatenate(trues, axis=0)

        n = pred_all.shape[0]

        # 윈도우 인덱스 1개가 stride 샘플 만큼 이동한 것과 유사하므로,
        # 시간축을 더 맞추려면 stride를 반영하는 게 좋습니다.
        dt = stride / fs

        # 20초에 해당하는 윈도우 개수
        n_show = int(window_seconds / dt)
        n_show = min(n, n_show, max_points)
        t = np.arange(n_show) * dt

        if target_mode == "sequence":
            k = -1  # 마지막 tick (10 tick ahead)
            pred_all = pred_all[:, k, :]  # (N, D)
            true_all = true_all[:, k, :]  # (N, D)

        for j in range(output_dim):
            name = out_names[j] if j < len(out_names) else f"out{j}"
            fig = plt.figure(figsize=(14, 5))
            ax = plt.gca()

            ax.plot(t, true_all[:n_show, j], label="true")
            ax.plot(t, pred_all[:n_show, j], label="pred")
            ax.set_title(f"{cond} | {name} | test={test_subjects} | model={model_type} | target={target_mode}")
            ax.set_xlabel("Time [s]")
            ax.set_ylabel(name)
            ax.grid(True)
            ax.legend()

            # 스크롤 확대/축소 연결
            attach_zoom_factory(fig)

            # ---- 버튼: x축 토글 (20-25초 <-> 전체) ----
            # 버튼 영역(figure 좌표): [left, bottom, width, height]
            btn_ax = fig.add_axes([0.75, 0.02, 0.22, 0.06])
            btn = Button(btn_ax, "Toggle 20-25s")

            # 전체 범위 저장 (현재 플롯의 x 데이터 기준)
            full_xlim = ax.get_xlim()
            zoom_xlim = (20.0, 25.0)

            # fig에 상태 저장
            fig._toggle_zoom_state = False
            fig._toggle_full_xlim = full_xlim

            def _on_toggle(_event):
                # 토글 상태 반전
                fig._toggle_zoom_state = not fig._toggle_zoom_state

                if fig._toggle_zoom_state:
                    ax.set_xlim(zoom_xlim)
                else:
                    ax.set_xlim(fig._toggle_full_xlim)

                fig.canvas.draw_idle()

            btn.on_clicked(_on_toggle)

            # 버튼 객체가 GC로 사라지지 않게 참조 유지
            fig._toggle_btn = btn


    plt.show()

def build_nn_dataset(h5_path, sub_names, cond_names, input_vars, output_vars, input_window, output_window, stride=1,
    subject_selection=None, condition_selection=None, use_gating=False, alpha=0.5, debug_plot=False, lpf_cutoff=None, lpf_order=4):
    """
    기존의 모든 데이터 로드 및 전처리 기능을 유지하면서 다음 기능을 추가함:
    1) use_gating: True일 경우 입력 피처 끝에 좌/우 접촉 정보(0,1) 추가
    2) Adaptive LPF: 보행 주기에 연동된 1차 Butterworth 양방향 필터로 저주파 의도 속도 추출
    3) Debug Plot: accel_sine 조건에 대해 필터링 전후 시각화
    """
    X_list, Y_list = [], []
    fs = 100  # 데이터 샘플링 주파수 고정

    with h5py.File(h5_path, 'r') as f:
        # vel_debug_cache = {}   # key=sub, val=(raw_v, filtered_v)
        # -> 변경: condition별로 별도 cache 관리
        target_conds_for_plot = ['accel_sine', 'asym_60deg']
        vel_debug_caches = {c: {} for c in target_conds_for_plot}
        # ---- [기존 유지] subject selection ----
        if subject_selection is not None:
            eff_sub_names = sub_names.copy()
            if subject_selection.get('include') is not None:
                eff_sub_names = [
                    s for s in eff_sub_names
                    if s in subject_selection['include']
                ]
            eff_sub_names = [
                s for s in eff_sub_names
                if s not in subject_selection.get('exclude', [])
            ]
        else:
            eff_sub_names = sub_names

        # ---- [기존 유지] condition selection ----
        if condition_selection is not None:
            eff_cond_names = cond_names.copy()
            if condition_selection.get('include') is not None:
                eff_cond_names = [
                    c for c in eff_cond_names
                    if c in condition_selection['include']
                ]
            eff_cond_names = [
                c for c in eff_cond_names
                if c not in condition_selection.get('exclude', [])
            ]
        else:
            eff_cond_names = cond_names

        for sub in eff_sub_names:
            for cond in eff_cond_names:
                trial_path = f"{sub}/{cond}/trial_01"
                if trial_path not in f:
                    continue
                g = f[trial_path]

                # --- [추가 1] GRF 및 Contact 정보 추출 (Gating용) ---
                l_grf = g['MoCap/grf_measured/left/force/Fz'][:]
                r_grf = g['MoCap/grf_measured/right/force/Fz'][:]
                l_contact = get_ground_contact(l_grf)[:, None]
                r_contact = get_ground_contact(r_grf)[:, None]
                
                # --- [추가 2] 보행 주기 기반 적응형 필터링 (Reference 수정) ---
                # 기존 v_Y_true, v_Z_true 로드 (저주파 성분 추출용)
                raw_vy = g['Common/v_Y_true'][:][:, None]
                raw_vz = g['Common/v_Z_true'][:][:, None]
                raw_v = np.concatenate([raw_vy, raw_vz], axis=1)

                filtered_v = raw_v
                if lpf_cutoff is not None and lpf_cutoff > 0:
                    # print(f"Applying LPF: cutoff={lpf_cutoff}, order={lpf_order} for {sub}/{cond}")
                    filtered_v = butter_lowpass_filter(raw_v, lpf_cutoff, fs, order=lpf_order)

                # --- [추가 3] 디버깅 시각화 데이터 수집: subject당 1회만 저장 (표시는 나중에 일괄) ---
                if debug_plot and (cond in target_conds_for_plot):
                    c_cache = vel_debug_caches[cond]
                    if sub not in c_cache:
                        c_cache[sub] = (raw_v.copy(), filtered_v.copy())

                # ---- [기존 유지] Build input features ----
                input_feat = []
                for group_path, var_list in (input_vars or []):
                    grp = g
                    ok = True
                    for p in group_path.split('/'):
                        if not p: continue
                        if p not in grp:
                            ok = False
                            break
                        grp = grp[p]
                    if not ok: continue

                    for v in var_list:
                        if v not in grp: continue
                        arr = grp[v][:]
                        if arr.ndim == 1:
                            arr = arr[:, None]
                        input_feat.append(arr)

                # [추가 4] Gating 옵션: 입력 피처 끝에 접촉 정보 추가
                if use_gating:
                    input_feat.append(l_contact)
                    input_feat.append(r_contact)

                # ---- [기존 유지 및 수정] Build output features ----
                output_feat = []
                # output_vars 리스트를 순회하며 속도 데이터만 필터링된 버전으로 교체
                for group_path, var_list in (output_vars or []):
                    # v_Y_true 또는 v_Z_true가 포함된 경우 필터링된 filtered_v를 사용
                    if 'v_Y_true' in var_list or 'v_Z_true' in var_list:
                        # filtered_v는 [v_Y, v_Z]로 구성됨 (요청된 변수 순서에 따라 처리)
                        output_feat.append(filtered_v)
                    else:
                        # 그 외의 변수는 기존 로직대로 로드
                        grp = g
                        ok = True
                        for p in group_path.split('/'):
                            if not p: continue
                            if p not in grp:
                                ok = False
                                break
                            grp = grp[p]
                        if not ok: continue
                        for v in var_list:
                            if v not in grp: continue
                            arr = grp[v][:]
                            if arr.ndim == 1:
                                arr = arr[:, None]
                            output_feat.append(arr)

                if len(input_feat) == 0 or len(output_feat) == 0:
                    continue
                
                input_mat = np.concatenate(input_feat, axis=1)
                output_mat = np.concatenate(output_feat, axis=1)

                # ---- [기존 유지] NaN 보간 처리 ----
                trial_tag = f"{sub}/{cond}/trial_01"
                # verbose_nan 옵션을 지원하기 위해 condition_selection 확인
                verbose_nan = bool(condition_selection.get("verbose_nan", False)) if condition_selection else False
                # 기존 interp_nan_2d_linear 또는 _local 함수 사용 (파일명에 맞춰 호출)
                input_mat = interp_nan_2d_linear_local(input_mat, fill_all_nan_with=0.0, verbose=verbose_nan, tag=trial_tag + " input")
                output_mat = interp_nan_2d_linear_local(output_mat, fill_all_nan_with=0.0, verbose=verbose_nan, tag=trial_tag + " output")

                # ---- [기존 유지] Sliding Window 생성 ----
                n = len(input_mat)
                for i in range(0, n - input_window - output_window + 1, stride):
                    X = input_mat[i:i+input_window]
                    Y = output_mat[i+input_window:i+input_window+output_window]
                    X_list.append(X)
                    Y_list.append(Y)

    # 시각화 (조건별로 그림 그리기)
    # 시각화 (조건별로 그림 그리기)
    if debug_plot:
        all_figs = []
        for c_name, c_cache in vel_debug_caches.items():
            if len(c_cache) > 0:
                figs = plot_velocity_4subjects_per_fig(
                    c_cache,
                    fs=fs,
                    lpf_cutoff=lpf_cutoff,
                    subjects_per_fig=4,
                    title_prefix=f"Training target speed ({c_name}, all subjects)"
                )
                if figs:
                    all_figs.extend(figs)

        # 렌더링 강제(빈 창 방지)
        # 렌더링 강제(빈 창 방지)
        for fig in all_figs:
            try:
                fig.canvas.draw()
                fig.canvas.flush_events()
            except Exception:
                pass

        # 여기서 멈춤: 열린 figure들을 모두 닫아야 return되어 training 진행
        plt.show(block=True)
        plt.close('all')


    if len(X_list) == 0:
        return np.empty((0, input_window, 0), dtype=np.float32), np.empty((0, output_window, 0), dtype=np.float32)

    X_arr = np.stack(X_list).astype(np.float32)
    Y_arr = np.stack(Y_list).astype(np.float32)
    return X_arr, Y_arr

def load_subject_fsr_imu_kin(data_path, sub_names, cond_names, data_length):
    Sub = {}
    with h5py.File(data_path, 'r') as f:
        for sub_idx, sub_name in enumerate(sub_names, start=1):
            Sub[sub_idx] = {}

            # sub_info: mass, height를 여기서 한 번만 읽어서 저장
            try:
                Sub[sub_idx]['mass_kg'] = f[f"{sub_name}/sub_info/mass_kg"][()]
            except Exception:
                Sub[sub_idx]['mass_kg'] = 'N/A'
            try:
                Sub[sub_idx]['height_m'] = f[f"{sub_name}/sub_info/hight_m"][()]
            except Exception:
                Sub[sub_idx]['height_m'] = 'N/A'
            # FSR1
            for side, prefix in [('Left','L'), ('Right','R')]:
                key = f'fsr{prefix}1'
                arr = np.empty((data_length, len(cond_names)), dtype=np.float32)
                for c, cond in enumerate(cond_names):
                    d = f[f"{sub_name}/{cond}/trial_01/Insole/{side}/fsr{prefix}1"]
                    n = min(len(d), data_length)
                    arr[:n, c] = d[:n].astype(np.float32)
                    if n < data_length:
                        arr[n:, c] = np.nan
                Sub[sub_idx][key] = arr
            # Back IMU
            for out_key, h5key in [('BackIMUAccX','Accel_X'), ('BackIMUAccY','Accel_Y'), ('BackIMUAccZ','Accel_Z'),
                                   ('BackIMUGyrX','Gyro_X'),  ('BackIMUGyrY','Gyro_Y'),  ('BackIMUGyrZ','Gyro_Z')]:
                arr = np.empty((data_length, len(cond_names)), dtype=np.float32)
                for c, cond in enumerate(cond_names):
                    d = f[f"{sub_name}/{cond}/trial_01/Back_imu/{h5key}"]
                    n = min(len(d), data_length)
                    arr[:n, c] = d[:n].astype(np.float32)
                    if n < data_length:
                        arr[n:, c] = np.nan
                Sub[sub_idx][out_key] = arr
            # Common v_Y_true, v_Z_true
            for out_key, h5key in [('v_Y_true','v_Y_true'), ('v_Z_true','v_Z_true')]:
                arr = np.empty((data_length, len(cond_names)), dtype=np.float32)
                for c, cond in enumerate(cond_names):
                    try:
                        d = f[f"{sub_name}/{cond}/trial_01/Common/{h5key}"]
                        n = min(len(d), data_length)
                        arr[:n, c] = d[:n].astype(np.float32)
                        if n < data_length:
                            arr[n:, c] = np.nan
                    except Exception:
                        arr[:, c] = np.nan
                Sub[sub_idx][out_key] = arr
            # GRF Fz
            for out_key, lr, comp in [('LGRFz','left','Fz'), ('RGRFz','right','Fz')]:
                arr = np.empty((data_length, len(cond_names)), dtype=np.float32)
                for c, cond in enumerate(cond_names):
                    try:
                        d = f[f"{sub_name}/{cond}/trial_01/MoCap/grf_measured/{lr}/force/{comp}"]
                        n = min(len(d), data_length)
                        arr[:n, c] = d[:n].astype(np.float32)
                        if n < data_length:
                            arr[n:, c] = np.nan
                    except Exception:
                        arr[:, c] = np.nan
                Sub[sub_idx][out_key] = arr
            # Robot Encoder
            for out_key, h5key in [('currentLH','currentLH'), ('currentRH','currentRH'), ('incPosLH','incPosLH'), ('incPosRH','incPosRH')]:
                arr = np.empty((data_length, len(cond_names)), dtype=np.float32)
                for c, cond in enumerate(cond_names):
                    try:
                        d = f[f"{sub_name}/{cond}/trial_01/Robot/{h5key}"]
                        n = min(len(d), data_length)
                        arr[:n, c] = d[:n].astype(np.float32)
                        if n < data_length:
                            arr[n:, c] = np.nan
                    except Exception:
                        arr[:, c] = np.nan
                Sub[sub_idx][out_key] = arr
            # Kinematics (각도)
            kin_map = [
                ('LhipFE','hip_flexion_l'), ('LhipAA','hip_adduction_l'), ('LhipRot','hip_rotation_l'),
                ('RhipFE','hip_flexion_r'), ('RhipAA','hip_adduction_r'), ('RhipRot','hip_rotation_r'),
                ('LkneeFE','knee_angle_l'), ('RkneeFE','knee_angle_r'),
                ('LankleFE','ankle_angle_l'), ('RankleFE','ankle_angle_r')
            ]
            for out_key, h5key in kin_map:
                arr = np.empty((data_length, len(cond_names)), dtype=np.float32)
                for c, cond in enumerate(cond_names):
                    try:
                        d = f[f"{sub_name}/{cond}/trial_01/MoCap/kin_q/{h5key}"]
                        n = min(len(d), data_length)
                        arr[:n, c] = d[:n].astype(np.float32)
                        if n < data_length:
                            arr[n:, c] = np.nan
                    except Exception:
                        arr[:, c] = np.nan
                Sub[sub_idx][out_key] = arr
    return Sub

def visualize_speed(raw_v, filtered_v, fs=100, lpf_cutoff=None, title="Velocity filtering (training target)", show=True):
    import numpy as np
    import matplotlib.pyplot as plt

    T = raw_v.shape[0]
    t = np.arange(T) / fs

    fig, axes = plt.subplots(2, 1, figsize=(14, 8), sharex=True)
    fig.suptitle(title, fontsize=16)

    # v_Y
    axes[0].plot(t, raw_v[:, 0], color='silver', label='Raw v_Y', alpha=0.7)
    axes[0].plot(t, filtered_v[:, 0], 'r', linewidth=2, label='Filtered v_Y')
    axes[0].set_ylabel('Velocity [m/s]')
    axes[0].legend()
    axes[0].grid(True)

    # v_Z
    axes[1].plot(t, raw_v[:, 1], color='silver', label='Raw v_Z', alpha=0.7)
    axes[1].plot(t, filtered_v[:, 1], 'r', linewidth=2, label='Filtered v_Z')
    axes[1].set_ylabel('Velocity [m/s]')
    axes[1].set_xlabel('Time [s]')
    axes[1].legend()
    axes[1].grid(True)

    if lpf_cutoff is not None:
        fig.text(0.02, 0.01, f"Butterworth LPF cutoff = {lpf_cutoff} Hz")

    plt.tight_layout()
    attach_zoom_factory(fig)

    # ---- 버튼: x축 토글 (20-25초 <-> 전체) ----
    # 버튼 영역(figure 좌표): [left, bottom, width, height]
    # visualize_speed는 subplot이 꽉 차 있을 수 있으므로 위치 조정 필요
    # tight_layout 후에 axes 위치가 잡히므로, 별도의 axes를 추가
    
    # 2행 1열 구조이므로 하단 여백이 좁을 수 있음. tight_layout(rect=[0, 0.08, 1, 1]) 처럼 아래 여백 확보 후 추가 권장
    # 여기서는 기존 코드 구조상 레이아웃 조정 없이 간단히 우측 하단이나 별도 위치에 추가
    btn_ax = fig.add_axes([0.75, 0.01, 0.22, 0.05])
    btn = Button(btn_ax, "Toggle 20-25s")

    # 전체 범위 저장 (현재 플롯의 x 데이터 기준)
    # axes[1]이 Time 축을 공유하므로 이를 기준으로 함
    full_xlim = axes[1].get_xlim()
    zoom_xlim = (20.0, 25.0)

    # fig에 상태 저장
    fig._toggle_zoom_state = False
    fig._toggle_full_xlim = full_xlim

    def _on_toggle(_event):
        fig._toggle_zoom_state = not fig._toggle_zoom_state
        if fig._toggle_zoom_state:
            # 줌하기 전에 현재 full 범위를 업데이트(혹시 사용자가 팬/줌 했을 수 있으므로)하고 싶다면 여기서 get_xlim()을 다시 해도 됨
            # 여기서는 편의상 처음 렌더링 시점의 범위를 full로 사용
            for ax in axes:
                ax.set_xlim(zoom_xlim)
        else:
            for ax in axes:
                ax.set_xlim(fig._toggle_full_xlim)
        fig.canvas.draw_idle()

    btn.on_clicked(_on_toggle)
    fig._toggle_btn = btn  # GC 방지

    if show:
        import matplotlib.pyplot as plt
        plt.show(block=False)
        plt.pause(0.01)

    return fig

def plot_velocity_4subjects_per_fig(vel_debug_cache, fs=100, lpf_cutoff=None, subjects_per_fig=4, title_prefix=""):
    """
    vel_debug_cache: dict[sub] = (raw_v(T,2), filtered_v(T,2))
    한 figure에 4명(subjects_per_fig)씩 묶어 표시.
    레이아웃: rows=subjects_per_fig, cols=2 (왼쪽 v_Y, 오른쪽 v_Z)
    각 subplot에는 raw/filtered를 함께 plot.
    """
    import numpy as np
    import matplotlib.pyplot as plt

    subs = list(vel_debug_cache.keys())
    figs = []

    # subject 4명씩 배치
    for start in range(0, len(subs), subjects_per_fig):
        batch = subs[start:start + subjects_per_fig]

        fig, axes = plt.subplots(len(batch), 2, figsize=(18, 3.8 * len(batch)), sharex=False)
        if len(batch) == 1:
            axes = np.array([axes])  # (1,2) 형태로 맞춤

        for r, sub in enumerate(batch):
            raw_v, filtered_v = vel_debug_cache[sub]
            T = raw_v.shape[0]
            t = np.arange(T) / fs

            # v_Y (col 0)
            ax0 = axes[r, 0]
            ax0.plot(t, raw_v[:, 0], color="silver", label="Raw v_Y", alpha=0.7)
            ax0.plot(t, filtered_v[:, 0], "r", linewidth=2, label="Filtered v_Y")
            ax0.set_ylabel(f"{sub}\nVelocity [m/s]")
            ax0.grid(True)
            if r == 0:
                ax0.set_title("v_Y")
            ax0.legend(loc="upper right")

            # v_Z (col 1)
            ax1 = axes[r, 1]
            ax1.plot(t, raw_v[:, 1], color="silver", label="Raw v_Z", alpha=0.7)
            ax1.plot(t, filtered_v[:, 1], "r", linewidth=2, label="Filtered v_Z")
            ax1.grid(True)
            if r == 0:
                ax1.set_title("v_Z")
            ax1.legend(loc="upper right")

            if r == len(batch) - 1:
                ax0.set_xlabel("Time [s]")
                ax1.set_xlabel("Time [s]")

        title = title_prefix
        if lpf_cutoff is not None:
            if title:
                title += " | "
            title += f"Butterworth LPF cutoff = {lpf_cutoff} Hz"
        fig.suptitle(title, fontsize=14)

        plt.tight_layout(rect=[0, 0, 1, 0.96])
        
        # 하단 여백 확보를 위해 subplots_adjust 등을 쓸 수도 있지만, tight_layout rect로 상단만 비웠음.
        # 하단에 버튼을 넣으려면 rect=[0, 0.05, 1, 0.96] 정도로 하는게 좋음.
        # 일단은 겹치더라도 추가.
        plt.subplots_adjust(bottom=0.08)
        
        attach_zoom_factory(fig)

        # ---- 버튼 추가 ----
        btn_ax = fig.add_axes([0.75, 0.01, 0.22, 0.05])
        btn = Button(btn_ax, "Toggle 20-25s")

        # 여기서는 subplot이 여러 개 (rows x 2)
        # axes는 2차원 배열 (rows, 2)
        # sharex=False로 되어있으나 시간축은 비슷할 것임. 대표로 첫번째 ax의 xlim을 full로 잡거나, 
        # 각 ax별로 full을 따로 저장해야할 수도 있음. 
        # 코드상 sharex=False이므로 개별 full_xlim 저장 필요
        
        # 각 ax별 full range 저장
        full_xlims = {}
        for row_axes in axes:
            for ax in row_axes:
                full_xlims[ax] = ax.get_xlim()
        
        zoom_xlim = (20.0, 25.0)
        
        fig._toggle_zoom_state = False
        
        # 클로저 문제를 피하기 위해 함수 내 함수 정의
        def _on_toggle(event, f=fig, axs=axes, f_xlims=full_xlims, z_lim=zoom_xlim):
            f._toggle_zoom_state = not f._toggle_zoom_state
            is_zoom = f._toggle_zoom_state
            
            for row_axes in axs:
                for ax in row_axes:
                    if is_zoom:
                        ax.set_xlim(z_lim)
                    else:
                        ax.set_xlim(f_xlims[ax])
            f.canvas.draw_idle()

        # lambda나 functools.partial 대신 직접 정의한 함수 연결
        btn.on_clicked(_on_toggle)
        fig._toggle_btn = btn  # GC 방지

        figs.append(fig)

    return figs


def print_hdf5_structure_simple(file_path):
    import h5py
    with h5py.File(file_path, 'r') as f:
        print(f"\n[HDF5 Structure (Simplified): {file_path}]")
        # Exclude S001, S002, S003
        subjects = [sub for sub in f.keys() if sub not in ['S001', 'S002', 'S003']]
        print("Subjects:")
        for sub in subjects:
            print(f"  {sub}")
        # Collect all unique trial/condition names
        trial_set = set()
        for sub in subjects:
            for cond in f[sub].keys():
                if not cond.startswith('sub_info'):
                    trial_set.add(cond)
        print("\nTrial/Condition types:")
        for cond in sorted(trial_set):
            print(f"  {cond}")
        # Show example structure for one trial_01 with MoCap
        example_sub = None
        example_trial = None
        for sub in subjects:
            for cond in f[sub].keys():
                if 'trial_01' in f[sub][cond] and 'MoCap' in f[sub][cond]['trial_01']:
                    example_sub = sub
                    example_trial = cond
                    break
            if example_sub:
                break
        def print_compact_group(g, indent=0):
            prefix = '  ' * indent
            keys = list(g.keys())
            group_keys = [k for k in keys if isinstance(g[k], h5py.Group)]
            dataset_keys = [k for k in keys if isinstance(g[k], h5py.Dataset)]
            # 그룹명 출력 (최상위는 생략)
            if indent > 0:
                print(f"{prefix}{g.name.split('/')[-1]}/")
            # 데이터셋 여러 줄로 wrap될 때도 들여쓰기 동일하게
            if dataset_keys:
                line = ''
                maxlen = 100  # 한 줄 최대 길이 (조정 가능)
                current = ''
                for i, d in enumerate(dataset_keys):
                    part = (d + ' / ') if i < len(dataset_keys)-1 else d
                    if len(current) + len(part) > maxlen:
                        print(f"{prefix}  {current.rstrip(' / ')}")
                        current = part
                    else:
                        current += part
                if current:
                    print(f"{prefix}  {current.rstrip(' / ')}")
            # 하위 그룹 재귀적으로 출력
            for k in group_keys:
                print_compact_group(g[k], indent+1)
        if example_sub and example_trial:
            print(f"\nExample structure for {example_sub}/{example_trial}/trial_01:")
            print_compact_group(f[example_sub][example_trial]['trial_01'])

def zoom_factory(ax, base_scale=1.2):
    def zoom(event):
        # Only react when the mouse is over this axes
        if event.inaxes is not ax:
            return

        cur_xlim = ax.get_xlim()
        cur_ylim = ax.get_ylim()
        xdata = event.xdata
        ydata = event.ydata
        if xdata is None or ydata is None:
            return
        if event.button == 'up':
            scale_factor = 1 / base_scale
        elif event.button == 'down':
            scale_factor = base_scale
        else:
            scale_factor = 1
        new_width = (cur_xlim[1] - cur_xlim[0]) * scale_factor
        new_height = (cur_ylim[1] - cur_ylim[0]) * scale_factor
        relx = (cur_xlim[1] - xdata) / (cur_xlim[1] - cur_xlim[0])
        rely = (cur_ylim[1] - ydata) / (cur_ylim[1] - cur_ylim[0])
        ax.set_xlim([xdata - new_width * (1 - relx), xdata + new_width * relx])
        ax.set_ylim([ydata - new_height * (1 - rely), ydata + new_height * rely])
        ax.figure.canvas.draw_idle()
    return zoom

def attach_zoom_factory(fig, base_scale=1.2):
    """Attach ONE scroll-event handler per figure."""
    def _on_scroll(event):
        ax = event.inaxes
        if ax is None:
            return
        # ax가 fig의 axes 중 하나일 때만 줌
        if ax not in fig.axes:
            return
        zoom_factory(ax, base_scale=base_scale)(event)

    fig.canvas.mpl_connect('scroll_event', _on_scroll)

def plot_trials_with_nan(h5_path, sub_names, cond_names, input_vars, output_vars, fs=100, max_trials=50):
    import h5py
    import numpy as np
    import matplotlib.pyplot as plt

    def _read_group_vars(trial_group, group_path, var_list):
        grp = trial_group
        for p in group_path.split('/'):
            if p:
                grp = grp[p]
        cols = []
        for v in var_list:
            arr = grp[v][:]
            if arr.ndim == 1:
                arr = arr[:, None]
            cols.append(arr)
        if not cols:
            return None
        return np.concatenate(cols, axis=1)

    shown = 0

    with h5py.File(h5_path, 'r') as f:
        for sub in sub_names:
            for cond in cond_names:
                trial_path = f"{sub}/{cond}/trial_01"
                if trial_path not in f:
                    continue

                g = f[trial_path]

                X_parts = []
                for group_path, var_list in input_vars:
                    try:
                        mat = _read_group_vars(g, group_path, var_list)
                        if mat is not None:
                            X_parts.append(mat)
                    except Exception:
                        continue
                if not X_parts:
                    continue
                X_mat = np.concatenate(X_parts, axis=1)

                Y_parts = []
                for group_path, var_list in output_vars:
                    try:
                        mat = _read_group_vars(g, group_path, var_list)
                        if mat is not None:
                            Y_parts.append(mat)
                    except Exception:
                        continue
                if not Y_parts:
                    continue
                Y_mat = np.concatenate(Y_parts, axis=1)

                has_nan = np.isnan(X_mat).any() or np.isnan(Y_mat).any()
                if not has_nan:
                    continue

                shown += 1
                T = X_mat.shape[0]
                t = np.arange(T) / fs

                fig, axes = plt.subplots(2, 1, figsize=(16, 9), sharex=True)
                attach_zoom_factory(fig)
                fig.suptitle(f"NaN detected: {sub} / {cond} / trial_01", fontsize=14)

                axes[0].plot(t, X_mat)
                axes[0].set_title("Inputs (input_vars)")
                axes[0].grid(True)

                axes[1].plot(t, Y_mat)
                axes[1].set_title("Outputs (output_vars)")
                axes[1].grid(True)
                axes[1].set_xlabel("Time [s]")

                plt.tight_layout(rect=[0, 0, 1, 0.95])

                if shown >= max_trials:
                    # plt.show()
                    print(f"[INFO] NaN 포함 trial이 {shown}개 이상이라 max_trials={max_trials}에서 중단했습니다.")
                    return

    if shown <= 0:
        print("[INFO] NaN이 포함된 trial 데이터셋이 없습니다.")

def interp_nan_2d_linear_local(A, fill_all_nan_with=0.0, verbose=False, tag=""):
    """
    A: (T, D) 2D array
    각 feature column별로 NaN을 선형 보간.
    특정 column이 전부 NaN이면 fill_all_nan_with로 채움.
    """
    import numpy as np

    A = np.asarray(A, dtype=float)
    if A.ndim != 2:
        return A

    out = A.copy()
    T = out.shape[0]
    idx = np.arange(T)

    for j in range(out.shape[1]):
        col = out[:, j]
        mask = np.isfinite(col)

        # NaN 없음
        if mask.all():
            continue

        # 전부 NaN
        if not mask.any():
            out[:, j] = fill_all_nan_with
            if verbose:
                print(f"[WARN] All-NaN column filled with {fill_all_nan_with}: {tag} col={j}")
            continue

        # 부분 NaN -> 선형 보간
        out[~mask, j] = np.interp(idx[~mask], idx[mask], col[mask])

    return out

def plot_fft_velocity_analysis(h5_path, target_sub, cond_names, fs=100):
    """
    특정 피험자(target_sub)의 모든 조건(cond_names)에 대해 
    v_Y_true, v_Z_true 속도의 FFT 결과를 하나의 Figure에 서브플롯으로 시각화.
    두 속도 성분을 동일한 plot에 겹쳐서 표시.
    """
    import matplotlib.pyplot as plt
    import numpy as np
    from scipy.fft import fft, fftfreq
    
    # 3행 4열 (최대 12개 조건 가정)
    n_cols = 4
    n_rows = (len(cond_names) + n_cols - 1) // n_cols
    
    fig, axes = plt.subplots(n_rows, n_cols, figsize=(16, 3*n_rows), constrained_layout=True)
    axes = axes.flatten()
    
    # Load paths
    var_path_y = "Common/v_Y_true"
    var_path_z = "Common/v_Z_true"

    with h5py.File(h5_path, 'r') as f:
        for idx, cond in enumerate(cond_names):
            ax = axes[idx]
            trial_path = f"{target_sub}/{cond}/trial_01"
            
            if trial_path not in f:
                ax.text(0.5, 0.5, "No Data", ha='center', va='center')
                ax.set_title(cond)
                continue
                
            g = f[trial_path]
            
            # Helper to load and process data
            def get_fft_amp(grp, v_path):
                # path parsing
                parts = v_path.split('/')
                curr = grp
                for p in parts[:-1]:
                    if p not in curr: return None
                    curr = curr[p]
                vname = parts[-1]
                if vname not in curr: return None
                
                d = curr[vname][:]
                if d.ndim > 1: d = d.flatten()
                
                # NaN interp
                if np.isnan(d).any():
                    nans, x = np.isnan(d), lambda z: z.nonzero()[0]
                    if np.all(nans):
                        d[:] = 0.0
                    else:
                        d[nans] = np.interp(x(nans), x(~nans), d[~nans])
                
                # FFT
                N = len(d)
                yf = fft(d)
                # xf = fftfreq(N, 1/fs)[:N//2] # identical for both
                amp = 2.0/N * np.abs(yf[0:N//2])
                return amp, N

            # Calculate FFT for Y and Z
            res_y = get_fft_amp(g, var_path_y)
            res_z = get_fft_amp(g, var_path_z)
            
            if res_y is None and res_z is None:
                ax.text(0.5, 0.5, "Vars Not Found", ha='center', va='center')
                ax.set_title(cond)
                continue
            
            # Frequency axis (assumes same N for both if both exist)
            N = res_y[1] if res_y else (res_z[1] if res_z else 0)
            xf = fftfreq(N, 1/fs)[:N//2]
            
            # Plot
            if res_y is not None:
                ax.plot(xf, res_y[0], label='v_Y', alpha=0.8)
            if res_z is not None:
                ax.plot(xf, res_z[0], label='v_Z', alpha=0.8)
                
            ax.set_title(cond)
            ax.set_xlabel("Freq [Hz]")
            ax.set_ylabel("Mag")
            ax.set_xlim(0, 5) 
            ax.grid(True)
            ax.legend(fontsize=8)
            
        # Hide unused subplots
        for idx in range(len(cond_names), len(axes)):
            axes[idx].axis('off')
            
    fig.suptitle(f"Velocity FFT Analysis (v_Y, v_Z) - Subject: {target_sub}", fontsize=16)
    fig.canvas.manager.set_window_title(f"FFT Analysis: {target_sub}")
    
    return fig

def calculate_and_plot_feature_importance(model, X_test, Y_test, input_vars, device, criterion, fs, stride, tag="BestModel"):
    """
    Permutation Importance:
    X_test의 각 channel(feature)을 하나씩 섞어서 Loss가 얼마나 증가하는지 측정.
    """
    import torch
    import matplotlib.pyplot as plt
    import numpy as np
    
    model.eval()
    
    # 1. Base Loss
    inputs = torch.from_numpy(X_test).float().to(device)
    targets = torch.from_numpy(Y_test).float().to(device)
    
    with torch.no_grad():
        preds = model(inputs)
        base_loss = criterion(preds, targets).item()
    
    print(f"[{tag}] Baseline Loss: {base_loss:.6f}")
    
    # 2. Get Feature Names
    # build_nn_dataset 로직 순서대로 이름 생성
    feature_names = []
    for group_path, var_list in input_vars:
        for v in var_list:
            feature_names.append(f"{group_path.split('/')[-1]}/{v}")
            
    # X_test.shape = (N, Window, Channels)
    n_features = X_test.shape[2]
    
    # Gating 정보(l_contact, r_contact)가 추가되었을 수 있음 (use_gating logic)
    # input_vars 길이보다 n_features가 크다면 뒤쪽은 Gating feature임
    if len(feature_names) < n_features:
        diff = n_features - len(feature_names)
        if diff == 2:
            feature_names.extend(["Gating/L_contact", "Gating/R_contact"])
        else:
            for k in range(diff):
                feature_names.append(f"Extra_{k}")
                
    importances = []
    
    # 3. Permutation Loop
    # 메모리 절약을 위해 numpy상에서 shuffle 후 tensor 변환
    X_test_perm = X_test.copy()
    
    for i in range(n_features):
        old_col = X_test_perm[:, :, i].copy()
        
        # Shuffle along batch dimension (preserve temporal structure within sample? No, usually shuffle samples)
        # Permutation Importance: "Break association between feature and target"
        # Shuffling across samples (axis 0) for the whole column matrix (N, T) is standard.
        np.random.shuffle(X_test_perm[:, :, i])
        
        inputs_p = torch.from_numpy(X_test_perm).float().to(device)
        with torch.no_grad():
            preds_p = model(inputs_p)
            loss_p = criterion(preds_p, targets).item()
            
        imp = loss_p - base_loss
        importances.append(imp)
        print(f"  Feature {i} ({feature_names[i]}): Loss={loss_p:.6f}, Imp={imp:.6f}")
        
        # Restore
        X_test_perm[:, :, i] = old_col

    # 4. Plot
    importances = np.array(importances)
    indices = np.argsort(importances)
    
    sorted_names = [feature_names[i] for i in indices]
    sorted_imp = importances[indices]
    
    # 너무 많으면 상위 20개만
    if len(sorted_names) > 20:
        sorted_names = sorted_names[-20:]
        sorted_imp = sorted_imp[-20:]
        
    fig, ax = plt.subplots(figsize=(10, 8))
    ax.barh(range(len(sorted_names)), sorted_imp, align='center')
    ax.set_yticks(range(len(sorted_names)))
    ax.set_yticklabels(sorted_names)
    ax.set_xlabel("Increase in Loss (Importance)")
    ax.set_title(f"Permutation Importance ({tag})")
    ax.grid(axis='x')
    plt.tight_layout()
    
    
    return fig

def plot_model_summary(results):
    """
    results: list of (model, seed, test_loss, ckpt, config)
    모델별 Test Loss (Huber) 평균 및 표준편차를 Bar Plot으로 시각화
    """
    import matplotlib.pyplot as plt
    import numpy as np

    # Group by model
    model_stats = {}
    for r in results:
        m, s, loss, ckpt, cfg = r
        if m not in model_stats:
            model_stats[m] = []
        model_stats[m].append(loss)
    
    models = sorted(model_stats.keys())
    means = []
    stds = []
    
    print("\n=== Summary (test_loss) ===")
    for m in models:
        vals = model_stats[m]
        mean = np.mean(vals)
        std = np.std(vals) if len(vals) > 1 else 0.0
        
        means.append(mean)
        stds.append(std)
        print(f"{m}: mean={mean:.6f}, std={std:.6f}, n={len(vals)}")
        
    # Plot
    fig, ax = plt.subplots(figsize=(10, 6))
    x_pos = np.arange(len(models))
    
    ax.bar(x_pos, means, yerr=stds, align='center', alpha=0.7, ecolor='black', capsize=10)
    ax.set_xticks(x_pos)
    ax.set_xticklabels(models)
    ax.set_ylabel('Test Loss (Huber)')
    ax.set_title('Model Comparison Results')
    ax.grid(axis='y', linestyle='--', alpha=0.7)
    
    # Add value labels
    for i, v in enumerate(means):
        ax.text(i, v + stds[i] + 0.0001, f"{v:.4f}", ha='center', va='bottom')
        
    plt.tight_layout()
    return fig

if __name__ == "__main__":
    import yaml
    from pathlib import Path

    # Load Base Config
    config_path = Path("configs/base_config.yaml")
    if not config_path.exists():
        print("base_config.yaml not found!")
        exit(1)
        
    with open(config_path, 'r') as f:
        base_config = yaml.safe_load(f)

    # Global Settings from Config
    data_path = base_config.get("data_path", "combined_data.h5")
    all_subjects = sorted(base_config.get("subjects"))
    cond_names = base_config.get("conditions")
    
    input_vars = base_config.get("input_vars")
    output_vars = base_config.get("output_vars")
    
    time_window_input = base_config.get("time_window_input", 100)
    time_window_output = 10 
    stride = 10
    
    lpf_cutoff = base_config.get("lpf_cutoff", 6.0)
    lpf_order = base_config.get("lpf_order", 5)

    fs = 100
    print_hdf5_structure_simple(data_path)
    
    # Models to Compare
    # You can add/remove models here
    model_list = ["TCN_MLP", "TCN_LSTM_MLP"] 
    seeds = [42] # Reduced for speed, user can expand

    all_results = [] # (model, seed, folder_sub, metrics)
    
    print(f"[INFO] Starting Model Comparison with LOSO. Subjects: {all_subjects}")
    
    # Visualization (One-off)
    try:
        target_fft_sub = all_subjects[0]
        # plot_fft_velocity_analysis(data_path, target_fft_sub, cond_names, fs=100)
        # plt.show(block=False)
        pass # Skip for now to focus on training loop
    except: pass
    
    # LOSO Loop
    for i in range(len(all_subjects)):
        test_sub = all_subjects[i]
        val_sub = all_subjects[i-1]
        train_subs = [s for s in all_subjects if s != test_sub and s != val_sub]
        
        print(f"\n==================================================")
        print(f" Fold: Test={test_sub}, Val={val_sub}")
        print(f"==================================================")
        
        # Build Datasets
        X_train, Y_train = build_nn_dataset(
            data_path, train_subs, cond_names,
            input_vars, output_vars,
            time_window_input, time_window_output, stride,
            lpf_cutoff=lpf_cutoff, lpf_order=lpf_order
        )
        X_val, Y_val = build_nn_dataset(
            data_path, [val_sub], cond_names,
            input_vars, output_vars,
            time_window_input, time_window_output, stride,
            lpf_cutoff=lpf_cutoff, lpf_order=lpf_order
        )
        X_test, Y_test = build_nn_dataset(
            data_path, [test_sub], cond_names,
            input_vars, output_vars,
            time_window_input, time_window_output, stride,
            lpf_cutoff=lpf_cutoff, lpf_order=lpf_order
        )
        
        if len(X_train) == 0: 
            print("Skipping empty fold.")
            continue
            
        print("Train:", X_train.shape, Y_train.shape)
        print("Val  :", X_val.shape,   Y_val.shape)
        print("Test :", X_test.shape,  Y_test.shape)

        # Prepare Config for this run
        # We can either tune or use fixed config.
        # Let's use base_config, but ensure TCN stuff is passed correctly.
        train_cfg = base_config.copy()
        train_cfg["epochs"] = base_config.get("epochs", 15)
        # Ensure TCN layers/hidden are present if needed, handled by train_speed_estimator logic we updated

        target_mode = "sequence"
        
        for m in model_list:
            print(f"   [Model: {m}]")
            for sd in seeds:
                ckpt_name = f"ckpt_{test_sub}_{m}_seed{sd}.pt"
                
                # Check if we need live plotter? Maybe too chaotic for 48 runs?
                # Keep it off or optional.
                
                metrics = train_speed_estimator(
                    X_train, Y_train,
                    X_val, Y_val,
                    X_test, Y_test,
                    model_type=m,
                    target_mode=target_mode,
                    cfg=train_cfg,
                    seed=sd,
                    save_path=ckpt_name,
                    live_plotter=None # Disable for massive loop
                )
                
                print(f"      -> Seed={sd}: Huber={metrics['test_huber']:.6f}, Params={metrics['n_params']}")
                
                all_results.append((m, sd, metrics['test_huber'], ckpt_name, metrics.get("model_config", {})))

    # Summary Plot
    if all_results:
        # Convert to format expected by plot_model_summary
        # (model, seed, loss, ckpt, config) - we used (model, seed, loss, ckpt, config) above?
        # Actually plot_model_summary expects: list of (model, seed, test_loss, ckpt, config)
        # My append above: (m, sd, metrics['test_huber'], ckpt_name, metrics.get("model_config", {}))
        # Matches!
        
        summary_fig = plot_model_summary(all_results)
        plt.show(block=True)
    else:
        print("No results to plot.")

    # [수정] 결과 요약 플롯 (Figure로 띄우기)
    summary_fig = plot_model_summary(results)

    best = min(results, key=lambda x: x[2])  # (model, seed, loss, ckpt, config)
    best_model, best_seed, best_loss, best_ckpt, best_config = best

    print(f"Done. Best test_huber: {best_loss:.6f} (model={best_model}, seed={best_seed})")

    # [추가] Best Model Load & Feature Importance
    print("\n=== Calculating Feature Importance for Best Model ===")
    
    # Re-instantiate model
    input_dim = X_test.shape[2]
    horizon = Y_test.shape[1]
    output_dim = Y_test.shape[2]
    
    best_net_cls = {
        "TCN_FC": TCN_FC, "TCN_MLP": TCN_MLP,
        "TCN_GRU_FC": TCN_GRU_FC, "TCN_GRU_MLP": TCN_GRU_MLP,
        "TCN_LSTM_FC": TCN_LSTM_FC, "TCN_LSTM_MLP": TCN_LSTM_MLP
    }[best_model]
    
    # Construct args based on model type
    base_args = {
         "input_dim": input_dim, "output_dim": output_dim, "horizon": horizon,
         "channels": tuple(train_cfg.get("tcn_channels", (64, 64, 128))),
         "kernel_size": int(train_cfg.get("kernel_size", 3)),
         "dropout": float(train_cfg.get("dropout_p", 0.1))
    }
    
    # Add model-specific args from best_config
    if best_model == "TCN_MLP":
        base_args["mlp_hidden"] = best_config.get("mlp_hidden", 256)
    elif best_model in ["TCN_GRU_FC", "TCN_GRU_MLP", "TCN_LSTM_FC", "TCN_LSTM_MLP"]:
        base_args["hidden_size"] = best_config.get("hidden_size", 128)
        base_args["num_layers"] = best_config.get("num_layers", 1)
        
    best_net = best_net_cls(**base_args)
    
    # Load weights
    ckpt_obj = torch.load(best_ckpt)
    state = ckpt_obj["state_dict"] if isinstance(ckpt_obj, dict) and "state_dict" in ckpt_obj else ckpt_obj
    best_net.load_state_dict(state)
    
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    best_net.to(device)
    
    criterion = torch.nn.HuberLoss(delta=train_cfg.get("huber_delta", 0.5))
    
    fi_fig = calculate_and_plot_feature_importance(
        best_net, X_test, Y_test, input_vars, device, criterion, fs, stride, tag=f"Best_{best_model}"
    )
    
    plot_pred_true_all_conditions(
        data_path=data_path,
        sub_names=sub_names,
        cond_names=cond_names,
        input_vars=input_vars,
        output_vars=output_vars,
        time_window_input=time_window_input,
        time_window_output=time_window_output,
        stride=stride,
        test_subjects=TEST_SUBJECTS,
        model_type=best_model,
        target_mode=target_mode,   # sequence로 맞추기
        ckpt_path=best_ckpt,
        batch_size=512,
        fs=100,
        max_points=5000,
        model_cfg=train_cfg,
        lpf_cutoff=LPF_CUTOFF,
        lpf_order=LPF_ORDER
    )
    
    # [수정] 마지막에 모든 플롯 유지
    plt.show(block=True)
