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

# 데이터셋 로드
data_path = '251030_combined_data.h5'

# subject, condition 정보
sub_names = ['S004','S005','S006','S007','S008','S009','S010','S011']
cond_names = [
    'accel_sine','asym_30deg','asym_60deg','cadence_120p','cadence_90p',
    'crouch','decline_5deg','incline_10deg','level_08mps','level_12mps','level_16mps'
]
data_length = 12000

class WindowDataset(Dataset):
    def __init__(self, X, Y, target_mode="sequence"):
        self.X = torch.from_numpy(X).float()
        Y = torch.from_numpy(Y).float()  # (N, H, D)

        if target_mode == "sequence":
            self.Y = Y  # (N, H, D) 그대로
        elif target_mode == "mean":
            self.Y = Y.mean(dim=1)  # (N, D)
        elif target_mode == "last":
            self.Y = Y[:, -1, :]    # (N, D)
        else:
            raise ValueError("target_mode must be 'sequence', 'mean' or 'last'")

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
        return y.view(-1, self.horizon, self.output_dim)  # (B, H, D)


class TCN_GRUDecoder(nn.Module):
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


class TCN_LSTMDecoder(nn.Module):
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


class TCN_TemporalConvDecoder(nn.Module):
    def __init__(self, input_dim, output_dim, horizon, channels=(64,64,128), kernel_size=3, dropout=0.1):
        super().__init__()
        self.enc = TCNEncoder(input_dim, channels, kernel_size, dropout=dropout)
        self.horizon = horizon
        self.output_dim = output_dim
        # length 1 -> length H 로 병렬 생성
        self.deconv = nn.ConvTranspose1d(self.enc.out_ch, output_dim, kernel_size=horizon, stride=1)

    def forward(self, x):
        ctx = self.enc(x)            # (B, C)
        z = ctx.unsqueeze(-1)        # (B, C, 1)
        y = self.deconv(z)           # (B, D, H)
        return y.transpose(1, 2)     # (B, H, D)

def train_speed_estimator(
    X_train, Y_train,
    X_val, Y_val,
    X_test, Y_test,
    model_type="tcn",
    target_mode="mean",
    cfg=None,
    seed=42,
    save_path="speed_estimator_best.pt"
):
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
    tcn_channels = tuple(cfg.get("tcn_channels", (64, 64, 128)))
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

    if model_type == "tcn_mlp":
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

    elif model_type == "tcn_gru":
        hidden0 = int(cfg.get("rnn_hidden", 128))
        num_layers  = int(cfg.get("rnn_layers", 1))
        hidden_size = _tune_hidden(
            lambda h: TCN_GRUDecoder(input_dim, output_dim, horizon,
                                    channels=tcn_channels, kernel_size=kernel_size,
                                    dropout=dropout_p, hidden_size=h, num_layers=num_layers),
            hidden0
        )
        model = TCN_GRUDecoder(input_dim, output_dim, horizon,
                            channels=tcn_channels, kernel_size=kernel_size,
                            dropout=dropout_p, hidden_size=hidden_size, num_layers=num_layers)
        rnn_hidden = hidden_size
        rnn_layers = num_layers

    elif model_type == "tcn_lstm":
        hidden0 = int(cfg.get("rnn_hidden", 128))
        num_layers  = int(cfg.get("rnn_layers", 1))
        hidden_size = _tune_hidden(
            lambda h: TCN_LSTMDecoder(input_dim, output_dim, horizon,
                                    channels=tcn_channels, kernel_size=kernel_size,
                                    dropout=dropout_p, hidden_size=h, num_layers=num_layers),
            hidden0
        )
        model = TCN_LSTMDecoder(input_dim, output_dim, horizon,
                                channels=tcn_channels, kernel_size=kernel_size,
                                dropout=dropout_p, hidden_size=hidden_size, num_layers=num_layers)
        rnn_hidden = hidden_size
        rnn_layers = num_layers

    elif model_type == "tcn_tconv":
        model = TCN_TemporalConvDecoder(input_dim, output_dim, horizon,
                                        channels=tcn_channels, kernel_size=kernel_size,
                                        dropout=dropout_p)
    else:
        raise ValueError("model_type must be one of: tcn_mlp, tcn_gru, tcn_lstm, tcn_tconv")

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
    model_cfg=None
):
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    # 모델을 만들기 위한 dim 확보: 첫 cond로 한번 뽑아서 input_dim/output_dim 결정
    first_cond = cond_names[0]
    X0, Y0 = build_nn_dataset(
        data_path, sub_names, cond_names,
        input_vars, output_vars,
        time_window_input, time_window_output, stride,
        subject_selection={'include': test_subjects, 'exclude': []},
        condition_selection={'include': [first_cond], 'exclude': []}
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

    if model_type == "tcn_mlp":
        model = TCN_MLP(input_dim, output_dim, horizon, channels=tcn_channels, kernel_size=kernel_size, dropout=dropout_p, mlp_hidden=int(model_cfg.get("mlp_hidden", 256)))
    elif model_type == "tcn_gru":
        model = TCN_GRUDecoder(input_dim, output_dim, horizon, channels=tcn_channels, kernel_size=kernel_size, dropout=dropout_p, hidden_size=int(model_cfg.get("rnn_hidden", 128)), num_layers=int(model_cfg.get("rnn_layers", 1)))
    elif model_type == "tcn_lstm":
        model = TCN_LSTMDecoder(input_dim, output_dim, horizon, channels=tcn_channels, kernel_size=kernel_size, dropout=dropout_p, hidden_size=int(model_cfg.get("rnn_hidden", 128)), num_layers=int(model_cfg.get("rnn_layers", 1)))
    elif model_type == "tcn_tconv":
        model = TCN_TemporalConvDecoder(input_dim, output_dim, horizon, channels=tcn_channels, kernel_size=kernel_size, dropout=dropout_p)
    else:
        raise ValueError("unsupported model_type")

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
            condition_selection={'include': [cond], 'exclude': []}
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
            plt.plot(t, true_all[:n_show, j], label="true")
            plt.plot(t, pred_all[:n_show, j], label="pred")
            plt.title(f"{cond} | {name} | test={test_subjects} | model={model_type} | target={target_mode}")
            plt.xlabel("Time [s]")
            plt.ylabel(name)
            plt.grid(True)
            plt.legend()

            # 스크롤 확대/축소 연결
            attach_zoom_factory(fig)


    plt.show()

def build_nn_dataset(h5_path, sub_names, cond_names, input_vars, output_vars, input_window, output_window, stride=1,
    subject_selection=None, condition_selection=None):
    """
    input_vars: list of (group_path, variable_list) tuples, e.g.
        [ ('Robot', ['incPosLH','incPosRH']),
          ('Back_imu', ['Accel_X',...]), ... ]
    output_vars: list of (group_path, variable_list) tuples, e.g.
        [ ('Common', ['v_Y_true','v_Z_true']) ]
    """
    X_list, Y_list = [], []
    with h5py.File(h5_path, 'r') as f:
        # ---- subject selection ----
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

        # ---- condition selection ----
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
                # Build input features
                input_feat = []
                for group_path, var_list in (input_vars or []):
                    grp = g
                    ok = True
                    for p in group_path.split('/'):
                        if not p:
                            continue
                        if p not in grp:
                            ok = False
                            break
                        grp = grp[p]
                    if not ok:
                        continue

                    for v in var_list:
                        if v not in grp:
                            continue
                        arr = grp[v][:]
                        if arr.ndim == 1:
                            arr = arr[:,None]
                        input_feat.append(arr)
                # Build output features
                output_feat = []
                for group_path, var_list in (output_vars or []):
                    grp = g
                    ok = True
                    for p in group_path.split('/'):
                        if not p:
                            continue
                        if p not in grp:
                            ok = False
                            break
                        grp = grp[p]
                    if not ok:
                        continue

                    for v in var_list:
                        if v not in grp:
                            continue
                        arr = grp[v][:]
                        if arr.ndim == 1:
                            arr = arr[:,None]
                        output_feat.append(arr)
                        
                if len(input_feat) == 0 or len(output_feat) == 0:
                    continue
                input_mat = np.concatenate(input_feat, axis=1)
                output_mat = np.concatenate(output_feat, axis=1)

                # 여기 추가: NaN이 있으면 학습 데이터에 반영되도록 보간
                trial_tag = f"{sub}/{cond}/trial_01"
                verbose_nan = bool(condition_selection.get("verbose_nan", False)) if condition_selection else False
                input_mat = interp_nan_2d_linear_local(input_mat, fill_all_nan_with=0.0, verbose=verbose_nan, tag=trial_tag + " input")
                output_mat = interp_nan_2d_linear_local(output_mat, fill_all_nan_with=0.0, verbose=verbose_nan, tag=trial_tag + " output")

                n = len(input_mat)
                for i in range(0, n - input_window - output_window + 1, stride):
                    X = input_mat[i:i+input_window]
                    Y = output_mat[i+input_window:i+input_window+output_window]
                    X_list.append(X)
                    Y_list.append(Y)
    if len(X_list) == 0:
        return np.empty((0, input_window, 0), dtype=np.float32), np.empty((0, output_window, 0), dtype=np.float32)

    X_arr = np.stack(X_list).astype(np.float32)
    Y_arr = np.stack(Y_list).astype(np.float32)
    return X_arr, Y_arr

def load_subject_fsr_imu(data_path, sub_names, cond_names, data_length):
    Sub = {}
    with h5py.File(data_path, 'r') as f:
        for sub_idx, sub_name in enumerate(sub_names, start=1):
            Sub[sub_idx] = {}
            # FSR1
            for side, prefix in [('Left','L'), ('Right','R')]:
                key = f'fsr{prefix}1'
                arr = np.empty((data_length, len(cond_names)), dtype=np.float32)
                for c, cond in enumerate(cond_names):
                    d = f[f"{sub_name}/{cond}/trial_01/Insole/{side}/{key}"]
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
    return Sub

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

def visualize_subject_sensors(Sub, sub_names, cond_names, data_length, fs=100, seconds=20):
    for sub_idx, sub_name in enumerate(sub_names, start=1):
        D = Sub[sub_idx]
        fig, axes = plt.subplots(3, 1, figsize=(14, 8), sharex=True)
        fig.suptitle(f"Subject {sub_name}", fontsize=16)
        t = np.arange(data_length) / fs
        # FSR1 (Left/Right)
        for side in ['L', 'R']:
            key = f'fsr{side}1'
            arr = D[key]
            mean_data = np.nanmean(arr, axis=1)
            axes[0].plot(t, mean_data, label=f'fsr1_{side}')
        axes[0].set_title('FSR1 (Left/Right)')
        axes[0].legend()
        axes[0].grid(True)
        # IMU Acc (X/Y/Z)
        for axis in ['X', 'Y', 'Z']:
            arr = D['BackIMUAcc'+axis]
            mean_data = np.nanmean(arr, axis=1)
            axes[1].plot(t, mean_data, label=f'Acc_{axis}')
        axes[1].set_title('Back IMU Acceleration')
        axes[1].legend()
        axes[1].grid(True)
        # IMU Gyr (X/Y/Z)
        for axis in ['X', 'Y', 'Z']:
            arr = D['BackIMUGyr'+axis]
            mean_data = np.nanmean(arr, axis=1)
            axes[2].plot(t, mean_data, label=f'Gyr_{axis}')
        axes[2].set_title('Back IMU Gyroscope')
        axes[2].legend()
        axes[2].grid(True)
        axes[2].set_xlabel('Time [s]')
        plt.tight_layout(rect=[0, 0, 1, 0.96])
        # plt.show()

def visualize_subject_sensors_all(Sub, sub_names, cond_names, data_length, fs=100, seconds=30):
    figs = []
    T_show = min(int(seconds * fs), data_length)
    t = np.arange(T_show) / fs
    cond_idx = 0  # 첫 번째 조건만 사용

    for sub_idx, sub_name in enumerate(sub_names, start=1):
        D = Sub[sub_idx]

        # load 단계에서 저장해둔 값 사용
        mass = D.get('mass_kg', 'N/A')
        height = D.get('height_m', 'N/A')

        fig, axes = plt.subplots(3, 2, figsize=(16, 12), sharex=True)
        figs.append(fig)
        fig.suptitle(f"Subject {sub_name} (mass: {mass} kg, height: {height} m)", fontsize=16)
        # 1. FSR1 (Left/Right)
        for side, prefix in [('Left', 'L'), ('Right', 'R')]:
            key = f'fsr{prefix}1'
            arr = D.get(key)
            if arr is not None:
                data = arr[:T_show, cond_idx]
                axes[0,0].plot(t, data, label=f'fsr{prefix}1', linewidth=1)
        axes[0,0].set_title('FSR1 (Left/Right)')
        axes[0,0].legend()
        axes[0,0].grid(True)
        # 2. Back IMU Acc (X/Y/Z)
        for axis in ['X', 'Y', 'Z']:
            arr = D.get(f'BackIMUAcc{axis}')
            if arr is not None:
                data = arr[:T_show, cond_idx]
                axes[0,1].plot(t, data, label=f'Acc_{axis}', linewidth=1)
        axes[0,1].set_title('Back IMU Acceleration')
        axes[0,1].legend()
        axes[0,1].grid(True)
        # 3. Back IMU Gyr (X/Y/Z)
        for axis in ['X', 'Y', 'Z']:
            arr = D.get(f'BackIMUGyr{axis}')
            if arr is not None:
                data = arr[:T_show, cond_idx]
                axes[1,0].plot(t, data, label=f'Gyr_{axis}', linewidth=1)
        axes[1,0].set_title('Back IMU Gyroscope')
        axes[1,0].legend()
        axes[1,0].grid(True)
        # 4. Common v_Y_true, v_Z_true
        arr_vy = D.get('v_Y_true')
        arr_vz = D.get('v_Z_true')
        if arr_vy is not None and arr_vz is not None:
            data_vy = arr_vy[:T_show, cond_idx]
            data_vz = arr_vz[:T_show, cond_idx]
            axes[1,1].plot(t, data_vy, label='v_Y_true', linewidth=1)
            axes[1,1].plot(t, data_vz, label='v_Z_true', linewidth=1)
            axes[1,1].set_title('Common v_Y_true, v_Z_true')
            axes[1,1].legend()
            axes[1,1].grid(True)
        else:
            axes[1,1].set_title('Common v_Y_true, v_Z_true (missing)')
        # 5. GRF Fz (left/right)
        arr_LFz = D.get('LGRFz')
        arr_RFz = D.get('RGRFz')
        if arr_LFz is not None and arr_RFz is not None:
            data_LFz = arr_LFz[:T_show, cond_idx]
            data_RFz = arr_RFz[:T_show, cond_idx]
            axes[2,0].plot(t, data_LFz, label='LGRFz', linewidth=1)
            axes[2,0].plot(t, data_RFz, label='RGRFz', linewidth=1)
            axes[2,0].set_title('GRF Fz (Left/Right)')
            axes[2,0].legend()
            axes[2,0].grid(True)
        else:
            axes[2,0].set_title('GRF Fz (missing)')
        # 6. Robot Encoder (currentLH/currentRH만 plot)
        arr_LH = D.get('currentLH')
        arr_RH = D.get('currentRH')
        if arr_LH is not None and arr_RH is not None:
            data_LH = arr_LH[:T_show, cond_idx]
            data_RH = arr_RH[:T_show, cond_idx]
            axes[2,1].plot(t, data_LH, label='currentLH', linewidth=1)
            axes[2,1].plot(t, data_RH, label='currentRH', linewidth=1)
            axes[2,1].set_title('Robot Encoder (currentLH/currentRH)')
            axes[2,1].legend()
            axes[2,1].grid(True)
        else:
            axes[2,1].set_title('Robot Encoder (missing)')
        # Add scroll zoom to the first figure (connect once per figure)
        attach_zoom_factory(fig)

        for ax in axes.flat:
            ax.set_xlabel('Time [s]')

        plt.tight_layout(rect=[0, 0, 1, 0.96])

        # Kinematics 3x2 subplot
        fig2, axes2 = plt.subplots(3, 2, figsize=(14, 10), sharex=True)
        figs.append(fig2)
        fig2.suptitle(f"Subject {sub_name} - Kinematics (mass: {mass} kg, height: {height} m)", fontsize=16)

        kin_pairs = [
            (['LhipFE','hip_flexion_l'], ['RhipFE','hip_flexion_r'], 'Hip FE'),
            (['LhipAA','hip_adduction_l'], ['RhipAA','hip_adduction_r'], 'Hip AA'),
            (['LhipRot','hip_rotation_l'], ['RhipRot','hip_rotation_r'], 'Hip Rot'),
            (['LkneeFE','knee_angle_l'], ['RkneeFE','knee_angle_r'], 'Knee FE'),
            (['LankleFE','ankle_angle_l'], ['RankleFE','ankle_angle_r'], 'Ankle FE'),
            (['incPosLH'], ['incPosRH'], 'Robot Encoder (incPosLH/RH)')
        ]

        for i, (lkeys, rkeys, title) in enumerate(kin_pairs):
            row, col = divmod(i, 2)
            arrL = next((D.get(k) for k in lkeys if D.get(k) is not None), None)
            arrR = next((D.get(k) for k in rkeys if D.get(k) is not None), None)
            if arrL is not None and arrR is not None:
                dataL = arrL[:T_show, cond_idx]
                dataR = arrR[:T_show, cond_idx]
                axes2[row, col].plot(t, dataL, label=lkeys[0], linewidth=1)
                axes2[row, col].plot(t, dataR, label=rkeys[0], linewidth=1)
                axes2[row, col].set_title(title)
                axes2[row, col].legend()
                axes2[row, col].grid(True)
            else:
                axes2[row, col].set_title(f'{title} (missing)')

        # Add scroll zoom to the second figure (connect once per figure)
        attach_zoom_factory(fig2)

        for ax in axes2.flat:
            ax.set_xlabel('Time [s]')

        plt.tight_layout(rect=[0, 0, 1, 0.96])
    # plt.show()

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

def interp_nan_1d_linear(x):
    import numpy as np
    x = np.asarray(x, dtype=float)
    n = x.size
    if n == 0:
        return x
    idx = np.arange(n)
    mask = np.isfinite(x)
    if mask.all():
        return x
    if not mask.any():
        return x  # 전부 NaN이면 그대로 반환 (2D에서 처리)
    y = x.copy()
    y[~mask] = np.interp(idx[~mask], idx[mask], x[mask])
    return y

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
    for j in range(out.shape[1]):
        col = out[:, j]
        if np.isfinite(col).all():
            continue
        if not np.isfinite(col).any():
            out[:, j] = fill_all_nan_with
            if verbose:
                print(f"[WARN] All-NaN column filled with {fill_all_nan_with}: {tag} col={j}")
            continue
        out[:, j] = interp_nan_1d_linear(col)
    return out

if __name__ == "__main__":
    fs = 100
    print_hdf5_structure_simple(data_path)
    Sub = load_subject_fsr_imu_kin(data_path, sub_names, cond_names, data_length)
    visualize_subject_sensors_all(Sub, sub_names, cond_names, data_length)

    # ===============================
    # User-defined subject split
    # ===============================
    TRAIN_SUBJECTS = ['S004', 'S005', 'S006', 'S007', 'S008', 'S009']
    VAL_SUBJECTS   = ['S010']
    TEST_SUBJECTS  = ['S011']

    def make_subject_selection(include_list):
        return {'include': include_list, 'exclude': []}

    CONDITION_SELECTION = {
        'include': None,      # e.g. ['level_12mps']
        'exclude': []         # e.g. ['crouch']
    }

    input_vars = [
        ('Robot', ['incPosLH', 'incPosRH']),
        ('Back_imu', ['Accel_X', 'Accel_Y', 'Accel_Z', 'Gyro_X', 'Gyro_Y', 'Gyro_Z', 'Pitch', 'Roll', 'Yaw']),
        ('MoCap/grf_measured/left/force', ['Fx', 'Fy', 'Fz']),
        ('MoCap/grf_measured/right/force', ['Fx', 'Fy', 'Fz']),
        ('MoCap/kin_q', ['ankle_angle_l', 'ankle_angle_r', 'hip_adduction_l', 'hip_adduction_r', 'hip_flexion_l', 'hip_flexion_r', 'hip_rotation_l', 'hip_rotation_r', 'knee_angle_l', 'knee_angle_r']),
        ('MoCap/kin_qdot', ['ankle_angle_l', 'ankle_angle_r', 'hip_adduction_l', 'hip_adduction_r', 'hip_flexion_l', 'hip_flexion_r', 'hip_rotation_l', 'hip_rotation_r', 'knee_angle_l', 'knee_angle_r'])
    ]
    output_vars = [
        ('Common', ['v_Y_true', 'v_Z_true'])
    ]

    time_window_input = 100  # input window length (samples)
    time_window_output = 10  # output window length (samples)
    stride = 10

    plot_trials_with_nan(data_path, sub_names, cond_names, input_vars, output_vars, fs=100, max_trials=50)
    plt.show(block=False)
    plt.pause(10)  # GUI 이벤트 루프에 제어권 잠깐 넘김

    SUBJECT_SELECTION_TRAIN = make_subject_selection(TRAIN_SUBJECTS)
    SUBJECT_SELECTION_VAL   = make_subject_selection(VAL_SUBJECTS)
    SUBJECT_SELECTION_TEST  = make_subject_selection(TEST_SUBJECTS)

    X_train, Y_train = build_nn_dataset(
        data_path, sub_names, cond_names,
        input_vars, output_vars,
        time_window_input, time_window_output, stride,
        subject_selection=SUBJECT_SELECTION_TRAIN,
        condition_selection=CONDITION_SELECTION
    )

    X_val, Y_val = build_nn_dataset(
        data_path, sub_names, cond_names,
        input_vars, output_vars,
        time_window_input, time_window_output, stride,
        subject_selection=SUBJECT_SELECTION_VAL,
        condition_selection=CONDITION_SELECTION
    )

    X_test, Y_test = build_nn_dataset(
        data_path, sub_names, cond_names,
        input_vars, output_vars,
        time_window_input, time_window_output, stride,
        subject_selection=SUBJECT_SELECTION_TEST,
        condition_selection=CONDITION_SELECTION
    )

    print("Train:", X_train.shape, Y_train.shape)
    print("Val  :", X_val.shape,   Y_val.shape)
    print("Test :", X_test.shape,  Y_test.shape)

    train_cfg = {
        "batch_size": 1024,
        "val_batch_size": 1024,
        "epochs": 20,
        "patience": 5,
        "dropout_p": 0.3,
        "lr": 3e-4,
        "weight_decay": 1e-2,
        "cos_T0": 10,
        "cos_Tmult": 1,
        "eta_min": 1e-5,
        "huber_delta": 0.5,
        "print_every": 50,
        "tcn_channels": (64, 64, 128),
        "kernel_size": 3,

        # 파라미터 budget 맞추기
        "param_budget": None,   # None이면 main에서 tcn_mlp 기준으로 자동 설정
        "param_tol": 0.03,      # 3% 정도 오차 허용
        "max_hidden": 1024      # mlp_hidden / rnn_hidden 탐색 상한
    }

    if train_cfg.get("param_budget", None) is None:
        # (input_dim, output_dim, horizon)을 여기서 알 수 있어야 함
        input_dim = X_train.shape[2]
        horizon = Y_train.shape[1]         # target_mode="sequence" 가정
        output_dim = Y_train.shape[2]

        ref = TCN_MLP(
            input_dim=input_dim,
            output_dim=output_dim,
            horizon=horizon,
            channels=tuple(train_cfg.get("tcn_channels", (64, 64, 128))),
            kernel_size=int(train_cfg.get("kernel_size", 3)),
            dropout=float(train_cfg.get("dropout_p", 0.1)),
            mlp_hidden=int(train_cfg.get("mlp_hidden", 256)),
        )
        ref_budget = sum(p.numel() for p in ref.parameters() if p.requires_grad)
        train_cfg["param_budget"] = int(ref_budget)
        print(f"[PARAM_BUDGET] reference=tcn_mlp params={ref_budget}")

    # 미래 10 tick 전체를 학습하려면 sequence 모드로
    target_mode = "sequence"

    model_list = ["tcn_mlp", "tcn_gru", "tcn_lstm", "tcn_tconv"]
    seeds = [42, 43, 44]  # 최소 3개, 여유되면 5개 이상 권장

    results = []

    for m in model_list:
        for sd in seeds:
            ckpt = f"speed_estimator_best_{m}_seed{sd}.pt"
            metrics = train_speed_estimator(
                X_train, Y_train,
                X_val, Y_val,
                X_test, Y_test,
                model_type=m,
                target_mode=target_mode,
                cfg=train_cfg,
                seed=sd,
                save_path=ckpt
            )
            print(f"[RESULT] model={m} seed={sd} test_huber={metrics['test_huber']:.6f} "
                f"params={metrics['n_params']} ckpt={ckpt}")

            if metrics.get("step_mae") is not None:
                step_mae = metrics["step_mae"].mean(axis=1)    # (H,)
                step_rmse = metrics["step_rmse"].mean(axis=1)  # (H,)
                dt_fs = 1.0 / fs
                ahead_s_fs = (np.arange(len(step_mae)) + 1) * dt_fs
                ahead_s_stride = (np.arange(len(step_mae)) + 1) * (stride / fs)
                print("  ahead_s(fs):", np.array2string(ahead_s_fs, precision=3, separator=","))
                print("  ahead_s(stride):", np.array2string(ahead_s_stride, precision=3, separator=","))
                print("  step_MAE(mean over D):", np.array2string(step_mae, precision=4, separator=","))
                print("  step_RMSE(mean over D):", np.array2string(step_rmse, precision=4, separator=","))

            results.append((m, sd, float(metrics["test_huber"]), ckpt))

    # 결과 요약 출력
    print("\n=== Summary (test_loss) ===")
    for m in model_list:
        vals = [r[2] for r in results if r[0] == m]
        if len(vals) == 0:
            continue
        mean = sum(vals) / len(vals)
        std = (sum((v-mean)**2 for v in vals) / max(1, len(vals)-1))**0.5 if len(vals) > 1 else 0.0
        print(f"{m}: mean={mean:.6f}, std={std:.6f}, n={len(vals)}")

    best = min(results, key=lambda x: x[2])  # (model, seed, loss, ckpt)
    best_model, best_seed, best_loss, best_ckpt = best

    print(f"Done. Best test_huber: {best_loss:.6f} (model={best_model}, seed={best_seed})")

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
        model_cfg=train_cfg
    )
