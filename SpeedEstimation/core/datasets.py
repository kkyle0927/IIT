import torch
import numpy as np
from torch.utils.data import Dataset


class LazyWindowDataset(Dataset):
    def __init__(self, X_list, Y_list, input_window, output_window, stride, target_mode="sequence", est_tick_ranges=None, augment=False, noise_std=0.0, ar_k=0, device=None):
        """
        X_list: list of np.array (T, D_in) - Raw time series
        Y_list: list of np.array (T, D_out)
        target_mode: 'sequence', 'mean', 'last' (Used only if est_tick_ranges is None)
        est_tick_ranges: list of ints, e.g. [1, 2, 3] or [10]. If set, Y is (D_out, len(ranges)) or similar
        augment: bool, if True apply random noise
        noise_std: float, standard deviation of Gaussian noise
        ar_k: int, number of AR feedback channels to return as y_prev (0 = disabled)
        device: torch.device or None. If set, all tensors are moved to this device at init
                (eliminates per-batch .to(device) overhead; effective when dataset fits in GPU VRAM)
        """
        self.X_list = [torch.from_numpy(x).float() for x in X_list]
        self.Y_list = [torch.from_numpy(y).float() for y in Y_list]
        if device is not None:
            self.X_list = [x.to(device) for x in self.X_list]
            self.Y_list = [y.to(device) for y in self.Y_list]
        self.input_window = input_window
        self.output_window = output_window
        self.stride = stride
        self.target_mode = target_mode
        self.est_tick_ranges = est_tick_ranges
        self.augment = augment
        self.noise_std = noise_std
        self.ar_k = ar_k  # [AR Feedback] number of AR channels

        # Determine max lookahead for validity check
        if self.est_tick_ranges:
            self.max_lookahead = max(self.est_tick_ranges)
        else:
            self.max_lookahead = output_window

        # Pre-calculate valid indices
        self.indices = []
        for s_idx, x_data in enumerate(self.X_list):
            length = x_data.shape[0]
            limit = length - input_window - self.max_lookahead + 1
            if limit > 0:
                for t in range(0, limit, stride):
                    self.indices.append((s_idx, t))

    def __len__(self):
        return len(self.indices)

    def __getitem__(self, idx):
        s_idx, t = self.indices[idx]

        x_data = self.X_list[s_idx]
        y_data = self.Y_list[s_idx]

        limit = t + self.input_window

        if self.augment and self.noise_std > 0:
            x_win = x_data[t : limit].clone()
            x_win += torch.randn_like(x_win) * self.noise_std
        else:
            x_win = x_data[t : limit]  # view (no copy needed when not augmenting)

        if self.est_tick_ranges:
            y_indices = [(limit + k - 1) for k in self.est_tick_ranges]
            y_win = y_data[y_indices]  # (Len(ranges), D_out)
            first_target_idx = y_indices[0]
        else:
            end = limit + self.output_window
            y_win = y_data[limit : end]  # Sequence target (Win_Out, D_out)
            first_target_idx = limit

        if self.target_mode == "mean":
            y_win = y_win.mean(dim=0)
        elif self.target_mode == "last":
            y_win = y_win[-1, :]

        # [AR Feedback] Return y_prev = ground truth one step before first prediction target
        if self.ar_k > 0:
            y_prev_idx = first_target_idx - 1
            y_prev_raw = y_data[y_prev_idx]  # (D_out,)
            D_out = y_prev_raw.shape[0]
            if D_out >= self.ar_k:
                y_prev = y_prev_raw[:self.ar_k]
            else:
                y_prev = y_prev_raw.repeat(self.ar_k)[:self.ar_k]
            return x_win, y_win, y_prev  # (T, D_in), y_win, (ar_k,)

        return x_win, y_win
