import torch
import numpy as np
from torch.utils.data import Dataset


def _time_warp_1d(x, sigma=0.2, n_knots=4):
    """Apply random time warping to a 1D sequence by distorting the time axis.
    x: (T, D) tensor
    Returns: warped (T, D) tensor
    """
    T = x.shape[0]
    if T < n_knots + 2:
        return x

    # Generate random warping knots
    orig_steps = torch.linspace(0, 1, T, device=x.device)
    knot_pos = torch.linspace(0, 1, n_knots + 2, device=x.device)
    # Random perturbation of knot positions (keep endpoints fixed)
    perturbation = torch.randn(n_knots, device=x.device) * sigma
    knot_pos_warped = knot_pos.clone()
    knot_pos_warped[1:-1] += perturbation
    # Ensure monotonicity
    knot_pos_warped, _ = torch.sort(knot_pos_warped)
    knot_pos_warped = torch.clamp(knot_pos_warped, 0.0, 1.0)

    # Interpolate warped time axis
    warped_steps = torch.zeros_like(orig_steps)
    for i in range(len(knot_pos) - 1):
        mask = (orig_steps >= knot_pos[i]) & (orig_steps <= knot_pos[i + 1])
        if mask.any():
            frac = (orig_steps[mask] - knot_pos[i]) / (knot_pos[i + 1] - knot_pos[i] + 1e-8)
            warped_steps[mask] = knot_pos_warped[i] + frac * (knot_pos_warped[i + 1] - knot_pos_warped[i])

    # Convert warped steps to indices
    warped_indices = warped_steps * (T - 1)
    warped_indices = torch.clamp(warped_indices, 0, T - 1)

    # Linear interpolation
    idx_floor = warped_indices.long()
    idx_ceil = torch.clamp(idx_floor + 1, max=T - 1)
    frac = (warped_indices - idx_floor.float()).unsqueeze(-1)

    x_warped = x[idx_floor] * (1 - frac) + x[idx_ceil] * frac
    return x_warped


def _expand_like(base, ref):
    while base.ndim < ref.ndim:
        if isinstance(base, torch.Tensor):
            base = base.unsqueeze(-1)
        else:
            base = np.expand_dims(base, axis=-1)
    return base


def extract_target_base(x, target_transform=None, input_mean=None, input_std=None):
    if not target_transform:
        return None
    if target_transform.get("mode") != "delta_from_input":
        return None

    idx = int(target_transform["input_channel_idx"])

    if isinstance(x, torch.Tensor):
        base = x[-1, idx] if x.dim() == 2 else x[:, -1, idx]
        if input_std is not None:
            scale = input_std[idx] if isinstance(input_std, torch.Tensor) else float(input_std[idx])
            base = base * scale
        if input_mean is not None:
            mean = input_mean[idx] if isinstance(input_mean, torch.Tensor) else float(input_mean[idx])
            base = base + mean
        return base

    base = x[-1, idx] if x.ndim == 2 else x[:, -1, idx]
    if input_std is not None:
        base = base * input_std[idx]
    if input_mean is not None:
        base = base + input_mean[idx]
    return base


def apply_target_transform(y, x, target_transform=None, input_mean=None, input_std=None):
    base = extract_target_base(x, target_transform, input_mean=input_mean, input_std=input_std)
    if base is None:
        return y
    return y - _expand_like(base, y)


def inverse_target_transform(y, x, target_transform=None, input_mean=None, input_std=None):
    base = extract_target_base(x, target_transform, input_mean=input_mean, input_std=input_std)
    if base is None:
        return y
    return y + _expand_like(base, y)


class LazyWindowDataset(Dataset):
    def __init__(self, X_list, Y_list, input_window, output_window, stride, target_mode="sequence", est_tick_ranges=None, augment=False, noise_std=0.0, ar_k=0, device=None, time_warp_sigma=0.0, noise_std_per_dim=None, target_transform=None, input_mean=None, input_std=None, return_index_info=False):
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
        time_warp_sigma: float, if > 0, apply random time warping with this sigma (only during augment)
        noise_std_per_dim: 1-D tensor/array of shape (D_in,) for per-feature noise augmentation
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
        self.time_warp_sigma = time_warp_sigma
        self.target_transform = target_transform
        self.input_mean = None if input_mean is None else torch.as_tensor(input_mean, dtype=torch.float32, device=device)
        self.input_std = None if input_std is None else torch.as_tensor(input_std, dtype=torch.float32, device=device)
        self.return_index_info = return_index_info
        # Per-dim noise: 1-D tensor of shape (D_in,) — overrides noise_std when set
        if noise_std_per_dim is not None:
            self.noise_std_per_dim = torch.as_tensor(noise_std_per_dim, dtype=torch.float32)
            if device is not None:
                self.noise_std_per_dim = self.noise_std_per_dim.to(device)
        else:
            self.noise_std_per_dim = None

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

        if self.augment and self.noise_std_per_dim is not None:
            x_win = x_data[t : limit].clone()
            x_win += torch.randn_like(x_win) * self.noise_std_per_dim
        elif self.augment and self.noise_std > 0:
            x_win = x_data[t : limit].clone()
            x_win += torch.randn_like(x_win) * self.noise_std
        else:
            x_win = x_data[t : limit]  # view (no copy needed when not augmenting)

        # Time warp augmentation (input only, applied after noise)
        if self.augment and self.time_warp_sigma > 0:
            if not isinstance(x_win, torch.Tensor) or not x_win.is_contiguous():
                x_win = x_win.clone()
            x_win = _time_warp_1d(x_win, sigma=self.time_warp_sigma)

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

        y_win = apply_target_transform(
            y_win, x_win,
            target_transform=self.target_transform,
            input_mean=self.input_mean,
            input_std=self.input_std,
        )

        # [AR Feedback] Return y_prev = ground truth one step before first prediction target
        if self.ar_k > 0:
            y_prev_idx = first_target_idx - 1
            y_prev_raw = y_data[y_prev_idx]  # (D_out,)
            D_out = y_prev_raw.shape[0]
            if D_out >= self.ar_k:
                y_prev = y_prev_raw[:self.ar_k]
            else:
                y_prev = y_prev_raw.repeat(self.ar_k)[:self.ar_k]
            if self.return_index_info:
                return x_win, y_win, y_prev, s_idx, t
            return x_win, y_win, y_prev  # (T, D_in), y_win, (ar_k,)

        if self.return_index_info:
            return x_win, y_win, s_idx, t
        return x_win, y_win
