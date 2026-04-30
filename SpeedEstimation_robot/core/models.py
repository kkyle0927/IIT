import torch
import torch.nn as nn
import torch.nn.functional as F
from typing import Optional


_LOG_2PI = 1.8378770664093453


def _reshape_mdn_raw(raw: torch.Tensor, output_dim: int, num_mixtures: int) -> torch.Tensor:
    last_dim = output_dim * num_mixtures * 3
    if raw.shape[-1] != last_dim:
        raise ValueError(
            f"Invalid MDN raw shape {tuple(raw.shape)} for output_dim={output_dim}, "
            f"num_mixtures={num_mixtures}."
        )
    return raw.view(*raw.shape[:-1], output_dim, num_mixtures, 3)


def split_mdn_output(raw: torch.Tensor, output_dim: int, num_mixtures: int):
    reshaped = _reshape_mdn_raw(raw, output_dim, num_mixtures)
    logits = reshaped[..., 0]
    means = reshaped[..., 1]
    log_scales = torch.clamp(reshaped[..., 2], min=-7.0, max=5.0)
    return logits, means, log_scales


def mdn_expected_value(raw: torch.Tensor, output_dim: int, num_mixtures: int) -> torch.Tensor:
    logits, means, _ = split_mdn_output(raw, output_dim, num_mixtures)
    weights = F.softmax(logits, dim=-1)
    return torch.sum(weights * means, dim=-1)


def mdn_predictive_std(raw: torch.Tensor, output_dim: int, num_mixtures: int) -> torch.Tensor:
    logits, means, log_scales = split_mdn_output(raw, output_dim, num_mixtures)
    weights = F.softmax(logits, dim=-1)
    variances = torch.exp(2.0 * log_scales)
    mean = torch.sum(weights * means, dim=-1)
    second_moment = torch.sum(weights * (variances + means.pow(2)), dim=-1)
    pred_var = torch.clamp(second_moment - mean.pow(2), min=1e-8)
    return torch.sqrt(pred_var)


class MDNLoss(nn.Module):
    def __init__(self, output_dim: int, num_mixtures: int, reduction: str = "mean"):
        super().__init__()
        self.output_dim = output_dim
        self.num_mixtures = num_mixtures
        self.reduction = reduction

    def forward(self, raw: torch.Tensor, target: torch.Tensor) -> torch.Tensor:
        logits, means, log_scales = split_mdn_output(raw, self.output_dim, self.num_mixtures)

        if target.dim() == raw.dim() - 1:
            target = target.unsqueeze(-1)

        target = target.unsqueeze(-1)
        inv_std = torch.exp(-log_scales)
        sq_term = ((target - means) * inv_std).pow(2)
        comp_log_prob = -0.5 * (sq_term + 2.0 * log_scales + _LOG_2PI)
        log_weights = F.log_softmax(logits, dim=-1)
        nll = -torch.logsumexp(log_weights + comp_log_prob, dim=-1)

        if self.reduction == "none":
            return nll
        if self.reduction == "sum":
            return nll.sum()
        return nll.mean()


class Chomp1d(nn.Module):
    """Remove the last `chomp_size` elements to keep causality after padding."""
    def __init__(self, chomp_size: int):
        super().__init__()
        self.chomp_size = chomp_size

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        if self.chomp_size == 0:
            return x
        return x[:, :, :-self.chomp_size]


class TemporalBlock(nn.Module):
    def __init__(
        self,
        in_ch: int,
        out_ch: int,
        kernel_size: int,
        stride: int,
        dilation: int,
        dropout: float,
        norm_type: Optional[str] = None,
    ):
        super().__init__()
        from torch.nn.utils import weight_norm
        padding = (kernel_size - 1) * dilation  # causal padding

        self.conv1 = nn.Conv1d(in_ch, out_ch, kernel_size, stride=stride, padding=padding, dilation=dilation)
        self.chomp1 = Chomp1d(padding)
        self.relu1 = nn.ReLU(inplace=True)
        self.drop1 = nn.Dropout(dropout)

        self.conv2 = nn.Conv1d(out_ch, out_ch, kernel_size, stride=stride, padding=padding, dilation=dilation)
        self.chomp2 = Chomp1d(padding)
        self.relu2 = nn.ReLU(inplace=True)
        self.drop2 = nn.Dropout(dropout)

        if norm_type == 'weight':
            self.conv1 = weight_norm(self.conv1)
            self.conv2 = weight_norm(self.conv2)
            self.bn1 = nn.Identity()
            self.bn2 = nn.Identity()
        elif norm_type == 'batch':
            self.bn1 = nn.BatchNorm1d(out_ch)
            self.bn2 = nn.BatchNorm1d(out_ch)
        elif norm_type == 'layer':
            self.bn1 = nn.GroupNorm(1, out_ch)
            self.bn2 = nn.GroupNorm(1, out_ch)
        elif norm_type == 'instance':
            self.bn1 = nn.InstanceNorm1d(out_ch)
            self.bn2 = nn.InstanceNorm1d(out_ch)
        else:
            self.bn1 = nn.Identity()
            self.bn2 = nn.Identity()

        self.net = nn.Sequential(
            self.conv1, self.chomp1, self.bn1, self.relu1, self.drop1,
            self.conv2, self.chomp2, self.bn2, self.relu2, self.drop2
        )

        self.downsample = nn.Conv1d(in_ch, out_ch, 1) if in_ch != out_ch else nn.Identity()
        self.relu = nn.ReLU(inplace=True)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        out = self.net(x)
        res = self.downsample(x)
        return self.relu(out + res)


class TCNEncoder(nn.Module):
    def __init__(self, input_dim, channels=(64, 64, 128), kernel_size=3, dropout=0.1, norm_type=None):
        super().__init__()

        layers = []
        in_ch = input_dim
        for i, ch in enumerate(channels):
            dilation = 2 ** i
            layers.append(TemporalBlock(in_ch, ch, kernel_size, stride=1, dilation=dilation, dropout=dropout, norm_type=norm_type))
            in_ch = ch

        self.network = nn.Sequential(*layers)
        self.out_ch = channels[-1]

    def forward(self, x):
        # x: (B, T, D) -> (B, D, T)
        # Handle 2D input (B, D) -> (B, 1, D)
        if x.dim() == 2:
            x = x.unsqueeze(1)
        x = x.transpose(1, 2)
        y = self.network(x)
        return y[:, :, -1]


class TCN_MLP(nn.Module):
    def __init__(self, input_dim, output_dim, horizon, channels=(64,64,128), kernel_size=3,
                 dropout=0.1, head_dropout=None, mlp_hidden=128,
                 use_input_norm=False,
                 tcn_norm=None,
                 mlp_norm=None, **kwargs):
        super().__init__()

        # Residual skip connection: output = network(x) + x[:, -1, idx]
        residual_skip = kwargs.get('residual_skip', None)
        self.residual_skip_idx = None
        if residual_skip and residual_skip.get('enable', False):
            self.residual_skip_idx = int(residual_skip['input_channel_idx'])

        self.use_input_norm = use_input_norm
        if use_input_norm:
            norm_type = kwargs.get('input_norm_type', 'layer')
            if norm_type == 'instance':
                self.input_norm = nn.InstanceNorm1d(input_dim)
            else:
                self.input_norm = nn.LayerNorm(input_dim)

        # TCN Encoder
        self.enc = TCNEncoder(input_dim, channels, kernel_size, dropout, norm_type=tcn_norm)

        self.horizon = horizon
        self.output_dim = output_dim

        # Calculate Head Input Size from TCN Output
        head_in = self.enc.out_ch

        # MLP Head construction
        head_layers = []

        if isinstance(mlp_hidden, (list, tuple)):
            h_list = mlp_hidden
        else:
            h_list = [mlp_hidden]

        if head_dropout is None: head_dropout = dropout

        curr_in = head_in
        for h_size in h_list:
            head_layers.append(nn.Linear(curr_in, h_size))
            if mlp_norm == 'batch':
                head_layers.append(nn.BatchNorm1d(h_size))
            elif mlp_norm == 'layer':
                head_layers.append(nn.LayerNorm(h_size))

            head_layers.append(nn.ReLU())
            if head_dropout > 0:
                head_layers.append(nn.Dropout(head_dropout))

            curr_in = h_size

        self.head_base = nn.Sequential(*head_layers)

        # Final Output Layer
        self.head_out = nn.Linear(curr_in, horizon * output_dim)

    def _get_residual_base(self, x):
        """Extract skip connection base value from raw input (before normalization)."""
        if self.residual_skip_idx is not None:
            return x[:, -1, self.residual_skip_idx:self.residual_skip_idx+1]  # (B, 1)
        return None

    def _add_residual(self, out, base):
        """Add residual skip connection to output."""
        if base is not None:
            out = out + base
        return out

    def forward(self, x):
        # x: (B, T, D)
        if x.dim() == 2:
            x = x.unsqueeze(1)

        base = self._get_residual_base(x)

        if self.use_input_norm:
            if isinstance(self.input_norm, nn.InstanceNorm1d):
                x = x.transpose(1, 2)
                x = self.input_norm(x)
                x = x.transpose(1, 2)
            else:
                x = self.input_norm(x)

        ctx = self.enc(x)  # (B, C)
        feat = self.head_base(ctx)  # (B, Hidden)
        out = self.head_out(feat)  # (B, H*out)

        if self.horizon > 1:
            out = out.view(-1, self.horizon, self.output_dim)

        return self._add_residual(out, base)


class StanceGatedTCN(TCN_MLP):
    def __init__(self, *args, **kwargs):
        gating_dim = kwargs.pop('gating_dim', 1)
        super().__init__(*args, **kwargs)

        if 'input_dim' in kwargs:
            input_dim = kwargs['input_dim']
        elif args:
            input_dim = args[0]
        else:
            raise ValueError("StanceGatedTCN: 'input_dim' must be provided in args or kwargs.")

        self.gating_net = nn.Sequential(
            nn.Linear(input_dim, 16),
            nn.ReLU(),
            nn.Linear(16, input_dim),
            nn.Sigmoid()
        )

    def forward(self, x):
        if x.dim() == 2:
            x = x.unsqueeze(1)

        base = self._get_residual_base(x)

        gate = self.gating_net(x)  # (B, T, D)
        x_gated = x * gate

        # Call parent forward but skip its residual (we handle it here)
        if self.use_input_norm:
            if isinstance(self.input_norm, nn.InstanceNorm1d):
                x_gated = x_gated.transpose(1, 2)
                x_gated = self.input_norm(x_gated)
                x_gated = x_gated.transpose(1, 2)
            else:
                x_gated = self.input_norm(x_gated)

        ctx = self.enc(x_gated)
        feat = self.head_base(ctx)
        out = self.head_out(feat)

        if self.horizon > 1:
            out = out.view(-1, self.horizon, self.output_dim)

        return self._add_residual(out, base)


class AttentionTCN(TCN_MLP):
    def __init__(self, *args, **kwargs):
        atten_type = kwargs.pop('attention_type', 'temporal')
        heads = kwargs.pop('attention_heads', 4)
        super().__init__(*args, **kwargs)

        self.atten_w = nn.Linear(self.enc.out_ch, 1)

    def forward(self, x):
        if x.dim() == 2:
            x = x.unsqueeze(1)

        base = self._get_residual_base(x)

        if self.use_input_norm:
            x = self.input_norm(x)

        x_t = x.transpose(1, 2)  # (B, C, T)
        enc_seq = self.enc.network(x_t)  # (B, C_out, T)

        seq = enc_seq.transpose(1, 2)  # (B, T, C)
        scores = self.atten_w(seq)  # (B, T, 1)
        weights = F.softmax(scores, dim=1)  # (B, T, 1)

        ctx = torch.sum(seq * weights, dim=1)  # (B, C)

        feat = self.head_base(ctx)
        out = self.head_out(feat)

        if self.horizon > 1:
            out = out.view(-1, self.horizon, self.output_dim)

        return self._add_residual(out, base)


class TCN_GRU_Head(TCN_MLP):
    """TCN encoder + GRU head (replaces MLP head)."""
    def __init__(self, *args, gru_hidden=32, **kwargs):
        super().__init__(*args, **kwargs)
        enc_out = self.enc.out_ch
        self.gru = nn.GRU(enc_out, gru_hidden, batch_first=True)
        self.head_out = nn.Linear(gru_hidden, self.horizon * self.output_dim)
        self.head_base = nn.Identity()
        self._gru_hidden = gru_hidden

    def forward(self, x):
        if x.dim() == 2:
            x = x.unsqueeze(1)

        base = self._get_residual_base(x)

        if self.use_input_norm:
            if isinstance(self.input_norm, nn.InstanceNorm1d):
                x = x.transpose(1, 2)
                x = self.input_norm(x)
                x = x.transpose(1, 2)
            else:
                x = self.input_norm(x)
        x_t = x.transpose(1, 2)  # (B, D, T)
        enc_seq = self.enc.network(x_t)  # (B, C, T)
        enc_seq = enc_seq.transpose(1, 2)  # (B, T, C)
        gru_out, _ = self.gru(enc_seq)  # (B, T, gru_hidden)
        ctx = gru_out[:, -1, :]  # Last step: (B, gru_hidden)
        out = self.head_out(ctx)  # (B, H*out)
        if self.horizon > 1:
            out = out.view(-1, self.horizon, self.output_dim)
        return self._add_residual(out, base)


# =========================================================================
# archive_round5 — New Model Variants
# =========================================================================

class CausalSmoothTCN_GRU(TCN_GRU_Head):
    """TCN_GRU + learnable causal smoothing layer on output.

    A 1-D depthwise convolution (kernel_size = smooth_window) is applied
    *after* the GRU output, initialized as a uniform moving-average filter.
    During training the kernel adapts to learn the optimal causal smoothing.
    """
    def __init__(self, *args, smooth_window: int = 11, **kwargs):
        super().__init__(*args, **kwargs)
        self.smooth_window = smooth_window
        # Causal 1-D conv on GRU hidden → GRU hidden (per-channel)
        self._smooth_conv = nn.Conv1d(
            self._gru_hidden, self._gru_hidden,
            kernel_size=smooth_window,
            padding=smooth_window - 1,  # causal padding (trim after)
            groups=self._gru_hidden,    # depthwise
            bias=False,
        )
        # Init as uniform average
        nn.init.constant_(self._smooth_conv.weight, 1.0 / smooth_window)

    def forward(self, x):
        if x.dim() == 2:
            x = x.unsqueeze(1)

        base = self._get_residual_base(x)

        if self.use_input_norm:
            if isinstance(self.input_norm, nn.InstanceNorm1d):
                x = x.transpose(1, 2)
                x = self.input_norm(x)
                x = x.transpose(1, 2)
            else:
                x = self.input_norm(x)

        x_t = x.transpose(1, 2)
        enc_seq = self.enc.network(x_t)
        enc_seq = enc_seq.transpose(1, 2)
        gru_out, _ = self.gru(enc_seq)  # (B, T, H_gru)

        # Apply causal smoothing: (B, H_gru, T) -> conv -> trim -> (B, T, H_gru)
        smoothed = self._smooth_conv(gru_out.transpose(1, 2))
        smoothed = smoothed[:, :, :gru_out.size(1)]  # causal: remove future
        smoothed = smoothed.transpose(1, 2)

        ctx = smoothed[:, -1, :]
        out = self.head_out(ctx)
        if self.horizon > 1:
            out = out.view(-1, self.horizon, self.output_dim)
        return self._add_residual(out, base)


class MultiScaleTCN_GRU(TCN_MLP):
    """Parallel TCN encoders with different kernel sizes, fused before GRU.

    Three parallel TCN branches (kernel_size = 3, 5, 7) extract multi-scale
    temporal features.  Their outputs are concatenated and projected to a
    shared dimension before being fed to a GRU head.
    """
    def __init__(self, *args, gru_hidden: int = 32,
                 kernel_sizes=(3, 5, 7), **kwargs):
        # Build parent for input_norm / horizon / output_dim management
        super().__init__(*args, **kwargs)

        input_dim = args[0] if args else kwargs['input_dim']
        channels = kwargs.get('channels', (32, 32, 64, 64, 128, 128))
        if isinstance(channels, list):
            channels = tuple(channels)
        dropout = kwargs.get('dropout', 0.1)
        tcn_norm = kwargs.get('tcn_norm', None)

        # Build parallel encoders
        self.branches = nn.ModuleList()
        for ks in kernel_sizes:
            enc = TCNEncoder(input_dim, channels, ks, dropout, norm_type=tcn_norm)
            self.branches.append(enc)

        fused_dim = channels[-1] * len(kernel_sizes)
        self.fusion = nn.Linear(fused_dim, channels[-1])

        self.gru = nn.GRU(channels[-1], gru_hidden, batch_first=True)
        self.head_out = nn.Linear(gru_hidden, self.horizon * self.output_dim)
        self.head_base = nn.Identity()

    def forward(self, x):
        if x.dim() == 2:
            x = x.unsqueeze(1)

        base = self._get_residual_base(x)

        if self.use_input_norm:
            if isinstance(self.input_norm, nn.InstanceNorm1d):
                x = x.transpose(1, 2)
                x = self.input_norm(x)
                x = x.transpose(1, 2)
            else:
                x = self.input_norm(x)

        x_t = x.transpose(1, 2)  # (B, D, T)

        branch_outs = []
        for enc in self.branches:
            enc_seq = enc.network(x_t)  # (B, C, T)
            branch_outs.append(enc_seq)

        # Concatenate along channel dim: (B, C*N_branches, T)
        fused = torch.cat(branch_outs, dim=1)
        fused = fused.transpose(1, 2)  # (B, T, C*N)
        fused = F.relu(self.fusion(fused))  # (B, T, C)

        gru_out, _ = self.gru(fused)
        ctx = gru_out[:, -1, :]
        out = self.head_out(ctx)
        if self.horizon > 1:
            out = out.view(-1, self.horizon, self.output_dim)
        return self._add_residual(out, base)


class DualBranchTCN_GRU(TCN_MLP):
    """Lightweight late-fusion model for realtime robot inference.

    Branch 0 is intended for trunk/body-frame IMU features and branch 1 for
    exoskeleton kinematics/torque features. Each branch has its own TCN encoder.
    The fused sequence is then processed by a compact GRU head.
    """
    def __init__(self, *args, gru_hidden: int = 32, branch_input_dims=None,
                 branch_channels=None, **kwargs):
        super().__init__(*args, **kwargs)

        input_dim = args[0] if args else kwargs["input_dim"]
        channels = kwargs.get("channels", (32, 32, 64, 64))
        if isinstance(channels, list):
            channels = tuple(channels)
        dropout = kwargs.get("dropout", 0.1)
        tcn_norm = kwargs.get("tcn_norm", None)

        if not branch_input_dims:
            raise ValueError("DualBranchTCN_GRU requires branch_input_dims")
        branch_input_dims = tuple(int(v) for v in branch_input_dims)
        if sum(branch_input_dims) != input_dim:
            raise ValueError(
                f"branch_input_dims={branch_input_dims} do not sum to input_dim={input_dim}"
            )

        if branch_channels is None:
            branch_channels = channels
        if isinstance(branch_channels, list):
            branch_channels = tuple(branch_channels)

        self.branch_input_dims = branch_input_dims
        self.branch_imu = TCNEncoder(branch_input_dims[0], branch_channels, kwargs.get("kernel_size", 3), dropout, norm_type=tcn_norm)
        self.branch_robot = TCNEncoder(branch_input_dims[1], branch_channels, kwargs.get("kernel_size", 3), dropout, norm_type=tcn_norm)

        branch_out = branch_channels[-1]
        self.fusion = nn.Linear(branch_out * 2, branch_out)
        self.gru = nn.GRU(branch_out, gru_hidden, batch_first=True)
        self.head_out = nn.Linear(gru_hidden, self.horizon * self.output_dim)
        self.head_base = nn.Identity()

    def forward(self, x):
        if x.dim() == 2:
            x = x.unsqueeze(1)

        base = self._get_residual_base(x)

        if self.use_input_norm:
            if isinstance(self.input_norm, nn.InstanceNorm1d):
                x = x.transpose(1, 2)
                x = self.input_norm(x)
                x = x.transpose(1, 2)
            else:
                x = self.input_norm(x)

        d0 = self.branch_input_dims[0]
        x_imu = x[:, :, :d0]
        x_robot = x[:, :, d0:]

        seq_imu = self.branch_imu.network(x_imu.transpose(1, 2)).transpose(1, 2)
        seq_robot = self.branch_robot.network(x_robot.transpose(1, 2)).transpose(1, 2)
        fused = torch.cat([seq_imu, seq_robot], dim=-1)
        fused = F.relu(self.fusion(fused))

        gru_out, _ = self.gru(fused)
        ctx = gru_out[:, -1, :]
        out = self.head_out(ctx)
        if self.horizon > 1:
            out = out.view(-1, self.horizon, self.output_dim)
        return self._add_residual(out, base)


class MultiBranchTCN_GRU(TCN_MLP):
    """N-branch late-fusion TCN+GRU for heterogeneous realtime sensor groups."""
    def __init__(self, *args, gru_hidden: int = 32, branch_input_dims=None,
                 branch_channels=None, **kwargs):
        super().__init__(*args, **kwargs)

        input_dim = args[0] if args else kwargs["input_dim"]
        channels = kwargs.get("channels", (32, 32, 64, 64))
        if isinstance(channels, list):
            channels = tuple(channels)
        dropout = kwargs.get("dropout", 0.1)
        tcn_norm = kwargs.get("tcn_norm", None)

        if not branch_input_dims or len(branch_input_dims) < 2:
            raise ValueError("MultiBranchTCN_GRU requires at least two branch_input_dims")
        branch_input_dims = tuple(int(v) for v in branch_input_dims)
        if sum(branch_input_dims) != input_dim:
            raise ValueError(
                f"branch_input_dims={branch_input_dims} do not sum to input_dim={input_dim}"
            )

        if branch_channels is None:
            branch_channels = channels
        if isinstance(branch_channels, list):
            branch_channels = tuple(branch_channels)

        self.branch_input_dims = branch_input_dims
        self.branches = nn.ModuleList([
            TCNEncoder(dim, branch_channels, kwargs.get("kernel_size", 3), dropout, norm_type=tcn_norm)
            for dim in branch_input_dims
        ])

        branch_out = branch_channels[-1]
        self.fusion = nn.Linear(branch_out * len(branch_input_dims), branch_out)
        self.gru = nn.GRU(branch_out, gru_hidden, batch_first=True)
        self.head_out = nn.Linear(gru_hidden, self.horizon * self.output_dim)
        self.head_base = nn.Identity()

    def forward(self, x):
        if x.dim() == 2:
            x = x.unsqueeze(1)

        base = self._get_residual_base(x)

        if self.use_input_norm:
            if isinstance(self.input_norm, nn.InstanceNorm1d):
                x = x.transpose(1, 2)
                x = self.input_norm(x)
                x = x.transpose(1, 2)
            else:
                x = self.input_norm(x)

        seqs = []
        start = 0
        for dim, branch in zip(self.branch_input_dims, self.branches):
            x_part = x[:, :, start:start + dim]
            seqs.append(branch.network(x_part.transpose(1, 2)).transpose(1, 2))
            start += dim

        fused = torch.cat(seqs, dim=-1)
        fused = F.relu(self.fusion(fused))
        gru_out, _ = self.gru(fused)
        ctx = gru_out[:, -1, :]
        out = self.head_out(ctx)
        if self.horizon > 1:
            out = out.view(-1, self.horizon, self.output_dim)
        return self._add_residual(out, base)


class _KinematicsConditionBase(TCN_MLP):
    """Shared base for kinematics-first models conditioned by torque-context."""
    def __init__(self, *args, gru_hidden: int = 32, branch_input_dims=None,
                 branch_channels=None, **kwargs):
        super().__init__(*args, **kwargs)

        input_dim = args[0] if args else kwargs["input_dim"]
        channels = kwargs.get("channels", (32, 32, 64, 64))
        if isinstance(channels, list):
            channels = tuple(channels)
        dropout = kwargs.get("dropout", 0.1)
        tcn_norm = kwargs.get("tcn_norm", None)
        if not branch_input_dims or len(branch_input_dims) != 2:
            raise ValueError("KinematicsCondition models require branch_input_dims=[main, ctx]")
        branch_input_dims = tuple(int(v) for v in branch_input_dims)
        if sum(branch_input_dims) != input_dim:
            raise ValueError(
                f"branch_input_dims={branch_input_dims} do not sum to input_dim={input_dim}"
            )

        if branch_channels is None:
            branch_channels = channels
        if isinstance(branch_channels, list):
            branch_channels = tuple(branch_channels)

        self.branch_input_dims = branch_input_dims
        self.main_encoder = TCNEncoder(branch_input_dims[0], branch_channels, kwargs.get("kernel_size", 3), dropout, norm_type=tcn_norm)
        self.ctx_encoder = TCNEncoder(branch_input_dims[1], branch_channels, kwargs.get("kernel_size", 3), dropout, norm_type=tcn_norm)
        branch_out = branch_channels[-1]
        self.main_gru = nn.GRU(branch_out, gru_hidden, batch_first=True)
        self.ctx_gru = nn.GRU(branch_out, gru_hidden, batch_first=True)
        self.base_head = nn.Linear(gru_hidden, self.horizon * self.output_dim)
        self.head_base = nn.Identity()
        self._ctx_hidden = gru_hidden
        self._main_hidden = gru_hidden

    def _split_inputs(self, x):
        d0 = self.branch_input_dims[0]
        return x[:, :, :d0], x[:, :, d0:]

    def _encode(self, x):
        if x.dim() == 2:
            x = x.unsqueeze(1)
        base = self._get_residual_base(x)

        if self.use_input_norm:
            if isinstance(self.input_norm, nn.InstanceNorm1d):
                x = x.transpose(1, 2)
                x = self.input_norm(x)
                x = x.transpose(1, 2)
            else:
                x = self.input_norm(x)

        x_main, x_ctx = self._split_inputs(x)
        main_seq = self.main_encoder.network(x_main.transpose(1, 2)).transpose(1, 2)
        ctx_seq = self.ctx_encoder.network(x_ctx.transpose(1, 2)).transpose(1, 2)
        return base, main_seq, ctx_seq

    def _reshape_out(self, out):
        if self.horizon > 1:
            out = out.view(-1, self.horizon, self.output_dim)
        return out


class FiLMConditionedTCN_GRU(_KinematicsConditionBase):
    """Kinematics-first predictor, torque-context only modulates feature interpretation."""
    def __init__(self, *args, film_scale: float = 0.5, **kwargs):
        super().__init__(*args, **kwargs)
        out_ch = self.main_encoder.out_ch
        self.gamma = nn.Linear(self.ctx_encoder.out_ch, out_ch)
        self.beta = nn.Linear(self.ctx_encoder.out_ch, out_ch)
        self.film_scale = float(film_scale)

    def forward(self, x):
        base, main_seq, ctx_seq = self._encode(x)
        gamma = torch.tanh(self.gamma(ctx_seq)) * self.film_scale
        beta = torch.tanh(self.beta(ctx_seq)) * self.film_scale
        mod_seq = main_seq * (1.0 + gamma) + beta
        main_out, _ = self.main_gru(mod_seq)
        out = self.base_head(main_out[:, -1, :])
        if self.horizon > 1:
            out = out.view(-1, self.horizon, self.output_dim)
        return self._add_residual(out, base)


class BoundedResidualTCN_GRU(_KinematicsConditionBase):
    """Kinematics-first base speed plus bounded torque-driven residual correction."""
    def __init__(self, *args, residual_scale: float = 0.30, **kwargs):
        super().__init__(*args, **kwargs)
        self.ctx_head = nn.Linear(self._ctx_hidden, self.horizon * self.output_dim)
        self.residual_scale = float(residual_scale)

    def forward(self, x):
        base, main_seq, ctx_seq = self._encode(x)
        main_out, _ = self.main_gru(main_seq)
        ctx_out, _ = self.ctx_gru(ctx_seq)
        y_base = self.base_head(main_out[:, -1, :])
        residual = torch.tanh(self.ctx_head(ctx_out[:, -1, :])) * self.residual_scale
        out = y_base + residual
        out = self._reshape_out(out)
        return self._add_residual(out, base)


class AffineCalibTCN_GRU(_KinematicsConditionBase):
    """Kinematics-first base speed with torque-conditioned affine recalibration."""
    def __init__(self, *args, alpha_scale: float = 0.30, beta_scale: float = 0.25, **kwargs):
        super().__init__(*args, **kwargs)
        self.alpha_head = nn.Linear(self._ctx_hidden, self.horizon * self.output_dim)
        self.beta_head = nn.Linear(self._ctx_hidden, self.horizon * self.output_dim)
        self.alpha_scale = float(alpha_scale)
        self.beta_scale = float(beta_scale)

    def forward(self, x):
        base, main_seq, ctx_seq = self._encode(x)
        main_out, _ = self.main_gru(main_seq)
        ctx_out, _ = self.ctx_gru(ctx_seq)
        y_base = self.base_head(main_out[:, -1, :])
        alpha = 1.0 + torch.tanh(self.alpha_head(ctx_out[:, -1, :])) * self.alpha_scale
        beta = torch.tanh(self.beta_head(ctx_out[:, -1, :])) * self.beta_scale
        out = alpha * y_base + beta
        out = self._reshape_out(out)
        return self._add_residual(out, base)


class PositiveResidualTCN_GRU(_KinematicsConditionBase):
    """Kinematics-first predictor with torque-driven positive residual only."""
    def __init__(self, *args, residual_scale: float = 0.25, gate_scale: float = 1.0, **kwargs):
        super().__init__(*args, **kwargs)
        self.mag_head = nn.Linear(self._ctx_hidden, self.horizon * self.output_dim)
        self.gate_head = nn.Linear(self._ctx_hidden, self.horizon * self.output_dim)
        self.residual_scale = float(residual_scale)
        self.gate_scale = float(gate_scale)

    def forward(self, x):
        base, main_seq, ctx_seq = self._encode(x)
        main_out, _ = self.main_gru(main_seq)
        ctx_out, _ = self.ctx_gru(ctx_seq)
        y_base = self.base_head(main_out[:, -1, :])
        mag = F.softplus(self.mag_head(ctx_out[:, -1, :]))
        gate = torch.sigmoid(self.gate_head(ctx_out[:, -1, :]) * self.gate_scale)
        residual = gate * mag * self.residual_scale
        out = y_base + residual
        out = self._reshape_out(out)
        return self._add_residual(out, base)


class AssistExpertTCN_GRU(_KinematicsConditionBase):
    """Blend a kinematics-first base expert and an assisted expert using torque-context."""
    def __init__(self, *args, gate_max: float = 0.6, **kwargs):
        super().__init__(*args, **kwargs)
        self.assist_head = nn.Linear(self._main_hidden + self._ctx_hidden, self.horizon * self.output_dim)
        self.gate_head = nn.Linear(self._ctx_hidden, self.horizon * self.output_dim)
        self.gate_max = float(gate_max)

    def forward(self, x):
        base, main_seq, ctx_seq = self._encode(x)
        main_out, _ = self.main_gru(main_seq)
        ctx_out, _ = self.ctx_gru(ctx_seq)
        main_h = main_out[:, -1, :]
        ctx_h = ctx_out[:, -1, :]
        y_base = self.base_head(main_h)
        y_assist = self.assist_head(torch.cat([main_h, ctx_h], dim=-1))
        gate = torch.sigmoid(self.gate_head(ctx_h)) * self.gate_max
        out = y_base + gate * (y_assist - y_base)
        out = self._reshape_out(out)
        return self._add_residual(out, base)


class SlowFastCalibTCN_GRU(_KinematicsConditionBase):
    """Base + slow calibration + fast residual for assist-dependent domain shift.

    Main branch handles human kinematics / IMU dynamics.
    Context branch handles slower assist context such as torque envelopes, deflection,
    subject info, speed priors, and guarded preview features.
    """
    def __init__(
        self,
        *args,
        bias_scale: float = 0.20,
        residual_scale: float = 0.12,
        **kwargs,
    ):
        super().__init__(*args, **kwargs)
        self.bias_scale = float(bias_scale)
        self.residual_scale = float(residual_scale)
        self.slow_bias_head = nn.Linear(self.ctx_encoder.out_ch, self.horizon * self.output_dim)
        self.fast_residual_head = nn.Linear(
            self._main_hidden + self._ctx_hidden, self.horizon * self.output_dim
        )

    def forward(self, x):
        base, main_seq, ctx_seq = self._encode(x)
        main_out, _ = self.main_gru(main_seq)
        ctx_out, _ = self.ctx_gru(ctx_seq)

        main_h = main_out[:, -1, :]
        ctx_h = ctx_out[:, -1, :]
        slow_ctx = ctx_seq.mean(dim=1)

        y_base = self.base_head(main_h)
        slow_bias = torch.tanh(self.slow_bias_head(slow_ctx)) * self.bias_scale
        fast_residual = torch.tanh(
            self.fast_residual_head(torch.cat([main_h, ctx_h], dim=-1))
        ) * self.residual_scale

        out = y_base + slow_bias + fast_residual
        out = self._reshape_out(out)
        return self._add_residual(out, base)


class StructuredSlowFastTCN_GRU(_KinematicsConditionBase):
    """Structured two-timescale estimator for assisted-condition calibration.

    Fast branch estimates nominal locomotion dynamics from IMU/kinematics.
    Slow branch estimates assist-related calibration drift from causal robot
    summaries. Optional auxiliary and subject-calibration heads can be enabled
    without changing the main deploy path.
    """
    def __init__(
        self,
        *args,
        bias_scale: float = 0.18,
        residual_scale: float = 0.10,
        use_aux_head: bool = False,
        aux_dim: int = 1,
        use_subject_calibration: bool = False,
        subject_ctx_dims: int = 0,
        alpha_scale: float = 0.08,
        beta_scale: float = 0.05,
        **kwargs,
    ):
        super().__init__(*args, **kwargs)
        self.bias_scale = float(bias_scale)
        self.residual_scale = float(residual_scale)
        self.use_aux_head = bool(use_aux_head)
        self.aux_dim = int(aux_dim)
        self.use_subject_calibration = bool(use_subject_calibration)
        self.subject_ctx_dims = int(subject_ctx_dims)
        self.alpha_scale = float(alpha_scale)
        self.beta_scale = float(beta_scale)

        self.slow_bias_head = nn.Linear(self.ctx_encoder.out_ch, self.horizon * self.output_dim)
        self.fast_residual_head = nn.Linear(
            self._main_hidden + self._ctx_hidden, self.horizon * self.output_dim
        )

        if self.use_aux_head:
            self.aux_head = nn.Sequential(
                nn.Linear(self._ctx_hidden, max(16, self._ctx_hidden // 2)),
                nn.ReLU(inplace=True),
                nn.Linear(max(16, self._ctx_hidden // 2), self.aux_dim),
            )

        if self.use_subject_calibration:
            if self.subject_ctx_dims <= 0 or self.subject_ctx_dims > self.branch_input_dims[1]:
                raise ValueError(
                    "StructuredSlowFastTCN_GRU subject calibration requires "
                    "0 < subject_ctx_dims <= slow/context branch input dim"
                )
            calib_in = self._ctx_hidden + self.subject_ctx_dims
            self.subj_alpha_head = nn.Linear(calib_in, self.horizon * self.output_dim)
            self.subj_beta_head = nn.Linear(calib_in, self.horizon * self.output_dim)

    def forward(self, x):
        base, main_seq, ctx_seq = self._encode(x)
        main_out, _ = self.main_gru(main_seq)
        ctx_out, _ = self.ctx_gru(ctx_seq)

        main_h = main_out[:, -1, :]
        ctx_h = ctx_out[:, -1, :]
        slow_ctx = ctx_seq.mean(dim=1)

        nominal = self.base_head(main_h)
        slow_bias = torch.tanh(self.slow_bias_head(slow_ctx)) * self.bias_scale
        fast_residual = torch.tanh(
            self.fast_residual_head(torch.cat([main_h, ctx_h], dim=-1))
        ) * self.residual_scale

        pred = nominal + slow_bias + fast_residual

        if self.use_subject_calibration:
            _, x_ctx = self._split_inputs(x if x.dim() == 3 else x.unsqueeze(1))
            subj_ctx = x_ctx[:, :, -self.subject_ctx_dims:].mean(dim=1)
            calib_in = torch.cat([ctx_h, subj_ctx], dim=-1)
            alpha = 1.0 + torch.tanh(self.subj_alpha_head(calib_in)) * self.alpha_scale
            beta = torch.tanh(self.subj_beta_head(calib_in)) * self.beta_scale
            pred = alpha * pred + beta
        else:
            alpha = beta = None

        pred = self._reshape_out(pred)
        pred = self._add_residual(pred, base)

        if not (self.use_aux_head or self.use_subject_calibration):
            return {
                "pred": pred,
                "nominal_component": self._reshape_out(nominal),
                "slow_bias": self._reshape_out(slow_bias),
                "fast_residual": self._reshape_out(fast_residual),
            }

        ret = {
            "pred": pred,
            "nominal_component": self._reshape_out(nominal),
            "slow_bias": self._reshape_out(slow_bias),
            "fast_residual": self._reshape_out(fast_residual),
        }
        if self.use_aux_head:
            ret["assist_aux"] = self.aux_head(ctx_h)
        if self.use_subject_calibration:
            ret["subject_alpha"] = self._reshape_out(alpha)
            ret["subject_beta"] = self._reshape_out(beta)
        return ret


class AdaptiveComplementaryTCN_GRU(TCN_MLP):
    """Fast nominal + absolute-speed anchor + slow assist calibration.

    Branch 0: fast inertial / kinematic process
    Branch 1: absolute-speed anchor features (cadence, excursion, trunk pitch, geometry)
    Branch 2: slow assist/context summaries
    """
    def __init__(
        self,
        *args,
        gru_hidden: int = 32,
        branch_input_dims=None,
        branch_channels=None,
        bias_scale: float = 0.18,
        residual_scale: float = 0.08,
        gate_temperature: float = 1.0,
        **kwargs,
    ):
        super().__init__(*args, **kwargs)
        input_dim = args[0] if args else kwargs["input_dim"]
        channels = kwargs.get("channels", (32, 32, 64, 64))
        if isinstance(channels, list):
            channels = tuple(channels)
        dropout = kwargs.get("dropout", 0.1)
        tcn_norm = kwargs.get("tcn_norm", None)
        if not branch_input_dims or len(branch_input_dims) != 3:
            raise ValueError("AdaptiveComplementaryTCN_GRU requires branch_input_dims=[fast, anchor, slow]")
        branch_input_dims = tuple(int(v) for v in branch_input_dims)
        if sum(branch_input_dims) != input_dim:
            raise ValueError(f"branch_input_dims={branch_input_dims} do not sum to input_dim={input_dim}")
        if branch_channels is None:
            branch_channels = channels
        if isinstance(branch_channels, list):
            branch_channels = tuple(branch_channels)

        self.branch_input_dims = branch_input_dims
        self.fast_encoder = TCNEncoder(branch_input_dims[0], branch_channels, kwargs.get("kernel_size", 3), dropout, norm_type=tcn_norm)
        self.anchor_encoder = TCNEncoder(branch_input_dims[1], branch_channels, kwargs.get("kernel_size", 3), dropout, norm_type=tcn_norm)
        self.slow_encoder = TCNEncoder(branch_input_dims[2], branch_channels, kwargs.get("kernel_size", 3), dropout, norm_type=tcn_norm)

        branch_out = branch_channels[-1]
        self.fast_gru = nn.GRU(branch_out, gru_hidden, batch_first=True)
        self.anchor_gru = nn.GRU(branch_out, gru_hidden, batch_first=True)
        self.slow_gru = nn.GRU(branch_out, gru_hidden, batch_first=True)
        self.fast_head = nn.Linear(gru_hidden, self.horizon * self.output_dim)
        self.anchor_head = nn.Linear(gru_hidden, self.horizon * self.output_dim)
        self.gate_head = nn.Linear(gru_hidden * 3, self.horizon * self.output_dim)
        self.slow_bias_head = nn.Linear(gru_hidden, self.horizon * self.output_dim)
        self.fast_residual_head = nn.Linear(gru_hidden * 2, self.horizon * self.output_dim)
        self.bias_scale = float(bias_scale)
        self.residual_scale = float(residual_scale)
        self.gate_temperature = float(gate_temperature)
        self.head_base = nn.Identity()

    def _split(self, x):
        d0, d1, d2 = self.branch_input_dims
        x0 = x[:, :, :d0]
        x1 = x[:, :, d0:d0 + d1]
        x2 = x[:, :, d0 + d1:d0 + d1 + d2]
        return x0, x1, x2

    def forward(self, x):
        if x.dim() == 2:
            x = x.unsqueeze(1)
        base = self._get_residual_base(x)
        if self.use_input_norm:
            if isinstance(self.input_norm, nn.InstanceNorm1d):
                x = x.transpose(1, 2)
                x = self.input_norm(x)
                x = x.transpose(1, 2)
            else:
                x = self.input_norm(x)

        x_fast, x_anchor, x_slow = self._split(x)
        fast_seq = self.fast_encoder.network(x_fast.transpose(1, 2)).transpose(1, 2)
        anchor_seq = self.anchor_encoder.network(x_anchor.transpose(1, 2)).transpose(1, 2)
        slow_seq = self.slow_encoder.network(x_slow.transpose(1, 2)).transpose(1, 2)

        fast_h = self.fast_gru(fast_seq)[0][:, -1, :]
        anchor_h = self.anchor_gru(anchor_seq)[0][:, -1, :]
        slow_h = self.slow_gru(slow_seq)[0][:, -1, :]

        y_fast = self.fast_head(fast_h)
        y_anchor = self.anchor_head(anchor_h)
        gate = torch.sigmoid(self.gate_head(torch.cat([fast_h, anchor_h, slow_h], dim=-1)) / max(self.gate_temperature, 1e-6))
        slow_bias = torch.tanh(self.slow_bias_head(slow_h)) * self.bias_scale
        fast_residual = torch.tanh(self.fast_residual_head(torch.cat([fast_h, anchor_h], dim=-1))) * self.residual_scale

        pred = (1.0 - gate) * y_fast + gate * y_anchor + slow_bias + fast_residual
        pred = self._reshape_maybe(pred)
        pred = self._add_residual(pred, base)
        return {
            "pred": pred,
            "fast_component": self._reshape_maybe(y_fast),
            "anchor_component": self._reshape_maybe(y_anchor),
            "slow_bias": self._reshape_maybe(slow_bias),
            "fast_residual": self._reshape_maybe(fast_residual),
            "anchor_gate": self._reshape_maybe(gate),
        }

    def _reshape_maybe(self, out):
        if self.horizon > 1:
            out = out.view(-1, self.horizon, self.output_dim)
        return out


class CadenceStepLengthTCN_GRU(TCN_MLP):
    """Robot+trunk-only cadence × step-length decomposition baseline."""
    def __init__(
        self,
        *args,
        gru_hidden: int = 32,
        branch_input_dims=None,
        branch_channels=None,
        residual_scale: float = 0.05,
        **kwargs,
    ):
        super().__init__(*args, **kwargs)
        input_dim = args[0] if args else kwargs["input_dim"]
        channels = kwargs.get("channels", (32, 32, 64, 64))
        if isinstance(channels, list):
            channels = tuple(channels)
        dropout = kwargs.get("dropout", 0.1)
        tcn_norm = kwargs.get("tcn_norm", None)
        if not branch_input_dims or len(branch_input_dims) != 2:
            raise ValueError("CadenceStepLengthTCN_GRU requires branch_input_dims=[fast, anchor]")
        branch_input_dims = tuple(int(v) for v in branch_input_dims)
        if sum(branch_input_dims) != input_dim:
            raise ValueError(f"branch_input_dims={branch_input_dims} do not sum to input_dim={input_dim}")
        if branch_channels is None:
            branch_channels = channels
        if isinstance(branch_channels, list):
            branch_channels = tuple(branch_channels)
        self.branch_input_dims = branch_input_dims
        self.fast_encoder = TCNEncoder(branch_input_dims[0], branch_channels, kwargs.get("kernel_size", 3), dropout, norm_type=tcn_norm)
        self.anchor_encoder = TCNEncoder(branch_input_dims[1], branch_channels, kwargs.get("kernel_size", 3), dropout, norm_type=tcn_norm)
        branch_out = branch_channels[-1]
        self.fast_gru = nn.GRU(branch_out, gru_hidden, batch_first=True)
        self.anchor_gru = nn.GRU(branch_out, gru_hidden, batch_first=True)
        self.cadence_head = nn.Linear(gru_hidden, self.horizon * self.output_dim)
        self.step_head = nn.Linear(gru_hidden * 2, self.horizon * self.output_dim)
        self.residual_head = nn.Linear(gru_hidden * 2, self.horizon * self.output_dim)
        self.residual_scale = float(residual_scale)
        self.head_base = nn.Identity()

    def forward(self, x):
        if x.dim() == 2:
            x = x.unsqueeze(1)
        base = self._get_residual_base(x)
        if self.use_input_norm:
            if isinstance(self.input_norm, nn.InstanceNorm1d):
                x = x.transpose(1, 2)
                x = self.input_norm(x)
                x = x.transpose(1, 2)
            else:
                x = self.input_norm(x)
        d0 = self.branch_input_dims[0]
        x_fast = x[:, :, :d0]
        x_anchor = x[:, :, d0:]
        fast_seq = self.fast_encoder.network(x_fast.transpose(1, 2)).transpose(1, 2)
        anchor_seq = self.anchor_encoder.network(x_anchor.transpose(1, 2)).transpose(1, 2)
        fast_h = self.fast_gru(fast_seq)[0][:, -1, :]
        anchor_h = self.anchor_gru(anchor_seq)[0][:, -1, :]
        cadence = F.softplus(self.cadence_head(anchor_h))
        step_length = F.softplus(self.step_head(torch.cat([fast_h, anchor_h], dim=-1)))
        residual = torch.tanh(self.residual_head(torch.cat([fast_h, anchor_h], dim=-1))) * self.residual_scale
        pred = cadence * step_length + residual
        if self.horizon > 1:
            pred = pred.view(-1, self.horizon, self.output_dim)
            cadence = cadence.view(-1, self.horizon, self.output_dim)
            step_length = step_length.view(-1, self.horizon, self.output_dim)
            residual = residual.view(-1, self.horizon, self.output_dim)
        pred = self._add_residual(pred, base)
        return {
            "pred": pred,
            "cadence_component": cadence,
            "step_length_component": step_length,
            "fast_residual": residual,
        }


class PersistentBiasTCN_GRU(AdaptiveComplementaryTCN_GRU):
    """Complementary estimator with an explicit persistent slow bias state."""
    def __init__(self, *args, state_leak: float = 0.92, **kwargs):
        super().__init__(*args, **kwargs)
        self.state_leak = float(state_leak)
        self.state_proj = nn.Linear(self.slow_gru.hidden_size, self.slow_gru.hidden_size)

    def forward(self, x):
        if x.dim() == 2:
            x = x.unsqueeze(1)
        base = self._get_residual_base(x)
        if self.use_input_norm:
            if isinstance(self.input_norm, nn.InstanceNorm1d):
                x = x.transpose(1, 2)
                x = self.input_norm(x)
                x = x.transpose(1, 2)
            else:
                x = self.input_norm(x)
        x_fast, x_anchor, x_slow = self._split(x)
        fast_seq = self.fast_encoder.network(x_fast.transpose(1, 2)).transpose(1, 2)
        anchor_seq = self.anchor_encoder.network(x_anchor.transpose(1, 2)).transpose(1, 2)
        slow_seq = self.slow_encoder.network(x_slow.transpose(1, 2)).transpose(1, 2)

        fast_h = self.fast_gru(fast_seq)[0][:, -1, :]
        anchor_h = self.anchor_gru(anchor_seq)[0][:, -1, :]
        slow_out = self.slow_gru(slow_seq)[0]
        persistent = slow_out[:, 0, :]
        for t in range(1, slow_out.shape[1]):
            persistent = self.state_leak * persistent + (1.0 - self.state_leak) * slow_out[:, t, :]
        persistent = torch.tanh(self.state_proj(persistent))

        y_fast = self.fast_head(fast_h)
        y_anchor = self.anchor_head(anchor_h)
        gate = torch.sigmoid(self.gate_head(torch.cat([fast_h, anchor_h, persistent], dim=-1)) / max(self.gate_temperature, 1e-6))
        slow_bias = torch.tanh(self.slow_bias_head(persistent)) * self.bias_scale
        fast_residual = torch.tanh(self.fast_residual_head(torch.cat([fast_h, anchor_h], dim=-1))) * self.residual_scale
        pred = (1.0 - gate) * y_fast + gate * y_anchor + slow_bias + fast_residual
        pred = self._reshape_maybe(pred)
        pred = self._add_residual(pred, base)
        return {
            "pred": pred,
            "fast_component": self._reshape_maybe(y_fast),
            "anchor_component": self._reshape_maybe(y_anchor),
            "slow_bias": self._reshape_maybe(slow_bias),
            "fast_residual": self._reshape_maybe(fast_residual),
            "anchor_gate": self._reshape_maybe(gate),
            "persistent_state": persistent,
        }


class SpeedModeResidualTCN_GRU(AdaptiveComplementaryTCN_GRU):
    """Classify coarse speed mode and add residual correction."""
    def __init__(self, *args, mode_centers=None, mode_residual_scale: float = 0.12, **kwargs):
        super().__init__(*args, residual_scale=0.0, **kwargs)
        if mode_centers is None:
            mode_centers = [0.75, 1.00, 1.25]
        self.register_buffer("mode_centers", torch.tensor(mode_centers, dtype=torch.float32).view(1, 1, -1))
        self.mode_head = nn.Linear(self.anchor_gru.hidden_size + self.slow_gru.hidden_size, len(mode_centers))
        self.mode_residual_head = nn.Linear(self.fast_gru.hidden_size + self.anchor_gru.hidden_size, self.horizon * self.output_dim)
        self.mode_residual_scale = float(mode_residual_scale)

    def forward(self, x):
        base, fast_seq, anchor_seq, slow_seq = None, None, None, None
        if x.dim() == 2:
            x = x.unsqueeze(1)
        base = self._get_residual_base(x)
        if self.use_input_norm:
            if isinstance(self.input_norm, nn.InstanceNorm1d):
                x = x.transpose(1, 2)
                x = self.input_norm(x)
                x = x.transpose(1, 2)
            else:
                x = self.input_norm(x)
        x_fast, x_anchor, x_slow = self._split(x)
        fast_seq = self.fast_encoder.network(x_fast.transpose(1, 2)).transpose(1, 2)
        anchor_seq = self.anchor_encoder.network(x_anchor.transpose(1, 2)).transpose(1, 2)
        slow_seq = self.slow_encoder.network(x_slow.transpose(1, 2)).transpose(1, 2)
        fast_h = self.fast_gru(fast_seq)[0][:, -1, :]
        anchor_h = self.anchor_gru(anchor_seq)[0][:, -1, :]
        slow_h = self.slow_gru(slow_seq)[0][:, -1, :]
        mode_logits = self.mode_head(torch.cat([anchor_h, slow_h], dim=-1))
        mode_probs = F.softmax(mode_logits, dim=-1)
        centers = self.mode_centers
        anchor = torch.sum(mode_probs.unsqueeze(1) * centers, dim=-1)
        residual = torch.tanh(self.mode_residual_head(torch.cat([fast_h, anchor_h], dim=-1))) * self.mode_residual_scale
        slow_bias = torch.tanh(self.slow_bias_head(slow_h)) * self.bias_scale
        pred = anchor + residual + slow_bias
        pred = self._reshape_maybe(pred)
        pred = self._add_residual(pred, base)
        return {
            "pred": pred,
            "speed_mode_logits": mode_logits,
            "anchor_component": self._reshape_maybe(anchor),
            "slow_bias": self._reshape_maybe(slow_bias),
            "fast_residual": self._reshape_maybe(residual),
        }


class ObserverStateTCN_GRU(TCN_MLP):
    """Observer-style latent speed state updated by fast/anchor measurements."""
    def __init__(
        self,
        *args,
        gru_hidden: int = 32,
        branch_input_dims=None,
        branch_channels=None,
        bias_scale: float = 0.12,
        residual_scale: float = 0.05,
        drift_scale: float = 0.06,
        update_stride: int = 8,
        **kwargs,
    ):
        super().__init__(*args, **kwargs)
        input_dim = args[0] if args else kwargs["input_dim"]
        channels = kwargs.get("channels", (32, 32, 64, 64))
        if isinstance(channels, list):
            channels = tuple(channels)
        dropout = kwargs.get("dropout", 0.1)
        tcn_norm = kwargs.get("tcn_norm", None)
        if not branch_input_dims or len(branch_input_dims) != 3:
            raise ValueError("ObserverStateTCN_GRU requires branch_input_dims=[fast, anchor, slow]")
        branch_input_dims = tuple(int(v) for v in branch_input_dims)
        if sum(branch_input_dims) != input_dim:
            raise ValueError(f"branch_input_dims={branch_input_dims} do not sum to input_dim={input_dim}")
        if branch_channels is None:
            branch_channels = channels
        if isinstance(branch_channels, list):
            branch_channels = tuple(branch_channels)

        self.branch_input_dims = branch_input_dims
        self.fast_encoder = TCNEncoder(branch_input_dims[0], branch_channels, kwargs.get("kernel_size", 3), dropout, norm_type=tcn_norm)
        self.anchor_encoder = TCNEncoder(branch_input_dims[1], branch_channels, kwargs.get("kernel_size", 3), dropout, norm_type=tcn_norm)
        self.slow_encoder = TCNEncoder(branch_input_dims[2], branch_channels, kwargs.get("kernel_size", 3), dropout, norm_type=tcn_norm)
        enc_dim = branch_channels[-1]
        self.state_dim = int(gru_hidden)
        self.update_stride = max(1, int(update_stride))
        self.bias_scale = float(bias_scale)
        self.residual_scale = float(residual_scale)
        self.drift_scale = float(drift_scale)

        self.fast_proj = nn.Linear(enc_dim, self.state_dim)
        self.anchor_proj = nn.Linear(enc_dim, self.state_dim)
        self.slow_proj = nn.Linear(enc_dim, self.state_dim)
        self.anchor_gain = nn.Linear(self.state_dim * 2, self.state_dim)
        self.fast_gain = nn.Linear(self.state_dim * 2, self.state_dim)
        self.drift_head = nn.Linear(self.state_dim, self.state_dim)
        self.state_head = nn.Linear(self.state_dim, self.horizon * self.output_dim)
        self.anchor_head = nn.Linear(self.state_dim, self.horizon * self.output_dim)
        self.bias_head = nn.Linear(self.state_dim, self.horizon * self.output_dim)
        self.residual_head = nn.Linear(self.state_dim * 2, self.horizon * self.output_dim)
        self.head_base = nn.Identity()

    def _split(self, x):
        d0, d1, d2 = self.branch_input_dims
        return x[:, :, :d0], x[:, :, d0:d0 + d1], x[:, :, d0 + d1:d0 + d1 + d2]

    def _reshape_maybe(self, out):
        if self.horizon > 1:
            return out.view(-1, self.horizon, self.output_dim)
        return out

    def _downsample_seq(self, seq):
        idx = torch.arange(self.update_stride - 1, seq.shape[1], self.update_stride, device=seq.device)
        if idx.numel() == 0:
            idx = torch.tensor([seq.shape[1] - 1], device=seq.device)
        return seq.index_select(1, idx)

    def forward(self, x):
        if x.dim() == 2:
            x = x.unsqueeze(1)
        base = self._get_residual_base(x)
        if self.use_input_norm:
            if isinstance(self.input_norm, nn.InstanceNorm1d):
                x = x.transpose(1, 2)
                x = self.input_norm(x)
                x = x.transpose(1, 2)
            else:
                x = self.input_norm(x)

        x_fast, x_anchor, x_slow = self._split(x)
        fast_seq = self.fast_encoder.network(x_fast.transpose(1, 2)).transpose(1, 2)
        anchor_seq = self.anchor_encoder.network(x_anchor.transpose(1, 2)).transpose(1, 2)
        slow_seq = self.slow_encoder.network(x_slow.transpose(1, 2)).transpose(1, 2)

        fast_ds = self._downsample_seq(fast_seq)
        anchor_ds = self._downsample_seq(anchor_seq)
        slow_ds = self._downsample_seq(slow_seq)
        steps = min(fast_ds.shape[1], anchor_ds.shape[1], slow_ds.shape[1])
        fast_ds = fast_ds[:, :steps, :]
        anchor_ds = anchor_ds[:, :steps, :]
        slow_ds = slow_ds[:, :steps, :]

        state = torch.tanh(self.anchor_proj(anchor_ds[:, 0, :]))
        fast_lat_last = state
        anchor_lat_last = state
        slow_lat_last = state
        for t in range(steps):
            fast_lat = torch.tanh(self.fast_proj(fast_ds[:, t, :]))
            anchor_lat = torch.tanh(self.anchor_proj(anchor_ds[:, t, :]))
            slow_lat = torch.tanh(self.slow_proj(slow_ds[:, t, :]))
            gain_anchor = torch.sigmoid(self.anchor_gain(torch.cat([state, slow_lat], dim=-1)))
            gain_fast = torch.sigmoid(self.fast_gain(torch.cat([state, slow_lat], dim=-1)))
            drift = torch.tanh(self.drift_head(slow_lat)) * self.drift_scale
            state = state + drift + gain_anchor * (anchor_lat - state) + gain_fast * (fast_lat - state)
            fast_lat_last = fast_lat
            anchor_lat_last = anchor_lat
            slow_lat_last = slow_lat

        nominal = self.state_head(state)
        anchor_component = self.anchor_head(anchor_lat_last)
        slow_bias = torch.tanh(self.bias_head(slow_lat_last)) * self.bias_scale
        fast_residual = torch.tanh(self.residual_head(torch.cat([state, fast_lat_last], dim=-1))) * self.residual_scale
        pred = nominal + slow_bias + fast_residual
        pred = self._reshape_maybe(pred)
        pred = self._add_residual(pred, base)
        return {
            "pred": pred,
            "nominal_component": self._reshape_maybe(nominal),
            "anchor_component": self._reshape_maybe(anchor_component),
            "slow_bias": self._reshape_maybe(slow_bias),
            "fast_residual": self._reshape_maybe(fast_residual),
            "observer_state": state,
        }


class FactorizedAnchorTCN_GRU(TCN_MLP):
    """Estimate speed via a factorized cadence × step-length anchor plus residual."""
    def __init__(
        self,
        *args,
        gru_hidden: int = 32,
        branch_input_dims=None,
        branch_channels=None,
        residual_scale: float = 0.06,
        **kwargs,
    ):
        super().__init__(*args, **kwargs)
        input_dim = args[0] if args else kwargs["input_dim"]
        channels = kwargs.get("channels", (32, 32, 64, 64))
        if isinstance(channels, list):
            channels = tuple(channels)
        dropout = kwargs.get("dropout", 0.1)
        tcn_norm = kwargs.get("tcn_norm", None)
        if not branch_input_dims or len(branch_input_dims) != 2:
            raise ValueError("FactorizedAnchorTCN_GRU requires branch_input_dims=[fast, anchor]")
        branch_input_dims = tuple(int(v) for v in branch_input_dims)
        if sum(branch_input_dims) != input_dim:
            raise ValueError(f"branch_input_dims={branch_input_dims} do not sum to input_dim={input_dim}")
        if branch_channels is None:
            branch_channels = channels
        if isinstance(branch_channels, list):
            branch_channels = tuple(branch_channels)

        self.branch_input_dims = branch_input_dims
        self.fast_encoder = TCNEncoder(branch_input_dims[0], branch_channels, kwargs.get("kernel_size", 3), dropout, norm_type=tcn_norm)
        self.anchor_encoder = TCNEncoder(branch_input_dims[1], branch_channels, kwargs.get("kernel_size", 3), dropout, norm_type=tcn_norm)
        enc_dim = branch_channels[-1]
        self.fast_gru = nn.GRU(enc_dim, gru_hidden, batch_first=True)
        self.anchor_gru = nn.GRU(enc_dim, gru_hidden, batch_first=True)
        self.log_cadence_head = nn.Linear(gru_hidden, self.horizon * self.output_dim)
        self.log_step_head = nn.Linear(gru_hidden * 2, self.horizon * self.output_dim)
        self.direct_speed_head = nn.Linear(gru_hidden * 2, self.horizon * self.output_dim)
        self.mix_gate_head = nn.Linear(gru_hidden * 2, self.horizon * self.output_dim)
        self.residual_head = nn.Linear(gru_hidden * 2, self.horizon * self.output_dim)
        self.residual_scale = float(residual_scale)
        self.head_base = nn.Identity()

    def _reshape_maybe(self, out):
        if self.horizon > 1:
            return out.view(-1, self.horizon, self.output_dim)
        return out

    def forward(self, x):
        if x.dim() == 2:
            x = x.unsqueeze(1)
        base = self._get_residual_base(x)
        if self.use_input_norm:
            if isinstance(self.input_norm, nn.InstanceNorm1d):
                x = x.transpose(1, 2)
                x = self.input_norm(x)
                x = x.transpose(1, 2)
            else:
                x = self.input_norm(x)

        d0 = self.branch_input_dims[0]
        x_fast = x[:, :, :d0]
        x_anchor = x[:, :, d0:]
        fast_seq = self.fast_encoder.network(x_fast.transpose(1, 2)).transpose(1, 2)
        anchor_seq = self.anchor_encoder.network(x_anchor.transpose(1, 2)).transpose(1, 2)
        fast_h = self.fast_gru(fast_seq)[0][:, -1, :]
        anchor_h = self.anchor_gru(anchor_seq)[0][:, -1, :]
        joint_h = torch.cat([fast_h, anchor_h], dim=-1)

        cadence = F.softplus(self.log_cadence_head(anchor_h))
        step_length = F.softplus(self.log_step_head(joint_h))
        factorized_speed = cadence * step_length
        direct_speed = F.softplus(self.direct_speed_head(joint_h))
        mix_gate = torch.sigmoid(self.mix_gate_head(joint_h))
        anchor_speed = mix_gate * factorized_speed + (1.0 - mix_gate) * direct_speed
        residual = torch.tanh(self.residual_head(joint_h)) * self.residual_scale
        pred = anchor_speed + residual
        pred = self._reshape_maybe(pred)
        pred = self._add_residual(pred, base)
        return {
            "pred": pred,
            "nominal_component": self._reshape_maybe(anchor_speed),
            "anchor_component": self._reshape_maybe(anchor_speed),
            "cadence_component": self._reshape_maybe(cadence),
            "step_length_component": self._reshape_maybe(step_length),
            "fast_residual": self._reshape_maybe(residual),
        }


class MemoryCalibratorTCN_GRU(TCN_MLP):
    """Nominal predictor with a causal memory calibrator over slow context."""
    def __init__(
        self,
        *args,
        gru_hidden: int = 32,
        branch_input_dims=None,
        branch_channels=None,
        bias_scale: float = 0.14,
        residual_scale: float = 0.04,
        calibrator_stride: int = 10,
        **kwargs,
    ):
        super().__init__(*args, **kwargs)
        input_dim = args[0] if args else kwargs["input_dim"]
        channels = kwargs.get("channels", (32, 32, 64, 64))
        if isinstance(channels, list):
            channels = tuple(channels)
        dropout = kwargs.get("dropout", 0.1)
        tcn_norm = kwargs.get("tcn_norm", None)
        if not branch_input_dims or len(branch_input_dims) != 3:
            raise ValueError("MemoryCalibratorTCN_GRU requires branch_input_dims=[fast, anchor, slow]")
        branch_input_dims = tuple(int(v) for v in branch_input_dims)
        if sum(branch_input_dims) != input_dim:
            raise ValueError(f"branch_input_dims={branch_input_dims} do not sum to input_dim={input_dim}")
        if branch_channels is None:
            branch_channels = channels
        if isinstance(branch_channels, list):
            branch_channels = tuple(branch_channels)

        self.branch_input_dims = branch_input_dims
        self.fast_encoder = TCNEncoder(branch_input_dims[0], branch_channels, kwargs.get("kernel_size", 3), dropout, norm_type=tcn_norm)
        self.anchor_encoder = TCNEncoder(branch_input_dims[1], branch_channels, kwargs.get("kernel_size", 3), dropout, norm_type=tcn_norm)
        self.slow_encoder = TCNEncoder(branch_input_dims[2], branch_channels, kwargs.get("kernel_size", 3), dropout, norm_type=tcn_norm)
        enc_dim = branch_channels[-1]
        self.fast_gru = nn.GRU(enc_dim, gru_hidden, batch_first=True)
        self.anchor_gru = nn.GRU(enc_dim, gru_hidden, batch_first=True)
        self.fast_head = nn.Linear(gru_hidden, self.horizon * self.output_dim)
        self.anchor_head = nn.Linear(gru_hidden, self.horizon * self.output_dim)
        self.gate_head = nn.Linear(gru_hidden * 2, self.horizon * self.output_dim)
        self.residual_head = nn.Linear(gru_hidden * 2, self.horizon * self.output_dim)
        self.calib_cell = nn.GRUCell(enc_dim * 3, gru_hidden)
        self.calib_bias_head = nn.Linear(gru_hidden, self.horizon * self.output_dim)
        self.calib_gate_head = nn.Linear(gru_hidden, self.horizon * self.output_dim)
        self.bias_scale = float(bias_scale)
        self.residual_scale = float(residual_scale)
        self.calibrator_stride = max(1, int(calibrator_stride))
        self.head_base = nn.Identity()

    def _split(self, x):
        d0, d1, d2 = self.branch_input_dims
        return x[:, :, :d0], x[:, :, d0:d0 + d1], x[:, :, d0 + d1:d0 + d1 + d2]

    def _reshape_maybe(self, out):
        if self.horizon > 1:
            return out.view(-1, self.horizon, self.output_dim)
        return out

    def _downsample_seq(self, seq):
        idx = torch.arange(self.calibrator_stride - 1, seq.shape[1], self.calibrator_stride, device=seq.device)
        if idx.numel() == 0:
            idx = torch.tensor([seq.shape[1] - 1], device=seq.device)
        return seq.index_select(1, idx)

    def forward(self, x):
        if x.dim() == 2:
            x = x.unsqueeze(1)
        base = self._get_residual_base(x)
        if self.use_input_norm:
            if isinstance(self.input_norm, nn.InstanceNorm1d):
                x = x.transpose(1, 2)
                x = self.input_norm(x)
                x = x.transpose(1, 2)
            else:
                x = self.input_norm(x)

        x_fast, x_anchor, x_slow = self._split(x)
        fast_seq = self.fast_encoder.network(x_fast.transpose(1, 2)).transpose(1, 2)
        anchor_seq = self.anchor_encoder.network(x_anchor.transpose(1, 2)).transpose(1, 2)
        slow_seq = self.slow_encoder.network(x_slow.transpose(1, 2)).transpose(1, 2)
        fast_h = self.fast_gru(fast_seq)[0][:, -1, :]
        anchor_h = self.anchor_gru(anchor_seq)[0][:, -1, :]

        y_fast = self.fast_head(fast_h)
        y_anchor = self.anchor_head(anchor_h)
        mix_gate = torch.sigmoid(self.gate_head(torch.cat([fast_h, anchor_h], dim=-1)))
        nominal = (1.0 - mix_gate) * y_fast + mix_gate * y_anchor

        fast_ds = self._downsample_seq(fast_seq)
        anchor_ds = self._downsample_seq(anchor_seq)
        slow_ds = self._downsample_seq(slow_seq)
        steps = min(fast_ds.shape[1], anchor_ds.shape[1], slow_ds.shape[1])
        calib_state = torch.zeros(fast_seq.shape[0], self.fast_gru.hidden_size, device=x.device, dtype=fast_seq.dtype)
        for t in range(steps):
            cell_in = torch.cat([fast_ds[:, t, :], anchor_ds[:, t, :], slow_ds[:, t, :]], dim=-1)
            calib_state = self.calib_cell(cell_in, calib_state)

        bias_gate = torch.sigmoid(self.calib_gate_head(calib_state))
        slow_bias = torch.tanh(self.calib_bias_head(calib_state)) * self.bias_scale * bias_gate
        fast_residual = torch.tanh(self.residual_head(torch.cat([fast_h, anchor_h], dim=-1))) * self.residual_scale
        pred = nominal + slow_bias + fast_residual
        pred = self._reshape_maybe(pred)
        pred = self._add_residual(pred, base)
        return {
            "pred": pred,
            "nominal_component": self._reshape_maybe(nominal),
            "anchor_component": self._reshape_maybe(y_anchor),
            "slow_bias": self._reshape_maybe(slow_bias),
            "fast_residual": self._reshape_maybe(fast_residual),
            "memory_state": calib_state,
        }


class PrivilegedKinMemoryCalibratorTCN_GRU(MemoryCalibratorTCN_GRU):
    """Memory calibrator with training-time auxiliary lower-limb kinematics supervision.

    The model predicts speed from allowed deploy inputs only, but it also predicts
    privileged hip/knee/ankle kinematics during training. The estimated
    kinematics are projected back into a small correction term so the speed path
    can benefit from the internal gait-state estimate without using mocap at
    inference time.
    """
    def __init__(self, *args, aux_dim: int = 6, kin_scale: float = 0.04, **kwargs):
        super().__init__(*args, **kwargs)
        hidden = self.fast_gru.hidden_size * 2 + self.calib_cell.hidden_size
        aux_hidden = max(32, hidden // 2)
        self.aux_dim = int(aux_dim)
        self.kin_scale = float(kin_scale)
        self.kin_head = nn.Sequential(
            nn.Linear(hidden, aux_hidden),
            nn.ReLU(inplace=True),
            nn.Linear(aux_hidden, self.horizon * self.aux_dim),
        )
        self.kin_corr_head = nn.Sequential(
            nn.Linear(self.horizon * self.aux_dim, max(16, self.aux_dim * 2)),
            nn.ReLU(inplace=True),
            nn.Linear(max(16, self.aux_dim * 2), self.horizon * self.output_dim),
        )

    def _reshape_aux(self, out):
        if self.horizon > 1:
            return out.view(-1, self.horizon, self.aux_dim)
        return out

    def forward(self, x):
        if x.dim() == 2:
            x = x.unsqueeze(1)
        base = self._get_residual_base(x)
        if self.use_input_norm:
            if isinstance(self.input_norm, nn.InstanceNorm1d):
                x = x.transpose(1, 2)
                x = self.input_norm(x)
                x = x.transpose(1, 2)
            else:
                x = self.input_norm(x)

        x_fast, x_anchor, x_slow = self._split(x)
        fast_seq = self.fast_encoder.network(x_fast.transpose(1, 2)).transpose(1, 2)
        anchor_seq = self.anchor_encoder.network(x_anchor.transpose(1, 2)).transpose(1, 2)
        slow_seq = self.slow_encoder.network(x_slow.transpose(1, 2)).transpose(1, 2)
        fast_h = self.fast_gru(fast_seq)[0][:, -1, :]
        anchor_h = self.anchor_gru(anchor_seq)[0][:, -1, :]

        y_fast = self.fast_head(fast_h)
        y_anchor = self.anchor_head(anchor_h)
        mix_gate = torch.sigmoid(self.gate_head(torch.cat([fast_h, anchor_h], dim=-1)))
        nominal = (1.0 - mix_gate) * y_fast + mix_gate * y_anchor

        fast_ds = self._downsample_seq(fast_seq)
        anchor_ds = self._downsample_seq(anchor_seq)
        slow_ds = self._downsample_seq(slow_seq)
        steps = min(fast_ds.shape[1], anchor_ds.shape[1], slow_ds.shape[1])
        calib_state = torch.zeros(fast_seq.shape[0], self.fast_gru.hidden_size, device=x.device, dtype=fast_seq.dtype)
        for t in range(steps):
            cell_in = torch.cat([fast_ds[:, t, :], anchor_ds[:, t, :], slow_ds[:, t, :]], dim=-1)
            calib_state = self.calib_cell(cell_in, calib_state)

        bias_gate = torch.sigmoid(self.calib_gate_head(calib_state))
        slow_bias = torch.tanh(self.calib_bias_head(calib_state)) * self.bias_scale * bias_gate
        fast_residual = torch.tanh(self.residual_head(torch.cat([fast_h, anchor_h], dim=-1))) * self.residual_scale

        kin_hidden = torch.cat([fast_h, anchor_h, calib_state], dim=-1)
        aux_kin_raw = self.kin_head(kin_hidden)
        kin_correction = torch.tanh(self.kin_corr_head(aux_kin_raw)) * self.kin_scale

        pred = nominal + slow_bias + fast_residual + kin_correction
        pred = self._reshape_maybe(pred)
        pred = self._add_residual(pred, base)
        return {
            "pred": pred,
            "nominal_component": self._reshape_maybe(nominal),
            "anchor_component": self._reshape_maybe(y_anchor),
            "slow_bias": self._reshape_maybe(slow_bias),
            "fast_residual": self._reshape_maybe(fast_residual + kin_correction),
            "memory_state": calib_state,
            "aux_kinematics": self._reshape_aux(aux_kin_raw),
        }


class AssistSwitchHeadTCN_GRU(AdaptiveComplementaryTCN_GRU):
    """Shared nominal backbone with assist off/on-specific correction heads."""
    def __init__(
        self,
        *args,
        on_scale: float = 0.12,
        off_scale: float = 0.05,
        hard_switch: bool = False,
        assist_state_index: int = -3,
        **kwargs,
    ):
        super().__init__(*args, **kwargs)
        self.off_bias_head = nn.Linear(self.slow_gru.hidden_size, self.horizon * self.output_dim)
        self.on_bias_head = nn.Linear(self.slow_gru.hidden_size, self.horizon * self.output_dim)
        self.off_residual_head = nn.Linear(self.fast_gru.hidden_size + self.anchor_gru.hidden_size, self.horizon * self.output_dim)
        self.on_residual_head = nn.Linear(self.fast_gru.hidden_size + self.anchor_gru.hidden_size, self.horizon * self.output_dim)
        self.on_scale = float(on_scale)
        self.off_scale = float(off_scale)
        self.hard_switch = bool(hard_switch)
        self.assist_state_index = int(assist_state_index)

    def _assist_state_from_slow_input(self, x_slow):
        idx = self.assist_state_index
        state = x_slow[:, :, idx].mean(dim=1, keepdim=True)
        if self.hard_switch:
            state = (state > 0.5).to(x_slow.dtype)
        return torch.clamp(state, 0.0, 1.0)

    def forward(self, x):
        if x.dim() == 2:
            x = x.unsqueeze(1)
        base = self._get_residual_base(x)
        if self.use_input_norm:
            if isinstance(self.input_norm, nn.InstanceNorm1d):
                x = x.transpose(1, 2)
                x = self.input_norm(x)
                x = x.transpose(1, 2)
            else:
                x = self.input_norm(x)
        x_fast, x_anchor, x_slow = self._split(x)
        fast_seq = self.fast_encoder.network(x_fast.transpose(1, 2)).transpose(1, 2)
        anchor_seq = self.anchor_encoder.network(x_anchor.transpose(1, 2)).transpose(1, 2)
        slow_seq = self.slow_encoder.network(x_slow.transpose(1, 2)).transpose(1, 2)
        fast_h = self.fast_gru(fast_seq)[0][:, -1, :]
        anchor_h = self.anchor_gru(anchor_seq)[0][:, -1, :]
        slow_h = self.slow_gru(slow_seq)[0][:, -1, :]

        y_fast = self.fast_head(fast_h)
        y_anchor = self.anchor_head(anchor_h)
        gate = torch.sigmoid(self.gate_head(torch.cat([fast_h, anchor_h, slow_h], dim=-1)) / max(self.gate_temperature, 1e-6))
        nominal = (1.0 - gate) * y_fast + gate * y_anchor

        off_bias = torch.tanh(self.off_bias_head(slow_h)) * self.off_scale
        on_bias = torch.tanh(self.on_bias_head(slow_h)) * self.on_scale
        off_res = torch.tanh(self.off_residual_head(torch.cat([fast_h, anchor_h], dim=-1))) * (self.residual_scale * 0.75)
        on_res = torch.tanh(self.on_residual_head(torch.cat([fast_h, anchor_h], dim=-1))) * (self.residual_scale * 1.25)

        assist_state = self._assist_state_from_slow_input(x_slow)
        correction = (1.0 - assist_state) * (off_bias + off_res) + assist_state * (on_bias + on_res)
        pred = nominal + correction
        pred = self._reshape_maybe(pred)
        pred = self._add_residual(pred, base)
        return {
            "pred": pred,
            "anchor_component": self._reshape_maybe(y_anchor),
            "nominal_component": self._reshape_maybe(nominal),
            "slow_bias": self._reshape_maybe((1.0 - assist_state) * off_bias + assist_state * on_bias),
            "fast_residual": self._reshape_maybe((1.0 - assist_state) * off_res + assist_state * on_res),
            "assist_state": self._reshape_maybe(assist_state),
        }


class AssistOnBiasCorrectorTCN_GRU(AdaptiveComplementaryTCN_GRU):
    """Nominal predictor with a stronger bias corrector that only activates when assist is on."""
    def __init__(self, *args, on_bias_scale: float = 0.18, assist_state_index: int = -3, **kwargs):
        super().__init__(*args, **kwargs)
        self.on_bias_scale = float(on_bias_scale)
        self.on_bias_head = nn.Linear(self.slow_gru.hidden_size, self.horizon * self.output_dim)
        self.assist_state_index = int(assist_state_index)

    def forward(self, x):
        if x.dim() == 2:
            x = x.unsqueeze(1)
        base = self._get_residual_base(x)
        if self.use_input_norm:
            if isinstance(self.input_norm, nn.InstanceNorm1d):
                x = x.transpose(1, 2)
                x = self.input_norm(x)
                x = x.transpose(1, 2)
            else:
                x = self.input_norm(x)
        x_fast, x_anchor, x_slow = self._split(x)
        fast_seq = self.fast_encoder.network(x_fast.transpose(1, 2)).transpose(1, 2)
        anchor_seq = self.anchor_encoder.network(x_anchor.transpose(1, 2)).transpose(1, 2)
        slow_seq = self.slow_encoder.network(x_slow.transpose(1, 2)).transpose(1, 2)

        fast_h = self.fast_gru(fast_seq)[0][:, -1, :]
        anchor_h = self.anchor_gru(anchor_seq)[0][:, -1, :]
        slow_h = self.slow_gru(slow_seq)[0][:, -1, :]
        y_fast = self.fast_head(fast_h)
        y_anchor = self.anchor_head(anchor_h)
        gate = torch.sigmoid(self.gate_head(torch.cat([fast_h, anchor_h, slow_h], dim=-1)) / max(self.gate_temperature, 1e-6))
        nominal = (1.0 - gate) * y_fast + gate * y_anchor
        fast_residual = torch.tanh(self.fast_residual_head(torch.cat([fast_h, anchor_h], dim=-1))) * self.residual_scale
        assist_state = torch.clamp(x_slow[:, :, self.assist_state_index].mean(dim=1, keepdim=True), 0.0, 1.0)
        slow_bias = assist_state * (torch.tanh(self.on_bias_head(slow_h)) * self.on_bias_scale)
        pred = nominal + fast_residual + slow_bias
        pred = self._reshape_maybe(pred)
        pred = self._add_residual(pred, base)
        return {
            "pred": pred,
            "anchor_component": self._reshape_maybe(y_anchor),
            "nominal_component": self._reshape_maybe(nominal),
            "slow_bias": self._reshape_maybe(slow_bias),
            "fast_residual": self._reshape_maybe(fast_residual),
            "assist_state": self._reshape_maybe(assist_state),
        }


class SpeedScaleSwitchTCN_GRU(AdaptiveComplementaryTCN_GRU):
    """Shared nominal backbone with low/high-speed-specific calibration heads.

    The routing gate is learned from anchor + slow context instead of assist-only
    metadata so the model can separately calibrate ~1.0 m/s and ~1.25 m/s plateaus.
    """
    def __init__(
        self,
        *args,
        low_scale: float = 0.05,
        high_scale: float = 0.14,
        speed_temperature: float = 1.0,
        hard_switch: bool = False,
        **kwargs,
    ):
        super().__init__(*args, **kwargs)
        slow_dim = self.slow_gru.hidden_size
        joint_dim = self.fast_gru.hidden_size + self.anchor_gru.hidden_size
        self.low_bias_head = nn.Linear(slow_dim, self.horizon * self.output_dim)
        self.high_bias_head = nn.Linear(slow_dim, self.horizon * self.output_dim)
        self.low_residual_head = nn.Linear(joint_dim, self.horizon * self.output_dim)
        self.high_residual_head = nn.Linear(joint_dim, self.horizon * self.output_dim)
        self.speed_gate_head = nn.Linear(self.anchor_gru.hidden_size + slow_dim, self.horizon * self.output_dim)
        self.low_scale = float(low_scale)
        self.high_scale = float(high_scale)
        self.speed_temperature = float(speed_temperature)
        self.hard_switch = bool(hard_switch)

    def forward(self, x):
        if x.dim() == 2:
            x = x.unsqueeze(1)
        base = self._get_residual_base(x)
        if self.use_input_norm:
            if isinstance(self.input_norm, nn.InstanceNorm1d):
                x = x.transpose(1, 2)
                x = self.input_norm(x)
                x = x.transpose(1, 2)
            else:
                x = self.input_norm(x)

        x_fast, x_anchor, x_slow = self._split(x)
        fast_seq = self.fast_encoder.network(x_fast.transpose(1, 2)).transpose(1, 2)
        anchor_seq = self.anchor_encoder.network(x_anchor.transpose(1, 2)).transpose(1, 2)
        slow_seq = self.slow_encoder.network(x_slow.transpose(1, 2)).transpose(1, 2)

        fast_h = self.fast_gru(fast_seq)[0][:, -1, :]
        anchor_h = self.anchor_gru(anchor_seq)[0][:, -1, :]
        slow_h = self.slow_gru(slow_seq)[0][:, -1, :]

        y_fast = self.fast_head(fast_h)
        y_anchor = self.anchor_head(anchor_h)
        nominal_gate = torch.sigmoid(
            self.gate_head(torch.cat([fast_h, anchor_h, slow_h], dim=-1)) / max(self.gate_temperature, 1e-6)
        )
        nominal = (1.0 - nominal_gate) * y_fast + nominal_gate * y_anchor

        speed_gate = torch.sigmoid(
            self.speed_gate_head(torch.cat([anchor_h, slow_h], dim=-1)) / max(self.speed_temperature, 1e-6)
        )
        if self.hard_switch:
            speed_gate = (speed_gate > 0.5).to(speed_gate.dtype)

        joint_h = torch.cat([fast_h, anchor_h], dim=-1)
        low_bias = torch.tanh(self.low_bias_head(slow_h)) * self.low_scale
        high_bias = torch.tanh(self.high_bias_head(slow_h)) * self.high_scale
        low_residual = torch.tanh(self.low_residual_head(joint_h)) * (self.residual_scale * 0.85)
        high_residual = torch.tanh(self.high_residual_head(joint_h)) * (self.residual_scale * 1.20)

        slow_bias = (1.0 - speed_gate) * low_bias + speed_gate * high_bias
        fast_residual = (1.0 - speed_gate) * low_residual + speed_gate * high_residual
        pred = nominal + slow_bias + fast_residual
        pred = self._reshape_maybe(pred)
        pred = self._add_residual(pred, base)
        return {
            "pred": pred,
            "anchor_component": self._reshape_maybe(y_anchor),
            "nominal_component": self._reshape_maybe(nominal),
            "slow_bias": self._reshape_maybe(slow_bias),
            "fast_residual": self._reshape_maybe(fast_residual),
            "speed_scale_gate": self._reshape_maybe(speed_gate),
        }


class RegimeExpertTCN_GRU(AdaptiveComplementaryTCN_GRU):
    """Three-regime expert corrector: off / on-low / on-high."""
    def __init__(self, *args, expert_scale: float = 0.14, assist_state_index: int = -3, assist_high_index: int = -2, **kwargs):
        super().__init__(*args, **kwargs)
        self.regime_gate = nn.Linear(self.slow_gru.hidden_size + 2, 3)
        self.expert_heads = nn.ModuleList([
            nn.Linear(self.slow_gru.hidden_size + self.fast_gru.hidden_size + self.anchor_gru.hidden_size, self.horizon * self.output_dim)
            for _ in range(3)
        ])
        self.expert_scale = float(expert_scale)
        self.assist_state_index = int(assist_state_index)
        self.assist_high_index = int(assist_high_index)

    def forward(self, x):
        if x.dim() == 2:
            x = x.unsqueeze(1)
        base = self._get_residual_base(x)
        if self.use_input_norm:
            if isinstance(self.input_norm, nn.InstanceNorm1d):
                x = x.transpose(1, 2)
                x = self.input_norm(x)
                x = x.transpose(1, 2)
            else:
                x = self.input_norm(x)
        x_fast, x_anchor, x_slow = self._split(x)
        fast_seq = self.fast_encoder.network(x_fast.transpose(1, 2)).transpose(1, 2)
        anchor_seq = self.anchor_encoder.network(x_anchor.transpose(1, 2)).transpose(1, 2)
        slow_seq = self.slow_encoder.network(x_slow.transpose(1, 2)).transpose(1, 2)
        fast_h = self.fast_gru(fast_seq)[0][:, -1, :]
        anchor_h = self.anchor_gru(anchor_seq)[0][:, -1, :]
        slow_h = self.slow_gru(slow_seq)[0][:, -1, :]
        y_fast = self.fast_head(fast_h)
        y_anchor = self.anchor_head(anchor_h)
        gate = torch.sigmoid(self.gate_head(torch.cat([fast_h, anchor_h, slow_h], dim=-1)) / max(self.gate_temperature, 1e-6))
        nominal = (1.0 - gate) * y_fast + gate * y_anchor

        assist_active = x_slow[:, :, self.assist_state_index].mean(dim=1, keepdim=True)
        assist_high = x_slow[:, :, self.assist_high_index].mean(dim=1, keepdim=True)
        regime_logits = self.regime_gate(torch.cat([slow_h, assist_active, assist_high], dim=-1))
        regime_probs = F.softmax(regime_logits, dim=-1)
        expert_in = torch.cat([fast_h, anchor_h, slow_h], dim=-1)
        corr = 0.0
        for idx, head in enumerate(self.expert_heads):
            part = torch.tanh(head(expert_in)) * self.expert_scale
            corr = corr + regime_probs[:, idx:idx+1] * part
        pred = nominal + corr
        pred = self._reshape_maybe(pred)
        pred = self._add_residual(pred, base)
        return {
            "pred": pred,
            "anchor_component": self._reshape_maybe(y_anchor),
            "nominal_component": self._reshape_maybe(nominal),
            "fast_residual": self._reshape_maybe(corr),
            "regime_logits": regime_logits,
        }


class AssistSplitTCN_GRU(TCN_MLP):
    """Fully split off/on model routed by causal assist-active proxy."""
    def __init__(
        self,
        *args,
        gru_hidden: int = 32,
        branch_input_dims=None,
        branch_channels=None,
        bias_scale: float = 0.16,
        residual_scale: float = 0.06,
        **kwargs,
    ):
        super().__init__(*args, **kwargs)
        input_dim = args[0] if args else kwargs["input_dim"]
        channels = kwargs.get("channels", (32, 32, 64, 64))
        if isinstance(channels, list):
            channels = tuple(channels)
        dropout = kwargs.get("dropout", 0.1)
        tcn_norm = kwargs.get("tcn_norm", None)
        if not branch_input_dims or len(branch_input_dims) != 3:
            raise ValueError("AssistSplitTCN_GRU requires branch_input_dims=[fast, anchor, slow]")
        branch_input_dims = tuple(int(v) for v in branch_input_dims)
        if sum(branch_input_dims) != input_dim:
            raise ValueError(f"branch_input_dims={branch_input_dims} do not sum to input_dim={input_dim}")
        if branch_channels is None:
            branch_channels = channels
        if isinstance(branch_channels, list):
            branch_channels = tuple(branch_channels)
        self.branch_input_dims = branch_input_dims
        self.bias_scale = float(bias_scale)
        self.residual_scale = float(residual_scale)
        self.head_base = nn.Identity()

        def _pair():
            enc_fast = TCNEncoder(branch_input_dims[0], branch_channels, kwargs.get("kernel_size", 3), dropout, norm_type=tcn_norm)
            enc_anchor = TCNEncoder(branch_input_dims[1], branch_channels, kwargs.get("kernel_size", 3), dropout, norm_type=tcn_norm)
            gru_fast = nn.GRU(branch_channels[-1], gru_hidden, batch_first=True)
            gru_anchor = nn.GRU(branch_channels[-1], gru_hidden, batch_first=True)
            fast_head = nn.Linear(gru_hidden, self.horizon * self.output_dim)
            anchor_head = nn.Linear(gru_hidden, self.horizon * self.output_dim)
            mix_head = nn.Linear(gru_hidden * 2, self.horizon * self.output_dim)
            return enc_fast, enc_anchor, gru_fast, gru_anchor, fast_head, anchor_head, mix_head

        (
            self.off_fast_encoder, self.off_anchor_encoder, self.off_fast_gru, self.off_anchor_gru,
            self.off_fast_head, self.off_anchor_head, self.off_mix_head
        ) = _pair()
        (
            self.on_fast_encoder, self.on_anchor_encoder, self.on_fast_gru, self.on_anchor_gru,
            self.on_fast_head, self.on_anchor_head, self.on_mix_head
        ) = _pair()
        self.slow_encoder = TCNEncoder(branch_input_dims[2], branch_channels, kwargs.get("kernel_size", 3), dropout, norm_type=tcn_norm)
        self.slow_gru = nn.GRU(branch_channels[-1], gru_hidden, batch_first=True)
        self.off_bias_head = nn.Linear(gru_hidden, self.horizon * self.output_dim)
        self.on_bias_head = nn.Linear(gru_hidden, self.horizon * self.output_dim)

    def _split(self, x):
        d0, d1, d2 = self.branch_input_dims
        return x[:, :, :d0], x[:, :, d0:d0+d1], x[:, :, d0+d1:d0+d1+d2]

    def _reshape_maybe(self, out):
        if self.horizon > 1:
            return out.view(-1, self.horizon, self.output_dim)
        return out

    def _forward_branch(self, x_fast, x_anchor, fast_encoder, anchor_encoder, fast_gru, anchor_gru, fast_head, anchor_head, mix_head):
        fast_seq = fast_encoder.network(x_fast.transpose(1, 2)).transpose(1, 2)
        anchor_seq = anchor_encoder.network(x_anchor.transpose(1, 2)).transpose(1, 2)
        fast_h = fast_gru(fast_seq)[0][:, -1, :]
        anchor_h = anchor_gru(anchor_seq)[0][:, -1, :]
        y_fast = fast_head(fast_h)
        y_anchor = anchor_head(anchor_h)
        mix = torch.sigmoid(mix_head(torch.cat([fast_h, anchor_h], dim=-1)))
        return (1.0 - mix) * y_fast + mix * y_anchor, fast_h, anchor_h

    def forward(self, x):
        if x.dim() == 2:
            x = x.unsqueeze(1)
        base = self._get_residual_base(x)
        if self.use_input_norm:
            if isinstance(self.input_norm, nn.InstanceNorm1d):
                x = x.transpose(1, 2)
                x = self.input_norm(x)
                x = x.transpose(1, 2)
            else:
                x = self.input_norm(x)
        x_fast, x_anchor, x_slow = self._split(x)
        off_nominal, _, _ = self._forward_branch(x_fast, x_anchor, self.off_fast_encoder, self.off_anchor_encoder, self.off_fast_gru, self.off_anchor_gru, self.off_fast_head, self.off_anchor_head, self.off_mix_head)
        on_nominal, _, _ = self._forward_branch(x_fast, x_anchor, self.on_fast_encoder, self.on_anchor_encoder, self.on_fast_gru, self.on_anchor_gru, self.on_fast_head, self.on_anchor_head, self.on_mix_head)
        slow_seq = self.slow_encoder.network(x_slow.transpose(1, 2)).transpose(1, 2)
        slow_h = self.slow_gru(slow_seq)[0][:, -1, :]
        off_bias = torch.tanh(self.off_bias_head(slow_h)) * self.bias_scale
        on_bias = torch.tanh(self.on_bias_head(slow_h)) * self.bias_scale
        assist_state = torch.clamp(x_slow[:, :, -3].mean(dim=1, keepdim=True), 0.0, 1.0)
        pred = (1.0 - assist_state) * (off_nominal + off_bias) + assist_state * (on_nominal + on_bias)
        pred = self._reshape_maybe(pred)
        pred = self._add_residual(pred, base)
        return {
            "pred": pred,
            "anchor_component": self._reshape_maybe((1.0 - assist_state) * off_nominal + assist_state * on_nominal),
            "off_nominal": self._reshape_maybe(off_nominal),
            "on_nominal": self._reshape_maybe(on_nominal),
            "assist_state": self._reshape_maybe(assist_state),
        }


class _ThreeBranchTemporalBase(TCN_MLP):
    """Shared utilities for three-branch causal models."""
    def __init__(self, *args, branch_input_dims=None, branch_channels=None, **kwargs):
        super().__init__(*args, **kwargs)
        input_dim = args[0] if args else kwargs["input_dim"]
        channels = kwargs.get("channels", (32, 32, 64, 64))
        if isinstance(channels, list):
            channels = tuple(channels)
        dropout = kwargs.get("dropout", 0.1)
        tcn_norm = kwargs.get("tcn_norm", None)
        if not branch_input_dims or len(branch_input_dims) != 3:
            raise ValueError(f"{self.__class__.__name__} requires branch_input_dims=[fast, anchor, slow]")
        branch_input_dims = tuple(int(v) for v in branch_input_dims)
        if sum(branch_input_dims) != input_dim:
            raise ValueError(f"branch_input_dims={branch_input_dims} do not sum to input_dim={input_dim}")
        if branch_channels is None:
            branch_channels = channels
        if isinstance(branch_channels, list):
            branch_channels = tuple(branch_channels)

        self.branch_input_dims = branch_input_dims
        self.fast_encoder = TCNEncoder(branch_input_dims[0], branch_channels, kwargs.get("kernel_size", 3), dropout, norm_type=tcn_norm)
        self.anchor_encoder = TCNEncoder(branch_input_dims[1], branch_channels, kwargs.get("kernel_size", 3), dropout, norm_type=tcn_norm)
        self.slow_encoder = TCNEncoder(branch_input_dims[2], branch_channels, kwargs.get("kernel_size", 3), dropout, norm_type=tcn_norm)
        self.branch_out_dim = branch_channels[-1]
        self.head_base = nn.Identity()

    def _split(self, x):
        d0, d1, d2 = self.branch_input_dims
        return x[:, :, :d0], x[:, :, d0:d0 + d1], x[:, :, d0 + d1:d0 + d1 + d2]

    def _reshape_maybe(self, out):
        if self.horizon > 1:
            return out.view(-1, self.horizon, self.output_dim)
        return out

    def _norm_input(self, x):
        if self.use_input_norm:
            if isinstance(self.input_norm, nn.InstanceNorm1d):
                x = x.transpose(1, 2)
                x = self.input_norm(x)
                x = x.transpose(1, 2)
            else:
                x = self.input_norm(x)
        return x

    def _encode_sequences(self, x):
        x = self._norm_input(x)
        x_fast, x_anchor, x_slow = self._split(x)
        fast_seq = self.fast_encoder.network(x_fast.transpose(1, 2)).transpose(1, 2)
        anchor_seq = self.anchor_encoder.network(x_anchor.transpose(1, 2)).transpose(1, 2)
        slow_seq = self.slow_encoder.network(x_slow.transpose(1, 2)).transpose(1, 2)
        return fast_seq, anchor_seq, slow_seq

    @staticmethod
    def _last_hidden(seq, gru):
        return gru(seq)[0][:, -1, :]

    @staticmethod
    def _downsample_seq(seq, stride):
        stride = max(1, int(stride))
        idx = torch.arange(stride - 1, seq.shape[1], stride, device=seq.device)
        if idx.numel() == 0:
            idx = torch.tensor([seq.shape[1] - 1], device=seq.device)
        return seq.index_select(1, idx)


class StrideProgressionTCN_GRU(_ThreeBranchTemporalBase):
    """Summarize one window as phase/stride bins before regressing speed."""
    def __init__(
        self,
        *args,
        gru_hidden: int = 32,
        phase_bins: int = 4,
        bias_scale: float = 0.10,
        residual_scale: float = 0.04,
        **kwargs,
    ):
        super().__init__(*args, **kwargs)
        enc_dim = self.branch_out_dim
        self.phase_bins = max(2, int(phase_bins))
        self.bias_scale = float(bias_scale)
        self.residual_scale = float(residual_scale)
        self.phase_head = nn.Linear(enc_dim, self.phase_bins)
        self.bin_proj = nn.Linear(enc_dim * 2, gru_hidden)
        self.stride_gru = nn.GRU(gru_hidden, gru_hidden, batch_first=True)
        self.slow_gru = nn.GRU(enc_dim, gru_hidden, batch_first=True)
        self.nominal_head = nn.Linear(gru_hidden, self.horizon * self.output_dim)
        self.anchor_head = nn.Linear(gru_hidden, self.horizon * self.output_dim)
        self.mix_head = nn.Linear(gru_hidden * 2, self.horizon * self.output_dim)
        self.bias_head = nn.Linear(gru_hidden, self.horizon * self.output_dim)
        self.residual_head = nn.Linear(gru_hidden * 2, self.horizon * self.output_dim)

    def forward(self, x):
        if x.dim() == 2:
            x = x.unsqueeze(1)
        base = self._get_residual_base(x)
        fast_seq, anchor_seq, slow_seq = self._encode_sequences(x)
        phase_logits = self.phase_head(anchor_seq).transpose(1, 2)
        phase_w = torch.softmax(phase_logits, dim=-1)
        pooled_fast = torch.einsum("bkt,btd->bkd", phase_w, fast_seq)
        pooled_anchor = torch.einsum("bkt,btd->bkd", phase_w, anchor_seq)
        stride_bins = F.relu(self.bin_proj(torch.cat([pooled_fast, pooled_anchor], dim=-1)))
        stride_h = self.stride_gru(stride_bins)[0][:, -1, :]
        slow_h = self.slow_gru(slow_seq)[0][:, -1, :]
        nominal = self.nominal_head(stride_h)
        anchor = self.anchor_head(stride_h)
        mix = torch.sigmoid(self.mix_head(torch.cat([stride_h, slow_h], dim=-1)))
        slow_bias = torch.tanh(self.bias_head(slow_h)) * self.bias_scale
        fast_residual = torch.tanh(self.residual_head(torch.cat([stride_h, slow_h], dim=-1))) * self.residual_scale
        pred = (1.0 - mix) * nominal + mix * anchor + slow_bias + fast_residual
        pred = self._reshape_maybe(pred)
        pred = self._add_residual(pred, base)
        return {
            "pred": pred,
            "nominal_component": self._reshape_maybe(nominal),
            "anchor_component": self._reshape_maybe(anchor),
            "slow_bias": self._reshape_maybe(slow_bias),
            "fast_residual": self._reshape_maybe(fast_residual),
        }


class PDRHybridTCN_GRU(_ThreeBranchTemporalBase):
    """Classical inertial progression estimate plus learned anchor/bias correction."""
    def __init__(
        self,
        *args,
        gru_hidden: int = 32,
        pdr_scale: float = 0.10,
        bias_scale: float = 0.10,
        residual_scale: float = 0.04,
        **kwargs,
    ):
        super().__init__(*args, **kwargs)
        enc_dim = self.branch_out_dim
        self.pdr_scale = float(pdr_scale)
        self.bias_scale = float(bias_scale)
        self.residual_scale = float(residual_scale)
        self.anchor_gru = nn.GRU(enc_dim, gru_hidden, batch_first=True)
        self.slow_gru = nn.GRU(enc_dim, gru_hidden, batch_first=True)
        self.pdr_delta_head = nn.Linear(enc_dim, self.output_dim)
        self.pdr_gain_head = nn.Linear(enc_dim, self.output_dim)
        self.pdr_proj = nn.Linear(self.output_dim, self.horizon * self.output_dim)
        self.anchor_head = nn.Linear(gru_hidden, self.horizon * self.output_dim)
        self.mix_head = nn.Linear(gru_hidden * 2, self.horizon * self.output_dim)
        self.bias_head = nn.Linear(gru_hidden, self.horizon * self.output_dim)
        self.residual_head = nn.Linear(gru_hidden * 2, self.horizon * self.output_dim)

    def forward(self, x):
        if x.dim() == 2:
            x = x.unsqueeze(1)
        base = self._get_residual_base(x)
        fast_seq, anchor_seq, slow_seq = self._encode_sequences(x)
        pdr_delta = torch.tanh(self.pdr_delta_head(fast_seq))
        pdr_gain = torch.sigmoid(self.pdr_gain_head(anchor_seq))
        pdr_state = torch.cumsum(pdr_delta * pdr_gain, dim=1)[:, -1, :] / float(max(1, fast_seq.shape[1]))
        pdr_component = self.pdr_proj(pdr_state) * self.pdr_scale
        anchor_h = self.anchor_gru(anchor_seq)[0][:, -1, :]
        slow_h = self.slow_gru(slow_seq)[0][:, -1, :]
        anchor = F.softplus(self.anchor_head(anchor_h))
        mix = torch.sigmoid(self.mix_head(torch.cat([anchor_h, slow_h], dim=-1)))
        slow_bias = torch.tanh(self.bias_head(slow_h)) * self.bias_scale
        fast_residual = torch.tanh(self.residual_head(torch.cat([anchor_h, slow_h], dim=-1))) * self.residual_scale
        pred = mix * pdr_component + (1.0 - mix) * anchor + slow_bias + fast_residual
        pred = self._reshape_maybe(pred)
        pred = self._add_residual(pred, base)
        return {
            "pred": pred,
            "nominal_component": self._reshape_maybe(pdr_component),
            "anchor_component": self._reshape_maybe(anchor),
            "slow_bias": self._reshape_maybe(slow_bias),
            "fast_residual": self._reshape_maybe(fast_residual),
        }


class PredictedKinCascadeTCN_GRU(PrivilegedKinMemoryCalibratorTCN_GRU):
    """Use predicted lower-limb kinematics as a main anchor, not just an auxiliary head."""
    def __init__(self, *args, kin_mix_scale: float = 0.20, **kwargs):
        super().__init__(*args, **kwargs)
        aux_flat_dim = self.horizon * self.aux_dim
        out_flat_dim = self.horizon * self.output_dim
        self.kin_anchor_head = nn.Sequential(
            nn.Linear(aux_flat_dim, max(24, aux_flat_dim)),
            nn.ReLU(inplace=True),
            nn.Linear(max(24, aux_flat_dim), out_flat_dim),
        )
        self.kin_gate_head = nn.Linear(aux_flat_dim + out_flat_dim, out_flat_dim)
        self.kin_mix_scale = float(kin_mix_scale)

    def forward(self, x):
        out = super().forward(x)
        aux_kin = out["aux_kinematics"]
        nominal = out["nominal_component"]
        aux_flat = aux_kin.reshape(aux_kin.shape[0], -1)
        nominal_flat = nominal.reshape(nominal.shape[0], -1)
        kin_anchor = F.softplus(self.kin_anchor_head(aux_flat))
        kin_gate = torch.sigmoid(self.kin_gate_head(torch.cat([aux_flat, nominal_flat], dim=-1)))
        kin_refine = kin_gate * (kin_anchor - nominal_flat) * self.kin_mix_scale
        pred = out["pred"].reshape(out["pred"].shape[0], -1) + kin_refine
        out["pred"] = self._reshape_maybe(pred)
        out["anchor_component"] = self._reshape_maybe(kin_anchor)
        out["fast_residual"] = self._reshape_maybe(out["fast_residual"].reshape(out["fast_residual"].shape[0], -1) + kin_refine)
        out["kin_anchor_component"] = self._reshape_maybe(kin_anchor)
        return out


class ProgressionDistillTCN_GRU(PredictedKinCascadeTCN_GRU):
    """Student predicts a progression anchor that is distilled from privileged kinematics."""
    def __init__(self, *args, progression_scale: float = 0.16, **kwargs):
        super().__init__(*args, **kwargs)
        aux_flat_dim = self.horizon * self.aux_dim
        out_flat_dim = self.horizon * self.output_dim
        self.student_progression_head = nn.Sequential(
            nn.Linear(aux_flat_dim, max(24, aux_flat_dim)),
            nn.ReLU(inplace=True),
            nn.Linear(max(24, aux_flat_dim), out_flat_dim),
        )
        self.progression_gate_head = nn.Linear(aux_flat_dim + out_flat_dim, out_flat_dim)
        self.progression_scale = float(progression_scale)

    def forward(self, x):
        out = super().forward(x)
        aux_kin = out["aux_kinematics"]
        anchor = out["anchor_component"]
        aux_flat = aux_kin.reshape(aux_kin.shape[0], -1)
        anchor_flat = anchor.reshape(anchor.shape[0], -1)
        student_progression = F.softplus(self.student_progression_head(aux_flat))
        prog_gate = torch.sigmoid(self.progression_gate_head(torch.cat([aux_flat, anchor_flat], dim=-1)))
        refine = prog_gate * (student_progression - anchor_flat) * self.progression_scale
        pred = out["pred"].reshape(out["pred"].shape[0], -1) + refine
        out["pred"] = self._reshape_maybe(pred)
        out["distill_progression"] = self._reshape_maybe(student_progression)
        return out


class AssistDeconfoundTCN_GRU(_ThreeBranchTemporalBase):
    """Subtract an assist-context latent from the nominal representation before speed readout."""
    def __init__(
        self,
        *args,
        gru_hidden: int = 32,
        deconfound_scale: float = 0.25,
        bias_scale: float = 0.10,
        residual_scale: float = 0.04,
        **kwargs,
    ):
        super().__init__(*args, **kwargs)
        enc_dim = self.branch_out_dim
        self.fast_gru = nn.GRU(enc_dim, gru_hidden, batch_first=True)
        self.anchor_gru = nn.GRU(enc_dim, gru_hidden, batch_first=True)
        self.slow_gru = nn.GRU(enc_dim, gru_hidden, batch_first=True)
        self.nominal_proj = nn.Linear(gru_hidden, gru_hidden)
        self.anchor_proj = nn.Linear(gru_hidden, gru_hidden)
        self.deconfound_head = nn.Linear(gru_hidden, gru_hidden)
        self.nominal_head = nn.Linear(gru_hidden, self.horizon * self.output_dim)
        self.anchor_head = nn.Linear(gru_hidden, self.horizon * self.output_dim)
        self.mix_head = nn.Linear(gru_hidden * 3, self.horizon * self.output_dim)
        self.bias_head = nn.Linear(gru_hidden, self.horizon * self.output_dim)
        self.residual_head = nn.Linear(gru_hidden * 2, self.horizon * self.output_dim)
        self.deconfound_scale = float(deconfound_scale)
        self.bias_scale = float(bias_scale)
        self.residual_scale = float(residual_scale)

    def forward(self, x):
        if x.dim() == 2:
            x = x.unsqueeze(1)
        base = self._get_residual_base(x)
        fast_seq, anchor_seq, slow_seq = self._encode_sequences(x)
        fast_h = self.fast_gru(fast_seq)[0][:, -1, :]
        anchor_h = self.anchor_gru(anchor_seq)[0][:, -1, :]
        slow_h = self.slow_gru(slow_seq)[0][:, -1, :]
        nominal_z = torch.tanh(self.nominal_proj(fast_h))
        anchor_z = torch.tanh(self.anchor_proj(anchor_h))
        assist_delta = torch.tanh(self.deconfound_head(slow_h)) * self.deconfound_scale
        clean_z = nominal_z - assist_delta
        nominal = self.nominal_head(clean_z)
        anchor = self.anchor_head(anchor_z)
        mix = torch.sigmoid(self.mix_head(torch.cat([clean_z, anchor_z, slow_h], dim=-1)))
        slow_bias = torch.tanh(self.bias_head(slow_h)) * self.bias_scale
        fast_residual = torch.tanh(self.residual_head(torch.cat([clean_z, anchor_z], dim=-1))) * self.residual_scale
        pred = (1.0 - mix) * nominal + mix * anchor + slow_bias + fast_residual
        pred = self._reshape_maybe(pred)
        pred = self._add_residual(pred, base)
        return {
            "pred": pred,
            "nominal_component": self._reshape_maybe(nominal),
            "anchor_component": self._reshape_maybe(anchor),
            "slow_bias": self._reshape_maybe(slow_bias),
            "fast_residual": self._reshape_maybe(fast_residual),
        }


class MultiAnchorFusionTCN_GRU(_ThreeBranchTemporalBase):
    """Fuse multiple anchor hypotheses by learned uncertainty."""
    def __init__(
        self,
        *args,
        gru_hidden: int = 32,
        bias_scale: float = 0.10,
        residual_scale: float = 0.04,
        **kwargs,
    ):
        super().__init__(*args, **kwargs)
        enc_dim = self.branch_out_dim
        self.fast_gru = nn.GRU(enc_dim, gru_hidden, batch_first=True)
        self.anchor_gru = nn.GRU(enc_dim, gru_hidden, batch_first=True)
        self.slow_gru = nn.GRU(enc_dim, gru_hidden, batch_first=True)
        self.fast_anchor_head = nn.Linear(gru_hidden, self.horizon * self.output_dim)
        self.geom_anchor_head = nn.Linear(gru_hidden, self.horizon * self.output_dim)
        self.cross_anchor_head = nn.Linear(gru_hidden * 2, self.horizon * self.output_dim)
        self.fast_unc_head = nn.Linear(gru_hidden, self.horizon * self.output_dim)
        self.geom_unc_head = nn.Linear(gru_hidden, self.horizon * self.output_dim)
        self.cross_unc_head = nn.Linear(gru_hidden * 2, self.horizon * self.output_dim)
        self.bias_head = nn.Linear(gru_hidden, self.horizon * self.output_dim)
        self.residual_head = nn.Linear(gru_hidden * 3, self.horizon * self.output_dim)
        self.bias_scale = float(bias_scale)
        self.residual_scale = float(residual_scale)

    def forward(self, x):
        if x.dim() == 2:
            x = x.unsqueeze(1)
        base = self._get_residual_base(x)
        fast_seq, anchor_seq, slow_seq = self._encode_sequences(x)
        fast_h = self.fast_gru(fast_seq)[0][:, -1, :]
        anchor_h = self.anchor_gru(anchor_seq)[0][:, -1, :]
        slow_h = self.slow_gru(slow_seq)[0][:, -1, :]
        joint_h = torch.cat([fast_h, anchor_h], dim=-1)
        a_fast = self.fast_anchor_head(fast_h)
        a_geom = self.geom_anchor_head(anchor_h)
        a_cross = self.cross_anchor_head(joint_h)
        logvars = torch.stack(
            [
                self.fast_unc_head(fast_h),
                self.geom_unc_head(anchor_h),
                self.cross_unc_head(joint_h),
            ],
            dim=1,
        )
        weights = torch.softmax(-logvars, dim=1)
        anchors = torch.stack([a_fast, a_geom, a_cross], dim=1)
        anchor_fused = torch.sum(weights * anchors, dim=1)
        slow_bias = torch.tanh(self.bias_head(slow_h)) * self.bias_scale
        fast_residual = torch.tanh(self.residual_head(torch.cat([fast_h, anchor_h, slow_h], dim=-1))) * self.residual_scale
        pred = anchor_fused + slow_bias + fast_residual
        pred = self._reshape_maybe(pred)
        pred = self._add_residual(pred, base)
        return {
            "pred": pred,
            "nominal_component": self._reshape_maybe(anchor_fused),
            "anchor_component": self._reshape_maybe(anchor_fused),
            "slow_bias": self._reshape_maybe(slow_bias),
            "fast_residual": self._reshape_maybe(fast_residual),
        }


class AdaptiveBiasStateTCN_GRU(_ThreeBranchTemporalBase):
    """Update a slow bias state from anchor/fast disagreement over time."""
    def __init__(
        self,
        *args,
        gru_hidden: int = 32,
        bias_scale: float = 0.14,
        residual_scale: float = 0.04,
        update_stride: int = 8,
        **kwargs,
    ):
        super().__init__(*args, **kwargs)
        enc_dim = self.branch_out_dim
        self.fast_gru = nn.GRU(enc_dim, gru_hidden, batch_first=True)
        self.anchor_gru = nn.GRU(enc_dim, gru_hidden, batch_first=True)
        self.fast_head = nn.Linear(gru_hidden, self.horizon * self.output_dim)
        self.anchor_head = nn.Linear(gru_hidden, self.horizon * self.output_dim)
        self.mix_head = nn.Linear(gru_hidden * 2, self.horizon * self.output_dim)
        self.fast_state_proj = nn.Linear(enc_dim, gru_hidden)
        self.anchor_state_proj = nn.Linear(enc_dim, gru_hidden)
        self.slow_state_proj = nn.Linear(enc_dim, gru_hidden)
        self.bias_cell = nn.GRUCell(gru_hidden * 2, gru_hidden)
        self.bias_head = nn.Linear(gru_hidden, self.horizon * self.output_dim)
        self.residual_head = nn.Linear(gru_hidden * 2, self.horizon * self.output_dim)
        self.bias_scale = float(bias_scale)
        self.residual_scale = float(residual_scale)
        self.update_stride = max(1, int(update_stride))

    def forward(self, x):
        if x.dim() == 2:
            x = x.unsqueeze(1)
        base = self._get_residual_base(x)
        fast_seq, anchor_seq, slow_seq = self._encode_sequences(x)
        fast_h = self.fast_gru(fast_seq)[0][:, -1, :]
        anchor_h = self.anchor_gru(anchor_seq)[0][:, -1, :]
        y_fast = self.fast_head(fast_h)
        y_anchor = self.anchor_head(anchor_h)
        mix = torch.sigmoid(self.mix_head(torch.cat([fast_h, anchor_h], dim=-1)))
        nominal = (1.0 - mix) * y_fast + mix * y_anchor

        fast_ds = self._downsample_seq(fast_seq, self.update_stride)
        anchor_ds = self._downsample_seq(anchor_seq, self.update_stride)
        slow_ds = self._downsample_seq(slow_seq, self.update_stride)
        steps = min(fast_ds.shape[1], anchor_ds.shape[1], slow_ds.shape[1])
        state = torch.zeros(fast_ds.shape[0], self.fast_gru.hidden_size, device=x.device, dtype=fast_ds.dtype)
        for t in range(steps):
            fast_s = torch.tanh(self.fast_state_proj(fast_ds[:, t, :]))
            anchor_s = torch.tanh(self.anchor_state_proj(anchor_ds[:, t, :]))
            slow_s = torch.tanh(self.slow_state_proj(slow_ds[:, t, :]))
            disagreement = anchor_s - fast_s
            state = self.bias_cell(torch.cat([disagreement, slow_s], dim=-1), state)

        slow_bias = torch.tanh(self.bias_head(state)) * self.bias_scale
        fast_residual = torch.tanh(self.residual_head(torch.cat([fast_h, anchor_h], dim=-1))) * self.residual_scale
        pred = nominal + slow_bias + fast_residual
        pred = self._reshape_maybe(pred)
        pred = self._add_residual(pred, base)
        return {
            "pred": pred,
            "nominal_component": self._reshape_maybe(nominal),
            "anchor_component": self._reshape_maybe(y_anchor),
            "slow_bias": self._reshape_maybe(slow_bias),
            "fast_residual": self._reshape_maybe(fast_residual),
            "memory_state": state,
        }


class MonotoneProgressionTCN_GRU(_ThreeBranchTemporalBase):
    """Monotone-constrained speed head over cadence/step/progression components."""
    def __init__(
        self,
        *args,
        gru_hidden: int = 32,
        bias_scale: float = 0.08,
        residual_scale: float = 0.03,
        **kwargs,
    ):
        super().__init__(*args, **kwargs)
        enc_dim = self.branch_out_dim
        self.fast_gru = nn.GRU(enc_dim, gru_hidden, batch_first=True)
        self.anchor_gru = nn.GRU(enc_dim, gru_hidden, batch_first=True)
        self.slow_gru = nn.GRU(enc_dim, gru_hidden, batch_first=True)
        self.cadence_head = nn.Linear(gru_hidden, self.horizon * self.output_dim)
        self.step_head = nn.Linear(gru_hidden, self.horizon * self.output_dim)
        self.progression_head = nn.Linear(gru_hidden, self.horizon * self.output_dim)
        self.anchor_res_head = nn.Linear(gru_hidden * 2, self.horizon * self.output_dim)
        self.bias_head = nn.Linear(gru_hidden, self.horizon * self.output_dim)
        self.residual_head = nn.Linear(gru_hidden * 2, self.horizon * self.output_dim)
        self.raw_w_cad = nn.Parameter(torch.tensor(0.5))
        self.raw_w_step = nn.Parameter(torch.tensor(0.5))
        self.raw_w_prog = nn.Parameter(torch.tensor(0.5))
        self.bias_scale = float(bias_scale)
        self.residual_scale = float(residual_scale)

    def forward(self, x):
        if x.dim() == 2:
            x = x.unsqueeze(1)
        base = self._get_residual_base(x)
        fast_seq, anchor_seq, slow_seq = self._encode_sequences(x)
        fast_h = self.fast_gru(fast_seq)[0][:, -1, :]
        anchor_h = self.anchor_gru(anchor_seq)[0][:, -1, :]
        slow_h = self.slow_gru(slow_seq)[0][:, -1, :]
        cadence = F.softplus(self.cadence_head(anchor_h))
        step = F.softplus(self.step_head(anchor_h))
        progression = F.softplus(self.progression_head(fast_h))
        w_cad = F.softplus(self.raw_w_cad)
        w_step = F.softplus(self.raw_w_step)
        w_prog = F.softplus(self.raw_w_prog)
        anchor_res = F.softplus(self.anchor_res_head(torch.cat([fast_h, anchor_h], dim=-1)))
        anchor_speed = w_cad * cadence * (1.0 + w_step * step) + w_prog * progression + 0.1 * anchor_res
        slow_bias = torch.tanh(self.bias_head(slow_h)) * self.bias_scale
        fast_residual = torch.tanh(self.residual_head(torch.cat([fast_h, anchor_h], dim=-1))) * self.residual_scale
        pred = anchor_speed + slow_bias + fast_residual
        pred = self._reshape_maybe(pred)
        pred = self._add_residual(pred, base)
        return {
            "pred": pred,
            "nominal_component": self._reshape_maybe(anchor_speed),
            "anchor_component": self._reshape_maybe(anchor_speed),
            "slow_bias": self._reshape_maybe(slow_bias),
            "fast_residual": self._reshape_maybe(fast_residual),
            "cadence_component": self._reshape_maybe(cadence),
            "step_length_component": self._reshape_maybe(step),
        }


class TCN_GRU_MDN(TCN_GRU_Head):
    """TCN-GRU backbone with MDN output head.

    Default forward returns the mixture-expected mean so existing evaluation code
    can treat it like a standard regressor. Training can request raw mixture
    parameters via return_params=True.
    """
    def __init__(self, *args, num_mixtures: int = 5, **kwargs):
        super().__init__(*args, **kwargs)
        self.num_mixtures = int(num_mixtures)
        self.head_out = nn.Linear(
            self._gru_hidden,
            self.horizon * self.output_dim * self.num_mixtures * 3,
        )

    def _forward_raw(self, x: torch.Tensor) -> torch.Tensor:
        if x.dim() == 2:
            x = x.unsqueeze(1)

        if self.use_input_norm:
            if isinstance(self.input_norm, nn.InstanceNorm1d):
                x = x.transpose(1, 2)
                x = self.input_norm(x)
                x = x.transpose(1, 2)
            else:
                x = self.input_norm(x)

        x_t = x.transpose(1, 2)
        enc_seq = self.enc.network(x_t)
        enc_seq = enc_seq.transpose(1, 2)
        gru_out, _ = self.gru(enc_seq)
        ctx = gru_out[:, -1, :]
        out = self.head_out(ctx)
        if self.horizon > 1:
            out = out.view(-1, self.horizon, self.output_dim * self.num_mixtures * 3)
        return out

    def forward(
        self,
        x: torch.Tensor,
        return_params: bool = False,
        return_std: bool = False,
    ):
        raw = self._forward_raw(x)
        mean = mdn_expected_value(raw, self.output_dim, self.num_mixtures)

        if return_params and return_std:
            std = mdn_predictive_std(raw, self.output_dim, self.num_mixtures)
            return mean, std, raw
        if return_params:
            return raw
        if return_std:
            std = mdn_predictive_std(raw, self.output_dim, self.num_mixtures)
            return mean, std
        return mean
