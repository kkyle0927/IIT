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
