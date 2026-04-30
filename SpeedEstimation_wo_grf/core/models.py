import torch
import torch.nn as nn
import torch.nn.functional as F
from typing import Optional


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
            while base.dim() < out.dim():
                base = base.unsqueeze(1)
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


class ObserverTCN_GRU(TCN_GRU_Head):
    """Causal observer-style model: propagated prior + gated learned correction."""
    def __init__(self, *args, gru_hidden=32, observer=None, **kwargs):
        self.observer_cfg = observer or {}
        super().__init__(*args, gru_hidden=gru_hidden, **kwargs)

        self.base_idx = int(self.observer_cfg["base_input_channel_idx"])
        accel_idx = self.observer_cfg.get("accel_input_channel_idx")
        self.accel_idx = None if accel_idx is None else int(accel_idx)
        self.correction_scale = float(self.observer_cfg.get("correction_scale", 1.0))
        self.gate_bias = float(self.observer_cfg.get("gate_bias", -1.0))
        prop_scale = float(self.observer_cfg.get("propagation_scale", 1.0))
        self.learnable_propagation_scale = bool(self.observer_cfg.get("learnable_propagation_scale", False))

        if self.learnable_propagation_scale:
            self.propagation_scale = nn.Parameter(torch.tensor(prop_scale, dtype=torch.float32))
        else:
            self.register_buffer("propagation_scale", torch.tensor(prop_scale, dtype=torch.float32), persistent=False)

        self.delta_head = nn.Linear(self._gru_hidden, self.horizon * self.output_dim)
        self.gain_head = nn.Linear(self._gru_hidden, self.horizon * self.output_dim)
        nn.init.constant_(self.gain_head.bias, self.gate_bias)

    def _encode_ctx(self, x):
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
        return gru_out[:, -1, :]

    def _reshape_scalar_source(self, src, batch_size):
        src = src.view(batch_size, 1)
        if self.output_dim > 1:
            src = src.expand(-1, self.output_dim)
        if self.horizon > 1:
            src = src.unsqueeze(1).expand(-1, self.horizon, -1)
        return src

    def forward(self, x):
        if x.dim() == 2:
            x = x.unsqueeze(1)

        x_raw = x
        batch_size = x_raw.size(0)
        ctx = self._encode_ctx(x)

        base = x_raw[:, -1, self.base_idx]
        prop = base
        if self.accel_idx is not None:
            prop = prop + self.propagation_scale * x_raw[:, -1, self.accel_idx]
        prop = self._reshape_scalar_source(prop, batch_size)

        delta = self.delta_head(ctx)
        gate = torch.sigmoid(self.gain_head(ctx))

        if self.horizon > 1:
            delta = delta.view(-1, self.horizon, self.output_dim)
            gate = gate.view(-1, self.horizon, self.output_dim)
        else:
            delta = delta.view(-1, self.output_dim)
            gate = gate.view(-1, self.output_dim)

        return prop + gate * (self.correction_scale * delta)


class FusionTCN_GRU(TCN_GRU_Head):
    """Confidence-weighted fusion between learned estimate and causal priors."""
    def __init__(self, *args, gru_hidden=32, fusion=None, **kwargs):
        self.fusion_cfg = fusion or {}
        super().__init__(*args, gru_hidden=gru_hidden, **kwargs)

        self.prior_indices = [int(i) for i in self.fusion_cfg.get("prior_input_channel_indices", [])]
        self.use_net_branch = bool(self.fusion_cfg.get("use_net_branch", True))
        self.weight_temperature = float(self.fusion_cfg.get("weight_temperature", 1.0))
        self.logit_bias = self.fusion_cfg.get("logit_bias")

        self.num_sources = len(self.prior_indices) + (1 if self.use_net_branch else 0)
        if self.num_sources < 1:
            raise ValueError("FusionTCN_GRU requires at least one source")

        if self.use_net_branch:
            self.net_head = nn.Linear(self._gru_hidden, self.horizon * self.output_dim)
        else:
            self.net_head = None
        self.weight_head = nn.Linear(self._gru_hidden, self.horizon * self.output_dim * self.num_sources)
        if self.logit_bias is not None and len(self.logit_bias) == self.num_sources:
            bias = torch.tensor(self.logit_bias, dtype=torch.float32).repeat(self.horizon * self.output_dim)
            with torch.no_grad():
                self.weight_head.bias.copy_(bias)

    def _encode_ctx(self, x):
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
        return gru_out[:, -1, :]

    def _expand_prior(self, prior, batch_size):
        prior = prior.view(batch_size, 1)
        if self.output_dim > 1:
            prior = prior.expand(-1, self.output_dim)
        prior = prior.unsqueeze(1)
        if self.horizon > 1:
            prior = prior.expand(-1, self.horizon, -1)
        return prior

    def forward(self, x):
        if x.dim() == 2:
            x = x.unsqueeze(1)

        x_raw = x
        batch_size = x_raw.size(0)
        ctx = self._encode_ctx(x)

        sources = []
        if self.use_net_branch:
            net_pred = self.net_head(ctx)
            if self.horizon > 1:
                net_pred = net_pred.view(-1, self.horizon, self.output_dim)
            else:
                net_pred = net_pred.view(-1, self.output_dim).unsqueeze(1)
            sources.append(net_pred)

        for idx in self.prior_indices:
            prior = self._expand_prior(x_raw[:, -1, idx], batch_size)
            sources.append(prior)

        logits = self.weight_head(ctx)
        logits = logits.view(-1, self.horizon if self.horizon > 1 else 1, self.output_dim, self.num_sources)
        weights = F.softmax(logits / max(self.weight_temperature, 1e-6), dim=-1)

        stacked = torch.stack(sources, dim=-1)  # (B, H, O, S)
        out = torch.sum(weights * stacked, dim=-1)
        if self.horizon == 1:
            out = out.squeeze(1)
        return out


def build_model_from_cfg(model_cfg, input_dim, output_dim, horizon, use_input_norm=True):
    model_cfg = model_cfg or {}
    tcn_channels = model_cfg.get("channels", (64, 64, 128))
    kernel_size = model_cfg.get("kernel_size", 3)
    dropout_p = float(model_cfg.get("dropout", 0.5))
    head_dropout = model_cfg.get("head_dropout", dropout_p)
    mlp_hidden = model_cfg.get("head_hidden", 128)
    model_norm_type = model_cfg.get("model_norm", None)
    input_norm_type = model_cfg.get("input_norm_type", model_norm_type or "layer")
    residual_skip = model_cfg.get("residual_skip", None)

    common_args = dict(
        input_dim=input_dim,
        output_dim=output_dim,
        horizon=horizon,
        channels=tcn_channels,
        kernel_size=kernel_size,
        dropout=dropout_p,
        head_dropout=head_dropout,
        mlp_hidden=mlp_hidden,
        use_input_norm=use_input_norm,
        input_norm_type=input_norm_type,
        tcn_norm=model_norm_type,
        mlp_norm=model_norm_type,
        residual_skip=residual_skip,
    )

    model_type = model_cfg.get("type", "TCN")
    if model_type == "StanceGatedTCN":
        return StanceGatedTCN(**common_args, gating_dim=model_cfg.get("gating_dim", 1))
    if model_type == "AttentionTCN":
        return AttentionTCN(**common_args, attention_type=model_cfg.get("attention_type", "temporal"), attention_heads=model_cfg.get("attention_heads", 4))
    if model_type == "TCN_GRU":
        return TCN_GRU_Head(**common_args, gru_hidden=model_cfg.get("gru_hidden", 32))
    if model_type == "CausalSmoothTCN_GRU":
        return CausalSmoothTCN_GRU(**common_args, gru_hidden=model_cfg.get("gru_hidden", 32), smooth_window=model_cfg.get("smooth_window", 11))
    if model_type == "MultiScaleTCN_GRU":
        return MultiScaleTCN_GRU(**common_args, gru_hidden=model_cfg.get("gru_hidden", 32), kernel_sizes=tuple(model_cfg.get("kernel_sizes", [3, 5, 7])))
    if model_type == "ObserverTCN_GRU":
        return ObserverTCN_GRU(**common_args, gru_hidden=model_cfg.get("gru_hidden", 32), observer=model_cfg.get("observer", {}))
    if model_type == "FusionTCN_GRU":
        return FusionTCN_GRU(**common_args, gru_hidden=model_cfg.get("gru_hidden", 32), fusion=model_cfg.get("fusion", {}))
    return TCN_MLP(**common_args)
