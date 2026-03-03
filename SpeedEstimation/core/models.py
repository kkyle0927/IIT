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

    def forward(self, x):
        # x: (B, T, D)
        if x.dim() == 2:
            x = x.unsqueeze(1)

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

        return out


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

        gate = self.gating_net(x)  # (B, T, D)
        x_gated = x * gate
        return super().forward(x_gated)


class AttentionTCN(TCN_MLP):
    def __init__(self, *args, **kwargs):
        atten_type = kwargs.pop('attention_type', 'temporal')
        heads = kwargs.pop('attention_heads', 4)
        super().__init__(*args, **kwargs)

        self.atten_w = nn.Linear(self.enc.out_ch, 1)

    def forward(self, x):
        if x.dim() == 2:
            x = x.unsqueeze(1)

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

        return out


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
        return out
