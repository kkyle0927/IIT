import torch
import torch.fft

from core.datasets import extract_target_base, inverse_target_transform


def _denorm_input_channel(x, idx, mean=None, std=None):
    ch = x[..., idx]
    if mean is not None and std is not None:
        ch = ch * std[idx] + mean[idx]
    return ch


def get_custom_losses(config, device, feature_names=None, scaler_mean=None, scaler_std=None):
    """
    Returns a dictionary of custom loss functions and their weights based on config.
    Structure: {'name': {'fn': func, 'weight': float}}
    """
    losses = {}
    train_cfg = config.get("02_train", {}).get("train", {})

    # 1. Smoothness Loss
    w_smooth = float(train_cfg.get("smoothness_loss_weight", 0.0))
    if w_smooth > 0:
        def smoothness_fn(pred):
            if pred.dim() == 3 and pred.shape[1] > 1:
                diff = pred[:, 1:, :] - pred[:, :-1, :]
                return torch.mean(diff ** 2)
            return torch.tensor(0.0, device=pred.device)

        losses["smoothness"] = {"fn": smoothness_fn, "weight": w_smooth}

    # 2. Kinematic/Physics Loss
    w_kin_pos = float(train_cfg.get("kinematic_loss_weight_pos", 0.0))
    w_kin_limit = float(train_cfg.get("kinematic_loss_weight_limit", 0.0))

    if w_kin_pos > 0 or w_kin_limit > 0:
        def kinematic_fn(x, pred):
            return torch.tensor(0.0, device=pred.device)

        losses["kinematic"] = {"fn": kinematic_fn, "weight": w_kin_pos + w_kin_limit}

    # 3. Gradient Penalty Loss (biomechanical speed change limit)
    w_grad = float(train_cfg.get("gradient_penalty_weight", 0.0))
    max_dv = float(train_cfg.get("gradient_penalty_max_dv", 0.025))  # m/s per sample at 100Hz
    if w_grad > 0:
        def gradient_penalty_fn(pred):
            if pred.dim() == 3 and pred.shape[1] > 1:
                # pred: (B, T, D) — penalize consecutive time-step changes exceeding max_dv
                dv = pred[:, 1:, :] - pred[:, :-1, :]
                excess = torch.clamp(dv.abs() - max_dv, min=0)
                return torch.mean(excess ** 2)
            elif pred.dim() == 2 and pred.shape[0] > 1:
                # pred: (B, D) — single-step, no temporal gradient to penalize
                pass
            return torch.tensor(0.0, device=pred.device)

        losses["gradient_penalty"] = {"fn": gradient_penalty_fn, "weight": w_grad}
        print(f"[Loss] Gradient Penalty: weight={w_grad}, max_dv={max_dv}")

    # 3b. Frequency Penalty Loss
    penalty_type = train_cfg.get("loss_penalty")
    if penalty_type == "frequency_cutoff_penalty":
        w_freq = float(train_cfg.get("loss_penalty_weight", 0.0))

        def freq_loss_fn(pred, target):
            if pred.shape[1] < 10:
                return torch.tensor(0.0, device=pred.device)

            freq_pred = torch.fft.rfft(pred, dim=1)
            freq_target = torch.fft.rfft(target, dim=1)

            mag_pred = torch.abs(freq_pred)
            mag_target = torch.abs(freq_target)

            threshold = 1e-4
            mask = mag_target < threshold

            penalty = mag_pred * mask.float()
            return torch.mean(penalty)

        if w_freq > 0:
            losses["freq_penalty"] = {"fn": freq_loss_fn, "weight": w_freq}

    # 4. Soft prior alignment loss
    w_prior = float(train_cfg.get("prior_loss_weight", 0.0))
    prior_idx = train_cfg.get("prior_input_channel_idx")
    if w_prior > 0 and prior_idx is not None:
        target_transform = config.get("02_train", {}).get("data", {}).get("target_transform")
        tt_mean = None if scaler_mean is None else torch.as_tensor(scaler_mean, dtype=torch.float32, device=device)
        tt_std = None if scaler_std is None else torch.as_tensor(scaler_std, dtype=torch.float32, device=device)
        prior_cfg = {"mode": "delta_from_input", "input_channel_idx": int(prior_idx)}

        def prior_alignment_fn(x, pred):
            pred_abs = inverse_target_transform(
                pred, x,
                target_transform=target_transform,
                input_mean=tt_mean,
                input_std=tt_std,
            )
            base = extract_target_base(
                x, prior_cfg,
                input_mean=tt_mean,
                input_std=tt_std,
            )
            if base is None:
                return torch.tensor(0.0, device=pred.device)
            base = base.unsqueeze(-1) if pred_abs.dim() >= 2 else base
            while base.dim() < pred_abs.dim():
                base = base.unsqueeze(1)
            return torch.mean((pred_abs - base) ** 2)

        losses["prior_alignment"] = {"fn": prior_alignment_fn, "weight": w_prior}
        print(f"[Loss] Prior Alignment: weight={w_prior}, input_channel_idx={int(prior_idx)}")

    # 5. High-speed underprediction penalty, optionally gated by torque-context.
    w_under = float(train_cfg.get("underprediction_loss_weight", 0.0))
    if w_under > 0:
        target_thr = float(train_cfg.get("underprediction_target_threshold", 0.8))
        ctx_idx = train_cfg.get("underprediction_context_channel_idx")
        ctx_thr = float(train_cfg.get("underprediction_context_threshold", 0.0))
        ctx_scale = float(train_cfg.get("underprediction_context_scale", 1.0))
        ctx_gamma = float(train_cfg.get("underprediction_context_gamma", 1.0))
        margin = float(train_cfg.get("underprediction_margin", 0.0))
        x_mean = None if scaler_mean is None else torch.as_tensor(scaler_mean, dtype=torch.float32, device=device)
        x_std = None if scaler_std is None else torch.as_tensor(scaler_std, dtype=torch.float32, device=device)
        if ctx_idx is not None:
            ctx_idx = int(ctx_idx)

        def underprediction_fn(x, pred, target):
            err = torch.relu(target - pred - margin)
            weights = (target >= target_thr).to(err.dtype)

            if ctx_idx is not None:
                ctx = _denorm_input_channel(x, ctx_idx, mean=x_mean, std=x_std)
                if ctx.dim() > 1:
                    ctx = ctx[:, -1]
                while ctx.dim() < err.dim():
                    ctx = ctx.unsqueeze(-1)
                ctx_w = torch.clamp((ctx.abs() - ctx_thr) / max(ctx_scale, 1e-6), min=0.0)
                weights = weights * (1.0 + ctx_gamma * ctx_w)

            return torch.mean(weights * (err ** 2))

        losses["underprediction"] = {"fn": underprediction_fn, "weight": w_under}
        print(
            "[Loss] Underprediction: "
            f"weight={w_under}, target_thr={target_thr}, "
            f"context_idx={ctx_idx}, context_thr={ctx_thr}, context_scale={ctx_scale}, context_gamma={ctx_gamma}"
        )

    # 6. Batch-level high-speed bias penalty.
    w_batch_bias = float(train_cfg.get("batch_bias_loss_weight", 0.0))
    if w_batch_bias > 0:
        batch_thr = float(train_cfg.get("batch_bias_target_threshold", 0.85))
        batch_margin = float(train_cfg.get("batch_bias_margin", 0.0))
        under_only = bool(train_cfg.get("batch_bias_under_only", True))

        def batch_bias_fn(pred, target):
            mask = (target >= batch_thr).to(pred.dtype)
            denom = torch.clamp(mask.sum(), min=1.0)
            mean_err = torch.sum((pred - target) * mask) / denom
            if under_only:
                return torch.relu(-(mean_err + batch_margin)) ** 2
            return mean_err ** 2

        losses["batch_bias"] = {"fn": batch_bias_fn, "weight": w_batch_bias}
        print(
            "[Loss] Batch Bias: "
            f"weight={w_batch_bias}, target_thr={batch_thr}, margin={batch_margin}, under_only={under_only}"
        )

    # 7. Assist-gated high-speed bias penalty for offset-heavy assisted windows.
    w_assist_bias = float(train_cfg.get("assist_bias_loss_weight", 0.0))
    if w_assist_bias > 0:
        target_thr = float(train_cfg.get("assist_bias_target_threshold", 0.85))
        ctx_idx = train_cfg.get("assist_bias_context_channel_idx")
        ctx_thr = float(train_cfg.get("assist_bias_context_threshold", 0.0))
        ctx_scale = float(train_cfg.get("assist_bias_context_scale", 1.0))
        ctx_gamma = float(train_cfg.get("assist_bias_context_gamma", 1.0))
        margin = float(train_cfg.get("assist_bias_margin", 0.0))
        under_only = bool(train_cfg.get("assist_bias_under_only", True))
        x_mean = None if scaler_mean is None else torch.as_tensor(scaler_mean, dtype=torch.float32, device=device)
        x_std = None if scaler_std is None else torch.as_tensor(scaler_std, dtype=torch.float32, device=device)
        if ctx_idx is not None:
            ctx_idx = int(ctx_idx)

        def assist_bias_fn(x, pred, target):
            mask = (target >= target_thr).to(pred.dtype)
            if ctx_idx is not None:
                ctx = _denorm_input_channel(x, ctx_idx, mean=x_mean, std=x_std)
                if ctx.dim() > 1:
                    ctx = ctx[:, -1]
                while ctx.dim() < mask.dim():
                    ctx = ctx.unsqueeze(-1)
                ctx_mag = torch.clamp((ctx.abs() - ctx_thr) / max(ctx_scale, 1e-6), min=0.0)
                mask = mask * (ctx_mag > 0).to(mask.dtype)
                weights = 1.0 + ctx_gamma * ctx_mag
            else:
                weights = torch.ones_like(mask)

            weighted_mask = mask * weights
            denom = torch.clamp(weighted_mask.sum(), min=1.0)
            mean_err = torch.sum((pred - target) * weighted_mask) / denom
            if under_only:
                return torch.relu(-(mean_err + margin)) ** 2
            return mean_err ** 2

        losses["assist_bias"] = {"fn": assist_bias_fn, "weight": w_assist_bias}
        print(
            "[Loss] Assist Bias: "
            f"weight={w_assist_bias}, target_thr={target_thr}, "
            f"context_idx={ctx_idx}, context_thr={ctx_thr}, context_scale={ctx_scale}, context_gamma={ctx_gamma}, "
            f"margin={margin}, under_only={under_only}"
        )

    return losses
