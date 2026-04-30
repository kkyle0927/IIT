import torch
import torch.fft

from core.datasets import extract_target_base, inverse_target_transform


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

    # 3c. Derivative Matching Loss (encourage local speed-change shape over future horizons)
    w_deriv = float(train_cfg.get("derivative_loss_weight", 0.0))
    if w_deriv > 0:
        data_cfg = {}
        data_cfg.update(config.get("shared", {}).get("data", {}) or {})
        data_cfg.update(config.get("02_train", {}).get("data", {}) or {})
        est_tick_ranges = data_cfg.get("est_tick_ranges")
        window_output = data_cfg.get("window_output") or data_cfg.get("time_window_output") or config.get("window_output", 1)
        fs = float(data_cfg.get("fs", config.get("fs", 100.0)))
        derivative_mode = str(train_cfg.get("derivative_loss_mode", "mse")).lower()
        normalize_by_dt = bool(train_cfg.get("derivative_loss_normalize_by_dt", True))

        if est_tick_ranges and len(est_tick_ranges) > 1:
            dt_steps = [max(1, int(est_tick_ranges[i + 1]) - int(est_tick_ranges[i])) for i in range(len(est_tick_ranges) - 1)]
        elif int(window_output) > 1:
            dt_steps = [1] * (int(window_output) - 1)
        else:
            dt_steps = []

        dt_tensor = None
        if dt_steps:
            dt_sec = torch.tensor(dt_steps, dtype=torch.float32, device=device) / fs
            dt_tensor = dt_sec.view(1, -1, 1)

        def derivative_matching_fn(pred, target):
            if pred.dim() != 3 or target.dim() != 3 or pred.shape[1] <= 1:
                return torch.tensor(0.0, device=pred.device)
            pred_diff = pred[:, 1:, :] - pred[:, :-1, :]
            target_diff = target[:, 1:, :] - target[:, :-1, :]
            if normalize_by_dt and dt_tensor is not None and dt_tensor.shape[1] == pred_diff.shape[1]:
                pred_diff = pred_diff / dt_tensor
                target_diff = target_diff / dt_tensor
            if derivative_mode in ["l1", "mae", "abs"]:
                return torch.mean(torch.abs(pred_diff - target_diff))
            return torch.mean((pred_diff - target_diff) ** 2)

        losses["derivative_matching"] = {"fn": derivative_matching_fn, "weight": w_deriv}
        print(
            f"[Loss] Derivative Matching: weight={w_deriv}, mode={derivative_mode}, "
            f"normalize_by_dt={normalize_by_dt}, steps={dt_steps if dt_steps else 'N/A'}"
        )

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

    return losses
