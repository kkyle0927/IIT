import torch
import torch.fft


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

    # 3. Frequency Penalty Loss
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

    return losses
