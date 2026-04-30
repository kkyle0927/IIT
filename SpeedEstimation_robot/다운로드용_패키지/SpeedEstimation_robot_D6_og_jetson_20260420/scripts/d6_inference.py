import argparse
import sys
from pathlib import Path

import numpy as np
import torch
import yaml
from scipy.signal import butter, sosfilt

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from src.core.models import CausalSmoothTCN_GRU
from src.data.body_frame import estimate_tilt_cf_imu_features
from src.data.model_com_speed import compute_imu_integrated_speed


FEATURE_ORDER = [
    "acc_hx",
    "acc_hy",
    "acc_up",
    "gyro_hx",
    "gyro_hy",
    "gyro_up",
    "roll_cf",
    "pitch_cf",
    "vel_hx",
    "vel_hy",
    "acc_h_mag",
    "vel_h_mag",
    "hip_angle_l",
    "hip_angle_linvel_l",
    "hip_angle_r",
    "hip_angle_linvel_r",
    "cadence_mean",
    "speed_anchor_geom",
]


def _causal_ema(signal: np.ndarray, alpha: float) -> np.ndarray:
    x = np.nan_to_num(np.asarray(signal, dtype=np.float32).flatten())
    y = np.zeros_like(x)
    if len(x) == 0:
        return y
    y[0] = x[0]
    for i in range(1, len(x)):
        y[i] = alpha * x[i] + (1.0 - alpha) * y[i - 1]
    return y


def _alpha_from_tau(tau_s: float, fs_hz: float) -> float:
    tau_s = max(float(tau_s), 1e-3)
    return 1.0 - np.exp(-1.0 / (tau_s * fs_hz))


def _causal_derivative(signal: np.ndarray, fs_hz: float) -> np.ndarray:
    x = np.nan_to_num(np.asarray(signal, dtype=np.float32).flatten())
    d = np.zeros_like(x)
    if len(x) > 1:
        d[1:] = (x[1:] - x[:-1]) * float(fs_hz)
    return d


def _build_treadmill_acc_mean(speed_leftbelt, speed_rightbelt, fs_hz: float) -> np.ndarray:
    left = np.asarray(speed_leftbelt, dtype=np.float32).flatten()
    right = np.asarray(speed_rightbelt, dtype=np.float32).flatten()
    if len(left) != len(right):
        raise ValueError(f"speed_leftbelt length mismatch: expected {len(left)}, got {len(right)}")
    mean_speed = 0.5 * (left + right)
    return _causal_derivative(mean_speed, fs_hz).astype(np.float32)


def _causal_running_rms(signal: np.ndarray, window_samples: int) -> np.ndarray:
    x = np.nan_to_num(np.asarray(signal, dtype=np.float32).flatten())
    w = max(int(window_samples), 1)
    out = np.zeros_like(x)
    sq_cum = np.concatenate([[0.0], np.cumsum(x.astype(np.float64) ** 2)])
    for i in range(len(x)):
        start = max(0, i - w + 1)
        n = i - start + 1
        power = (sq_cum[i + 1] - sq_cum[start]) / max(n, 1)
        out[i] = np.sqrt(max(power, 0.0))
    return out.astype(np.float32)


def _causal_lpf_1d(signal_1d: np.ndarray, cutoff_hz: float, fs_hz: float, order: int = 4) -> np.ndarray:
    x = np.asarray(signal_1d, dtype=np.float32).flatten()
    sos = butter(order, cutoff_hz, btype="low", fs=fs_hz, output="sos")
    return sosfilt(sos, x).astype(np.float32)


def _require_keys(payload, keys):
    missing = [k for k in keys if k not in payload]
    if missing:
        raise KeyError(f"Missing required keys: {missing}")


def _height_m_from_payload(payload):
    if "height_m" in payload:
        return float(np.asarray(payload["height_m"]).reshape(-1)[0])
    if "height_mm" in payload:
        return float(np.asarray(payload["height_mm"]).reshape(-1)[0]) / 1000.0
    raise KeyError("height_m or height_mm is required")


def _build_cadence_and_anchor(thigh_l, thigh_r, height_m, fs_hz):
    alpha_center = _alpha_from_tau(0.6, fs_hz)
    thigh_dot_l = _causal_derivative(thigh_l, fs_hz)
    thigh_dot_r = _causal_derivative(thigh_r, fs_hz)
    thigh_center_l = thigh_l - _causal_ema(thigh_l, alpha_center)
    thigh_center_r = thigh_r - _causal_ema(thigh_r, alpha_center)

    thigh_amp_l = np.maximum(_causal_running_rms(thigh_center_l, int(0.6 * fs_hz)), 5.0)
    thigh_amp_r = np.maximum(_causal_running_rms(thigh_center_r, int(0.6 * fs_hz)), 5.0)
    thigh_vel_scale_l = np.maximum(_causal_running_rms(thigh_dot_l, int(0.6 * fs_hz)), 20.0)
    thigh_vel_scale_r = np.maximum(_causal_running_rms(thigh_dot_r, int(0.6 * fs_hz)), 20.0)

    phase_l = np.unwrap(np.arctan2(thigh_dot_l / thigh_vel_scale_l, thigh_center_l / thigh_amp_l))
    phase_r = np.unwrap(np.arctan2(thigh_dot_r / thigh_vel_scale_r, thigh_center_r / thigh_amp_r))
    phase_rate_l = np.clip(_causal_derivative(phase_l.astype(np.float32), fs_hz) / (2.0 * np.pi), 0.0, 4.0)
    phase_rate_r = np.clip(_causal_derivative(phase_r.astype(np.float32), fs_hz) / (2.0 * np.pi), 0.0, 4.0)
    cadence_l = _causal_ema(phase_rate_l.astype(np.float32), _alpha_from_tau(0.25, fs_hz))
    cadence_r = _causal_ema(phase_rate_r.astype(np.float32), _alpha_from_tau(0.25, fs_hz))
    cadence_mean = (0.5 * (cadence_l + cadence_r)).astype(np.float32)

    leg_length_m = max(0.53 * float(height_m), 0.5)
    excursion = _causal_running_rms(np.deg2rad(thigh_l - thigh_r), int(0.5 * fs_hz))
    step_length_proxy = (leg_length_m * excursion).astype(np.float32)
    speed_anchor_geom = (cadence_mean * step_length_proxy).astype(np.float32)
    return cadence_mean, speed_anchor_geom


def build_d6_feature_matrix(input_npz: Path, fs_hz: float = 100.0) -> np.ndarray:
    payload = np.load(input_npz)
    _require_keys(
        payload,
        [
            "accel_x",
            "accel_y",
            "accel_z",
            "gyro_x",
            "gyro_y",
            "gyro_z",
            "hip_angle_l",
            "hip_angle_r",
            "thigh_angle_l",
            "thigh_angle_r",
            "speed_leftbelt",
            "speed_rightbelt",
        ],
    )
    height_m = _height_m_from_payload(payload)

    accel_x = np.asarray(payload["accel_x"], dtype=np.float32).flatten()
    accel_y = np.asarray(payload["accel_y"], dtype=np.float32).flatten()
    accel_z = np.asarray(payload["accel_z"], dtype=np.float32).flatten()
    gyro_x = np.asarray(payload["gyro_x"], dtype=np.float32).flatten()
    gyro_y = np.asarray(payload["gyro_y"], dtype=np.float32).flatten()
    gyro_z = np.asarray(payload["gyro_z"], dtype=np.float32).flatten()
    hip_angle_l = np.asarray(payload["hip_angle_l"], dtype=np.float32).flatten()
    hip_angle_r = np.asarray(payload["hip_angle_r"], dtype=np.float32).flatten()
    thigh_angle_l = np.asarray(payload["thigh_angle_l"], dtype=np.float32).flatten()
    thigh_angle_r = np.asarray(payload["thigh_angle_r"], dtype=np.float32).flatten()
    speed_leftbelt = np.asarray(payload["speed_leftbelt"], dtype=np.float32).flatten()
    speed_rightbelt = np.asarray(payload["speed_rightbelt"], dtype=np.float32).flatten()

    n = len(accel_x)
    for name, arr in {
        "accel_y": accel_y,
        "accel_z": accel_z,
        "gyro_x": gyro_x,
        "gyro_y": gyro_y,
        "gyro_z": gyro_z,
        "hip_angle_l": hip_angle_l,
        "hip_angle_r": hip_angle_r,
        "thigh_angle_l": thigh_angle_l,
        "thigh_angle_r": thigh_angle_r,
    }.items():
        if len(arr) != n:
            raise ValueError(f"{name} length mismatch: expected {n}, got {len(arr)}")

    acc_local = np.stack([accel_x, accel_y, accel_z], axis=1).astype(np.float32)
    gyro_local = np.stack([gyro_x, gyro_y, gyro_z], axis=1).astype(np.float32)

    treadmill_acc = _build_treadmill_acc_mean(speed_leftbelt, speed_rightbelt, fs_hz)

    tilt = estimate_tilt_cf_imu_features(
        acc_local,
        gyro_local,
        fs_hz,
        mode="tvcf",
        trust_ema=0.92,
        tvcf_omega_low=0.5,
        tvcf_omega_high=2.5,
    )
    tilt["acc_hy"] = (tilt["acc_hy"][:, 0] + treadmill_acc).astype(np.float32)[:, None]

    vel_hx = compute_imu_integrated_speed(tilt["acc_hx"][:, 0], fs_hz)
    vel_hy = compute_imu_integrated_speed(tilt["acc_hy"][:, 0], fs_hz)
    acc_h_mag = np.sqrt(tilt["acc_hx"][:, 0] ** 2 + tilt["acc_hy"][:, 0] ** 2).astype(np.float32)
    vel_h_mag = np.sqrt(vel_hx ** 2 + vel_hy ** 2).astype(np.float32)

    leg_length_m = max(0.53 * float(height_m), 0.5)
    hip_linvel_l = np.deg2rad(_causal_lpf_1d(_causal_derivative(hip_angle_l, fs_hz), 30.0, fs_hz, order=4)) * leg_length_m
    hip_linvel_r = np.deg2rad(_causal_lpf_1d(_causal_derivative(hip_angle_r, fs_hz), 30.0, fs_hz, order=4)) * leg_length_m
    cadence_mean, speed_anchor_geom = _build_cadence_and_anchor(thigh_angle_l, thigh_angle_r, height_m, fs_hz)

    feature_matrix = np.stack(
        [
            tilt["acc_hx"][:, 0],
            tilt["acc_hy"][:, 0],
            tilt["acc_up"][:, 0],
            tilt["gyro_hx"][:, 0],
            tilt["gyro_hy"][:, 0],
            tilt["gyro_up"][:, 0],
            tilt["roll_cf"],
            tilt["pitch_cf"],
            vel_hx,
            vel_hy,
            acc_h_mag,
            vel_h_mag,
            hip_angle_l,
            hip_linvel_l.astype(np.float32),
            hip_angle_r,
            hip_linvel_r.astype(np.float32),
            cadence_mean,
            speed_anchor_geom,
        ],
        axis=1,
    ).astype(np.float32)
    return feature_matrix


def load_bundle(bundle_dir: Path, device: str = "cpu"):
    bundle_dir = Path(bundle_dir)
    with open(bundle_dir / "config.yaml", "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)

    model_cfg = cfg["shared"]["model"]
    input_dim = len(FEATURE_ORDER)
    model = CausalSmoothTCN_GRU(
        input_dim=input_dim,
        output_dim=1,
        horizon=1,
        channels=tuple(model_cfg["channels"]),
        kernel_size=model_cfg.get("kernel_size", 5),
        dropout=model_cfg.get("dropout", 0.15),
        use_input_norm=model_cfg.get("use_input_norm", True),
        input_norm_type=model_cfg.get("input_norm_type", "instance"),
        tcn_norm=model_cfg.get("model_norm", "batch"),
        head_dropout=model_cfg.get("head_dropout", 0.2),
        mlp_hidden=model_cfg.get("head_hidden", [20]),
        gru_hidden=model_cfg.get("gru_hidden", 12),
        smooth_window=model_cfg.get("smooth_window", 9),
    )
    state = torch.load(bundle_dir / "model.pt", map_location=device)
    model.load_state_dict(state)
    model.to(device)
    model.eval()

    scaler = np.load(bundle_dir / "scaler.npz")
    return {
        "model": model,
        "cfg": cfg,
        "mean": scaler["mean"].astype(np.float32),
        "scale": scaler["scale"].astype(np.float32),
    }


@torch.no_grad()
def run_d6_inference(bundle_dir: Path, input_npz: Path, output_npz: Path, device: str = "cpu"):
    bundle = load_bundle(bundle_dir, device=device)
    cfg = bundle["cfg"]
    feature_matrix = build_d6_feature_matrix(input_npz, fs_hz=100.0)

    mean = bundle["mean"]
    scale = bundle["scale"]
    x_norm = (feature_matrix - mean[None, :]) / scale[None, :]

    window_size = int(cfg["shared"]["data"]["window_size"])
    horizon = int(cfg["shared"]["data"]["est_tick_ranges"][0])
    n = x_norm.shape[0]

    preds = np.full(n, np.nan, dtype=np.float32)
    model = bundle["model"]

    for t in range(window_size - 1, n - horizon):
        x_win = x_norm[t - window_size + 1 : t + 1]
        x_tensor = torch.from_numpy(x_win[None, :, :]).to(device=device, dtype=torch.float32)
        y_hat = model(x_tensor).detach().cpu().numpy().reshape(-1)[0]
        preds[t + horizon] = np.float32(y_hat)

    np.savez(
        output_npz,
        prediction_speed_y=preds,
        feature_matrix=feature_matrix,
        feature_names=np.asarray(FEATURE_ORDER),
        fs_hz=np.asarray([100.0], dtype=np.float32),
        window_size=np.asarray([window_size], dtype=np.int32),
        horizon_ticks=np.asarray([horizon], dtype=np.int32),
    )


def main():
    parser = argparse.ArgumentParser(description="Run D6 full-LOSO checkpoint inference from raw NPZ signals.")
    parser.add_argument("--bundle-dir", required=True, type=Path)
    parser.add_argument("--input-npz", required=True, type=Path)
    parser.add_argument("--output-npz", required=True, type=Path)
    parser.add_argument("--device", default="cpu")
    args = parser.parse_args()
    run_d6_inference(args.bundle_dir, args.input_npz, args.output_npz, device=args.device)


if __name__ == "__main__":
    main()
