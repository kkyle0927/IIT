# eval_all_ckpts_abs.py
# 절대경로 고정 + 전 모델 Test 성능(bar) + 상위 5개 모델 True/Pred 그리드
# + 5개 아키텍처 각자의 최고 성능 비교(bar) + 그 5개 True/Pred 그리드
# + 막대가 많은 경우 자동 가로폭/폰트 축소/여백 최소화
# + 성능 지표 식 표시
# + 최고 성능 모델에 대해 cond별 20초 플롯
# + test dataset에 대해 cond별 RMSE%, NRMSE% (unit: %) 계산 및 CSV 저장

import os, glob, json, time, re
import numpy as np
import torch
import matplotlib
if os.environ.get("DISPLAY","")=="" and os.name != "nt":
    matplotlib.use("Agg")
import matplotlib.pyplot as plt
from tqdm import tqdm

from SpeedEstimator_train_v3 import (
    CONFIG, DEFAULT_FEAT_SPEC,
    load_all_subjects, count_and_fix_nans_in_Sub, preprocess_back_imu,
    normalize_fsr_by_mass, normalize_grf_by_mass, apply_tick_filter,
    SpeedDataset, _names_to_indices, _conds_to_ids, build_model_by_arch, SpeedNet
)

# ===== 절대경로(본인 PC에 맞게) =====
CKPT_DIR     = r"C:\Users\ChanyoungKo\OneDrive - 한국과학기술원 기계공학과\Kyle\KAIST\ExoLab\Research and projects\2025_Intention Intelligence Team\NetworkCode\SpeedEstimation"
CKPT_PATTERN = "SE_*.pt"
SAVE_DIR     = CKPT_DIR

# ---------------- 유틸 ----------------
def stack_preds(dl, model, device):
    model.eval()
    P_list, G_list = [], []
    with torch.no_grad():
        for Xb, Yb in dl:
            Xb = Xb.to(device, non_blocking=True)
            pred = model(Xb).cpu().numpy()
            P_list.append(pred); G_list.append(Yb.numpy())
    if not P_list:
        return np.empty((0,2), np.float32), np.empty((0,2), np.float32)
    return np.vstack(P_list), np.vstack(G_list)

def metrics(P, G):
    mse  = np.mean((P - G)**2, axis=0)
    rmse = np.sqrt(mse)
    mae  = np.mean(np.abs(P - G), axis=0)
    return dict(
        mse=list(map(float, mse)),
        rmse=list(map(float, rmse)),
        mae=list(map(float, mae)),
        mse_total=float(np.mean(mse)),
        rmse_total=float(np.mean(rmse)),
        mae_total=float(np.mean(mae)),
        n=int(len(P))
    )

def rmse_and_norm_perc(P, G):
    """
    RMSE%, NRMSE% 계산 (둘 다 unit: %)
    RMSE%   = RMSE / mean(|y|) * 100
    NRMSE%  = RMSE / (max(y) - min(y)) * 100
    """
    if len(G) == 0:
        return {
            "rmse": np.array([np.nan, np.nan], dtype=np.float32),
            "rmse_pct": np.array([np.nan, np.nan], dtype=np.float32),
            "nrmse_pct": np.array([np.nan, np.nan], dtype=np.float32),
        }
    rmse = np.sqrt(np.mean((P - G) ** 2, axis=0))
    mean_abs = np.mean(np.abs(G), axis=0)
    denom_range = np.max(G, axis=0) - np.min(G, axis=0)

    eps = 1e-9
    rmse_pct = rmse / np.maximum(mean_abs, eps) * 100.0
    nrmse_pct = rmse / np.maximum(denom_range, eps) * 100.0

    return {
        "rmse": rmse,
        "rmse_pct": rmse_pct,
        "nrmse_pct": nrmse_pct,
    }

# 성능 지표 식(플롯에 같이 표시)
METRIC_FORMULAS = (
    "MSE      = mean((ŷ - y)^2)\n"
    "RMSE     = sqrt(MSE)\n"
    "RMSE(%)  = RMSE / mean(|y|) * 100\n"
    "NRMSE(%) = RMSE / (max(y) - min(y)) * 100"
)

def build_test_loader(Sub, cfg, mu, sd, device, spec, use_conds=None):
    """
    use_conds: None이면 cfg['use_conds'] 사용.
               리스트로 주면 해당 cond만 사용.
    """
    test_sub_indices = _names_to_indices(cfg["sub_names"], cfg.get("test_subs", []))
    cond_names = cfg["cond_names"]
    if use_conds is None:
        use_conds = cfg["use_conds"]
    cond_ids = _conds_to_ids(cond_names, use_conds)
    ds_test = SpeedDataset(
        Sub, use_sub_indices=test_sub_indices, L=cfg["L"], S=cfg["S"],
        use_cond_ids=cond_ids, fit_norm=False, mu=mu, sd=sd, apply_norm=True
    )
    if len(ds_test) == 0:
        raise RuntimeError(f"테스트 샘플이 없습니다. test_subs/use_conds/L/S 확인 (use_conds={use_conds})")
    tdl = torch.utils.data.DataLoader(
        ds_test, batch_size=cfg.get("val_batch_size", 1024), shuffle=False, num_workers=0,
        pin_memory=(device.type=="cuda")
    )
    return ds_test, tdl

def make_model_from_ckpt(ckpt, input_dim_current, cfg, device):
    arch = ckpt.get("arch", cfg.get("arch", "TCN_LSTM_MHA"))
    hp   = ckpt.get("hp", {})
    input_dim_ckpt = int(ckpt.get("input_dim", input_dim_current))
    strict_flag = (input_dim_ckpt == input_dim_current)
    try:
        model = build_model_by_arch(arch, input_dim_current, hp, p=cfg["dropout_p"]).to(device)
    except Exception:
        model = SpeedNet(
            input_dim_current,
            d_model=cfg.get("d_model",192),
            lstm_h=cfg.get("lstm_h",512),
            out_dim=cfg.get("out_dim",2),
            p=cfg["dropout_p"],
            attn_heads=cfg.get("attn_heads",1)
        ).to(device)
    state_key = "model_state" if "model_state" in ckpt else "state_dict"
    model.load_state_dict(ckpt[state_key], strict=strict_flag)
    return model, arch, hp, strict_flag, input_dim_ckpt

# --------- 라벨 압축 + 막대그래프 개선 공통 함수 ---------
def _compact_label(ckpt_name, arch_hint=None):
    """
    'SE_TCN_GRU_L160_S2_blocks2_d96_h128_kset5x3.pt' ->
    'TCN_GRU|L160_S2|blocks2_d96_h128_kset5x3'
    """
    base = ckpt_name.replace(".pt","")
    m = re.match(r"SE_(.+?)_(L\d+_S\d+)_(.+)", base)
    if m:
        arch, ls, tag = m.group(1), m.group(2), m.group(3)
        return f"{arch}|{ls}|{tag}"
    # fallback
    return f"{arch_hint or '?'}|{base}"

def _apply_compact_style():
    plt.rcParams.update({
        "font.size": 8,
        "axes.labelsize": 9,
        "xtick.labelsize": 7,
        "ytick.labelsize": 7,
        "legend.fontsize": 7
    })

def plot_rmse_bars_all(records, out_png):
    """
    records: list of dicts with keys:
      {"ckpt": "...\\SE_TCN_GRU_L160_S2_....pt", "arch": "TCN_GRU", "rmse_total": 0.123}
    """
    if not records:
        print("[warn] no records for plot_rmse_bars_all")
        return

    # 정렬
    recs = sorted(records, key=lambda r: r["rmse_total"])
    labels = [_compact_label(os.path.basename(r["ckpt"]), r.get("arch")) for r in recs]
    values = [float(r["rmse_total"]) for r in recs]

    N = len(values)
    _apply_compact_style()

    # 막대 수에 비례한 가로폭
    per_bar_inch = 0.24
    fig_w = max(12, min(36, N * per_bar_inch))
    fig_h = 5.0

    fig, ax = plt.subplots(figsize=(fig_w, fig_h))
    x = np.arange(N)
    ax.bar(x, values, width=0.8, linewidth=0)
    ax.set_xlim(-0.5, N - 0.5)
    ax.set_ylabel("RMSE (avg of vY,vZ)")
    ax.set_xlabel("Model variants")
    ax.set_title(f"All checkpoints RMSE (N={N})", loc="left")

    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=90, ha="center")
    ax.grid(axis="y", linestyle=":", linewidth=0.5)
    ax.margins(x=0.002)

    # 성능 지표 식 표시
    fig.text(
        0.99, 0.01, METRIC_FORMULAS,
        ha="right", va="bottom", fontsize=7,
        family="monospace",
        bbox=dict(boxstyle="round", alpha=0.1)
    )

    fig.tight_layout()
    fig.savefig(out_png, dpi=200, bbox_inches="tight")
    plt.close(fig)

def plot_rmse_bars_by_arch(best_by_arch_list, out_png):
    """
    best_by_arch_list: list of tuples (arch_label, rmse_total)
    """
    if not best_by_arch_list:
        print("[warn] no records for plot_rmse_bars_by_arch")
        return

    _apply_compact_style()
    # 성능 오름차순
    best_by_arch_list = sorted(best_by_arch_list, key=lambda x: x[1])
    labels = [a for a, _ in best_by_arch_list]
    values = [v for _, v in best_by_arch_list]

    N = len(values)
    fig_w = max(8, 1.6 * N)
    fig_h = 4.5
    fig, ax = plt.subplots(figsize=(fig_w, fig_h))
    x = np.arange(N)
    ax.bar(x, values, width=0.8, linewidth=0)
    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=0, ha="center")
    ax.set_ylabel("RMSE (avg of vY,vZ)")
    ax.set_title("Best-of-architecture: Test RMSE", loc="left")
    ax.grid(axis="y", linestyle=":", linewidth=0.5)

    # 성능 지표 식 표시
    fig.text(
        0.99, 0.01, METRIC_FORMULAS,
        ha="right", va="bottom", fontsize=7,
        family="monospace",
        bbox=dict(boxstyle="round", alpha=0.1)
    )

    fig.tight_layout()
    fig.savefig(out_png, dpi=200, bbox_inches="tight")
    plt.close(fig)

# --------- 최고 성능 모델: cond별 20초(≈2000 window) 플롯 ---------
def plot_bestmodel_allconds_20s(best_ckpt_path, Sub, cfg, spec, device, stamp):
    """
    rmse_total이 가장 좋은 checkpoint 하나에 대해,
    cfg['use_conds']에 포함된 각 cond마다
    True/Pred vY, vZ를 약 20초 구간(여기서는 2000 window)만 플롯.
    """
    ckpt = torch.load(best_ckpt_path, map_location=device)
    mu, sd = ckpt["mu"], ckpt["sd"]

    use_conds_all = cfg["use_conds"]  # 예: ["level_08mps", "accel_sine", ...]
    top_label = _compact_label(os.path.basename(best_ckpt_path), ckpt.get("arch", None))

    # 단위 정보
    dim_vy = DEFAULT_FEAT_SPEC["target"].get("dimless_vy", True)
    dim_vz = DEFAULT_FEAT_SPEC["target"].get("dimless_vz", True)
    uy = "(dimless)" if dim_vy else "(m/s)"
    uz = "(dimless)" if dim_vz else "(m/s)"

    # 약 20초 분량
    N = 2000

    rows = len(use_conds_all)
    cols = 2
    fig, axes = plt.subplots(rows, cols, figsize=(14, 2.6*rows), sharex=False)
    if rows == 1:
        axes = np.array([axes])

    for r_i, cond_name in enumerate(use_conds_all):
        ds_test, tdl = build_test_loader(Sub, cfg, mu, sd, device, spec, use_conds=[cond_name])
        model, arch, hp, strict_flag, _ = make_model_from_ckpt(ckpt, ds_test.X.shape[2], cfg, device)
        P, G = stack_preds(tdl, model, device)
        if len(P) == 0:
            continue

        n = min(N, len(P))
        xs = np.arange(n)
        title = f"{cond_name} | {top_label}"

        ax1 = axes[r_i, 0]
        ax1.plot(xs, G[:n,0], label=f"True vY {uy}")
        ax1.plot(xs, P[:n,0], label=f"Pred vY {uy}")
        ax1.grid(True, linestyle=":", linewidth=0.5)
        if r_i == 0:
            ax1.set_title("vY")
        ax1.set_ylabel(title, fontsize=9)
        if r_i == rows-1:
            ax1.set_xlabel("window idx")

        ax2 = axes[r_i, 1]
        ax2.plot(xs, G[:n,1], label=f"True vZ {uz}")
        ax2.plot(xs, P[:n,1], label=f"Pred vZ {uz}")
        ax2.grid(True, linestyle=":", linewidth=0.5)
        if r_i == 0:
            ax2.set_title("vZ")
        if r_i == rows-1:
            ax2.set_xlabel("window idx")

    handles, labels_legend = axes[0,0].get_legend_handles_labels()
    fig.legend(handles, labels_legend, loc="upper center", ncol=4)
    plt.tight_layout(rect=[0, 0, 1, 0.96])

    out_path = os.path.join(SAVE_DIR, f"bestmodel_allconds_20s_{stamp}.png")
    plt.savefig(out_path, dpi=200, bbox_inches="tight")
    plt.close(fig)

    return out_path

# --------- 베스트 ckpt: cond별 RMSE% / NRMSE% 테이블 ---------
def write_best_cond_table(best_ckpt_path, Sub, cfg, spec, device, stamp):
    """
    베스트 체크포인트 1개에 대해 cond별 RMSE% / NRMSE%를
    vY, vZ 각각 따로 표로 저장.

    CSV 형태:
        ,accel_sine,asym_30deg,...,Total
        NRMSE% vY, ...
        NRMSE% vZ, ...
        RMSE% vY,  ...
        RMSE% vZ,  ...
    """
    ckpt = torch.load(best_ckpt_path, map_location=device)
    mu, sd = ckpt["mu"], ckpt["sd"]
    cond_list = cfg["use_conds"]

    # cond별 값 저장용
    nrmse_vy_list = []
    nrmse_vz_list = []
    rmse_vy_list  = []
    rmse_vz_list  = []

    for cond_name in cond_list:
        ds_c, tdl_c = build_test_loader(Sub, cfg, mu, sd, device, spec,
                                        use_conds=[cond_name])
        model, arch, hp, strict_flag, _ = make_model_from_ckpt(
            ckpt, ds_c.X.shape[2], cfg, device
        )
        P_c, G_c = stack_preds(tdl_c, model, device)
        stat = rmse_and_norm_perc(P_c, G_c)

        rmse_vy = float(stat["rmse_pct"][0])
        rmse_vz = float(stat["rmse_pct"][1])
        nrmse_vy = float(stat["nrmse_pct"][0])
        nrmse_vz = float(stat["nrmse_pct"][1])

        rmse_vy_list.append(rmse_vy)
        rmse_vz_list.append(rmse_vz)
        nrmse_vy_list.append(nrmse_vy)
        nrmse_vz_list.append(nrmse_vz)

    # 각 metric의 cond 평균 (Total)
    def _mean(lst):
        arr = np.array(lst, dtype=float)
        arr = arr[~np.isnan(arr)]
        if arr.size == 0:
            return np.nan
        return float(arr.mean())

    total_nrmse_vy = _mean(nrmse_vy_list)
    total_nrmse_vz = _mean(nrmse_vz_list)
    total_rmse_vy  = _mean(rmse_vy_list)
    total_rmse_vz  = _mean(rmse_vz_list)

    out_csv_best = os.path.join(SAVE_DIR, f"eval_bycond_best_{stamp}.csv")
    with open(out_csv_best, "w", encoding="utf-8") as f:
        # 헤더: 공백 + cond들 + Total
        header = ["", *cond_list, "Total"]
        f.write(",".join(header) + "\n")

        row1 = ["NRMSE% vY"] + [f"{v:.2f}" for v in nrmse_vy_list] + [f"{total_nrmse_vy:.2f}"]
        row2 = ["NRMSE% vZ"] + [f"{v:.2f}" for v in nrmse_vz_list] + [f"{total_nrmse_vz:.2f}"]
        row3 = ["RMSE% vY"]  + [f"{v:.2f}" for v in rmse_vy_list]  + [f"{total_rmse_vy:.2f}"]
        row4 = ["RMSE% vZ"]  + [f"{v:.2f}" for v in rmse_vz_list]  + [f"{total_rmse_vz:.2f}"]

        f.write(",".join(row1) + "\n")
        f.write(",".join(row2) + "\n")
        f.write(",".join(row3) + "\n")
        f.write(",".join(row4) + "\n")

    return out_csv_best


# ---------------- main ----------------
def main():
    cfg  = CONFIG
    spec = DEFAULT_FEAT_SPEC
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    # 데이터 1회 로드·전처리
    Sub = load_all_subjects(path=cfg["path"], sub_names=cfg["sub_names"],
                            cond_names=cfg["cond_names"], data_length=cfg["data_length"])
    count_and_fix_nans_in_Sub(Sub)
    preprocess_back_imu(Sub, pitch_is_deg=True)
    for i, D in Sub.items():
        m = float(D.get('mass_kg', np.nan))
        normalize_fsr_by_mass(Sub, i, m)
        if spec["include"].get("grf", False):
            normalize_grf_by_mass(Sub, i, m)
    apply_tick_filter(Sub, passes=2)

    # 입력 차원 파악
    _test_idx = _names_to_indices(cfg["sub_names"], cfg.get("test_subs", []))
    _cond_ids = _conds_to_ids(cfg["cond_names"], cfg["use_conds"])
    ds_dummy = SpeedDataset(Sub, use_sub_indices=_test_idx, L=cfg["L"], S=cfg["S"],
                            use_cond_ids=_cond_ids, fit_norm=True)
    F_current = ds_dummy.X.shape[2] if len(ds_dummy) > 0 else None
    if F_current is None:
        raise RuntimeError("입력 차원 산출 실패")

    # 체크포인트 수집
    ckpts = sorted(glob.glob(os.path.join(CKPT_DIR, CKPT_PATTERN)))
    if not ckpts:
        raise SystemExit(f"평가할 체크포인트 없음: {CKPT_DIR}\\{CKPT_PATTERN}")

    # 평가 루프
    stamp = time.strftime("%Y%m%d_%H%M%S")
    out_csv = os.path.join(SAVE_DIR, f"eval_summary_{stamp}.csv")
    with open(out_csv, "w", encoding="utf-8") as f:
        f.write("ckpt,arch,hp,input_dim_ckpt,input_dim_cur,n,rmse_total,rmse_vY,rmse_vZ,mse_vY,mse_vZ,mae_vY,mae_vZ,strict\n")

    # cond별 RMSE%, NRMSE%를 테이블 형태로 저장할 CSV (ckpt별·vY/vZ 분리)
    cond_list = cfg["use_conds"]
    out_csv_cond = os.path.join(SAVE_DIR, f"eval_bycond_{stamp}.csv")
    with open(out_csv_cond, "w", encoding="utf-8") as f:
        header = ["ckpt", "metric"] + cond_list + ["Total"]
        f.write(",".join(header) + "\n")

    results = []
    for ckpt_path in tqdm(ckpts, desc="Evaluate"):
        try:
            ckpt = torch.load(ckpt_path, map_location=device)
            mu, sd = ckpt["mu"], ckpt["sd"]

            # 전체 test set
            ds_test, tdl = build_test_loader(Sub, cfg, mu, sd, device, spec)
            model, arch, hp, strict_flag, input_dim_ckpt = make_model_from_ckpt(ckpt, ds_test.X.shape[2], cfg, device)

            P, G = stack_preds(tdl, model, device)
            mets = metrics(P, G)

            results.append({
                "ckpt": ckpt_path,
                "name": os.path.basename(ckpt_path),
                "arch": arch,
                "hp": hp,
                "rmse_total": mets["rmse_total"],
                "rmse": mets["rmse"],
                "mse":  mets["mse"],
                "mae":  mets["mae"],
                "n": mets["n"],
            })

            # 기본 summary CSV
            with open(out_csv, "a", encoding="utf-8") as f:
                f.write("{},{},{},{},{},{},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{}\n".format(
                    os.path.basename(ckpt_path),
                    arch,
                    json.dumps(hp, ensure_ascii=False),
                    input_dim_ckpt,
                    ds_test.X.shape[2],
                    mets["n"],
                    mets["rmse_total"],
                    mets["rmse"][0], mets["rmse"][1],
                    mets["mse"][0],  mets["mse"][1],
                    mets["mae"][0],  mets["mae"][1],
                    int(strict_flag)
                ))

            # ---- cond별 RMSE%, NRMSE% 계산 (ckpt별·vY/vZ 분리용 CSV) ----
            cond_stats = {}
            for cond_name in cond_list:
                ds_c, tdl_c = build_test_loader(Sub, cfg, mu, sd, device, spec, use_conds=[cond_name])
                P_c, G_c = stack_preds(tdl_c, model, device)
                stat = rmse_and_norm_perc(P_c, G_c)
                cond_stats[cond_name] = stat

            def _mean_over_conds(key, idx):
                vals = []
                for c in cond_list:
                    v = cond_stats[c][key][idx]
                    if not np.isnan(v):
                        vals.append(v)
                if not vals:
                    return np.nan
                return float(np.mean(vals))

            rows_to_write = []
            # vY
            rmse_vy = [float(cond_stats[c]["rmse_pct"][0]) for c in cond_list]
            nrmse_vy = [float(cond_stats[c]["nrmse_pct"][0]) for c in cond_list]
            rows_to_write.append(("RMSE%_vY", rmse_vy, _mean_over_conds("rmse_pct", 0)))
            rows_to_write.append(("NRMSE%_vY", nrmse_vy, _mean_over_conds("nrmse_pct", 0)))
            # vZ
            rmse_vz = [float(cond_stats[c]["rmse_pct"][1]) for c in cond_list]
            nrmse_vz = [float(cond_stats[c]["nrmse_pct"][1]) for c in cond_list]
            rows_to_write.append(("RMSE%_vZ", rmse_vz, _mean_over_conds("rmse_pct", 1)))
            rows_to_write.append(("NRMSE%_vZ", nrmse_vz, _mean_over_conds("nrmse_pct", 1)))

            with open(out_csv_cond, "a", encoding="utf-8") as f:
                for metric_name, vals, tot in rows_to_write:
                    vals_str = [f"{v:.2f}" for v in vals]
                    line = [os.path.basename(ckpt_path), metric_name] + vals_str + [f"{tot:.2f}"]
                    f.write(",".join(line) + "\n")

            mean_nrmse_vY = _mean_over_conds("nrmse_pct", 0)
            mean_nrmse_vZ = _mean_over_conds("nrmse_pct", 1)
            print(f"{os.path.basename(ckpt_path)} | mean NRMSE% vY={mean_nrmse_vY:.2f} | vZ={mean_nrmse_vZ:.2f}")

        except Exception as e:
            with open(out_csv, "a", encoding="utf-8") as f:
                f.write(f"{os.path.basename(ckpt_path)},-,-,-,-,0,NaN,NaN,NaN,NaN,NaN,NaN,NaN,error:{str(e).replace(',',';')}\n")

    if not results:
        print("유효한 평가 결과가 없습니다.")
        return

    # ===== 그림 1: 전 모델 RMSE_total bar =====
    bar_all_path = os.path.join(SAVE_DIR, f"eval_bar_rmse_{stamp}.png")
    plot_rmse_bars_all(results, bar_all_path)

    # ===== 그림 2: 상위 5개 True/Pred 그리드 =====
    order = np.argsort([r["rmse_total"] for r in results])
    topk = min(5, len(results))
    top_idx = order[:topk]
    rows, cols = topk, 2
    fig, axes = plt.subplots(rows, cols, figsize=(14, 2.6*rows), sharex=False)
    if rows == 1:
        axes = np.array([axes])
    N = 2000
    dim_vy = DEFAULT_FEAT_SPEC["target"].get("dimless_vy", True)
    dim_vz = DEFAULT_FEAT_SPEC["target"].get("dimless_vz", True)
    uy = "(dimless)" if dim_vy else "(m/s)"
    uz = "(dimless)" if dim_vz else "(m/s)"

    for r_i, idx in enumerate(top_idx):
        ckpt_path = results[idx]["ckpt"]
        ckpt = torch.load(ckpt_path, map_location=device)
        mu, sd = ckpt["mu"], ckpt["sd"]
        ds_test, tdl = build_test_loader(Sub, cfg, mu, sd, device, spec)
        model, arch, hp, strict_flag, _ = make_model_from_ckpt(ckpt, ds_test.X.shape[2], cfg, device)
        P, G = stack_preds(tdl, model, device)

        n = min(N, len(P))
        xs = np.arange(n)
        title = f"{_compact_label(os.path.basename(ckpt_path), arch)} | RMSE={results[idx]['rmse_total']:.4f}"

        ax = axes[r_i, 0]
        ax.plot(xs, G[:n,0], label=f"True vY {uy}")
        ax.plot(xs, P[:n,0], label=f"Pred vY {uy}")
        ax.grid(True, linestyle=":", linewidth=0.5)
        if r_i == 0: ax.set_title("vY")
        ax.set_ylabel(title, fontsize=9)
        if r_i == rows-1: ax.set_xlabel("window idx")

        ax2 = axes[r_i, 1]
        ax2.plot(xs, G[:n,1], label=f"True vZ {uz}")
        ax2.plot(xs, P[:n,1], label=f"Pred vZ {uz}")
        ax2.grid(True, linestyle=":", linewidth=0.5)
        if r_i == 0: ax2.set_title("vZ")
        if r_i == rows-1: ax2.set_xlabel("window idx")

    handles, labels_legend = axes[0,0].get_legend_handles_labels()
    fig.legend(handles, labels_legend, loc="upper center", ncol=4)
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    grid_top5_path = os.path.join(SAVE_DIR, f"top5_true_vs_pred_{stamp}.png")
    plt.savefig(grid_top5_path, dpi=200, bbox_inches="tight")
    plt.close()

    # ===== 그림 3,4: 아키텍처별 최고 성능 집계 =====
    best_by_arch_idx = {}
    for i, r in enumerate(results):
        a = r["arch"]
        if a not in best_by_arch_idx or r["rmse_total"] < results[best_by_arch_idx[a]]["rmse_total"]:
            best_by_arch_idx[a] = i

    target_arch_order = ["TCN_GRU","GRU2","TinyTCN","TCN_LSTM","SmallTCN_LSTM_Attn"]
    picked = [(a, best_by_arch_idx[a]) for a in target_arch_order if a in best_by_arch_idx]
    if picked:
        arch_best = [(a, results[i]["rmse_total"]) for a, i in picked]
        bar_archbest_path = os.path.join(SAVE_DIR, f"eval_bar_arch_best_{stamp}.png")
        plot_rmse_bars_by_arch(arch_best, bar_archbest_path)

        ord2 = np.argsort([results[i]["rmse_total"] for _, i in picked])
        rows2, cols2 = len(picked), 2
        fig2, axes2 = plt.subplots(rows2, cols2, figsize=(14, 2.6*rows2), sharex=False)
        if rows2 == 1:
            axes2 = np.array([axes2])

        for r_i, j in enumerate(ord2):
            arch, idx = picked[j]
            ckpt_path = results[idx]["ckpt"]
            ckpt = torch.load(ckpt_path, map_location=device)
            mu, sd = ckpt["mu"], ckpt["sd"]
            ds_test, tdl = build_test_loader(Sub, cfg, mu, sd, device, spec)
            model, arch2, hp, strict_flag, _ = make_model_from_ckpt(ckpt, ds_test.X.shape[2], cfg, device)
            P, G = stack_preds(tdl, model, device)

            n = min(2000, len(P))
            xs = np.arange(n)
            title = f"{arch2} | {_compact_label(os.path.basename(ckpt_path), arch2)} | RMSE={results[idx]['rmse_total']:.4f}"

            a1 = axes2[r_i, 0]
            a1.plot(xs, G[:n,0], label=f"True vY {uy}")
            a1.plot(xs, P[:n,0], label=f"Pred vY {uy}")
            a1.grid(True, linestyle=":", linewidth=0.5)
            if r_i == 0: a1.set_title("vY")
            a1.set_ylabel(title, fontsize=9)
            if r_i == rows2-1: a1.set_xlabel("window idx")

            a2 = axes2[r_i, 1]
            a2.plot(xs, G[:n,1], label=f"True vZ {uz}")
            a2.plot(xs, P[:n,1], label=f"Pred vZ {uz}")
            a2.grid(True, linestyle=":", linewidth=0.5)
            if r_i == 0: a2.set_title("vZ")
            if r_i == rows2-1: a2.set_xlabel("window idx")

        hnd, lab = axes2[0,0].get_legend_handles_labels()
        fig2.legend(hnd, lab, loc="upper center", ncol=4)
        plt.tight_layout(rect=[0, 0, 1, 0.96])
        grid_archbest_path = os.path.join(SAVE_DIR, f"archbest_true_vs_pred_{stamp}.png")
        plt.savefig(grid_archbest_path, dpi=200, bbox_inches="tight")
        plt.close()

        print(f"[DONE] Best-of-arch bar: {bar_archbest_path}")
        print(f"[DONE] Best-of-arch grid: {grid_archbest_path}")

    # ===== 최고 성능 모델 1개에 대해, cond별 20초 플롯 + cond별 RMSE/NRMSE 표 =====
    best_idx = int(np.argmin([r["rmse_total"] for r in results]))
    best_ckpt_path = results[best_idx]["ckpt"]
    best_allconds_path = plot_bestmodel_allconds_20s(best_ckpt_path, Sub, cfg, spec, device, stamp)
    best_cond_table_path = write_best_cond_table(best_ckpt_path, Sub, cfg, spec, device, stamp)

    print(f"[DONE] CSV (summary): {out_csv}")
    print(f"[DONE] CSV (by-cond, detailed): {out_csv_cond}")
    print(f"[DONE] CSV (by-cond, best model table): {best_cond_table_path}")
    print(f"[DONE] All-model bar: {bar_all_path}")
    print(f"[DONE] Top-5 grid: {grid_top5_path}")
    print(f"[DONE] Best-model all-conds 20s grid: {best_allconds_path}")

if __name__ == "__main__":
    main()
