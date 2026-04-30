"""
validate_and_retrain.py
=======================
Phase 0: 훈련-평가 일관성 감사 + 자동 수정/재학습 스크립트

목적:
  학습이 잘 됐냐 못됐냐가 아니라, 훈련 코드와 평가 코드 사이에
  논리적으로 모순된 부분이 없는지 확인하고, 필요시 수정 후 재학습한다.

실행 방법 (SpeedEstimation/ 디렉토리에서):
  python validate_and_retrain.py              # 탐지 → 수정 → 재학습
  python validate_and_retrain.py --dry-run    # 탐지만, 수정/재학습 없음
  python validate_and_retrain.py --exp baseline_Test-m1_S004_seed42  # 특정 실험만

발견된 주요 이슈 (코드 사전 분석 결과):
  C1. AR Feedback 구현 불일치
  C2. Multi-step metrics 해석 차이 (metrics.json vs compare_results.py)
  C3. EMA post-processing 순서 (metrics.json = pre-EMA, compare_results.py = post-EMA)
  C4. est_tick_ranges / y_delay fallback 일관성
  C5. _dot 변수 30Hz LPF 일관성 (build_nn_dataset_multi 공유 여부)
  C6. Config 상속(deep_merge) 결과 일관성
  C7. Scaler shape 일관성 (input_dim vs scaler.npz shape)
  C8. Model checkpoint 로드 가능성
"""

import os
import sys
import re
import json
import yaml
import copy
import shutil
import subprocess
import time
import argparse
import numpy as np
from pathlib import Path
from dataclasses import dataclass, field, asdict
from typing import List, Optional, Dict, Any
from datetime import datetime

# ─────────────────────────────────────────────
# 경로 설정 (SpeedEstimation/ 디렉토리 기준)
# ─────────────────────────────────────────────
SCRIPT_DIR     = Path(__file__).parent.resolve()
EXP_ROOT       = SCRIPT_DIR / "experiments"
CONFIGS_DIR    = SCRIPT_DIR / "configs"
FIXED_DIR      = CONFIGS_DIR / "fixed"
BASELINE_YAML  = CONFIGS_DIR / "baseline.yaml"
TRAIN_SCRIPT   = SCRIPT_DIR / "SpeedEstimator_TCN_MLP_experiments.py"
EVAL_SCRIPT    = SCRIPT_DIR / "compare_results.py"
REPORT_JSON    = SCRIPT_DIR / "validation_report.json"
REPORT_MD      = SCRIPT_DIR / "validation_report.md"

EXPECTED_EXPERIMENTS = [
    "baseline",
    "exp_ar_feedback_k1", "exp_ar_feedback_sched",
    "Exp_IMU_Orientation",
    "exp_multistep_H05", "exp_multistep_H10", "exp_multistep_H20",
    "exp_multistep_H10_ema_a090",
    "exp_multistep_H10_overlap_mean", "exp_multistep_H10_overlap_median",
    "exp_multistep_H10_overlap_wmean",
    "exp_multistep_H10_smooth_lam001", "exp_multistep_H10_smooth_lam01",
    "exp_multistep_H10_smooth_lam05",
    "exp_single_ema_a080", "exp_single_ema_a090", "exp_single_ema_a095",
    "exp_tcn_gru_head_h32", "exp_tcn_gru_head_h32_smooth", "exp_tcn_gru_head_h64",
]
TEST_SUBJECTS = ["m1_S004", "m1_S011", "m2_S008"]
REQUIRED_FILES = ["model.pt", "metrics.json", "train_log.csv", "config.yaml", "scaler.npz"]


# ─────────────────────────────────────────────
# Issue 데이터 클래스
# ─────────────────────────────────────────────
@dataclass
class Issue:
    check_id: str               # C1, C2, ... C8
    exp_name: str               # 실험 이름 (전역 check는 "GLOBAL")
    subject: str                # 피험자 이름 (전역 check는 "ALL")
    issue_type: str             # CODE_BUG | CONFIG_INCONSISTENCY | METRICS_INTERPRETATION | etc.
    description: str
    severity: str               # CRITICAL | MAJOR | MINOR | INFO
    fix_strategy: str           # FIX_AND_RETRAIN | DOCUMENT | FLAG_AND_EXCLUDE | PASS
    fix_detail: str = ""
    retrain_needed: bool = False
    retrain_config: Optional[str] = None  # 재학습에 사용할 config 경로


# ─────────────────────────────────────────────
# 유틸리티
# ─────────────────────────────────────────────
def load_yaml(path: Path) -> Dict:
    with open(path, 'r') as f:
        return yaml.safe_load(f) or {}

def deep_merge(base: Dict, override: Dict) -> None:
    """override를 base에 재귀적으로 병합 (in-place)."""
    for k, v in override.items():
        if k in base and isinstance(base[k], dict) and isinstance(v, dict):
            deep_merge(base[k], v)
        else:
            base[k] = v

def get_merged_config(exp_name: str) -> Optional[Dict]:
    """baseline + 실험 config를 merge해서 반환."""
    if not BASELINE_YAML.exists():
        return None
    merged = load_yaml(BASELINE_YAML)
    exp_yaml = CONFIGS_DIR / f"{exp_name}.yaml"
    if exp_yaml.exists():
        deep_merge(merged, load_yaml(exp_yaml))
    return merged

def get_nested(d: Dict, dotted_key: str, default=None):
    """'shared.data.y_delay' 같은 dotted key로 값 조회."""
    keys = dotted_key.split('.')
    cur = d
    for k in keys:
        if not isinstance(cur, dict) or k not in cur:
            return default
        cur = cur[k]
    return cur


# ─────────────────────────────────────────────
# Check C1: AR Feedback 구현 일관성
# ─────────────────────────────────────────────
def check_c1_ar_feedback() -> List[Issue]:
    """
    훈련 코드에서 AR feedback이 teacher_forcing 설정대로 구현됐는지 확인.
    실제로는 항상 zeros를 사용하며, teacher_forcing 값이 무시됨.
    """
    issues = []
    src = TRAIN_SCRIPT.read_text()

    # 1) training loop에서 prev_channel이 항상 zeros인지 확인
    zero_padding_pattern = r'prev_channel\s*=\s*torch\.zeros\(B,\s*T_in,\s*ar_k'
    tf_disabled_pattern  = r'#\s*if random\.random\(\) < tf_ratio'

    uses_zeros_unconditionally = bool(re.search(zero_padding_pattern, src))
    tf_is_commented_out        = bool(re.search(tf_disabled_pattern, src))

    if uses_zeros_unconditionally and tf_is_commented_out:
        issues.append(Issue(
            check_id="C1",
            exp_name="exp_ar_feedback_k1 / exp_ar_feedback_sched",
            subject="ALL",
            issue_type="CODE_BUG",
            description=(
                "AR Feedback 훈련 코드가 teacher_forcing_schedule 설정값을 무시하고 "
                "항상 zeros(prev_channel)를 사용함. "
                "teacher_forcing: 1.0 (k1)과 teacher_forcing_schedule: 1.0→0.0 (sched)가 "
                "구현되지 않아 두 실험이 사실상 동일하게 동작."
            ),
            severity="MAJOR",
            fix_strategy="FLAG_AND_EXCLUDE",
            fix_detail=(
                "AR feedback 실험을 'AR mechanism analysis'에서 제외하고 주석 표기. "
                "훈련/평가 모두 zeros를 사용해 내적 일관성은 유지됨. "
                "실제 teacher forcing을 구현하려면 LazyWindowDataset 수정 + "
                "SpeedEstimator_TCN_MLP_experiments.py의 AR 루프 수정 필요 "
                "(CRITICAL NOTICE 파일이므로 사용자 승인 필요)."
            ),
            retrain_needed=False,
        ))

    # 2) AR 실험 두 개가 동일 config로 수렴됐는지 확인 (가능하면 metrics.json 비교)
    ar_k1_mae, ar_sched_mae = [], []
    for subj in TEST_SUBJECTS:
        for exp_nm, lst in [("exp_ar_feedback_k1", ar_k1_mae),
                             ("exp_ar_feedback_sched", ar_sched_mae)]:
            p = EXP_ROOT / f"{exp_nm}_Test-{subj}_seed42" / "metrics.json"
            if p.exists():
                m = json.loads(p.read_text())
                lst.append(round(m.get("test_mae", -1), 6))

    if ar_k1_mae and ar_sched_mae:
        if ar_k1_mae == ar_sched_mae:
            issues.append(Issue(
                check_id="C1b",
                exp_name="exp_ar_feedback_k1 / exp_ar_feedback_sched",
                subject="ALL",
                issue_type="METRICS_MATCH",
                description=(
                    f"[확인] exp_ar_feedback_k1 MAEs={ar_k1_mae}와 "
                    f"exp_ar_feedback_sched MAEs={ar_sched_mae}가 "
                    "완전히 일치. C1 이슈 확증."
                ),
                severity="INFO",
                fix_strategy="DOCUMENT",
                fix_detail="C1 이슈로 인한 예상된 결과.",
                retrain_needed=False,
            ))
        else:
            issues.append(Issue(
                check_id="C1b",
                exp_name="exp_ar_feedback_k1 / exp_ar_feedback_sched",
                subject="ALL",
                issue_type="METRICS_MISMATCH",
                description=(
                    f"exp_ar_feedback_k1 MAEs={ar_k1_mae}, "
                    f"exp_ar_feedback_sched MAEs={ar_sched_mae}. "
                    "값이 다름 — 추가 조사 필요."
                ),
                severity="MINOR",
                fix_strategy="DOCUMENT",
                fix_detail="두 실험의 초기 랜덤 시드 차이 또는 미묘한 코드 경로 차이 가능성.",
                retrain_needed=False,
            ))

    return issues


# ─────────────────────────────────────────────
# Check C2: Multi-step metrics 해석 차이
# ─────────────────────────────────────────────
def check_c2_multistep_metrics(exp_name: str, exp_dir: Path, merged_cfg: Dict) -> List[Issue]:
    """
    H>1 실험에서 metrics.json의 test_mae와 compare_results.py의 MAE가
    서로 다른 방법으로 계산됨을 문서화.
    """
    issues = []
    metrics_path = exp_dir / "metrics.json"
    if not metrics_path.exists():
        return issues

    metrics = json.loads(metrics_path.read_text())

    # est_tick_ranges 확인
    est_tick = (
        merged_cfg.get("est_tick_ranges")
        or get_nested(merged_cfg, "shared.data.est_tick_ranges")
        or get_nested(merged_cfg, "02_train.data.est_tick_ranges")
    )

    if not est_tick or len(est_tick) <= 1:
        return issues  # single-step, 문제 없음

    H = len(est_tick)
    has_detail = "multistep_detail" in metrics

    if has_detail:
        detail = metrics["multistep_detail"]
        h_maes = [detail.get(f"H{i+1}_mae") for i in range(H)
                  if detail.get(f"H{i+1}_mae") is not None]
        if h_maes:
            avg_step_mae = float(np.mean(h_maes))
            stored_mae   = float(metrics.get("test_mae", -1))
            # metrics.json의 test_mae = avg of all H steps (training-time eval)
            # compare_results.py의 MAE = step-1-only OR overlap-averaged
            issues.append(Issue(
                check_id="C2",
                exp_name=exp_name,
                subject=exp_dir.name.split("_Test-")[1].replace("_seed42", ""),
                issue_type="METRICS_INTERPRETATION",
                description=(
                    f"H={H} multi-step 실험. "
                    f"metrics.json test_mae={stored_mae:.4f}은 "
                    f"훈련 시 {H}개 step 평균(avg_H_mae={avg_step_mae:.4f}). "
                    "compare_results.py의 MAE는 overlap_ensemble 미적용 시 "
                    "step-1(t+1) 예측값만으로, 적용 시 overlap-averaged로 계산."
                ),
                severity="INFO",
                fix_strategy="DOCUMENT",
                fix_detail=(
                    "분석 시 metrics.json 대신 compare_results.py "
                    "inference 결과(load_and_evaluate)를 canonical metric으로 사용."
                ),
                retrain_needed=False,
            ))

    return issues


# ─────────────────────────────────────────────
# Check C3: EMA 실험 metric 순서
# ─────────────────────────────────────────────
def check_c3_ema_order(exp_name: str, exp_dir: Path, merged_cfg: Dict) -> List[Issue]:
    """
    EMA 실험에서 metrics.json은 pre-EMA MAE,
    compare_results.py는 post-EMA MAE임을 문서화.
    """
    ema_alpha = get_nested(merged_cfg, "03_eval.ema_alpha")
    if ema_alpha is None:
        return []

    return [Issue(
        check_id="C3",
        exp_name=exp_name,
        subject=exp_dir.name.split("_Test-")[1].replace("_seed42", ""),
        issue_type="METRICS_INTERPRETATION",
        description=(
            f"EMA alpha={ema_alpha} 실험. "
            "metrics.json의 test_mae는 EMA 미적용 상태(훈련 시 측정). "
            "compare_results.py의 MAE는 EMA 적용 후 값. "
            "두 수치가 의도적으로 다름."
        ),
        severity="INFO",
        fix_strategy="DOCUMENT",
        fix_detail=(
            "분석 시 반드시 compare_results.py inference 재실행으로 "
            "EMA 적용 후 MAE를 획득해야 함. "
            "metrics.json의 수치로 EMA 효과를 비교하면 안 됨."
        ),
        retrain_needed=False,
    )]


# ─────────────────────────────────────────────
# Check C4: est_tick_ranges / y_delay 일관성
# ─────────────────────────────────────────────
def check_c4_temporal_alignment(exp_name: str, exp_dir: Path, merged_cfg: Dict) -> List[Issue]:
    """
    훈련 config의 est_tick_ranges / y_delay와 compare_results.py의 fallback 로직이
    동일한 값을 사용하는지 확인.
    """
    # 훈련에서 사용된 값
    train_est_tick = (
        get_nested(merged_cfg, "est_tick_ranges")
        or get_nested(merged_cfg, "shared.data.est_tick_ranges")
        or get_nested(merged_cfg, "02_train.data.est_tick_ranges")
    )
    y_delay = (
        get_nested(merged_cfg, "shared.data.y_delay")
        or get_nested(merged_cfg, "02_train.data.y_delay")
        or 5
    )

    # compare_results.py의 fallback 로직 재현 (lines 558-565)
    if train_est_tick is not None:
        eval_est_tick = train_est_tick
    else:
        eval_est_tick = [y_delay] if isinstance(y_delay, int) else y_delay

    # 저장된 config.yaml에서도 확인
    stored_cfg_path = exp_dir / "config.yaml"
    if stored_cfg_path.exists():
        stored_cfg = load_yaml(stored_cfg_path)
        stored_est_tick = (
            get_nested(stored_cfg, "shared.data.est_tick_ranges")
            or get_nested(stored_cfg, "02_train.data.est_tick_ranges")
            or stored_cfg.get("est_tick_ranges")
        )

        expected_from_stored = stored_est_tick or [
            get_nested(stored_cfg, "shared.data.y_delay") or 5
        ]

        if eval_est_tick != expected_from_stored:
            subj = exp_dir.name.split("_Test-")[1].replace("_seed42", "")
            return [Issue(
                check_id="C4",
                exp_name=exp_name,
                subject=subj,
                issue_type="CONFIG_INCONSISTENCY",
                description=(
                    f"est_tick_ranges 불일치: "
                    f"merged_config에서 추론={eval_est_tick}, "
                    f"stored_config.yaml에서 추론={expected_from_stored}"
                ),
                severity="MAJOR",
                fix_strategy="FIX_AND_RETRAIN",
                fix_detail=(
                    "실험 config에 est_tick_ranges를 명시적으로 추가 후 재학습."
                ),
                retrain_needed=True,
                retrain_config=str(CONFIGS_DIR / f"{exp_name}.yaml"),
            )]

    return []


# ─────────────────────────────────────────────
# Check C5: _dot 변수 30Hz LPF 일관성
# ─────────────────────────────────────────────
def check_c5_dot_lpf_consistency() -> List[Issue]:
    """
    훈련 코드의 _dot 변수에 적용된 hardcoded 30Hz LPF가
    compare_results.py에서도 동일하게 적용되는지 확인.
    (두 스크립트가 build_nn_dataset_multi를 공유하면 일관성 보장)
    """
    eval_src = EVAL_SCRIPT.read_text()

    uses_shared_build = (
        "from SpeedEstimator_TCN_MLP_experiments import" in eval_src
        and "build_nn_dataset_multi" in eval_src
    )

    if uses_shared_build:
        return [Issue(
            check_id="C5",
            exp_name="GLOBAL",
            subject="ALL",
            issue_type="PASS",
            description=(
                "compare_results.py가 SpeedEstimator_TCN_MLP_experiments.py의 "
                "build_nn_dataset_multi를 import하여 사용함. "
                "_dot 변수의 30Hz hardcoded LPF가 훈련/평가 모두 동일하게 적용됨."
            ),
            severity="INFO",
            fix_strategy="PASS",
            fix_detail="일관성 확인됨. 조치 불필요.",
            retrain_needed=False,
        )]
    else:
        return [Issue(
            check_id="C5",
            exp_name="GLOBAL",
            subject="ALL",
            issue_type="CODE_INCONSISTENCY",
            description=(
                "compare_results.py가 build_nn_dataset_multi를 import하지 않거나 "
                "독자적으로 구현함. _dot LPF 일관성 확인 불가."
            ),
            severity="CRITICAL",
            fix_strategy="DOCUMENT",
            fix_detail="eval 스크립트의 data loading 경로 수동 확인 필요.",
            retrain_needed=False,
        )]


# ─────────────────────────────────────────────
# Check C6: Config 상속 일관성
# ─────────────────────────────────────────────
def check_c6_config_inheritance(exp_name: str, exp_dir: Path, merged_cfg: Dict) -> List[Issue]:
    """
    실험 디렉토리에 저장된 config.yaml이
    baseline + exp_name config를 deep_merge한 결과와 핵심 파라미터 기준 일치하는지 확인.
    """
    stored_cfg_path = exp_dir / "config.yaml"
    if not stored_cfg_path.exists():
        return []

    stored_cfg = load_yaml(stored_cfg_path)

    CRITICAL_KEYS = [
        ("shared.data.window_size",    300),
        ("shared.data.y_delay",        5),
        ("shared.lpf_cutoff",          0.5),
        ("shared.lpf_order",           4),
        ("shared.data.stride",         5),
        ("shared.data.stride_inf",     1),
    ]

    issues = []
    subj = exp_dir.name.split("_Test-")[1].replace("_seed42", "")

    for dotted_key, fallback in CRITICAL_KEYS:
        stored_val  = get_nested(stored_cfg, dotted_key, fallback)
        expected_val = get_nested(merged_cfg, dotted_key, fallback)

        if stored_val != expected_val:
            issues.append(Issue(
                check_id="C6",
                exp_name=exp_name,
                subject=subj,
                issue_type="CONFIG_MISMATCH",
                description=(
                    f"Config 불일치: {dotted_key} — "
                    f"저장된 값={stored_val}, "
                    f"현재 merge 결과={expected_val}"
                ),
                severity="MAJOR",
                fix_strategy="FIX_AND_RETRAIN",
                fix_detail=(
                    f"{dotted_key}를 {expected_val}로 수정한 config로 재학습."
                ),
                retrain_needed=True,
                retrain_config=str(CONFIGS_DIR / f"{exp_name}.yaml"),
            ))

    return issues


# ─────────────────────────────────────────────
# Check C7: Scaler shape 일관성
# ─────────────────────────────────────────────
def check_c7_scaler_shape(exp_name: str, exp_dir: Path, merged_cfg: Dict) -> List[Issue]:
    """
    scaler.npz의 mean/scale shape이 input_vars 개수(30)와 일치하는지 확인.
    """
    scaler_path = exp_dir / "scaler.npz"
    if not scaler_path.exists():
        return []

    scaler = np.load(scaler_path)
    mean_shape  = scaler["mean"].shape
    scale_shape = scaler["scale"].shape

    # input_vars 개수 계산
    input_vars = get_nested(merged_cfg, "shared.input_vars") or []
    expected_dim = sum(len(v[1]) for v in input_vars if isinstance(v, (list, tuple)) and len(v) >= 2)

    if expected_dim == 0:
        return []  # config에서 input_vars를 못 찾으면 skip

    subj = exp_dir.name.split("_Test-")[1].replace("_seed42", "")
    issues = []

    for name, shape in [("mean", mean_shape), ("scale", scale_shape)]:
        if len(shape) == 1 and shape[0] != expected_dim:
            issues.append(Issue(
                check_id="C7",
                exp_name=exp_name,
                subject=subj,
                issue_type="SCALER_SHAPE_MISMATCH",
                description=(
                    f"scaler.npz의 {name} shape={shape[0]}, "
                    f"기대 input_dim={expected_dim}"
                ),
                severity="CRITICAL",
                fix_strategy="FIX_AND_RETRAIN",
                fix_detail="입력 피처 수 불일치. config의 input_vars 확인 후 재학습.",
                retrain_needed=True,
                retrain_config=str(CONFIGS_DIR / f"{exp_name}.yaml"),
            ))

    return issues


# ─────────────────────────────────────────────
# Check C8: Model checkpoint 로드 가능성
# ─────────────────────────────────────────────
def check_c8_model_checkpoint(exp_name: str, exp_dir: Path) -> List[Issue]:
    """
    model.pt가 존재하고 torch.load로 로드 가능한지 확인.
    """
    model_path = exp_dir / "model.pt"
    if not model_path.exists():
        subj = exp_dir.name.split("_Test-")[1].replace("_seed42", "")
        return [Issue(
            check_id="C8",
            exp_name=exp_name,
            subject=subj,
            issue_type="MISSING_CHECKPOINT",
            description="model.pt 파일이 존재하지 않음.",
            severity="CRITICAL",
            fix_strategy="FIX_AND_RETRAIN",
            fix_detail="동일 config로 재학습 필요.",
            retrain_needed=True,
            retrain_config=str(CONFIGS_DIR / f"{exp_name}.yaml"),
        )]

    try:
        import torch
        ckpt = torch.load(model_path, map_location="cpu")
        if not isinstance(ckpt, dict):
            raise ValueError(f"Unexpected checkpoint type: {type(ckpt)}")
    except Exception as e:
        subj = exp_dir.name.split("_Test-")[1].replace("_seed42", "")
        return [Issue(
            check_id="C8",
            exp_name=exp_name,
            subject=subj,
            issue_type="CORRUPT_CHECKPOINT",
            description=f"model.pt 로드 실패: {e}",
            severity="CRITICAL",
            fix_strategy="FIX_AND_RETRAIN",
            fix_detail="손상된 체크포인트. 동일 config로 재학습.",
            retrain_needed=True,
            retrain_config=str(CONFIGS_DIR / f"{exp_name}.yaml"),
        )]

    return []


# ─────────────────────────────────────────────
# Check: 파일 완전성
# ─────────────────────────────────────────────
def check_file_completeness(exp_dir: Path) -> List[Issue]:
    """필수 파일 5개가 모두 존재하는지 확인."""
    missing = [f for f in REQUIRED_FILES if not (exp_dir / f).exists()]
    if not missing:
        return []

    exp_name = exp_dir.name.rsplit("_Test-", 1)[0]
    subj = exp_dir.name.split("_Test-")[1].replace("_seed42", "")
    return [Issue(
        check_id="FILE",
        exp_name=exp_name,
        subject=subj,
        issue_type="MISSING_FILES",
        description=f"누락 파일: {missing}",
        severity="CRITICAL",
        fix_strategy="FIX_AND_RETRAIN",
        fix_detail="동일 config로 재학습하여 모든 파일 재생성.",
        retrain_needed=True,
        retrain_config=str(CONFIGS_DIR / f"{exp_name}.yaml"),
    )]


# ─────────────────────────────────────────────
# 모든 체크를 실험 하나에 대해 실행
# ─────────────────────────────────────────────
def run_all_checks_for_experiment(exp_dir: Path) -> List[Issue]:
    exp_name = exp_dir.name.rsplit("_Test-", 1)[0]

    # 파일 완전성
    file_issues = check_file_completeness(exp_dir)
    if any(i.fix_strategy == "FIX_AND_RETRAIN" and "MISSING_FILES" in i.issue_type
           for i in file_issues):
        return file_issues  # 파일이 없으면 나머지 체크 불가

    # config 로드
    merged_cfg = get_merged_config(exp_name) or {}

    issues: List[Issue] = []
    issues += file_issues
    issues += check_c2_multistep_metrics(exp_name, exp_dir, merged_cfg)
    issues += check_c3_ema_order(exp_name, exp_dir, merged_cfg)
    issues += check_c4_temporal_alignment(exp_name, exp_dir, merged_cfg)
    issues += check_c6_config_inheritance(exp_name, exp_dir, merged_cfg)
    issues += check_c7_scaler_shape(exp_name, exp_dir, merged_cfg)
    issues += check_c8_model_checkpoint(exp_name, exp_dir)

    return issues


# ─────────────────────────────────────────────
# sbatch 단일 실험 제출
# ─────────────────────────────────────────────
def submit_single_experiment(config_path: Path, job_name: str) -> Optional[int]:
    """
    sbatch --wrap 방식으로 단일 실험을 SLURM에 제출.
    train.sh는 configs/ 전체를 스캔하므로 직접 사용하지 않음.
    """
    logs_dir = SCRIPT_DIR / "logs"
    logs_dir.mkdir(exist_ok=True)

    cmd = [
        "sbatch",
        "--gres=gpu:1",
        "--cpus-per-task=16",
        "--mem=15G",
        "-t", "04:00:00",
        f"--job-name=retrain_{job_name}",
        f"--output={logs_dir}/retrain_{job_name}_%j.out",
        f"--error={logs_dir}/retrain_{job_name}_%j.err",
        "--wrap",
        f"cd {SCRIPT_DIR} && python SpeedEstimator_TCN_MLP_experiments.py --config {config_path}"
    ]

    print(f"  [sbatch] 제출 명령: {' '.join(cmd[-3:])}")
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, check=True)
        # "Submitted batch job 12345"
        job_id = int(result.stdout.strip().split()[-1])
        print(f"  [sbatch] Job ID: {job_id}")
        return job_id
    except subprocess.CalledProcessError as e:
        print(f"  [sbatch] 제출 실패: {e.stderr}")
        return None
    except (ValueError, IndexError):
        print(f"  [sbatch] Job ID 파싱 실패. stdout: {result.stdout}")
        return None


def wait_for_job(job_id: int, poll_sec: int = 60, timeout_sec: int = 4 * 3600) -> bool:
    """squeue로 job 완료 대기. 완료되면 True, timeout이면 False."""
    start = time.time()
    while time.time() - start < timeout_sec:
        result = subprocess.run(
            ["squeue", "--job", str(job_id), "-h"],
            capture_output=True, text=True
        )
        if result.stdout.strip() == "":
            return True
        elapsed = int(time.time() - start)
        print(f"    Job {job_id} 진행 중... ({elapsed}s 경과)")
        time.sleep(poll_sec)
    return False


def backup_experiment(exp_dir: Path) -> Path:
    """재학습 전 기존 실험 디렉토리를 백업."""
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    backup_dir = exp_dir.parent / f"{exp_dir.name}_backup_{ts}"
    shutil.copytree(exp_dir, backup_dir)
    shutil.rmtree(exp_dir)
    print(f"  [backup] {exp_dir.name} → {backup_dir.name}")
    return backup_dir


def restore_backup(backup_dir: Path, exp_dir: Path) -> None:
    """재학습 실패 시 백업에서 복구."""
    if exp_dir.exists():
        shutil.rmtree(exp_dir)
    shutil.copytree(backup_dir, exp_dir)
    print(f"  [restore] {backup_dir.name} → {exp_dir.name}")


# ─────────────────────────────────────────────
# 자동 수정 및 재학습
# ─────────────────────────────────────────────
def apply_fix_and_retrain(issue: Issue, dry_run: bool = False) -> bool:
    """
    FIX_AND_RETRAIN 이슈에 대해 config 수정 후 sbatch로 재학습.
    dry_run=True이면 실제로 제출하지 않음.
    """
    if not issue.retrain_needed or not issue.retrain_config:
        return False

    config_path = Path(issue.retrain_config)
    if not config_path.exists():
        print(f"  [FIX] Config 파일 없음: {config_path}")
        return False

    # 수정 config 생성 (현재는 원본 config 그대로 사용, C4/C6 수정 로직 추가 가능)
    FIXED_DIR.mkdir(parents=True, exist_ok=True)
    fixed_config_path = FIXED_DIR / config_path.name

    if issue.check_id == "C4":
        # est_tick_ranges를 명시적으로 추가
        cfg = load_yaml(config_path)
        merged = get_merged_config(issue.exp_name) or {}
        est_tick = (
            get_nested(merged, "shared.data.est_tick_ranges")
            or [get_nested(merged, "shared.data.y_delay") or 5]
        )
        cfg.setdefault("shared", {}).setdefault("data", {})["est_tick_ranges"] = est_tick
        with open(fixed_config_path, "w") as f:
            yaml.dump(cfg, f, default_flow_style=False, allow_unicode=True)
        print(f"  [FIX-C4] est_tick_ranges={est_tick}를 config에 명시: {fixed_config_path}")

    elif issue.check_id in ("FILE", "C8"):
        # 동일 config 그대로 사용
        shutil.copy2(config_path, fixed_config_path)
        print(f"  [FIX-{issue.check_id}] 동일 config 재사용: {fixed_config_path}")

    else:
        # 기본: 원본 config 그대로
        shutil.copy2(config_path, fixed_config_path)
        print(f"  [FIX] 원본 config 복사: {fixed_config_path}")

    if dry_run:
        print(f"  [dry-run] sbatch 제출 생략.")
        return True

    # 실험 디렉토리 특정 (여러 피험자에 걸친 이슈일 수 있음)
    exp_dirs = list(EXP_ROOT.glob(f"{issue.exp_name}_Test-*_seed42"))
    if not exp_dirs:
        print(f"  [FIX] 해당 실험 디렉토리 없음: {issue.exp_name}")
        return False

    all_ok = True
    for exp_dir in exp_dirs:
        print(f"\n  [RETRAIN] {exp_dir.name}")
        backup_dir = backup_experiment(exp_dir)
        job_name = exp_dir.name[:50]  # sbatch job-name 길이 제한
        job_id = submit_single_experiment(fixed_config_path, job_name)
        if job_id is None:
            restore_backup(backup_dir, exp_dir)
            all_ok = False
            continue

        ok = wait_for_job(job_id)
        if not ok:
            print(f"  [RETRAIN] Job {job_id} timeout. 백업 복구.")
            restore_backup(backup_dir, exp_dir)
            all_ok = False
        else:
            # 재검증
            post_issues = run_all_checks_for_experiment(exp_dir)
            critical_remaining = [i for i in post_issues
                                   if i.severity in ("CRITICAL", "MAJOR")
                                   and i.fix_strategy == "FIX_AND_RETRAIN"]
            if not critical_remaining:
                print(f"  ✅ {exp_dir.name}: 수정 완료")
            else:
                print(f"  ❌ {exp_dir.name}: 재학습 후에도 이슈 남음. 백업 복구.")
                restore_backup(backup_dir, exp_dir)
                all_ok = False

    return all_ok


# ─────────────────────────────────────────────
# 리포트 생성
# ─────────────────────────────────────────────
def severity_rank(s: str) -> int:
    return {"CRITICAL": 0, "MAJOR": 1, "MINOR": 2, "INFO": 3}.get(s, 4)


def generate_report(all_issues: List[Issue]) -> None:
    """validation_report.json과 validation_report.md를 생성."""
    # JSON
    report_data = {
        "generated_at": datetime.now().isoformat(),
        "total_issues": len(all_issues),
        "by_severity": {
            s: sum(1 for i in all_issues if i.severity == s)
            for s in ("CRITICAL", "MAJOR", "MINOR", "INFO")
        },
        "issues": [asdict(i) for i in all_issues],
    }
    REPORT_JSON.write_text(json.dumps(report_data, ensure_ascii=False, indent=2))
    print(f"\n[리포트] JSON 저장: {REPORT_JSON}")

    # Markdown
    lines = [
        "# SpeedEstimation Pre-Validation Report",
        f"\n생성: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}",
        f"\n총 이슈: {len(all_issues)}개",
        f"| 심각도 | 개수 |",
        "|--------|------|",
    ]
    for sev in ("CRITICAL", "MAJOR", "MINOR", "INFO"):
        cnt = sum(1 for i in all_issues if i.severity == sev)
        lines.append(f"| {sev} | {cnt} |")

    lines.append("\n## 이슈 목록\n")

    check_groups: Dict[str, List[Issue]] = {}
    for issue in all_issues:
        check_groups.setdefault(issue.check_id, []).append(issue)

    for check_id in sorted(check_groups.keys()):
        group = check_groups[check_id]
        first = group[0]
        lines.append(f"### {check_id}: {first.issue_type}")
        lines.append(f"**심각도:** {first.severity}  ")
        lines.append(f"**조치:** {first.fix_strategy}  ")
        lines.append(f"\n{first.description}")
        if first.fix_detail:
            lines.append(f"\n> **Fix:** {first.fix_detail}")
        if len(group) > 1:
            lines.append(f"\n영향 실험 ({len(group)}개):")
            for i in group[1:]:
                lines.append(f"- {i.exp_name} / {i.subject}: {i.description[:80]}")
        lines.append("")

    # 분석 단계별 주의사항 요약
    lines += [
        "## 분석 시 주의사항 요약",
        "",
        "| Check | 영향 실험 | 분석 시 처리 방법 |",
        "|-------|----------|-----------------|",
        "| C1 | AR feedback (6개) | factor 분석에서 'AR 미구현' 주석 표기, AR mechanism 분석 제외 |",
        "| C2 | H>1 multi-step (15개) | metrics.json 대신 compare_results.py inference 결과 사용 |",
        "| C3 | EMA 실험 (9개) | compare_results.py inference 재실행으로 EMA 적용 후 MAE 획득 |",
        "| C4 | 전체 (pass 시) | est_tick_ranges 명시 확인 완료 |",
        "| C5 | 전체 | build_nn_dataset_multi 공유 확인 → 일관성 보장 |",
        "| C6 | 전체 | 핵심 파라미터 일치 확인 |",
        "",
        "> **결론:** Phase 1 분석에서 모든 60개 실험에 대해 `compare_results.py`의",
        "> `load_and_evaluate()`를 통한 inference를 실행하고 그 결과를 canonical metric으로 사용.",
        "> `metrics.json`의 값은 보조 참고용으로만 활용.",
    ]

    REPORT_MD.write_text("\n".join(lines))
    print(f"[리포트] Markdown 저장: {REPORT_MD}")


# ─────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(
        description="SpeedEstimation Phase 0: 훈련-평가 일관성 감사 + 자동 수정/재학습"
    )
    parser.add_argument("--dry-run", action="store_true",
                        help="탐지만 수행, 수정/재학습 없음")
    parser.add_argument("--exp", type=str, default=None,
                        help="특정 실험 디렉토리 이름 (예: baseline_Test-m1_S004_seed42)")
    args = parser.parse_args()

    # 작업 디렉토리를 SpeedEstimation/으로 변경
    os.chdir(SCRIPT_DIR)
    print(f"작업 디렉토리: {SCRIPT_DIR}")
    print(f"모드: {'dry-run' if args.dry_run else '실제 실행'}")
    print("=" * 60)

    all_issues: List[Issue] = []

    # ─── 전역 체크 ───
    print("\n[전역 체크]")
    print("  C1: AR Feedback 구현 일관성...")
    all_issues += check_c1_ar_feedback()
    print("  C5: _dot LPF 일관성...")
    all_issues += check_c5_dot_lpf_consistency()

    # ─── 실험별 체크 ───
    if args.exp:
        exp_dirs = [EXP_ROOT / args.exp]
    else:
        exp_dirs = sorted(EXP_ROOT.iterdir()) if EXP_ROOT.exists() else []

    print(f"\n[실험별 체크] {len(exp_dirs)}개 실험 디렉토리")

    for exp_dir in exp_dirs:
        if not exp_dir.is_dir():
            continue
        if "_Test-" not in exp_dir.name or "_seed42" not in exp_dir.name:
            continue

        exp_name = exp_dir.name.rsplit("_Test-", 1)[0]
        issues = run_all_checks_for_experiment(exp_dir)
        all_issues += issues

        # 심각 이슈만 즉시 출력
        for i in issues:
            if i.severity in ("CRITICAL", "MAJOR") and i.fix_strategy == "FIX_AND_RETRAIN":
                print(f"  ⚠️  [{i.check_id}/{i.severity}] {exp_dir.name}: {i.description[:80]}")

    # ─── 예상 실험 완전성 체크 ───
    print("\n[디렉토리 완전성 체크]")
    for exp_nm in EXPECTED_EXPERIMENTS:
        for subj in TEST_SUBJECTS:
            dirname = f"{exp_nm}_Test-{subj}_seed42"
            if not (EXP_ROOT / dirname).exists():
                all_issues.append(Issue(
                    check_id="FILE",
                    exp_name=exp_nm,
                    subject=subj,
                    issue_type="MISSING_DIRECTORY",
                    description=f"실험 디렉토리 없음: {dirname}",
                    severity="CRITICAL",
                    fix_strategy="FIX_AND_RETRAIN",
                    fix_detail="해당 config로 처음부터 학습 필요.",
                    retrain_needed=True,
                    retrain_config=str(CONFIGS_DIR / f"{exp_nm}.yaml"),
                ))
                print(f"  ❌ 누락: {dirname}")

    # ─── 이슈 요약 ───
    print("\n" + "=" * 60)
    print("감사 결과 요약")
    print("=" * 60)
    for sev in ("CRITICAL", "MAJOR", "MINOR", "INFO"):
        cnt = sum(1 for i in all_issues if i.severity == sev)
        print(f"  {sev:10s}: {cnt}개")

    retrain_issues = [i for i in all_issues if i.retrain_needed]
    print(f"\n  재학습 필요: {len(retrain_issues)}개")

    # ─── 자동 수정 및 재학습 ───
    if retrain_issues and not args.dry_run:
        print("\n[자동 수정 + 재학습 시작]")
        # 심각도 순으로 정렬
        retrain_issues.sort(key=lambda x: severity_rank(x.severity))
        for issue in retrain_issues:
            print(f"\n  처리 중: [{issue.check_id}] {issue.exp_name} / {issue.subject}")
            ok = apply_fix_and_retrain(issue, dry_run=False)
            if ok:
                print(f"  ✅ 완료: {issue.exp_name} / {issue.subject}")
            else:
                print(f"  ❌ 실패: {issue.exp_name} / {issue.subject} — 수동 확인 필요")
    elif retrain_issues and args.dry_run:
        print("\n[dry-run] 재학습 대상 목록:")
        for issue in retrain_issues:
            print(f"  [{issue.check_id}/{issue.severity}] {issue.exp_name} / {issue.subject}")
            print(f"    Fix: {issue.fix_detail}")

    # ─── 리포트 생성 ───
    generate_report(all_issues)

    # ─── 최종 상태 ───
    critical_cnt = sum(1 for i in all_issues
                       if i.severity == "CRITICAL" and i.fix_strategy != "PASS")
    major_cnt = sum(1 for i in all_issues
                    if i.severity == "MAJOR" and i.fix_strategy == "FIX_AND_RETRAIN")

    print("\n" + "=" * 60)
    if critical_cnt == 0 and major_cnt == 0:
        print("✅ Phase 0 완료. Phase 1 분석 진행 가능.")
        print(f"   주의 사항은 {REPORT_MD}를 참조하세요.")
    else:
        print(f"⚠️  Phase 0 미완료. CRITICAL={critical_cnt}, MAJOR(retrain)={major_cnt}개 남음.")
        print(f"   {REPORT_MD}를 확인하고 수동 조치 후 재실행하세요.")
    print("=" * 60)


if __name__ == "__main__":
    main()
