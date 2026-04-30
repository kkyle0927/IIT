try:
    from IPython.display import display
except ImportError:
    def display(obj):
        if hasattr(obj, 'to_string'):
            print(obj.to_string())
        else:
            print(obj)

# ─────────────────────────────────────────────
# 0. 환경 설정 및 임포트
# ─────────────────────────────────────────────
import os, sys, json, yaml, copy, re, warnings, pickle
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import seaborn as sns
from pathlib import Path
from scipy import stats, signal
from itertools import combinations
from datetime import datetime
warnings.filterwarnings('ignore')

# SpeedEstimation/ 디렉토리를 기준으로 실행
NOTEBOOK_DIR = Path('/home/chanyoungko/IIT/SpeedEstimation')
os.chdir(NOTEBOOK_DIR)
sys.path.insert(0, str(NOTEBOOK_DIR))

# 시각화 설정
plt.rcParams.update({
    'font.size': 12, 'axes.titlesize': 13, 'axes.labelsize': 12,
    'figure.dpi': 120, 'savefig.dpi': 150,
    'figure.facecolor': 'white', 'axes.grid': True, 'grid.alpha': 0.3,
    'axes.spines.top': False, 'axes.spines.right': False,
})
PALETTE = sns.color_palette('tab10', 10)

# 출력 디렉토리
OUTPUT_DIR = NOTEBOOK_DIR / 'analysis_output'
OUTPUT_DIR.mkdir(exist_ok=True)

print(f'작업 디렉토리: {os.getcwd()}')

# ─────────────────────────────────────────────
# 0-1. 실험 메타데이터 정의
# ─────────────────────────────────────────────
EXP_ROOT  = NOTEBOOK_DIR / 'experiments'
SUBJECTS  = ['m1_S004', 'm1_S011', 'm2_S008']
CONDITIONS = [
    'level_075mps', 'level_08mps', 'level_100mps',
    'level_12mps',  'level_125mps', 'level_16mps',
    'accel_sine', 'decline_5deg', 'incline_10deg', 'stopandgo'
]
CONDITION_GROUPS = {
    'Steady-State': ['level_075mps','level_08mps','level_100mps',
                     'level_12mps','level_125mps','level_16mps'],
    'Dynamic':      ['accel_sine', 'stopandgo'],
    'Terrain':      ['decline_5deg', 'incline_10deg'],
}

# factor_group, variant_label, 분석 제외 플래그(구 C1 이슈 — AR feedback 재학습 완료로 모두 False)
EXPERIMENT_META = {
    'baseline':                         ('Baseline',       'Baseline',      False),
    'exp_multistep_H05':                 ('Horizon',        'H=5',           False),
    'exp_multistep_H10':                 ('Horizon',        'H=10',          False),
    'exp_multistep_H20':                 ('Horizon',        'H=20',          False),
    'exp_multistep_H10_smooth_lam001':   ('Smooth_Loss',    'λ=0.01',        False),
    'exp_multistep_H10_smooth_lam01':    ('Smooth_Loss',    'λ=0.1',         False),
    'exp_multistep_H10_smooth_lam05':    ('Smooth_Loss',    'λ=0.5',         False),
    'exp_multistep_H10_overlap_mean':    ('Overlap',        'Mean',          False),
    'exp_multistep_H10_overlap_median':  ('Overlap',        'Median',        False),
    'exp_multistep_H10_overlap_wmean':   ('Overlap',        'WeightedMean',  False),
    'exp_single_ema_a080':               ('EMA',            'α=0.80',        False),
    'exp_single_ema_a090':               ('EMA',            'α=0.90',        False),
    'exp_single_ema_a095':               ('EMA',            'α=0.95',        False),
    'exp_tcn_gru_head_h32':              ('Architecture',   'GRU_h32',       False),
    'exp_tcn_gru_head_h64':              ('Architecture',   'GRU_h64',       False),
    'exp_multistep_H10_ema_a090':        ('Combined',       'H10+EMA0.9',    False),
    'exp_tcn_gru_head_h32_smooth':       ('Combined',       'GRU+Smooth',    False),
    'exp_ar_feedback_k1':                ('AR_Feedback',    'k=1',           False),  # 재학습 완료
    'exp_ar_feedback_sched':             ('AR_Feedback',    'Scheduled',     False),  # 재학습 완료
    'Exp_IMU_Orientation':               ('Input_Features', 'IMU_Orient',    False),
}

# Phase 0 validation_report.json 로드
REPORT_PATH = NOTEBOOK_DIR / 'validation_report.json'
if REPORT_PATH.exists():
    val_report = json.loads(REPORT_PATH.read_text())
    print(f'[Phase 0 리포트 로드됨] 이슈 총 {val_report["total_issues"]}개')
    print(f'  심각도별:', val_report['by_severity'])
else:
    print('[주의] validation_report.json 없음 — validate_and_retrain.py 먼저 실행 권장')
    val_report = {}

print(f'\n실험 설정 수: {len(EXPERIMENT_META)}')
print(f'테스트 피험자: {SUBJECTS}')
print(f'조건 수: {len(CONDITIONS)}')

# ─────────────────────────────────────────────
# Phase 1: compare_results.py 임포트 및 캐시 체크
# ─────────────────────────────────────────────
import torch
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
print(f'Device: {device}')

from compare_results import load_and_evaluate, calculate_smoothness, calculate_lag
print('compare_results.py 임포트 완료')

CACHE_AGG   = OUTPUT_DIR / 'computed_metrics_cache.pkl'
CACHE_COND  = OUTPUT_DIR / 'per_condition_cache.pkl'
FORCE_RECOMPUTE = False  # True로 바꾸면 캐시 무시하고 재계산

def run_inference_all_experiments(force=False):
    """
    60개 모델 전체에 대해 load_and_evaluate() 실행.
    캐시가 있으면 로드, 없으면 계산 후 저장.
    """
    if CACHE_AGG.exists() and not force:
        print(f'[캐시 로드] {CACHE_AGG}')
        with open(CACHE_AGG, 'rb') as f:
            return pickle.load(f)

    print('[Inference 시작] 60개 모델 — 시간이 걸립니다...')
    records = []

    for exp_name, (factor_group, variant_label, c1_flagged) in EXPERIMENT_META.items():
        for subj in SUBJECTS:
            dirname = f'{exp_name}_Test-{subj}_seed42'
            exp_dir = EXP_ROOT / dirname
            if not exp_dir.exists():
                print(f'  [SKIP] {dirname} — 디렉토리 없음')
                continue

            print(f'  [{exp_name}/{subj}] inference...')
            try:
                result = load_and_evaluate(str(exp_dir), device=device, return_seqs=True)
            except Exception as e:
                print(f'    ❌ 실패: {e}')
                result = None

            if result is None:
                continue

            records.append({
                'exp_name':     exp_name,
                'subject':      subj,
                'factor_group': factor_group,
                'variant':      variant_label,
                'c1_flagged':   c1_flagged,
                'mae':          result['mae'],
                'rmse':         result['rmse'],
                'r2':           result['r2'],
                'lag_ms':       result['lag_ms'],
                'jitter':       result['smoothness'],
                'y_pred':       result.get('y_pred_seq'),
                'y_true':       result.get('y_true_seq'),
                'offsets':      result.get('series_offsets'),
            })

            # metrics.json에서 추가 정보
            metrics_path = exp_dir / 'metrics.json'
            if metrics_path.exists():
                m = json.loads(metrics_path.read_text())
                records[-1]['n_params']    = m.get('n_params', 0)
                records[-1]['latency_ms']  = m.get('host_latency_ms', 0)
                records[-1]['stored_mae']  = m.get('test_mae', np.nan)

    with open(CACHE_AGG, 'wb') as f:
        pickle.dump(records, f)
    print(f'[캐시 저장] {CACHE_AGG}')
    return records


raw_records = run_inference_all_experiments(force=FORCE_RECOMPUTE)
print(f'\n총 {len(raw_records)}개 결과 로드됨')

# ─────────────────────────────────────────────
# DataFrame 생성 (y_pred/y_true 제외 — 메모리 절약)
# ─────────────────────────────────────────────
df_full = pd.DataFrame(raw_records)

# 시계열 분리 보관
seq_store = {(r['exp_name'], r['subject']): {
    'y_pred': r['y_pred'], 'y_true': r['y_true'], 'offsets': r['offsets']
} for r in raw_records if r.get('y_pred') is not None}

# 분석용 DataFrame (시계열 제외)
df = df_full.drop(columns=['y_pred', 'y_true', 'offsets'], errors='ignore')

print(df.groupby('exp_name')[['mae','jitter','lag_ms']].mean().round(4).to_string())

# ─────────────────────────────────────────────
# Phase 2-1: 피험자 평균 요약 테이블
# ─────────────────────────────────────────────
agg = df.groupby(['exp_name','factor_group','variant','c1_flagged']).agg(
    mean_mae   = ('mae',      'mean'),
    std_mae    = ('mae',      'std'),
    mean_rmse  = ('rmse',     'mean'),
    mean_jitter= ('jitter',   'mean'),
    std_jitter = ('jitter',   'std'),
    mean_lag   = ('lag_ms',   'mean'),
    mean_r2    = ('r2',       'mean'),
    mean_latency= ('latency_ms','mean'),
    n_params   = ('n_params', 'mean'),
).reset_index().sort_values('mean_mae')

print('=== Top-5 by MAE ===')
print(agg[['exp_name','mean_mae','std_mae','mean_jitter','mean_lag']].head().to_string(index=False))
print('\n=== Top-5 Smoothest (lowest jitter) ===')
print(agg.sort_values('mean_jitter')[['exp_name','mean_mae','mean_jitter']].head().to_string(index=False))

# ─────────────────────────────────────────────
# Phase 2-2: 전체 요약 Heatmap
# ─────────────────────────────────────────────
metrics_to_show = ['mean_mae', 'mean_rmse', 'mean_jitter', 'mean_lag', 'mean_r2']
heatmap_data = agg.set_index('exp_name')[metrics_to_show].copy()

# z-score 정규화 (열별)
heatmap_norm = (heatmap_data - heatmap_data.mean()) / (heatmap_data.std() + 1e-9)
# r2는 높을수록 좋으므로 반전
heatmap_norm['mean_r2'] = -heatmap_norm['mean_r2']

fig, axes = plt.subplots(1, 2, figsize=(18, 10), gridspec_kw={'width_ratios': [1, 1]})

# 왼쪽: 실제 값
sns.heatmap(heatmap_data, annot=True, fmt='.4f', cmap='YlOrRd',
            ax=axes[0], linewidths=0.5)
axes[0].set_title('실제 지표값')

# 오른쪽: 정규화된 값 (낮을수록 좋음, 빨간색)
sns.heatmap(heatmap_norm, annot=True, fmt='.2f', cmap='RdYlGn_r',
            ax=axes[1], linewidths=0.5, center=0)
axes[1].set_title('Z-score 정규화 (빨간색=나쁨, 초록색=좋음)\nr2는 반전 적용')

# C1 flagged 실험 표시
c1_exps = agg[agg['c1_flagged']]['exp_name'].tolist()
for ax in axes:
    yticklabels = ax.get_yticklabels()
    for label in yticklabels:
        if label.get_text() in c1_exps:
            label.set_color('orange')
            label.set_text(label.get_text() + ' ⚠️C1')

plt.suptitle('모든 실험 지표 요약 Heatmap (⚠️C1 = AR feedback 미구현)', fontsize=14, y=1.01)
plt.tight_layout()
plt.savefig(OUTPUT_DIR / 'phase2_summary_heatmap.png', bbox_inches='tight')
plt.show()

# ─────────────────────────────────────────────
# Phase 3 공통 플롯 함수
# ─────────────────────────────────────────────
def plot_factor_analysis(factor_name: str, baseline_variant: str = 'Baseline',
                          save_prefix: str = ''):
    """
    한 factor group에 대해:
    (a) MAE bar chart  (b) Jitter bar chart  (c) MAE-Jitter phase diagram
    을 3-panel figure로 출력.
    """
    # factor group 추출
    fdata = df[df['factor_group'] == factor_name].copy()
    # baseline도 포함
    base_data = df[df['factor_group'] == 'Baseline'].copy()
    base_data['variant'] = 'Baseline'
    fdata = pd.concat([base_data, fdata[fdata['variant'] != 'Baseline']],
                       ignore_index=True)

    variants = fdata['variant'].unique().tolist()
    # Baseline 맨 앞으로
    if 'Baseline' in variants:
        variants = ['Baseline'] + [v for v in variants if v != 'Baseline']

    # 피험자별 집계
    pivot_mae    = fdata.pivot_table(index='subject', columns='variant', values='mae')
    pivot_jitter = fdata.pivot_table(index='subject', columns='variant', values='jitter')
    pivot_lag    = fdata.pivot_table(index='subject', columns='variant', values='lag_ms')

    fig, axes = plt.subplots(1, 3, figsize=(18, 5))
    colors = PALETTE[:len(variants)]

    for ax, pivot, ylabel, title in [
        (axes[0], pivot_mae,    'MAE (m/s)',      f'{factor_name}: MAE'),
        (axes[1], pivot_jitter, 'Jitter (m/s²)',  f'{factor_name}: Jitter (Smoothness)'),
    ]:
        means = pivot[variants].mean()
        stds  = pivot[variants].std()
        x = range(len(variants))
        bars = ax.bar(x, means, yerr=stds, capsize=5, color=colors, alpha=0.8, edgecolor='black')
        ax.set_xticks(x); ax.set_xticklabels(variants, rotation=35, ha='right')
        ax.set_ylabel(ylabel); ax.set_title(title)
        # baseline 기준선
        if 'Baseline' in means.index:
            ax.axhline(means['Baseline'], color='red', linestyle='--', alpha=0.6,
                       label=f'Baseline={means["Baseline"]:.4f}')
        ax.legend(fontsize=9)

    # Phase diagram
    ax = axes[2]
    mean_mae_by_v    = pivot_mae[variants].mean()
    mean_jitter_by_v = pivot_jitter[variants].mean()
    for i, v in enumerate(variants):
        marker = '*' if v == 'Baseline' else 'o'
        ax.scatter(mean_mae_by_v[v], mean_jitter_by_v[v],
                   color=colors[i], s=120, marker=marker, zorder=3, label=v)
        ax.annotate(v, (mean_mae_by_v[v], mean_jitter_by_v[v]),
                    xytext=(5, 5), textcoords='offset points', fontsize=8)
    ax.set_xlabel('Mean MAE (m/s) [↓ 좋음]')
    ax.set_ylabel('Mean Jitter (m/s²) [↓ 좋음]')
    ax.set_title(f'{factor_name}: Accuracy vs. Smoothness')
    ax.legend(loc='upper right', fontsize=8)

    plt.suptitle(f'Factor Analysis: {factor_name}', fontsize=14, fontweight='bold')
    plt.tight_layout()
    if save_prefix:
        plt.savefig(OUTPUT_DIR / f'{save_prefix}_factor_{factor_name}.png', bbox_inches='tight')
    plt.show()

    # 수치 요약 테이블 출력
    summary = pd.DataFrame({
        'variant':    variants,
        'mean_MAE':   [pivot_mae[v].mean()    for v in variants],
        'std_MAE':    [pivot_mae[v].std()     for v in variants],
        'mean_Jitter':[pivot_jitter[v].mean() for v in variants],
        'mean_Lag_ms':[pivot_lag[v].mean()    for v in variants if v in pivot_lag.columns],
    }).set_index('variant')
    display(summary.round(4))
    return summary

# 3.1 Prediction Horizon
print('=== 3.1 Prediction Horizon (H=1→5→10→20) ===')
s31 = plot_factor_analysis('Horizon', save_prefix='p3')

# 3.2 Smoothness Loss
print('=== 3.2 Smoothness Loss Regularization (λ) ===')
s32 = plot_factor_analysis('Smooth_Loss', save_prefix='p3')

# 3.3 Overlap Aggregation
print('=== 3.3 Overlap Aggregation (H=10 기반) ===')
s33 = plot_factor_analysis('Overlap', save_prefix='p3')

# 3.4 EMA Post-processing
print('=== 3.4 EMA Post-processing (α값 비교) ===')
print('[주의] C3 이슈: EMA 실험의 MAE는 compare_results.py inference 결과 (EMA 적용 후)')
s34 = plot_factor_analysis('EMA', save_prefix='p3')

# 추가: α vs. lag_ms 라인 플롯
ema_data = df[df['factor_group'] == 'EMA'].copy()
alpha_map = {'α=0.80': 0.80, 'α=0.90': 0.90, 'α=0.95': 0.95}
base_data_ema = df[df['factor_group'] == 'Baseline'][['subject','mae','jitter','lag_ms']].copy()
base_data_ema['alpha'] = 0.0

fig, axes = plt.subplots(1, 3, figsize=(15, 4))
for i, (col, ylabel, title) in enumerate([
    ('mae',    'MAE (m/s)',     'EMA α vs. MAE'),
    ('jitter', 'Jitter',        'EMA α vs. Jitter'),
    ('lag_ms', 'Lag (ms)',      'EMA α vs. Lag'),
]):
    ax = axes[i]
    for subj in SUBJECTS:
        pts_x, pts_y = [0.0], [base_data_ema[base_data_ema['subject']==subj][col].values[0]]
        for v, a in alpha_map.items():
            row = ema_data[(ema_data['variant']==v)&(ema_data['subject']==subj)]
            if not row.empty:
                pts_x.append(a); pts_y.append(row[col].values[0])
        ax.plot(pts_x, pts_y, 'o-', label=subj)
    ax.set_xlabel('EMA α'); ax.set_ylabel(ylabel); ax.set_title(title)
    ax.legend(fontsize=9)
plt.suptitle('EMA α vs. 성능 지표 (α=0 = baseline)', fontsize=13)
plt.tight_layout()
plt.savefig(OUTPUT_DIR / 'p3_EMA_alpha_tradeoff.png', bbox_inches='tight')
plt.show()

# 3.5 Architecture
print('=== 3.5 Model Architecture (MLP vs. GRU) ===')
s35 = plot_factor_analysis('Architecture', save_prefix='p3')

# 추가: n_params vs. MAE scatter
arch_data = df[df['factor_group'].isin(['Baseline','Architecture'])].groupby('exp_name').agg(
    mean_mae=('mae','mean'), n_params=('n_params','mean'), variant=('variant','first')
).reset_index()

fig, ax = plt.subplots(figsize=(7, 5))
for _, row in arch_data.iterrows():
    ax.scatter(row['n_params']/1e6, row['mean_mae'], s=100)
    ax.annotate(row['variant'], (row['n_params']/1e6, row['mean_mae']),
                xytext=(5,5), textcoords='offset points', fontsize=9)
ax.set_xlabel('파라미터 수 (M)'); ax.set_ylabel('Mean MAE (m/s)')
ax.set_title('아키텍처: 파라미터 수 vs. 정확도')
plt.tight_layout()
plt.savefig(OUTPUT_DIR / 'p3_arch_params_vs_mae.png', bbox_inches='tight')
plt.show()

# 3.6 Combined Strategies
print('=== 3.6 Combined Strategies ===')
s36 = plot_factor_analysis('Combined', save_prefix='p3')

# 3.7 AR Feedback
print('=== 3.7 AR Feedback (k=1: teacher forcing 100%, sched: 1.0→0.0 scheduled sampling) ===')
print('[재학습 완료] AR feedback 구현 후 재학습. sequential AR inference로 평가.')
s37 = plot_factor_analysis('AR_Feedback', save_prefix='p3')

# 3.8 Input Features
print('=== 3.8 Input Features (raw IMU vs. IMU Orientation) ===')
s38 = plot_factor_analysis('Input_Features', save_prefix='p3')

# ─────────────────────────────────────────────
# Phase 4-1: 전체 Pareto Frontier
# ─────────────────────────────────────────────
def pareto_front(pts):
    """최소화 목표: 두 값 모두 낮을수록 좋음."""
    pts = np.array(pts)
    is_pareto = np.ones(len(pts), dtype=bool)
    for i, p in enumerate(pts):
        if is_pareto[i]:
            dominated = np.all(pts <= p, axis=1) & np.any(pts < p, axis=1)
            is_pareto[dominated] = False
    return is_pareto

factor_colors = {
    'Baseline': 'black', 'Horizon': PALETTE[0], 'Smooth_Loss': PALETTE[1],
    'Overlap': PALETTE[2], 'EMA': PALETTE[3], 'Architecture': PALETTE[4],
    'Combined': PALETTE[5], 'AR_Feedback': PALETTE[6], 'Input_Features': PALETTE[7],
}

fig, ax = plt.subplots(figsize=(12, 8))

for fg in agg['factor_group'].unique():
    subset = agg[agg['factor_group'] == fg]
    color = factor_colors.get(fg, 'gray')
    alpha = 0.5 if fg == 'AR_Feedback' else 0.9
    ax.scatter(subset['mean_mae'], subset['mean_jitter'],
               color=color, s=100, alpha=alpha, label=fg, zorder=3)
    for _, row in subset.iterrows():
        label = row['variant'] + (' ⚠️' if row['c1_flagged'] else '')
        ax.annotate(label, (row['mean_mae'], row['mean_jitter']),
                    xytext=(5, 4), textcoords='offset points', fontsize=8, alpha=0.8)

# Pareto 전선
pts = agg[['mean_mae','mean_jitter']].values
pareto_mask = pareto_front(pts)
pareto_pts  = agg[pareto_mask].sort_values('mean_mae')
ax.plot(pareto_pts['mean_mae'], pareto_pts['mean_jitter'],
        'k--', alpha=0.5, linewidth=1.5, label='Pareto Frontier', zorder=2)

ax.set_xlabel('Mean MAE (m/s) [↓ 정확도 좋음]', fontsize=12)
ax.set_ylabel('Mean Jitter (m/s²) [↓ 스무스함 좋음]', fontsize=12)
ax.set_title('Smoothness vs. Accuracy Tradeoff: 전체 실험', fontsize=14)
ax.legend(bbox_to_anchor=(1.02, 1), fontsize=9)
plt.tight_layout()
plt.savefig(OUTPUT_DIR / 'phase4_pareto_frontier.png', bbox_inches='tight')
plt.show()

print('\nPareto Optimal 실험:')
print(pareto_pts[['exp_name','variant','mean_mae','mean_jitter','mean_lag']].to_string(index=False))

# ─────────────────────────────────────────────
# Phase 4-2: Composite Score (가중치별)
# ─────────────────────────────────────────────
norm_mae    = (agg['mean_mae']    - agg['mean_mae'].min()) / (agg['mean_mae'].max() - agg['mean_mae'].min() + 1e-9)
norm_jitter = (agg['mean_jitter'] - agg['mean_jitter'].min()) / (agg['mean_jitter'].max() - agg['mean_jitter'].min() + 1e-9)

score_df = agg[['exp_name','variant','factor_group']].copy()
for w_a, w_s in [(0.5, 0.5), (0.7, 0.3), (0.3, 0.7)]:
    score_df[f'score_{int(w_a*10)}{int(w_s*10)}'] = (
        w_a * (1 - norm_mae) + w_s * (1 - norm_jitter)
    )

print('=== Composite Score Top-5 (정확도:스무스=5:5) ===')
print(score_df.sort_values('score_55', ascending=False)[
    ['exp_name','score_55','score_73','score_37']].head(8).to_string(index=False))

# ─────────────────────────────────────────────
# Phase 5: per-condition inference 실행
# ─────────────────────────────────────────────
CACHE_COND_FILE = OUTPUT_DIR / 'per_condition_cache.pkl'

def compute_per_condition_metrics(force=False):
    """
    각 실험 × 조건에 대해 MAE와 jitter 계산.
    seq_store의 y_pred / y_true / offsets를 이용해 trial별로 분리.
    """
    if CACHE_COND_FILE.exists() and not force:
        with open(CACHE_COND_FILE, 'rb') as f:
            return pickle.load(f)

    # 조건 order는 compare_results.py에서 build_nn_dataset_multi 호출 순서와 동일
    import yaml
    _bp = 'configs/baseline.yaml' if os.path.exists('configs/baseline.yaml') else 'config_backups/baseline.yaml'
    baseline_cfg = yaml.safe_load(open(_bp))
    cond_order = baseline_cfg.get('shared', {}).get('conditions', CONDITIONS)

    records = []
    for (exp_name, subj), seqs in seq_store.items():
        y_pred  = seqs['y_pred']
        y_true  = seqs['y_true']
        offsets = seqs['offsets']

        if y_pred is None or offsets is None:
            continue

        vp = y_pred.squeeze() if y_pred.ndim > 1 else y_pred
        vt = y_true.squeeze() if y_true.ndim > 1 else y_true
        if vp.ndim > 1: vp = vp[:, 0]
        if vt.ndim > 1: vt = vt[:, 0]

        n_segs = len(offsets)
        for i, start in enumerate(offsets):
            end = offsets[i+1] if i < n_segs - 1 else len(vp)
            if start >= end:
                continue
            seg_p = vp[start:end]
            seg_t = vt[start:end]

            if len(seg_p) < 10:
                continue

            cond = cond_order[i] if i < len(cond_order) else f'cond_{i}'
            meta = EXPERIMENT_META.get(exp_name, ('Unknown', exp_name, False))

            records.append({
                'exp_name':     exp_name,
                'subject':      subj,
                'condition':    cond,
                'factor_group': meta[0],
                'variant':      meta[1],
                'c1_flagged':   meta[2],
                'mae':          np.mean(np.abs(seg_p - seg_t)),
                'rmse':         np.sqrt(np.mean((seg_p - seg_t)**2)),
                'jitter':       np.std(np.diff(seg_p)),
            })

    with open(CACHE_COND_FILE, 'wb') as f:
        pickle.dump(records, f)
    return records


cond_records = compute_per_condition_metrics(force=False)
df_cond = pd.DataFrame(cond_records)
print(f'Per-condition records: {len(df_cond)}개')
print(df_cond.groupby('condition')['mae'].mean().sort_values().round(4))

# ─────────────────────────────────────────────
# Phase 5-2: Factor group별 Condition Heatmap
# ─────────────────────────────────────────────
focus_groups = ['Horizon', 'EMA', 'Overlap', 'Smooth_Loss']

fig, axes = plt.subplots(2, 2, figsize=(22, 14))

for ax, fg in zip(axes.flat, focus_groups):
    # baseline + target factor
    subset = df_cond[df_cond['factor_group'].isin(['Baseline', fg])]
    pivot  = subset.pivot_table(
        index='variant', columns='condition', values='mae', aggfunc='mean'
    )
    # Baseline을 맨 위로
    if 'Baseline' in pivot.index:
        pivot = pd.concat([pivot.loc[['Baseline']], pivot.drop('Baseline')])

    sns.heatmap(pivot, annot=True, fmt='.3f', cmap='YlOrRd',
                linewidths=0.3, ax=ax,
                cbar_kws={'label': 'MAE (m/s)'})
    ax.set_title(f'Factor: {fg}', fontsize=12)
    ax.set_xlabel('Condition'); ax.set_ylabel('Variant')

plt.suptitle('Factor × Condition MAE Heatmap (피험자 평균)', fontsize=14, y=1.01)
plt.tight_layout()
plt.savefig(OUTPUT_DIR / 'phase5_condition_heatmap.png', bbox_inches='tight')
plt.show()

# Phase 5-3: Steady-State vs. Dynamic vs. Terrain 비교
def assign_group(cond):
    for g, conds in CONDITION_GROUPS.items():
        if cond in conds: return g
    return 'Other'

df_cond['cond_group'] = df_cond['condition'].apply(assign_group)

fig, axes = plt.subplots(1, 2, figsize=(16, 6))
for ax, metric, ylabel in [(axes[0], 'mae', 'MAE (m/s)'),
                             (axes[1], 'jitter', 'Jitter')]:
    # factor_group별 × cond_group별 평균
    pivot = df_cond[~df_cond['c1_flagged']].pivot_table(
        index='factor_group', columns='cond_group', values=metric, aggfunc='mean'
    )
    pivot.plot(kind='bar', ax=ax, rot=30, colormap='Set2', edgecolor='black')
    ax.set_ylabel(ylabel); ax.set_title(f'Factor별 × 조건 유형별 {ylabel}')
    ax.legend(title='조건 유형')

plt.suptitle('Condition 유형별 성능: Dynamic 조건에서 어느 Factor가 효과적인가?',
             fontsize=13)
plt.tight_layout()
plt.savefig(OUTPUT_DIR / 'phase5_condgroup_comparison.png', bbox_inches='tight')
plt.show()

# Phase 6-1: 피험자별 MAE bar chart
fig, axes = plt.subplots(1, 3, figsize=(22, 7), sharey=False)

for ax, subj in zip(axes, SUBJECTS):
    subj_df = df[df['subject'] == subj].sort_values('mae')
    colors_subj = [factor_colors.get(row['factor_group'], 'gray')
                   for _, row in subj_df.iterrows()]
    alphas = [0.4 if row['c1_flagged'] else 0.9 for _, row in subj_df.iterrows()]
    bars = ax.barh(range(len(subj_df)), subj_df['mae'],
                   color=colors_subj, alpha=0.85, edgecolor='black', linewidth=0.5)
    ax.set_yticks(range(len(subj_df)))
    labels = [v + (' ⚠️' if c else '') for v, c in
               zip(subj_df['variant'], subj_df['c1_flagged'])]
    ax.set_yticklabels(labels, fontsize=9)
    ax.set_xlabel('MAE (m/s)')
    ax.set_title(f'Subject: {subj}', fontsize=12)
    ax.axvline(subj_df[subj_df['variant']=='Baseline']['mae'].values[0],
               color='red', linestyle='--', alpha=0.5, label='Baseline')
    ax.legend(fontsize=9)

plt.suptitle('피험자별 MAE 순위 (낮을수록 좋음)', fontsize=14)
plt.tight_layout()
plt.savefig(OUTPUT_DIR / 'phase6_per_subject_mae.png', bbox_inches='tight')
plt.show()

# Phase 6-2: CV scatter (피험자 간 일관성)
cv_data = df.groupby('exp_name').agg(
    mean_mae  = ('mae', 'mean'),
    std_mae   = ('mae', 'std'),
    variant   = ('variant', 'first'),
    factor    = ('factor_group', 'first'),
    c1        = ('c1_flagged', 'first'),
).reset_index()
cv_data['cv'] = cv_data['std_mae'] / (cv_data['mean_mae'] + 1e-9)

fig, ax = plt.subplots(figsize=(10, 7))
for _, row in cv_data.iterrows():
    color  = factor_colors.get(row['factor'], 'gray')
    marker = 'x' if row['c1'] else 'o'
    ax.scatter(row['mean_mae'], row['cv'], color=color, s=100, marker=marker)
    ax.annotate(row['variant'] + (' ⚠️' if row['c1'] else ''),
                (row['mean_mae'], row['cv']),
                xytext=(4, 4), textcoords='offset points', fontsize=8)

ax.set_xlabel('Mean MAE across subjects (m/s)')
ax.set_ylabel('CV = std/mean (낮을수록 일관적)')
ax.set_title('피험자 간 일관성: Mean MAE vs. CV')
# 이상적 영역 표시 (낮은 MAE + 낮은 CV)
ax.axhline(cv_data['cv'].median(), color='gray', linestyle=':', alpha=0.5, label='CV median')
ax.axvline(cv_data['mean_mae'].median(), color='gray', linestyle='--', alpha=0.5, label='MAE median')
ax.legend(fontsize=9)
plt.tight_layout()
plt.savefig(OUTPUT_DIR / 'phase6_cv_consistency.png', bbox_inches='tight')
plt.show()

from scipy.stats import wilcoxon, friedmanchisquare

def cohens_d(a, b):
    diff = np.array(a) - np.array(b)
    return np.mean(diff) / (np.std(diff) + 1e-9)

def bootstrap_ci(data, n_boot=10000, ci=0.95):
    data = np.array(data)
    boot = np.random.choice(data, (n_boot, len(data)), replace=True)
    means = boot.mean(axis=1)
    lo = np.percentile(means, (1 - ci) / 2 * 100)
    hi = np.percentile(means, (1 + ci) / 2 * 100)
    return lo, hi

# 각 실험 vs. baseline 비교
baseline_maes = df[df['exp_name'] == 'baseline'].set_index('subject')['mae']
baseline_jitters = df[df['exp_name'] == 'baseline'].set_index('subject')['jitter']

stat_rows = []
for exp_name in EXPERIMENT_META:
    if exp_name == 'baseline': continue
    exp_df = df[df['exp_name'] == exp_name].set_index('subject')
    if exp_df.empty: continue

    common_subjs = [s for s in SUBJECTS if s in exp_df.index and s in baseline_maes.index]
    if len(common_subjs) < 2: continue

    exp_maes    = [exp_df.loc[s, 'mae']    for s in common_subjs]
    exp_jitters = [exp_df.loc[s, 'jitter'] for s in common_subjs]
    base_maes_c   = [baseline_maes[s]    for s in common_subjs]
    base_jitters_c= [baseline_jitters[s] for s in common_subjs]

    d_mae    = cohens_d(base_maes_c, exp_maes)     # 양수 = exp가 더 좋음
    d_jitter = cohens_d(base_jitters_c, exp_jitters)

    # Wilcoxon (n=3이라 대부분 p=0.25)
    try:
        _, p_mae = wilcoxon(base_maes_c, exp_maes)
    except:
        p_mae = np.nan

    meta = EXPERIMENT_META[exp_name]
    stat_rows.append({
        'exp_name': exp_name, 'variant': meta[1], 'factor_group': meta[0],
        'c1_flagged': meta[2],
        'delta_mae':    np.mean(base_maes_c) - np.mean(exp_maes),
        'pct_change':   (np.mean(base_maes_c) - np.mean(exp_maes)) / np.mean(base_maes_c) * 100,
        'cohens_d_mae': d_mae,
        'delta_jitter': np.mean(base_jitters_c) - np.mean(exp_jitters),
        'cohens_d_jitter': d_jitter,
        'p_mae_wilcoxon': p_mae,
    })

df_stats = pd.DataFrame(stat_rows).sort_values('delta_mae', ascending=False)
print('=== MAE 개선도 (baseline 대비) ===')
display(df_stats[['variant','factor_group','delta_mae','pct_change',
                  'cohens_d_mae','cohens_d_jitter','p_mae_wilcoxon',
                  'c1_flagged']].round(4).to_string())

# Effect size 시각화
fig, axes = plt.subplots(1, 2, figsize=(16, 7))
valid = df_stats[~df_stats['c1_flagged']].sort_values('cohens_d_mae', ascending=False)

for ax, col, title, ref in [
    (axes[0], 'cohens_d_mae',    "Cohen's d (MAE): 양수=MAE 개선",    0),
    (axes[1], 'cohens_d_jitter', "Cohen's d (Jitter): 양수=Jitter 감소", 0),
]:
    colors_bar = ['steelblue' if v > 0 else 'salmon' for v in valid[col]]
    ax.barh(range(len(valid)), valid[col], color=colors_bar, edgecolor='black')
    ax.set_yticks(range(len(valid)))
    ax.set_yticklabels(valid['variant'], fontsize=9)
    ax.axvline(ref, color='black', linewidth=1)
    ax.set_xlabel("Cohen's d")
    ax.set_title(title)
    # 효과 크기 기준선
    for thresh, label in [(0.2, 'small'), (0.5, 'medium'), (0.8, 'large')]:
        ax.axvline(thresh, color='gray', linestyle=':', alpha=0.4)
        ax.axvline(-thresh, color='gray', linestyle=':', alpha=0.4)

plt.suptitle("Effect Size 요약 (n=3 subjects) — p-value보다 Cohen's d가 신뢰성 높음",
             fontsize=13)
plt.tight_layout()
plt.savefig(OUTPUT_DIR / 'phase7_effect_size.png', bbox_inches='tight')
plt.show()

# ─────────────────────────────────────────────
# Phase 8: train_log.csv 로드 및 수렴 곡선 비교
# ─────────────────────────────────────────────
import warnings
warnings.filterwarnings('ignore')

def load_train_logs(exp_name_list, subj=None):
    """
    주어진 실험 목록에 대해 train_log.csv를 로드.
    subj=None이면 첫 번째 피험자(m1_S004) 사용.
    """
    if subj is None: subj = SUBJECTS[0]
    logs = {}
    for exp_name in exp_name_list:
        p = EXP_ROOT / f'{exp_name}_Test-{subj}_seed42' / 'train_log.csv'
        if p.exists():
            try:
                logs[exp_name] = pd.read_csv(p)
            except:
                pass
    return logs


def plot_training_curves(exp_groups: dict, title: str, save_name: str):
    """
    exp_groups: {label: [exp_name1, exp_name2, ...]}
    각 그룹을 같은 색으로 표시 (피험자별로 alpha 구분)
    """
    fig, axes = plt.subplots(1, 2, figsize=(16, 5))

    all_logs = {}
    for label, exp_list in exp_groups.items():
        for exp_name in exp_list:
            for subj in SUBJECTS:
                p = EXP_ROOT / f'{exp_name}_Test-{subj}_seed42' / 'train_log.csv'
                if p.exists():
                    try:
                        all_logs[(label, exp_name, subj)] = pd.read_csv(p)
                    except:
                        pass

    colors_g = {label: c for label, c in zip(exp_groups.keys(), PALETTE)}
    alphas_s  = {s: a for s, a in zip(SUBJECTS, [0.9, 0.6, 0.4])}

    for ax, col, ylabel in [
        (axes[0], 'train_loss', 'Train Loss'),
        (axes[1], 'val_loss',   'Val Loss'),
    ]:
        for (label, exp_name, subj), log_df in all_logs.items():
            if col not in log_df.columns: continue
            ax.plot(log_df['epoch'], log_df[col],
                    color=colors_g[label],
                    alpha=alphas_s.get(subj, 0.5),
                    linewidth=1.5,
                    label=f'{label}_{subj}' if subj == SUBJECTS[0] else '')
        ax.set_xlabel('Epoch'); ax.set_ylabel(ylabel)
        ax.set_title(ylabel + ' Curves')
        ax.legend(fontsize=8)

    plt.suptitle(title, fontsize=13)
    plt.tight_layout()
    plt.savefig(OUTPUT_DIR / f'phase8_{save_name}.png', bbox_inches='tight')
    plt.show()


# Horizon 비교
plot_training_curves(
    {'Baseline': ['baseline'],
     'H=5':  ['exp_multistep_H05'],
     'H=10': ['exp_multistep_H10'],
     'H=20': ['exp_multistep_H20']},
    title='Prediction Horizon별 학습 곡선 비교',
    save_name='horizon_training_curves'
)

# Architecture 비교
plot_training_curves(
    {'Baseline':  ['baseline'],
     'GRU_h32': ['exp_tcn_gru_head_h32'],
     'GRU_h64': ['exp_tcn_gru_head_h64']},
    title='Architecture별 학습 곡선 비교',
    save_name='arch_training_curves'
)

# Phase 8-2: 수렴 통계 테이블
conv_rows = []
for exp_name in EXPERIMENT_META:
    for subj in SUBJECTS:
        p = EXP_ROOT / f'{exp_name}_Test-{subj}_seed42' / 'train_log.csv'
        if not p.exists(): continue
        log = pd.read_csv(p)
        if log.empty: continue

        initial_val  = log['val_loss'].iloc[0]
        best_val     = log['val_loss'].min()
        final_train  = log['train_loss'].iloc[-1]
        final_val    = log['val_loss'].iloc[-1]
        improvement  = (initial_val - best_val) / (initial_val + 1e-9) * 100

        meta = EXPERIMENT_META.get(exp_name, ('Unknown', exp_name, False))
        conv_rows.append({
            'exp_name': exp_name, 'subject': subj,
            'factor_group': meta[0], 'variant': meta[1],
            'final_epoch': len(log),
            'best_val_loss': best_val,
            'val_improvement_pct': improvement,
            'overfit_gap': final_val - final_train,
        })

df_conv = pd.DataFrame(conv_rows)
conv_summary = df_conv.groupby('exp_name').agg(
    mean_epochs    = ('final_epoch',         'mean'),
    mean_best_val  = ('best_val_loss',        'mean'),
    mean_improve   = ('val_improvement_pct',  'mean'),
    mean_overfit   = ('overfit_gap',          'mean'),
).round(3)

print('=== 수렴 통계 (factor group별 정렬) ===')
display(conv_summary.sort_values('mean_epochs'))

# ─────────────────────────────────────────────
# Phase 9-1: 6-Panel 메인 요약 피규어
# ─────────────────────────────────────────────
fig = plt.figure(figsize=(22, 14))
gs  = gridspec.GridSpec(2, 3, figure=fig, hspace=0.45, wspace=0.35)

# (a) Pareto Frontier (간략 버전)
ax_a = fig.add_subplot(gs[0, 0])
for fg in agg['factor_group'].unique():
    s = agg[agg['factor_group'] == fg]
    ax_a.scatter(s['mean_mae'], s['mean_jitter'],
                 color=factor_colors.get(fg, 'gray'), s=60, label=fg, alpha=0.8)
pareto_pts_sorted = pareto_pts.sort_values('mean_mae')
ax_a.plot(pareto_pts_sorted['mean_mae'], pareto_pts_sorted['mean_jitter'],
          'k--', alpha=0.5, linewidth=1.5)
ax_a.set_xlabel('MAE (m/s)'); ax_a.set_ylabel('Jitter')
ax_a.set_title('(a) Pareto Frontier')
ax_a.legend(fontsize=6, loc='upper right')

# (b) EMA α tradeoff
ax_b = fig.add_subplot(gs[0, 1])
ema_agg = agg[agg['factor_group'] == 'EMA'][['variant','mean_mae','mean_jitter','mean_lag']]
base_row = agg[agg['exp_name'] == 'baseline'][['mean_mae','mean_jitter','mean_lag']].iloc[0]
all_ema = pd.concat([pd.DataFrame([{'variant': 'Baseline(α=0)',
                                     'mean_mae': base_row['mean_mae'],
                                     'mean_jitter': base_row['mean_jitter'],
                                     'mean_lag': base_row['mean_lag']}]),
                     ema_agg], ignore_index=True)
x = range(len(all_ema))
ax_b2 = ax_b.twinx()
ax_b.plot(x, all_ema['mean_mae'],    'b-o', label='MAE', linewidth=2)
ax_b.plot(x, all_ema['mean_jitter'], 'g-s', label='Jitter', linewidth=2)
ax_b2.plot(x, all_ema['mean_lag'],   'r-^', label='Lag (ms)', linewidth=2, linestyle='--')
ax_b.set_xticks(x); ax_b.set_xticklabels(all_ema['variant'], rotation=30, ha='right', fontsize=8)
ax_b.set_ylabel('MAE / Jitter'); ax_b2.set_ylabel('Lag (ms)', color='red')
ax_b.set_title('(b) EMA α Tradeoff')
ax_b.legend(loc='upper left', fontsize=8); ax_b2.legend(loc='upper right', fontsize=8)

# (c) Horizon 라인 플롯
ax_c = fig.add_subplot(gs[0, 2])
h_data = agg[agg['factor_group'].isin(['Baseline','Horizon'])].sort_values('variant')
for metric, style, color in [('mean_mae','o-','blue'),('mean_jitter','s--','green')]:
    ax_c.plot(h_data['variant'], h_data[metric], style, color=color,
              linewidth=2, label=metric)
ax_c.set_xlabel('Horizon'); ax_c.set_ylabel('Metric')
ax_c.set_title('(c) Prediction Horizon')
ax_c.legend(fontsize=9)

# (d) Overlap aggregation bar
ax_d = fig.add_subplot(gs[1, 0])
ovlp_data = agg[agg['factor_group'].isin(['Baseline','Overlap'])]
x_d = range(len(ovlp_data))
ax_d.bar(x_d, ovlp_data['mean_mae'], alpha=0.7, color=PALETTE[:len(ovlp_data)],
          edgecolor='black')
ax_d.set_xticks(x_d); ax_d.set_xticklabels(ovlp_data['variant'], rotation=30, ha='right', fontsize=9)
ax_d.set_ylabel('MAE (m/s)'); ax_d.set_title('(d) Overlap Aggregation')

# (e) Condition heatmap (top-5 실험)
ax_e = fig.add_subplot(gs[1, 1])
top5_exps = agg.sort_values('mean_mae').head(5)['exp_name'].tolist()
if 'baseline' not in top5_exps: top5_exps = ['baseline'] + top5_exps[:4]
cond_top5 = df_cond[df_cond['exp_name'].isin(top5_exps)]
pivot_e   = cond_top5.pivot_table(index='exp_name', columns='condition',
                                   values='mae', aggfunc='mean')
sns.heatmap(pivot_e.round(3), annot=True, fmt='.3f', cmap='YlOrRd',
            linewidths=0.3, ax=ax_e, cbar=False, annot_kws={'size': 7})
ax_e.set_title('(e) Top-5 × Condition MAE')
ax_e.tick_params(axis='x', rotation=45, labelsize=7)
ax_e.tick_params(axis='y', labelsize=7)

# (f) Best config trajectory on accel_sine
ax_f = fig.add_subplot(gs[1, 2])
best_exp = agg.sort_values('mean_mae').iloc[0]['exp_name']
best_subj = df[df['exp_name'] == best_exp].sort_values('mae').iloc[0]['subject']

if (best_exp, best_subj) in seq_store:
    seqs = seq_store[(best_exp, best_subj)]
    yp   = seqs['y_pred'].squeeze() if seqs['y_pred'] is not None else None
    yt   = seqs['y_true'].squeeze() if seqs['y_true'] is not None else None
    if yp is not None and yt is not None:
        if yp.ndim > 1: yp = yp[:, 0]
        if yt.ndim > 1: yt = yt[:, 0]
        show_len = min(500, len(yp))
        t = np.arange(show_len) / 100.0  # 초 단위
        ax_f.plot(t, yt[:show_len], 'k-', linewidth=1.5, label='Ground Truth', alpha=0.8)
        ax_f.plot(t, yp[:show_len], 'r-', linewidth=1.5, label=f'{best_exp}', alpha=0.8)
        ax_f.set_xlabel('Time (s)'); ax_f.set_ylabel('Speed (m/s)')
        ax_f.set_title(f'(f) Best Config Trajectory ({best_subj})')
        ax_f.legend(fontsize=8)
else:
    ax_f.text(0.5, 0.5, '시계열 데이터 없음\n(return_seqs=True로 재실행)',
              ha='center', va='center', transform=ax_f.transAxes)

plt.suptitle('SpeedEstimation Factor Analysis — 최종 요약', fontsize=15, fontweight='bold')
plt.savefig(OUTPUT_DIR / 'phase9_main_summary.png', bbox_inches='tight', dpi=150)
plt.show()

# Phase 9-2: 최종 권고 테이블
recommendations = pd.DataFrame([
    {'Use Case': '실시간 제어 (latency 우선)',
     '추천 Config': 'baseline + EMA(α=0.90)',
     '이유': '저지연 + 충분한 smoothness. metrics.json MAE ≒ EMA 적용 후 MAE'},
    {'Use Case': '오프라인 분석 (accuracy 우선)',
     '추천 Config': 'exp_tcn_gru_head_h32',
     '이유': '피험자 평균 MAE 최저. 단, m2_S008에서 분산이 큼 (주의)'},
    {'Use Case': 'Dynamic 조건 (가속/감속 多)',
     '추천 Config': 'exp_multistep_H10_ema_a090',
     '이유': 'H=10 시간 맥락 + EMA smoothing 복합 적용'},
    {'Use Case': '피험자 간 일반화 우선',
     '추천 Config': 'Exp_IMU_Orientation',
     '이유': '피험자 간 CV 낮음 (일관성 높음)'},
    {'Use Case': 'Smoothness 최우선',
     '추천 Config': 'exp_multistep_H10_overlap_median',
     '이유': '구조적 overlap averaging으로 jitter 감소'},
])

display(recommendations)
recommendations.to_csv(OUTPUT_DIR / 'recommendations.csv', index=False)

print('\n[주의 사항 요약]')
print('  ✅  AR feedback 실험(k1, sched)은 teacher_forcing 구현 후 재학습 완료')
print('  ⚠️  C2: multi-step metrics.json 값 ≠ compare_results.py 값')
print('  ⚠️  C3: EMA 실험의 metrics.json = pre-EMA MAE')
print('\n분석 결과 저장 경로:', OUTPUT_DIR)

# Phase 9-3: LaTeX 요약 테이블 (상위 8개 실험)
top8 = agg.sort_values('mean_mae').head(8)[[
    'exp_name', 'variant', 'mean_mae', 'std_mae',
    'mean_jitter', 'std_jitter', 'mean_lag', 'mean_r2'
]].copy()

# baseline 대비 MAE 변화율
baseline_mean_mae = agg[agg['exp_name']=='baseline']['mean_mae'].values[0]
top8['delta_mae_pct'] = (baseline_mean_mae - top8['mean_mae']) / baseline_mean_mae * 100

print('=== Top-8 요약 테이블 ===')
display(top8.round(4))

# LaTeX 출력
print('\n=== LaTeX 형식 ===')
print(top8[['variant','mean_mae','std_mae','mean_jitter','mean_lag','mean_r2','delta_mae_pct']]
      .rename(columns={
          'variant': 'Config',
          'mean_mae': 'MAE',
          'std_mae': '±std',
          'mean_jitter': 'Jitter',
          'mean_lag': 'Lag (ms)',
          'mean_r2': 'R²',
          'delta_mae_pct': 'ΔMAE (%)',
      })
      .to_latex(index=False, float_format='%.4f', escape=False))

