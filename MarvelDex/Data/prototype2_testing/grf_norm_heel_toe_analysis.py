"""
GRF-Normalized FSR Analysis: Heel (FSR2), Toe (FSR7), Sum
- Scatter: FSR vs GRF/BW (stance only)
- Gait cycle profiles (before vs after)
- Within-trial temporal trends
"""
import matplotlib
matplotlib.use('Agg')

import pandas as pd
import numpy as np
from scipy import signal, stats
import matplotlib.pyplot as plt
import seaborn as sns
import os, io, warnings
warnings.filterwarnings('ignore')

plt.rcParams['figure.dpi'] = 150
sns.set_style('whitegrid')

RESULT_DIR = 'result'
ROBOT_FS = 500
GRF_FS = 2000
GRF_THRESH = 50  # N

LEFT_FSR = [f'LeftFSR{i}' for i in range(1, 15)]
RIGHT_FSR = [f'RightFSR{i}' for i in range(1, 15)]

# Heel = FSR2, Toe = FSR7 (from peak timing analysis)
HEEL_CH = 2
TOE_CH = 7

TARGETS = ['before_level', 'after_level']
LABELS = {'before_level': 'Before (S1, 1st use)', 'after_level': 'After (S2, 3rd use)'}
COLORS = {'before_level': '#1f77b4', 'after_level': '#2ca02c'}

SIGNAL_NAMES = ['Heel (FSR2)', 'Toe (FSR7)', 'FSR Sum']

# ============================================================
# Load data
# ============================================================
print("=== Loading data ===")
robot_files = {
    'before_level': 'before/level_075mps_lv0_01_processed.csv',
    'after_level':  'after/level_075mps_lv0_01_processed.csv',
}
grf_files = {
    'before_level': 'before/level_075mps_lv0_01.csv',
    'after_level':  'after/level_075mps_lv0_01.csv',
}

data = {}
for key, path in robot_files.items():
    data[key] = pd.read_csv(path)

def load_grf_csv(filepath):
    rows = []
    with open(filepath, 'r', encoding='utf-8-sig') as f:
        for i, line in enumerate(f):
            if i < 7: continue
            stripped = line.strip()
            if stripped.startswith('Trajectories'): break
            if stripped == '': continue
            rows.append(stripped)
    df = pd.read_csv(io.StringIO('\n'.join(rows)), header=None)
    return pd.DataFrame({'FP1_Fz': df[13], 'FP2_Fz': df[4]})

def downsample_grf(grf_fz, ratio=4):
    nyq = GRF_FS / 2
    b, a = signal.butter(4, (ROBOT_FS/2)*0.9 / nyq, btype='low')
    return signal.filtfilt(b, a, grf_fz)[::ratio]

# GRF: downsample + L/R assignment (no lag correction — system is synchronized)
# FP1=Left, FP2=Right (consistent across all trials from main analysis)
grf_aligned = {}
body_weight = {}

for key in TARGETS:
    grf_raw = load_grf_csv(grf_files[key])
    fp1_ds = np.abs(downsample_grf(grf_raw['FP1_Fz'].values))
    fp2_ds = np.abs(downsample_grf(grf_raw['FP2_Fz'].values))
    robot_len = len(data[key])
    fp1_ds = fp1_ds[:robot_len]
    fp2_ds = fp2_ds[:robot_len]
    if len(fp1_ds) < robot_len:
        fp1_ds = np.pad(fp1_ds, (0, robot_len - len(fp1_ds)), mode='edge')
        fp2_ds = np.pad(fp2_ds, (0, robot_len - len(fp2_ds)), mode='edge')

    grf_aligned[key] = {'Left': fp1_ds, 'Right': fp2_ds}
    bw_N = fp1_ds[:1000].mean() + fp2_ds[:1000].mean()
    body_weight[key] = bw_N
    print(f"  {key}: BW={bw_N:.1f}N ({bw_N/9.81:.1f}kg)")

# Walking onset
def detect_walking_onset(df, fs=500):
    hip = (df['LeftHipAngle'] + df['RightHipAngle']) / 2
    rstd = hip.rolling(window=fs).std()
    init_std = hip.iloc[:fs].std()
    thresh = max(init_std * 5, 0.5)
    exceed = rstd[rstd > thresh].index
    return max(0, exceed[0] - fs//2) if len(exceed) > 0 else 0

walk_onsets = {k: detect_walking_onset(data[k]) for k in TARGETS}

# Gait segmentation
def detect_strides(fsr_sum, fs=500):
    b, a = signal.butter(4, 15/(fs/2), btype='low')
    filt = signal.filtfilt(b, a, fsr_sum)
    threshold = np.mean(filt) * 0.15
    above = filt > threshold
    trans = np.diff(above.astype(int))
    hs = np.where(trans == 1)[0]
    to = np.where(trans == -1)[0]
    strides = []
    for i, h in enumerate(hs[:-1]):
        to_after = to[to > h + 150]
        if len(to_after) == 0: continue
        t = to_after[0]
        nh = hs[i+1]
        if nh > t + 100:
            strides.append({'hs': h, 'to': t, 'nhs': nh})
    return strides

gait = {}
for key in TARGETS:
    gait[key] = {}
    onset = walk_onsets[key]
    for side, cols in [('Left', LEFT_FSR), ('Right', RIGHT_FSR)]:
        fsr_sum = data[key][cols].sum(axis=1).values[onset:]
        gait[key][side] = detect_strides(fsr_sum)
        print(f"  {key} {side}: {len(gait[key][side])} strides")

# ============================================================
# Helper: get the 3 FSR signals for a given side
# ============================================================
def get_fsr_signals(data_df, side):
    """Returns dict of {'heel': array, 'toe': array, 'sum': array}"""
    cols = LEFT_FSR if side == 'Left' else RIGHT_FSR
    prefix = 'Left' if side == 'Left' else 'Right'
    return {
        'heel': data_df[f'{prefix}FSR{HEEL_CH}'].values.astype(float),
        'toe':  data_df[f'{prefix}FSR{TOE_CH}'].values.astype(float),
        'sum':  data_df[cols].sum(axis=1).values.astype(float),
    }

# ============================================================
# Compute data-driven activation thresholds from standing period
# Threshold = standing mean + 3 * standing std (per channel, per trial)
# ============================================================
print("\n=== Computing activation thresholds from standing noise ===")
fsr_thresholds = {}  # fsr_thresholds[key][side] = {'heel': val, 'toe': val, 'sum': val}

for key in TARGETS:
    fsr_thresholds[key] = {}
    onset = walk_onsets[key]
    standing_df = data[key].iloc[:onset]

    for side in ['Left', 'Right']:
        prefix = 'Left' if side == 'Left' else 'Right'
        cols = LEFT_FSR if side == 'Left' else RIGHT_FSR

        heel_stand = standing_df[f'{prefix}FSR{HEEL_CH}'].values.astype(float)
        toe_stand = standing_df[f'{prefix}FSR{TOE_CH}'].values.astype(float)
        sum_stand = standing_df[cols].sum(axis=1).values.astype(float)

        th = {
            'heel': np.mean(heel_stand) + 3 * np.std(heel_stand),
            'toe':  np.mean(toe_stand) + 3 * np.std(toe_stand),
            'sum':  np.mean(sum_stand) + 3 * np.std(sum_stand),
        }
        fsr_thresholds[key][side] = th
        print(f"  {key} {side}: heel>{th['heel']:.1f}, toe>{th['toe']:.1f}, sum>{th['sum']:.1f}")

# ============================================================
# PLOT 1: Per-stride peak scatter — FSR peak vs GRF peak / BW
#   Each stride = one point. Eliminates COP movement effect.
#   Heel: peak FSR2 during first 40% of stance (loading response)
#   Toe:  peak FSR7 during last 40% of stance (push-off)
#   Sum:  peak FSR sum during entire stance
# ============================================================
print("\n=== Generating per-stride peak scatter plots ===")

fig, axes = plt.subplots(4, 2, figsize=(14, 18))

# Helper: extract per-stride peaks
def extract_stride_peaks(data_df, grf_fz, strides, onset, side, bw):
    """For each stride, extract peak FSR and corresponding peak GRF/BW."""
    prefix = 'Left' if side == 'Left' else 'Right'
    cols = LEFT_FSR if side == 'Left' else RIGHT_FSR
    fsr2 = data_df[f'{prefix}FSR{HEEL_CH}'].values.astype(float)
    fsr7 = data_df[f'{prefix}FSR{TOE_CH}'].values.astype(float)
    fsr_sum = data_df[cols].sum(axis=1).values.astype(float)

    results = {'heel_fsr': [], 'heel_grf_bw': [],
               'toe_fsr': [], 'toe_grf_bw': [],
               'sum_fsr': [], 'sum_grf_bw': [],
               'stride_num': []}

    for si, s in enumerate(strides):
        hs = onset + s['hs']
        to = onset + s['to']
        nhs = onset + s['nhs']
        if to >= len(fsr2) or to >= len(grf_fz):
            continue

        stance_len = to - hs
        if stance_len < 20:
            continue

        # Heel: peak in first 40% of stance
        early_end = hs + int(stance_len * 0.4)
        heel_peak = fsr2[hs:early_end].max()
        # GRF at the time of heel peak
        heel_peak_idx = hs + np.argmax(fsr2[hs:early_end])
        heel_grf = grf_fz[heel_peak_idx] / bw

        # Toe: peak in last 40% of stance
        late_start = to - int(stance_len * 0.4)
        toe_peak = fsr7[late_start:to].max()
        toe_peak_idx = late_start + np.argmax(fsr7[late_start:to])
        toe_grf = grf_fz[toe_peak_idx] / bw

        # Sum: peak during entire stance
        sum_peak = fsr_sum[hs:to].max()
        sum_peak_idx = hs + np.argmax(fsr_sum[hs:to])
        sum_grf = grf_fz[sum_peak_idx] / bw

        results['heel_fsr'].append(heel_peak)
        results['heel_grf_bw'].append(heel_grf)
        results['toe_fsr'].append(toe_peak)
        results['toe_grf_bw'].append(toe_grf)
        results['sum_fsr'].append(sum_peak)
        results['sum_grf_bw'].append(sum_grf)
        results['stride_num'].append(si)

    return {k: np.array(v) for k, v in results.items()}

# Extract for all
stride_peaks = {}
for key in TARGETS:
    stride_peaks[key] = {}
    for side in ['Left', 'Right']:
        stride_peaks[key][side] = extract_stride_peaks(
            data[key], grf_aligned[key][side],
            gait[key][side], walk_onsets[key], side, body_weight[key])

# Row 0-2: Heel, Toe, Sum per-stride peaks
sig_configs = [
    ('heel', 'Heel (FSR2) — peak in first 40% stance'),
    ('toe',  'Toe (FSR7) — peak in last 40% stance'),
    ('sum',  'FSR Sum — peak during stance'),
]

for row, (sig_key, sig_name) in enumerate(sig_configs):
    fsr_key = f'{sig_key}_fsr'
    grf_key = f'{sig_key}_grf_bw'

    for col, side in enumerate(['Left', 'Right']):
        ax = axes[row, col]

        for key in TARGETS:
            sp = stride_peaks[key][side]
            x = sp[grf_key]
            y = sp[fsr_key]
            sn = sp['stride_num']

            # Color by stride number (time progression)
            sc = ax.scatter(x, y, c=sn, cmap='viridis' if key == 'before_level' else 'autumn',
                           s=15, alpha=0.7, edgecolors='none',
                           label=LABELS[key], vmin=0, vmax=sn.max())

            # Linear regression
            if len(x) > 2:
                sl, ic, r, p, se = stats.linregress(x, y)
                x_line = np.array([x.min(), x.max()])
                ax.plot(x_line, sl * x_line + ic, color=COLORS[key],
                        linewidth=2, linestyle='--',
                        label=f'  R²={r**2:.3f}, slope={sl:.1f}')

        ax.set_xlabel('Peak GRF / BW (at FSR peak time)')
        ax.set_ylabel(f'Peak {sig_name.split("—")[0].strip()} (ADC)')
        ax.set_title(f'{sig_name} — {side}', fontsize=10)
        ax.legend(fontsize=6, markerscale=2)

        # Colorbar for last key
        if col == 1:
            cb = plt.colorbar(sc, ax=ax, pad=0.01, aspect=30)
            cb.set_label('Stride #', fontsize=7)
            cb.ax.tick_params(labelsize=6)

# Row 3: FSR Sum per-stride peaks, time-thirds
third_colors = ['#2196F3', '#FF9800', '#F44336']
third_labels = ['Early (1st third)', 'Mid (2nd third)', 'Late (3rd third)']

plot_configs = [
    ('before_level', 'Left',  'Before (S1) — Left'),
    ('after_level',  'Right', 'After (S2) — Right'),
]

for col, (key, side, title) in enumerate(plot_configs):
    ax = axes[3, col]
    sp = stride_peaks[key][side]
    x_all = sp['sum_grf_bw']
    y_all = sp['sum_fsr']
    sn_all = sp['stride_num']
    n = len(x_all)
    third = n // 3

    for ti, (s_start, s_end) in enumerate([(0, third), (third, 2*third), (2*third, n)]):
        clr = third_colors[ti]
        lbl = third_labels[ti]
        x = x_all[s_start:s_end]
        y = y_all[s_start:s_end]

        ax.scatter(x, y, s=20, alpha=0.6, color=clr, edgecolors='none', label=lbl)

        if len(x) > 2:
            sl, ic, r, p, se = stats.linregress(x, y)
            x_line = np.array([x_all.min(), x_all.max()])
            ax.plot(x_line, sl * x_line + ic, color=clr, linewidth=2, linestyle='-')

    ax.set_xlabel('Peak GRF / BW')
    ax.set_ylabel('Peak FSR Sum (ADC)')
    ax.set_title(f'FSR Sum Time-Thirds — {title}', fontsize=11)
    ax.legend(fontsize=7, markerscale=2)

plt.suptitle('Per-Stride Peak FSR vs Peak GRF/BW (1 point = 1 stride)\n'
             'Row 1-3: color = stride # (dark=early, light=late). Row 4: time-thirds.',
             fontsize=13, fontweight='bold')
plt.tight_layout()
plt.savefig(os.path.join(RESULT_DIR, 'ht_01_scatter_fsr_vs_grf_bw.png'), dpi=150, bbox_inches='tight')
plt.close()
print("  Saved: ht_01_scatter_fsr_vs_grf_bw.png")

# ============================================================
# PLOT 2: Gait-cycle normalized profiles (before vs after)
# ============================================================
gc_pct = np.linspace(0, 100, 100)

fig, axes = plt.subplots(3, 2, figsize=(14, 12))

for row, (sig_key, sig_name) in enumerate(zip(['heel', 'toe', 'sum'], SIGNAL_NAMES)):
    for col, side in enumerate(['Left', 'Right']):
        ax = axes[row, col]

        for key in TARGETS:
            onset = walk_onsets[key]
            fsr_sigs = get_fsr_signals(data[key], side)
            fsr_full = fsr_sigs[sig_key]
            grf_full = grf_aligned[key][side]
            bw = body_weight[key]

            profiles = []
            for s in gait[key][side]:
                hs_abs = onset + s['hs']
                nhs_abs = onset + s['nhs']
                if nhs_abs >= len(fsr_full) or nhs_abs >= len(grf_full):
                    continue
                fsr_gc = fsr_full[hs_abs:nhs_abs]
                grf_gc = grf_full[hs_abs:nhs_abs]
                if len(fsr_gc) < 20:
                    continue
                # Normalize FSR by GRF/BW at each point
                grf_bw = grf_gc / bw
                # Where GRF > threshold, compute ratio; else keep FSR raw divided by 1
                # Actually: normalize FSR by BW only (simpler, consistent)
                # Let's show FSR / BW so it's weight-normalized
                fsr_norm = fsr_gc / bw
                fsr_interp = np.interp(np.linspace(0, 1, 100),
                                       np.linspace(0, 1, len(fsr_norm)), fsr_norm)
                profiles.append(fsr_interp)

            profiles = np.array(profiles)
            mean_p = np.mean(profiles, axis=0)
            std_p = np.std(profiles, axis=0)
            ax.plot(gc_pct, mean_p, color=COLORS[key], linewidth=2,
                    label=f'{LABELS[key]} (n={len(profiles)})')
            ax.fill_between(gc_pct, mean_p - std_p, mean_p + std_p,
                            color=COLORS[key], alpha=0.12)

        ax.set_ylabel(f'{sig_name} / BW (ADC/N)')
        ax.set_title(f'{sig_name} — {side} Foot', fontsize=10)
        ax.set_xlim(0, 100)
        if row == 2:
            ax.set_xlabel('Gait Cycle (%)')
        if row == 0:
            ax.legend(fontsize=8)

plt.suptitle('Body-Weight Normalized FSR over Gait Cycle: Before vs After\n'
             'Heel (FSR2), Toe (FSR7), Sum — divided by body weight (N)',
             fontsize=13, fontweight='bold')
plt.tight_layout()
plt.savefig(os.path.join(RESULT_DIR, 'ht_02_gait_cycle_bw_normalized.png'), dpi=150, bbox_inches='tight')
plt.close()
print("  Saved: ht_02_gait_cycle_bw_normalized.png")

# ============================================================
# PLOT 3: Within-trial temporal trend (stride-by-stride)
# ============================================================
fig, axes = plt.subplots(3, 4, figsize=(20, 12))

for row, (sig_key, sig_name) in enumerate(zip(['heel', 'toe', 'sum'], SIGNAL_NAMES)):
    col_idx = 0
    for key in TARGETS:
        for side in ['Left', 'Right']:
            ax = axes[row, col_idx]
            onset = walk_onsets[key]
            fsr_sigs = get_fsr_signals(data[key], side)
            fsr_full = fsr_sigs[sig_key]
            grf_full = grf_aligned[key][side]
            bw = body_weight[key]

            # Per-stride: mean FSR/BW during stance where GRF > thresh
            stride_ratios = []
            for s in gait[key][side]:
                hs_abs = onset + s['hs']
                to_abs = onset + s['to']
                if to_abs >= len(fsr_full) or to_abs >= len(grf_full):
                    continue
                fsr_stance = fsr_full[hs_abs:to_abs]
                grf_stance = grf_full[hs_abs:to_abs]
                valid = grf_stance > GRF_THRESH
                if valid.sum() > 10:
                    stride_ratios.append(np.mean(fsr_stance[valid]) / bw)
                else:
                    stride_ratios.append(np.nan)

            y = np.array(stride_ratios)
            x = np.arange(len(y))
            valid = ~np.isnan(y)

            ax.scatter(x[valid], y[valid], s=10, alpha=0.4, color=COLORS[key])

            if valid.sum() > 2:
                sl, ic, r, p, se = stats.linregress(x[valid], y[valid])
                ax.plot(x, ic + sl * x, 'k-', linewidth=2,
                        label=f'slope={sl:.6f}\np={p:.4f}')
                ax.legend(fontsize=7)

            ax.set_xlabel('Stride #')
            if col_idx == 0:
                ax.set_ylabel(f'{sig_name}/BW (ADC/N)')
            title_short = 'Before' if key == 'before_level' else 'After'
            ax.set_title(f'{title_short} {side}', fontsize=9)
            col_idx += 1

plt.suptitle('Within-Trial Temporal Trend: FSR/BW per Stride (Stance Mean)\n'
             'Negative slope = sensitivity decreasing over time',
             fontsize=13, fontweight='bold')
plt.tight_layout()
plt.savefig(os.path.join(RESULT_DIR, 'ht_03_within_trial_trend.png'), dpi=150, bbox_inches='tight')
plt.close()
print("  Saved: ht_03_within_trial_trend.png")

# ============================================================
# PLOT 4: Within-trial gait cycle evolution (early/mid/late)
# ============================================================
fig, axes = plt.subplots(3, 4, figsize=(20, 12))
third_colors = ['#2196F3', '#FF9800', '#F44336']
third_labels = ['Early', 'Mid', 'Late']

for row, (sig_key, sig_name) in enumerate(zip(['heel', 'toe', 'sum'], SIGNAL_NAMES)):
    col_idx = 0
    for key in TARGETS:
        for side in ['Left', 'Right']:
            ax = axes[row, col_idx]
            onset = walk_onsets[key]
            fsr_sigs = get_fsr_signals(data[key], side)
            fsr_full = fsr_sigs[sig_key]
            grf_full = grf_aligned[key][side]
            bw = body_weight[key]
            strides_list = gait[key][side]

            # Extract BW-normalized gait cycle profiles
            profiles = []
            for s in strides_list:
                hs_abs = onset + s['hs']
                nhs_abs = onset + s['nhs']
                if nhs_abs >= len(fsr_full):
                    continue
                seg = fsr_full[hs_abs:nhs_abs] / bw
                if len(seg) < 20:
                    continue
                profiles.append(np.interp(np.linspace(0, 1, 100),
                                          np.linspace(0, 1, len(seg)), seg))
            profiles = np.array(profiles)
            n = len(profiles)
            third = n // 3

            for ti, (s_start, s_end) in enumerate([(0, third), (third, 2*third), (2*third, n)]):
                subset = profiles[s_start:s_end]
                if len(subset) == 0:
                    continue
                mean_p = np.mean(subset, axis=0)
                ax.plot(gc_pct, mean_p, color=third_colors[ti], linewidth=1.5,
                        label=f'{third_labels[ti]} ({len(subset)})')

            ax.set_xlim(0, 100)
            if col_idx == 0:
                ax.set_ylabel(f'{sig_name}/BW')
            if row == 2:
                ax.set_xlabel('Gait Cycle (%)')
            title_short = 'Before' if key == 'before_level' else 'After'
            ax.set_title(f'{title_short} {side}', fontsize=9)
            if row == 0:
                ax.legend(fontsize=6)
            col_idx += 1

plt.suptitle('Within-Trial Gait Cycle Evolution: Early / Mid / Late\n'
             'Heel (FSR2), Toe (FSR7), Sum — BW-normalized',
             fontsize=13, fontweight='bold')
plt.tight_layout()
plt.savefig(os.path.join(RESULT_DIR, 'ht_04_within_trial_gait_evolution.png'), dpi=150, bbox_inches='tight')
plt.close()
print("  Saved: ht_04_within_trial_gait_evolution.png")

# ============================================================
# PLOT 5: Scatter FSR vs GRF/BW — split by early/late half
#   FSR > noise threshold (data-driven)
# ============================================================
fig, axes = plt.subplots(3, 2, figsize=(14, 14))

for row, (sig_key, sig_name) in enumerate(zip(['heel', 'toe', 'sum'], SIGNAL_NAMES)):
    for col, side in enumerate(['Left', 'Right']):
        ax = axes[row, col]

        for key in TARGETS:
            onset = walk_onsets[key]
            fsr_sigs = get_fsr_signals(data[key], side)
            fsr_walk = fsr_sigs[sig_key][onset:]
            grf_walk = grf_aligned[key][side][onset:]
            bw = body_weight[key]
            th = fsr_thresholds[key][side][sig_key]
            n_walk = len(fsr_walk)
            half = n_walk // 2

            for period, sl_range in [('Early', slice(0, half)), ('Late', slice(half, n_walk))]:
                fsr_seg = fsr_walk[sl_range]
                grf_seg = grf_walk[sl_range]
                active = fsr_seg > th
                fsr_pts = fsr_seg[active]
                grf_bw_pts = grf_seg[active] / bw

                n_plot = min(5000, len(fsr_pts))
                if n_plot == 0:
                    continue
                idx = np.random.choice(len(fsr_pts), n_plot, replace=False)

                key_short = 'Bef' if key == 'before_level' else 'Aft'
                clr = COLORS[key] if period == 'Early' else '#d62728' if key == 'before_level' else '#8B0000'
                alpha = 0.08 if period == 'Early' else 0.15

                ax.scatter(grf_bw_pts[idx], fsr_pts[idx], s=1, alpha=alpha, color=clr)

                if len(fsr_pts) > 10:
                    sl, ic, r, p, se = stats.linregress(grf_bw_pts, fsr_pts)
                    x_line = np.array([grf_bw_pts.min(), grf_bw_pts.max()])
                    ls = '-' if period == 'Early' else '--'
                    ax.plot(x_line, sl * x_line + ic, color=clr, linewidth=2,
                            linestyle=ls, label=f'{key_short} {period}: R²={r**2:.3f}')

        ax.set_xlabel('GRF / Body Weight')
        ax.set_ylabel(f'{sig_name} (ADC)')
        ax.set_title(f'{sig_name} — {side}', fontsize=10)
        ax.legend(fontsize=6, markerscale=5)

plt.suptitle('FSR vs GRF/BW: Early vs Late Half of Trial\n'
             'FSR > noise threshold — within-trial calibration shift',
             fontsize=13, fontweight='bold')
plt.tight_layout()
plt.savefig(os.path.join(RESULT_DIR, 'ht_05_scatter_early_vs_late.png'), dpi=150, bbox_inches='tight')
plt.close()
print("  Saved: ht_05_scatter_early_vs_late.png")

# ============================================================
# Print summary statistics
# ============================================================
print("\n" + "=" * 70)
print("SUMMARY")
print("=" * 70)

print(f"\nSensor mapping: Heel = FSR{HEEL_CH}, Toe = FSR{TOE_CH}")
print(f"Body weight: Before = {body_weight['before_level']/9.81:.1f} kg, "
      f"After = {body_weight['after_level']/9.81:.1f} kg")

print(f"\n{'Signal':<12} {'Trial':<28} {'Side':<6} {'Mean FSR/BW':<14} {'R² vs GRF/BW':<14} {'Slope':<10}")
print("-" * 84)

for sig_key, sig_name in zip(['heel', 'toe', 'sum'], SIGNAL_NAMES):
    for key in TARGETS:
        for side in ['Left', 'Right']:
            onset = walk_onsets[key]
            fsr_sigs = get_fsr_signals(data[key], side)
            fsr_full = fsr_sigs[sig_key]
            grf_full = grf_aligned[key][side]
            bw = body_weight[key]

            fsr_pts = []
            grf_bw_pts = []
            for s in gait[key][side]:
                hs_abs = onset + s['hs']
                to_abs = onset + s['to']
                if to_abs >= len(fsr_full) or to_abs >= len(grf_full):
                    continue
                fsr_stance = fsr_full[hs_abs:to_abs]
                grf_stance = grf_full[hs_abs:to_abs]
                valid = grf_stance > GRF_THRESH
                fsr_pts.extend(fsr_stance[valid])
                grf_bw_pts.extend(grf_stance[valid] / bw)

            fsr_pts = np.array(fsr_pts)
            grf_bw_pts = np.array(grf_bw_pts)
            mean_fsr_bw = np.mean(fsr_pts) / bw

            if len(fsr_pts) > 10:
                sl, ic, r, p, se = stats.linregress(grf_bw_pts, fsr_pts)
                print(f"{sig_name:<12} {LABELS[key]:<28} {side:<6} {mean_fsr_bw:<14.4f} {r**2:<14.4f} {sl:<10.2f}")
            else:
                print(f"{sig_name:<12} {LABELS[key]:<28} {side:<6} {'N/A':<14} {'N/A':<14} {'N/A':<10}")

print("\nDone!")
