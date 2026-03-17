"""
GRF-Normalized FSR Comparison: before_level vs after_level
- FSR/GRF ratio (ADC/N) removes body weight effect → pure sensor sensitivity
- Gait cycle normalized profiles
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
os.makedirs(RESULT_DIR, exist_ok=True)

ROBOT_FS = 500
GRF_FS = 2000

LEFT_FSR = [f'LeftFSR{i}' for i in range(1, 15)]
RIGHT_FSR = [f'RightFSR{i}' for i in range(1, 15)]

TARGETS = ['before_level', 'after_level']
LABELS = {'before_level': 'Before (S1, 1st use)', 'after_level': 'After (S2, 3rd use)'}
COLORS = {'before_level': '#1f77b4', 'after_level': '#2ca02c'}

# ============================================================
# 1. Load robot data
# ============================================================
print("=== Loading data ===")
robot_files = {
    'before_level': 'before/level_075mps_lv0_01_processed.csv',
    'after_level':  'after/level_075mps_lv0_01_processed.csv',
}
data = {}
for key, path in robot_files.items():
    data[key] = pd.read_csv(path)
    print(f"  {key}: {data[key].shape}")

# ============================================================
# 2. Load & downsample GRF
# ============================================================
grf_files = {
    'before_level': 'before/level_075mps_lv0_01.csv',
    'after_level':  'after/level_075mps_lv0_01.csv',
}

def load_grf_csv(filepath):
    rows = []
    with open(filepath, 'r', encoding='utf-8-sig') as f:
        for i, line in enumerate(f):
            if i < 7:
                continue
            stripped = line.strip()
            if stripped.startswith('Trajectories'):
                break
            if stripped == '':
                continue
            rows.append(stripped)
    csv_text = '\n'.join(rows)
    df = pd.read_csv(io.StringIO(csv_text), header=None)
    return pd.DataFrame({'FP1_Fz': df[13], 'FP2_Fz': df[4]})

def downsample_grf(grf_fz, ratio=4):
    nyq = GRF_FS / 2
    cutoff = (ROBOT_FS / 2) * 0.9
    b, a = signal.butter(4, cutoff / nyq, btype='low')
    filtered = signal.filtfilt(b, a, grf_fz)
    return filtered[::ratio]

grf_aligned = {}
for key in TARGETS:
    grf_raw = load_grf_csv(grf_files[key])
    # FP1=Left, FP2=Right (determined from cross-correlation in main analysis)
    left_grf = np.abs(downsample_grf(grf_raw['FP1_Fz'].values))
    right_grf = np.abs(downsample_grf(grf_raw['FP2_Fz'].values))
    robot_len = len(data[key])
    left_grf = left_grf[:robot_len]
    right_grf = right_grf[:robot_len]
    if len(left_grf) < robot_len:
        left_grf = np.pad(left_grf, (0, robot_len - len(left_grf)), mode='edge')
        right_grf = np.pad(right_grf, (0, robot_len - len(right_grf)), mode='edge')
    grf_aligned[key] = {'Left': left_grf, 'Right': right_grf}
    print(f"  {key} GRF loaded, standing total: {left_grf[:1000].mean() + right_grf[:1000].mean():.0f} N")

# ============================================================
# 3. Detect walking onset (hip angle)
# ============================================================
def detect_walking_onset(df, fs=500):
    hip = (df['LeftHipAngle'] + df['RightHipAngle']) / 2
    rolling_std = hip.rolling(window=fs, center=False).std()
    initial_std = hip.iloc[:fs].std()
    threshold = max(initial_std * 5, 0.5)
    exceed = rolling_std[rolling_std > threshold].index
    if len(exceed) > 0:
        return max(0, exceed[0] - fs // 2)
    return 0

walk_onsets = {}
for key in TARGETS:
    walk_onsets[key] = detect_walking_onset(data[key])
    print(f"  {key} walking onset: {walk_onsets[key]/ROBOT_FS:.2f}s")

# ============================================================
# 4. Gait segmentation (FSR-based)
# ============================================================
def detect_strides(fsr_sum, fs=500):
    b, a = signal.butter(4, 15 / (fs / 2), btype='low')
    filt = signal.filtfilt(b, a, fsr_sum)
    threshold = np.mean(filt) * 0.15
    above = filt > threshold
    trans = np.diff(above.astype(int))
    hs = np.where(trans == 1)[0]
    to = np.where(trans == -1)[0]
    min_stance = int(0.3 * fs)
    min_swing = int(0.2 * fs)
    strides = []
    for i, h in enumerate(hs[:-1]):
        to_after = to[to > h + min_stance]
        if len(to_after) == 0:
            continue
        t = to_after[0]
        nh = hs[i + 1]
        if nh > t + min_swing:
            strides.append({'hs': h, 'to': t, 'nhs': nh})
    return strides

gait = {}
for key in TARGETS:
    gait[key] = {}
    onset = walk_onsets[key]
    for side, cols in [('Left', LEFT_FSR), ('Right', RIGHT_FSR)]:
        fsr_sum = data[key][cols].sum(axis=1).values[onset:]
        strides = detect_strides(fsr_sum)
        gait[key][side] = strides
        print(f"  {key} {side}: {len(strides)} strides")

# ============================================================
# 5. Compute GRF-normalized FSR per stride
#    FSR_norm(t) = FSR_sum(t) / GRF_Fz(t)  [ADC/N]
#    Only during stance (GRF > 50N)
# ============================================================
GRF_THRESH = 50  # N, minimum GRF to consider as valid stance

def extract_normalized_profiles(data_df, grf_fz, strides, onset, fsr_cols, n_points=100):
    """
    For each stride, compute FSR/GRF ratio profile normalized to gait cycle.
    Returns: profiles (n_strides x n_points), raw peaks, etc.
    """
    fsr_sum = data_df[fsr_cols].sum(axis=1).values
    profiles_fsr = []       # raw FSR gait cycle profiles
    profiles_grf = []       # raw GRF gait cycle profiles
    profiles_ratio = []     # FSR/GRF ratio gait cycle profiles
    peak_ratios = []        # peak FSR/GRF per stride
    mean_ratios = []        # mean FSR/GRF during stance per stride
    peak_fsrs = []
    peak_grfs = []

    for s in strides:
        hs_abs = onset + s['hs']
        to_abs = onset + s['to']
        nhs_abs = onset + s['nhs']

        if nhs_abs >= len(fsr_sum) or nhs_abs >= len(grf_fz):
            continue

        # Full gait cycle (HS to next HS)
        fsr_gc = fsr_sum[hs_abs:nhs_abs].astype(float)
        grf_gc = grf_fz[hs_abs:nhs_abs].astype(float)

        if len(fsr_gc) < 20:
            continue

        # Normalize to 0-100% gait cycle
        x_orig = np.linspace(0, 1, len(fsr_gc))
        x_norm = np.linspace(0, 1, n_points)
        fsr_interp = np.interp(x_norm, x_orig, fsr_gc)
        grf_interp = np.interp(x_norm, x_orig, grf_gc)

        # Compute ratio (only where GRF > threshold, else 0)
        ratio_interp = np.where(grf_interp > GRF_THRESH,
                                fsr_interp / grf_interp, 0.0)

        profiles_fsr.append(fsr_interp)
        profiles_grf.append(grf_interp)
        profiles_ratio.append(ratio_interp)

        # Stance phase metrics
        stance_fsr = fsr_sum[hs_abs:to_abs].astype(float)
        stance_grf = grf_fz[hs_abs:to_abs].astype(float)
        valid = stance_grf > GRF_THRESH
        if valid.sum() > 10:
            ratios = stance_fsr[valid] / stance_grf[valid]
            peak_ratios.append(np.max(ratios))
            mean_ratios.append(np.mean(ratios))
        else:
            peak_ratios.append(np.nan)
            mean_ratios.append(np.nan)

        peak_fsrs.append(stance_fsr.max())
        peak_grfs.append(stance_grf.max())

    return {
        'fsr': np.array(profiles_fsr),
        'grf': np.array(profiles_grf),
        'ratio': np.array(profiles_ratio),
        'peak_ratio': np.array(peak_ratios),
        'mean_ratio': np.array(mean_ratios),
        'peak_fsr': np.array(peak_fsrs),
        'peak_grf': np.array(peak_grfs),
    }

# Extract for both trials, both feet
results = {}
for key in TARGETS:
    results[key] = {}
    for side, cols in [('Left', LEFT_FSR), ('Right', RIGHT_FSR)]:
        results[key][side] = extract_normalized_profiles(
            data[key], grf_aligned[key][side],
            gait[key][side], walk_onsets[key], cols
        )
        n = len(results[key][side]['ratio'])
        mr = np.nanmean(results[key][side]['mean_ratio'])
        print(f"  {key} {side}: {n} valid strides, mean FSR/GRF ratio = {mr:.3f} ADC/N")

gc_pct = np.linspace(0, 100, 100)

# ============================================================
# PLOT 1: Raw FSR/GRF ratio range comparison (box + violin)
# ============================================================
print("\n=== Generating plots ===")

fig, axes = plt.subplots(1, 2, figsize=(12, 5))
for ax, side in zip(axes, ['Left', 'Right']):
    data_list = []
    labels_list = []
    for key in TARGETS:
        valid = results[key][side]['mean_ratio']
        valid = valid[~np.isnan(valid)]
        data_list.append(valid)
        labels_list.append(LABELS[key])

    parts = ax.violinplot(data_list, positions=[0, 1], showmeans=True, showmedians=True)
    for i, (pc, key) in enumerate(zip(parts['bodies'], TARGETS)):
        pc.set_facecolor(COLORS[key])
        pc.set_alpha(0.5)

    ax.boxplot(data_list, positions=[0, 1], widths=0.2, showfliers=False,
               medianprops=dict(color='black', linewidth=2))

    ax.set_xticks([0, 1])
    ax.set_xticklabels(labels_list, fontsize=9)
    ax.set_ylabel('Mean Stance FSR/GRF (ADC/N)')
    ax.set_title(f'{side} Foot')

    # Add stats
    d0, d1 = data_list
    stat, p = stats.mannwhitneyu(d0, d1, alternative='two-sided')
    ax.text(0.5, 0.95, f'Mann-Whitney p={p:.4f}', transform=ax.transAxes,
            ha='center', va='top', fontsize=9,
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

plt.suptitle('GRF-Normalized FSR: Before vs After (same task: level walking)\n'
             'FSR/GRF ratio (ADC/N) — higher = more sensitive sensor',
             fontsize=13, fontweight='bold')
plt.tight_layout()
plt.savefig(os.path.join(RESULT_DIR, 'grf_norm_01_ratio_distribution.png'), dpi=150, bbox_inches='tight')
plt.close()
print("  Saved: grf_norm_01_ratio_distribution.png")

# ============================================================
# PLOT 2: Gait-cycle normalized FSR/GRF ratio profiles
# ============================================================
fig, axes = plt.subplots(2, 1, figsize=(12, 8))

for ax, side in zip(axes, ['Left', 'Right']):
    for key in TARGETS:
        profiles = results[key][side]['ratio']
        mean_p = np.mean(profiles, axis=0)
        std_p = np.std(profiles, axis=0)

        ax.plot(gc_pct, mean_p, color=COLORS[key], linewidth=2.5,
                label=f'{LABELS[key]} (n={len(profiles)})')
        ax.fill_between(gc_pct, mean_p - std_p, mean_p + std_p,
                        color=COLORS[key], alpha=0.15)

    ax.set_xlabel('Gait Cycle (%)')
    ax.set_ylabel('FSR/GRF Ratio (ADC/N)')
    ax.set_title(f'{side} Foot')
    ax.legend(fontsize=10)
    ax.set_xlim(0, 100)

    # Mark approximate stance/swing boundary (~60%)
    ax.axvline(60, color='gray', linestyle=':', alpha=0.5)
    ax.text(30, ax.get_ylim()[1] * 0.9, 'Stance', ha='center', fontsize=9, color='gray')
    ax.text(80, ax.get_ylim()[1] * 0.9, 'Swing', ha='center', fontsize=9, color='gray')

plt.suptitle('GRF-Normalized FSR Profile over Gait Cycle\n'
             'Mean ± 1 SD. Ratio = 0 during swing (GRF < 50N)',
             fontsize=13, fontweight='bold')
plt.tight_layout()
plt.savefig(os.path.join(RESULT_DIR, 'grf_norm_02_gait_cycle_profiles.png'), dpi=150, bbox_inches='tight')
plt.close()
print("  Saved: grf_norm_02_gait_cycle_profiles.png")

# ============================================================
# PLOT 3: Gait cycle profiles — also show raw FSR and raw GRF separately
# ============================================================
fig, axes = plt.subplots(3, 2, figsize=(14, 12))

row_titles = ['Raw FSR Sum (ADC)', 'Raw GRF Fz (N)', 'FSR/GRF Ratio (ADC/N)']
profile_keys = ['fsr', 'grf', 'ratio']

for row, (pkey, ytitle) in enumerate(zip(profile_keys, row_titles)):
    for col, side in enumerate(['Left', 'Right']):
        ax = axes[row, col]
        for key in TARGETS:
            profiles = results[key][side][pkey]
            mean_p = np.mean(profiles, axis=0)
            std_p = np.std(profiles, axis=0)
            ax.plot(gc_pct, mean_p, color=COLORS[key], linewidth=2,
                    label=LABELS[key])
            ax.fill_between(gc_pct, mean_p - std_p, mean_p + std_p,
                            color=COLORS[key], alpha=0.12)
        ax.set_ylabel(ytitle)
        ax.set_title(f'{side} Foot' if row == 0 else '')
        ax.set_xlim(0, 100)
        if row == 2:
            ax.set_xlabel('Gait Cycle (%)')
        if row == 0:
            ax.legend(fontsize=9)

plt.suptitle('Gait Cycle Comparison: Before vs After (Level Walking)\n'
             'Row 1: Raw FSR, Row 2: Raw GRF, Row 3: GRF-Normalized FSR',
             fontsize=14, fontweight='bold')
plt.tight_layout()
plt.savefig(os.path.join(RESULT_DIR, 'grf_norm_03_triple_gait_cycle.png'), dpi=150, bbox_inches='tight')
plt.close()
print("  Saved: grf_norm_03_triple_gait_cycle.png")

# ============================================================
# PLOT 4: Within-trial temporal trend of FSR/GRF ratio
# ============================================================
fig, axes = plt.subplots(2, 2, figsize=(14, 8))

for col, key in enumerate(TARGETS):
    for row, side in enumerate(['Left', 'Right']):
        ax = axes[row, col]
        r = results[key][side]
        x = np.arange(len(r['mean_ratio']))
        y = r['mean_ratio']
        valid = ~np.isnan(y)

        ax.scatter(x[valid], y[valid], s=10, alpha=0.4, color=COLORS[key])

        if valid.sum() > 2:
            slope, intercept, rval, p, se = stats.linregress(x[valid], y[valid])
            ax.plot(x, intercept + slope * x, 'k-', linewidth=2,
                    label=f'slope={slope:.5f}/stride\np={p:.4f}')
            ax.legend(fontsize=8)

        ax.set_xlabel('Stride #')
        ax.set_ylabel('Mean Stance FSR/GRF (ADC/N)')
        ax.set_title(f'{LABELS[key]} — {side}', fontsize=10)

plt.suptitle('Within-Trial Temporal Trend of GRF-Normalized FSR\n'
             'Negative slope = sensitivity decreasing over time',
             fontsize=13, fontweight='bold')
plt.tight_layout()
plt.savefig(os.path.join(RESULT_DIR, 'grf_norm_04_within_trial_trend.png'), dpi=150, bbox_inches='tight')
plt.close()
print("  Saved: grf_norm_04_within_trial_trend.png")

# ============================================================
# PLOT 5: Within-trial gait cycle evolution (early / mid / late thirds)
# ============================================================
fig, axes = plt.subplots(2, 2, figsize=(14, 8))
third_colors = ['#2196F3', '#FF9800', '#F44336']
third_labels = ['Early (1st third)', 'Mid (2nd third)', 'Late (3rd third)']

for col, key in enumerate(TARGETS):
    for row, side in enumerate(['Left', 'Right']):
        ax = axes[row, col]
        profiles = results[key][side]['ratio']
        n = len(profiles)
        third = n // 3

        for ti, (start, end, clr, lbl) in enumerate(zip(
            [0, third, 2*third],
            [third, 2*third, n],
            third_colors, third_labels
        )):
            subset = profiles[start:end]
            mean_p = np.mean(subset, axis=0)
            ax.plot(gc_pct, mean_p, color=clr, linewidth=2, label=f'{lbl} (n={len(subset)})')
            ax.fill_between(gc_pct, mean_p - np.std(subset, axis=0),
                            mean_p + np.std(subset, axis=0), color=clr, alpha=0.08)

        ax.set_xlim(0, 100)
        ax.set_xlabel('Gait Cycle (%)')
        ax.set_ylabel('FSR/GRF (ADC/N)')
        ax.set_title(f'{LABELS[key]} — {side}', fontsize=10)
        ax.legend(fontsize=7)

plt.suptitle('Within-Trial Gait Cycle Evolution (GRF-Normalized FSR)\n'
             'Early→Mid→Late: does the ratio profile change over time?',
             fontsize=13, fontweight='bold')
plt.tight_layout()
plt.savefig(os.path.join(RESULT_DIR, 'grf_norm_05_within_trial_evolution.png'), dpi=150, bbox_inches='tight')
plt.close()
print("  Saved: grf_norm_05_within_trial_evolution.png")

# ============================================================
# PLOT 6: Per-sensor GRF-normalized comparison (individual FSR channels)
# ============================================================
fig, axes = plt.subplots(2, 1, figsize=(16, 8))

for ax, (side, cols) in zip(axes, [('Left', LEFT_FSR), ('Right', RIGHT_FSR)]):
    x = np.arange(14)
    width = 0.35

    for ki, key in enumerate(TARGETS):
        onset = walk_onsets[key]
        strides_list = gait[key][side]
        grf_fz = grf_aligned[key][side]

        sensor_ratios = []
        for ch in cols:
            ch_ratios = []
            for s in strides_list:
                hs_abs = onset + s['hs']
                to_abs = onset + s['to']
                if to_abs >= len(grf_fz):
                    continue
                fsr_stance = data[key][ch].values[hs_abs:to_abs].astype(float)
                grf_stance = grf_fz[hs_abs:to_abs]
                valid = grf_stance > GRF_THRESH
                if valid.sum() > 10:
                    ch_ratios.append(np.mean(fsr_stance[valid] / grf_stance[valid]))
            sensor_ratios.append(np.mean(ch_ratios) if ch_ratios else 0)

        offset = -width/2 + ki * width
        ax.bar(x + offset, sensor_ratios, width, label=LABELS[key],
               color=COLORS[key], alpha=0.8)

    ax.set_xticks(x)
    ax.set_xticklabels([f'{side[0]}FSR{i}' for i in range(1, 15)], rotation=45, ha='right')
    ax.set_ylabel('Mean Stance FSR/GRF (ADC/N)')
    ax.set_title(f'{side} Foot — Per-Sensor GRF-Normalized Comparison')
    ax.legend()

plt.suptitle('Per-Sensor GRF-Normalized FSR: Before vs After\n'
             'Shows which individual sensors changed most',
             fontsize=13, fontweight='bold')
plt.tight_layout()
plt.savefig(os.path.join(RESULT_DIR, 'grf_norm_06_per_sensor.png'), dpi=150, bbox_inches='tight')
plt.close()
print("  Saved: grf_norm_06_per_sensor.png")

# ============================================================
# PLOT 7: Summary — key metrics side by side
# ============================================================
fig, axes = plt.subplots(1, 4, figsize=(18, 5))

# 7a: Peak ratio
ax = axes[0]
for side_i, side in enumerate(['Left', 'Right']):
    for ki, key in enumerate(TARGETS):
        pr = results[key][side]['peak_ratio']
        pr = pr[~np.isnan(pr)]
        pos = side_i * 2.5 + ki * 0.8
        bp = ax.boxplot([pr], positions=[pos], widths=0.6, showfliers=False,
                        patch_artist=True)
        bp['boxes'][0].set_facecolor(COLORS[key])
        bp['boxes'][0].set_alpha(0.6)
ax.set_xticks([0.4, 2.9])
ax.set_xticklabels(['Left', 'Right'])
ax.set_ylabel('Peak FSR/GRF (ADC/N)')
ax.set_title('Peak Ratio')
ax.legend([plt.Rectangle((0,0),1,1, fc=COLORS[k], alpha=0.6) for k in TARGETS],
          [LABELS[k] for k in TARGETS], fontsize=7)

# 7b: Mean ratio
ax = axes[1]
for side_i, side in enumerate(['Left', 'Right']):
    for ki, key in enumerate(TARGETS):
        mr = results[key][side]['mean_ratio']
        mr = mr[~np.isnan(mr)]
        pos = side_i * 2.5 + ki * 0.8
        bp = ax.boxplot([mr], positions=[pos], widths=0.6, showfliers=False,
                        patch_artist=True)
        bp['boxes'][0].set_facecolor(COLORS[key])
        bp['boxes'][0].set_alpha(0.6)
ax.set_xticks([0.4, 2.9])
ax.set_xticklabels(['Left', 'Right'])
ax.set_ylabel('Mean Stance FSR/GRF (ADC/N)')
ax.set_title('Mean Ratio')

# 7c: Within-trial slope
ax = axes[2]
slopes_data = []
for side in ['Left', 'Right']:
    for key in TARGETS:
        r = results[key][side]
        y = r['mean_ratio']
        valid = ~np.isnan(y)
        x = np.arange(len(y))
        if valid.sum() > 2:
            sl, _, _, p, _ = stats.linregress(x[valid], y[valid])
        else:
            sl = 0
        slopes_data.append(sl)

x_pos = np.arange(4)
bar_colors = [COLORS['before_level'], COLORS['after_level']] * 2
ax.bar(x_pos, slopes_data, color=bar_colors, alpha=0.7)
ax.set_xticks(x_pos)
ax.set_xticklabels(['L-Before', 'L-After', 'R-Before', 'R-After'], rotation=15, fontsize=8)
ax.axhline(0, color='gray', linestyle='--', linewidth=0.5)
ax.set_ylabel('Slope (ratio/stride)')
ax.set_title('Within-Trial Trend')

# 7d: Subject weight from GRF
ax = axes[3]
weights = []
for key in TARGETS:
    onset = walk_onsets[key]
    total = grf_aligned[key]['Left'][:onset].mean() + grf_aligned[key]['Right'][:onset].mean()
    weights.append(total / 9.81)
ax.bar([0, 1], weights, color=[COLORS[k] for k in TARGETS], alpha=0.7)
ax.set_xticks([0, 1])
ax.set_xticklabels([LABELS[k] for k in TARGETS], fontsize=8)
ax.set_ylabel('Body Weight (kg)')
ax.set_title('Subject Weight\n(from standing GRF)')

plt.suptitle('GRF-Normalized FSR Summary: Before vs After (Level Walking)',
             fontsize=14, fontweight='bold')
plt.tight_layout()
plt.savefig(os.path.join(RESULT_DIR, 'grf_norm_07_summary.png'), dpi=150, bbox_inches='tight')
plt.close()
print("  Saved: grf_norm_07_summary.png")

# ============================================================
# Print summary
# ============================================================
print("\n" + "=" * 70)
print("SUMMARY: GRF-Normalized FSR Comparison (before_level vs after_level)")
print("=" * 70)
print(f"\nSubject weight: Before={weights[0]:.1f}kg, After={weights[1]:.1f}kg")
print(f"\nMean Stance FSR/GRF Ratio (ADC/N):")
for side in ['Left', 'Right']:
    for key in TARGETS:
        r = results[key][side]
        mr = r['mean_ratio']
        valid = ~np.isnan(mr)
        print(f"  {LABELS[key]:30s} {side}: {np.nanmean(mr):.4f} ± {np.nanstd(mr):.4f} (n={valid.sum()})")

print(f"\nWithin-trial slope (ratio/stride):")
for side in ['Left', 'Right']:
    for key in TARGETS:
        r = results[key][side]
        y = r['mean_ratio']
        valid = ~np.isnan(y)
        x = np.arange(len(y))
        if valid.sum() > 2:
            sl, _, _, p, _ = stats.linregress(x[valid], y[valid])
            print(f"  {LABELS[key]:30s} {side}: slope={sl:.6f}, p={p:.4f}")

print("\nDone!")
