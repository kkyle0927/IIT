import matplotlib
matplotlib.use('Agg')  # Non-interactive backend for saving

import os
RESULT_DIR = 'result'
os.makedirs(RESULT_DIR, exist_ok=True)
_fig_counter = [0]

def save_and_close(fname=None):
    _fig_counter[0] += 1
    if fname is None:
        fname = f'fig_{_fig_counter[0]:02d}'
    path = os.path.join(RESULT_DIR, f'{fname}.png')
    import matplotlib.pyplot as plt
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close('all')
    print(f'  Saved: {path}')

import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
from scipy import signal, stats
import seaborn as sns
from pathlib import Path
import warnings
warnings.filterwarnings('ignore')

plt.rcParams['figure.figsize'] = (14, 6)
plt.rcParams['figure.dpi'] = 120
sns.set_style('whitegrid')

# Constants
SAMPLING_RATE = 500  # Hz

# File paths
FILES = {
    'before_level':   'before/level_075mps_lv0_01_processed.csv',
    'before_decline': 'before/decline_5deg_lv0_01_processed.csv',
    'after_level':    'after/level_075mps_lv0_01_processed.csv',
    'after_decline':  'after/decline_5deg_lv0_01_processed.csv',
}

# Temporal order of sensor usage
TEMPORAL_ORDER = ['before_level', 'before_decline', 'after_level', 'after_decline']
TEMPORAL_LABELS = ['S1-Level (1st)', 'S1-Decline (2nd)', 'S2-Level (3rd)', 'S2-Decline (4th)']
COLORS = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728']

# FSR channel names
LEFT_FSR = [f'LeftFSR{i}' for i in range(1, 15)]
RIGHT_FSR = [f'RightFSR{i}' for i in range(1, 15)]
ALL_FSR = LEFT_FSR + RIGHT_FSR

# Hip angle columns
HIP_ANGLES = ['LeftHipAngle', 'RightHipAngle']

# Load all data
data = {}
for key, path in FILES.items():
    data[key] = pd.read_csv(path)
    print(f"{key}: {data[key].shape} ({data[key].shape[0]/SAMPLING_RATE:.1f}s)")

print(f"\nFSR columns check: {ALL_FSR[0]}, ..., {ALL_FSR[-1]}")
print(f"Hip angle columns: {HIP_ANGLES}")

# --- CELL BREAK ---

def detect_walking_onset(df, fs=500, window_sec=1.0, stride_sec=0.25, threshold_factor=5.0):
    """
    Detect walking onset using hip angle variability.
    
    Uses a sliding window std of combined L/R hip angles.
    Walking onset = first point where std exceeds threshold_factor * standing_std.
    """
    window = int(window_sec * fs)
    stride = int(stride_sec * fs)
    
    # Combined hip angle signal (average of L and R)
    hip_combined = (df['LeftHipAngle'] + df['RightHipAngle']) / 2
    
    # Compute sliding window std
    rolling_std = hip_combined.rolling(window=window, center=False).std()
    
    # Estimate standing noise from first 1 second (definitely standing)
    initial_std = hip_combined.iloc[:fs].std()
    
    # Threshold: standing noise * factor
    threshold = max(initial_std * threshold_factor, 0.5)  # minimum threshold of 0.5 deg
    
    # Find first index where rolling_std exceeds threshold
    exceed_idx = rolling_std[rolling_std > threshold].index
    if len(exceed_idx) > 0:
        onset_idx = exceed_idx[0]
        # Subtract half window to get actual onset (rolling is backward-looking)
        onset_idx = max(0, onset_idx - window // 2)
    else:
        onset_idx = 0  # fallback: assume all walking
    
    return onset_idx, rolling_std

# Detect walking onset for all trials
walk_onsets = {}
rolling_stds = {}
standing = {}
walking = {}

for cond in TEMPORAL_ORDER:
    onset, rstd = detect_walking_onset(data[cond])
    walk_onsets[cond] = onset
    rolling_stds[cond] = rstd
    standing[cond] = data[cond].iloc[:onset]
    walking[cond] = data[cond].iloc[onset:].reset_index(drop=True)
    print(f"{cond}: walking onset at sample {onset} ({onset/SAMPLING_RATE:.2f}s), "
          f"standing={onset} samples, walking={len(walking[cond])} samples")

# --- CELL BREAK ---

# Validation plot: Hip angle + detected walking onset
fig, axes = plt.subplots(2, 2, figsize=(16, 10))
axes = axes.flatten()

for i, (cond, label) in enumerate(zip(TEMPORAL_ORDER, TEMPORAL_LABELS)):
    ax = axes[i]
    t = np.arange(len(data[cond])) / SAMPLING_RATE
    
    ax.plot(t, data[cond]['LeftHipAngle'], alpha=0.7, label='Left Hip', linewidth=0.5)
    ax.plot(t, data[cond]['RightHipAngle'], alpha=0.7, label='Right Hip', linewidth=0.5)
    
    onset_t = walk_onsets[cond] / SAMPLING_RATE
    ax.axvline(x=onset_t, color='red', linestyle='--', linewidth=2, label=f'Walking onset ({onset_t:.2f}s)')
    
    ax.set_title(label)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Hip Angle (deg)')
    ax.set_xlim(0, min(15, t[-1]))  # Show first 15 seconds for detail
    ax.legend(fontsize=8)

plt.suptitle('Walking Onset Detection (Hip Angle)', fontsize=14, fontweight='bold')
plt.tight_layout()
save_and_close('fig_01_walking_onset_detection')

# --- CELL BREAK ---

# Compute noise metrics (std only) per sensor per condition during standing
noise_stats = []
for cond in TEMPORAL_ORDER:
    for ch in ALL_FSR:
        vals = standing[cond][ch]
        noise_stats.append({
            'condition': cond,
            'channel': ch,
            'side': 'Left' if ch.startswith('Left') else 'Right',
            'sensor_num': int(ch.replace('LeftFSR', '').replace('RightFSR', '')),
            'noise_std': vals.std(),
            'noise_range': vals.max() - vals.min(),
        })
noise_df = pd.DataFrame(noise_stats)

# Plot 1: Noise std grouped bar chart
fig, axes = plt.subplots(2, 1, figsize=(16, 10))

for ax, side, fsr_cols in zip(axes, ['Left', 'Right'], [LEFT_FSR, RIGHT_FSR]):
    subset = noise_df[noise_df['side'] == side]
    x = np.arange(14)
    width = 0.2
    
    for j, (cond, label, color) in enumerate(zip(TEMPORAL_ORDER, TEMPORAL_LABELS, COLORS)):
        vals = subset[subset['condition'] == cond]['noise_std'].values
        ax.bar(x + j * width, vals, width, label=label, color=color, alpha=0.8)
    
    ax.set_xlabel('Sensor Number')
    ax.set_ylabel('Noise Std (ADC counts)')
    ax.set_title(f'{side} Foot - Standing Noise Level')
    ax.set_xticks(x + 1.5 * width)
    ax.set_xticklabels([f'{side}FSR{i}' for i in range(1, 15)], rotation=45, ha='right')
    ax.legend(fontsize=8)

plt.suptitle('Baseline Noise Level (Std) per Sensor', fontsize=14, fontweight='bold')
plt.tight_layout()
save_and_close('fig_02_noise_std_bar_chart')

# --- CELL BREAK ---

# Plot 2: Noise std heatmap (conditions x sensors)
pivot_noise = noise_df.pivot_table(index='condition', columns='channel', values='noise_std')
pivot_noise = pivot_noise.reindex(TEMPORAL_ORDER)  # ensure temporal order
pivot_noise = pivot_noise[ALL_FSR]  # ensure sensor order

fig, ax = plt.subplots(figsize=(18, 4))
sns.heatmap(pivot_noise, annot=True, fmt='.2f', cmap='YlOrRd', ax=ax,
            yticklabels=TEMPORAL_LABELS, cbar_kws={'label': 'Noise Std (ADC)'})
ax.set_title('Noise Level Heatmap: Conditions × Sensors', fontsize=14, fontweight='bold')
ax.set_xlabel('FSR Channel')
ax.set_ylabel('')
plt.tight_layout()
save_and_close('fig_03_noise_heatmap')

# Statistical test: Wilcoxon signed-rank test (1st vs 4th usage)
first_noise = noise_df[noise_df['condition'] == 'before_level']['noise_std'].values
last_noise = noise_df[noise_df['condition'] == 'after_decline']['noise_std'].values
stat, p_val = stats.wilcoxon(first_noise, last_noise)
print(f"\nWilcoxon signed-rank test (1st vs 4th usage noise std):")
print(f"  Statistic = {stat:.4f}, p-value = {p_val:.6f}")
print(f"  Mean noise (1st): {first_noise.mean():.3f}, Mean noise (4th): {last_noise.mean():.3f}")
print(f"  {'Significant' if p_val < 0.05 else 'Not significant'} difference (α=0.05)")

# --- CELL BREAK ---

def detect_gait_events(fsr_sum, fs=500, min_stance_ms=300, min_swing_ms=200):
    """
    Detect heel strike (HS) and toe off (TO) from FSR sum signal.
    1. Low-pass filter at 15 Hz
    2. Adaptive threshold = mean * 0.15
    3. Rising crossing = HS, falling crossing = TO
    4. Enforce minimum stance/swing durations
    """
    b, a = signal.butter(4, 15 / (fs / 2), btype='low')
    filtered = signal.filtfilt(b, a, fsr_sum)
    
    threshold = np.mean(filtered) * 0.15
    above = filtered > threshold
    
    transitions = np.diff(above.astype(int))
    hs_candidates = np.where(transitions == 1)[0]
    to_candidates = np.where(transitions == -1)[0]
    
    min_stance = int(min_stance_ms * fs / 1000)
    min_swing = int(min_swing_ms * fs / 1000)
    
    strides = []
    for i, hs in enumerate(hs_candidates[:-1]):
        to_after = to_candidates[to_candidates > hs + min_stance]
        if len(to_after) == 0:
            continue
        to_idx = to_after[0]
        next_hs = hs_candidates[i + 1]
        if next_hs > to_idx + min_swing:
            strides.append({
                'hs_idx': hs,
                'to_idx': to_idx,
                'next_hs_idx': next_hs,
                'stance_dur': (to_idx - hs) / fs,
                'swing_dur': (next_hs - to_idx) / fs,
                'stride_dur': (next_hs - hs) / fs,
            })
    return strides, filtered

# Apply to all conditions and both feet
gait_events = {}
filtered_sums = {}

for cond in TEMPORAL_ORDER:
    gait_events[cond] = {}
    filtered_sums[cond] = {}
    for side, fsr_cols in [('Left', LEFT_FSR), ('Right', RIGHT_FSR)]:
        fsr_sum = walking[cond][fsr_cols].sum(axis=1).values
        strides, filt = detect_gait_events(fsr_sum)
        gait_events[cond][side] = strides
        filtered_sums[cond][side] = filt
        print(f"{cond} {side}: {len(strides)} strides, "
              f"mean stride duration = {np.mean([s['stride_dur'] for s in strides]):.3f}s")

# --- CELL BREAK ---

# Validation plot: 10-second window with HS/TO markers
fig, axes = plt.subplots(4, 2, figsize=(18, 16))

for i, (cond, label) in enumerate(zip(TEMPORAL_ORDER, TEMPORAL_LABELS)):
    for j, (side, fsr_cols) in enumerate([('Left', LEFT_FSR), ('Right', RIGHT_FSR)]):
        ax = axes[i, j]
        fsr_sum = walking[cond][fsr_cols].sum(axis=1).values
        filt = filtered_sums[cond][side]
        strides = gait_events[cond][side]
        
        # Show 10-second window from the middle of the trial
        mid = len(fsr_sum) // 2
        start = max(0, mid - 5 * SAMPLING_RATE)
        end = min(len(fsr_sum), mid + 5 * SAMPLING_RATE)
        t = np.arange(start, end) / SAMPLING_RATE
        
        ax.plot(t, fsr_sum[start:end], alpha=0.4, color='gray', linewidth=0.5, label='Raw')
        ax.plot(t, filt[start:end], color=COLORS[i], linewidth=1, label='Filtered')
        
        for s in strides:
            if start <= s['hs_idx'] < end:
                ax.axvline(s['hs_idx'] / SAMPLING_RATE, color='red', alpha=0.5, linewidth=0.8)
            if start <= s['to_idx'] < end:
                ax.axvline(s['to_idx'] / SAMPLING_RATE, color='blue', alpha=0.5, linewidth=0.8)
        
        ax.set_title(f'{label} - {side} Foot')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('FSR Sum')
        if i == 0 and j == 0:
            ax.plot([], [], color='red', alpha=0.5, label='HS')
            ax.plot([], [], color='blue', alpha=0.5, label='TO')
            ax.legend(fontsize=7)

plt.suptitle('Gait Event Detection Validation (10s window from mid-trial)', fontsize=14, fontweight='bold')
plt.tight_layout()
save_and_close('fig_04_gait_event_validation')

# --- CELL BREAK ---

# Extract per-stride metrics for temporal trend analysis
stride_metrics = {}

for cond in TEMPORAL_ORDER:
    stride_metrics[cond] = {}
    for side, fsr_cols in [('Left', LEFT_FSR), ('Right', RIGHT_FSR)]:
        strides = gait_events[cond][side]
        fsr_sum = walking[cond][fsr_cols].sum(axis=1).values
        
        peaks = []
        swing_baselines = []
        stride_times = []  # mid-stride time in seconds
        per_sensor_peaks = {ch: [] for ch in fsr_cols}
        
        for s in strides:
            # Peak FSR sum during stance
            stance_segment = fsr_sum[s['hs_idx']:s['to_idx']]
            peaks.append(stance_segment.max())
            
            # Swing phase baseline (mean FSR when foot is off ground)
            swing_segment = fsr_sum[s['to_idx']:s['next_hs_idx']]
            swing_baselines.append(swing_segment.mean())
            
            # Time of mid-stride
            stride_times.append((s['hs_idx'] + s['next_hs_idx']) / 2 / SAMPLING_RATE)
            
            # Per-sensor peaks during stance
            for ch in fsr_cols:
                ch_stance = walking[cond][ch].values[s['hs_idx']:s['to_idx']]
                per_sensor_peaks[ch].append(ch_stance.max())
        
        stride_metrics[cond][side] = {
            'peaks': np.array(peaks),
            'swing_baselines': np.array(swing_baselines),
            'stride_times': np.array(stride_times),
            'per_sensor_peaks': {ch: np.array(v) for ch, v in per_sensor_peaks.items()},
            'stride_numbers': np.arange(len(peaks)),
        }

print("Stride metrics extracted for all conditions and sides.")

# --- CELL BREAK ---

# Plot 1: Stride number vs Peak FSR sum (scatter + regression) — 2x4 grid
fig, axes = plt.subplots(2, 4, figsize=(20, 8), sharey='row')

for j, (side, fsr_cols) in enumerate([('Left', LEFT_FSR), ('Right', RIGHT_FSR)]):
    for i, (cond, label, color) in enumerate(zip(TEMPORAL_ORDER, TEMPORAL_LABELS, COLORS)):
        ax = axes[j, i]
        sm = stride_metrics[cond][side]
        x = sm['stride_numbers']
        y = sm['peaks']
        
        ax.scatter(x, y, s=8, alpha=0.4, color=color)
        
        # Linear regression
        if len(x) > 2:
            slope, intercept, r, p, se = stats.linregress(x, y)
            ax.plot(x, intercept + slope * x, color='black', linewidth=2,
                    label=f'slope={slope:.3f}\np={p:.4f}')
            ax.legend(fontsize=7, loc='lower left')
        
        ax.set_title(f'{label}\n{side} Foot', fontsize=9)
        ax.set_xlabel('Stride #')
        if i == 0:
            ax.set_ylabel(f'{side} Peak FSR Sum')

plt.suptitle('Within-Trial Peak FSR Sum Trend (Degradation = negative slope)',
             fontsize=14, fontweight='bold')
plt.tight_layout()
save_and_close('fig_05_peak_fsr_trend')

# --- CELL BREAK ---

# Plot 2: Stride number vs Swing Baseline (drift analysis) — 2x4 grid
fig, axes = plt.subplots(2, 4, figsize=(20, 8), sharey='row')

for j, (side, fsr_cols) in enumerate([('Left', LEFT_FSR), ('Right', RIGHT_FSR)]):
    for i, (cond, label, color) in enumerate(zip(TEMPORAL_ORDER, TEMPORAL_LABELS, COLORS)):
        ax = axes[j, i]
        sm = stride_metrics[cond][side]
        x = sm['stride_numbers']
        y = sm['swing_baselines']
        
        ax.scatter(x, y, s=8, alpha=0.4, color=color)
        
        if len(x) > 2:
            slope, intercept, r, p, se = stats.linregress(x, y)
            ax.plot(x, intercept + slope * x, color='black', linewidth=2,
                    label=f'slope={slope:.4f}\np={p:.4f}')
            ax.legend(fontsize=7, loc='upper left')
        
        ax.set_title(f'{label}\n{side} Foot', fontsize=9)
        ax.set_xlabel('Stride #')
        if i == 0:
            ax.set_ylabel(f'{side} Swing Baseline')

plt.suptitle('Within-Trial Swing Phase Baseline Drift (Drift = positive slope)',
             fontsize=14, fontweight='bold')
plt.tight_layout()
save_and_close('fig_06_swing_baseline_drift')

# --- CELL BREAK ---

# Plot 3: Per-sensor degradation slope heatmap (28 channels x 4 trials)
# Compute linear regression slope for each sensor's peak value vs stride number

sensor_slopes = []
for cond in TEMPORAL_ORDER:
    for side, fsr_cols in [('Left', LEFT_FSR), ('Right', RIGHT_FSR)]:
        sm = stride_metrics[cond][side]
        x = sm['stride_numbers']
        for ch in fsr_cols:
            y = sm['per_sensor_peaks'][ch]
            if len(x) > 2:
                slope, _, _, p, _ = stats.linregress(x, y)
            else:
                slope, p = 0, 1
            sensor_slopes.append({
                'condition': cond,
                'channel': ch,
                'slope': slope,
                'p_value': p,
                'significant': p < 0.05,
            })

slope_df = pd.DataFrame(sensor_slopes)

# Pivot for heatmap
pivot_slope = slope_df.pivot_table(index='condition', columns='channel', values='slope')
pivot_slope = pivot_slope.reindex(TEMPORAL_ORDER)[ALL_FSR]

pivot_pval = slope_df.pivot_table(index='condition', columns='channel', values='p_value')
pivot_pval = pivot_pval.reindex(TEMPORAL_ORDER)[ALL_FSR]

fig, ax = plt.subplots(figsize=(20, 5))
sns.heatmap(pivot_slope, annot=True, fmt='.3f', cmap='RdBu_r', center=0, ax=ax,
            yticklabels=TEMPORAL_LABELS, cbar_kws={'label': 'Slope (ADC/stride)'})

# Mark significant cells with asterisk
for i in range(pivot_pval.shape[0]):
    for j in range(pivot_pval.shape[1]):
        if pivot_pval.iloc[i, j] < 0.05:
            ax.text(j + 0.5, i + 0.85, '*', ha='center', va='center',
                    fontsize=12, fontweight='bold', color='black')

ax.set_title('Per-Sensor Peak Value Slope (ADC/stride) — * = p<0.05\n'
             'Red = increasing, Blue = decreasing (degradation)', fontsize=13, fontweight='bold')
ax.set_xlabel('FSR Channel')
plt.tight_layout()
save_and_close('fig_07_per_sensor_slope_heatmap')

# Summary statistics
print("\n=== Within-Trial Degradation Summary ===")
for cond, label in zip(TEMPORAL_ORDER, TEMPORAL_LABELS):
    subset = slope_df[slope_df['condition'] == cond]
    n_sig = subset['significant'].sum()
    n_neg = (subset['slope'] < 0).sum()
    mean_slope = subset['slope'].mean()
    print(f"{label}: mean slope = {mean_slope:.4f}, "
          f"{n_neg}/28 sensors with negative slope, "
          f"{n_sig}/28 significant (p<0.05)")

# --- CELL BREAK ---

def extract_stride_profiles(fsr_signal, strides, normalize_length=100):
    """Extract and time-normalize each stride's FSR profile to 0-100% gait cycle."""
    profiles = []
    for s in strides:
        segment = fsr_signal[s['hs_idx']:s['next_hs_idx']]
        if len(segment) < 10:
            continue
        normalized = np.interp(
            np.linspace(0, 1, normalize_length),
            np.linspace(0, 1, len(segment)),
            segment
        )
        profiles.append(normalized)
    return np.array(profiles)

# Extract profiles for all conditions
stride_profiles = {}
for cond in TEMPORAL_ORDER:
    stride_profiles[cond] = {}
    for side, fsr_cols in [('Left', LEFT_FSR), ('Right', RIGHT_FSR)]:
        fsr_sum = walking[cond][fsr_cols].sum(axis=1).values
        profiles = extract_stride_profiles(fsr_sum, gait_events[cond][side])
        stride_profiles[cond][side] = profiles

# Plot: Normalized gait cycle overlay (2x4 grid) — early vs late strides
fig, axes = plt.subplots(2, 4, figsize=(20, 8))
gc_pct = np.linspace(0, 100, 100)

for j, side in enumerate(['Left', 'Right']):
    for i, (cond, label, color) in enumerate(zip(TEMPORAL_ORDER, TEMPORAL_LABELS, COLORS)):
        ax = axes[j, i]
        profiles = stride_profiles[cond][side]
        n = len(profiles)
        half = n // 2
        
        # Early strides (first half) in lighter color
        for p in profiles[:half]:
            ax.plot(gc_pct, p, color=color, alpha=0.08, linewidth=0.5)
        ax.plot(gc_pct, profiles[:half].mean(axis=0), color=color, linewidth=2,
                label=f'Early (n={half})', linestyle='-')
        
        # Late strides (second half) in darker color
        for p in profiles[half:]:
            ax.plot(gc_pct, p, color='gray', alpha=0.08, linewidth=0.5)
        ax.plot(gc_pct, profiles[half:].mean(axis=0), color='black', linewidth=2,
                label=f'Late (n={n-half})', linestyle='--')
        
        ax.set_title(f'{label}\n{side} ({n} strides)', fontsize=9)
        ax.set_xlabel('Gait Cycle (%)')
        if i == 0:
            ax.set_ylabel(f'{side} FSR Sum')
        ax.legend(fontsize=7)

plt.suptitle('Normalized Gait Cycles: Early (color) vs Late (black dashed) Strides',
             fontsize=14, fontweight='bold')
plt.tight_layout()
save_and_close('fig_08_gait_cycle_overlay_early_late')

# --- CELL BREAK ---

# CV comparison: first half vs second half of trial
cv_data = []
for cond, label in zip(TEMPORAL_ORDER, TEMPORAL_LABELS):
    for side in ['Left', 'Right']:
        sm = stride_metrics[cond][side]
        peaks = sm['peaks']
        n = len(peaks)
        half = n // 2
        
        early_cv = np.std(peaks[:half]) / np.mean(peaks[:half]) * 100 if np.mean(peaks[:half]) > 0 else 0
        late_cv = np.std(peaks[half:]) / np.mean(peaks[half:]) * 100 if np.mean(peaks[half:]) > 0 else 0
        
        cv_data.append({'condition': label, 'side': side, 'half': 'Early', 'CV': early_cv})
        cv_data.append({'condition': label, 'side': side, 'half': 'Late', 'CV': late_cv})

cv_df = pd.DataFrame(cv_data)

fig, axes = plt.subplots(1, 2, figsize=(14, 5))
for ax, side in zip(axes, ['Left', 'Right']):
    subset = cv_df[cv_df['side'] == side]
    x = np.arange(len(TEMPORAL_LABELS))
    width = 0.35
    
    early = subset[subset['half'] == 'Early']['CV'].values
    late = subset[subset['half'] == 'Late']['CV'].values
    
    ax.bar(x - width/2, early, width, label='Early (1st half)', color='steelblue', alpha=0.8)
    ax.bar(x + width/2, late, width, label='Late (2nd half)', color='coral', alpha=0.8)
    
    ax.set_xlabel('Trial')
    ax.set_ylabel('CV of Peak FSR Sum (%)')
    ax.set_title(f'{side} Foot - Stride Consistency')
    ax.set_xticks(x)
    ax.set_xticklabels(TEMPORAL_LABELS, rotation=15, ha='right', fontsize=8)
    ax.legend()

plt.suptitle('Coefficient of Variation: Early vs Late Strides\n(Higher CV in late = sensor becoming inconsistent)',
             fontsize=13, fontweight='bold')
plt.tight_layout()
save_and_close('fig_09_cv_early_vs_late')

# --- CELL BREAK ---

# Compute stance-phase mean per sensor, split by early/late halves
spatial_data = []

for cond, label in zip(TEMPORAL_ORDER, TEMPORAL_LABELS):
    for side, fsr_cols in [('Left', LEFT_FSR), ('Right', RIGHT_FSR)]:
        strides = gait_events[cond][side]
        n = len(strides)
        half = n // 2
        
        for period, stride_range in [('Early', range(half)), ('Late', range(half, n))]:
            for ch in fsr_cols:
                vals = []
                for si in stride_range:
                    s = strides[si]
                    stance = walking[cond][ch].values[s['hs_idx']:s['to_idx']]
                    vals.append(stance.mean())
                spatial_data.append({
                    'condition': label,
                    'cond_key': cond,
                    'side': side,
                    'channel': ch,
                    'sensor_num': int(ch.replace('LeftFSR', '').replace('RightFSR', '')),
                    'period': period,
                    'mean_stance': np.mean(vals),
                })

spatial_df = pd.DataFrame(spatial_data)

# Plot 1: Heatmap grid (each trial's sensor distribution)
fig, axes = plt.subplots(2, 4, figsize=(20, 6))

for i, (cond, label) in enumerate(zip(TEMPORAL_ORDER, TEMPORAL_LABELS)):
    for j, (side, fsr_cols) in enumerate([('Left', LEFT_FSR), ('Right', RIGHT_FSR)]):
        ax = axes[j, i]
        subset = spatial_df[(spatial_df['cond_key'] == cond) & 
                           (spatial_df['side'] == side) & 
                           (spatial_df['period'] == 'Early')]
        vals = subset.sort_values('sensor_num')['mean_stance'].values.reshape(1, -1)
        
        sns.heatmap(vals, ax=ax, cmap='YlOrRd', annot=True, fmt='.1f',
                    xticklabels=[str(i) for i in range(1, 15)],
                    yticklabels=False, cbar=False, vmin=0)
        ax.set_title(f'{label}\n{side}', fontsize=9)
        if j == 1:
            ax.set_xlabel('Sensor #')

plt.suptitle('Sensor Spatial Distribution (Mean Stance FSR)', fontsize=14, fontweight='bold')
plt.tight_layout()
save_and_close('fig_10_spatial_distribution_heatmap')

# --- CELL BREAK ---

# Plot 2: Early vs Late per-sensor change (diverging bar chart)
fig, axes = plt.subplots(2, 4, figsize=(20, 8))

for i, (cond, cond_key, label) in enumerate(zip(TEMPORAL_LABELS, TEMPORAL_ORDER, TEMPORAL_LABELS)):
    for j, (side, fsr_cols) in enumerate([('Left', LEFT_FSR), ('Right', RIGHT_FSR)]):
        ax = axes[j, i]
        
        early = spatial_df[(spatial_df['cond_key'] == cond_key) & 
                          (spatial_df['side'] == side) & 
                          (spatial_df['period'] == 'Early')].sort_values('sensor_num')
        late = spatial_df[(spatial_df['cond_key'] == cond_key) & 
                         (spatial_df['side'] == side) & 
                         (spatial_df['period'] == 'Late')].sort_values('sensor_num')
        
        early_vals = early['mean_stance'].values
        late_vals = late['mean_stance'].values
        
        # Percent change: (late - early) / early * 100
        pct_change = np.where(early_vals > 1,
                              (late_vals - early_vals) / early_vals * 100,
                              0)
        
        colors_bar = ['#d62728' if v < 0 else '#2ca02c' for v in pct_change]
        ax.barh(range(14), pct_change, color=colors_bar, alpha=0.7)
        ax.set_yticks(range(14))
        ax.set_yticklabels([f'{side[0]}FSR{k}' for k in range(1, 15)], fontsize=7)
        ax.axvline(0, color='black', linewidth=0.5)
        ax.set_title(f'{label}\n{side}', fontsize=9)
        if j == 1:
            ax.set_xlabel('Change (%)')

plt.suptitle('Within-Trial Spatial Change: Early→Late (% change in mean stance FSR)\n'
             'Red = decreased, Green = increased', fontsize=13, fontweight='bold')
plt.tight_layout()
save_and_close('fig_11_spatial_change_early_late')

# --- CELL BREAK ---

# Compute SNR per sensor: signal (stance peak mean) / noise (standing std)
# Split by early/late halves of trial
snr_data = []

for cond, cond_label in zip(TEMPORAL_ORDER, TEMPORAL_LABELS):
    for side, fsr_cols in [('Left', LEFT_FSR), ('Right', RIGHT_FSR)]:
        strides = gait_events[cond][side]
        n = len(strides)
        half = n // 2
        
        for ch in fsr_cols:
            noise = standing[cond][ch].std()
            if noise < 0.01:
                noise = 0.01  # avoid division by zero
            
            for period, stride_range in [('Early', range(half)), ('Late', range(half, n))]:
                peaks = []
                for si in stride_range:
                    s = strides[si]
                    ch_stance = walking[cond][ch].values[s['hs_idx']:s['to_idx']]
                    peaks.append(ch_stance.max())
                
                signal_level = np.mean(peaks) if peaks else 0
                snr_linear = signal_level / noise
                snr_db = 20 * np.log10(snr_linear) if snr_linear > 0 else 0
                
                snr_data.append({
                    'condition': cond_label,
                    'cond_key': cond,
                    'channel': ch,
                    'side': side,
                    'sensor_num': int(ch.replace('LeftFSR', '').replace('RightFSR', '')),
                    'period': period,
                    'signal': signal_level,
                    'noise': noise,
                    'snr_db': snr_db,
                })

snr_df = pd.DataFrame(snr_data)

# Plot: SNR bar chart (early vs late, per trial)
fig, axes = plt.subplots(2, 2, figsize=(18, 10))

for i, (cond_key, label) in enumerate(zip(TEMPORAL_ORDER, TEMPORAL_LABELS)):
    ax = axes[i // 2, i % 2]
    subset = snr_df[snr_df['cond_key'] == cond_key]
    
    x = np.arange(28)
    width = 0.35
    
    early_snr = subset[subset['period'] == 'Early'].sort_values('channel')['snr_db'].values
    late_snr = subset[subset['period'] == 'Late'].sort_values('channel')['snr_db'].values
    
    ax.bar(x - width/2, early_snr, width, label='Early', color='steelblue', alpha=0.8)
    ax.bar(x + width/2, late_snr, width, label='Late', color='coral', alpha=0.8)
    
    ax.set_title(label)
    ax.set_xlabel('FSR Channel')
    ax.set_ylabel('SNR (dB)')
    ax.set_xticks(x)
    ax.set_xticklabels(ALL_FSR, rotation=90, fontsize=6)
    ax.legend(fontsize=8)

plt.suptitle('Signal-to-Noise Ratio: Early vs Late Halves of Trial', fontsize=14, fontweight='bold')
plt.tight_layout()
save_and_close('fig_12_snr_early_vs_late')

# Summary
print("\n=== SNR Summary (mean across all 28 sensors) ===")
for cond_key, label in zip(TEMPORAL_ORDER, TEMPORAL_LABELS):
    subset = snr_df[snr_df['cond_key'] == cond_key]
    early_mean = subset[subset['period'] == 'Early']['snr_db'].mean()
    late_mean = subset[subset['period'] == 'Late']['snr_db'].mean()
    print(f"{label}: Early={early_mean:.1f} dB, Late={late_mean:.1f} dB, "
          f"Change={late_mean - early_mean:+.1f} dB")

# --- CELL BREAK ---

# Plot 1: Full trial FSR sum with standing/walking boundary
fig, axes = plt.subplots(4, 1, figsize=(18, 14), sharex=False)

for i, (cond, label, color) in enumerate(zip(TEMPORAL_ORDER, TEMPORAL_LABELS, COLORS)):
    ax = axes[i]
    df = data[cond]
    t = np.arange(len(df)) / SAMPLING_RATE
    
    left_sum = df[LEFT_FSR].sum(axis=1).values
    right_sum = df[RIGHT_FSR].sum(axis=1).values
    
    ax.plot(t, left_sum, alpha=0.6, linewidth=0.3, color='blue', label='Left FSR Sum')
    ax.plot(t, right_sum, alpha=0.6, linewidth=0.3, color='red', label='Right FSR Sum')
    
    onset_t = walk_onsets[cond] / SAMPLING_RATE
    ax.axvline(x=onset_t, color='black', linestyle='--', linewidth=1.5,
               label=f'Walk onset ({onset_t:.1f}s)')
    ax.axvspan(0, onset_t, alpha=0.1, color='gray', label='Standing')
    
    ax.set_title(label, fontsize=11)
    ax.set_ylabel('FSR Sum')
    ax.legend(fontsize=7, loc='upper right')
    if i == 3:
        ax.set_xlabel('Time (s)')

plt.suptitle('Full Trial FSR Sum Time Series', fontsize=14, fontweight='bold')
plt.tight_layout()
save_and_close('fig_13_full_trial_fsr_sum')

# --- CELL BREAK ---

# Plot 2: Representative gait cycles — trial 초반 5개 vs 후반 5개 stride
fig, axes = plt.subplots(2, 4, figsize=(20, 8))
gc_pct = np.linspace(0, 100, 100)

for j, side in enumerate(['Left', 'Right']):
    for i, (cond, label, color) in enumerate(zip(TEMPORAL_ORDER, TEMPORAL_LABELS, COLORS)):
        ax = axes[j, i]
        profiles = stride_profiles[cond][side]
        n = len(profiles)
        
        # First 5 and last 5 strides
        n_show = min(5, n // 2)
        early_5 = profiles[:n_show]
        late_5 = profiles[-n_show:]
        
        for k, p in enumerate(early_5):
            ax.plot(gc_pct, p, color=color, alpha=0.6, linewidth=1)
        for k, p in enumerate(late_5):
            ax.plot(gc_pct, p, color='black', alpha=0.6, linewidth=1, linestyle='--')
        
        # Mean lines
        ax.plot(gc_pct, early_5.mean(axis=0), color=color, linewidth=2.5, label=f'Early {n_show}')
        ax.plot(gc_pct, late_5.mean(axis=0), color='black', linewidth=2.5, 
                linestyle='--', label=f'Late {n_show}')
        
        ax.set_title(f'{label}\n{side}', fontsize=9)
        ax.set_xlabel('Gait Cycle (%)')
        if i == 0:
            ax.set_ylabel(f'{side} FSR Sum')
        ax.legend(fontsize=7)

plt.suptitle('Representative Gait Cycles: First 5 vs Last 5 Strides',
             fontsize=14, fontweight='bold')
plt.tight_layout()
save_and_close('fig_14_representative_gait_cycles')

# --- CELL BREAK ---

# Plot 3: Individual sensor raw signal — top 4 active sensors, 10s window (early vs late)
# Find the 4 highest-amplitude sensors from before_level
ref_cond = 'before_level'
sensor_amplitudes = {}
for ch in ALL_FSR:
    sensor_amplitudes[ch] = walking[ref_cond][ch].quantile(0.95)

top_sensors = sorted(sensor_amplitudes, key=sensor_amplitudes.get, reverse=True)[:4]
print(f"Top 4 active sensors: {top_sensors}")

fig, axes = plt.subplots(4, 4, figsize=(20, 14))

for row, ch in enumerate(top_sensors):
    for col, (cond, label, color) in enumerate(zip(TEMPORAL_ORDER, TEMPORAL_LABELS, COLORS)):
        ax = axes[row, col]
        sig = walking[cond][ch].values
        n_samples = len(sig)
        
        # Early 10s window (from start of walking)
        early_end = min(10 * SAMPLING_RATE, n_samples)
        t_early = np.arange(early_end) / SAMPLING_RATE
        
        # Late 10s window (last 10 seconds)
        late_start = max(0, n_samples - 10 * SAMPLING_RATE)
        t_late = np.arange(late_start, n_samples) / SAMPLING_RATE
        
        ax.plot(t_early, sig[:early_end], color=color, alpha=0.7, linewidth=0.5, label='Early 10s')
        ax.plot(t_late, sig[late_start:], color='black', alpha=0.7, linewidth=0.5, label='Late 10s')
        
        ax.set_title(f'{label}\n{ch}', fontsize=8)
        if row == 3:
            ax.set_xlabel('Time (s)')
        if col == 0:
            ax.set_ylabel('FSR (ADC)')
        if row == 0 and col == 0:
            ax.legend(fontsize=6)

plt.suptitle('Individual Sensor Raw Signal: Early vs Late 10s Windows',
             fontsize=14, fontweight='bold')
plt.tight_layout()
save_and_close('fig_15_individual_sensor_raw')

# --- CELL BREAK ---

# Summary Table
summary_rows = []

for cond, label in zip(TEMPORAL_ORDER, TEMPORAL_LABELS):
    row = {'Trial': label}
    
    # 1. Mean noise std (standing)
    cond_noise = noise_df[noise_df['condition'] == cond]['noise_std']
    row['Noise Std (mean)'] = f"{cond_noise.mean():.2f}"
    
    # 2. Mean peak FSR sum (L/R)
    for side in ['Left', 'Right']:
        sm = stride_metrics[cond][side]
        row[f'Peak FSR Sum ({side[0]})'] = f"{sm['peaks'].mean():.1f}"
    
    # 3. Peak slope (within-trial degradation rate)
    for side in ['Left', 'Right']:
        sm = stride_metrics[cond][side]
        x = sm['stride_numbers']
        y = sm['peaks']
        if len(x) > 2:
            slope, _, _, p, _ = stats.linregress(x, y)
            sig = '*' if p < 0.05 else ''
            row[f'Peak Slope ({side[0]})'] = f"{slope:.3f}{sig}"
        else:
            row[f'Peak Slope ({side[0]})'] = 'N/A'
    
    # 4. Baseline drift slope
    for side in ['Left', 'Right']:
        sm = stride_metrics[cond][side]
        x = sm['stride_numbers']
        y = sm['swing_baselines']
        if len(x) > 2:
            slope, _, _, p, _ = stats.linregress(x, y)
            sig = '*' if p < 0.05 else ''
            row[f'Drift Slope ({side[0]})'] = f"{slope:.4f}{sig}"
        else:
            row[f'Drift Slope ({side[0]})'] = 'N/A'
    
    # 5. CV early vs late
    for side in ['Left', 'Right']:
        sm = stride_metrics[cond][side]
        peaks = sm['peaks']
        n = len(peaks)
        half = n // 2
        early_cv = np.std(peaks[:half]) / np.mean(peaks[:half]) * 100
        late_cv = np.std(peaks[half:]) / np.mean(peaks[half:]) * 100
        row[f'CV Early/Late ({side[0]})'] = f"{early_cv:.1f}% / {late_cv:.1f}%"
    
    # 6. Number of strides
    for side in ['Left', 'Right']:
        row[f'Strides ({side[0]})'] = len(gait_events[cond][side])
    
    summary_rows.append(row)

summary_table = pd.DataFrame(summary_rows)
summary_table = summary_table.set_index('Trial')
print("=== FSR Degradation Summary Table ===")
print("(* = p<0.05 for slope significance)")
print()
print(summary_table.T)

# --- CELL BREAK ---

# Summary Dashboard: 6 key metrics across 4 trials
fig, axes = plt.subplots(2, 3, figsize=(18, 10))

x_pos = np.arange(4)

# 1. Noise Std
ax = axes[0, 0]
for side, marker in [('Left', 'o'), ('Right', 's')]:
    vals = []
    for cond in TEMPORAL_ORDER:
        side_noise = noise_df[(noise_df['condition'] == cond) & 
                             (noise_df['side'] == side)]['noise_std'].mean()
        vals.append(side_noise)
    ax.plot(x_pos, vals, marker=marker, linewidth=2, label=f'{side}')
ax.set_title('Noise Level (Standing Std)')
ax.set_ylabel('Std (ADC)')
ax.set_xticks(x_pos)
ax.set_xticklabels(TEMPORAL_LABELS, rotation=20, ha='right', fontsize=7)
ax.legend()

# 2. Mean Peak FSR Sum
ax = axes[0, 1]
for side, marker in [('Left', 'o'), ('Right', 's')]:
    vals = [stride_metrics[c][side]['peaks'].mean() for c in TEMPORAL_ORDER]
    ax.plot(x_pos, vals, marker=marker, linewidth=2, label=f'{side}')
ax.set_title('Mean Peak FSR Sum')
ax.set_ylabel('Peak (ADC)')
ax.set_xticks(x_pos)
ax.set_xticklabels(TEMPORAL_LABELS, rotation=20, ha='right', fontsize=7)
ax.legend()

# 3. Within-trial Peak Slope
ax = axes[0, 2]
for side, marker in [('Left', 'o'), ('Right', 's')]:
    vals = []
    for cond in TEMPORAL_ORDER:
        sm = stride_metrics[cond][side]
        slope, _, _, _, _ = stats.linregress(sm['stride_numbers'], sm['peaks'])
        vals.append(slope)
    ax.plot(x_pos, vals, marker=marker, linewidth=2, label=f'{side}')
ax.axhline(0, color='gray', linestyle='--', linewidth=0.5)
ax.set_title('Within-Trial Peak Slope')
ax.set_ylabel('Slope (ADC/stride)')
ax.set_xticks(x_pos)
ax.set_xticklabels(TEMPORAL_LABELS, rotation=20, ha='right', fontsize=7)
ax.legend()

# 4. Baseline Drift Slope
ax = axes[1, 0]
for side, marker in [('Left', 'o'), ('Right', 's')]:
    vals = []
    for cond in TEMPORAL_ORDER:
        sm = stride_metrics[cond][side]
        slope, _, _, _, _ = stats.linregress(sm['stride_numbers'], sm['swing_baselines'])
        vals.append(slope)
    ax.plot(x_pos, vals, marker=marker, linewidth=2, label=f'{side}')
ax.axhline(0, color='gray', linestyle='--', linewidth=0.5)
ax.set_title('Swing Baseline Drift Slope')
ax.set_ylabel('Slope (ADC/stride)')
ax.set_xticks(x_pos)
ax.set_xticklabels(TEMPORAL_LABELS, rotation=20, ha='right', fontsize=7)
ax.legend()

# 5. CV (Late half)
ax = axes[1, 1]
for side, marker in [('Left', 'o'), ('Right', 's')]:
    vals = []
    for cond in TEMPORAL_ORDER:
        sm = stride_metrics[cond][side]
        peaks = sm['peaks']
        half = len(peaks) // 2
        late_cv = np.std(peaks[half:]) / np.mean(peaks[half:]) * 100
        vals.append(late_cv)
    ax.plot(x_pos, vals, marker=marker, linewidth=2, label=f'{side}')
ax.set_title('CV of Peak (Late Half)')
ax.set_ylabel('CV (%)')
ax.set_xticks(x_pos)
ax.set_xticklabels(TEMPORAL_LABELS, rotation=20, ha='right', fontsize=7)
ax.legend()

# 6. Mean SNR (dB)
ax = axes[1, 2]
for side, marker in [('Left', 'o'), ('Right', 's')]:
    vals = []
    for cond_key in TEMPORAL_ORDER:
        subset = snr_df[(snr_df['cond_key'] == cond_key) & (snr_df['side'] == side)]
        vals.append(subset['snr_db'].mean())
    ax.plot(x_pos, vals, marker=marker, linewidth=2, label=f'{side}')
ax.set_title('Mean SNR')
ax.set_ylabel('SNR (dB)')
ax.set_xticks(x_pos)
ax.set_xticklabels(TEMPORAL_LABELS, rotation=20, ha='right', fontsize=7)
ax.legend()

plt.suptitle('FSR Sensor Degradation Summary Dashboard\n'
             '(Degradation indicators: Noise↑, Peak↓, Peak Slope<0, Drift Slope>0, CV↑, SNR↓)',
             fontsize=14, fontweight='bold')
plt.tight_layout()
save_and_close('fig_16_summary_dashboard')

print("\n=== Interpretation Guide ===")
print("Within each trial (cleanest comparison - same subject, same task, same speed):")
print("  - Peak Slope < 0: sensitivity decreases over time → degradation")
print("  - Drift Slope > 0: baseline rises over time → sensor drift")
print("  - Late CV > Early CV: stride consistency worsens → sensor becoming unreliable")
print("\nAcross trials (confounded by subject/task differences):")
print("  - Noise Std increasing: sensor getting noisier")
print("  - SNR decreasing: signal quality deteriorating")

# --- CELL BREAK ---

GRF_FS = 2000  # GRF sampling rate
ROBOT_FS = 500  # Robot sampling rate
DOWNSAMPLE_RATIO = GRF_FS // ROBOT_FS  # 4

GRF_FILES = {
    'before_level':   'before/level_075mps_lv0_01.csv',
    'before_decline': 'before/decline_5deg_lv0_01.csv',
    'after_level':    'after/level_075mps_lv0_01.csv',
    'after_decline':  'after/decline_5deg_lv0_01.csv',
}

def load_grf_csv(filepath):
    """
    Parse Vicon Devices CSV: skip header rows (lines 1-7), read until 'Trajectories' line.
    Extract Fz for force plate 1 and 2.
    Column layout: Frame, SubFrame, [FP2: Fx,Fy,Fz,Mx,My,Mz,Cx,Cy,Cz], [FP1: Fx,Fy,Fz,Mx,My,Mz,Cx,Cy,Cz]
    """
    rows = []
    with open(filepath, 'r', encoding='utf-8-sig') as f:
        for i, line in enumerate(f):
            if i < 7:  # skip first 7 lines (header)
                continue
            stripped = line.strip()
            if stripped.startswith('Trajectories'):
                break
            if stripped == '':
                continue
            rows.append(stripped)
    
    # Parse CSV rows
    import io
    csv_text = '\n'.join(rows)
    df = pd.read_csv(io.StringIO(csv_text), header=None)
    
    # Column indices: 0=Frame, 1=SubFrame, 2=FP2_Fx, 3=FP2_Fy, 4=FP2_Fz, ..., 11=FP1_Fx, 12=FP1_Fy, 13=FP1_Fz
    grf_df = pd.DataFrame({
        'Frame': df[0],
        'SubFrame': df[1],
        'FP1_Fz': df[13],   # Force plate 1 vertical
        'FP2_Fz': df[4],    # Force plate 2 vertical
    })
    
    return grf_df

# Load all GRF data
grf_data = {}
for key, path in GRF_FILES.items():
    grf_data[key] = load_grf_csv(path)
    n_rows = len(grf_data[key])
    duration = n_rows / GRF_FS
    print(f"{key}: {n_rows} rows ({duration:.1f}s at {GRF_FS}Hz), "
          f"Robot: {len(data[key])} rows ({len(data[key])/ROBOT_FS:.1f}s)")

# Quick check: Fz values during standing (first 2 seconds)
print("\n--- Standing GRF Fz (first 2s mean) ---")
for key in TEMPORAL_ORDER:
    fp1_stand = grf_data[key]['FP1_Fz'].iloc[:2*GRF_FS].mean()
    fp2_stand = grf_data[key]['FP2_Fz'].iloc[:2*GRF_FS].mean()
    print(f"{key}: FP1_Fz={fp1_stand:.1f}N, FP2_Fz={fp2_stand:.1f}N, "
          f"Total={fp1_stand+fp2_stand:.1f}N")

# --- CELL BREAK ---

# Downsample GRF from 2000Hz to 500Hz using anti-aliasing filter + decimation
def downsample_grf(grf_fz, ratio=4):
    """Downsample GRF signal with anti-alias filter."""
    # Anti-alias low-pass filter at Nyquist/2 of target rate
    nyq = GRF_FS / 2
    cutoff = (ROBOT_FS / 2) * 0.9  # slightly below Nyquist of target
    b, a = signal.butter(4, cutoff / nyq, btype='low')
    filtered = signal.filtfilt(b, a, grf_fz)
    # Decimate
    return filtered[::ratio]

# Downsample and store
grf_500hz = {}
for key in TEMPORAL_ORDER:
    fp1_ds = downsample_grf(grf_data[key]['FP1_Fz'].values)
    fp2_ds = downsample_grf(grf_data[key]['FP2_Fz'].values)
    
    # Make Fz positive (convention: Vicon reports negative for downward force)
    fp1_ds = np.abs(fp1_ds)
    fp2_ds = np.abs(fp2_ds)
    
    # Trim to match robot data length
    robot_len = len(data[key])
    fp1_ds = fp1_ds[:robot_len]
    fp2_ds = fp2_ds[:robot_len]
    
    # Pad if shorter (unlikely but safe)
    if len(fp1_ds) < robot_len:
        fp1_ds = np.pad(fp1_ds, (0, robot_len - len(fp1_ds)), mode='edge')
        fp2_ds = np.pad(fp2_ds, (0, robot_len - len(fp2_ds)), mode='edge')
    
    grf_500hz[key] = {'FP1_Fz': fp1_ds, 'FP2_Fz': fp2_ds}
    print(f"{key}: GRF downsampled to {len(fp1_ds)} samples (robot={robot_len})")

# --- CELL BREAK ---

# Cross-correlation for synchronization and L/R force plate matching
def find_lag_and_corr(sig_a, sig_b, fs=500, max_lag_sec=2.0):
    """
    Compute cross-correlation between two signals and find the lag that maximizes it.
    Uses scipy.signal.correlate for efficiency.
    Returns: (best_lag_samples, max_correlation)
    Positive lag means sig_b is delayed relative to sig_a.
    """
    max_lag = int(max_lag_sec * fs)

    # Use middle 30 seconds to avoid transients
    mid = len(sig_a) // 2
    win = min(15 * fs, mid)
    a = sig_a[mid - win:mid + win].copy()
    b = sig_b[mid - win:mid + win].copy()

    # Normalize
    a = (a - np.mean(a)) / (np.std(a) + 1e-10)
    b = (b - np.mean(b)) / (np.std(b) + 1e-10)

    # Full cross-correlation
    corr = signal.correlate(a, b, mode='full')
    corr = corr / len(a)  # normalize

    # Center index corresponds to lag=0
    center = len(a) - 1
    lag_range = slice(center - max_lag, center + max_lag + 1)
    corr_window = corr[lag_range]
    lags = np.arange(-max_lag, max_lag + 1)

    best_idx = np.argmax(corr_window)
    best_lag = lags[best_idx]
    best_corr = corr_window[best_idx]

    return best_lag, best_corr

# Run cross-correlation for all trials
fp_assignments = {}

for key in TEMPORAL_ORDER:
    fsr_left = data[key][LEFT_FSR].sum(axis=1).values.astype(float)
    fsr_right = data[key][RIGHT_FSR].sum(axis=1).values.astype(float)
    fp1 = grf_500hz[key]['FP1_Fz']
    fp2 = grf_500hz[key]['FP2_Fz']

    # Test all 4 combinations
    lag_fp1_l, corr_fp1_l = find_lag_and_corr(fsr_left, fp1)
    lag_fp1_r, corr_fp1_r = find_lag_and_corr(fsr_right, fp1)
    lag_fp2_l, corr_fp2_l = find_lag_and_corr(fsr_left, fp2)
    lag_fp2_r, corr_fp2_r = find_lag_and_corr(fsr_right, fp2)

    # Option A: FP1=Left, FP2=Right
    corr_A = corr_fp1_l + corr_fp2_r
    # Option B: FP1=Right, FP2=Left
    corr_B = corr_fp1_r + corr_fp2_l

    if corr_A >= corr_B:
        fp_assignments[key] = {
            'Left': 'FP1', 'Right': 'FP2',
            'left_lag': lag_fp1_l, 'right_lag': lag_fp2_r,
            'left_corr': corr_fp1_l, 'right_corr': corr_fp2_r,
            'total_corr': corr_A
        }
    else:
        fp_assignments[key] = {
            'Left': 'FP2', 'Right': 'FP1',
            'left_lag': lag_fp2_l, 'right_lag': lag_fp1_r,
            'left_corr': corr_fp2_l, 'right_corr': corr_fp1_r,
            'total_corr': corr_B
        }

    a = fp_assignments[key]
    print(f"{key}:")
    print(f"  Left={a['Left']} (lag={a['left_lag']} samples = {a['left_lag']/ROBOT_FS*1000:.1f}ms, r={a['left_corr']:.3f})")
    print(f"  Right={a['Right']} (lag={a['right_lag']} samples = {a['right_lag']/ROBOT_FS*1000:.1f}ms, r={a['right_corr']:.3f})")

# Consistency check
left_fps = [fp_assignments[k]['Left'] for k in TEMPORAL_ORDER]
print(f"\nForce plate assignment consistency: {left_fps}")
if len(set(left_fps)) == 1:
    print(f"Consistent: Left foot always on {left_fps[0]}")
else:
    print("WARNING: Inconsistent assignment - will use per-trial assignment")

# --- CELL BREAK ---

# Create aligned GRF signals (matched to robot data, lag-corrected, L/R assigned)
grf_aligned = {}

for key in TEMPORAL_ORDER:
    a = fp_assignments[key]
    left_fp_key = f"{a['Left']}_Fz"
    right_fp_key = f"{a['Right']}_Fz"
    
    left_grf = grf_500hz[key][left_fp_key].copy()
    right_grf = grf_500hz[key][right_fp_key].copy()
    
    # Apply lag correction by shifting GRF
    def shift_signal(sig, lag):
        """Shift signal by lag samples. Positive lag = GRF is delayed -> shift left."""
        if lag > 0:
            return np.concatenate([sig[lag:], np.full(lag, sig[-1])])
        elif lag < 0:
            return np.concatenate([np.full(-lag, sig[0]), sig[:lag]])
        return sig
    
    left_grf = shift_signal(left_grf, a['left_lag'])
    right_grf = shift_signal(right_grf, a['right_lag'])
    
    # Trim to robot data length
    robot_len = len(data[key])
    left_grf = left_grf[:robot_len]
    right_grf = right_grf[:robot_len]
    
    grf_aligned[key] = {'Left_Fz': left_grf, 'Right_Fz': right_grf}

# Validation: overlay FSR sum and GRF for 10-second window
fig, axes = plt.subplots(4, 2, figsize=(18, 14))

for i, (key, label, color) in enumerate(zip(TEMPORAL_ORDER, TEMPORAL_LABELS, COLORS)):
    for j, side in enumerate(['Left', 'Right']):
        ax = axes[i, j]
        fsr_cols = LEFT_FSR if side == 'Left' else RIGHT_FSR
        fsr_sum = data[key][fsr_cols].sum(axis=1).values
        grf_fz = grf_aligned[key][f'{side}_Fz']
        
        # 10-second window from middle
        mid = len(fsr_sum) // 2
        start = mid - 5 * ROBOT_FS
        end = mid + 5 * ROBOT_FS
        t = np.arange(start, end) / ROBOT_FS
        
        ax_grf = ax.twinx()
        ax.plot(t, fsr_sum[start:end], color=color, alpha=0.7, linewidth=0.8, label='FSR Sum')
        ax_grf.plot(t, grf_fz[start:end], color='black', alpha=0.6, linewidth=0.8, label='GRF Fz')
        
        ax.set_title(f'{label} - {side}', fontsize=9)
        ax.set_ylabel('FSR Sum (ADC)', color=color)
        ax_grf.set_ylabel('GRF Fz (N)', color='black')
        if i == 3:
            ax.set_xlabel('Time (s)')
        if i == 0 and j == 0:
            ax.legend(loc='upper left', fontsize=7)
            ax_grf.legend(loc='upper right', fontsize=7)

plt.suptitle('GRF-FSR Synchronization Validation (10s mid-trial window)',
             fontsize=14, fontweight='bold')
plt.tight_layout()
save_and_close('fig_17_grf_fsr_sync_validation')

# --- CELL BREAK ---

# Compute FSR-GRF calibration (linear regression) per trial per foot
# During walking portion only, using stance phases

calib_results = {}

fig, axes = plt.subplots(2, 4, figsize=(20, 8))

for i, (key, label, color) in enumerate(zip(TEMPORAL_ORDER, TEMPORAL_LABELS, COLORS)):
    calib_results[key] = {}
    onset = walk_onsets[key]
    
    for j, side in enumerate(['Left', 'Right']):
        ax = axes[j, i]
        fsr_cols = LEFT_FSR if side == 'Left' else RIGHT_FSR
        
        # Get walking portion
        fsr_sum_walk = data[key][fsr_cols].sum(axis=1).values[onset:]
        grf_fz_walk = grf_aligned[key][f'{side}_Fz'][onset:]
        
        # Use stance phases only (where GRF > 50N threshold)
        stance_mask = grf_fz_walk > 50
        fsr_stance = fsr_sum_walk[stance_mask]
        grf_stance = grf_fz_walk[stance_mask]
        
        # Linear regression: FSR = slope * GRF + intercept
        slope, intercept, r, p, se = stats.linregress(grf_stance, fsr_stance)
        
        calib_results[key][side] = {
            'slope': slope,        # ADC/N — FSR sensitivity
            'intercept': intercept,
            'r_squared': r**2,
            'p_value': p,
            'se': se,
        }
        
        # Scatter plot with regression line
        # Subsample for plotting (too many points)
        n_plot = min(5000, len(grf_stance))
        idx = np.random.choice(len(grf_stance), n_plot, replace=False)
        ax.scatter(grf_stance[idx], fsr_stance[idx], s=1, alpha=0.1, color=color)
        
        grf_range = np.array([grf_stance.min(), grf_stance.max()])
        ax.plot(grf_range, slope * grf_range + intercept, 'k-', linewidth=2,
                label=f'y={slope:.3f}x+{intercept:.1f}\nR²={r**2:.3f}')
        
        ax.set_title(f'{label}\n{side}', fontsize=9)
        ax.set_xlabel('GRF Fz (N)')
        if i == 0:
            ax.set_ylabel(f'{side} FSR Sum (ADC)')
        ax.legend(fontsize=7)

plt.suptitle('FSR vs GRF Calibration (Stance Phase Only)\nSlope = FSR sensitivity (ADC/N)',
             fontsize=14, fontweight='bold')
plt.tight_layout()
save_and_close('fig_18_fsr_grf_calibration')

# Summary table
print("\n=== FSR-GRF Calibration Summary ===")
print(f"{'Trial':<20} {'Side':<6} {'Slope (ADC/N)':<14} {'Intercept':<10} {'R²':<8}")
print("-" * 60)
for key, label in zip(TEMPORAL_ORDER, TEMPORAL_LABELS):
    for side in ['Left', 'Right']:
        c = calib_results[key][side]
        print(f"{label:<20} {side:<6} {c['slope']:.4f}       {c['intercept']:.2f}     {c['r_squared']:.4f}")

# --- CELL BREAK ---

# Within-trial calibration drift: split each trial into 4 quarters and compute slope per quarter
fig, axes = plt.subplots(2, 4, figsize=(18, 8))

quarter_calib = {}
for i, (key, label, color) in enumerate(zip(TEMPORAL_ORDER, TEMPORAL_LABELS, COLORS)):
    quarter_calib[key] = {}
    onset = walk_onsets[key]
    
    for j, side in enumerate(['Left', 'Right']):
        ax = axes[j, i]
        fsr_cols = LEFT_FSR if side == 'Left' else RIGHT_FSR
        fsr_sum_walk = data[key][fsr_cols].sum(axis=1).values[onset:]
        grf_fz_walk = grf_aligned[key][f'{side}_Fz'][onset:]
        
        n = len(fsr_sum_walk)
        quarter_size = n // 4
        slopes = []
        quarter_labels = ['Q1', 'Q2', 'Q3', 'Q4']
        
        for q in range(4):
            start_q = q * quarter_size
            end_q = (q + 1) * quarter_size if q < 3 else n
            
            fsr_q = fsr_sum_walk[start_q:end_q]
            grf_q = grf_fz_walk[start_q:end_q]
            
            mask = grf_q > 50
            if mask.sum() > 100:
                sl, _, r, _, _ = stats.linregress(grf_q[mask], fsr_q[mask])
                slopes.append(sl)
            else:
                slopes.append(np.nan)
        
        quarter_calib[key][side] = slopes
        
        ax.bar(range(4), slopes, color=[color]*4, alpha=0.8)
        ax.set_xticks(range(4))
        ax.set_xticklabels(quarter_labels)
        ax.set_title(f'{label}\n{side}', fontsize=9)
        if i == 0:
            ax.set_ylabel('Slope (ADC/N)')
        
        # Add trend line
        valid = [(x, y) for x, y in enumerate(slopes) if not np.isnan(y)]
        if len(valid) >= 2:
            xs, ys = zip(*valid)
            sl_trend, _, _, _, _ = stats.linregress(xs, ys)
            ax.plot([0, 3], [ys[0], ys[0] + sl_trend * 3], 'k--', linewidth=1.5,
                    label=f'trend: {sl_trend:.4f}/Q')
            ax.legend(fontsize=7)

plt.suptitle('Within-Trial FSR Sensitivity Drift (Slope ADC/N per Quarter)\n'
             'Decreasing slope = sensitivity loss over time',
             fontsize=13, fontweight='bold')
plt.tight_layout()
save_and_close('fig_19_sensitivity_drift_quarters')

# --- CELL BREAK ---

def compute_rise_time(sig, fs=500, low_pct=0.1, high_pct=0.9):
    """
    Compute rise time from low_pct to high_pct of peak value.
    Looks at the first rising edge (heel strike loading response).
    Returns rise time in milliseconds, or NaN if can't compute.
    """
    peak = np.max(sig)
    baseline = np.min(sig[:len(sig)//4])  # baseline from first quarter
    amplitude = peak - baseline
    
    if amplitude < 10:  # too small signal
        return np.nan
    
    low_thresh = baseline + low_pct * amplitude
    high_thresh = baseline + high_pct * amplitude
    
    # Find first crossing of low threshold
    above_low = np.where(sig > low_thresh)[0]
    if len(above_low) == 0:
        return np.nan
    t_low = above_low[0]
    
    # Find first crossing of high threshold after t_low
    above_high = np.where(sig[t_low:] > high_thresh)[0]
    if len(above_high) == 0:
        return np.nan
    t_high = t_low + above_high[0]
    
    rise_samples = t_high - t_low
    return rise_samples / fs * 1000  # ms

# Compute rise times for each stride using both FSR and GRF
rise_time_data = []

for key, label in zip(TEMPORAL_ORDER, TEMPORAL_LABELS):
    onset = walk_onsets[key]
    
    for side in ['Left', 'Right']:
        fsr_cols = LEFT_FSR if side == 'Left' else RIGHT_FSR
        strides = gait_events[key][side]
        
        fsr_sum_full = data[key][fsr_cols].sum(axis=1).values
        grf_fz_full = grf_aligned[key][f'{side}_Fz']
        
        for si, s in enumerate(strides):
            # Stance phase segment (HS to TO)
            hs_abs = onset + s['hs_idx']
            to_abs = onset + s['to_idx']
            
            if hs_abs >= len(fsr_sum_full) or to_abs >= len(grf_fz_full):
                continue
            
            fsr_stance = fsr_sum_full[hs_abs:to_abs]
            grf_stance = grf_fz_full[hs_abs:to_abs]
            
            if len(fsr_stance) < 20:
                continue
            
            fsr_rt = compute_rise_time(fsr_stance)
            grf_rt = compute_rise_time(grf_stance)
            
            rise_time_data.append({
                'condition': label,
                'cond_key': key,
                'side': side,
                'stride_num': si,
                'fsr_rise_time_ms': fsr_rt,
                'grf_rise_time_ms': grf_rt,
                'rt_diff_ms': fsr_rt - grf_rt if not (np.isnan(fsr_rt) or np.isnan(grf_rt)) else np.nan,
            })

rt_df = pd.DataFrame(rise_time_data)
rt_df_clean = rt_df.dropna(subset=['fsr_rise_time_ms', 'grf_rise_time_ms'])

print(f"Rise time computed for {len(rt_df_clean)}/{len(rt_df)} strides (rest had invalid signals)")
print(f"\n=== Mean Rise Times ===")
for key, label in zip(TEMPORAL_ORDER, TEMPORAL_LABELS):
    subset = rt_df_clean[rt_df_clean['cond_key'] == key]
    print(f"{label}:")
    print(f"  FSR: {subset['fsr_rise_time_ms'].mean():.1f} ± {subset['fsr_rise_time_ms'].std():.1f} ms")
    print(f"  GRF: {subset['grf_rise_time_ms'].mean():.1f} ± {subset['grf_rise_time_ms'].std():.1f} ms")
    print(f"  Diff (FSR-GRF): {subset['rt_diff_ms'].mean():.1f} ± {subset['rt_diff_ms'].std():.1f} ms")

# --- CELL BREAK ---

# Plot 1: Rise time comparison — FSR vs GRF per trial
fig, axes = plt.subplots(1, 4, figsize=(18, 5))

for i, (key, label, color) in enumerate(zip(TEMPORAL_ORDER, TEMPORAL_LABELS, COLORS)):
    ax = axes[i]
    subset = rt_df_clean[rt_df_clean['cond_key'] == key]
    
    bp_data = [subset['grf_rise_time_ms'].values, subset['fsr_rise_time_ms'].values]
    bp = ax.boxplot(bp_data, labels=['GRF', 'FSR'], widths=0.5,
                    patch_artist=True, showfliers=False)
    bp['boxes'][0].set_facecolor('lightblue')
    bp['boxes'][1].set_facecolor(color)
    bp['boxes'][1].set_alpha(0.6)
    
    ax.set_title(label, fontsize=10)
    ax.set_ylabel('Rise Time (ms)')

plt.suptitle('Rise Time Comparison: GRF vs FSR (10%→90% peak)',
             fontsize=14, fontweight='bold')
plt.tight_layout()
save_and_close('fig_20_rise_time_comparison')

# Plot 2: Within-trial rise time trend (stride number vs rise time diff)
fig, axes = plt.subplots(2, 4, figsize=(20, 8))

for j, side in enumerate(['Left', 'Right']):
    for i, (key, label, color) in enumerate(zip(TEMPORAL_ORDER, TEMPORAL_LABELS, COLORS)):
        ax = axes[j, i]
        subset = rt_df_clean[(rt_df_clean['cond_key'] == key) & (rt_df_clean['side'] == side)]
        
        x = subset['stride_num'].values
        y = subset['rt_diff_ms'].values
        
        ax.scatter(x, y, s=8, alpha=0.4, color=color)
        
        if len(x) > 2:
            slope, intercept, r, p, _ = stats.linregress(x, y)
            ax.plot(x, intercept + slope * x, 'k-', linewidth=2,
                    label=f'slope={slope:.3f}\np={p:.4f}')
            ax.legend(fontsize=7)
        
        ax.axhline(0, color='gray', linestyle='--', linewidth=0.5)
        ax.set_title(f'{label}\n{side}', fontsize=9)
        ax.set_xlabel('Stride #')
        if i == 0:
            ax.set_ylabel('Rise Time Diff\n(FSR - GRF, ms)')

plt.suptitle('Within-Trial Rise Time Difference Trend\n'
             'Positive = FSR slower than GRF. Increasing slope = FSR getting slower over time.',
             fontsize=13, fontweight='bold')
plt.tight_layout()
save_and_close('fig_21_rise_time_trend')

# --- CELL BREAK ---

# Plot 3: Normalized gait cycle overlay — FSR vs GRF (representative strides)
fig, axes = plt.subplots(2, 4, figsize=(20, 8))
gc_pct = np.linspace(0, 100, 100)

for j, side in enumerate(['Left', 'Right']):
    for i, (key, label, color) in enumerate(zip(TEMPORAL_ORDER, TEMPORAL_LABELS, COLORS)):
        ax = axes[j, i]
        fsr_cols = LEFT_FSR if side == 'Left' else RIGHT_FSR
        onset = walk_onsets[key]
        strides = gait_events[key][side]
        
        fsr_sum = data[key][fsr_cols].sum(axis=1).values
        grf_fz = grf_aligned[key][f'{side}_Fz']
        
        # Take 10 strides from middle
        mid_stride = len(strides) // 2
        sel_strides = strides[mid_stride-5:mid_stride+5]
        
        fsr_profiles = []
        grf_profiles = []
        for s in sel_strides:
            hs_abs = onset + s['hs_idx']
            nhs_abs = onset + s['next_hs_idx']
            
            fsr_seg = fsr_sum[hs_abs:nhs_abs]
            grf_seg = grf_fz[hs_abs:nhs_abs]
            
            if len(fsr_seg) < 10:
                continue
            
            # Normalize to 0-100% gait cycle
            fsr_norm = np.interp(np.linspace(0, 1, 100), np.linspace(0, 1, len(fsr_seg)), fsr_seg)
            grf_norm = np.interp(np.linspace(0, 1, 100), np.linspace(0, 1, len(grf_seg)), grf_seg)
            
            # Scale FSR to match GRF amplitude for visual comparison
            fsr_profiles.append(fsr_norm)
            grf_profiles.append(grf_norm)
        
        if fsr_profiles:
            fsr_mean = np.mean(fsr_profiles, axis=0)
            grf_mean = np.mean(grf_profiles, axis=0)
            
            # Normalize both to 0-1 for shape comparison
            fsr_01 = (fsr_mean - fsr_mean.min()) / (fsr_mean.max() - fsr_mean.min() + 1e-10)
            grf_01 = (grf_mean - grf_mean.min()) / (grf_mean.max() - grf_mean.min() + 1e-10)
            
            ax.plot(gc_pct, grf_01, 'k-', linewidth=2, label='GRF (ref)')
            ax.plot(gc_pct, fsr_01, color=color, linewidth=2, label='FSR')
            ax.fill_between(gc_pct, grf_01, fsr_01, alpha=0.15, color='red')
        
        ax.set_title(f'{label}\n{side}', fontsize=9)
        ax.set_xlabel('Gait Cycle (%)')
        if i == 0:
            ax.set_ylabel('Normalized (0-1)')
        ax.legend(fontsize=7)
        ax.set_ylim(-0.1, 1.1)

plt.suptitle('Normalized Gait Cycle Shape: FSR vs GRF (mid-trial 10 strides)\n'
             'Red shading = shape difference between FSR and GRF',
             fontsize=13, fontweight='bold')
plt.tight_layout()
save_and_close('fig_22_gait_cycle_fsr_vs_grf')

# --- CELL BREAK ---

# GRF-FSR comprehensive summary
print("=" * 80)
print("GRF-FSR CROSS-VALIDATION SUMMARY")
print("=" * 80)

# 1. Synchronization
print("\n--- 1. Synchronization ---")
for key, label in zip(TEMPORAL_ORDER, TEMPORAL_LABELS):
    a = fp_assignments[key]
    print(f"{label}: Left={a['Left']}(r={a['left_corr']:.3f}, lag={a['left_lag']/ROBOT_FS*1000:.1f}ms), "
          f"Right={a['Right']}(r={a['right_corr']:.3f}, lag={a['right_lag']/ROBOT_FS*1000:.1f}ms)")

# 2. Calibration (sensitivity)
print("\n--- 2. FSR Sensitivity (ADC/N) ---")
print(f"{'Trial':<22} {'Left Slope':<14} {'Left R²':<10} {'Right Slope':<14} {'Right R²':<10}")
for key, label in zip(TEMPORAL_ORDER, TEMPORAL_LABELS):
    cl = calib_results[key]['Left']
    cr = calib_results[key]['Right']
    print(f"{label:<22} {cl['slope']:.4f}       {cl['r_squared']:.4f}     "
          f"{cr['slope']:.4f}       {cr['r_squared']:.4f}")

# 3. Rise time
print("\n--- 3. Rise Time (ms) ---")
print(f"{'Trial':<22} {'FSR (ms)':<14} {'GRF (ms)':<14} {'Diff (ms)':<14}")
for key, label in zip(TEMPORAL_ORDER, TEMPORAL_LABELS):
    subset = rt_df_clean[rt_df_clean['cond_key'] == key]
    fsr_rt = subset['fsr_rise_time_ms'].mean()
    grf_rt = subset['grf_rise_time_ms'].mean()
    diff = subset['rt_diff_ms'].mean()
    print(f"{label:<22} {fsr_rt:.1f}          {grf_rt:.1f}          {diff:+.1f}")

# 4. Subject weight estimation from standing GRF
print("\n--- 4. Subject Weight Estimation ---")
for key, label in zip(TEMPORAL_ORDER, TEMPORAL_LABELS):
    onset = walk_onsets[key]
    # Standing GRF (both feet)
    left_stand = grf_aligned[key]['Left_Fz'][:onset].mean()
    right_stand = grf_aligned[key]['Right_Fz'][:onset].mean()
    total_N = left_stand + right_stand
    weight_kg = total_N / 9.81
    print(f"{label}: Total standing GRF = {total_N:.1f}N → Est. weight = {weight_kg:.1f}kg")

# Dashboard plot
fig, axes = plt.subplots(1, 3, figsize=(18, 5))
x_pos = np.arange(4)

# Sensitivity (slope ADC/N)
ax = axes[0]
for side, marker in [('Left', 'o'), ('Right', 's')]:
    vals = [calib_results[k][side]['slope'] for k in TEMPORAL_ORDER]
    ax.plot(x_pos, vals, marker=marker, linewidth=2, markersize=8, label=side)
ax.set_title('FSR Sensitivity (ADC/N)')
ax.set_ylabel('Slope (ADC/N)')
ax.set_xticks(x_pos)
ax.set_xticklabels(TEMPORAL_LABELS, rotation=20, ha='right', fontsize=7)
ax.legend()

# Rise time difference
ax = axes[1]
for side, marker in [('Left', 'o'), ('Right', 's')]:
    vals = []
    for key in TEMPORAL_ORDER:
        subset = rt_df_clean[(rt_df_clean['cond_key'] == key) & (rt_df_clean['side'] == side)]
        vals.append(subset['rt_diff_ms'].mean())
    ax.plot(x_pos, vals, marker=marker, linewidth=2, markersize=8, label=side)
ax.axhline(0, color='gray', linestyle='--', linewidth=0.5)
ax.set_title('Rise Time Diff (FSR - GRF)')
ax.set_ylabel('Difference (ms)')
ax.set_xticks(x_pos)
ax.set_xticklabels(TEMPORAL_LABELS, rotation=20, ha='right', fontsize=7)
ax.legend()

# R-squared (calibration quality)
ax = axes[2]
for side, marker in [('Left', 'o'), ('Right', 's')]:
    vals = [calib_results[k][side]['r_squared'] for k in TEMPORAL_ORDER]
    ax.plot(x_pos, vals, marker=marker, linewidth=2, markersize=8, label=side)
ax.set_title('FSR-GRF Correlation (R²)')
ax.set_ylabel('R²')
ax.set_xticks(x_pos)
ax.set_xticklabels(TEMPORAL_LABELS, rotation=20, ha='right', fontsize=7)
ax.legend()
ax.set_ylim(0, 1)

plt.suptitle('GRF-FSR Cross-Validation Dashboard\n'
             'Degradation: sensitivity↓, rise time diff↑, R²↓',
             fontsize=14, fontweight='bold')
plt.tight_layout()
save_and_close('fig_23_grf_fsr_dashboard')

# (Old duplicate cross-correlation code removed - using find_lag_and_corr above)