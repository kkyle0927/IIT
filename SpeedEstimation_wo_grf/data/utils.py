import numpy as np
from scipy.signal import butter, filtfilt


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
fs = 100
time_window_output = 10
stride = 10

CONDITION_SELECTION = {'include': None, 'exclude': []}


# ---------------------------------------------------------------------------
# Variable Parsing
# ---------------------------------------------------------------------------
def make_subject_selection(include_list):
    return {'include': include_list, 'exclude': []}


def parse_vars(var_list_from_yaml):
    """
    Converts list-based YAML vars [[path, [vars]], ...] to tuple-based list.
    """
    if not var_list_from_yaml:
        return []
    parsed = []
    for item in var_list_from_yaml:
        if isinstance(item, (list, tuple)) and len(item) >= 2:
            gpath = item[0]
            vars = item[1]
            parsed.append((gpath, vars))
    return parsed


# ---------------------------------------------------------------------------
# Signal Filtering
# ---------------------------------------------------------------------------
def butter_lowpass_filter(data, cutoff, fs, order=4):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    if data.ndim == 1:
        y = filtfilt(b, a, data)
    else:
        y = filtfilt(b, a, data, axis=0)
    return y


def butter_highpass_filter(data, cutoff, fs, order=4):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='high', analog=False)
    if data.ndim == 1:
        y = filtfilt(b, a, data)
    else:
        y = filtfilt(b, a, data, axis=0)
    return y


# ---------------------------------------------------------------------------
# Ground Contact
# ---------------------------------------------------------------------------
def get_ground_contact(grf_fz, threshold=20.0):
    return (np.nan_to_num(grf_fz) > threshold).astype(float)


# ---------------------------------------------------------------------------
# H5 Group Helper
# ---------------------------------------------------------------------------
def get_data_from_group(g, path):
    parts = path.split('/')
    curr = g
    for p in parts:
        if p in curr: curr = curr[p]
        else: return None
    return curr[:]


# ---------------------------------------------------------------------------
# Filtering Visualization (Interactive)
# ---------------------------------------------------------------------------
def plot_filtering_check(data_path, sub, cond, output_vars, lpf_cutoff, fs=100):
    """
    Visualizes the effect of LPF on the reference velocity for a single trial.
    Blocks execution until closed.
    Includes Button to Scale Up (5s window) and Slider to Scroll.
    """
    import h5py
    import numpy as np
    import matplotlib.pyplot as plt
    from matplotlib.widgets import Slider, Button

    print(f"\n[INFO] Visualizing Filtering Effect for {sub}/{cond}...")
    with h5py.File(data_path, 'r') as f:
        if sub not in f or cond not in f[sub]:
            print("  -> Sample not found.")
            return

        cond_grp = f[sub][cond]
        lv_key = list(cond_grp.keys())[0]
        trial_key = list(cond_grp[lv_key].keys())[0]
        trial_grp = cond_grp[lv_key][trial_key]

        raw_vals = {}
        filtered_vals = {}

        for gpath, vars in output_vars:
            g = trial_grp
            parts = gpath.split('/')
            valid = True
            for p in parts:
                if p in g: g = g[p]
                else: valid=False; break
            if not valid: continue

            for v in g:
                if v in g:
                    raw = g[v][:]
                    raw = np.nan_to_num(raw)
                    raw_vals[v] = raw

                    if lpf_cutoff is not None:
                        from scipy.signal import butter, filtfilt
                        nyq = 0.5 * fs
                        normal_cutoff = lpf_cutoff / nyq
                        b, a = butter(4, normal_cutoff, btype='low', analog=False)
                        filt = filtfilt(b, a, raw)
                        filtered_vals[v] = filt

        if not raw_vals:
            print("  -> No output vars found to plot.")
            return

        num_plots = len(raw_vals)
        fig, axes = plt.subplots(num_plots, 1, figsize=(10, 4*num_plots), sharex=True)
        if num_plots == 1: axes = [axes]

        plt.subplots_adjust(bottom=0.25)

        lines_raw = []
        lines_filt = []

        max_idx = 0
        for i, (vname, rdata) in enumerate(raw_vals.items()):
            max_idx = max(max_idx, len(rdata))
            ax = axes[i]
            l1, = ax.plot(rdata, label='Raw', alpha=0.5, color='gray')
            lines_raw.append(l1)

            if vname in filtered_vals:
                l2, = ax.plot(filtered_vals[vname], label=f'Filtered ({lpf_cutoff}Hz)', color='red', linewidth=1.5)
                lines_filt.append(l2)

            ax.set_title(f"Reference Signal: {vname}")
            ax.legend()
            ax.grid(True)

        axcolor = 'lightgoldenrodyellow'

        ax_scroll = plt.axes([0.2, 0.1, 0.65, 0.03], facecolor=axcolor)
        s_scroll = Slider(ax_scroll, 'Scroll', 0, max_idx, valinit=0)

        ax_button = plt.axes([0.8, 0.025, 0.1, 0.04])
        btn_zoom = Button(ax_button, 'Zoom 5s', color=axcolor, hovercolor='0.975')

        window_size_samples = 5 * fs
        is_zoomed = [False]

        def update(val):
            if not is_zoomed[0]:
                return
            start = int(s_scroll.val)
            end = start + window_size_samples
            for ax in axes:
                ax.set_xlim(start, end)
            fig.canvas.draw_idle()

        def toggle_zoom(event):
            is_zoomed[0] = not is_zoomed[0]
            if is_zoomed[0]:
                btn_zoom.label.set_text("Reset View")
                update(s_scroll.val)
            else:
                btn_zoom.label.set_text("Zoom 5s")
                for ax in axes:
                    ax.set_xlim(auto=True)
                    ax.relim()
                    ax.autoscale_view()
                fig.canvas.draw_idle()

        s_scroll.on_changed(update)
        btn_zoom.on_clicked(toggle_zoom)

        print("  -> Interactively check filtering. Close to continue...")
        plt.show(block=True)
