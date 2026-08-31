#!/usr/bin/env python3
"""Build poster-resolution figures for the ICNR 2026 poster.

Every number is read from the locked IJCAS evidence under
Experiment/paper_evidence/ or cropped from IJCAS/revision_submit/Fig*.png.
Nothing is recomputed and no model inference is required.

Figure geometry: each plot is authored at exactly the size it occupies on the
poster (COL_IN wide), axes margins are set explicitly and bbox_inches is NOT
used, so the saved aspect equals the placement box and the figure fills the
column at 1:1 scale. Font/line/marker sizes are given in points (an absolute
physical unit); scaling them by SCALE while scaling each figure's height_cm by
the same SCALE keeps every fractional margin below correct (top/bottom reserve
the same fraction of a proportionally taller canvas) while making every mark
and letter print SCALE times larger on the physical poster.

Font: Liberation Serif — metric-compatible with Times New Roman (this machine
has no licensed Times New Roman TTF to rasterize with). The poster's editable
PowerPoint text is set to the literal font "Times New Roman" and will render
with the real typeface in PowerPoint; only these baked PNG charts substitute
the metric-compatible clone.

Palette: the manuscript's red/green/blue fails CVD validation (Raw red vs Ridge
green, deutan ΔE 3.9). Raw and Proposed keep the manuscript's red and blue,
which pass every check as a pair; the two secondary baselines move to neutral
slate so colour is never the only thing separating two comparable series.
"""
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.font_manager as fm
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image

ROOT = Path("/home/chanyoungko/IIT/Misalign_compensation")
EV = ROOT / "Experiment/paper_evidence"
SRC_FIGS = ROOT / "IJCAS/revision_submit"
OUT = Path(__file__).resolve().parents[1] / "assets"

LO_FONTS = Path("/home/chanyoungko/opt/libreoffice/opt/libreoffice26.2/share/fonts/truetype")
for _f in ["LiberationSerif-Regular.ttf", "LiberationSerif-Bold.ttf",
           "LiberationSerif-Italic.ttf", "LiberationSerif-BoldItalic.ttf"]:
    fm.fontManager.addfont(str(LO_FONTS / _f))

REF = "#111111"      # motion-capture reference
RAW = "#d62728"      # raw robot angle          (manuscript red)
PROP = "#1f77b4"     # proposed estimator       (manuscript blue)
RIDGE = "#94A0AC"    # linear baseline          (de-emphasised)
BASE = "#5D6873"     # same TCN without yaw     (de-emphasised)
HF_C, HE_C = "#e6ab02", "#7a9a01"
GRID = "#D8DDE3"
INK = "#1A1A1A"       # every chart text is this ink — no grey text anywhere

SCALE = 1.07          # physical size multiplier for every pt-based spec below


def fs(pt):
    """Scaled font size, rounded to the nearest point."""
    return round(pt * SCALE)


plt.rcParams.update({
    "font.family": "Liberation Serif",
    "font.size": fs(26), "axes.labelsize": fs(29), "axes.titlesize": fs(26),
    "xtick.labelsize": fs(26), "ytick.labelsize": fs(26), "legend.fontsize": fs(25),
    "text.color": INK, "axes.labelcolor": INK,
    "xtick.color": INK, "ytick.color": INK,
    "axes.edgecolor": "#9AA3AC", "axes.linewidth": 1.8 * SCALE,
    "xtick.major.width": 1.8 * SCALE, "ytick.major.width": 1.8 * SCALE,
    "xtick.major.size": 8 * SCALE, "ytick.major.size": 8 * SCALE,
    "grid.color": GRID, "grid.linewidth": 1.4 * SCALE,
    "axes.spines.top": False, "axes.spines.right": False,
    "figure.facecolor": "white", "savefig.facecolor": "white",
})
DPI = 200
COL_IN = 14.06          # poster column content width, inches (35.7 cm) — fixed;
                         # this is what final on-page size is bound to, so only
                         # SCALE (not COL_IN) makes marks bigger on the poster.
CM = 2.54


def load(rel):
    return json.loads((EV / rel).read_text())


def figure(height_cm, **margins):
    fig = plt.figure(figsize=(COL_IN, height_cm * SCALE / CM))
    fig.subplots_adjust(**margins)
    return fig


def save(fig, name):
    path = OUT / name
    fig.savefig(path, dpi=DPI)
    plt.close(fig)
    w, h = Image.open(path).size
    print(f"{name}: {w}x{h}  aspect {w / h:.2f}")


def toprow(fig, ax, handles=None, labels=None, ncol=3, x=0.5, y=1.0, markerscale=1.0):
    """One legend row above the axes, in the space reserved by subplots_adjust."""
    if handles is None:
        handles, labels = ax.get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", bbox_to_anchor=(x, y),
               ncol=ncol, frameon=False, columnspacing=2.6, handlelength=2.0,
               markerscale=markerscale)


# ---------------------------------------------------------------- source crops
def crop(src, rows, cols, name):
    im = Image.open(SRC_FIGS / src).convert("RGBA")
    flat = Image.new("RGB", im.size, "white")
    flat.paste(im, mask=im.split()[-1])
    flat.crop((cols[0], rows[0], cols[1], rows[1])).save(OUT / name)
    print(f"{name}: {Image.open(OUT / name).size}")


def source_crops():
    crop("Fig1.png", (9, 648), (25, 957), "fig_p1_concept.png")
    crop("Fig2.png", (18, 411), (33, 885), "fig_p4_setup.png")
    crop("Fig2.png", (515, 1052), (33, 885), "fig_p3_sensors.png")
    # Fig4: the causal-TCN architecture diagram, used as-is (already the
    # manuscript's own figure) — trim to its content bounding box only.
    im = Image.open(SRC_FIGS / "Fig4.png").convert("RGBA")
    flat = Image.new("RGB", im.size, "white")
    flat.paste(im, mask=im.split()[-1])
    bbox = Image.open(SRC_FIGS / "Fig4.png").convert("L").point(
        lambda v: 0 if v > 250 else 255).getbbox()
    pad = 6
    box = (max(0, bbox[0] - pad), max(0, bbox[1] - pad),
           min(flat.width, bbox[2] + pad), min(flat.height, bbox[3] + pad))
    flat.crop(box).save(OUT / "fig_p_architecture.png")
    print(f"fig_p_architecture.png: {Image.open(OUT / 'fig_p_architecture.png').size}")


# ------------------------------------------------------------------ P2 yaw cue
def fig_yaw():
    st = load("biomechanics/additional_mocap_relative_yaw_stats.json")["primary_result_left"]
    amp = st["amplitude_by_speed"]
    subs = st["subjects_with_all_three_speeds"]
    speeds, means, sds = [], [], []
    for key in ["level_075mps", "level_100mps", "level_125mps"]:
        v = np.array([amp[key]["subject_values_deg"][s] for s in subs])
        speeds.append(amp[key]["speed_mps"])
        means.append(v.mean())
        sds.append(v.std(ddof=1))

    fig = figure(11.0, left=0.135, right=0.99, top=0.93, bottom=0.215)
    ax = fig.add_subplot(111)
    ax.errorbar(speeds, means, yerr=sds, color=PROP, lw=6, marker="o", ms=21,
                capsize=14, capthick=4, elinewidth=3.5, zorder=3)
    for x, y in zip(speeds, means):
        ax.annotate(f"{y:.2f}°", (x, y), textcoords="offset points", xytext=(0, 26),
                    ha="center", fontsize=fs(30), fontweight="bold", color=PROP)
    ax.set_xlabel("Walking speed [m/s]")
    ax.set_ylabel("Yaw range [°]")
    ax.set_xticks(speeds)
    ax.set_xlim(0.66, 1.34)
    ax.set_ylim(6, 33)
    ax.grid(axis="y", zorder=0)
    ax.set_axisbelow(True)
    ax.text(0.015, 0.97,
            f"cycle-wise  r = {st['r_cycle_mean']:.2f} ± {st['r_cycle_sd']:.2f}"
            f"   ({st['n_cycles']:,} cycles)\n"
            f"per-participant  r = {st['r_subject_mean']:.2f} ± {st['r_subject_sd']:.2f}"
            f"   ({st['n_subject_negative']}/10 negative)",
            transform=ax.transAxes, ha="left", va="top", fontsize=fs(23), color=INK,
            linespacing=1.35)
    save(fig, "fig_p2_yaw.png")


# ------------------------------------------------------- R1 no-assist 16-fold
def fig_indomain():
    d = load("final_model/in_domain/review_yaw_indomain_stride1_results_v2.json")
    ms, per = d["model_summaries"], d["participant_seed_averaged_metrics_deg"]
    rows = [("Raw robot angle", "raw", RAW), ("Ridge (linear)", "ridge_alpha1", RIDGE),
            ("TCN, no yaw", "base", BASE), ("Proposed", "proposed", PROP)]

    fig = figure(10.0, left=0.255, right=0.99, top=0.90, bottom=0.235, wspace=0.30)
    gs = fig.add_gridspec(1, 2, width_ratios=[1.45, 1.0],
                          left=0.255, right=0.99, top=0.90, bottom=0.235, wspace=0.30)
    ax1, ax2 = fig.add_subplot(gs[0]), fig.add_subplot(gs[1])

    y = np.arange(len(rows))[::-1]
    vals = [ms[k]["mean_deg"]["Overall"] for _, k, _ in rows]
    errs = [ms[k]["participant_sample_sd_deg"]["Overall"] for _, k, _ in rows]
    ax1.barh(y, vals, xerr=errs, height=0.62, color=[c for _, _, c in rows], zorder=3,
             error_kw=dict(elinewidth=3.5, capsize=12, capthick=3.5, ecolor="#3A4048"))
    for yy, v, e in zip(y, vals, errs):
        ax1.text(v + e + 0.30, yy, f"{v:.2f}°", va="center", fontsize=fs(29),
                 fontweight="bold", color=INK)
    ax1.set_yticks(y)
    ax1.set_yticklabels([n for n, _, _ in rows])
    ax1.set_xlabel("Overall MAE [°]")
    ax1.set_xlim(0, 10.4)
    ax1.grid(axis="x", zorder=0)
    ax1.set_axisbelow(True)
    ax1.set_title("(a)  group mean ± SD", pad=12)

    subs = list(per["raw"].keys())
    r = np.array([per["raw"][s]["Overall"] for s in subs])
    p = np.array([per["proposed"][s]["Overall"] for s in subs])
    for rv, pv in zip(r, p):
        ax2.plot([0, 1], [rv, pv], color="#C3CAD2", lw=2.6, zorder=1)
    ax2.scatter(np.zeros_like(r), r, s=255, color=RAW, zorder=3)
    ax2.scatter(np.ones_like(p), p, s=255, color=PROP, zorder=3)
    ax2.set_xlim(-0.42, 1.42)
    ax2.set_xticks([0, 1])
    ax2.set_xticklabels(["Raw", "Proposed"])
    ax2.set_ylabel("Overall MAE [°]", labelpad=2)
    ax2.set_ylim(min(r.min(), p.min()) - 0.6, r.max() + 2.4)
    ax2.grid(axis="y", zorder=0)
    ax2.set_axisbelow(True)
    ax2.set_title("(b)  per participant", pad=12)
    ax2.text(0.5, 0.97, "16 / 16", transform=ax2.transAxes, ha="center", va="top",
             fontsize=fs(32), fontweight="bold", color=PROP)
    save(fig, "fig_r1_indomain.png")


# ------------------------------------------------- R2 unseen-condition shifts
def fig_ood():
    d = load("final_model/ood/review_yaw_operational_results.json")["protocols"]
    labels = ["Assistance\n(lv4 / lv7)\nn = 10", "Faster walking\n(1.25 m/s)\nn = 13",
              "Stop-and-go\nwalking\nn = 16"]
    keys = ["assist", "speed", "sag"]
    g = lambda k, m, f: d[k]["model_summaries"][m]["sample"][f][0]
    raw = [g(k, "raw", "participant_equal_mean_deg") for k in keys]
    raw_sd = [g(k, "raw", "participant_sample_sd_deg") for k in keys]
    pro = [g(k, "proposed", "participant_equal_mean_deg") for k in keys]
    pro_sd = [g(k, "proposed", "participant_sample_sd_deg") for k in keys]

    x, w = np.arange(3), 0.32
    fig = figure(11.3, left=0.105, right=0.99, top=0.825, bottom=0.335)
    ax = fig.add_subplot(111)
    ek = dict(elinewidth=3.5, capsize=13, capthick=3.5, ecolor="#3A4048")
    b1 = ax.bar(x - w / 2 - 0.012, raw, w, yerr=raw_sd, color=RAW, zorder=3,
                label="Raw robot angle", error_kw=ek)
    b2 = ax.bar(x + w / 2 + 0.012, pro, w, yerr=pro_sd, color=PROP, zorder=3,
                label="Proposed", error_kw=ek)
    for xi, rv, pv, rs in zip(x, raw, pro, raw_sd):
        ax.annotate(f"−{100 * (rv - pv) / rv:.1f}%", (xi, rv + rs + 0.6), ha="center",
                    fontsize=fs(34), fontweight="bold", color=PROP)
        ax.text(xi - w / 2 - 0.012, rv / 2, f"{rv:.2f}°", ha="center", va="center",
                fontsize=fs(26), color="white", fontweight="bold")
        ax.text(xi + w / 2 + 0.012, pv / 2, f"{pv:.2f}°", ha="center", va="center",
                fontsize=fs(26), color="white", fontweight="bold")
    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.set_ylabel("Overall MAE [°]")
    ax.set_ylim(0, 13.2)
    ax.grid(axis="y", zorder=0)
    ax.set_axisbelow(True)
    toprow(fig, ax, [b1, b2], ["Raw robot angle", "Proposed"], ncol=2)
    save(fig, "fig_r2_ood.png")


# ------------------------------------------ R3 assistance shift, phase + trace
def fig_assist():
    d = load("figure_evidence/review_yaw_operational_figure_summary.json")
    ph = np.array(d["phase_percent"])
    pw, rt = d["phase_wise_mae"], d["representative_trajectory"]

    fig = figure(11.6, left=0.105, right=0.99, top=0.735, bottom=0.205)
    gs = fig.add_gridspec(1, 2, width_ratios=[1.0, 1.32],
                          left=0.105, right=0.99, top=0.735, bottom=0.205, wspace=0.23)
    ax1, ax2 = fig.add_subplot(gs[0]), fig.add_subplot(gs[1])

    r = np.array(pw["raw"]["participant_equal_mean_mae_deg"])
    p = np.array(pw["proposed"]["participant_equal_mean_mae_deg"])
    ax1.fill_between(ph, p, r, color=PROP, alpha=0.10, zorder=2)
    ax1.plot(ph, r, color=RAW, lw=6, ls="--", zorder=3)
    ax1.plot(ph, p, color=PROP, lw=6, zorder=3)
    ax1.set_xlabel("Gait phase [%]")
    ax1.set_ylabel("MAE [°]", fontsize=fs(27))
    ax1.set_xlim(0, 100)
    ax1.set_ylim(0, 21)
    ax1.grid(zorder=0)
    ax1.set_axisbelow(True)
    ax1.set_title("(a)  across the gait cycle", pad=12)

    t = np.array(rt["time_s"])
    tar, rw, pr = (np.array(rt[k]) for k in ("target_deg", "raw_deg", "proposed_deg"))
    hi, lo = rt["hf_threshold_deg"], rt["he_threshold_deg"]
    top, bot = max(tar.max(), rw.max()), min(tar.min(), rw.min())
    ax2.axhspan(hi, top + 4, color=HF_C, alpha=0.13, zorder=0)
    ax2.axhspan(bot - 4, lo, color=HE_C, alpha=0.13, zorder=0)
    l0, = ax2.plot(t, tar, color=REF, lw=6, zorder=3, label="True (mocap)")
    l1, = ax2.plot(t, rw, color=RAW, lw=5.2, ls="--", zorder=3, label="Raw robot angle")
    l2, = ax2.plot(t, pr, color=PROP, lw=5.2, zorder=3, label="Proposed")
    ax2.set_xlabel("Time [s]")
    ax2.set_ylabel("Thigh angle [°]", labelpad=2)
    ax2.set_xlim(t[0], t[-1])
    ax2.set_ylim(bot - 4, top + 4)
    ax2.grid(zorder=0)
    ax2.set_axisbelow(True)
    ax2.set_title("(b)  representative assisted trial", pad=12)
    toprow(fig, ax2, [l0, l1, l2], ["True (mocap)", "Raw robot angle", "Proposed"], ncol=3, y=0.99)
    save(fig, "fig_r3_assist.png")


# ------------------------------------------------------- R4 application view
def fig_application():
    d = load("figure_evidence/review_yaw_operational_figure_summary.json")
    rom, ex = d["participant_equal_cycle_rom"], d["assist_ood_exceedance"]

    fig = figure(11.3, left=0.125, right=0.99, top=0.572, bottom=0.262)
    gs = fig.add_gridspec(1, 2, width_ratios=[1.0, 1.28],
                          left=0.125, right=0.99, top=0.572, bottom=0.262, wspace=0.30)
    ax1, ax2 = fig.add_subplot(gs[0]), fig.add_subplot(gs[1])

    x = [0, 1]
    series = [("Reference (mocap)", "reference", REF, "o", "-"),
              ("Raw robot angle", "raw", RAW, "s", "--"),
              ("Proposed", "proposed", PROP, "^", "-")]
    handles = []
    for lab, key, col, mk, ls in series:
        m = [rom[lv][key]["participant_equal_mean_deg"] for lv in ("lv0", "lv7")]
        s = [rom[lv][key]["participant_sample_sd_deg"] for lv in ("lv0", "lv7")]
        h = ax1.errorbar(x, m, yerr=s, color=col, lw=5.2, ls=ls, marker=mk, ms=20,
                         capsize=12, capthick=3.5, elinewidth=3.2, label=lab, zorder=3)
        handles.append(h)
    ax1.set_xticks(x)
    ax1.set_xticklabels(["no assist\n(lv0)", "assist\n(lv7)"])
    ax1.set_ylabel("Thigh ROM [°]")
    ax1.set_xlim(-0.35, 1.35)
    ax1.set_ylim(26, 72)
    ax1.grid(axis="y", zorder=0)
    ax1.set_axisbelow(True)
    ax1.set_title("(a)  thigh ROM", pad=12)

    for key, col, ls in [("raw", RAW, "--"), ("proposed", PROP, "-")]:
        th = np.array(ex[key]["thresholds_deg"])
        pr = 100 * np.array(ex[key]["participant_equal_mean_probability"])
        lo = 100 * np.array(ex[key]["participant_bootstrap_ci95_lower"])
        up = 100 * np.array(ex[key]["participant_bootstrap_ci95_upper"])
        ax2.fill_between(th, lo, up, color=col, alpha=0.15, zorder=2)
        ax2.plot(th, pr, color=col, lw=6, ls=ls, zorder=3)
        v = np.interp(5.0, th, pr)
        ax2.annotate(f"{v:.1f}%", (5.0, v), textcoords="offset points",
                     xytext=(18, 14 if key == "raw" else -6), fontsize=fs(30),
                     fontweight="bold", color=col, zorder=4)
    ax2.axvline(5.0, color="#9AA3AC", lw=3, zorder=1)
    ax2.set_xlabel("Absolute error threshold [°]")
    ax2.set_ylabel("Exceedance [%]", labelpad=2)
    ax2.set_xlim(0, 20)
    ax2.set_ylim(0, 102)
    ax2.grid(zorder=0)
    ax2.set_axisbelow(True)
    ax2.set_title("(b)  how often large errors occur", pad=12)
    toprow(fig, ax1, handles, [s[0] for s in series], ncol=3, y=0.99, markerscale=0.62)
    save(fig, "fig_r4_application.png")


if __name__ == "__main__":
    OUT.mkdir(exist_ok=True)
    source_crops()
    fig_yaw()
    fig_indomain()
    fig_ood()
    fig_assist()
    fig_application()
