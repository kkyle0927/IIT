from __future__ import annotations

import argparse
import shutil
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import pandas as pd


def copytree_replace(src: Path, dst: Path) -> None:
    if dst.exists():
        shutil.rmtree(dst)
    shutil.copytree(src, dst)


def sync_selected_folds(src_root: Path, dst_root: Path, pattern: str) -> list[Path]:
    synced = []
    for src_fold in sorted(src_root.glob(pattern)):
        if not src_fold.is_dir():
            continue
        dst_fold = dst_root / src_fold.name
        copytree_replace(src_fold, dst_fold)
        synced.append(dst_fold)
    return synced


def load_feature_importance_tables(final_root: Path) -> pd.DataFrame:
    rows = []
    for csv_path in sorted(final_root.glob("exp_NA_A9_D6_tvcf_roll18_og_screen_noassist_Test-*/**/feature_importance.csv")):
        try:
            df = pd.read_csv(csv_path)
        except Exception:
            continue
        if not {"feature", "importance"}.issubset(df.columns):
            continue
        fold = csv_path.parents[1].name
        tmp = df[["feature", "importance"]].copy()
        tmp["fold"] = fold
        rows.append(tmp)
    if not rows:
        return pd.DataFrame(columns=["fold", "feature", "importance"])
    return pd.concat(rows, ignore_index=True)


def save_feature_importance_aggregate(fi_long: pd.DataFrame, out_dir: Path) -> None:
    if fi_long.empty:
        return

    agg = (
        fi_long.groupby("feature", as_index=False)["importance"]
        .agg(["mean", "std", "count"])
        .reset_index()
        .sort_values("mean", ascending=False)
    )
    agg.columns = ["feature", "mean_importance", "std_importance", "count"]
    agg.to_csv(out_dir / "aggregate_feature_importance_mean_std.csv", index=False)

    top = agg.head(18).iloc[::-1]
    plt.figure(figsize=(9, max(6, 0.35 * len(top))))
    plt.barh(
        top["feature"],
        top["mean_importance"],
        xerr=top["std_importance"].fillna(0.0),
        color="#4C78A8",
        alpha=0.9,
        ecolor="#222222",
        capsize=3,
    )
    plt.xlabel("Permutation importance (mean ± std across LOSO folds)")
    plt.ylabel("Feature")
    plt.title("Aggregate Feature Importance")
    plt.tight_layout()
    plt.savefig(out_dir / "aggregate_feature_importance_mean_std.png", dpi=180, bbox_inches="tight")
    plt.close()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--src-root", required=True)
    parser.add_argument("--dst-root", required=True)
    parser.add_argument(
        "--pattern",
        default="exp_NA_A9_D6_tvcf_roll18_og_screen_noassist_Test-*",
    )
    args = parser.parse_args()

    src_root = Path(args.src_root)
    dst_root = Path(args.dst_root)
    dst_root.mkdir(parents=True, exist_ok=True)

    synced = sync_selected_folds(src_root, dst_root, args.pattern)
    print(f"[INFO] Synced {len(synced)} fold directories into {dst_root}")

    fi_long = load_feature_importance_tables(dst_root)
    save_feature_importance_aggregate(fi_long, dst_root)
    print(f"[INFO] Feature importance rows: {len(fi_long)}")


if __name__ == "__main__":
    main()
