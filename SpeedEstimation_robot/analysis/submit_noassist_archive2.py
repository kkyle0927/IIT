from __future__ import annotations

import csv
import subprocess
from pathlib import Path

import yaml


ROOT = Path("/home/chanyoungko/IIT/SpeedEstimation_robot")
SCREEN_SUMMARY = ROOT / "compare_result/no_assist/archive_1/final_ablation_summary.csv"
ARCHIVE2_DIR = ROOT / "configs/no_assist/archive_2"

WINNER_MAP = {
    "exp_NA_A1_V0_w2_screen_noassist": {
        "source": ROOT / "configs/archive_season3/archive_7/exp_S3A7_V0_w2_reference_refresh.yaml",
        "exp_name": "exp_NA_A2_V0_w2_full_loso_noassist",
    },
    "exp_NA_A1_V7_predkin_screen_noassist": {
        "source": ROOT / "configs/archive_season3/archive_7/exp_S3A7_V7_predkin_anchormain.yaml",
        "exp_name": "exp_NA_A2_V7_predkin_full_loso_noassist",
    },
}

PARTS = [
    ("part1", ["m2_S001", "m2_S002", "m2_S003", "m2_S004"], "idx1", "SpeedRobot_NA_A2_train_1"),
    ("part2", ["m2_S005", "m2_S006", "m2_S007", "m2_S008"], "idx3", "SpeedRobot_NA_A2_train_2"),
]


def read_winner() -> tuple[str, dict]:
    if not SCREEN_SUMMARY.exists():
        raise FileNotFoundError(f"Missing screening summary: {SCREEN_SUMMARY}")

    rows = []
    with SCREEN_SUMMARY.open("r", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            group = row.get("group")
            if group in WINNER_MAP:
                rows.append((group, float(row["mae"])))

    if not rows:
        raise RuntimeError("No screening rows found for V0/V7 in final_ablation_summary.csv")

    winner_group, winner_mae = sorted(rows, key=lambda x: x[1])[0]
    winner_meta = WINNER_MAP[winner_group]
    print(f"[WINNER] {winner_group} mae={winner_mae:.6f}")
    return winner_group, winner_meta


def ensure_noassist(cfg: dict) -> dict:
    for key_path in [
        ("shared", "data"),
        ("02_train", "data"),
        ("03_eval", "data"),
    ]:
        cursor = cfg
        for key in key_path:
            cursor = cursor.setdefault(key, {})
        cursor["level_selection"] = {"include": ["lv0"]}
    return cfg


def write_configs(winner_meta: dict) -> list[Path]:
    ARCHIVE2_DIR.mkdir(parents=True, exist_ok=True)
    for old in ARCHIVE2_DIR.glob("exp_NA_A2_*.yaml"):
        old.unlink()

    with winner_meta["source"].open("r") as f:
        base_cfg = yaml.safe_load(f)

    base_cfg = ensure_noassist(base_cfg)
    base_cfg["exp_name"] = winner_meta["exp_name"]

    out_paths = []
    for part_name, loso_subjects, _partition, _job_name in PARTS:
        cfg = yaml.safe_load(yaml.safe_dump(base_cfg, sort_keys=False))
        cfg.setdefault("03_eval", {}).setdefault("split", {})["loso_subjects"] = loso_subjects
        out_path = ARCHIVE2_DIR / f"{winner_meta['exp_name']}_{part_name}.yaml"
        with out_path.open("w") as f:
            yaml.safe_dump(cfg, f, sort_keys=False)
        out_paths.append(out_path)
        print(f"[WRITE] {out_path}")
    return out_paths


def submit_jobs(cfg_paths: list[Path]) -> None:
    job_ids = []
    for cfg_path, (part_name, _loso_subjects, partition, job_name) in zip(cfg_paths, PARTS):
        out = subprocess.check_output(
            [
                "sbatch",
                "-p",
                partition,
                f"--gres=gpu:{partition}:1",
                "-J",
                job_name,
                str(ROOT / "train.sh"),
                str(cfg_path),
            ],
            text=True,
        ).strip()
        job_id = out.split()[-1]
        job_ids.append(job_id)
        print(f"[SUBMIT] {part_name}: {job_name} -> {job_id}")

    eval_out = subprocess.check_output(
        [
            "sbatch",
            "-p",
            "idx1",
            "--gres=gpu:idx1:1",
            f"--dependency=afterok:{job_ids[0]}:{job_ids[1]}",
            "-J",
            "SpeedRobot_NA_A2_eval_1",
            "--export=ALL,EVAL_SKIP_FEATURE_IMPORTANCE=1,EVAL_FULL_PLOT_TOPK=1,EVAL_FAST_EVAL=1,EVAL_PARALLEL_WORKERS=1",
            str(ROOT / "eval_server.sh"),
            "experiments",
            "--auto",
            "--filter",
            "exp_NA_A2_*",
        ],
        text=True,
    ).strip()
    print(f"[SUBMIT] eval -> {eval_out.split()[-1]}")


def main() -> None:
    _winner_group, winner_meta = read_winner()
    cfg_paths = write_configs(winner_meta)
    submit_jobs(cfg_paths)


if __name__ == "__main__":
    main()
