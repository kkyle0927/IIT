Misalign_compensation lightweight package for GPT analysis

Included:
- Core code: model_training.py, compare_results.py, config_utils.py, train/eval scripts
- All archive configs: configs/archive_season1, archive_season2, archive_season3
- All experiment-level metrics.json files under experiments/
- compare_result CSV/TSV/JSON summaries only (no PNG)
- historical_model_ranking.csv and all_experiment_metrics_manifest.csv
- combined_data_structure.txt / .json describing H5 layout (raw H5 excluded)
- existing internal reports/docs PDFs/DOCX

Excluded to keep size small:
- combined_data_S008.h5 raw data
- logs/
- *.png under compare_result and elsewhere
- model checkpoints (*.pt, *.pth)
- large caches and temporary files

How to use:
1. Start with README_FOR_GPT.txt
2. Read historical_model_ranking.csv for historical ordering
3. Read all_experiment_metrics_manifest.csv for per-experiment results
4. Use configs/ + model_training.py to understand structure, inputs, targets, and training setup
5. Use combined_data_structure.* to understand H5 hierarchy without raw data
