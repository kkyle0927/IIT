# GPT Pro Packet: archive_season3/archive_2

This packet is a compact handoff for external LLM review of `archive_season3/archive_2` in `SpeedEstimation_robot`.

Goal of this archive:
- Reduce steady offset in `level_100mps` and `level_125mps`, especially `lv4` and `lv7`
- Keep fast dynamics while improving absolute plateau level
- Stay causal / online-feasible at inference time

Read in this order:
1. `overview.md`
2. `notes/key_findings.md`
3. `csv/top5_summary.csv`
4. `csv/lv4_lv7_offset_focus.csv`
5. `csv/final_ablation_summary.csv`
6. `plots/` for the top 5 models
7. `llm_questions.md`

Top 5 models in this packet:
- `P6`: shared backbone + assist on/off switch dual head
- `P4`: nominal + stronger assist-on bias corrector
- `P7`: regime mixture expert
- `R1`: persistent bias state reference
- `P8`: full assist off/on split model

Important project constraints:
- Final target is `LPF(common/speed_y)` with offline bidirectional LPF
- Raw inputs allowed: robot sensors, trunk IMU, subject anthropometrics
- No mocap, GRF, direct treadmill speed input, or leaky derived features from them
- Input preprocessing and inference must remain causal / online-feasible
- `common/assist_level` is not used as an inference input

