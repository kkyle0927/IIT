# Questions For GPT Pro

Please review the attached results for `archive_season3/archive_2` and answer these questions.

1. Why does the steady offset persist most strongly in `level_100mps` and `level_125mps`, especially `lv4` and `lv7`, even after removing raw torque from the fast path in earlier archives?
2. Does `P6 switch_dualhead` look like a principled solution, or is it mainly acting as a regime-specific shortcut?
3. Is `P4 onbias_corrector` a better long-term direction than `P6` for a paper contribution, given its simpler structure?
4. Looking at the `condition_level_summary.csv` tables and the plots, which model seems best for reducing plateau offset specifically, not just overall MAE?
5. Would you recommend pushing further on:
   - assist on/off routed heads
   - persistent streaming bias state
   - stronger absolute-speed anchor
   - plateau-specific loss
   - some other structure
6. What would be the strongest next archive if the goal is:
   - better `lv4/lv7` plateau alignment
   - preserved transition timing
   - causal online inference
7. Which architecture from this archive would be strongest as a paper contribution, and why?

## Extra context
- The target is `LPF(common/speed_y)`, not treadmill speed.
- Input preprocessing and model inference must remain causal / online-feasible.
- `common/assist_level` is not allowed as an inference input.

