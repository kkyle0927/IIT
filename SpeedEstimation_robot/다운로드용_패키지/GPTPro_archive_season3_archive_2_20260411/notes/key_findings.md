# Key Findings

## What clearly worked
- `P6 switch_dualhead` is the current accuracy-best model in this archive.
- `P4 onbias_corrector` is the strongest pure bias-correction style model and looks very deployable.
- `P7 regime_mixture` is competitive, but weaker than `P6` and `P4`.

## What this suggests
- Splitting assist-off and assist-on behavior at the prediction head helps.
- Raw assist-related information still seems useful, but mainly as calibration/routing context, not as a direct fast-path predictor.
- A stronger bias corrector helps more than a tiny bias head.

## What did not win
- Full split `P8` did not beat the lighter routed variants.
- The paper-inspired cadence anchor was useful as an idea source, but the direct anchor model variants were not top performers here.

## Important nuance
`R1` still has very strong RMSE and is a useful reference, but it is much slower than `P6`/`P4`.

## Recommended interpretation axes
- `accuracy-best`: lowest overall MAE / RMSE
- `offset-best`: smallest steady plateau bias in high-assist `lv4/lv7`
- `deployable-best`: low latency with good accuracy and stable bias behavior

