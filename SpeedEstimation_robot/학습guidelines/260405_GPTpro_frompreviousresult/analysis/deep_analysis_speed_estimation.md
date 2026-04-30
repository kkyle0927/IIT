# Deep analysis of uploaded SpeedEstimation package

## What was in the uploaded package
The zip package contained:
- `overview.md`, `error_notes.md`, `llm_questions.md`
- `top5_summary.csv/json`
- five model cards
- PNG plots for the top-5 models (`trajectory`, `training_curves`, `m2_S008_level_100mps`, `m2_S008_level_125mps`, `m2_S008_stopandgo`)

It did **not** include raw trial-level numeric arrays such as per-timestep prediction CSVs, HDF5 signals, or condition-wise metrics tables. So exact bias statistics could not be recomputed numerically from the upload alone; the analysis below is based on the uploaded summaries and plots.

## Immediate high-confidence findings
1. The current error is **not** mainly a capacity problem.
   - `G2_fullkin_deeptcn` uses ~1.60M params but is not better than the 80k–230k models.
   - This strongly suggests the bottleneck is representation / objective / structure, not a lack of raw model size.

2. The current error is **not** well explained by a single global affine calibration error.
   - `L7_affine_calibration_biasloss` does not materially separate itself from J1/J7.
   - Visual plots show condition- and regime-specific errors, not one constant offset or gain mismatch.

3. The dominant failure mode is a **state-dependent underprediction in assisted conditions**, especially at `lv4` / `lv7`.
   - This is strongest in `level_125mps`, visible in all top-5 models.
   - It also appears in `level_100mps` as intermittent deep troughs during otherwise steady plateaus.

4. The problem looks like **assist-dependent representation mismatch / domain shift with a secondary calibration component**, not pure random noise.
   - Rise timing is often reasonable.
   - Stop-and-go timing is usually acceptable.
   - The model often reaches the right regime but settles too low, or produces phase-locked dips under assistance.

5. The failure is at least partly **timescale-specific**.
   - High-frequency timing is often okay.
   - Low-frequency plateau level is too low under assistance.
   - Some models also show gait-phase-synchronous troughs, implying that assistance-related features are being overused at the instantaneous timescale.

## Model-by-model interpretation
### 1) J7_hybrid_longwin_envelope
- Best MAE in the uploaded top-5.
- Best overall “balanced” model, but visually still shows strong assisted plateau underprediction.
- In `level_125mps`, the `lv7` plateau stays clearly low for long periods.
- In `level_100mps`, assisted conditions show repeated trough-like collapses during steady walking.

Interpretation:
- Long window + richer hybrid features help average accuracy.
- But the robot-derived branch still seems to leak assistance effects into instantaneous speed prediction.

### 2) J1_hybrid_power_base
- Very close to J7 with roughly half the parameters.
- Suggests the main gain comes from the basic hybrid idea itself, not necessarily from all the extra envelope/long-window complexity.
- Still shows plateau underprediction at `lv4/lv7`, especially in `level_125mps`.

Interpretation:
- Strong efficiency baseline.
- Good candidate for the next round of structured changes because it is simpler and cheaper than J7.

### 3) L7_affine_calibration_biasloss
- Calibration-aware structure helps in some local segments, but does not solve the core problem.
- In assisted plateaus it still drifts low or behaves inconsistently.

Interpretation:
- Useful negative result.
- The bias is not a simple affine mapping error; it is conditional, nonlinear, and likely gait-state/assist-state dependent.

### 4) G2_fullkin_deeptcn
- Huge model, no clear advantage.
- Often gets the transition shape reasonably well, but assisted plateau level still goes low.

Interpretation:
- Raw capacity is not the answer.
- Bigger end-to-end temporal models do not remove the confounding created by assisted locomotion.

### 5) M8_positive_residual_bestshot
- Worst MAE of the top-5, but best RMSE among them.
- Visually smoother and often better in stop-and-go than some other models.
- Still has persistent downward bias in high-speed assisted plateaus.

Interpretation:
- This is a very important clue.
- M8 seems to reduce catastrophic spikes / extreme deviations, but at the price of a more uniform amplitude compression / downward bias.
- So residual-style correction is directionally promising, but the current residual head is solving the wrong subproblem.

## Cross-model pattern that matters most
The most important pattern across the plots is this:

- In many assisted cases, the model gets the **timing** roughly right.
- It misses the **plateau magnitude**.
- In some models, it additionally produces **phase-locked dips** during otherwise steady walking.

That means the current architecture is mixing together two different estimation problems:

1. **Fast kinematic/inertial tracking**
   - “When is speed rising/falling?”
   - “What is the gait-cycle timing?”

2. **Slow assisted-regime calibration**
   - “Given assistance, what should the steady or quasi-steady speed level be?”
   - “How much should robot behavior alter the mapping from sensor pattern to actual speed?”

Right now these appear to be entangled in one predictor.

## Highest-value hypothesis
The best hypothesis consistent with the uploaded evidence is:

> The model is using assistance-related features too directly at the instantaneous 100 Hz timescale. Under higher assistance, biological effort-related signals shrink or shift, while the target speed does not shrink proportionally. The model therefore interprets assisted sensor changes as lower speed, producing slow plateau underprediction and sometimes phase-locked troughs.

This explains why:
- raw larger models do not solve it,
- affine calibration does not solve it,
- hybrid models help but still fail in assisted plateaus,
- residual variants reduce spikes but do not fix slow bias.

## Strongest next model idea
### Two-timescale decomposed estimator
Use a structured model that **forces** robot/assist features to influence the prediction mainly at a slow timescale.

#### Branch A: fast nominal-speed branch
Inputs:
- body-frame IMU
- hip/thigh kinematics
- `imu_integrated_speed`

Role:
- estimate the fast / nominal component of speed
- capture rise/fall timing and short-timescale gait dynamics

#### Branch B: slow assist-calibration branch
Inputs:
- stride-level or causal-EMA robot features only
  - power sums
  - torque envelope / RMS
  - deflection RMS
  - stride impulse
  - cadence / phase summaries
  - subject info

Role:
- estimate a slowly varying calibration term / bias term
- should **not** directly create high-frequency speed fluctuations

#### Combine
`y_hat = y_nominal_fast + b_slow`

with either:
- a slew-rate penalty on `b_slow`, or
- direct low-pass filtering / downsample-upsample structure for the slow branch.

### Why this is better than the current affine calibration attempt
Because the current error is not a single affine offset.
It is:
- assist-state dependent,
- nonlinear,
- slow in its mean bias,
- but sometimes phase-locked in its local troughs.

A slow calibration branch can fix the plateau level while preventing robot features from driving sample-wise collapses.

## Strongest training idea
### Low-frequency / high-frequency target decomposition
Since the target already uses offline bidirectional LPF, it is natural to decompose the label during training:

- `y_LF`: very low-frequency plateau / trend component
- `y_HF = y - y_LF`: residual gait-scale component

Train:
- slow assist branch -> `y_LF`
- fast inertial/kinematic branch -> `y_HF` or the nominal component

This is a strong fit to the visual evidence because the current failures are mostly low-frequency plateau bias, while timing is often acceptable.

## Specific experiments I would prioritize
### Priority 1 — Remove raw torque from the fast branch
Keep only:
- IMU + kinematics + `imu_integrated_speed` in the fast branch
- robot features only in the slow calibration branch

Why:
- The steady-plateau troughs look like phase-local confounding from instantaneous assist signals.
- Raw torque is likely too easy for the model to misuse.

### Priority 2 — Bias-aware steady-state loss
Add an auxiliary loss active when `|dy/dt|` is small:
- signed bias loss or Huber on low-pass error
- optionally weighted more when assist-intensity proxy is high

Why:
- MAE/RMSE alone underweight the exact problem you care about.
- The plots show plateau underprediction, not just transient spikes.

### Priority 3 — Continuous assist-intensity proxy
Do **not** use forbidden `assist_level`, but build a continuous causal proxy from allowed signals:
- torque_abs_ema_slow_sum
- motor_power sum
- deflection_rms sum
- stride impulse

Use it only to condition the slow calibration branch or as an auxiliary prediction target.

Why:
- The domain shift is clearly assist-related.
- A latent or derived “assist intensity” state is more physically meaningful than a hard categorical level.

### Priority 4 — Regime-aware evaluation and possibly regime-aware heads
Split evaluation into:
- steady plateau
- rise onset
- fall / deceleration
- stop-and-go bursts
- assisted high-speed plateau

Why:
- Different models appear to win in different regimes.
- Average MAE is hiding the important tradeoff structure.

### Priority 5 — Sample balancing by regime × assist intensity
Make sure training batches are balanced across:
- `lv0 / lv4 / lv7`
- condition type
- steady vs transient windows

Why:
- If assisted steady plateaus are underrepresented, the network will optimize average error and still fail where you care most.

## Metrics you should add immediately
The next comparison sheet should include more than MAE/RMSE:

1. **Signed bias** per condition and per assist bucket
2. **Steady-state bias** for windows with small `|dy/dt|`
3. **Calibration slope / intercept** from `pred` vs `GT`
4. **High-assist plateau bias** after onset settles
5. **Cycle-synchronous error amplitude** or spectral error around gait frequency

Without these, the paper-grade error story will stay blurry.

## Best baseline for the next round
Use **J1_hybrid_power_base** as the engineering baseline and **J7_hybrid_longwin_envelope** as the accuracy reference.

Why J1 as the main baseline:
- nearly tied with J7,
- much smaller,
- simpler feature set,
- easier to modify structurally without many confounds.

Why J7 as the upper reference:
- best uploaded MAE,
- shows what the current family can do when given more context.

## Most paper-worthy narrative from the current evidence
A strong paper story would be:

> In assisted walking, instantaneous robot signals confound direct speed regression. A two-timescale biomechanics-aware estimator that separates fast human locomotion dynamics from slow assist-induced calibration improves plateau accuracy and reduces assist-dependent underprediction while preserving causal deployability.

That is much stronger than “we tried a bigger model and it worked a bit better.”
