## Offset-Focused Literature Archive

Goal: remove assisted-condition speed offset in `SpeedEstimation_robot` without using `common/assist_level`, GRF, mocap, or treadmill speed as input.

### Archived references

1. Keehong Seo, "Real-Time Estimation of Walking Speed and Stride Length Using an IMU Embedded in a Robotic Hip Exoskeleton," ICRA 2023, DOI `10.1109/ICRA48891.2023.10160770`
   Link: https://doi.org/10.1109/ICRA48891.2023.10160770
   Local copy: `PreviousStudy/Real-Time_Estimation_of_Walking_Speed_and_Stride_Length_Using_an_IMU_Embedded_in_a_Robotic_Hip_Exoskeleton.pdf`
   Takeaway: exoskeleton-embedded sensing can estimate walking speed robustly, and stride-related latent structure is useful instead of a single monolithic speed regressor.

2. J. Lee et al., "Walking Speed and Uncertainty Estimation Using Mixture Density Networks for Dynamic Ambulatory Environments," EMBC 2024, DOI `10.1109/EMBC53108.2024.10781927`
   Link: https://doi.org/10.1109/EMBC53108.2024.10781927
   Local copy: `PreviousStudy/Walking_Speed_and_Uncertainty_Estimation_Using_Mixture_Density_Networks_for_Dynamic_Ambulatory_Environments.pdf`
   Takeaway: dynamic/transition conditions need condition-aware correction rather than a single globally calibrated mapping.

3. C. Medrano et al., "Continuous gait phase and speed-adaptive state estimation for robotic hip exoskeletons," 2023
   Link: https://pmc.ncbi.nlm.nih.gov/articles/PMC10249462/
   Takeaway: phase, phase rate, and stride-length/task states should be modeled separately; assistance should modulate gait state interpretation rather than replace it.

4. S. Byun et al., "Walking-speed estimation using a single inertial measurement unit for the older adults: a machine learning approach," 2019
   Link: https://pmc.ncbi.nlm.nih.gov/articles/PMC6932800/
   Takeaway: regime-specific specialized regressors can reduce systematic error that a single global regressor leaves behind.

5. K. Zeng et al., "Multi-rate gait phase estimation under dynamic walking conditions," 2024
   Link: https://pmc.ncbi.nlm.nih.gov/articles/PMC11054798/
   Takeaway: dynamic conditions benefit from short/long temporal context being used differently instead of uniformly.

### Archive_8 mapping

- `M1`, `M2`: repaired bounded-residual baseline with richer `J7 + L6` feature partition
- `M3`, `M4`: positive-only assist residual, motivated by offset being mostly negative underestimation
- `M5`, `M6`: assist-specific expert blending, motivated by regime-specific regressors
- `M7`: identity-centered affine calibration as a direct plateau-compression test
- `M8`: stronger positive-residual best-shot with slightly wider capacity but still realtime-friendly

### Design rules applied

- Main branch = human kinematics + trunk IMU + cadence/phase
- Context branch = torque, motor angle, deflection, power, slow torque envelope, stride impulse
- `common/assist_level` not used
- High-speed assisted offset is targeted by assist-gated mean-bias loss
