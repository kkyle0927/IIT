# Archive 9

`D6` treadmill-to-overground feasibility screening.

- Baseline retraining is skipped; compare against `archive_7` / `archive_8` D6 results.
- New feature chain replaces the forward-like horizontal TVCF features with overground-compensated variants:
  - `acc_hy_og = acc_hy + a_treadmill`
  - `vel_hy_og = leaky_integrate(acc_hy_og)`
  - `acc_h_mag_og = sqrt(acc_hx^2 + acc_hy_og^2)`
  - `vel_h_mag_og = sqrt(vel_hx^2 + vel_hy_og^2)`
- 3-fold LOSO feasibility only:
  - `m2_S001`
  - `m2_S004`
  - `m2_S008`
