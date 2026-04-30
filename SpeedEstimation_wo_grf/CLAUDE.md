# SpeedEstimation_wo_grf

Exoskeleton + biomechanics 센서(mocap, IMU, robot proprioception) 기반 corrected speed 추정 딥러닝 프로젝트. 입력에서 GRF는 사용하지 않는다.

## 유지보수 규칙

- 프로젝트 구조, 코드 변경 등이 발생하면 이 CLAUDE.md도 함께 업데이트할 것
- 코드 구조가 변경되면 (새 모듈 추가 등) 관련 섹션 반영

## 파일 생성 규칙

- 새로운 파일을 repository에 추가하기 전 반드시 사용자에게 한국어로 파일 경로, 목적, 생성 이유를 설명하고 허락을 받을 것
- 작업 중 임시 파일을 만들었다면 같은 작업 안에서 직접 삭제할 경우 사용자 허락 없이 진행해도 됨
- 기존 코드 실행 결과로 생성되는 `png`, `log`, `csv`, `pt`, `npz` 등의 산출물은 사용자 허락 없이 생성되어도 됨
- `configs/archive_seasonX/archive_Y/...` 또는 `experiments/archive_seasonX/archive_Y/...`를 평가할 때 결과는 반드시 `compare_result/archive_seasonX/archive_Y/...`에 저장되도록 유지할 것
- archive 실험용 `exp_name`은 repository 전체에서 유일해야 한다. 다른 archive와 `experiments/<exp_name>_...` 폴더가 충돌하지 않도록 `exp_S2A5_...`처럼 archive 식별자를 포함할 것

## CLAUDE.md 작성 규칙

- 이 파일에 규칙을 추가하거나 수정할 때 `가능하면`, `적절히`, `웬만하면` 같은 모호한 표현을 쓰지 말 것
- 규칙은 검증 가능한 조건-행동 형식으로 작성할 것. 예: `X를 바꾸면 Y도 같이 갱신할 것`
- 권장 표현보다 금지 표현을 우선 사용할 것. 예: `X를 쓰지 말고 Y만 사용할 것`
- 이 파일은 200줄 이하를 유지할 것
- 규칙이 길어지면 핵심 규칙만 이 파일에 남기고 나머지는 별도 문서나 규칙 파일로 분리할 것

## 수정 제한 파일

아래 파일들은 수정 전 반드시 사용자에게 한국어로 변경 내용/이유를 설명하고 허락을 받아야 함:
- `SpeedEstimator_TCN_MLP_experiments.py` — GOLDEN BASELINE 학습 스크립트
- `train.sh` — SLURM 학습 제출 스크립트

## 실행 환경

- conda 환경: `IIT`
- SLURM 파티션: `idx0`~`idx3` (대시보드 GPU 번호 = SLURM idx 번호)
- GPU 독점 방지: 최대 3개까지만 사용
- 학습: `sbatch -p idxN --job-name="이름" --gres=gpu:idxN:1 --mem=15G --cpus-per-task=8 train.sh configs/path/to/config.yaml`
- 평가: `sbatch -p idxN --job-name="이름" --gres=gpu:idxN:1 eval_server.sh --auto --ablation`
- 빠른 평가가 필요하면 `EVAL_STRIDE=N` 환경변수로 evaluation stride를 늘릴 것. 1차 fast eval은 `EVAL_STRIDE=3` 이상을 우선 사용하고, 최종 full plot eval만 `EVAL_STRIDE=1`을 사용할 것
- `model.residual_skip`을 쓰는 실험을 평가할 때는 `compare_results.py`가 동일한 `residual_skip` 설정을 모델 생성에 전달하도록 유지할 것. 학습만 skip을 쓰고 평가에서 skip이 빠진 상태로 돌리지 말 것
- `shared.data.target_transform` 또는 `02_train.data.target_transform`을 추가한 실험은 평가와 테스트 metric 계산에서 반드시 같은 역변환을 적용할 것. delta target으로 학습하고 absolute speed metric으로 되돌리지 않은 결과를 쓰지 말 것
- `03_eval.biomech_clamp_max_jerk`를 쓸 때는 단위를 `m/s^3`로 해석하고, overlap averaging 이후 trial별 예측 시퀀스에 대해 discrete second difference `v[t] - 2v[t-1] + v[t-2]`를 제한하도록 유지할 것
- `03_eval.adaptive_ema`, `03_eval.one_euro`, `03_eval.alpha_beta`를 쓸 때는 모두 trial별로 독립 적용되는 causal post-filter로 유지할 것. trial 경계를 넘어 상태를 이어붙이거나 미래 샘플을 참조하지 말 것
- `02_train.train.derivative_loss_weight`를 쓸 때는 multi-horizon output에서 인접 horizon 간 속도 차분을 target과 맞추는 loss로만 해석할 것. single-step output에서는 0 loss가 되도록 유지할 것
- `02_train.train.prior_loss_weight`와 `prior_input_channel_idx`를 쓰는 실험은 prior 채널 기준이 normalization 전 물리 단위가 되도록 유지할 것
- `02_train.train.condition_weights`를 쓰는 실험은 `data.loader`가 반환한 per-trial condition metadata를 기준으로 window-level weighted sampler가 적용되도록 유지할 것. metadata가 없는 상태에서 condition weight를 켜지 말 것
- `02_train.train.overlap_consistency_weight`를 쓰는 실험은 같은 trial의 overlapping window가 같은 absolute timestep에 대해 낸 예측끼리만 consistency penalty를 계산하도록 유지할 것. horizon 1 단일시점 예측에 equality penalty를 직접 걸지 말 것
- 본 프로젝트의 대목표는 treadmill belt speed를 직접 추정하는 것이 아니다. `common/v_Y_true`(프로젝트 내부에서 `common/speed_y` 의미의 corrected speed target으로 간주) 에 양방향 LPF를 적용한 값을 추정하는 것이다
- 입력 feature에 `forceplate/grf/*`, `contact_l`, `contact_r`를 넣지 말 것. 이 프로젝트에서 GRF는 입력으로 금지한다
- 입력 feature를 만들기 위해 GRF나 GRF-derived stance label을 사용하지 말 것. `derived/model_com_speed`의 기존 GRF 의존형 버전은 쓰지 말고, `derived/model_com_speed_nogrf` 또는 다른 GRF-free prior만 사용할 것
- contact 추정 실험에서는 GRF를 supervision target으로만 사용할 수 있다. contact model의 입력에는 GRF를 넣지 말 것
- 본 프로젝트에서 학습하는 모델은 나중에 실시간 inference가 가능해야 한다. 새 실험을 추가할 때는 입력 feature 전처리, derived feature 계산, 모델 추론까지 포함한 전체 입력 경로가 온라인 실행 가능할 정도로 충분히 가벼운지 확인할 것
- target 생성용 offline 전처리는 실시간 제약 예외로 둔다. `common/v_Y_true`에 대한 양방향 LPF와 학습용 `_dot`, `_ddot` 생성의 `np.gradient()`는 데이터셋 생성용 전처리로 허용한다
- 입력 feature용 전처리는 예외 없이 real-time processing 가능해야 한다. 입력 feature를 만들기 위해 양방향 LPF, 미래 구간 참조, 비인과 smoothing을 사용하지 말 것
- 새 실험을 추가할 때 입력 feature에 `common/assist_level`을 넣지 말 것. 기존 config를 수정할 때도 `common/assist_level`을 추가하지 말 것
- `archive_season3` 이후의 새 실험은 output target으로 `treadmill/left/speed_leftbelt`를 쓰지 말 것. `common/v_Y_true`만 사용할 것
- `archive_season3` 이후의 새 실험은 `common/v_Y_true`에 양방향 LPF를 적용한 값을 ground truth로 사용할 것. target LPF는 cutoff `0.5 Hz`, order `4`만 사용할 것. target LPF를 끄거나 causal LPF로 바꾸지 말 것
- distillation을 사용할 때 teacher는 학습 전용으로만 사용할 것. 최종 배포/평가 대상은 항상 student 1개 모델이어야 하며, teacher 의존 추론이나 ensemble 추론을 추가하지 말 것
- 복잡한 실험 설계, 논문 아이디어 제안, 결과 해석 요청에서는 바로 결론을 내리지 말 것. 먼저 기존 결과에서 성공한 가설과 실패한 가설을 분리하고, 대안 3개 이상을 비교한 뒤 최종안을 제시할 것
- 새 archive를 설계할 때는 accuracy, latency, parameter count, sensor reduction, deployability 중 어떤 축의 기여를 노리는지 먼저 명시하고, 그 축에서 baseline 대비 기대 이점을 설명할 것
- 사용자가 병렬 조사, delegation, subagent 활용을 명시적으로 요청한 경우에는 subagent를 적극 사용할 것. 요청이 없으면 현재 agent가 직접 수행할 것
- 추상적인 슬로건이나 감각적인 표현으로 제안을 정당화하지 말 것. 구조, 입력 feature, 계산량, 실시간성, 비교 기준을 근거로 설명할 것
- 로컬 테스트: `conda run -n IIT python SpeedEstimator_TCN_MLP_experiments.py --config configs/...`
- `train.sh`에 config 여러 개를 넘기면 같은 GPU 안에서 worker pool로 학습한다. 기본 worker 수는 3개이며, 기본 상태에서는 GPU util 기반 자동 증감을 끄고 OOM이 발생할 때만 target worker를 3→2→1로 낮춘다. 낮아진 상태에서 성공 완료가 나오면 target worker를 다시 3으로 복구한다. OOM이 난 config만 재큐잉한다.
- Slurm 로그로 리다이렉트되는 non-interactive 학습에서는 batch/validation `tqdm`를 출력하지 말 것. 학습 로그에는 epoch 요약과 주요 이벤트만 남길 것

### GPU 사용 규칙
- 사용자가 `gpu:idxN:1`로 지정할 때 N은 http://143.248.65.114:5050/ 대시보드 기준 GPU 번호 (nvidia-smi 번호 아님)
- train/eval 제출 시 반드시 `--job-name`을 지정하여 대시보드에서 job으로 식별 가능하게 할 것
- `configs/archive_seasonX/archive_Y/...` 또는 `experiments/archive_seasonX/archive_Y/...`를 서버에 제출할 때 `--job-name`은 반드시 `Speed_wo_grf_ASX_AY_train_N` 또는 `Speed_wo_grf_ASX_AY_eval_N` 형식을 사용할 것. 예: `Speed_wo_grf_AS2_A2_train_1`, `Speed_wo_grf_AS2_A2_eval_1`
- 작업 제출 전 대시보드(http://143.248.65.114:5050/)에서 다른 사용자가 점유 중인 GPU를 확인하고, 점유 중인 GPU에는 절대 작업을 올리지 말 것 (사용자가 특정 idx를 지정하더라도 해당 GPU가 타인 점유 중이면 사용자에게 알리고 다른 GPU를 제안)

## 프로젝트 구조

```
SpeedEstimation/
├── data/
│   ├── loader.py          # H5 데이터 로딩, derived feature 계산, trial-level 캐시 + per-trial metadata registry
│   ├── body_frame.py      # quaternion 기반 body-aligned IMU 변환
│   ├── model_com_speed.py # GRF-free COM speed priors + IMU integrated speed + contact heuristics
│   └── utils.py           # Butterworth 필터, GRF contact 감지, H5 유틸
├── core/
│   ├── models.py          # TCN_MLP, TCN_GRU_Head, AttentionTCN 등 6개 모델
│   ├── datasets.py        # LazyWindowDataset (windowing + time warp augmentation + optional index info)
│   └── losses.py          # smoothness, kinematic, gradient penalty, frequency penalty, prior alignment loss
├── training/
│   └── trainer.py         # 학습 루프, LOSO CV, early stopping, AMP, checkpointing, condition-weighted sampler, overlap consistency
├── configs/               # YAML 실험 설정
├── experiments/           # 학습 결과 (model.pt, metrics.json, scaler.npz)
├── SpeedEstimator_TCN_MLP_experiments.py  # 메인 학습 스크립트
├── compare_results.py     # 평가/비교/ablation 분석
├── validate_and_retrain.py # 학습-평가 일관성 감사
├── train.sh               # SLURM 학습 제출 (TUI config 선택 지원)
└── eval_server.sh         # SLURM 평가 제출
```

## 데이터 파이프라인

- 데이터: `combined_data_S008.h5`, 구조: `Subject/Condition/Level/Trial`
- 피험자: S001~S008, 조건: level_075/100/125mps, accel_sine, decline_5deg, incline_10deg, stopandgo
- 샘플링: 100Hz, 출력: `common/v_Y_true`에 양방향 LPF(cutoff 0.5Hz, order 4)를 적용한 corrected speed target (m/s)

### 입력 feature 그룹
- `mocap/kin_q`: hip_flexion, knee_angle, ankle_angle (l/r) + _dot, _ddot (18ch)
- `derived/body_frame_imu`: body-aligned accel/gyro + forward_accel_overground (7ch)
- `derived/model_com_speed_nogrf`: GRF-free phase/kinematic/fusion 기반 COM speed prior (3 variants)
- `derived/imu_integrated_speed`: forward_accel_overground의 leaky integration (1ch)
- `derived/imu_speed_causal`: treadmill acceleration 없이 body-frame forward accel만 적분한 causal inertial prior (1ch)
- `derived/stride_progression`: robot 좌우 hip angle excursion과 cadence proxy로 만든 causal stride progression prior (1ch)
- `derived/interaction_power`: robot hip encoder + torque로 만든 causal interaction power/work (`interaction_power_l/r`, `interaction_work_pos/neg`) (4ch)
- `derived/contact_estimate`: phase/kinematic/fusion 기반 causal left/right contact probability (6ch)
- `robot/left`, `robot/right`: exo torque (2ch)
- `sub_info`: height, weight (2ch)

### Derived features (H5에 없고 loader에서 on-the-fly 계산)
- `derived/body_frame_imu`: quaternion 기반 body frame 변환
- `derived/model_com_speed_nogrf`: joint angle chain velocity + GRF-free support blending
- `derived/imu_integrated_speed`: leaky integrator
- `derived/imu_speed_causal`: treadmill-independent leaky integrator
- `derived/stride_progression`: exoskeleton hip-angle 기반 저주파 progression proxy
- `derived/interaction_power`: torque × causal hip angular velocity + exponentially weighted work
- `derived/contact_estimate`: hip phase / kinematics / kinematics+IMU fusion 기반 support probability
- `_dot`, `_ddot` 변수: `np.gradient() * fs`

### Trial-level 캐시
`TRIAL_DATASET_CACHE`로 H5 로딩 + derived feature 계산 결과를 캐시하여 LOSO fold 간 재사용.
첫 fold에서 전체 trial 로딩 후 캐시, 이후 fold에서는 subject 기준 필터링만 수행.

## Residual Skip Connection

`model.residual_skip.enable: true` + `input_channel_idx: N` 설정 시:
- `output = network(x) + x[:, -1, N]` (input normalization 전 raw 값 사용)
- 물리적 base estimate(COM/IMU speed) 위에 network가 보정값만 학습

## 학습 설정 요약

- 모델: TCN_GRU (기본), channels [32,32,64,64,128,128], kernel 5, GRU hidden 32
- 학습: batch 128, epochs 30, lr 0.001, AMP, Huber loss (delta=0.1)
- CV: LOSO (Leave-One-Subject-Out)
- 옵션 플래그: `--run_distribution_analysis`, `--run_feature_importance` (기본 비활성)
