# SpeedEstimation_robot

Exoskeleton 착용 환경에서 IMU + exo 센서만으로 human gait speed를 추정하는 딥러닝 프로젝트.
프로젝트의 대목표는 treadmill belt speed를 causal하게 추정하는 것이 아니라 `common/speed_y`에 bidirectional LPF를 적용한 target을 추정하는 것이다.
mocap/GRF 없이 robot 센서만 사용.

## 유지보수 규칙

- 프로젝트 구조, 코드 변경 등이 발생하면 이 CLAUDE.md도 함께 업데이트할 것
- 코드 구조가 변경되면 (새 모듈 추가 등) 관련 섹션 반영

## 파일 생성 규칙

- 새로운 파일을 repository에 추가하기 전 반드시 사용자에게 한국어로 파일 경로, 목적, 생성 이유를 설명하고 허락을 받을 것
- 작업 중 임시 파일을 만들었다면 같은 작업 안에서 직접 삭제할 경우 사용자 허락 없이 진행해도 됨
- 기존 코드 실행 결과로 생성되는 `png`, `log`, `csv`, `pt`, `npz` 등의 산출물은 사용자 허락 없이 생성되어도 됨

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
- CPU 평가: `sbatch -p idxN --job-name="이름" cpu_eval.sh --auto --ablation`
- 로컬 테스트: `conda run -n IIT python SpeedEstimator_TCN_MLP_experiments.py --config configs/...`
- `train.sh`에 config 여러 개를 넘기면 같은 GPU 안에서 same-GPU multi-worker로 학습한다. 기본 시작 worker 수는 3개이고, GPU util 기준 자동 증감은 사용하지 않는다.
- `train.sh`에서 OOM이 발생하면 target worker 수를 1단계 낮추고 실패한 config만 재큐잉할 것
- `train.sh`에서 낮아진 worker 수로 성공 완료가 나오면 target worker 수를 기본값 3으로 복구할 것

### GPU 사용 규칙
- 사용자가 `gpu:idxN:1`로 지정할 때 N은 http://143.248.65.114:5050/ 대시보드 기준 GPU 번호 (nvidia-smi 번호 아님)
- train/eval 제출 시 반드시 `--job-name`을 지정하여 대시보드에서 job으로 식별 가능하게 할 것
- 작업 제출 전 대시보드(http://143.248.65.114:5050/)에서 다른 사용자가 점유 중인 GPU를 확인하고, 점유 중인 GPU에는 절대 작업을 올리지 말 것 (사용자가 특정 idx를 지정하더라도 해당 GPU가 타인 점유 중이면 사용자에게 알리고 다른 GPU를 제안)
- archive 실험을 서버에 제출할 때 job name은 `SpeedRobot_AS{season}_A{archive}_{train|eval}_{index}` 형식만 사용할 것
- 사용자가 feasibility 단계라고 지정하면 full LOSO를 돌리지 말고 config의 `loso_subjects`에 지정된 1개 subject만 실행할 것
- `configs/archive_seasonX/archive_Y/...`에 있는 실험을 평가할 때는 결과 plot과 ablation 산출물을 `compare_result/archive_seasonX/archive_Y/...`에 저장할 것
- `compare_results.py`의 평가 png에는 정답 speed와 예측 speed만 그릴 것. robot raw/pre 신호를 오버레이하지 말 것
- 본 프로젝트에서 새로 학습시키는 모델은 추후 실시간 inference에 사용할 수 있어야 한다. 입력 feature 전처리와 모델 추론은 causal 또는 online-friendly하게 유지하고, 파라미터 수·window 크기·branch 수를 과도하게 키우지 말 것
- 학습 target을 만들기 위한 offline 전처리에는 bidirectional LPF를 사용할 수 있다. 이 예외는 label/reference 생성에만 적용할 것
- 입력 feature 생성에 `filtfilt`, bidirectional LPF, 미래 샘플 참조, 중앙차분 기반 `np.gradient()`를 사용하지 말 것. 입력 feature용 미분/평활화는 backward difference, causal LPF, EMA 등 실시간 처리 가능한 방식만 사용할 것
- 논문용 새 derived feature를 추가할 때 `forceplate`, `grf`, `mocap`, 출력 speed 경로(`treadmill/*/speed_*`)를 읽지 말 것. 새 feature는 `robot/*`, `robot/back_imu`, `sub_info`만 사용해 계산할 것
- 논문용 학습 feature로 `common/assist_level` 같은 메타데이터를 직접 사용하지 말 것. 보조 상태는 `torque`, `motor_angle`, `deflection`, `actuator power` 같은 robot-side causal signal로만 추정할 것
- `cpu_eval.sh`는 GPU를 요청하지 말고 `--cpus-per-task=4`, `--mem=5G`, `CUDA_VISIBLE_DEVICES=""`로 CPU-only 평가만 실행할 것
- `cpu_eval.sh`는 single worker면 `COMPARE_RESULTS_RAM_GB=5`, multi-worker면 worker당 기본 `2GB`, 최종 ablation summary는 `4GB` 제한을 사용할 것
- archive 평가 기본 운영은 2단계만 사용할 것: 1차는 fast eval로 전체 실험 metric만 계산하고, 2차는 MAE 최고 1개만 full plot을 생성할 것
- `eval_server.sh`와 `cpu_eval.sh`의 기본값은 `EVAL_FAST_EVAL=1`, `EVAL_FULL_PLOT_TOPK=1`으로 둘 것. 전체 실험에 대해 처음부터 full plot을 생성하지 말 것
- fast eval의 metric pass는 `stride_eval=3`만 사용할 것. top-k full plot 재생성은 `stride_eval=1`로 다시 로드할 것
- `eval_server.sh`의 기본 평가 배치 크기는 `EVAL_BATCH_SIZE=256`, 상세 plot 배치는 `EVAL_DETAIL_BATCH_SIZE=128`, feature importance 배치는 `EVAL_FI_BATCH_SIZE=128`으로 둘 것
- `cpu_eval.sh`의 기본 평가 배치 크기는 `EVAL_BATCH_SIZE=128`, 상세 plot 배치는 `EVAL_DETAIL_BATCH_SIZE=64`, feature importance 배치는 `EVAL_FI_BATCH_SIZE=64`으로 둘 것
- 상세 condition png는 `Full + Zoom 2개`까지만 그릴 것. 세 번째 zoom row는 생성하지 말 것
- Slurm 로그로 리다이렉트되는 non-interactive 학습에서는 batch/validation `tqdm`를 출력하지 말 것. 학습 로그에는 epoch 요약과 주요 이벤트만 남길 것
- 복잡한 실험 설계, 논문 아이디어 제안, 결과 해석을 요청받으면 바로 결론을 내리지 말 것. 먼저 기존 결과에서 성공한 가설과 실패한 가설을 분리하고, 대안 3개 이상을 비교한 뒤 최종안을 제시할 것
- 새 archive를 설계할 때는 `accuracy`, `latency`, `parameter count`, `sensor reduction`, `deployability` 중 어떤 축의 기여를 노리는지 먼저 명시할 것. 이어서 그 축에서 baseline 대비 기대 이점을 구조, 입력 feature, 계산량 기준으로 설명할 것
- 사용자가 병렬 조사, delegation, subagent 활용을 명시적으로 요청한 경우에만 subagent를 사용할 것. 요청이 없으면 현재 agent가 직접 조사하고 구현할 것
- 실험 제안을 정당화할 때 추상적인 표현을 쓰지 말 것. 모델 구조, 입력 feature, branch 수, window 크기, 파라미터 수, 실시간 추론 가능성, 비교 baseline을 근거로 설명할 것

## 프로젝트 구조

```
SpeedEstimation_robot/
├── data/
│   ├── loader.py          # H5 데이터 로딩, derived feature 계산, trial-level 캐시
│   ├── body_frame.py      # quaternion 기반 body-aligned IMU 변환
│   ├── model_com_speed.py # IMU integrated speed (leaky integrator)
│   └── utils.py           # Butterworth 필터, GRF contact 감지, H5 유틸
├── core/
│   ├── models.py          # TCN_MLP, TCN_GRU_Head, SlowFast/StructuredSlowFast 등 시계열 모델
│   ├── datasets.py        # LazyWindowDataset (windowing + time warp augmentation)
│   └── losses.py          # smoothness, kinematic, gradient penalty loss
├── training/
│   └── trainer.py         # 학습 루프, LOSO CV, early stopping, AMP, checkpointing
├── configs/               # YAML 실험 설정
├── experiments/           # 학습 결과 (model.pt, metrics.json, scaler.npz)
├── SpeedEstimator_TCN_MLP_experiments.py  # 메인 학습 스크립트
├── compare_results.py     # 평가/비교/ablation 분석 + condition/regime summary csv 생성
├── validate_and_retrain.py # 학습-평가 일관성 감사
├── train.sh               # SLURM 학습 제출 (TUI config 선택 지원)
└── eval_server.sh         # SLURM 평가 제출
```

## 데이터 파이프라인

- 데이터: `combined_data_S008.h5` (symlink), 구조: `Subject/Condition/Level/Trial`
- 피험자: S001~S008, 조건: level_075/100/125mps, accel_sine, decline_5deg, incline_10deg, stopandgo
- 샘플링: 100Hz, 논문용 출력 target: `common/speed_y`에 bidirectional LPF를 적용한 human gait speed (m/s)

### Raw 데이터 (H5에 저장됨)
- `robot/back_imu`: accel_x/y/z, gyro_x/y/z, quat_w/x/y/z
- `robot/left`, `robot/right`: hip_angle, thigh_angle, torque
- `sub_info`: height, weight

### 가공된 입력 feature 그룹
- `derived/body_frame_imu`: body-aligned accel/gyro + forward_accel_overground (7ch)
- `derived/imu_integrated_speed`: IMU 기반 속도 추정 (1ch)
- `robot/left`, `robot/right`: hip_angle, thigh_angle, torque + _dot, _ddot (최대 14ch)
- `sub_info`: height, weight (2ch)

### Derived features (H5에 없고 loader에서 on-the-fly 계산)
- `derived/body_frame_imu`: accel_right/forward/up, gyro_right/forward/up, forward_accel_overground (quaternion 기반 body frame 변환)
- `derived/imu_integrated_speed`: forward_accel_overground의 leaky integration
- `derived/robot_biomech_rt`: robot-only pseudo gait timing, signed joint power/propulsion, bilateral symmetry, causal RMS envelope, pseudo speed prior, guarded torque preview
- output target filtering은 `target_lpf_mode`로 제어할 것. `zero_phase`는 offline `filtfilt`, `causal`은 online-feasible causal LPF, `none`은 추가 target smoothing 없음
- `_dot`, `_ddot` 변수는 입력 feature로 사용할 때 `np.gradient()`를 쓰지 말고 causal/backward difference 계열로 계산할 것
- `contact_l/r`: GRF z-force thresholding으로 계산

### Trial-level 캐시
`TRIAL_DATASET_CACHE`로 H5 로딩 + derived feature 계산 결과를 캐시하여 LOSO fold 간 재사용.
첫 fold에서 전체 trial 로딩 후 캐시, 이후 fold에서는 subject 기준 필터링만 수행.

## 학습 설정 요약

- 모델: TCN_GRU (기본), channels [32,32,64,64,128,128], kernel 5, GRU hidden 32
- 학습: batch 128, epochs 30, lr 0.001, AMP, Huber loss (delta=0.1)
- CV: LOSO (Leave-One-Subject-Out)
- 옵션 플래그: `--run_distribution_analysis`, `--run_feature_importance` (기본 비활성)
